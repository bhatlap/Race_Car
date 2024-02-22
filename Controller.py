#!/usr/bin/env python3
import numpy as np
from acados_template import AcadosOcp, AcadosModel, AcadosOcpSolver
from Helper_scripts.Models import bicycle_model
from casadi import *
import scipy.linalg as lin
from Helper_scripts.Models import CarModel


def acados_settings(track_width, Tf=1, N=50):

    ocp = AcadosOcp()
    model = bicycle_model()
    constraint = types.SimpleNamespace()

    # Model
    model_ac = AcadosModel()
    model_ac.name = model.name
    model_ac.f_impl_expr = model.f_impl_expr
    model_ac.f_expl_expr = model.f_expl_expr
    model_ac.x = model.x
    model_ac.xdot = model.xdot
    model_ac.u = model.u

    # set dimensions
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = nx + nu
    ny_e = nx

    """ OCP SETTINGS """
    ocp.dims.N = N
    Q = np.diag([1, 1, 1])
    R = np.diag([1, 1])
    Qt = Q

    ocp.cost.cost_type = 'LINEAR_LS'
    ocp.cost.cost_type_e = 'LINEAR_LS'

    # Stage Cost
    ocp.cost.W = lin.block_diag(Q, R)
    ocp.cost.Vx = np.zeros((ny, nx))
    ocp.cost.Vx[:nx, :nx] = np.eye(nx)
    ocp.cost.Vu = np.zeros((ny, nu))
    ocp.cost.Vu[-nu:, -nu:] = np.eye(nu)

    # Terminal Cost
    ocp.cost.W_e = Qt
    ocp.cost.Vx_e = np.eye(nx)

    # set intial references
    ocp.cost.yref = np.zeros((ny,))
    ocp.cost.yref_e = np.zeros((ny_e,))

    # State constraints

    #   Constraint at initial shooting node/Initial state
    ocp.constraints.x0 = np.array([0.0, 0.0, 0.0])

    #   Constraint at other shooting nodes
    ocp.constraints.lbx = np.array([-100, -100])
    ocp.constraints.ubx = np.array([100, 100])
    ocp.constraints.idxbx = np.array([0, 1])

    #   Constraint at final shooting node
    ocp.constraints.lbx_e = np.array([-100, -100])
    ocp.constraints.ubx_e = np.array([100, 100])
    ocp.constraints.idxbx_e = np.array([0, 1])

    # Input constraints
    ocp.constraints.lbu = np.array([-np.pi, 0.0])
    ocp.constraints.ubu = np.array([np.pi, 5.0])
    ocp.constraints.idxbu = np.array([0, 1])

    # OCP Model
    ocp.model = model_ac

    # Solver options
    ocp.solver_options.tf = Tf
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.nlp_solver_type = "SQP"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "IRK"
    ocp.solver_options.print_level = 0
    ocp.solver_options.nlp_solver_max_iter = 200
    # ocp.solver_options.qp_solver_iter_max = 200
    ocp.solver_options.tol = 1e-1
    # ocp.solver_options.qp_solver_cond_N = N

    # create solver
    acados_solver = AcadosOcpSolver(ocp, json_file="acados_ocp.json")

    return model, constraint, acados_solver


def generate_init_guess(x, N, max_speed):

    u0 = np.zeros(N) # Steering
    u1 = np.linspace(0, max_speed, N) # Velocity

    u_init = np.column_stack([u0, u1])
    car = CarModel()

    x_init = []
    for u in u_init:
        x = car.simulate(x, u)
        x_init.append(x.T)

    x_e = car.simulate(x,np.array([0.0, max_speed]))
    x_init.append(x_e.T)
    x_init = np.array(x_init).reshape(N+1, x.shape[0])

    return x_init, u_init


def set_init_guess(solver, init_x, init_u, N):

    for i in range(N+1):
        solver.set(i, 'x', init_x[i, :])

    for i in range(N):
        solver.set(i, 'u', init_u[i, :])


def warm_start(opt_x, opt_u, nx, nu):

    car = CarModel()

    xf = opt_x[-1, :]
    uf = opt_u[-1, :]

    xN_1 = car.simulate(xf, uf).reshape(1, nx)

    init_x = np.vstack((opt_x[1:], xN_1))
    init_u = np.vstack((opt_u[1:], uf))

    return init_x, init_u


class OCP:
    """
    Class to interact with and define an Optimal Control Problem.
    """

    def __init__(self, x0, xr, ur, i, horizon=1, f=50):

        self.ocp = AcadosOcp()  # OCP Object to formulate OCP
        self.ocp.model = bicycle_model()  # Car model
        self.ocp.code_export_directory = "Code_Trajectory_Tracking_OCP"

        self.nx = self.ocp.model.x.size()[0]
        self.nu = self.ocp. model.u.size()[0]
        self.ny = self.nx + self.nu

        self.Q = np.diag([1, 1, 1])  # np.diag([1e1, 1e-4, 1e4, 1e-2])
        self.R = np.diag([1, 1])  # 1e1 * np.eye(self.nu)

        self.Tf = horizon  # Prediction horizon in seconds
        self.N = f  # Steps per prediction horizon (discretization frequency)
        self.counter = 0

        self.xr = xr
        self.ur = ur

        self.x0 = x0
        self.current_iteration = i

    def build(self):
        """
        This function builds an OCP of the form:

            min(x,u) Σ((x-xr)T Q (x-xr) + (u-ur)T R (u-ur)) + (xN-xrN)T Qt (xN-xrN)
                s.t
                    x_dot = model
                    xlb <= x <= xub
                    ulb <= u <= uub

                x : []

        :return: ocp_solver : The solver object to be used to solve the OCP
        """

        """ _____________________________________________OCP MODELING_______________________________________________ """

        """ Cost Function """
        self.ocp.cost.cost_type = 'LINEAR_LS'
        self.ocp.cost.cost_type_e = 'LINEAR_LS'

        self.ocp.cost.W = lin.block_diag(self.Q, self.R)
        self.ocp.cost.Vx = np.zeros((self.ny, self.nx))
        self.ocp.cost.Vx[:self.nx, :self.nx] = np.eye(self.nx)
        self.ocp.cost.Vu = np.zeros((self.ny, self.nu))
        self.ocp.cost.Vx[-self.nu:, -self.nu:] = np.eye(self.nu)

        for j in range(self.N):
            xref = self.xr[self.current_iteration+j:]
            uref = self.ur[:]
            self.ocp.cost.yref = np.append(xref, uref)

        self.ocp.cost.W_e = self.Q
        self.ocp.cost.Vx_e = np.eye(self.nx)
        self.ocp.cost.yref_e = self.xr[-1]

        """ State Constraints """

        # self.ocp.constraints.lbx =
        # self.ocp.constraints.ubx =
        # self.ocp.constraints.idxbx =   # Constraint applied to 3rd state (cart position)

        """Input Constraints"""
        # self.ocp.constraints.lbu = np.array([]
        # self.ocp.constraints.ubu = np.array([])
        # self.ocp.constraints.idxbu = np.array([0, 1])
        # self.ocp.constraints.x0 = self.x0
        #
        # """Solver Options"""
        # self.ocp.dims.N = self.N
        # self.ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        # self.ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        # self.ocp.solver_options.integrator_type = 'IRK'  # System model integrator
        # self.ocp.solver_options.print_level = 0
        # self.ocp.solver_options.nlp_solver_type = 'SQP_RTI'
        # self.ocp.solver_options.qp_solver_cond_N = self.N  # Stages per Prediction horizon
        # self.ocp.solver_options.tol = 1e-2  # Tolerance
        # self.ocp.solver_options.nlp_solver_max_iter = 100
        # self.ocp.solver_options.tf = self.Tf  # Prediction horizon
        # self.ocp.solver_options.levenberg_marquardt = 10.0
        #
        # """ Generate .json file to build solver """
        # AcadosOcpSolver.generate(self.ocp, json_file='acados_MPC.json')
        # AcadosOcpSolver.build(self.ocp.code_export_directory, with_cython=True)


def main():
    pass


if __name__ == '__main__':
    main()

x = np.array([0,0,0])
N = 11
max_speed = 1
xin,uin = generate_init_guess(x, N, max_speed)
print(xin)
print(uin)
print(xin[-1, :])
