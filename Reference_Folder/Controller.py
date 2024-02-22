#!/usr/bin/env python3

from acados_template import AcadosOcp
from Helper_scripts.Models import Kinematic_Single_Track_Model
from casadi import *
import scipy.linalg as lin


class OCP:
    """
    Class to interact with and define an Optimal Control Problem.
    """

    def __init__(self, x0, xr, ur, horizon=1, f=20):

        self.ocp = AcadosOcp()  # OCP Object to formulate OCP
        self.ocp.model = Kinematic_Single_Track_Model()  # Car model
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
        self.ocp.cost.yref = np.append(self.xr, self.ur)

        self.ocp.cost.W_e = self.Q
        self.ocp.cost.Vx_e = np.eye(self.nx)
        self.ocp.cost.yref_e = self.xr

        # """ State Constraints """
        # self.ocp.constraints.lbx =
        # self.ocp.constraints.ubx =
        # self.ocp.constraints.idxbx =   # Constraint applied to 3rd state (cart position)

        """Input Constraints"""
        self.ocp.constraints.lbu = np.array([])
        self.ocp.constraints.ubu = np.array([])
        self.ocp.constraints.idxbu = np.array([0, 1])
        self.ocp.constraints.x0 = self.x0
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