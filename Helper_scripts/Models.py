from casadi import *
from acados_template import AcadosModel
from Helper_scripts. import


def bicycle_model(lr=0.17145, lf=0.15875, model_name="Kinematic_Bicycle_Model"):
    """

        Function referred from acados/examples/getting_started/pendulum_model.py
        :arguments:
            lr : Distance from center of gravity of car to the rear axle
            lf : Distance from center of gravity of car to the front axle

        :return:
            System Model: AcadosModel() object
    """
    # STATES
    sx = SX.sym('sx')  # x-coordinate of Center of Gravity of car
    sy = SX.sym('sy')  # y-coordinate of Center of Gravity of car
    psi = SX.sym('psi')  # Orientation of car

    # INPUTS
    delta = SX.sym('delta')  # Heading angle
    v = SX.sym('v')  # Velocity of car

    x = vertcat(sx, sy, psi)  # Vector of states
    u = vertcat(delta, v)

    # DIFFERENTIAL STATES
    sx_dot = SX.sym('sx_dot')
    sy_dot = SX.sym('sy_dot')
    psi_dot = SX.sym('psi_dot')

    xdot = vertcat(sx_dot, sy_dot, psi_dot)

    # EXPLICIT FORM OF STATE EQUATION
    f_expl = vertcat(
                u[1]*cos(x[2]),
                      u[1]*sin(x[2]),
                     (u[1]*tan(u[0]))/(lr+lf)
                     )

    f_impl = xdot - f_expl

    model = types.SimpleNamespace()
    model.name = model_name

    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl

    model.x = x
    model.x0 = np.array([0.0, 0.0, 0.0])
    model.xdot = xdot

    model.u = u

    return model


def Spatial_Dynamics(C1=0.5, C2=17.06, Cm1=12.0, Cm2=2.17, Cr2=0.1, Cr0=0.6, rho = 10, model_name="Spatial_Dy"):
    """

        Function referred from acados/examples/getting_started/pendulum_model.py
        :arguments:
            lr : Distance from center of gravity of car to the rear axle
            lf : Distance from center of gravity of car to the front axle

        :return:
            System Model: AcadosModel() object
    """

    #TIME DEPENDENT STATES
    x_t = SX.sym('x_t')
    y_t = SX.sym('y_t')
    psi_t = SX.sym('psi_t')
    v_t = SX.sym('v_t')

    # INPUTS
    delta = SX.sym('delta')  # Heading angle
    D = SX.sym('D') # Duty Cycle


    x = vertcat(x_t,y_t,psi_t,v_t)
    u = vertcat(delta,D)

    x_dot = vertcat(
              x[3]*cos(x[2]+C1*u[0]),
                    x[3]*sin(x[2]+C1*u[0]),
                    x[3]*u[0]*C2,
                    (Cm1 - Cm2*x[3])*u[1] - Cr2*x[3]**2 - Cr0 - ((x[3]*u[0])**2)*C2*C1**2
                    )


    # SPATIAL STATES
    e_y = SX.sym('e_y')  # lateral error
    e_psi = SX.sym('e_psi')  # orientation error
    v = SX.sym('v')  # velocity
    t = SX.sym('t')  # velocity


    x_new = vertcat(e_y, e_psi, v, t)  # Vector of spatial states

    # DIFFERENTIAL SPATIAL STATES
    e_y_dash = SX.sym('e_y_dash')
    e_psi_dash = SX.sym('e_psi_dash')
    v_dash = SX.sym('v_dash')
    t_dash = SX.sym('t_dash')

    x_dash = vertcat(e_y_dash,e_psi_dash,v_dash,t_dash)


    v_x = x[3]
    v_y = x[3]*C1*u[0]
    s_dot = (v_x*cos(x_new[1]) - v_y*sin(x_new[1]))*(1/(1-(x_new[0]/rho)))

    f_expl = vertcat(
                 (x[3]*sin(x_new[1]) + x[3]*C1*u[0]*cos(x_new[1]))/s_dot,
                       (x_dot[2]/s_dot) - 1/rho,
                        x_dot[3]/s_dot,
                        1/s_dot
                     )


    # EXPLICIT FORM OF STATE EQUATION

    f_impl = x_dash - f_expl

    model = types.SimpleNamespace()
    model.name = model_name

    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl

    model.x = x_new
    model.x0 = np.array([0.0, 0.0, 0.0,0.0])
    model.xdot = x_dash

    model.u = u

    return model


class CarModel:
    def __init__(self, lr= 0.17145, lf=0.15875, h=1/50):
        self.nx = 3  # Number of states
        self.nu = 2  # Number of inputs

        self.x_min = []  # State lower limits
        self.x_max = []  # State upper limits

        self.u_min = []  # Input lower bound
        self.u_max = []  # Input upper bound

        self.h = h  # Discretisation rate

        # Constants
        self.lr = lr
        self.lf = lf

    def system(self, x, u):
        """
    System function that returns the next state provided the current state and input
    Provides the continuous time dynamics

    input arguments:
    a. x - Current state
    b. u - Current input
        """

        xdot = vertcat(
                        u[1]*cos(x[2]),
                              u[1]*sin(x[2]),
                             (u[1]*tan(u[0]))/(self.lr+self.lf)
                      )


        return xdot

    def simulate(self, x, u):
        """
        Function to simulate the system for one time step using RK4 integration, given the current state and input
        :input:
            x - current state
            u - current input
        :return:
            xnext - Next state as a numpy array
        """
        k1 = self.system(x, u)
        k2 = self.system(x + self.h / 2 * k1, u)
        k3 = self.system(x + self.h / 2 * k2, u)
        k4 = self.system(x + self.h * k3, u)
        xnext = x + self.h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        xnext = xnext.full().reshape(self.nx,1)
        return xnext

class CarSpatialModel:
    def __init__(self, C1=0.5, C2 = 17.06, Cm1 = 12.0, Cm2 = 2.17, Cr2 = 0.1, Cr0 = 0.6, rho=10, h=1/50):
        self.nx = 4  # Number of states
        self.nu = 2  # Number of inputs

        self.x_min = []  # State lower limits
        self.x_max = []  # State upper limits

        self.u_min = []  # Input lower bound
        self.u_max = []  # Input upper bound

        self.h = h  # Discretisation rate

        # Constants
        self.C1 = C1
        self.C2 = C2
        self.Cm1 = Cm1
        self.Cm2 = Cm2
        self.Cr2 = Cr2
        self.Cr0 = Cr0
        self.rho = rho

    def spatial_system(self,x,u):


        x_dot = vertcat(
                  x[3] * cos(x[2] + self.C1 * u[0]),
                        x[3] * sin(x[2] + self.C1 * u[0]),
                        x[3] * u[0] * self.C2,
                        (self.C1 - self.Cm2 * x[3]) * u[1] - self.Cr2 * x[3] ** 2 - self.Cr0 - ((x[3] * u[0]) ** 2) * self.C2 * self.C2 ** 2
                       )

        v_x = x[3]
        v_y = x[3] * self.C2 * u[0]
        s_dot = (v_x * cos(x_new[1]) - v_y * sin(x_new[1])) * (1 / (1 - (x_new[0] / self.rho)))

        x_new = x[2] - psi_ref

        x_dash = vertcat(
                   (x[3] * sin(x_new[1]) + x[3] * self.C2 * u[0] * cos(x_new[1])) / s_dot,
                         (x_dot[2] / s_dot) - 1 / self.rho,
                          x_dot[3] / s_dot,
                          1 / s_dot
                        )
        return x_dash

    def spatial_simulate(self,x,x_new,u):

        k1 = self.spatial_system(x, u)
        k2 = self.system(x + self.h / 2 * k1, u)
        k3 = self.system(x + self.h / 2 * k2, u)
        k4 = self.system(x + self.h * k3, u)
        xnext = x + self.h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        xnext = xnext.full().reshape(self.nx, 1)

        return xnext




# class SimulatedCar(CarModel):
#     def __init__(self, g, l, b, m, h, x0):
#         super(SimulatedInvertedPendulum, self).__init__(g, l, b, m, h)
#
#         self.x = x0.reshape(self.nx, 1)
#
#     def step(self, u):
#         """
#     Function for finding the next state using the Runge-Kutta 4 discretisation scheme
#     Discrete time dynamics
#
#     input argument:
#     a. u - current input
#
#     output:
#     a. x - next state
#         """
#         k1 = self.system(self.x, u)
#         k2 = self.system(self.x + self.h / 2 * k1, u)
#         k3 = self.system(self.x + self.h / 2 * k2, u)
#         k4 = self.system(self.x + self.h * k3, u)
#         self.x = self.x + self.h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
#         self.x = self.x.full()
#
#         return self.x
#
#     def measure(self):
#         return self.x
#
#     def apply(self, u):
#
#         k1 = self.system(self.x, u)
#         k2 = self.system(self.x + self.h / 2 * k1, u)
#         k3 = self.system(self.x + self.h / 2 * k2, u)
#         k4 = self.system(self.x + self.h * k3, u)
#         self.x = self.x + self.h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
#         self.x = self.x.full()
#
#
# class RealInvertedPendulum(AbstractInvertedPendulum):
#     def __init__(self, g, l, b, m, h, port, baud_rate):
#         super(RealInvertedPendulum, self).__init__(g, l, b, m, h)
#
#         self.ser = SerialCom(port, baud_rate)
#         self.x = self.ser.measure()
#
#     def step(self, u):
#         self.ser.apply(u)
#         time.sleep(self.h)
#         self.x = self.ser.measure()
#
#         return self.x
#
#     def apply(self, u):
#         self.ser.apply(u)
#
#     def measure(self):
#         # dig the latest state value from the buffer
#         return self.ser.measure()
