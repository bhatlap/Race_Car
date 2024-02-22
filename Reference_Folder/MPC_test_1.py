import rospy
import numpy as np
from acados_template import AcadosOcp, AcadosOcpSolver
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from casadi import *
import csv

class MPCNode:
    def __init__(self):
        rospy.init_node('mpc_node', anonymous=True)

        # Load track data from CSV file
        self.track_data = self.load_track_data("/home/pawan/Thesis/F1env/src/f1tenth_simulator/scripts/Additional_Maps/f1tenth_racetracks/Silverstone/Silverstone_centerline.csv")

        # MPC setup
        self.acados_solver = self.setup_acados_solver()

        # ROS publishers
        self.track_pub = rospy.Publisher('/race_track', Path, queue_size=10)
        self.control_pub = rospy.Publisher('/control_signal', Float32MultiArray, queue_size=10)

    def load_track_data(self, file_path):
        data = np.genfromtxt(file_path, delimiter=',', skip_header=1)
        return data[:, 1:5]  # Assuming columns 2-5 are X, Y, left border, right border

    def setup_acados_solver(self):
        # Acados setup (Replace with your specific model and constraints)
        ocp = AcadosOcp()
        self.configure_ocp(ocp)

        acados_solver = AcadosOcpSolver(ocp, json_file='acados_ocp.json')

        return acados_solver

    def configure_ocp(self, ocp):
        # Configure acados ocp (Replace with your specific model and constraints)
        nx = 6  # state dimension
        nu = 2  # control dimension
        N = 20  # prediction horizon

        ocp.dims.N = N
        ocp.dims.nx = nx
        ocp.dims.nu = nu

        ocp.model.cost_y_expr = vertcat(0.1 * ocp.model.x[0] ** 2, 0.1 * ocp.model.x[1] ** 2)
        ocp.model.cost_y_expr_e = vertcat(0.1 * ocp.model.x[0] ** 2, 0.1 * ocp.model.x[1] ** 2)

        ocp.model.dyn_expr_f_expl = vertcat(
            ocp.model.x[0] + ocp.model.x[3] * MX.sin(ocp.model.x[4]) * ocp.model.u[0],
            ocp.model.x[1] + ocp.model.x[3] * MX.cos(ocp.model.x[4]) * ocp.model.u[0],
            ocp.model.x[2],
            ocp.model.x[3],
            ocp.model.x[4] + ocp.model.x[3] * ocp.model.u[1],
            ocp.model.x[5]
        )

        ocp.model.con_h_expr = vertcat(
            ocp.model.x[2],
            ocp.model.x[5]
        )

        ocp.constraints.con_ub = np.array([1.0, 0.2])  # upper bounds for constraints

        ocp_solver_opts = ocp.acados_ocp_opts
        ocp_solver_opts['print_level'] = 1
        ocp_solver_opts['sim_method_num_stages'] = 4
        ocp_solver_opts['sim_method_num_steps'] = 3

    def solve_mpc(self):
        # Get current state (Replace with your state estimation)
        current_state = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0])

        # Set initial state in acados solver
        self.acados_solver.set('constr_lbx_0', current_state)
        self.acados_solver.set('constr_ubx_0', current_state)

        # Solve MPC problem
        self.acados_solver.solve()

        # Get control signal
        u = self.acados_solver.get('u')

        return u

    def publish_track(self):
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"

        for point in self.track_data:
            pose_stamped = PoseStamped()
            pose_stamped.pose.position.x = point[0]
            pose_stamped.pose.position.y = point[1]
            path_msg.poses.append(pose_stamped)

        self.track_pub.publish(path_msg)

    def publish_control_signal(self, u):
        control_msg = Float32MultiArray()
        control_msg.data = u.flatten()
        self.control_pub.publish(control_msg)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            # Solve MPC
            u = self.solve_mpc()

            # Publish track and control signal
            self.publish_track()
            self.publish_control_signal(u)

            rate.sleep()

if __name__ == '__main__':
    mpc_node = MPCNode()
    mpc_node.run()
"""