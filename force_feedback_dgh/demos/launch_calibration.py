import pybullet as p
import numpy as np
import pinocchio as pin
import hppfcl
from dynamic_graph_head import ThreadHead, SimHead, HoldPDController, SimVicon, Vicon
from datetime import datetime
import dynamic_graph_manager_cpp_bindings
from force_feedback_dgh.mim_robots.robot_loader import load_bullet_wrapper, load_pinocchio_wrapper
from force_feedback_dgh.mim_robots.pybullet.env import BulletEnvWithGround
from force_feedback_dgh.mim_robots.robot_list import MiM_Robots
import pathlib
import os

python_path = pathlib.Path('.').absolute().parent / 'force_feedback_dgh'
os.sys.path.insert(1, str(python_path))
import launch_utils

from controllers.calibration import KukaViconCalibration as Controller

# # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# Choose experiment, load config and import controller  #  
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
SIM = True
config = launch_utils.load_config_file('calibration')
ROBOT_NAME = 'iiwa_convex_ft_sensor_shell'

# # # # # # # # # # # #
# Import robot model  #
# # # # # # # # # # # #
LOCKED_JOINTS = []
pin_robot = load_pinocchio_wrapper(ROBOT_NAME)  # , locked_joints=LOCKED_JOINTS)

# # # # # # # # # # # # #
# Setup control thread  #
# # # # # # # # # # # # #
if SIM:
    # Sim env + set initial state 
    config['T_tot'] = 15
    env = BulletEnvWithGround(p.GUI)
    robot_simulator = load_bullet_wrapper(ROBOT_NAME, locked_joints=LOCKED_JOINTS)
    env.add_robot(robot_simulator)
    q_init = np.asarray(config['q0'])
    v_init = np.asarray(config['dq0'])
    robot_simulator.reset_state(q_init, v_init)
    robot_simulator.forward_robot(q_init, v_init)
    # <<<<< Customize your PyBullet environment here if necessary
    vicon = SimVicon(['kuka_ef/kuka_ef'])
    head = SimHead(robot_simulator, with_sliders=False)

# !!!!!!!!!!!!!!!!
# !! REAL ROBOT !!
# !!!!!!!!!!!!!!!!
else:
    config['T_tot'] = 400
    path = MiM_Robots[ROBOT_NAME].dgm_path
    print(path)
    head = dynamic_graph_manager_cpp_bindings.DGMHead(path)
    target = None
    env = None
    vicon = Vicon('172.24.117.119:801', ['kuka_ef/kuka_ef'])

ctrl = Controller(head, pin_robot, config, run_sim=SIM, locked_joints=LOCKED_JOINTS)

thread_head = ThreadHead(
    1. / config['ctrl_freq'],  # dt.
    HoldPDController(head, 50., 0.5, with_sliders=False),  # Safety controllers.
    head,  # Heads to read / write from.
    [
        ('vicon', vicon)
    ],
    env  # Environment to step.
)

thread_head.switch_controllers(ctrl)

# # # # # # # # #
# Data logging  #
# # # # # # # # # <<<<<<<<<<<<< Choose data save path & log config here (cf. launch_utils)
# prefix     = ""
prefix = "/tmp/"
suffix = ""
LOG_FIELDS = launch_utils.get_log_config('calibration')

# # # # # # # # # # # 
# Launch experiment #
# # # # # # # # # # # 
if SIM:
    thread_head.start_logging(int(config['T_tot']),
                              prefix + 'calibration' + "_SIM_" + str(datetime.now().isoformat()) + suffix + ".mds",
                              LOG_FIELDS=LOG_FIELDS)
    thread_head.sim_run_timed(int(config['T_tot']))
    thread_head.stop_logging()
else:
    thread_head.start()
    thread_head.start_logging(20, prefix + 'calibration' + "_REAL_" + str(datetime.now().isoformat()) + suffix + ".mds",
                              LOG_FIELDS=LOG_FIELDS)

thread_head.plot_timing()  # <<<<<<<<<<<<< Comment out to skip timings plot
