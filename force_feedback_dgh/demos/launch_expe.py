import numpy as np
from datetime import datetime

import os
import pathlib
import sys
import pybullet as p
import pinocchio as pin

# eigenpy, crocoddyl: Add LD_LIBRARY_PATH to Env Var configs:
# LD_LIBRARY_PATH=/media/ducthan/376b23a1-5a02-4960-b3ca-24b2fcef8f891/10_IMPEDANCE_CONTROL/crocoddyl/release/lib:/media/ducthan/376b23a1-5a02-4960-b3ca-24b2fcef8f891/10_IMPEDANCE_CONTROL/pinocchio/release/lib:/media/ducthan/376b23a1-5a02-4960-b3ca-24b2fcef8f891/10_IMPEDANCE_CONTROL/eigenpy/release/lib:$LD_LIBRARY_PATH

# force_feedback_dgh
from force_feedback_dgh.mim_robots.robot_loader import load_bullet_wrapper, load_pinocchio_wrapper
from force_feedback_dgh.mim_robots.pybullet.env import BulletEnvWithGround
from force_feedback_dgh.mim_robots.robot_list import MiM_Robots
from utils.find_contact_point import compute_sensor_frame_transform
from dynamic_graph_head import ThreadHead, SimHead, HoldPDController, SimForcePlate, SimVicon, Vicon

# force_feedback_mpc
force_feedback_mpc_path = pathlib.Path(
    '.').absolute().parent.parent / 'force_feedback_mpc' / 'release/lib/python3.10/site-packages'
sys.path.insert(0, str(force_feedback_mpc_path))
from force_feedback_mpc.core_mpc_utils import sim_utils

# mim_solvers: uv pip install mim_solvers==0.1.1
import launch_utils

SIM = True
if not SIM:
    import dynamic_graph_manager_cpp_bindings

# # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# Choose experiment, load config and import controller  #  
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #
CONFIG_DIR = pathlib.Path(os.path.abspath(__file__)).parent.parent
EXP_NAME = 'polishing_soft_obstacle_vicon'  # <<<<<<<<<<<<< Choose experiment here (cf. launch_utils)
config = launch_utils.load_config_file(EXP_NAME, path_prefix=CONFIG_DIR)
MPCController = launch_utils.import_mpc_controller(EXP_NAME)
ROBOT_NAME = launch_utils.get_robot_name(EXP_NAME)

# # # # # # # # # # # #
# Import robot model  #
# # # # # # # # # # # #
LOCKED_JOINTS = ['A7']
pin_robot = load_pinocchio_wrapper(ROBOT_NAME, locked_joints=LOCKED_JOINTS)

# Add FT sensor frame to the model of the robot - this calibration routine changes the pinocchio model !
_, _, cMs, _ = compute_sensor_frame_transform(pin_robot, mount_piece_type='shell')

# # # # # # # # # # # # #
# Setup control thread  #
# # # # # # # # # # # # #
if SIM:
    # Sim env + set initial state 
    config['T_tot'] = 20
    env = BulletEnvWithGround(p.GUI)
    robot_simulator = load_bullet_wrapper(ROBOT_NAME, locked_joints=LOCKED_JOINTS)
    robot_simulator.pin_robot = pin_robot
    env.add_robot(robot_simulator)
    q_init = np.asarray(config['q0'])
    v_init = np.asarray(config['dq0'])
    robot_simulator.reset_state(q_init, v_init)
    robot_simulator.forward_robot(q_init, v_init)
    # <<<<< Customize your PyBullet environment here if necessary
    # Display reach target, contact surface & desired contact point 
    sim_utils.display_ball(np.asarray(config['contactPosition']) + np.asarray(config['oPc_offset']), RADIUS=0.02,
                           COLOR=[1., 0., 0., 0.2])
    contactId = sim_utils.display_contact_surface(pin.SE3(np.eye(3), np.asarray(config['contactPosition'])),
                                                  bullet_endeff_ids=robot_simulator.bullet_endeff_ids)
    sim_utils.set_lateral_friction(contactId, 0.5)
    sim_utils.set_contact_stiffness_and_damping(contactId, 10000, 500)
    # # Add obstacles 
    if ('obstacle' in EXP_NAME):
        print('\n Adding obstacle in PyBullet simulation \n')
        obstacle_id = sim_utils.setup_obstacle_collision(robot_simulator, robot_simulator.pin_robot, config,
                                                         TYPE=config['OBSTACLE_TYPE'])
    if ('vicon' in EXP_NAME):
        vicon = SimVicon(['tool/tool'])
    head = SimHead(robot_simulator, with_sliders=False, with_force_plate=True)
# !!!!!!!!!!!!!!!!
# !! REAL ROBOT !!
# !!!!!!!!!!!!!!!!
else:
    # Add obstacles 
    if ('obstacle' in EXP_NAME):
        print('\n Adding obstacle in real robot model \n')
        sim_utils.setup_obstacle_collision_no_sim(pin_robot, config, TYPE=config['OBSTACLE_TYPE'])
    config['T_tot'] = 400
    path = MiM_Robots['iiwa'].dgm_path
    print(path)
    head = dynamic_graph_manager_cpp_bindings.DGMHead(path)
    target = None
    env = None
    obstacle_id = None
    if ('vicon' in EXP_NAME):
        vicon = Vicon('172.24.117.119:801', ['tool/tool'])

ctrl = MPCController(head, pin_robot, config, run_sim=SIM, cMs=cMs, locked_joints=LOCKED_JOINTS)

if (SIM):
    # to allow obstacle update in pybullet when smulating vicon
    if ('obstacle' in EXP_NAME and 'vicon' in EXP_NAME):
        ctrl.obstacle_id = obstacle_id
    thread_head = ThreadHead(
        1. / config['ctrl_freq'],  # dt.
        HoldPDController(head, 50., 0.5, with_sliders=False),  # Safety controllers.
        head,  # Heads to read / write from.
        [('force_plate', SimForcePlate([robot_simulator]))],  # Simulated force plate
        env  # Environment to step.
    )
else:
    if ('vicon' in EXP_NAME):
        thread_head = ThreadHead(
            1. / config['ctrl_freq'],  # dt.
            HoldPDController(head, 50., 0.5, with_sliders=False),  # Safety controllers.
            head,  # Heads to read / write from.
            [
                ('vicon', vicon)
            ],
            env  # Environment to step.
        )
    else:
        thread_head = ThreadHead(
            1. / config['ctrl_freq'],  # dt.
            HoldPDController(head, 50., 0.5, with_sliders=False),  # Safety controllers.
            head,  # Heads to read / write from.
            [],
            env  # Environment to step.
        )

thread_head.switch_controllers(ctrl)

# # # # # # # # #
# Data logging  #
# # # # # # # # # <<<<<<<<<<<<< Choose data save path & log config here (cf. launch_utils)
# prefix     = "/home/skleff/Desktop/constrained_polishing/polishing_square/"
prefix = "TRO_RESUBMISSION/polishing_obstacle/vicon/"
# suffix     = '_fc=10_COULOMB='+str(config['USE_COULOMB_FRICTION'])+'_ESTIMATE_JOINT_TORQUES='+str(config['ESTIMATE_JOINT_TORQUES'])+'_OMEGA='+str(config['circle_omega'])
# suffix     = '_fc=10_'+'_USE_FORCE_INTEGRAL='+str(config['USE_FORCE_INTEGRAL'])+'_OMEGA='+str(config['circle_omega'])
suffix = '_ELBOW_PUSH'
LOG_FIELDS = launch_utils.get_log_config(EXP_NAME)

# # # # # # # # # # # 
# Launch experiment #
# # # # # # # # # # # 
if SIM:
    thread_head.start_logging(int(config['T_tot']),
                              prefix + EXP_NAME + "_SIM_" + str(datetime.now().isoformat()) + suffix + ".mds",
                              LOG_FIELDS=LOG_FIELDS)
    thread_head.sim_run_timed(int(config['T_tot']))
    thread_head.stop_logging()
else:
    thread_head.start()
    thread_head.start_logging(60, prefix + EXP_NAME + "_REAL_" + str(datetime.now().isoformat()) + suffix + ".mds",
                              LOG_FIELDS=LOG_FIELDS)

# thread_head.plot_timing() # <<<<<<<<<<<<< Comment out to skip timings plot
