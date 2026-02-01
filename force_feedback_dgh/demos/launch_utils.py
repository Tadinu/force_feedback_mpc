import yaml
import os
import sys
sys.path.append('.')
from croco_mpc_utils.utils import CustomLogger, GLOBAL_LOG_LEVEL, GLOBAL_LOG_FORMAT
logger = CustomLogger(__name__, GLOBAL_LOG_LEVEL, GLOBAL_LOG_FORMAT).logger


# # # # # # # # # # # # # # #
# EXPERIMENT LOADING UTILS  #
# # # # # # # # # # # # # # #

SUPPORTED_EXPERIMENTS = ['calibration',
                         'polishing_classical',
                         'polishing_lpf', 
                         'polishing_soft',
                         'polishing_classical_obstacle',
                         'polishing_classical_obstacle_vicon',
                         'polishing_classical_square',
                         'polishing_lpf_obstacle',
                         'polishing_soft_obstacle',
                         'polishing_soft_obstacle_vicon',
                         'polishing_soft_square',
                         'force_tracking_classical',
                         'force_tracking_lpf',
                         'force_tracking_soft']


def is_valid_exp_name(EXP_NAME):
    '''
    Check that exp name is valid
    '''
    try: 
        assert(EXP_NAME in SUPPORTED_EXPERIMENTS)
    except NameError:
        logger.error("Error : config file name must be in "+str(SUPPORTED_EXPERIMENTS))
        
        
def load_config_file(EXP_NAME, path_prefix=''):
    '''
    Load YAML config file corresponding to an experiment name
    '''
    is_valid_exp_name(EXP_NAME)
    config_path = os.path.join(path_prefix, 'config/'+EXP_NAME+".yml")
    logger.debug("Opening config file "+str(config_path))
    with open(config_path) as f:
        data = yaml.load(f, Loader=yaml.FullLoader)
    return data


def import_mpc_controller(EXP_NAME):
    '''
    Imports the MPC controller class corresponding to an experiment name
    '''
    is_valid_exp_name(EXP_NAME)
    if(EXP_NAME == 'polishing_classical'):     
        from controllers.polishing_classical import ClassicalMPCPolishing as MPCController
    elif(EXP_NAME == 'polishing_lpf'):  
        from controllers.polishing_lpf import LpfMPCPolishing  as MPCController
    elif(EXP_NAME == 'polishing_soft'): 
        from controllers.polishing_soft import SoftMPCPolishing as MPCController
    elif(EXP_NAME == 'polishing_classical_obstacle'):     
        from controllers.polishing_classical_obstacle import ClassicalMPCPolishingObstacle as MPCController
    elif(EXP_NAME == 'polishing_classical_obstacle_vicon'):     
        from controllers.polishing_classical_obstacle_vicon import ClassicalMPCPolishingObstacleVicon as MPCController
    elif(EXP_NAME == 'polishing_classical_square'):     
        from controllers.polishing_classical_square import ClassicalMPCPolishingSquare as MPCController
    elif(EXP_NAME == 'polishing_lpf_obstacle'):  
        from controllers.polishing_lpf_obstacle import LpfMPCPolishingObstacle  as MPCController
    elif(EXP_NAME == 'polishing_soft_obstacle'): 
        from controllers.polishing_soft_obstacle import SoftMPCPolishingObstacle as MPCController
    elif(EXP_NAME == 'polishing_soft_obstacle_vicon'): 
        from controllers.polishing_soft_obstacle_vicon import SoftMPCPolishingObstacleVicon as MPCController
    elif(EXP_NAME == 'polishing_soft_square'): 
        from controllers.polishing_soft_square import SoftMPCPolishingSquare as MPCController
    elif(EXP_NAME == 'force_tracking_classical'): 
        from controllers.force_tracking_classical import ClassicalMPCForceTrackinng as MPCController
    elif(EXP_NAME == 'force_tracking_lpf'): 
        from controllers.force_tracking_lpf import LpfMPCForceTracking as MPCController
    elif(EXP_NAME == 'force_tracking_soft'): 
        from controllers.force_tracking_soft import SoftMPCForceTracking as MPCController
    logger.debug("Imported MPC controller for experiment : "+str(EXP_NAME))
    return MPCController


def get_log_config(EXP_NAME):
    '''
    Returns the log configuration for an experiment name
    '''
    is_valid_exp_name(EXP_NAME)
    if(EXP_NAME == 'polishing_classical'):     
        log_config = POLISHING_LOGS_CLASSICAL
    elif(EXP_NAME == 'polishing_lpf'):  
        log_config = POLISHING_LOGS_LPF
    elif(EXP_NAME == 'polishing_soft'): 
        log_config = POLISHING_LOGS_SOFT
    elif(EXP_NAME == 'polishing_classical_obstacle'): 
        log_config = POLISHING_LOGS_CLASSICAL_OBSTACLE
    elif(EXP_NAME == 'polishing_classical_obstacle_vicon'): 
        log_config = POLISHING_LOGS_CLASSICAL_OBSTACLE_VICON
    elif(EXP_NAME == 'polishing_classical_square'): 
        log_config = POLISHING_LOGS_CLASSICAL_SQUARE
    elif(EXP_NAME == 'polishing_lpf_obstacle'):  
        log_config = POLISHING_LOGS_LPF_OBSTACLE
    elif(EXP_NAME == 'polishing_soft_obstacle'): 
        log_config = POLISHING_LOGS_SOFT_OBSTACLE
    elif(EXP_NAME == 'polishing_soft_obstacle_vicon'): 
        log_config = POLISHING_LOGS_SOFT_OBSTACLE_VICON
    elif(EXP_NAME == 'polishing_soft_square'): 
        log_config = POLISHING_LOGS_SOFT_SQUARE
    elif(EXP_NAME == 'force_tracking_classical'): 
        log_config = FORCE_TRACKING_LOGS_CLASSICAL
    elif(EXP_NAME == 'force_tracking_lpf'): 
        log_config = FORCE_TRACKING_LOGS_LPF
    elif(EXP_NAME == 'force_tracking_soft'): 
        log_config = FORCE_TRACKING_LOGS_SOFT
    elif(EXP_NAME == 'calibration'):
        log_config = LOGS_CALIBRATION 
    logger.debug("Data log fields : "+str(log_config))
    return log_config


def get_robot_name(EXP_NAME):
    '''
    Returns the URDF file name  
    '''
    is_valid_exp_name(EXP_NAME)
    if(EXP_NAME == 'polishing_classical'): 
        robot_name = 'iiwa_ft_sensor_shell'
    elif(EXP_NAME == 'polishing_lpf'):  
        robot_name = 'iiwa_ft_sensor_shell'
    elif(EXP_NAME == 'polishing_soft'): 
        robot_name = 'iiwa_ft_sensor_shell'
    elif(EXP_NAME == 'polishing_classical_obstacle'): 
        robot_name = 'iiwa_convex_ft_sensor_shell'
    elif(EXP_NAME == 'polishing_classical_obstacle_vicon'): 
        robot_name = 'iiwa_convex_ft_sensor_shell'
    elif(EXP_NAME == 'polishing_classical_square'): 
        robot_name = 'iiwa_convex_ft_sensor_shell'
    elif(EXP_NAME == 'polishing_lpf_obstacle'): 
        robot_name = 'iiwa_convex_ft_sensor_shell'
    elif(EXP_NAME == 'polishing_soft_obstacle'): 
        robot_name = 'iiwa_convex_ft_sensor_shell'
    elif(EXP_NAME == 'polishing_soft_obstacle_vicon'): 
        robot_name = 'iiwa_convex_ft_sensor_shell'
    elif(EXP_NAME == 'polishing_soft_square'): 
        robot_name = 'iiwa_convex_ft_sensor_shell'
    elif(EXP_NAME == 'force_tracking_classical'): 
        robot_name = 'iiwa_ft_sensor_shell'
    elif(EXP_NAME == 'force_tracking_lpf'): 
        robot_name = 'iiwa_ft_sensor_shell'
    elif(EXP_NAME == 'force_tracking_soft'): 
        robot_name = 'iiwa_ft_sensor_shell'
    logger.debug("Robot name : "+str(robot_name))
    return robot_name

# # # # # # # # # # # # 
# DATA LOGGING UTILS  #
# # # # # # # # # # # # 

LOGS_NONE = []

SSQP_LOGS_MINIMAL = ['KKT', 
                     'sqp_iter',
                     't_child']

CSQP_LOGS_MINIMAL = ['KKT',
                     'qp_iters',
                     'cost',
                     'gap_norm',
                     'constraint_norm',
                     'sqp_iter',
                     't_child']

POLISHING_LOGS_CLASSICAL = ['contact_force_3d_measured',
                            'target_force',
                            'KKT',
                            'sqp_iter',
                            't_child',
                            'joint_positions',
                            'target_position_x',
                            'target_position_y',
                            'target_position_z',
                            'force_est',
                            'joint_torques_measured',
                            'tau',
                            'joint_cmd_torques',
                            'tau_gravity',
                            'tau_ff',
                            'joint_velocities']
 
POLISHING_LOGS_LPF = ['contact_force_3d_measured',
                      'target_force',
                      'KKT',
                      'sqp_iter',
                      't_child',
                      'target_position_x',
                      'target_position_y',
                      'target_position_z',
                      'force_est',
                      'w',
                      'tau',
                      'tau_gravity',
                      'tau_estimated',
                      'tau_ff',
                      'joint_ext_torques',
                      'joint_ext_torques_filtered',
                      'joint_torques_measured_filtered',
                      'joint_cmd_torques',
                      'joint_torques_total',
                      'joint_torques_measured',
                      'joint_positions',
                      'joint_velocities',
                      'joint_accelerations',
                      'joint_accelerations_filtered']

POLISHING_LOGS_SOFT = ['contact_force_3d_measured',
                       'target_force',
                       'KKT',
                       'sqp_iter',
                       't_child',
                       'joint_positions',
                       'target_position_x',
                       'target_position_y',
                       'target_position_z',
                       'force_est',
                       'joint_torques_measured',
                       'tau',
                       'joint_cmd_torques',
                       'tau_gravity',
                       'tau_ff',
                       'joint_velocities', 
                       'obstacle1_xyzquat_pose',
                       'obstacle2_xyzquat_pose']

POLISHING_LOGS_CLASSICAL_OBSTACLE = ['contact_force_3d_measured',
                                    'target_force',
                                    'KKT',
                                    'qp_iters',
                                    'cost',
                                    'gap_norm',
                                    'constraint_norm',
                                    'sqp_iter',
                                    't_child',
                                    'joint_positions',
                                    'target_position_x',
                                    'target_position_y',
                                    'target_position_z',
                                    'force_est',
                                    'joint_torques_measured',
                                    'tau',
                                    'joint_cmd_torques',
                                    'tau_gravity',
                                    'tau_ff',
                                    'joint_velocities',
                                    'obstacle1_xyzquat_pose',
                                    'obstacle2_xyzquat_pose']

POLISHING_LOGS_CLASSICAL_OBSTACLE_VICON = POLISHING_LOGS_CLASSICAL_OBSTACLE + ['rod_pos_raw',
                    'rod_pos',
                    'rod_vel',
                    'rod_pos_raw',
                    'obstacle_visible',
                    'obstacle_safe',
                    'obstacle_quat_solver']

POLISHING_LOGS_CLASSICAL_SQUARE = POLISHING_LOGS_CLASSICAL_OBSTACLE + ['force_integral']

POLISHING_LOGS_LPF_OBSTACLE = ['contact_force_3d_measured',
                                'target_force',
                                'KKT',
                                'qp_iters',
                                'cost',
                                'gap_norm',
                                'constraint_norm',
                                'sqp_iter',
                                't_child',
                                'target_position_x',
                                'target_position_y',
                                'target_position_z',
                                'force_est',
                                'w',
                                'tau',
                                'tau_gravity',
                                'tau_estimated',
                                'tau_ff',
                                'joint_ext_torques',
                                'joint_ext_torques_filtered',
                                'joint_torques_measured_filtered',
                                'joint_cmd_torques',
                                'joint_torques_total',
                                'joint_torques_measured',
                                'joint_positions',
                                'joint_velocities',
                                'joint_accelerations',
                                'joint_accelerations_filtered']

POLISHING_LOGS_SOFT_OBSTACLE = ['contact_force_3d_measured',
                                'target_force',
                                'KKT',
                                'qp_iters',
                                'cost',
                                'gap_norm',
                                'constraint_norm',
                                'sqp_iter',
                                't_child',
                                'joint_positions',
                                'target_position_x',
                                'target_position_y',
                                'target_position_z',
                                'force_est',
                                'joint_torques_measured',
                                'tau',
                                'joint_cmd_torques',
                                'tau_gravity',
                                'tau_ff',
                                'joint_velocities'] 

POLISHING_LOGS_SOFT_OBSTACLE_VICON = POLISHING_LOGS_SOFT_OBSTACLE + ['rod_pos_raw',
                    'rod_pos',
                    'rod_vel',
                    'rod_pos_raw',
                    'obstacle_visible',
                    'obstacle_safe',
                    'obstacle_quat_solver']

POLISHING_LOGS_SOFT_SQUARE = POLISHING_LOGS_SOFT_OBSTACLE 

FORCE_TRACKING_LOGS_CLASSICAL = POLISHING_LOGS_CLASSICAL
FORCE_TRACKING_LOGS_LPF       = POLISHING_LOGS_LPF
FORCE_TRACKING_LOGS_SOFT      = POLISHING_LOGS_SOFT

LOGS_CALIBRATION = ['joint_positions',
                    'joint_velocities',
                    'tau',
                    'tau_gravity',
                    'joint_torques_measured',
                    'joint_cmd_torques',
                    'kuka_ef_pos',
                    'kuka_ef_vel']