import numpy as np
import yaml

class TSIDParameterTuning:
    def __init__(self, cfg_file_path) -> None:

        with open(cfg_file_path, 'r') as file:
            params = yaml.safe_load(file)
        
        scale = 0.5

        self.CoM_Kp = params['CoM_Kp'] * scale
        self.CoM_Kd = params['CoM_Kd']* scale
        self.postural_Kp = np.array(params['postural_Kp']) * 1.0
        self.postural_weight = np.array(params['postural_weight']) * 10
        self.foot_tracking_task_kp_lin = params['foot_tracking_task_kp_lin'] * scale
        self.foot_tracking_task_kd_lin = params['foot_tracking_task_kd_lin'] * scale
        self.foot_tracking_task_kp_ang = params['foot_tracking_task_kp_ang'] * scale
        self.foot_tracking_task_kd_ang = params['foot_tracking_task_kd_ang'] * scale
        self.root_tracking_task_weight = np.array(params['root_tracking_task_weight']) * scale
        self.root_link_kp_ang = params['root_link_kp_ang'] * scale
        self.root_link_kd_ang = params['root_link_kd_ang'] * scale

    def set_postural_gain(self, leg):
        self.postural_Kp = np.concatenate([leg, leg])

    def set_foot_task(self, kp_lin, kd_lin, kp_ang, kd_ang):
        self.foot_tracking_task_kp_lin = kp_lin
        self.foot_tracking_task_kd_lin = kd_lin
        self.foot_tracking_task_kp_ang = kp_ang
        self.foot_tracking_task_kd_ang = kd_ang

    def set_weights(self, postural_weight, root_weight):
        self.postural_weight = postural_weight
        self.root_tracking_task_weight = root_weight

    def set_root_task(self, kp_ang, kd_ang):
        self.root_link_kp_ang = kp_ang
        self.root_link_kd_ang = kd_ang

    def set_com_task(self, kp_com, kd_com):
        self.CoM_Kd = kd_com
        self.CoM_Kp = kp_com

    def set_from_x_k(self, x_k):
        # self.set_postural_gain(x_k[:4])
        # self.set_foot_task(x_k[4], x_k[5], x_k[6], x_k[7])
        # self.set_root_task(x_k[8], x_k[9])
        # self.set_com_task(x_k[10], x_k[11])
        raise NotImplementedError
