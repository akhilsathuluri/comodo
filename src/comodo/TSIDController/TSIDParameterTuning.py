import numpy as np


class TSIDParameterTuning:
    def __init__(self) -> None:
        scale = 0.5
        self.CoM_Kp = 15.0 * scale
        self.CoM_Kd = 7.0 * scale

        # since we only have one leg + symmetry
        self.postural_Kp = (
            np.array(
                [
                    150,
                    20,
                    20,
                    180,
                ]
                * 2
            )
            * 1.0
        )  # TODO symmetry

        self.postural_weight = 10 * np.ones(len(self.postural_Kp))
        self.foot_tracking_task_kp_lin = 30.0 * scale
        self.foot_tracking_task_kd_lin = 7.0 * scale
        self.foot_tracking_task_kp_ang = 300.0 * scale
        self.foot_tracking_task_kd_ang = 10.0 * scale
        self.root_tracking_task_weight = 1 * np.ones(3) * scale
        self.root_link_kp_ang = 20.0 * scale
        self.root_link_kd_ang = 10.0 * scale

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
        self.set_postural_gain(x_k[:4])
        self.set_foot_task(x_k[4], x_k[5], x_k[6], x_k[7])
        self.set_root_task(x_k[8], x_k[9])
        self.set_com_task(x_k[10], x_k[11])
