import numpy as np
import yaml
import os


class Config:
    def __init__(self) -> None:
        
        current_dir = os.path.dirname(os.path.abspath(__file__))
        mujoco_yaml_path = os.path.join(current_dir, "config", "real.yaml")
        with open(mujoco_yaml_path, "r") as f:
            config = yaml.load(f, Loader=yaml.FullLoader)
            self.net = config["net"]
            self.num_joints = config["num_joints"]
            self.lowcmd_topic = config["lowcmd_topic"]
            self.lowstate_topic = config["lowstate_topic"]
            self.control_dt = config["control_dt"]
            self.error_over_time = config["error_over_time"]
            self.enable_fk_state_estimation = config.get("enable_fk_state_estimation", True)
            self.fk_xml_path = config.get("fk_xml_path", "g1_description/g1_29dof_rev_1_0.xml")
            self.fk_contact_height_threshold = config.get("fk_contact_height_threshold", 0.03)
            
