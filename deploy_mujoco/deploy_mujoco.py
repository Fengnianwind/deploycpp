import sys
from pathlib import Path
sys.path.append(str(Path(__file__).parent.parent.absolute()))

from common.path_config import PROJECT_ROOT

import time
import mujoco.viewer
import mujoco
import numpy as np
import yaml
import os
from common.ctrlcomp import *
from FSM.FSM import *
from common.utils import get_gravity_orientation

# 使用 pynput 替代 pygame
from pynput import keyboard
import threading


def extract_contact_body_pairs(model, data):
    pairs = []
    for i in range(data.ncon):
        contact = data.contact[i]
        body1 = int(model.geom_bodyid[contact.geom1])
        body2 = int(model.geom_bodyid[contact.geom2])
        pairs.append((body1, body2))
    return pairs


def extract_body_angular_velocity_local(model, data, body_name):
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    velocity = np.zeros(6, dtype=np.float64)
    mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, body_id, velocity, 1)
    return velocity[:3].astype(np.float32)


def pd_control(target_q, q, kp, target_dq, dq, kd):
    """Calculates torques from position commands"""
    return (target_q - q) * kp + (target_dq - dq) * kd

class KeyboardControl:
    def __init__(self):
        self.running = True
        self.state_cmd = None
        
        # 按键状态
        self.key_states = {
            'up': False,    # 上箭头 或 W
            'down': False,  # 下箭头 或 S  
            'left': False,  # 左箭头 或 A
            'right': False, # 右箭头 或 D
            'j': False,     # 左转
            'l': False,     # 右转
            'e': False,     # R1
            'q': False,     # L1
            'w': False,     # W键
            'a': False,     # A键
            's': False,     # S键
            'd': False,     # D键
        }
        
        # 技能命令
        self.skill_cmd = None
        
        # 启动键盘监听
        self.listener_thread = threading.Thread(target=self._start_key_listener, daemon=True)
        self.listener_thread.start()
    
    def _start_key_listener(self):
        """启动键盘监听"""
        def on_press(key):
            try:
                key_char = key.char.lower()
                
                # 更新按键状态
                if key_char in self.key_states:
                    self.key_states[key_char] = True
                
                # 特殊功能键
                if key_char == 'm':  # START键
                    self.skill_cmd = "POS_RESET"
                    print("切换到位置重置模式")
                
                elif key_char == 'f':  # L3键
                    self.skill_cmd = "PASSIVE" 
                    print("切换到被动模式")
                
                elif key_char == 'backspace':  # BACKSPACE
                    print("BACKSPACE键按下 - 机器人站立")
                
                # 组合键检测
                if key_char == 'z' and self.key_states['e']:  # A + R1
                    self.skill_cmd = "LOCO"
                    print("切换到行走模式")
                
                elif key_char == 'a' and self.key_states['e']:  # X + R1
                    self.skill_cmd = "SKILL_1"
                    print("技能1")
                
                elif key_char == 's' and self.key_states['e']:  # Y + R1  
                    self.skill_cmd = "SKILL_2"
                    print("AnyAdapter")
                
                elif key_char == 'x' and self.key_states['e']:  # B + R1
                    self.skill_cmd = "SKILL_3"
                    print("技能3")
                
                elif key_char == 's' and self.key_states['q']:  # Y + L1
                    self.skill_cmd = "SKILL_4"
                    print("技能4")
                    
            except AttributeError:
                # 处理特殊键（方向键等）
                if key == keyboard.Key.up:
                    self.key_states['up'] = True
                elif key == keyboard.Key.down:
                    self.key_states['down'] = True
                elif key == keyboard.Key.left:
                    self.key_states['left'] = True
                elif key == keyboard.Key.right:
                    self.key_states['right'] = True
                elif key == keyboard.Key.esc:
                    self.running = False
                    print("退出程序")
        
        def on_release(key):
            try:
                key_char = key.char.lower()
                if key_char in self.key_states:
                    self.key_states[key_char] = False
            except AttributeError:
                if key == keyboard.Key.up:
                    self.key_states['up'] = False
                elif key == keyboard.Key.down:
                    self.key_states['down'] = False
                elif key == keyboard.Key.left:
                    self.key_states['left'] = False
                elif key == keyboard.Key.right:
                    self.key_states['right'] = False
        
        with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
            listener.join()
    
    def get_velocity_command(self):
        """获取速度命令"""
        # 前后移动
        forward_speed = 0.0
        if self.key_states['up'] or self.key_states['w']:
            forward_speed = 1.0
        elif self.key_states['down'] or self.key_states['s']:
            forward_speed = -1.0
        
        # 左右移动  
        lateral_speed = 0.0
        if self.key_states['left'] or self.key_states['a']:
            lateral_speed = -1.0
        elif self.key_states['right'] or self.key_states['d']:
            lateral_speed = 1.0
        
        # 转向
        turn_speed = 0.0
        if self.key_states['j']:
            turn_speed = 1.0
        elif self.key_states['l']:
            turn_speed = -1.0
        
        return [forward_speed, lateral_speed, turn_speed]
    
    def get_skill_command(self):
        """获取技能命令并重置"""
        cmd = self.skill_cmd
        self.skill_cmd = None  # 重置，避免重复触发
        return cmd

if __name__ == "__main__":
    current_dir = os.path.dirname(os.path.abspath(__file__))
    mujoco_yaml_path = os.path.join(current_dir, "config", "mujoco.yaml")
    with open(mujoco_yaml_path, "r") as f:
        config = yaml.load(f, Loader=yaml.FullLoader)
        xml_path = os.path.join(PROJECT_ROOT, config["xml_path"])
        simulation_dt = config["simulation_dt"]
        control_decimation = config["control_decimation"]
        
    m = mujoco.MjModel.from_xml_path(xml_path)
    d = mujoco.MjData(m)
    m.opt.timestep = simulation_dt
    mj_per_step_duration = simulation_dt * control_decimation
    num_joints = m.nu
    policy_output_action = np.zeros(num_joints, dtype=np.float32)
    kps = np.zeros(num_joints, dtype=np.float32)
    kds = np.zeros(num_joints, dtype=np.float32)
    sim_counter = 0
    
    state_cmd = StateAndCmd(num_joints)
    policy_output = PolicyOutput(num_joints)
    FSM_controller = FSM(state_cmd, policy_output)
    
    # 使用新的键盘控制器
    keyboard_control = KeyboardControl()
    
    print("=== 键盘控制说明 ===")
    print("1. 先按 m键 进入位控模式")
    print("2. 同时按住 E键 + Z键 进入行走模式")
    print("3. 按 BACKSPACE键 使机器人站立") 
    print("4. 使用方向键或WASD控制移动")
    print("5. J键: 左转, L键: 右转")
    print("6. 按 ESC键 退出")
    print("====================")
    
    with mujoco.viewer.launch_passive(m, d) as viewer:
        sim_start_time = time.time()
        while viewer.is_running() and keyboard_control.running:
            step_start = time.time()  # 将 step_start 移到循环开头
            
            try:
                # 获取技能命令
                skill_cmd = keyboard_control.get_skill_command()
                if skill_cmd:
                    if skill_cmd == "POS_RESET":
                        state_cmd.skill_cmd = FSMCommand.POS_RESET
                    elif skill_cmd == "PASSIVE":
                        state_cmd.skill_cmd = FSMCommand.PASSIVE
                    elif skill_cmd == "LOCO":
                        state_cmd.skill_cmd = FSMCommand.LOCO
                    elif skill_cmd == "SKILL_1":
                        state_cmd.skill_cmd = FSMCommand.SKILL_1
                    elif skill_cmd == "SKILL_2":
                        state_cmd.skill_cmd = FSMCommand.SKILL_2
                    elif skill_cmd == "SKILL_3":
                        state_cmd.skill_cmd = FSMCommand.SKILL_3
                    elif skill_cmd == "SKILL_4":
                        state_cmd.skill_cmd = FSMCommand.SKILL_4
                    print(f"执行技能命令: {skill_cmd}")
                
                # 获取速度命令
                vel_cmd = keyboard_control.get_velocity_command()
                state_cmd.vel_cmd[0] = vel_cmd[0]  # 前后
                state_cmd.vel_cmd[1] = vel_cmd[1]  # 左右
                state_cmd.vel_cmd[2] = vel_cmd[2]  # 转向
                
                # 调试信息
                if sim_counter % 100 == 0:
                    active_keys = [k for k, v in keyboard_control.key_states.items() if v]
                    if active_keys:
                        print(f"激活按键: {active_keys}")
                    #print(f"速度命令: 前{vel_cmd[0]:.1f} 侧{vel_cmd[1]:.1f} 转{vel_cmd[2]:.1f}")
                
                # 控制计算
                tau = pd_control(policy_output_action, d.qpos[7:], kps, np.zeros_like(kps), d.qvel[6:], kds)
                d.ctrl[:] = tau
                mujoco.mj_step(m, d)
                sim_counter += 1
                
                if sim_counter % control_decimation == 0:
                    qj = d.qpos[7:]
                    dqj = d.qvel[6:]
                    quat = d.qpos[3:7]
                    
                    omega = extract_body_angular_velocity_local(m, d, "pelvis")
                    gravity_orientation = get_gravity_orientation(quat)
                    
                    state_cmd.q = qj.copy()
                    state_cmd.dq = dqj.copy()
                    state_cmd.gravity_ori = gravity_orientation.copy()
                    state_cmd.base_quat = quat.copy()
                    state_cmd.ang_vel = omega.copy()
                    state_cmd.body_pos_w = d.xpos.copy()
                    state_cmd.body_quat_w = d.xquat.copy()
                    state_cmd.contact_body_pairs = extract_contact_body_pairs(m, d)
                    
                    FSM_controller.run()
                    policy_output_action = policy_output.actions.copy()
                    kps = policy_output.kps.copy()
                    kds = policy_output.kds.copy()
                    
            except Exception as e:
                print(f"错误: {str(e)}")
                import traceback
                traceback.print_exc()
            
            viewer.sync()
            
            # 控制循环频率
            time_until_next_step = m.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

    print("程序正常退出")
