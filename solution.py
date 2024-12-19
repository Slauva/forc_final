from pathlib import Path
import numpy as np
import os 
import pinocchio as pin
from simulator import Simulator
from utils import so3_error, jacobian
from convert import convert_to_gif
from dataclasses import dataclass

import matplotlib.pyplot as plt

@dataclass
class Monitor:
    e = []
    de = []
    s = []
    dV = []
    t = []
    tau = []
    xpos = []
    xdes = []

def convert_desired(desired: dict) -> tuple[pin.SE3, np.ndarray, np.ndarray]:
    desired_position = desired['pos']
    desired_quaternion = desired['quat']
    desired_quaternion_pin = np.array([*desired_quaternion[1:], desired_quaternion[0]])
    desired_se3 = pin.XYZQUATToSE3(np.concat([desired_position, desired_quaternion_pin]))
    return desired_se3, np.zeros((6,)), np.zeros((6,))
        
def task_space_controller(q: np.ndarray, dq: np.ndarray, t: float, desired: dict) -> np.ndarray:    
    # Compute all dynamics quantities
    ee_frame_id = model.getFrameId("end_effector")
    pin.computeAllTerms(model, data, q, dq)
    current = pin.updateFramePlacement(model, data, ee_frame_id)
    desired, dx_d, ddx_d = convert_desired(desired)
    Monitor.xpos.append(pin.SE3ToXYZQUAT(current))
    Monitor.xdes.append(pin.SE3ToXYZQUAT(desired))
    
    # Get current model matricies
    J, dJ = jacobian(model, data, ee_frame_id)
    Minv = np.linalg.inv(data.M)
    
    Mx = np.linalg.pinv(np.linalg.multi_dot([J, Minv, J.T]))
    Cx = np.linalg.multi_dot([Mx, J, Minv, data.C]) - Mx @ dJ
    gx = np.linalg.multi_dot([Mx, J, Minv, data.g])
    
    # Compute error
    e = so3_error(current, desired)
    de = dx_d - J @ dq
    
    # Koeff
    kp = 100
    kd = 20
    
    # Compute torque
    ddx_d = kp * e + kd * de - dJ @ dq
    f = Mx @ ddx_d + Cx @ de + gx
    tau = J.T @ f
    
    # Collect data monitor
    Monitor.t.append(t)
    Monitor.e.append(e)
    Monitor.de.append(de)
    Monitor.tau.append(tau)
    
    return tau

def sign(x: np.ndarray, e: float = 0, mode = 1) -> np.ndarray:
    if mode == 1:
        return x / (np.linalg.norm(x) + e)
    else:
        return x / np.linalg.norm(x) if np.linalg.norm(x) > e else x / e

def sliding_controller(q: np.ndarray, dq: np.ndarray, t: float, desired: dict) -> np.ndarray:
    # Compute all dynamics quantities
    ee_frame_id = model.getFrameId("end_effector")
    pin.computeAllTerms(model, data, q, dq)
    current = pin.updateFramePlacement(model, data, ee_frame_id)
    desired, dx_d, ddx_d = convert_desired(desired)
    Monitor.xpos.append(pin.SE3ToXYZQUAT(current))
    Monitor.xdes.append(pin.SE3ToXYZQUAT(desired))
    
    
    # Get current model matricies
    J, dJ = jacobian(model, data, ee_frame_id)
    
    # Compute error
    e = so3_error(current, desired)
    de = dx_d - J @ dq
    
    # Koeff [60, 8, 12] [120, 8, 12]    
    L, Kd, eps = np.array([120, 8, 12])
    
    # Sliding window
    s = de + L * e
    # print(s)
    
    # Lyapunov Stability Analysis
    dV = - Kd * np.linalg.norm(s)
    # print(dV, dV <= 0)
    
    # Convert to angles
    dq_d = np.linalg.inv(J) @ (dx_d + L * e)
    ddq_d = np.linalg.inv(J) @ ((ddx_d + L * de) - dJ @ dq)
    
    # Compute torque
    tau = data.M @ (ddq_d + L * (dq_d - dq)) + data.nle - Kd * sign(s, eps, mode=2)
    
    # Collect data monitor
    Monitor.t.append(t)
    Monitor.e.append(e)
    Monitor.de.append(de)
    Monitor.s.append(s)
    Monitor.dV.append(dV)
    Monitor.tau.append(tau)
    
    return tau

def pos_ori_plotter(fname: str = None):
    names = [
        "x", "y", "z", "q0", "q1", "q2", "q3"
    ]
    
    t = np.array(Monitor.t)
    xpos = np.array(Monitor.xpos)
    xdes = np.array(Monitor.xdes)
    
    fig, axes = plt.subplots(2, 1, figsize = (12, 12))
    axes[0].set_title("Position of End-Effector and Desired Point")
    axes[0].set_ylabel("Position")
    axes[1].set_title("Orientation of End-Effector and Desired Point")
    axes[1].set_ylabel("Orientation by Quaternion")
    
    for i, name in enumerate(names[:3]):
        axes[0].plot(t, xpos[:, i], label=name)
        axes[0].plot(t, xdes[:, i], label=f"Desired {name}", linestyle="--", alpha=0.7)
    for i, name in enumerate(names[3:]):
        axes[1].plot(t, xpos[:, i+3], label=name)
        axes[1].plot(t, xdes[:, i+3], label=f"Desired {name}", linestyle="--", alpha=0.7)
        
    for j in range(2):
        axes[j].grid(True, linewidth=1, linestyle="--", alpha=0.7, color="gray")
        axes[j].legend(ncol=3+j)
        axes[j].set_xlabel("Time (s)")
    
    plt.subplots_adjust(hspace=0.6)
    if fname is None:
        plt.show()
    else:
        plt.savefig(f"logs/plots/{fname}.svg")

def error_plotter(fname: str = None):
    joint_names = [
            "shoulder_pan",
            "shoulder_lift",
            "elbow",
            "wrist_1",
            "wrist_2",
            "wrist_3",
    ]
    
    t = np.array(Monitor.t)
    e = np.array(Monitor.e)
    de = np.array(Monitor.de)
    s = np.array(Monitor.s if len(Monitor.s) != 0 else np.zeros_like(de))
    
    fig, axes = plt.subplots(3, 1, figsize = (12, 18))
    axes[0].set_title(r"Position Error $e = x_{des} - x$")
    axes[0].set_ylabel(r"Position")
    axes[1].set_title(r"Velocity Error $\dot e = \dot x_{des} - J \dot q$")
    axes[1].set_ylabel(r"Velocity")
    axes[2].set_title(r"Sliding Window $s = \dot e + \Lambda e$")
    axes[2].set_ylabel(r"Sliding Parameter")
    
    for i, name in enumerate(joint_names):
        axes[0].plot(t, e[:, i], label=name)
    for i, name in enumerate(joint_names):
        axes[1].plot(t, de[:, i], label=name)
    for i, name in enumerate(joint_names):
        axes[2].plot(t, s[:, i], label=name)
        
    for j in range(3):
        axes[j].grid(True, linewidth=1, linestyle="--", alpha=0.7, color="gray")
        axes[j].legend(ncol=len(joint_names))
        axes[j].set_xlabel("Time (s)")
        
    plt.subplots_adjust(hspace=0.6)
    if fname is None:
        plt.show()
    else:
        plt.savefig(f"logs/plots/{fname}.svg")
    
def lyapunov_tau_plotter(fname: str = None):
    joint_names = [
            "shoulder_pan",
            "shoulder_lift",
            "elbow",
            "wrist_1",
            "wrist_2",
            "wrist_3",
    ]
    
    t = np.array(Monitor.t)
    tau = np.array(Monitor.tau)
    dV = np.array(Monitor.dV if len(Monitor.dV) != 0 else np.zeros_like(t))
    
    fig, axes = plt.subplots(2, 1, figsize = (12, 12))
    axes[0].set_title(r"Torque on Joints")
    axes[0].set_ylabel(r"Torque $\tau$")
    axes[1].set_title(r"Lyapunov Stability $\dot V = -K * ||s||$")
    axes[1].set_ylabel("Lyapunov Stability")
    
    for i, name in enumerate(joint_names):
        axes[0].plot(t, tau[:, i], label=name)
    axes[1].plot(t, dV)
        
    for j in range(2):
        axes[j].grid(True, linewidth=1, linestyle="--", alpha=0.7, color="gray")
        axes[j].legend(ncol=3)
        axes[j].set_xlabel("Time (s)")
        
    plt.subplots_adjust(hspace=0.6)
    if fname is None:
        plt.show()
    else:
        plt.savefig(f"logs/plots/{fname}.svg")

def main():
    # Create logging directories
    Path("logs/videos").mkdir(parents=True, exist_ok=True)
    Path("logs/plots").mkdir(parents=True, exist_ok=True)
    
    sim = Simulator(
        xml_path="robots/universal_robots_ur5e/scene.xml",
        enable_task_space=True,
        show_viewer=True,
        record_video=True,
        video_path="logs/videos/unknown.mp4",
        fps=30,
        width=1920,
        height=1080 
    )
    
    # Set joint damping coefficients
    damping = np.array([0.5, 0.5, 0.5, 0.1, 0.1, 0.1]) # Nm/rad/s
    sim.set_joint_damping(damping)
    
    # Set joint friction coefficients
    friction = np.array([1.5, 0.5, 0.5, 0.1, 0.1, 0.1]) # Nm
    sim.set_joint_friction(friction)
    
    # Modify end-effector mass
    sim.modify_body_properties("end_effector", mass=4)
    
    sim.video_path = Path("logs/videos/tsc.mp4")
    sim.set_controller(task_space_controller)
    sim.run(time_limit=1.5)
    pos_ori_plotter("se3_tsc")
    error_plotter("err_tsc")
    lyapunov_tau_plotter("lya_tsc")
    
    sim.video_path = Path("logs/videos/sliding.mp4")
    sim.set_controller(sliding_controller)
    sim.run(time_limit=1.5)
    pos_ori_plotter("se3_sliding")
    error_plotter("err_sliding")
    lyapunov_tau_plotter("lya_sliding")
    
    convert_to_gif()
    

if __name__ == "__main__":
    # Import the model
    current_dir = os.path.dirname(os.path.abspath(__file__))
    xml_path = os.path.join(current_dir, "robots/universal_robots_ur5e/ur5e.xml")
    model = pin.buildModelFromMJCF(xml_path)
    data = model.createData()
    main()