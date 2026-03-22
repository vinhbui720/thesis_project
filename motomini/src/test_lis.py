import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

def generate_precession_poses(center, R_frame1, alpha, steps):
    poses = []
    for i in range(steps):
        theta = 2 * np.pi * i / steps

        # Step 2: rotate Frame1 around its Z by theta → Frame 2
        R_step2 = R.from_rotvec(theta * R_frame1.apply([0, 0, 1]))
        R_frame2 = R_step2 * R_frame1

        # Step 3: tilt Frame2 around its X by alpha → Frame 3
        R_step3 = R.from_rotvec(alpha * R_frame2.apply([1, 0, 0]))
        R_frame3 = R_step3 * R_frame2

        poses.append((center.copy(), R_frame3))
    return poses


def plot_poses(poses, axis_len=0.02):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    colors = {'x': 'red', 'y': 'green', 'z': 'blue'}
    axes_dirs = {
        'x': np.array([1, 0, 0]),
        'y': np.array([0, 1, 0]),
        'z': np.array([0, 0, 1]),
    }

    for idx, (pos, rot) in enumerate(poses):
        # Plot origin point
        ax.scatter(*pos, color='black', s=20, zorder=5)
        ax.text(pos[0], pos[1], pos[2] + 0.005, f'wp_{idx}', fontsize=7)

        # Plot X Y Z axes of each pose
        for axis_name, axis_dir in axes_dirs.items():
            world_dir = rot.apply(axis_dir)
            end = pos + axis_len * world_dir
            ax.quiver(
                pos[0], pos[1], pos[2],
                world_dir[0], world_dir[1], world_dir[2],
                length=axis_len,
                color=colors[axis_name],
                normalize=False
            )

    # Draw a light cone outline connecting Z-axis tips
    z_tips = []
    for pos, rot in poses:
        z_world = rot.apply([0, 0, 1])
        tip = pos + axis_len * z_world
        z_tips.append(tip)

    # Close the loop
    z_tips.append(z_tips[0])
    z_tips = np.array(z_tips)
    ax.plot(z_tips[:, 0], z_tips[:, 1], z_tips[:, 2],
            'b--', linewidth=1.0, alpha=0.5, label='Z-tip cone')

    # Reference frame 1 (target base frame)
    center = poses[0][0]
    for axis_name, axis_dir in axes_dirs.items():
        ax.quiver(
            center[0], center[1], center[2],
            axis_dir[0], axis_dir[1], axis_dir[2],
            length=axis_len * 2,
            color=colors[axis_name],
            alpha=0.3,
            linewidth=2,
            linestyle='dashed'
        )

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Precession Poses\nRed=X  Green=Y  Blue=Z  Dashed=Frame1')

    # Legend
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], color='red',   label='X axis'),
        Line2D([0], [0], color='green', label='Y axis'),
        Line2D([0], [0], color='blue',  label='Z axis'),
        Line2D([0], [0], color='blue',  linestyle='dashed', alpha=0.5, label='Z-tip cone'),
    ]
    ax.legend(handles=legend_elements)

    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    # --- Same config as test1.py ---
    center    = np.array([0.18, 0.1, 0.24])
    R_frame1  = R.from_euler('xyz', [0, 0, 0])
    alpha     = np.deg2rad(20)
    steps     = 8

    poses = generate_precession_poses(center, R_frame1, alpha, steps)
    plot_poses(poses, axis_len=0.02)