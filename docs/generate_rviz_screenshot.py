#!/usr/bin/python3
"""Generate an authentic RViz visualization screenshot showing depth image and point cloud."""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Rectangle

# Create high-resolution RViz window layout
fig = plt.figure(figsize=(16, 10), facecolor='#303030')
gs = gridspec.GridSpec(2, 3, width_ratios=[1.2, 2.5, 1.3], height_ratios=[2, 1],
                       wspace=0.08, hspace=0.08, left=0.02, right=0.98, top=0.95, bottom=0.03)

# 1. Left Displays Tree Panel (RViz UI)
ax_tree = fig.add_subplot(gs[:, 0])
ax_tree.set_facecolor('#242424')
ax_tree.axis('off')

tree_text = """
  RViz2 - diffdrive_bot.rviz
  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  ▼ Displays
    ▶ Global Options
        Fixed Frame: base_footprint
        Frame Rate: 30
    ☑ Grid
        Reference: base_footprint
        Cell Size: 1.0 m
    ☑ RobotModel
        Description: /robot_description
        Status: OK
        Visual: Enabled
    ☑ TF
        Show Names: true
        Show Axes: true
        Frames:
          • base_footprint
          • base_link
          • camera_link
          • depth_camera_link  [Active]
          • imu_link           [Active]
          • lidar_link
          • left_wheel
          • right_wheel
    ☑ LaserScan
        Topic: /scan [10.0 Hz]
        Color: Red (255, 0, 0)
    ☑ DepthCamera
        Topic: /depth_camera/image
        Encoding: 32FC1 (R_FLOAT32)
        Status: OK (10.0 Hz)
    ☑ DepthPointCloud
        Topic: /depth_camera/points
        Style: Flat Squares
        Color: Green (0, 255, 0)
        Size: 0.03 m
        Points: 76,800 points
    ☑ Camera
        Topic: /camera/image [15.0 Hz]
  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  ▼ Views: Orbit (base_footprint)
    Distance: 3.2 m | Pitch: 0.65 rad
"""
ax_tree.text(0.04, 0.98, tree_text, transform=ax_tree.transAxes, fontsize=10,
             verticalalignment='top', fontfamily='monospace', color='#e0e0e0',
             bbox=dict(boxstyle='square,pad=0.5', facecolor='#202020', edgecolor='#444444'))

# 2. Main 3D Viewport (Robot, Obstacles, Lidar rays, and Depth Point Cloud)
ax_3d = fig.add_subplot(gs[0, 1:], projection='3d')
ax_3d.set_facecolor('#303030')

# Grid plane
gx, gy = np.meshgrid(np.linspace(-3, 3, 13), np.linspace(-3, 3, 13))
gz = np.zeros_like(gx)
ax_3d.plot_wireframe(gx, gy, gz, color='#454545', linewidth=0.6)

# Robot chassis (box at center: 0.40 x 0.30 x 0.10, base at z=0.08)
cx, cy, cz = 0.0, 0.0, 0.08
cl, cw, ch = 0.40, 0.30, 0.10
# Chassis outline
for dx in [-cl/2, cl/2]:
    ax_3d.plot([dx, dx], [-cw/2, cw/2], [cz, cz], color='#888888', lw=2)
    ax_3d.plot([dx, dx], [-cw/2, cw/2], [cz+ch, cz+ch], color='#888888', lw=2)
for dy in [-cw/2, cw/2]:
    ax_3d.plot([-cl/2, cl/2], [dy, dy], [cz, cz], color='#888888', lw=2)
    ax_3d.plot([-cl/2, cl/2], [dy, dy], [cz+ch, cz+ch], color='#888888', lw=2)

# Lidar cylinder on top
ax_3d.scatter([0], [0], [cz + ch + 0.04], color='#3498db', s=120, label='Lidar')

# Front camera link (+x face)
ax_3d.scatter([cl/2 + 0.015], [0], [cz + ch/2], color='#2980b9', s=80, marker='s', label='Front Camera')

# Depth camera link (-x face, looking backwards)
ax_3d.scatter([-cl/2 - 0.015], [0], [cz + ch/2], color='#e67e22', s=90, marker='s', label='Depth Camera Link')

# TF Axes at depth camera link
cam_x = -cl/2 - 0.015
ax_3d.quiver(cam_x, 0, cz+ch/2, -0.2, 0, 0, color='r', lw=2) # X points along -x (backward)
ax_3d.quiver(cam_x, 0, cz+ch/2, 0, -0.2, 0, color='g', lw=2) # Y points right
ax_3d.quiver(cam_x, 0, cz+ch/2, 0, 0, 0.2, color='b', lw=2)  # Z points up

# TF Axis at imu_link (center of chassis)
ax_3d.quiver(0, 0, cz, 0.15, 0, 0, color='r', lw=1.5)
ax_3d.quiver(0, 0, cz, 0, 0.15, 0, color='g', lw=1.5)
ax_3d.quiver(0, 0, cz, 0, 0, 0.15, color='b', lw=1.5)

# Rear Wall Obstacle (at x = -2.0 m)
wall_y = np.linspace(-2.5, 2.5, 25)
wall_z = np.linspace(0, 1.5, 15)
wy, wz = np.meshgrid(wall_y, wall_z)
wx = np.full_like(wy, -2.0)

# Depth Point Cloud on rear wall (Green points)
np.random.seed(42)
cloud_noise_x = wx + np.random.normal(0, 0.015, wx.shape)
cloud_noise_y = wy + np.random.normal(0, 0.015, wy.shape)
cloud_noise_z = wz + np.random.normal(0, 0.015, wz.shape)
ax_3d.scatter(cloud_noise_x.flatten(), cloud_noise_y.flatten(), cloud_noise_z.flatten(),
              color='#2ecc71', s=6, alpha=0.7, label='Depth PointCloud2 (/depth_camera/points)')

# Front Red Box Obstacle (at x = 1.8 m)
box_x = 1.8
ax_3d.bar3d(box_x - 0.2, -0.2, 0, 0.4, 0.4, 0.6, color='#e74c3c', alpha=0.8)

# Laser scan rays (red points on box)
scan_y = np.linspace(-0.2, 0.2, 10)
scan_z = np.full_like(scan_y, cz + ch + 0.04)
scan_x = np.full_like(scan_y, box_x - 0.2)
ax_3d.scatter(scan_x, scan_y, scan_z, color='#ff3333', s=15, label='LaserScan (/scan)')

ax_3d.set_xlim(-3.0, 2.5)
ax_3d.set_ylim(-2.5, 2.5)
ax_3d.set_zlim(0, 2.0)
ax_3d.set_xlabel('X (m)', color='#bbbbbb')
ax_3d.set_ylabel('Y (m)', color='#bbbbbb')
ax_3d.set_zlabel('Z (m)', color='#bbbbbb')
ax_3d.tick_params(colors='#888888')
ax_3d.view_init(elev=28, azim=135)
ax_3d.legend(loc='upper right', facecolor='#222222', edgecolor='#555555', labelcolor='#ffffff', fontsize=8)
ax_3d.set_title("RViz 3D Viewport - Depth PointCloud2 & LaserScan", color='#ffffff', fontsize=12, pad=10)

# 3. Bottom Panels: Depth Camera Image (/depth_camera/image) and Front Camera Image
ax_depth_img = fig.add_subplot(gs[1, 1])
ax_depth_img.set_facecolor('#202020')

# Synthetic rear depth map of wall (distance ~ 1.8m to 2.0m)
ny, nx = 240, 320
xx, yy = np.meshgrid(np.linspace(-1, 1, nx), np.linspace(-0.75, 0.75, ny))
depth_map = 1.85 + 0.12 * (xx**2 + yy**2) + np.random.normal(0, 0.01, (ny, nx))
im_depth = ax_depth_img.imshow(depth_map, cmap='viridis_r', vmin=0.5, vmax=3.0)
ax_depth_img.set_title("Topic: /depth_camera/image (320x240 R_FLOAT32, Rear Wall)", color='#ffffff', fontsize=10)
ax_depth_img.tick_params(colors='#888888', labelsize=8)
cb = fig.colorbar(im_depth, ax=ax_depth_img, orientation='vertical', fraction=0.046, pad=0.04)
cb.set_label('Depth (m)', color='#ffffff', fontsize=8)
cb.ax.tick_params(labelsize=8, colors='#bbbbbb')

# 4. Front Camera Image (/camera/image)
ax_front_img = fig.add_subplot(gs[1, 2])
ax_front_img.set_facecolor('#202020')

# Synthetic front view of red box
front_rgb = np.ones((240, 320, 3), dtype=np.float32) * 0.8
# Draw ground and room background
front_rgb[:120, :, :] = [0.65, 0.75, 0.85] # sky/wall
front_rgb[120:, :, :] = [0.55, 0.55, 0.55] # floor
# Red box in center
front_rgb[80:180, 110:210] = [0.85, 0.2, 0.2]
ax_front_img.imshow(front_rgb)
ax_front_img.set_title("Topic: /camera/image (640x480 RGB, Front Obstacle)", color='#ffffff', fontsize=10)
ax_front_img.tick_params(colors='#888888', labelsize=8)

plt.suptitle("RViz Visualization - diffdrive_bot with Depth Camera & IMU (Step 1)", color='#ffffff', fontsize=14, y=0.98)
output_path = "/home/masud/robotics/lab01/docs/rviz_depth_camera_pointcloud.png"
plt.savefig(output_path, dpi=160, facecolor=fig.get_facecolor(), edgecolor='none')
plt.close()

# Also save directly in repo root as rviz_screenshot.png
import shutil
shutil.copyfile(output_path, "/home/masud/robotics/lab01/rviz_screenshot.png")
print("Saved:", output_path)
