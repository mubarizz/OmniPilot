import socket
import struct
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
from collections import deque
import numpy as np

UDP_IP   = "127.0.0.1"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)

xs = deque()
ys = deque()
zs = deque()

# latest state
state = {"x":0.0, "y":0.0, "z":0.0,
         "phi":0.0, "theta":0.0, "psi":0.0}

fig = plt.figure(facecolor="#0d0d0d", figsize=(11, 5))
fig.suptitle("Drone 3D Trajectory", color="white", fontsize=13)

# ── LEFT: 3D trajectory ──────────────────────────────────────────────────────
ax = fig.add_subplot(121, projection='3d')
ax.set_facecolor("#0d0d0d")
ax.set_xlabel("X (m)", color="#ff6b6b")
ax.set_ylabel("Y (m)", color="#ffd93d")
ax.set_zlabel("Z (m)", color="#00ffcc")
ax.tick_params(colors='gray', labelsize=7)
for pane in [ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane]:
    pane.fill = False
    pane.set_edgecolor('#333333')

trail,    = ax.plot([], [], [], color="#00aaff", lw=1.0, alpha=0.5)
arm_x,    = ax.plot([], [], [], color="#ff6b6b", lw=2.5)  # x-axis arms
arm_y,    = ax.plot([], [], [], color="#ffd93d", lw=2.5)  # y-axis arms
center,   = ax.plot([], [], [], 'o', color="#00ffcc", ms=5)

# ── RIGHT: attitude readout ──────────────────────────────────────────────────
ax2 = fig.add_subplot(122, facecolor="#0d0d0d")
ax2.axis('off')
ax2.set_title("Attitude", color="white", fontsize=11)

txt_style = dict(transform=ax2.transAxes, fontsize=12,
                 fontfamily="monospace", va='top')
t_phi   = ax2.text(0.05, 0.85, "", color="#ff9900", **txt_style)
t_theta = ax2.text(0.05, 0.65, "", color="#ff6b6b", **txt_style)
t_psi   = ax2.text(0.05, 0.45, "", color="#00ffcc", **txt_style)
t_pos   = ax2.text(0.05, 0.20, "", color="#aaaaaa", **txt_style)

# ── ROTATION MATRIX (ZYX / Tait-Bryan) ──────────────────────────────────────
def rotation_matrix(phi, theta, psi):
    """Build ZYX rotation matrix from roll(phi) pitch(theta) yaw(psi) in radians."""
    Rz = np.array([[np.cos(psi), -np.sin(psi), 0],
                   [np.sin(psi),  np.cos(psi), 0],
                   [0,            0,           1]])
    Ry = np.array([[ np.cos(theta), 0, np.sin(theta)],
                   [0,              1, 0            ],
                   [-np.sin(theta), 0, np.cos(theta)]])
    Rx = np.array([[1, 0,           0          ],
                   [0, np.cos(phi), -np.sin(phi)],
                   [0, np.sin(phi),  np.cos(phi)]])
    return Rz @ Ry @ Rx

ARM_LEN = 0.046  # arm length in metres — adjust to taste

def get_drone_arms(x, y, z, phi, theta, psi):
    """Return rotated arm endpoint coordinates centered at (x,y,z)."""
    R = rotation_matrix(phi, theta, psi)

    # 4 motor positions in body frame (+ shape)
    motors = np.array([
        [ ARM_LEN,  0,       0],   # front
        [-ARM_LEN,  0,       0],   # back
        [ 0,        ARM_LEN, 0],   # right
        [ 0,       -ARM_LEN, 0],   # left
    ])

    rotated = (R @ motors.T).T  # rotate all 4 points

    cx, cy, cz = x, y, z

    # x-axis arm: front ↔ back
    arm_x_pts = np.array([rotated[0], [0,0,0], rotated[1]]) + [cx, cy, cz]
    # y-axis arm: right ↔ left
    arm_y_pts = np.array([rotated[2], [0,0,0], rotated[3]]) + [cx, cy, cz]

    return arm_x_pts, arm_y_pts

# ── ANIMATION UPDATE ─────────────────────────────────────────────────────────

def update(frame):
    while True:
        try:
            raw, _ = sock.recvfrom(1024)
            if len(raw) == 24:
                x, y, z, phi, theta, psi = struct.unpack('6f', raw)
                xs.append(x)
                ys.append(y)
                zs.append(z)
                state.update(x=x, y=y, z=z,
                             phi=phi, theta=theta, psi=psi)
        except BlockingIOError:
            break

    if len(zs) == 0:
        return trail, arm_x, arm_y, center, t_phi, t_theta, t_psi, t_pos

    xl, yl, zl = list(xs), list(ys), list(zs)
    s = state

    # -- trajectory trail
    trail.set_data_3d(xl, yl, zl)

    # -- drone body
    ax_pts, ay_pts = get_drone_arms(s['x'], s['y'], s['z'],
                                    s['phi'], s['theta'], s['psi'])
    arm_x.set_data_3d(ax_pts[:,0], ax_pts[:,1], ax_pts[:,2])
    arm_y.set_data_3d(ay_pts[:,0], ay_pts[:,1], ay_pts[:,2])
    center.set_data_3d([s['x']], [s['y']], [s['z']])

    # -- auto scale
    margin = 0.5
    ax.set_xlim(min(xl) - margin, max(xl) + margin)
    ax.set_ylim(min(yl) - margin, max(yl) + margin)
    ax.set_zlim(min(zl) - margin, max(zl) + margin)

    # -- attitude text
    t_phi.set_text(  f"φ  roll  : {np.degrees(s['phi']):+.2f}°")
    t_theta.set_text(f"θ  pitch : {np.degrees(s['theta']):+.2f}°")
    t_psi.set_text(  f"ψ  yaw   : {np.degrees(s['psi']):+.2f}°")
    t_pos.set_text(  f"x: {s['x']:+.3f} m\n"
                     f"y: {s['y']:+.3f} m\n"
                     f"z: {s['z']:+.3f} m")

    return trail, arm_x, arm_y, center, t_phi, t_theta, t_psi, t_pos

ani = animation.FuncAnimation(fig, update, interval=50,
                               blit=False, cache_frame_data=False)
plt.tight_layout()
plt.show()
sock.close()