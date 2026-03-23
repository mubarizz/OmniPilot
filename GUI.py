import sys
import socket
import struct
import subprocess
import numpy as np
from collections import deque

from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget,
                              QHBoxLayout, QVBoxLayout, QPushButton, QLabel, QLineEdit)
from PyQt5.QtCore import Qt, QTimer
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D

# ── ROTATION MATRIX ──────────────────────────────────────────────────────────
def rotation_matrix(phi, theta, psi):
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

ARM_LEN    = 0.08
VIEW_RANGE = 0.5

def get_drone_arms(x, y, z, phi, theta, psi):
    R = rotation_matrix(phi, theta, psi)
    motors = np.array([
        [ ARM_LEN,  0,       0],
        [-ARM_LEN,  0,       0],
        [ 0,        ARM_LEN, 0],
        [ 0,       -ARM_LEN, 0],
    ])
    rotated = (R @ motors.T).T
    arm_x_pts = np.array([rotated[0], [0,0,0], rotated[1]]) + [x, y, z]
    arm_y_pts = np.array([rotated[2], [0,0,0], rotated[3]]) + [x, y, z]
    return arm_x_pts, arm_y_pts

# ── MAIN APP CLASS ────────────────────────────────────────────────────────────
class DroneSimApp(QMainWindow):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Drone Simulator")
        self.resize(1200, 700)

        self.xs = deque()
        self.ys = deque()
        self.zs = deque()
        self.state = {"x":0.0,"y":0.0,"z":0.0,
                      "phi":0.0,"theta":0.0,"psi":0.0}
        self.running = False

        # receiving socket (MATLAB → Python)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 5005))
        self.sock.setblocking(False)

        # sending socket (Python → MATLAB)
        self.sock_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self._build_ui()

        self.timer = QTimer()
        self.timer.setInterval(50)
        self.timer.timeout.connect(self.update_plot)

    def _build_ui(self):

        # ── TOP BAR ──
        self.btn_simulate = QPushButton("▶  Simulate")
        self.btn_simulate.setFixedHeight(36)
        self.btn_simulate.setStyleSheet("""
            QPushButton {
                background-color: #1a1a2e;
                color: #00ffcc;
                border: 1px solid #00ffcc;
                border-radius: 6px;
                font-size: 13px;
                padding: 0 20px;
            }
            QPushButton:hover { background-color: #00ffcc; color: #0d0d0d; }
        """)
        self.btn_simulate.clicked.connect(self.on_simulate)

        self.status_label = QLabel("● Waiting for simulation...")
        self.status_label.setStyleSheet(
            "color: #666666; font-size: 12px; font-family: monospace;")

        top_bar = QWidget()
        top_bar.setFixedHeight(52)
        top_bar.setStyleSheet("background-color: #111111;")
        top_layout = QHBoxLayout()
        top_layout.addWidget(self.btn_simulate)
        top_layout.addWidget(self.status_label)
        top_layout.addStretch()
        top_bar.setLayout(top_layout)

        # ── MAIN 3D VIEW (left) ──
        self.fig_main = Figure(facecolor="#0d0d0d")
        self.canvas_main = FigureCanvas(self.fig_main)
        self.ax_main = self.fig_main.add_subplot(111, projection='3d')
        self.ax_main.set_facecolor("#0d0d0d")
        self.ax_main.set_xlabel("X", color="#ff6b6b")
        self.ax_main.set_ylabel("Y", color="#ffd93d")
        self.ax_main.set_zlabel("Z", color="#00ffcc")
        self.ax_main.tick_params(colors='gray', labelsize=7)
        for pane in [self.ax_main.xaxis.pane,
                     self.ax_main.yaxis.pane,
                     self.ax_main.zaxis.pane]:
            pane.fill = False
            pane.set_edgecolor('#333333')

        self.trail,  = self.ax_main.plot([], [], [], color="#00aaff", lw=1.0, alpha=0.5)
        self.arm_x,  = self.ax_main.plot([], [], [], color="#ff6b6b", lw=2.5)
        self.arm_y,  = self.ax_main.plot([], [], [], color="#ffd93d", lw=2.5)
        self.center, = self.ax_main.plot([], [], [], 'o', color="#00ffcc", ms=5)
        # target marker on main view
        self.target_marker, = self.ax_main.plot([], [], [], '*',
                                                 color="#ff9900", ms=10, zorder=5)

        # ── CLOSE-UP VIEW (right) ──
        self.fig_close = Figure(facecolor="#0d0d0d")
        self.canvas_close = FigureCanvas(self.fig_close)
        self.ax_close = self.fig_close.add_subplot(111, projection='3d')
        self.ax_close.set_facecolor("#0d0d0d")
        self.ax_close.set_title("Drone Close-up", color="white", fontsize=9)
        self.ax_close.tick_params(colors='gray', labelsize=7)
        for pane in [self.ax_close.xaxis.pane,
                     self.ax_close.yaxis.pane,
                     self.ax_close.zaxis.pane]:
            pane.fill = False
            pane.set_edgecolor('#333333')

        # close-up arms are always at origin — rotation only, no translation
        self.cl_arm_x,  = self.ax_close.plot([], [], [], color="#ff6b6b", lw=2.5)
        self.cl_arm_y,  = self.ax_close.plot([], [], [], color="#ffd93d", lw=2.5)
        self.cl_center, = self.ax_close.plot([], [], [], 'o', color="#00ffcc", ms=6)

        # ── TARGET PANEL ──
        lbl_style = "color: #888888; font-size: 11px; font-family: monospace;"
        inp_style = """
            QLineEdit {
                background-color: #1a1a2e;
                color: #ffffff;
                border: 1px solid #333333;
                border-radius: 4px;
                padding: 2px 6px;
                font-family: monospace;
                font-size: 12px;
                max-width: 70px;
            }
            QLineEdit:focus { border: 1px solid #00ffcc; }
        """

        # x
        lbl_x = QLabel("x (m):")
        lbl_x.setStyleSheet(lbl_style)
        self.input_x = QLineEdit("0.0")
        self.input_x.setStyleSheet(inp_style)

        # y
        lbl_y = QLabel("y (m):")
        lbl_y.setStyleSheet(lbl_style)
        self.input_y = QLineEdit("0.0")
        self.input_y.setStyleSheet(inp_style)

        # z
        lbl_z = QLabel("z (m):")
        lbl_z.setStyleSheet(lbl_style)
        self.input_z = QLineEdit("1.5")
        self.input_z.setStyleSheet(inp_style)

        # psi
        lbl_psi = QLabel("ψ yaw (°):")
        lbl_psi.setStyleSheet(lbl_style)
        self.input_psi = QLineEdit("10.0")
        self.input_psi.setStyleSheet(inp_style)

        # send button
        btn_send = QPushButton("Send Target")
        btn_send.setFixedHeight(30)
        btn_send.setStyleSheet("""
            QPushButton {
                background-color: #1a1a2e;
                color: #ff9900;
                border: 1px solid #ff9900;
                border-radius: 4px;
                font-size: 12px;
                padding: 0 14px;
            }
            QPushButton:hover { background-color: #ff9900; color: #0d0d0d; }
        """)
        btn_send.clicked.connect(self.send_target)

        target_bar = QWidget()
        target_bar.setFixedHeight(48)
        target_bar.setStyleSheet("background-color: #111111; border-top: 1px solid #222222;")
        target_layout = QHBoxLayout()
        target_layout.setContentsMargins(10, 6, 10, 6)

        for widget in [lbl_x, self.input_x,
                       lbl_y, self.input_y,
                       lbl_z, self.input_z,
                       lbl_psi, self.input_psi,
                       btn_send]:
            target_layout.addWidget(widget)

        target_layout.addStretch()
        target_bar.setLayout(target_layout)

        # ── LAYOUTS ──
        canvas_row = QWidget()
        canvas_layout = QHBoxLayout()
        canvas_layout.setContentsMargins(0, 0, 0, 0)
        canvas_layout.addWidget(self.canvas_main, 70)
        canvas_layout.addWidget(self.canvas_close, 30)
        canvas_row.setLayout(canvas_layout)

        central_widget = QWidget()
        central_widget.setStyleSheet("background-color: #0d0d0d;")
        main_layout = QVBoxLayout()
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)
        main_layout.addWidget(top_bar)
        main_layout.addWidget(target_bar)
        main_layout.addWidget(canvas_row)
        central_widget.setLayout(main_layout)

        self.setCentralWidget(central_widget)

        # store current target for marker display
        self.current_target = [0.0, 0.0, 1.5]  # x, y, z

    def on_simulate(self):
        if self.running:
            return

        subprocess.Popen([
            r"C:\Program Files\MATLAB\R2025b\bin\matlab.exe",
            "-batch", "main"
        ])

        self.running = True
        self.btn_simulate.setText("⏳  Running...")
        self.btn_simulate.setStyleSheet("""
            QPushButton {
                background-color: #1a1a2e;
                color: #666666;
                border: 1px solid #444444;
                border-radius: 6px;
                font-size: 13px;
                padding: 0 20px;
            }
        """)
        self.status_label.setText("● Simulation running...")
        self.status_label.setStyleSheet(
            "color: #00ffcc; font-size: 12px; font-family: monospace;")

        self.timer.start()

    def update_plot(self):
        # drain all available UDP packets
        while True:
            try:
                raw, _ = self.sock.recvfrom(1024)
                if len(raw) == 24:
                    x, y, z, phi, theta, psi = struct.unpack('6f', raw)
                    self.xs.append(x)
                    self.ys.append(y)
                    self.zs.append(z)
                    self.state.update(x=x, y=y, z=z,
                                      phi=phi, theta=theta, psi=psi)
            except BlockingIOError:
                break

        if len(self.zs) == 0:
            return

        xl, yl, zl = list(self.xs), list(self.ys), list(self.zs)
        s = self.state

        # ── UPDATE MAIN VIEW ──
        self.trail.set_data_3d(xl, yl, zl)

        ax_pts, ay_pts = get_drone_arms(s['x'], s['y'], s['z'],
                                        s['phi'], s['theta'], s['psi'])
        self.arm_x.set_data_3d(ax_pts[:,0], ax_pts[:,1], ax_pts[:,2])
        self.arm_y.set_data_3d(ay_pts[:,0], ay_pts[:,1], ay_pts[:,2])
        self.center.set_data_3d([s['x']], [s['y']], [s['z']])

        # target marker — orange star showing where drone should go
        tx, ty, tz = self.current_target
        self.target_marker.set_data_3d([tx], [ty], [tz])

        margin = 0.5
        self.ax_main.set_xlim(min(xl)-margin, max(xl)+margin)
        self.ax_main.set_ylim(min(yl)-margin, max(yl)+margin)
        self.ax_main.set_zlim(min(zl)-margin, max(zl)+margin)

        self.canvas_main.draw()

        # ── UPDATE CLOSE-UP VIEW — always centered at origin, rotation only ──
        ax_pts_local, ay_pts_local = get_drone_arms(0, 0, 0,
                                                     s['phi'], s['theta'], s['psi'])
        self.cl_arm_x.set_data_3d(ax_pts_local[:,0], ax_pts_local[:,1], ax_pts_local[:,2])
        self.cl_arm_y.set_data_3d(ay_pts_local[:,0], ay_pts_local[:,1], ay_pts_local[:,2])
        self.cl_center.set_data_3d([0], [0], [0])

        self.ax_close.set_xlim(-VIEW_RANGE, VIEW_RANGE)
        self.ax_close.set_ylim(-VIEW_RANGE, VIEW_RANGE)
        self.ax_close.set_zlim(-VIEW_RANGE, VIEW_RANGE)

        self.canvas_close.draw()

    def send_target(self):
        try:
            x   = float(self.input_x.text())
            y   = float(self.input_y.text())
            z   = float(self.input_z.text())
            psi = float(self.input_psi.text())

            # send 5 values: x, y, z, psi(radians)
            # MATLAB outer PID will compute phi and theta from x and y errors
            data = struct.pack('<4f', x, y, z, np.radians(psi))
            self.sock_out.sendto(data, ("127.0.0.1", 5006))

            # update target marker on main view
            self.current_target = [x, y, z]

            self.status_label.setText(
                f"● Target sent → x:{x}m  y:{y}m  z:{z}m  ψ:{psi}°")
        except ValueError:
            self.status_label.setText("● Invalid input — enter numbers only")

# ── RUN ───────────────────────────────────────────────────────────────────────
app = QApplication(sys.argv)
window = DroneSimApp()
window.show()
sys.exit(app.exec_())
