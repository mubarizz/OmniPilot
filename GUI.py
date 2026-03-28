import sys
import socket
import struct
import subprocess
import numpy as np
from collections import deque
from PyQt5 import QtWidgets, uic
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

ARM_LEN    = 0.25
VIEW_RANGE = 0.15

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
        uic.loadUi("designer.ui", self)
        self.matlab_process = None

        self.xs = deque()
        self.ys = deque()
        self.zs = deque()
        self.state = {"x":0.0,"y":0.0,"z":0.0,
                      "phi":0.0,"theta":0.0,"psi":0.0}
        self.running = False
        self.current_target = [0,0,1.5]
        # receiving socket (MATLAB → Python)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 5005))
        self.sock.setblocking(False)

        # sending socket (Python → MATLAB)
        self.sock_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.setup_plots()
        self.btn_launchsimulation.clicked.connect(self.on_simulate)
        self.btn_sendtarget.clicked.connect(self.send_target)

        self.timer = QTimer()
        self.timer.setInterval(50)
        self.timer.timeout.connect(self.update_plot)


        #self.slider_ki.valueChanged.connect(self.send_gains)
        #.slider_kd.valueChanged.connect(self.send_gains)
        #self.btn_retest.clicked.connect(self.reset_simulation)


    def setup_plots(self):
        self.fig_main = Figure(facecolor="#0d0d0d")
        self.canvas_main = FigureCanvas(self.fig_main)
        main_layout = QVBoxLayout(self.graph_3d)
        main_layout.addWidget(self.canvas_main)
        self.ax_main = self.fig_main.add_subplot(111, projection='3d')
        self.ax_main.set_facecolor("#0d0d0d")
        self.ax_main.set_xlabel("X", color="#ff6b6b")
        self.ax_main.set_ylabel("Y", color="#ffd93d")
        self.ax_main.set_zlabel("Z", color="#00ffcc")
        self.ax_main.tick_params(colors='gray', labelsize=7)
        for pane in [self.ax_main.xaxis.pane, self.ax_main.yaxis.pane, self.ax_main.zaxis.pane]:
            pane.fill = False
            pane.set_edgecolor('#333333')

        self.trail, = self.ax_main.plot([], [], [], color="#00aaff", lw=1.5, alpha=0.5)
        self.arm_x, = self.ax_main.plot([], [], [], color="#ff6b6b", lw=2.5)
        self.arm_y, = self.ax_main.plot([], [], [], color="#ffd93d", lw=2.5)
        self.center, = self.ax_main.plot([], [], [], 'o', color="#00ffcc", ms=5)
        self.target_marker, = self.ax_main.plot([], [], [], '*',
                                                color="#ff9900", ms=10, zorder=5)
        # ── CLOSE-UP VIEW ──
        self.fig_close = Figure(facecolor="#0d0d0d")
        self.canvas_close = FigureCanvas(self.fig_close)
        close_layout = QVBoxLayout(self.graph_closeup)
        close_layout.addWidget(self.canvas_close)
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
        self.cl_arm_x, = self.ax_close.plot([], [], [], color="#ff6b6b", lw=2.5)
        self.cl_arm_y, = self.ax_close.plot([], [], [], color="#ffd93d", lw=2.5)
        self.cl_center, = self.ax_close.plot([], [], [], 'o', color="#00ffcc", ms=6)


    def on_simulate(self):
        proc = getattr(self, 'matlab_process', None)
        if proc is not None:
            if proc.poll() is None:
                proc.kill()
                proc.wait()

        self.matlab_process = subprocess.Popen([
            r"C:\Program Files\MATLAB\R2025b\bin\matlab.exe",
            "-batch", "main"
        ])

        self.running = True
        self.btn_launchsimulation.setText("⏳  Running...")
        self.btn_launchsimulation.setStyleSheet("""
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
            # 1. Get text and remove any accidental spaces (strip)
            str_x = self.input_x.text().strip()
            str_y = self.input_y.text().strip()
            str_z = self.input_z.text().strip()
            str_psi = self.input_psi.text().strip()

            # 2. Check if any box is empty
            if not all([str_x, str_y, str_z, str_psi]):
                self.status_label.setText("● Error: All fields must be filled!")
                return

            # 3. Convert to float
            x, y, z, psi = float(str_x), float(str_y), float(str_z), float(str_psi)

            # 4. Pack and Send
            data = struct.pack('<4f', x, y, z, np.radians(psi))
            self.sock_out.sendto(data, ("127.0.0.1", 5006))

            # 5. Update UI
            self.current_target = [x, y, z]
            self.status_label.setText(f"● Target sent → x:{x}  y:{y}  z:{z}  ψ:{psi}°")
            self.status_label.setStyleSheet("color: #ff9900;") # Make it orange for success

        except ValueError:
            self.status_label.setText("● Invalid input — numbers only (e.g. 5 or 5.0)")
        except AttributeError as e:
            print(f"DEBUG ERROR: {e}")
            self.status_label.setText("● UI Name Error - Check Terminal")
    def send_gains(self):
        if not self.running:
            return
        try:

            axis_id = float(self.combo_axis.currentText())
            kp = float(self.slider_kp.value() /100)
            ki = float(self.slider_ki.value() /100)
            kd = float(self.slider_kd.value() /100)

            self.label_kp_val.setText(f"{kp:.2f}")
            self.label_ki_val.setText(f"{ki:.2f}")
            self.label_kd_val.setText(f"{kd:.2f}")

            header = 999.0
            gain_data = struct.pack('<5f',header, axis_id, kp, ki, kd)
            self.sock_out.sendto(gain_data, ("127.0.0.1", 5007))
        except Exception as e:
            print(f"UDP Error: {e}")

    def reset_simulation(self):
        # 1. STOP THE BACKGROUND PROCESS
        # We check if the attribute exists AND if it isn't None
        if hasattr(self, 'matlab_process') and self.matlab_process is not None:
            try:
                # Check if process is still actually running
                if self.matlab_process.poll() is None:
                    self.matlab_process.kill()
                    self.matlab_process.wait(timeout=2)  # Wait up to 2s for it to die
            except Exception as e:
                print(f"Process termination error: {e}")
            finally:
                # Crucial: Reset to None so the next check passes safely
                self.matlab_process = None

        # 2. STOP THE GUI UPDATES
        self.timer.stop()
        self.running = False

        # 3. CLEAR DATA BUFFERS
        self.xs.clear()
        self.ys.clear()
        self.zs.clear()
        self.state = {"x": 0.0, "y": 0.0, "z": 0.0, "phi": 0.0, "theta": 0.0, "psi": 0.0}

        # 4. CLEAR THE VISUALS
        self.trail.set_data_3d([], [], [])
        self.arm_x.set_data_3d([], [], [])
        self.arm_y.set_data_3d([], [], [])
        self.center.set_data_3d([], [], [])

        # Also clear the close-up view
        self.cl_arm_x.set_data_3d([], [], [])
        self.cl_arm_y.set_data_3d([], [], [])
        self.cl_center.set_data_3d([], [], [])

        self.canvas_main.draw()
        self.canvas_close.draw()

        # 5. UI FEEDBACK
        self.btn_launchsimulation.setText("Launch Simulation")
        self.btn_launchsimulation.setEnabled(True)
        self.btn_launchsimulation.setStyleSheet("")  # Reset to original style
        self.status_label.setText("● System Reset: Ready to launch.")
        self.status_label.setStyleSheet("color: #ffaa00;")
# ── RUN ───────────────────────────────────────────────────────────────────────
app = QtWidgets.QApplication(sys.argv)
window = DroneSimApp()
window.show()
sys.exit(app.exec_())
