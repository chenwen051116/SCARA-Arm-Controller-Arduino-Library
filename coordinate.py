import sys, re, serial, serial.tools.list_ports
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QPushButton, QLabel,
    QVBoxLayout, QHBoxLayout, QGridLayout, QComboBox, QLineEdit,
    QPlainTextEdit, QMessageBox, QSlider, QDial
)
from PyQt5.QtGui import QPainter, QPen, QColor, QDoubleValidator, QIntValidator
from PyQt5.QtCore import Qt, QPointF, QTimer

# Only Joint Pos, Speed/Accel, Path XYZ+θ₃, Smooth Path
MODES = [
    ("Joint Pos",   1),
    ("Speed/Accel", 2),
    ("Path XYZ+θ₃", 7),
    ("Smooth Path", 9),
]

class CoordinateWidget(QWidget):
    def __init__(self, parent=None, scale=10):
        super().__init__(parent)
        self.scale = scale
        reach_cm = 22.8 + 13.65
        dpx = int(reach_cm * 2 * scale)
        self.setFixedSize(dpx, dpx)
        self.radius_px = reach_cm * scale
        self.path = []
        self.last = None

    def paintEvent(self, event):
        p = QPainter(self); p.setRenderHint(QPainter.Antialiasing)
        cx, cy = self.width()//2, self.height()//2
        p.setPen(QPen(Qt.black, 2))
        p.drawLine(cx,0, cx,self.height())
        p.drawLine(0,cy, self.width(),cy)
        p.drawEllipse(QPointF(cx,cy), self.radius_px, self.radius_px)
        if self.path:
            mode_idx = self.window().mode_combo.currentIndex()
            cmd = MODES[mode_idx][1]
            light = QColor(173,216,230)
            pts = []
            for i, pt in enumerate(self.path):
                x, y = pt[0], pt[1]
                px = cx + x*self.scale
                py = cy - y*self.scale
                color = light if (cmd==9 and i < len(self.path)-1) else Qt.blue
                p.setPen(QPen(color,6))
                p.drawPoint(QPointF(px,py))
                pts.append(QPointF(px,py))
            p.setPen(QPen(light if cmd==9 else Qt.blue,2))
            for i in range(1,len(pts)):
                p.drawLine(pts[i-1], pts[i])
        if self.last:
            p.setPen(QPen(Qt.red,6))
            p.drawPoint(self.last)

    def mousePressEvent(self, ev):
        w   = self.window()
        idx = w.mode_combo.currentIndex()
        cmd = MODES[idx][1]
        if cmd not in (7,9):
            return
        cx, cy = self.width()//2, self.height()//2
        px, py = ev.x(), ev.y()
        x_cm = (px - cx) / self.scale
        y_cm = (cy - py) / self.scale
        self.last = QPointF(px,py)
        w.x_input.setText(f"{int(round(x_cm*100))}")
        w.y_input.setText(f"{int(round(y_cm*100))}")
        w.z_input.setText(str(w.z_slider.value()))
        w.update_coord(x_cm, y_cm)
        self.update()

    def clear(self):
        self.path.clear()
        self.last = None
        self.update()

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("SCARA Arm Controller")

        # recording state
        self.recording = False
        self.recorded_cmds = []
        self.following = False
        self.follow_index = 0

        # Left: Z slider + input + θ₃ dial + input
        self.z_slider = QSlider(Qt.Vertical)
        self.z_slider.setRange(0,17000)
        self.z_slider.setValue(17000)
        self.z_slider.valueChanged.connect(self.on_slider)
        self.z_label = QLabel("Z = 17000", alignment=Qt.AlignCenter)
        self.z_input2 = QLineEdit(); self.z_input2.setFixedWidth(80)
        self.z_input2.setValidator(QIntValidator(0,17000,self))
        self.z_input2.setText(str(self.z_slider.value()))
        self.z_input2.textChanged.connect(self.on_z_input2)

        self.j3_dial = QDial()
        self.j3_dial.setRange(-180,180)
        self.j3_dial.setWrapping(True)
        self.j3_dial.setNotchesVisible(True)
        self.j3_dial.valueChanged.connect(self.on_dial)
        self.j3_label = QLabel("θ₃ = 0°", alignment=Qt.AlignCenter)
        self.j3_input = QLineEdit(); self.j3_input.setFixedWidth(60)
        self.j3_input.setValidator(QIntValidator(-180,180,self))
        self.j3_input.setText(str(self.j3_dial.value()))
        self.j3_input.textChanged.connect(self.on_j3_input)

        left_layout = QVBoxLayout()
        left_layout.addWidget(self.z_label)
        left_layout.addWidget(self.z_slider)
        left_layout.addWidget(self.z_input2)
        left_layout.addSpacing(20)
        left_layout.addWidget(self.j3_label)
        left_layout.addWidget(self.j3_dial)
        left_layout.addWidget(self.j3_input)
        self.left_widget = QWidget(); self.left_widget.setLayout(left_layout)

        # Center: coord map
        self.graph = CoordinateWidget(self)

        # Controls panel
        self.coord_lbl = QLabel("x=0.00 cm, y=0.00 cm")
        self.x_input   = QLineEdit(); self.x_input.setFixedWidth(320)
        self.y_input   = QLineEdit(); self.y_input.setFixedWidth(320)
        self.z_input   = QLineEdit(); self.z_input.setFixedWidth(160)
        dv = QDoubleValidator(0,10000,2,self); iv = QIntValidator(0,17000,self)
        self.x_input.setValidator(dv); self.y_input.setValidator(dv); self.z_input.setValidator(iv)
        self.x_input.textChanged.connect(self.on_xy)
        self.y_input.textChanged.connect(self.on_xy)
        self.z_input.textChanged.connect(self.on_z)

        self.btn_confirm = QPushButton("Confirm"); self.btn_confirm.setFixedHeight(80)
        self.btn_confirm.clicked.connect(self.confirm_point)

        self.mode_combo = QComboBox()
        for name,_ in MODES: self.mode_combo.addItem(name)
        self.mode_combo.currentIndexChanged.connect(self.on_mode)

        self.j1 = QLineEdit(placeholderText="θ₁"); self.j1.setFixedWidth(60)
        self.j2 = QLineEdit(placeholderText="θ₂"); self.j2.setFixedWidth(60)
        self.speed = QLineEdit(placeholderText="speed"); self.speed.setFixedWidth(60)
        self.accel = QLineEdit(placeholderText="accel"); self.accel.setFixedWidth(60)

        self.serial_combo = QComboBox(); self.refresh_ports()
        self.btn_connect  = QPushButton("Connect"); self.btn_connect.setFixedHeight(80)
        self.btn_connect.clicked.connect(self.toggle)
        self.btn_clear    = QPushButton("Clear");   self.btn_clear.setFixedHeight(80)
        self.btn_clear.clicked.connect(self.graph.clear)
        self.btn_run      = QPushButton("Run");     self.btn_run.setFixedHeight(80)
        self.btn_run.clicked.connect(self.send)

        # new buttons
        self.btn_record  = QPushButton("Record");  self.btn_record.setFixedHeight(80)
        self.btn_record.clicked.connect(self.toggle_record)
        self.btn_follow  = QPushButton("Follow");  self.btn_follow.setFixedHeight(80)
        self.btn_follow.clicked.connect(self.follow_sequence)
        self.btn_follow.setEnabled(False)

        # export/print sequence button
        self.btn_export = QPushButton("Export"); self.btn_export.setFixedHeight(80)
        self.btn_export.clicked.connect(self.export_sequence)
        self.btn_export.setEnabled(False)

        self.log = QPlainTextEdit(readOnly=True)

        controls = [
            self.coord_lbl,
            self.x_input, self.y_input, self.z_input, self.btn_confirm,
            self.mode_combo,
            self.j1, self.j2,
            self.speed, self.accel,
            self.serial_combo, self.btn_connect,
            self.btn_clear, self.btn_run,
            self.btn_record, self.btn_follow,
            self.btn_export
        ]
        control_panel = QWidget()
        grid = QGridLayout(control_panel); grid.setSpacing(10)
        for i, w in enumerate(controls):
            r, c = divmod(i,2)
            grid.addWidget(w, r, c)

        char_panel = QWidget()
        char_layout = QGridLayout(char_panel)
        chars = [chr(c) for c in range(ord('A'), ord('L'))]
        for idx, ch in enumerate(chars):
            btn = QPushButton(ch); btn.setFixedHeight(40)
            btn.clicked.connect(lambda _, ch=ch: self.send_char(ch))
            r, c = divmod(idx, 6); char_layout.addWidget(btn, r, c)

        right_layout = QVBoxLayout()
        right_layout.addWidget(control_panel)
        right_layout.addWidget(self.log)
        right_layout.addWidget(char_panel)
        right = QWidget(); right.setLayout(right_layout)

        container = QWidget()
        main_h = QHBoxLayout(container)
        main_h.addWidget(self.left_widget, stretch=0)
        main_h.addWidget(self.graph,       stretch=1)
        main_h.addWidget(right,            stretch=0)
        self.setCentralWidget(container)

        self.ser = None; self.selected = (0,0)
        self.on_mode(0)

        # poll serial
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.read_serial)
        self.timer.start(100)

    def refresh_ports(self):
        self.serial_combo.clear()
        for p in serial.tools.list_ports.comports():
            self.serial_combo.addItem(p.device)

    # Record / End
    def toggle_record(self):
        if not self.recording:
            self.recording = True
            self.recorded_cmds.clear()
            self.btn_record.setText("End")
            self.btn_follow.setEnabled(False)
            self.btn_export.setEnabled(False)
            self.log.appendPlainText(">> Recording started")
        else:
            self.recording = False
            self.btn_record.setText("Record")
            self.btn_follow.setEnabled(bool(self.recorded_cmds))
            self.btn_export.setEnabled(bool(self.recorded_cmds))
            self.log.appendPlainText(">> Recording ended")

    # Export sequence
    def export_sequence(self):
        if not self.recorded_cmds:
            return QMessageBox.warning(self, "Warning", "No recorded commands to export")
        # clear the log and print joined commands
        self.log.clear()
        seq = ";".join(self.recorded_cmds)
        self.log.appendPlainText(seq)

    # Follow sequence
    def follow_sequence(self):
        if not self.recorded_cmds:
            return QMessageBox.warning(self, "Warning", "No recorded commands")
        self.following = True
        self.follow_index = 0
        self.log.appendPlainText(">> Following sequence")
        self._send_cmd(self.recorded_cmds[0])

    def on_slider(self, v):
        self.z_label.setText(f"Z = {v}")
        self.z_input2.setText(str(v))
        if self.z_input.hasAcceptableInput():
            self.z_input.setText(str(v))

    def on_z_input2(self, _):
        if self.z_input2.hasAcceptableInput():
            self.z_slider.setValue(int(self.z_input2.text()))

    def on_dial(self, v):
        self.j3_label.setText(f"θ₃ = {v}°")
        self.j3_input.setText(str(v))

    def on_j3_input(self, _):
        if self.j3_input.hasAcceptableInput():
            self.j3_dial.setValue(int(self.j3_input.text()))

    def on_xy(self, _):
        try:
            xm, ym = float(self.x_input.text()), float(self.y_input.text())
        except ValueError:
            return
        self.update_coord(xm/100, ym/100)

    def on_z(self, _):
        if self.z_input.hasAcceptableInput():
            self.z_slider.setValue(int(self.z_input.text()))

    def update_coord(self, x, y):
        self.selected = (x, y)
        self.coord_lbl.setText(f"x={x:.2f} cm, y={y:.2f} cm")
        cx, cy = self.graph.width()//2, self.graph.height()//2
        px, py = cx + x*self.graph.scale, cy - y*self.graph.scale
        self.graph.last = QPointF(px, py)
        self.graph.update()

    def confirm_point(self):
        idx = self.mode_combo.currentIndex(); cmd = MODES[idx][1]
        if cmd in (7, 9):
            x, y = self.selected
            z    = self.z_slider.value()
            a3   = self.j3_dial.value()
            self.graph.path.append((x, y, z, a3))
            self.graph.update()

    def on_mode(self, idx):
        cmd = MODES[idx][1]
        self.graph.clear()
        for w in [self.x_input, self.y_input, self.z_input, self.btn_confirm,
                  self.j1, self.j2, self.speed, self.accel]:
            w.setVisible(False)
        self.left_widget.setVisible(False)
        if cmd == 1:
            self.j1.setVisible(True); self.j2.setVisible(True)
            self.left_widget.setVisible(True)
        elif cmd == 2:
            self.speed.setVisible(True); self.accel.setVisible(True)
        else:
            self.x_input.setVisible(True); self.y_input.setVisible(True)
            self.z_input.setVisible(True); self.btn_confirm.setVisible(True)
            self.left_widget.setVisible(True)

    def toggle(self):
        if self.ser and self.ser.is_open:
            self.ser.close(); self.ser = None
            self.btn_connect.setText("Connect")
        else:
            port = self.serial_combo.currentText()
            try:
                self.ser = serial.Serial(port, 9600, timeout=1)
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Cannot open {port}:\n{e}")
                return
            self.btn_connect.setText("Disconnect")

    def send(self):
        idx = self.mode_combo.currentIndex(); cmd = MODES[idx][1]
        parts = [str(cmd)]
        if cmd == 1:
            parts += [self.j1.text(), self.j2.text(),
                      str(self.j3_dial.value()), str(self.z_slider.value())]
        elif cmd == 2:
            parts += [self.speed.text(), self.accel.text()]
        else:
            pts = self.graph.path
            if not pts:
                return QMessageBox.warning(self, "Warning", "No points recorded")
            parts.append(str(len(pts)))
            for x,y,z,a3 in pts:
                parts += [str(int(round(x*100))),
                          str(int(round(y*100))),
                          str(z), str(a3)]
        line = ",".join(parts)
        self.log.appendPlainText(line)
        self._send_cmd(line)
        if self.recording:
            self.recorded_cmds.append(line)

    def _send_cmd(self, line):
        if self.ser and self.ser.is_open:
            self.ser.write((line + "\n").encode())

    def send_char(self, ch):
        line = f"10,{ch}"
        self.log.appendPlainText(line)
        self._send_cmd(line)
        if self.recording:
            self.recorded_cmds.append(line)

    def read_serial(self):
        if self.ser and self.ser.is_open and self.ser.in_waiting:
            try:
                raw = self.ser.readline()
                line = raw.decode(errors='ignore').strip()
                if line:
                    self.log.appendPlainText(f"< {line}")
                    if self.following and line.lower() == "done":
                        self.follow_index += 1
                        if self.follow_index < len(self.recorded_cmds):
                            next_cmd = self.recorded_cmds[self.follow_index]
                            self.log.appendPlainText(next_cmd)
                            self._send_cmd(next_cmd)
                        else:
                            self.following = False
                            QMessageBox.information(self, "Follow", "Sequence complete")
            except Exception:
                pass

if __name__ == "__main__":
    app = QApplication(sys.argv)
    font = app.font(); font.setPointSize(16); app.setFont(font)
    win = MainWindow()
    win.resize(1000,700)
    win.show()
    sys.exit(app.exec_())
