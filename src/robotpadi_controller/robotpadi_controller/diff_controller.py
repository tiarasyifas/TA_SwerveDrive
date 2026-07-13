#!/usr/bin/env python3
"""
Differential Drive Controller + GUI
Publishes wheel velocities only (no steering commands).
"""

import threading
import tkinter as tk

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


# Robot geometry (metres)
TRACK_WIDTH = 0.950
HALF_TRACK = TRACK_WIDTH / 2.0
WHEEL_RADIUS = 0.2

# Limits
MAX_LINEAR = 2.0   # m/s
MAX_OMEGA = 3.0    # rad/s

# Colors (match swerve controller look)
DARK = "#f0f4f8"
MID = "#dce6f0"
ACCENT = "#3a7bd5"
CYAN = "#1a5fb4"
GREEN = "#1e7e34"
RED = "#c0392b"
WHITE = "#1c1c2e"
GRAY = "#5a6a7a"


def diff_kinematics(linear: float, omega: float):
    """Convert body linear/angular velocity to left/right wheel angular rates."""
    left_ms = linear - (omega * HALF_TRACK)
    right_ms = linear + (omega * HALF_TRACK)

    left_rad_s = left_ms / WHEEL_RADIUS
    right_rad_s = right_ms / WHEEL_RADIUS
    return left_rad_s, right_rad_s


class DiffController(Node):
    def __init__(self):
        super().__init__('diff_controller')

        self.wheel_pub = self.create_publisher(
            Float64MultiArray, '/wheel_controller/commands', 10)

        self.linear = 0.0
        self.omega = 0.0

        self.create_timer(0.05, self._publish)  # 20 Hz
        self.get_logger().info('Differential controller ready')

    def _publish(self):
        left_rad_s, right_rad_s = diff_kinematics(self.linear, self.omega)

        # Keep motor orientation consistent with existing wheel sign convention.
        wheel_signs = [-1, -1, 1, 1]
        wheel_rates = [left_rad_s, left_rad_s, right_rad_s, right_rad_s]

        msg = Float64MultiArray()
        msg.data = [wheel_signs[i] * wheel_rates[i] for i in range(4)]
        self.wheel_pub.publish(msg)


class DiffDiagram(tk.Canvas):
    """Simple left/right wheel speed diagram for differential drive."""

    W, H = 220, 200

    def __init__(self, parent, **kwargs):
        super().__init__(parent, width=self.W, height=self.H,
                         bg=MID, highlightthickness=0, **kwargs)
        self._left = 0.0
        self._right = 0.0
        self._draw()

    def update_wheels(self, left_rad_s: float, right_rad_s: float):
        self._left = left_rad_s
        self._right = right_rad_s
        self._draw()

    def _draw(self):
        self.delete('all')
        self.create_rectangle(50, 30, 170, 170, outline=GRAY, width=1, dash=(4, 3))
        self.create_line(110, 120, 110, 70, fill=CYAN, width=2, arrow=tk.LAST)
        self.create_text(110, 58, text='FRONT', fill=CYAN, font=('Courier', 8))

        max_spd = max(abs(self._left), abs(self._right), 0.1)

        def wheel_color(speed):
            intensity = min(int(abs(speed) / max_spd * 180 + 60), 240)
            if speed >= 0:
                return f"#{0:02x}{intensity // 2:02x}{intensity:02x}"
            return f"#{intensity:02x}{0:02x}{0:02x}"

        left_h = min(50, int(abs(self._left) / max_spd * 50))
        right_h = min(50, int(abs(self._right) / max_spd * 50))

        self.create_rectangle(62, 100 - left_h, 84, 100 + left_h,
                              fill=wheel_color(self._left), outline='')
        self.create_rectangle(136, 100 - right_h, 158, 100 + right_h,
                              fill=wheel_color(self._right), outline='')

        self.create_text(73, 155, text='LEFT', fill=WHITE, font=('Courier', 8, 'bold'))
        self.create_text(147, 155, text='RIGHT', fill=WHITE, font=('Courier', 8, 'bold'))
        self.create_text(73, 172, text=f'{self._left:.1f}r/s', fill=GRAY, font=('Courier', 7))
        self.create_text(147, 172, text=f'{self._right:.1f}r/s', fill=GRAY, font=('Courier', 7))


class DiffGUI:
    def __init__(self, node: DiffController):
        self.node = node
        self.root = tk.Tk()
        self.root.title('Differential Drive Controller')
        self.root.geometry('480x660')
        self.root.resizable(False, False)
        self.root.configure(bg=DARK)

        self._held = set()

        self._build()
        self.root.bind('<KeyPress>', self._key_press)
        self.root.bind('<KeyRelease>', self._key_release)
        self.root.focus_set()
        self._key_timer()

    def _build(self):
        bold = ('Courier', 10, 'bold')
        norm = ('Courier', 9)

        tk.Label(self.root, text='DIFFERENTIAL DRIVE', bg=DARK, fg=CYAN,
                 font=('Courier', 18, 'bold')).pack(pady=(14, 2))
        tk.Label(self.root, text='left/right wheel differential kinematics', bg=DARK, fg=GRAY,
                 font=('Courier', 8)).pack()

        self.diagram = DiffDiagram(self.root)
        self.diagram.pack(pady=10)

        sliders_frame = tk.Frame(self.root, bg=DARK)
        sliders_frame.pack(padx=20, fill=tk.X)

        self.linear_var = tk.DoubleVar(value=0.0)
        self.omega_var = tk.DoubleVar(value=0.0)

        def make_slider(parent, label, var, lo, hi, unit):
            row = tk.Frame(parent, bg=DARK)
            row.pack(fill=tk.X, pady=6)

            tk.Label(row, text=label, bg=DARK, fg=WHITE, font=bold,
                     width=12, anchor='w').pack(side=tk.LEFT)

            track_w, track_h, thumb_r = 220, 6, 9
            canvas_w, canvas_h = track_w + thumb_r * 2 + 4, thumb_r * 2 + 10
            canvas = tk.Canvas(row, width=canvas_w, height=canvas_h,
                               bg=DARK, highlightthickness=0, cursor='hand2')
            canvas.pack(side=tk.LEFT, padx=4)

            tx0 = thumb_r + 2
            tx1 = tx0 + track_w
            ty = canvas_h // 2

            def val_to_x(value):
                return tx0 + (value - lo) / (hi - lo) * track_w

            def x_to_val(x_pos):
                raw = lo + (x_pos - tx0) / track_w * (hi - lo)
                snapped = round(raw / 0.05) * 0.05
                return max(lo, min(hi, snapped))

            def redraw(*_):
                value = var.get()
                cx = val_to_x(value)
                canvas.delete('all')

                canvas.create_rectangle(tx0, ty - track_h // 2,
                                        tx1, ty + track_h // 2 + 1,
                                        fill='#c8d8e8', outline='')

                mid = val_to_x(0.0)
                x0f, x1f = min(mid, cx), max(mid, cx)
                if x1f > x0f:
                    canvas.create_rectangle(x0f, ty - track_h // 2,
                                            x1f, ty + track_h // 2 + 1,
                                            fill=ACCENT, outline='')

                canvas.create_oval(cx - thumb_r, ty - thumb_r,
                                   cx + thumb_r, ty + thumb_r,
                                   fill=ACCENT, outline='#ffffff', width=2)

            def click(event):
                var.set(x_to_val(event.x))
                self._on_slider()

            def drag(event):
                var.set(x_to_val(event.x))
                self._on_slider()

            def wheel(event):
                delta = 0.05 if (event.delta > 0 or event.num == 4) else -0.05
                value = round((var.get() + delta) / 0.05) * 0.05
                var.set(max(lo, min(hi, value)))
                self._on_slider()

            var.trace_add('write', redraw)
            redraw()

            canvas.bind('<Button-1>', click)
            canvas.bind('<B1-Motion>', drag)
            canvas.bind('<MouseWheel>', wheel)
            canvas.bind('<Button-4>', wheel)
            canvas.bind('<Button-5>', wheel)

            value_label = tk.Label(row, text=f'0.00 {unit}', bg=DARK, fg=CYAN,
                                   font=norm, width=9, anchor='e')
            value_label.pack(side=tk.LEFT, padx=6)
            return value_label

        self._linear_lbl = make_slider(
            sliders_frame, 'Linear v', self.linear_var, -MAX_LINEAR, MAX_LINEAR, 'm/s')
        self._omega_lbl = make_slider(
            sliders_frame, 'Yaw ω', self.omega_var, -MAX_OMEGA, MAX_OMEGA, 'r/s')

        hint = tk.Frame(self.root, bg=MID, bd=0)
        hint.pack(padx=20, pady=8, fill=tk.X)

        keys = [
            ('W/S', 'Forward / Back'),
            ('A/D', 'Rotate CCW / CW'),
            ('Space', 'Stop all'),
        ]
        for key, text in keys:
            row = tk.Frame(hint, bg=MID)
            row.pack(anchor='w', padx=8, pady=1)
            tk.Label(row, text=f'[{key}]', bg=MID, fg=CYAN,
                     font=('Courier', 9, 'bold'), width=8, anchor='w').pack(side=tk.LEFT)
            tk.Label(row, text=text, bg=MID, fg=GRAY,
                     font=('Courier', 9)).pack(side=tk.LEFT)

        self.status_lbl = tk.Label(self.root, text='● STOPPED',
                                   bg=DARK, fg=RED,
                                   font=('Courier', 13, 'bold'))
        self.status_lbl.pack(pady=(10, 6))

        button_row = tk.Frame(self.root, bg=DARK)
        button_row.pack()

        tk.Button(button_row, text='  STOP  ', command=self._stop,
                  bg=RED, fg='#ffffff', font=('Courier', 11, 'bold'),
                  relief=tk.FLAT, padx=10, pady=6).pack(side=tk.LEFT, padx=8)
        tk.Button(button_row, text='  QUIT  ', command=self._quit,
                  bg=ACCENT, fg='#ffffff', font=('Courier', 10),
                  relief=tk.FLAT, padx=10, pady=6).pack(side=tk.LEFT, padx=8)

        self._on_slider()

    def _on_slider(self):
        self.node.linear = self.linear_var.get()
        self.node.omega = self.omega_var.get()

        self._linear_lbl.config(text=f'{self.node.linear:+.2f} m/s')
        self._omega_lbl.config(text=f'{self.node.omega:+.2f} r/s')

        left, right = diff_kinematics(self.node.linear, self.node.omega)
        self.diagram.update_wheels(left, right)
        self._refresh_status()

    def _refresh_status(self):
        linear = self.node.linear
        omega = self.node.omega

        if abs(linear) < 0.02 and abs(omega) < 0.02:
            self.status_lbl.config(text='● STOPPED', fg=RED)
            return

        parts = []
        if linear > 0.02:
            parts.append('FWD')
        if linear < -0.02:
            parts.append('REV')
        if omega > 0.02:
            parts.append('↺ CCW')
        if omega < -0.02:
            parts.append('↻ CW')

        self.status_lbl.config(text='● ' + '  '.join(parts), fg=GREEN)

    _KEY_MAP = {
        'w': ('linear', 0.1),
        's': ('linear', -0.1),
        'a': ('omega', 0.15),
        'd': ('omega', -0.15),
    }

    def _key_press(self, event):
        key = event.keysym.lower()
        if key == 'space':
            self._stop()
            return
        self._held.add(key)

    def _key_release(self, event):
        self._held.discard(event.keysym.lower())

    def _key_timer(self):
        for key, (axis, delta) in self._KEY_MAP.items():
            if key not in self._held:
                continue

            if axis == 'linear':
                value = max(-MAX_LINEAR, min(MAX_LINEAR, self.linear_var.get() + delta))
                self.linear_var.set(value)
            else:
                value = max(-MAX_OMEGA, min(MAX_OMEGA, self.omega_var.get() + delta))
                self.omega_var.set(value)

            self._on_slider()

        self.root.after(50, self._key_timer)

    def _stop(self):
        self.linear_var.set(0.0)
        self.omega_var.set(0.0)
        self._on_slider()

    def _quit(self):
        self._stop()
        self.root.quit()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def _spin_ros(executor):
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.01)
    except Exception as exc:
        print(f'ROS error: {exc}')


def main():
    rclpy.init()
    node = DiffController()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    threading.Thread(target=_spin_ros, args=(executor,), daemon=True).start()

    print('Differential Controller started')
    print('  W/S  : forward / back')
    print('  A/D  : rotate CCW / CW')
    print('  Space: stop')

    gui = DiffGUI(node)
    try:
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        print('Shutdown complete.')


if __name__ == '__main__':
    main()
