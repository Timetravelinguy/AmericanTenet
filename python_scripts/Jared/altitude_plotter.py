import sys
import numpy as np
from collections import deque
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from PyQt5.QtCore import QTimer
import pyqtgraph as pg
from pymavlink import mavutil
from scipy.fft import fft

class DroneDashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PX4 Drone Testing Dashboard")
        self.resize(1400, 900)

        # MAVLink connection (adjust for your setup)
        self.mav = mavutil.mavlink_connection('udp:127.0.0.1:14445')

        # Central widget and layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        # Create plots in a grid layout
        grid = pg.GraphicsLayoutWidget()
        layout.addWidget(grid)

        # 1. Motor RPM vs Throttle PWM (Top-left)
        self.rpm_plot = grid.addPlot(title="Motor RPM vs Throttle PWM")
        self.rpm_plot.setLabel('left', 'RPM')
        self.rpm_plot.setLabel('bottom', 'Throttle PWM (us)')
        self.rpm_curve = self.rpm_plot.plot(pen='y')

        # 2. Battery Voltage/Current vs Throttle (Top-right)
        grid.nextRow()
        self.batt_plot = grid.addPlot(title="Battery vs Throttle")
        self.batt_plot.setLabel('left', 'Voltage (V) / Current (A)')
        self.batt_plot.setLabel('bottom', 'Throttle PWM (us)')
        self.volt_curve = self.batt_plot.plot(pen='g', name='Voltage')
        self.current_curve = self.batt_plot.plot(pen='r', name='Current')

        # 3. Vibration FFT (Bottom-left)
        self.fft_plot = grid.addPlot(title="Vibration FFT (Accelerometer X)")
        self.fft_plot.setLabel('left', 'Amplitude')
        self.fft_plot.setLabel('bottom', 'Frequency (Hz)')
        self.fft_curve = self.fft_plot.plot(pen='b')

        # 4. ESC Telemetry (Bottom-right)
        grid.nextRow()
        self.esc_plot = grid.addPlot(title="ESC Telemetry")
        self.esc_plot.setLabel('left', 'Temp (°C) / Current (A)')
        self.esc_plot.setLabel('bottom', 'Time (samples)')
        self.esc_temp_curve = self.esc_plot.plot(pen='m', name='Temp')
        self.esc_current_curve = self.esc_plot.plot(pen='c', name='Current')

        # Data buffers
        self.time_data = deque(maxlen=200)
        self.throttle_data = deque(maxlen=200)
        self.rpm_data = deque(maxlen=200)
        self.voltage_data = deque(maxlen=200)
        self.current_data = deque(maxlen=200)
        self.esc_temp_data = deque(maxlen=200)
        self.esc_current_data = deque(maxlen=200)
        self.accel_buffer = np.zeros(256)  # For FFT
        self.sample_count = 0

        # Update timer (50ms ~ 20Hz)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(50)

    def update_plots(self):
        # Process all waiting messages
        while True:
            msg = self.mav.recv_match(blocking=False)
            if not msg:
                break

            # 1. Motor RPM vs Throttle
            if msg.get_type() == 'RPM':
                self.rpm_data.append(msg.rpm1)
                # Get corresponding throttle (PWM or RC)
                throttle_msg = self.mav.recv_match(type='ACTUATOR_OUTPUT_STATUS', blocking=False) or \
                             self.mav.recv_match(type='RC_CHANNELS', blocking=False)
                if throttle_msg:
                    if throttle_msg.get_type() == 'ACTUATOR_OUTPUT_STATUS':
                        self.throttle_data.append(throttle_msg.output[0])
                    else:  # RC_CHANNELS
                        self.throttle_data.append(throttle_msg.chan3_raw)

            # 2. Battery vs Throttle
            elif msg.get_type() == 'BATTERY_STATUS':
                self.voltage_data.append(msg.voltages[0] / 1000.0)  # mV → V
                self.current_data.append(msg.current_battery / 100.0)  # cA → A

            # 3. Vibration FFT
            elif msg.get_type() == 'HIGHRES_IMU':
                self.accel_buffer[:-1] = self.accel_buffer[1:]
                self.accel_buffer[-1] = msg.xacc  # X-axis acceleration

            # 4. ESC Telemetry
            elif msg.get_type() == 'ESC_TELEMETRY_1_TO_4':
                self.esc_temp_data.append(msg.temperature[0])
                self.esc_current_data.append(msg.current[0] / 100.0)  # cA → A

        # Update time base
        self.sample_count += 1
        self.time_data.append(self.sample_count)

        # Update all plots
        if self.throttle_data and self.rpm_data:
            self.rpm_curve.setData(self.throttle_data, self.rpm_data)
        
        if self.throttle_data and self.voltage_data:
            self.volt_curve.setData(self.throttle_data, self.voltage_data)
            self.current_curve.setData(self.throttle_data, self.current_data)
        
        if len(self.accel_buffer) == 256:
            yf = fft(self.accel_buffer)
            xf = np.linspace(0, 50, 128)  # 50 Hz Nyquist
            self.fft_curve.setData(xf, 2/256 * np.abs(yf[:128]))
        
        if self.esc_temp_data:
            self.esc_temp_curve.setData(self.time_data[-len(self.esc_temp_data):], self.esc_temp_data)
            self.esc_current_curve.setData(self.time_data[-len(self.esc_current_data):], self.esc_current_data)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = DroneDashboard()
    window.show()
    sys.exit(app.exec_())