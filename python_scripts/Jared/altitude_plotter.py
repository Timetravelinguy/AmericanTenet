import sys
import numpy as np
from collections import deque
import time
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QGridLayout, QLabel, QStatusBar, QComboBox, QHBoxLayout
from PyQt5.QtCore import QTimer, pyqtSignal, QObject
import pyqtgraph as pg
from pymavlink import mavutil
from scipy.fft import fft, fftfreq
from wakeup import wakeup

class MAVLinkDataProcessor(QObject):
    """Handles MAVLink connection and data processing"""
    
    # Signals for data updates
    rpm_updated = pyqtSignal(float, float)  # rpm, throttle
    battery_updated = pyqtSignal(float, float, float)  # voltage, current, throttle
    imu_updated = pyqtSignal(float)  # acceleration
    esc_updated = pyqtSignal(float, float)  # temperature, current
    
    def __init__(self, connection_string='udpin:0.0.0.0:14550'):
        super().__init__()
        self.connection_string = connection_string
        self.mav = None
        self.last_throttle = 1000  # Default PWM value
        self.heartbeat_sent = False
        self.connect()
    
    def connect(self):
        """Establish MAVLink connection"""
        try:
            self.mav = mavutil.mavlink_connection(self.connection_string)
            print(f"Connected to MAVLink on {self.connection_string}")
            
            # Send heartbeat to wake up PX4 (similar to your wakeup function)
            if not self.heartbeat_sent:
                self.mav.mav.heartbeat_send(0, 0, 0, 0, 0, 0)
                print("Heartbeat sent to wake up PX4")
                self.heartbeat_sent = True
                
        except Exception as e:
            print(f"Failed to connect to MAVLink: {e}")
    
    def process_messages(self):
        """Process all waiting MAVLink messages"""
        if not self.mav:
            return
        
        try:
            # Request data streams if needed (do this periodically)
            if hasattr(self, 'last_request_time'):
                if time.time() - self.last_request_time > 5:  # Request every 5 seconds
                    self.request_data_streams()
            else:
                self.request_data_streams()
                self.last_request_time = time.time()
            
            # Process multiple messages per cycle for better throughput
            messages_processed = 0
            for _ in range(10):  # Process up to 10 messages per call
                msg = self.mav.recv_match(blocking=False)
                if not msg:
                    break
                
                messages_processed += 1
                self._handle_message(msg)
            
            # Debug: print if we're getting messages
            if messages_processed > 0:
                print(f"Processed {messages_processed} messages")
                
        except Exception as e:
            print(f"Error processing messages: {e}")
    
    def request_data_streams(self):
        """Request specific data streams from PX4"""
        if not self.mav:
            return
            
        try:
            # Request various data streams
            self.mav.mav.request_data_stream_send(
                self.mav.target_system,
                self.mav.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                4,  # 4 Hz
                1   # Enable
            )
            print("Requested data streams from PX4")
            
        except Exception as e:
            print(f"Error requesting data streams: {e}")
    
    def _handle_message(self, msg):
        """Handle individual MAVLink messages"""
        msg_type = msg.get_type()
        
        # Debug: print message types we're receiving (but filter out spam)
        if msg_type not in ['HEARTBEAT', 'SYSTEM_TIME', 'ATTITUDE', 'LOCAL_POSITION_NED']:
            print(f"Received message: {msg_type}")
        
        # === RPM DATA ===
        if msg_type == 'RPM':
            print(f"RPM: {msg.rpm1}, Throttle: {self.last_throttle}")
            self.rpm_updated.emit(msg.rpm1, self.last_throttle)
        
        # === THROTTLE/PWM DATA ===
        elif msg_type == 'ACTUATOR_OUTPUT_STATUS':
            self.last_throttle = msg.output[0]
            print(f"Actuator output: {msg.output[0]}")
            # Generate fake RPM based on throttle for demonstration
            fake_rpm = max(0, (msg.output[0] - 1000) * 10)  # Simple throttle to RPM conversion
            if fake_rpm > 0:
                self.rpm_updated.emit(fake_rpm, msg.output[0])
        
        elif msg_type == 'RC_CHANNELS':
            self.last_throttle = msg.chan3_raw
            print(f"RC Channel 3: {msg.chan3_raw}")
            # Generate fake RPM based on RC input
            fake_rpm = max(0, (msg.chan3_raw - 1000) * 8)
            if fake_rpm > 0:
                self.rpm_updated.emit(fake_rpm, msg.chan3_raw)
        
        elif msg_type == 'SERVO_OUTPUT_RAW':
            # Most common throttle source in PX4
            self.last_throttle = msg.servo3_raw
            print(f"Servo output CH3: {msg.servo3_raw}")
            # Generate simulated RPM data
            fake_rpm = max(0, (msg.servo3_raw - 1000) * 12)
            if fake_rpm > 100:  # Only show meaningful RPM values
                self.rpm_updated.emit(fake_rpm, msg.servo3_raw)
        
        # === BATTERY DATA ===
        elif msg_type == 'BATTERY_STATUS':
            voltage = msg.voltages[0] / 1000.0 if msg.voltages[0] > 0 else 0
            current = msg.current_battery / 100.0 if msg.current_battery != -1 else 0
            print(f"Battery - Voltage: {voltage}V, Current: {current}A, Throttle: {self.last_throttle}")
            if voltage > 0:
                self.battery_updated.emit(voltage, abs(current), self.last_throttle)
        
        elif msg_type == 'SYS_STATUS':
            # Alternative battery data source
            voltage = msg.voltage_battery / 1000.0 if msg.voltage_battery > 0 else 0
            current = msg.current_battery / 100.0 if msg.current_battery != -1 else 0
            print(f"Sys Status - Voltage: {voltage}V, Current: {current}A")
            if voltage > 0:
                self.battery_updated.emit(voltage, abs(current), self.last_throttle)
        
        # === IMU DATA ===
        elif msg_type == 'HIGHRES_IMU':
            print(f"IMU X-accel: {msg.xacc}")
            self.imu_updated.emit(msg.xacc)
        
        elif msg_type == 'SCALED_IMU2':
            accel_x = msg.xacc / 1000.0  # Convert from mg to m/s²
            self.imu_updated.emit(accel_x)
        
        elif msg_type == 'RAW_IMU':
            # Raw IMU data (more common)
            accel_x = msg.xacc / 1000.0  # Convert from mg to m/s²
            self.imu_updated.emit(accel_x)
        
        # === ESC DATA ===
        elif msg_type == 'ESC_TELEMETRY_1_TO_4':
            temp = msg.temperature[0] if msg.temperature[0] != 0 else 0
            current = msg.current[0] / 100.0 if msg.current[0] > 0 else 0
            print(f"ESC - Temp: {temp}°C, Current: {current}A")
            self.esc_updated.emit(temp, current)
        
        # === FALLBACK DATA FOR DEMONSTRATION ===
        elif msg_type == 'ATTITUDE':
            # Use attitude data as vibration proxy
            self.imu_updated.emit(msg.rollspeed * 10)
            
            # Generate fake ESC data based on attitude changes
            fake_temp = 20 + abs(msg.roll * 57.3) * 2  # Convert rad to degrees, scale for temp
            fake_current = abs(msg.pitch * 57.3) * 0.5  # Simulate current based on pitch
            if fake_temp > 20:
                self.esc_updated.emit(fake_temp, fake_current)
        
        elif msg_type == 'VFR_HUD':
            # Very common message with throttle info
            throttle_percent = msg.throttle
            throttle_pwm = 1000 + (throttle_percent * 10)  # Convert % to PWM
            self.last_throttle = throttle_pwm
            
            # Generate RPM from throttle percentage
            fake_rpm = throttle_percent * 100  # Simple conversion
            if fake_rpm > 0:
                print(f"VFR_HUD - Throttle: {throttle_percent}%, RPM: {fake_rpm}")
                self.rpm_updated.emit(fake_rpm, throttle_pwm)

class PlotWidget(pg.PlotWidget):
    """Enhanced plot widget with better styling"""
    
    def __init__(self, title, xlabel, ylabel, **kwargs):
        super().__init__(**kwargs)
        self.setTitle(title, size='12pt')
        self.setLabel('left', ylabel)
        self.setLabel('bottom', xlabel)
        self.showGrid(x=True, y=True, alpha=0.3)
        self.setBackground('w')
        
        # Add legend
        self.legend = self.addLegend()

class DroneDashboard(QMainWindow):
    def __init__(self, enable_test_data=False):
        super().__init__()
        self.setWindowTitle("PX4 Drone Testing Dashboard - Enhanced")
        self.resize(1600, 1200)  # Increased height for master plot
        
        # Test data mode
        self.enable_test_data = enable_test_data
        
        # Initialize data processor
        self.data_processor = MAVLinkDataProcessor()
        self.setup_data_connections()
        
        # Setup UI
        self.setup_ui()
        self.setup_data_buffers()
        
        # Update timer (20Hz for smooth updates)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_dashboard)
        self.timer.start(50)
        
        # Test data timer (if enabled)
        if self.enable_test_data:
            self.test_timer = QTimer()
            self.test_timer.timeout.connect(self.generate_test_data)
            self.test_timer.start(100)  # 10Hz test data
            print("Test data generation enabled")
        
        # Statistics
        self.update_count = 0
        self.start_time = time.time()
        self.test_time = 0
    
    def setup_data_connections(self):
        """Connect data processor signals to update methods"""
        self.data_processor.rpm_updated.connect(self.update_rpm_data)
        self.data_processor.battery_updated.connect(self.update_battery_data)
        self.data_processor.imu_updated.connect(self.update_imu_data)
        self.data_processor.esc_updated.connect(self.update_esc_data)
        
    def setup_ui(self):
        """Setup the user interface"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)
        
        # Status bar
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("Initializing...")
        
        # Create main layout with master plot on top
        main_layout = QVBoxLayout()
        
        # MASTER PLOT - spans full width at top
        self.master_plot = PlotWidget("Master Telemetry - All Data Over Time", "Time (samples)", "Values (Multiple Units)")
        self.master_plot.setMinimumHeight(300)  # Set minimum height for visibility
        main_layout.addWidget(self.master_plot)
        
        # DROPDOWN CONTROLS for plot selection
        controls_layout = QHBoxLayout()
        controls_layout.addWidget(QLabel("Graph Selection:"))
        
        # Define available plot types
        self.available_plots = {
            "RPM vs Throttle": self.create_rpm_plot,
            "Battery Performance": self.create_battery_plot, 
            "Vibration FFT": self.create_fft_plot,
            "ESC Telemetry": self.create_esc_plot,
            "Throttle Over Time": self.create_throttle_time_plot,
            "Battery Voltage Time": self.create_voltage_time_plot,
            "Battery Current Time": self.create_current_time_plot,
            "Acceleration Time": self.create_accel_time_plot,
            "ESC Temperature Time": self.create_esc_temp_time_plot,
            "Power Analysis": self.create_power_plot,
            "Efficiency Analysis": self.create_efficiency_plot,
            "RPM Time Series": self.create_rpm_time_plot
        }
        
        # Create dropdown menus for each quadrant
        self.dropdown_labels = ["Top-Left:", "Top-Right:", "Bottom-Left:", "Bottom-Right:"]
        self.dropdowns = []
        self.default_selections = ["RPM vs Throttle", "Battery Performance", "Vibration FFT", "ESC Telemetry"]
        
        for i, (label, default) in enumerate(zip(self.dropdown_labels, self.default_selections)):
            controls_layout.addWidget(QLabel(label))
            dropdown = QComboBox()
            dropdown.addItems(list(self.available_plots.keys()))
            dropdown.setCurrentText(default)
            dropdown.currentTextChanged.connect(lambda text, pos=i: self.change_plot(pos, text))
            self.dropdowns.append(dropdown)
            controls_layout.addWidget(dropdown)
        
        controls_layout.addStretch()  # Push controls to left
        main_layout.addLayout(controls_layout)
        
        # Create 2x2 grid for detailed plots below master plot
        self.plot_grid = QGridLayout()
        
        # Initialize plot widgets and curves storage
        self.plot_widgets = [None, None, None, None]  # Four quadrants
        self.plot_curves = {}  # Store curves for each plot type
        
        # Create initial plots based on default selections
        for i, plot_type in enumerate(self.default_selections):
            self.change_plot(i, plot_type)
        
        # Add grid to main layout
        plot_widget = QWidget()
        plot_widget.setLayout(self.plot_grid)
        main_layout.addWidget(plot_widget)
        
        # Set the main layout
        container = QWidget()
        container.setLayout(main_layout)
        layout.addWidget(container)
        
    def setup_data_buffers(self):
        """Initialize data storage buffers"""
        buffer_size = 300  # Increased buffer size for better visualization
        
        # Master plot data - time series for all parameters
        self.master_data = {
            "rpm": deque(maxlen=buffer_size),
            "voltage": deque(maxlen=buffer_size),
            "current": deque(maxlen=buffer_size),
            "throttle": deque(maxlen=buffer_size),
            "esc_temp": deque(maxlen=buffer_size),
            "esc_current": deque(maxlen=buffer_size),
            "accel_x": deque(maxlen=buffer_size),
        }
        
        # Master plot curves with different colors and scaling
        self.master_curves = {
            "rpm": self.master_plot.plot(pen=pg.mkPen('yellow', width=2), name='RPM (/100)'),
            "voltage": self.master_plot.plot(pen=pg.mkPen('green', width=2), name='Voltage (V)'),
            "current": self.master_plot.plot(pen=pg.mkPen('red', width=2), name='Current (A)'),
            "throttle": self.master_plot.plot(pen=pg.mkPen('blue', width=2), name='Throttle (/100)'),
            "esc_temp": self.master_plot.plot(pen=pg.mkPen('magenta', width=2), name='ESC Temp (°C)'),
            "esc_current": self.master_plot.plot(pen=pg.mkPen('cyan', width=2), name='ESC Current (A)'),
            "accel_x": self.master_plot.plot(pen=pg.mkPen('orange', width=2), name='Accel X (*10)'),
        }
        
        self.max_samples = buffer_size
        
        # Individual plot data buffers
        # RPM vs Throttle data
        self.rpm_throttle_data = deque(maxlen=buffer_size)
        self.rpm_values = deque(maxlen=buffer_size)
        
        # Battery data
        self.battery_throttle_data = deque(maxlen=buffer_size)
        self.voltage_values = deque(maxlen=buffer_size)
        self.current_values = deque(maxlen=buffer_size)
        
        # Time series data for individual plots
        self.throttle_time_data = deque(maxlen=buffer_size)
        self.voltage_time_data = deque(maxlen=buffer_size)
        self.current_time_data = deque(maxlen=buffer_size)
        self.accel_time_data = deque(maxlen=buffer_size)
        self.rpm_time_data = deque(maxlen=buffer_size)
        self.power_data = deque(maxlen=buffer_size)  # Voltage × Current
        self.efficiency_data = deque(maxlen=buffer_size)  # RPM per Watt
        self.efficiency_power = deque(maxlen=buffer_size)  # Power values for efficiency
        
        # FFT data
        self.fft_buffer_size = 512  # Power of 2 for efficient FFT
        self.accel_buffer = np.zeros(self.fft_buffer_size)
        self.fft_sample_rate = 100  # Assumed sample rate in Hz
        
        # ESC data
        self.time_data = deque(maxlen=buffer_size)
        self.esc_temp_data = deque(maxlen=buffer_size)
        self.esc_current_data = deque(maxlen=buffer_size)
        self.sample_count = 0
        
    def change_plot(self, position, plot_type):
        """Change the plot at the specified position to the selected type"""
        try:
            # Remove existing plot if any
            if self.plot_widgets[position] is not None:
                self.plot_grid.removeWidget(self.plot_widgets[position])
                self.plot_widgets[position].setParent(None)
                self.plot_widgets[position] = None
            
            # Create new plot
            if plot_type in self.available_plots:
                new_plot = self.available_plots[plot_type]()
                self.plot_widgets[position] = new_plot
                
                # Add to grid (convert position to row, col)
                row = position // 2
                col = position % 2
                self.plot_grid.addWidget(new_plot, row, col)
                
                print(f"Changed plot at position {position} to {plot_type}")
                
        except Exception as e:
            print(f"Error changing plot: {e}")
    
    def create_rpm_plot(self):
        """Create RPM vs Throttle plot"""
        plot = PlotWidget("Motor RPM vs Throttle PWM", "Throttle PWM (μs)", "RPM")
        curve = plot.plot(pen=pg.mkPen('y', width=2), name='Motor RPM')
        self.plot_curves['rpm_throttle'] = curve
        return plot
    
    def create_battery_plot(self):
        """Create Battery Performance plot"""
        plot = PlotWidget("Battery Performance vs Throttle", "Throttle PWM (μs)", "Voltage (V) / Current (A)")
        voltage_curve = plot.plot(pen=pg.mkPen('g', width=2), name='Voltage (V)')
        current_curve = plot.plot(pen=pg.mkPen('r', width=2), name='Current (A)')
        self.plot_curves['battery_voltage'] = voltage_curve
        self.plot_curves['battery_current'] = current_curve
        return plot
    
    def create_fft_plot(self):
        """Create Vibration FFT plot"""
        plot = PlotWidget("Vibration Analysis (X-axis Accelerometer)", "Frequency (Hz)", "Amplitude (m/s²)")
        curve = plot.plot(pen=pg.mkPen('b', width=2), name='FFT Magnitude')
        self.plot_curves['fft'] = curve
        return plot
    
    def create_esc_plot(self):
        """Create ESC Telemetry plot"""
        plot = PlotWidget("ESC Telemetry Over Time", "Time (samples)", "Temperature (°C) / Current (A)")
        temp_curve = plot.plot(pen=pg.mkPen('m', width=2), name='Temperature (°C)')
        current_curve = plot.plot(pen=pg.mkPen('c', width=2), name='Current (A)')
        self.plot_curves['esc_temp'] = temp_curve
        self.plot_curves['esc_current'] = current_curve
        return plot
    
    def create_throttle_time_plot(self):
        """Create Throttle over Time plot"""
        plot = PlotWidget("Throttle Over Time", "Time (samples)", "Throttle PWM (μs)")
        curve = plot.plot(pen=pg.mkPen('blue', width=2), name='Throttle PWM')
        self.plot_curves['throttle_time'] = curve
        return plot
    
    def create_voltage_time_plot(self):
        """Create Battery Voltage over Time plot"""
        plot = PlotWidget("Battery Voltage Over Time", "Time (samples)", "Voltage (V)")
        curve = plot.plot(pen=pg.mkPen('green', width=2), name='Battery Voltage')
        self.plot_curves['voltage_time'] = curve
        return plot
    
    def create_current_time_plot(self):
        """Create Battery Current over Time plot"""
        plot = PlotWidget("Battery Current Over Time", "Time (samples)", "Current (A)")
        curve = plot.plot(pen=pg.mkPen('red', width=2), name='Battery Current')
        self.plot_curves['current_time'] = curve
        return plot
    
    def create_accel_time_plot(self):
        """Create Acceleration over Time plot"""
        plot = PlotWidget("X-Axis Acceleration Over Time", "Time (samples)", "Acceleration (m/s²)")
        curve = plot.plot(pen=pg.mkPen('orange', width=2), name='X-Acceleration')
        self.plot_curves['accel_time'] = curve
        return plot
    
    def create_esc_temp_time_plot(self):
        """Create ESC Temperature over Time plot"""
        plot = PlotWidget("ESC Temperature Over Time", "Time (samples)", "Temperature (°C)")
        curve = plot.plot(pen=pg.mkPen('magenta', width=2), name='ESC Temperature')
        self.plot_curves['esc_temp_time'] = curve
        return plot
    
    def create_power_plot(self):
        """Create Power Analysis plot (Voltage × Current)"""
        plot = PlotWidget("Power Analysis Over Time", "Time (samples)", "Power (W)")
        curve = plot.plot(pen=pg.mkPen('purple', width=2), name='Power (V×I)')
        self.plot_curves['power'] = curve
        return plot
    
    def create_efficiency_plot(self):
        """Create Efficiency Analysis plot (RPM per Watt)"""
        plot = PlotWidget("Motor Efficiency (RPM per Watt)", "Power (W)", "RPM/Watt")
        curve = plot.plot(pen=pg.mkPen('brown', width=2), name='Efficiency', symbol='o', symbolSize=3)
        self.plot_curves['efficiency'] = curve
        return plot
    
    def create_rpm_time_plot(self):
        """Create RPM over Time plot"""
        plot = PlotWidget("RPM Over Time", "Time (samples)", "RPM")
        curve = plot.plot(pen=pg.mkPen('yellow', width=2), name='Motor RPM')
        self.plot_curves['rpm_time'] = curve
        return plot
    
    def update_rpm_data(self, rpm, throttle):
        """Update RPM vs throttle data"""
        if rpm > 0 and 800 <= throttle <= 2200:  # Reasonable PWM range
            # Individual plot data
            self.rpm_throttle_data.append(throttle)
            self.rpm_values.append(rpm)
            
            # Time series data
            self.throttle_time_data.append(throttle)
            self.rpm_time_data.append(rpm)
            
            # Master plot data (scaled for visibility)
            self.master_data["rpm"].append(rpm / 100)  # Scale RPM for better visualization
            self.master_data["throttle"].append((throttle - 1000) / 100)  # Scale throttle to 0-12 range
    
    def update_battery_data(self, voltage, current, throttle):
        """Update battery performance data"""
        if voltage > 0 and 800 <= throttle <= 2200:
            # Individual plot data
            self.battery_throttle_data.append(throttle)
            self.voltage_values.append(voltage)
            self.current_values.append(abs(current))  # Use absolute value for current
            
            # Time series data
            self.voltage_time_data.append(voltage)
            self.current_time_data.append(abs(current))
            
            # Power calculation (Voltage × Current)
            power = voltage * abs(current)
            self.power_data.append(power)
            
            # Efficiency calculation (RPM per Watt) if we have RPM data
            if len(self.rpm_time_data) > 0 and power > 0:
                latest_rpm = self.rpm_time_data[-1] if self.rpm_time_data else 0
                efficiency = latest_rpm / power if power > 0.1 else 0  # Avoid division by very small numbers
                self.efficiency_data.append(efficiency)
                self.efficiency_power.append(power)
            
            # Master plot data
            self.master_data["voltage"].append(voltage)
            self.master_data["current"].append(abs(current))
    
    def update_imu_data(self, accel_x):
        """Update IMU data for FFT analysis"""
        # Shift buffer and add new sample for FFT
        self.accel_buffer[:-1] = self.accel_buffer[1:]
        self.accel_buffer[-1] = accel_x
        
        # Time series data
        self.accel_time_data.append(accel_x)
        
        # Master plot data (scaled for visibility)
        self.master_data["accel_x"].append(accel_x * 10)  # Scale acceleration for visibility
    
    def update_esc_data(self, temperature, current):
        """Update ESC telemetry data"""
        self.sample_count += 1
        
        # Individual plot data
        self.time_data.append(self.sample_count)
        self.esc_temp_data.append(temperature)
        self.esc_current_data.append(abs(current))
        
        # Master plot data
        self.master_data["esc_temp"].append(temperature)
        self.master_data["esc_current"].append(abs(current))
    
    def generate_test_data(self):
        """Generate test data for demonstration purposes"""
        import math
        
        self.test_time += 0.1
        
        # Generate realistic test data
        base_throttle = 1200 + 400 * (math.sin(self.test_time * 0.5) + 1)  # 1200-2000 PWM
        base_rpm = max(0, (base_throttle - 1000) * 15 + np.random.normal(0, 50))
        
        # Battery data (voltage drops with current load)
        base_voltage = 14.8 - (base_throttle - 1000) * 0.002  # Voltage sag
        base_current = (base_throttle - 1000) * 0.02 + np.random.normal(0, 0.5)
        
        # ESC data
        esc_temp = 25 + (base_throttle - 1000) * 0.03 + np.random.normal(0, 2)
        esc_current = max(0, base_current * 0.25 + np.random.normal(0, 0.2))
        
        # IMU data (vibration increases with throttle)
        imu_accel = (base_throttle - 1000) * 0.01 * math.sin(self.test_time * 20) + np.random.normal(0, 0.5)
        
        # Emit test data
        self.data_processor.rpm_updated.emit(base_rpm, base_throttle)
        self.data_processor.battery_updated.emit(base_voltage, abs(base_current), base_throttle)
        self.data_processor.esc_updated.emit(esc_temp, esc_current)
        self.data_processor.imu_updated.emit(imu_accel)
    
    def update_dashboard(self):
        """Main update loop for dashboard"""
        # Process new MAVLink messages
        self.data_processor.process_messages()
        
        # Update plots
        self.update_plots()
        
        # Update status
        self.update_count += 1
        if self.update_count % 20 == 0:  # Update status every second
            elapsed = time.time() - self.start_time
            rate = self.update_count / elapsed if elapsed > 0 else 0
            data_points = len(self.rpm_values) + len(self.voltage_values) + len(self.esc_temp_data)
            status_msg = f"Update Rate: {rate:.1f} Hz | Data Points: {data_points}"
            if self.enable_test_data:
                status_msg += " | TEST MODE"
            self.status_bar.showMessage(status_msg)
    
    def update_plots(self):
        """Update all plot displays"""
        try:
            # Update Master Plot - All data over time
            self.update_master_plot()
            
            # Update individual plots based on current curves
            self.update_individual_plots()

        except Exception as e:
            print(f"Error updating plots: {e}")
    
    def update_individual_plots(self):
        """Update individual plots based on available curves"""
        # RPM vs Throttle plot
        if 'rpm_throttle' in self.plot_curves and len(self.rpm_throttle_data) > 1:
            self.plot_curves['rpm_throttle'].setData(list(self.rpm_throttle_data), list(self.rpm_values))
        
        # Battery plots
        if 'battery_voltage' in self.plot_curves and len(self.battery_throttle_data) > 1:
            self.plot_curves['battery_voltage'].setData(list(self.battery_throttle_data), list(self.voltage_values))
        if 'battery_current' in self.plot_curves and len(self.battery_throttle_data) > 1:
            self.plot_curves['battery_current'].setData(list(self.battery_throttle_data), list(self.current_values))
        
        # Time series plots
        if 'throttle_time' in self.plot_curves and len(self.throttle_time_data) > 1:
            time_axis = list(range(len(self.throttle_time_data)))
            self.plot_curves['throttle_time'].setData(time_axis, list(self.throttle_time_data))
        
        if 'voltage_time' in self.plot_curves and len(self.voltage_time_data) > 1:
            time_axis = list(range(len(self.voltage_time_data)))
            self.plot_curves['voltage_time'].setData(time_axis, list(self.voltage_time_data))
        
        if 'current_time' in self.plot_curves and len(self.current_time_data) > 1:
            time_axis = list(range(len(self.current_time_data)))
            self.plot_curves['current_time'].setData(time_axis, list(self.current_time_data))
        
        if 'accel_time' in self.plot_curves and len(self.accel_time_data) > 1:
            time_axis = list(range(len(self.accel_time_data)))
            self.plot_curves['accel_time'].setData(time_axis, list(self.accel_time_data))
        
        if 'rpm_time' in self.plot_curves and len(self.rpm_time_data) > 1:
            time_axis = list(range(len(self.rpm_time_data)))
            self.plot_curves['rpm_time'].setData(time_axis, list(self.rpm_time_data))
        
        if 'power' in self.plot_curves and len(self.power_data) > 1:
            time_axis = list(range(len(self.power_data)))
            self.plot_curves['power'].setData(time_axis, list(self.power_data))
        
        if 'efficiency' in self.plot_curves and len(self.efficiency_data) > 1:
            self.plot_curves['efficiency'].setData(list(self.efficiency_power), list(self.efficiency_data))
        
        # FFT plot
        if 'fft' in self.plot_curves and np.any(self.accel_buffer):
            # Apply window function to reduce spectral leakage
            windowed_data = self.accel_buffer * np.hanning(len(self.accel_buffer))
            
            # Compute FFT
            yf = fft(windowed_data)
            xf = fftfreq(len(windowed_data), 1/self.fft_sample_rate)
            
            # Take only positive frequencies up to Nyquist
            n = len(xf) // 2
            frequencies = xf[:n]
            magnitudes = 2.0/len(windowed_data) * np.abs(yf[:n])
            
            # Only show frequencies up to 50 Hz for typical drone vibrations
            max_freq_idx = np.where(frequencies <= 50)[0]
            if len(max_freq_idx) > 0:
                self.plot_curves['fft'].setData(frequencies[max_freq_idx], magnitudes[max_freq_idx])
        
        # ESC telemetry plots
        if 'esc_temp' in self.plot_curves and len(self.esc_temp_data) > 1:
            time_points = list(range(len(self.esc_temp_data)))
            self.plot_curves['esc_temp'].setData(time_points, list(self.esc_temp_data))
        if 'esc_current' in self.plot_curves and len(self.esc_current_data) > 1:
            time_points = list(range(len(self.esc_current_data)))
            self.plot_curves['esc_current'].setData(time_points, list(self.esc_current_data))
        if 'esc_temp_time' in self.plot_curves and len(self.esc_temp_data) > 1:
            time_points = list(range(len(self.esc_temp_data)))
            self.plot_curves['esc_temp_time'].setData(time_points, list(self.esc_temp_data))
    
    def update_master_plot(self):
        """Update the master plot with all telemetry data"""
        try:
            # Create time axis based on the longest data series
            max_len = max([len(data) for data in self.master_data.values()] + [1])
            time_axis = list(range(max_len))
            
            # Update each curve in the master plot
            for field, data in self.master_data.items():
                if field in self.master_curves and len(data) > 0:
                    # Create time axis for this specific data series
                    data_time = time_axis[-len(data):]
                    self.master_curves[field].setData(data_time, list(data))
                    
        except Exception as e:
            print(f"Error updating master plot: {e}")
    
    def closeEvent(self, event):
        """Handle application close"""
        print("Shutting down dashboard...")
        self.timer.stop()
        if hasattr(self, 'data_processor') and self.data_processor.mav:
            self.data_processor.mav.close()
        event.accept()

def main():
    """Main application entry point"""
    try:
        wakeup()
        app = QApplication(sys.argv)
        
        # Set application style
        app.setStyle('Fusion')
        
        # Create and show dashboard with test data enabled for demonstration
        dashboard = DroneDashboard(enable_test_data=True)  # Enable test data by default
        dashboard.show()
        
        print("Drone Dashboard started successfully!")
        print("Connect your drone and start sending MAVLink messages...")
        print("Test data is enabled - you should see simulated data flowing")
        
        sys.exit(app.exec_())
        
    except Exception as e:
        print(f"Failed to start application: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()