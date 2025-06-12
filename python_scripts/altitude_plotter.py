#=================================
# Real-time PX4 Altitude Plotter
# Author: You & Elliot Lee
#=================================
import sys
import time
from collections import deque
from PyQt5 import QtWidgets
import pyqtgraph as pg
from pymavlink import mavutil
from wakeup import wakeup

# Wake up PX4 stream
wakeup()

# Connect to PX4 via MAVLink
connection = mavutil.mavlink_connection('udpin:0.0.0.0:14445')
connection.wait_heartbeat()

# Data buffers
window_size = 300
altitude_data = deque(maxlen=window_size)
timestamps = deque(maxlen=window_size)

# Qt application and plot setup
app = QtWidgets.QApplication([])
win = pg.GraphicsLayoutWidget(title="PX4 Real-Time Altitude")
plot = win.addPlot(title="Relative Altitude (meters)")
plot.showGrid(x=True, y=True)

alt_curve = plot.plot(pen='y', name="Relative Altitude")
win.show()

# Update function
def update():
    msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
    if msg:
        timestamp = time.time()
        relative_alt_m = msg.relative_alt / 1000.0  # from millimeters to meters

        timestamps.append(timestamp - timestamps[0] if timestamps else 0)
        altitude_data.append(relative_alt_m)

        alt_curve.setData(timestamps, altitude_data)

# Timer to update the plot
timer = pg.QtCore.QTimer()
timer.timeout.connect(update)
timer.start(10000)  # Update every 100 ms

# Run the Qt event loop
if __name__ == '__main__':
    QtWidgets.QApplication.instance().exec_()
