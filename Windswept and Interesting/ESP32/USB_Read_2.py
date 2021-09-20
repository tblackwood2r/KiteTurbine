serial = False
try: 
    import serial
except:
    pass

import csv
import datetime as dt
import numpy as np

import matplotlib.pyplot as plt
import matplotlib.animation as animation
# from mpl_toolkits import mplot3d

# Limite the number of decimals on 
np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

# The number of values that are going to be charted
log_size = 200

# Create a figure instance and add a line chart
fig = plt.figure()
ax = fig.add_subplot(211, projection="polar")

# Create static arrays that data will pass through
xs = list(range(0, log_size))
xs_roll = [np.NaN] * log_size
xs_pitch = [np.NaN] * log_size
xs_yaw = [np.NaN] * log_size

# Set the expected chart limits, 
# x-number of data points and y-data range
ax.set_ylim([0, log_size - 1])

# Setup the lines that are going to be streamed 
# with names for legend
pol0, = ax.plot(xs_roll, xs, label="Roll")
pol1, = ax.plot(xs_pitch, xs, label="Pitch")
pol2, = ax.plot(xs_yaw, xs, label="Yaw")

# Establish chart parameters 
ax.set_title('Roll, Pitch, and Yaw')
ax.set_ylabel('Angle')
ax.legend(bbox_to_anchor=(2, 1))

ax1 = fig.add_subplot(212)
ax1.set_ylim([-2, 2])
ax1.set_xlim([0, log_size])

ys_x = [np.NaN] * log_size
ys_y = [np.NaN] * log_size
ys_z = [np.NaN] * log_size

vecX, = ax1.plot(xs, ys_y, label="X")
vecY, = ax1.plot(xs, ys_y, label="Y")
vecZ, = ax1.plot(xs, ys_z, label="Z")

# Format plot
ax1.set_title('Accelleration X, Y, Z')
ax1.set_xlabel('Sample')
ax1.set_ylabel('Accelleration')

plt.subplots_adjust(left=0.1,
                    bottom=0.1, 
                    right=0.9, 
                    top=0.9, 
                    wspace=0.4, 
                    hspace=0.4)

now = dt.datetime.now()

f = open("Data/" + now.strftime("%Y-%m-%d_%H:%M:%S") + " ESP32.csv", "w", encoding = 'UTF8', newline = '')
header = ['DateTime', 'Roll','Pitch', 'Yaw', 'X', 'Y', 'Z']
writer = csv.writer(f)
writer.writerow(header)

def get_readings(ser, now):
    # Function to get the reading from the serial port 
    # (string) and clean it into a numpy array.
    arr = np.array([np.NaN] * 7)
    if serial:
        line = ser.readline() # read bytes until line-ending
        line = line.decode(encoding='UTF-8') # convert to string    
        split_line = line.splitlines()[0].split(',')
        if len(split_line) == 7:
            try: 
                arr = np.array(split_line).astype(np.float)
            except:
                error_time = dt.datetime.now() - now
                print(f"{error_time}: Error in conversion: {split_line}")
                now = dt.datetime.now()
        else:
            error_time = dt.datetime.now() - now
            print(f"{error_time}: Error in length: {split_line}")
            now = dt.datetime.now()
    else:
        print('No Serial Connection')        
    return arr

def animate(i, ser, xs_roll, xs_pitch, xs_yaw, ys_x, ys_y, ys_z):
    global now

    # Get USB readings from function above
    pos = get_readings(ser, now)
    # print(pos)
    tmp_now = dt.datetime.now()
    writer.writerow([tmp_now] + list(pos[0:5]))
    # update = now + dt.timedelta(milliseconds = 200)

    if pos[6] == 11: 
        xs_roll.append(pos[0]/180*np.pi)
        xs_pitch.append(pos[1]/180*np.pi)
        xs_yaw.append(pos[2]/180*np.pi)
        
        ys_x.append(pos[3])
        ys_y.append(pos[4])
        ys_z.append(pos[5])

        # Limit y lists to log_size
        xs_roll = xs_roll[-log_size:]
        xs_pitch = xs_pitch[-log_size:]
        xs_yaw = xs_yaw[-log_size:]

        ys_x = ys_x[-log_size:]
        ys_y = ys_y[-log_size:]
        ys_z = ys_z[-log_size:]

        # Update the y items
        pol0.set_xdata(xs_roll)
        pol1.set_xdata(xs_pitch)
        pol2.set_xdata(xs_yaw)

        vecX.set_ydata(ys_x)
        vecY.set_ydata(ys_y)
        vecZ.set_ydata(ys_z)
    
    return pol0, pol1, pol2, vecX, vecY, vecZ

# Set serial port location and connection speed
port = '/dev/ttyUSB0'
baud = 115200 # 9600

if serial:    
    ser = serial.Serial(port, baud, timeout=1)
    ser.flushInput()
    # setup plot to call animate() funciton periodically
    ani = animation.FuncAnimation(fig, animate, fargs=(ser, xs_roll, xs_pitch, xs_yaw, ys_x, ys_y, ys_z), interval=1, blit=True)
    plt.show()
else:
    ser = 0
    # setup plot to call animate() funciton periodically
    ani = animation.FuncAnimation(fig, animate, fargs=(ser, xs_roll, xs_pitch, xs_yaw, ys_x, ys_y, ys_z), interval=1, blit=True)
    plt.show()

f.close()