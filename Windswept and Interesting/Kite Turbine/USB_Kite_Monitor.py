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
num_cols = 14

fig = plt.figure()              # Create a figure instance and add a line chart
xs = list(range(0, log_size))   # Create static x-axis array to initialize chart

# Plot Wind Data
ax_wind = fig.add_subplot(2,4,1)
ax_wind.set_ylim([0, 20])       # Set the expected chart limits with y-data range
ax_wind.set_xlim([0, log_size]) # and x-number of data points and 

ys_wind = [np.NaN] * log_size   # "ys_*" valuse will be passed through the function to udpate 

plt_wind, = ax_wind.plot(xs, ys_wind)   # Plot chart to be streamed 
ax_wind.set_title('Windspeed (m/s)')    # Title chart 

# Plot turn rate
ax_turn = fig.add_subplot(2,4,2)
ax_turn.set_ylim([-10, 200])
ax_turn.set_xlim([0, 200])
ys_turn = [np.NaN] * log_size
plt_turn, = ax_turn.plot(xs, ys_turn)
ax_turn.set_title('Turn Rate (RPM)')

# Plot Tip Speed
ax_tip = fig.add_subplot(2,4,3)
ax_tip.set_ylim([0, 50])
ax_tip.set_xlim([0, 200])
ys_tip = [np.NaN] * log_size
plt_tip, = ax_tip.plot(xs, ys_tip)
ax_tip.set_title('Tip Speed (m/s)')

# Plot Tension
ax_ten = fig.add_subplot(2,4,4)
ax_ten.set_ylim([0, 130])
ax_ten.set_xlim([0, 200])
ys_ten = [np.NaN] * log_size
plt_ten, = ax_ten.plot(xs, ys_ten)
ax_ten.set_title('Tension (kg)')

# Plot Current
ax_bCurr = fig.add_subplot(2,4,5)
ax_bCurr.set_ylim([0, 17])
ax_bCurr.set_xlim([0, 200])
ys_bCurr = [np.NaN] * log_size
plt_bCurr, = ax_bCurr.plot(xs, ys_bCurr, label="Brake Current")
ax_bCurr.set_title('Brake Current (A) [Start-up Brake = 16 A]')

# Plot Max Current
ys_bCurr_max = [np.NaN] * log_size
plt_bCurr_max, = ax_bCurr.plot(xs, ys_bCurr_max, "--", label="Max Brake Current")
ax_bCurr.legend()

# Plot Tip Speed Ratio
ax_TSR = fig.add_subplot(2,4,6)
ax_TSR.set_ylim([0, 10])
ax_TSR.set_xlim([0, 200])
ys_TSR = [np.NaN] * log_size
plt_TSR, = ax_TSR.plot(xs, ys_TSR, label="Tip Speed Ratio")
ax_TSR.set_title('Tip Speed Ratio')

# Plot Set Tip Speed Ratio on same axis as Tip Speed Ratio
ys_TSR_set = [np.NaN] * log_size
plt_TSR_set, = ax_TSR.plot(xs, ys_TSR_set, "--", label="Set Tip Speed Ratio")
ax_TSR.legend()

# Plot Tension-Torque Ratio
ax_TTR = fig.add_subplot(2,4,7)
ax_TTR.set_ylim([0, 25])
ax_TTR.set_xlim([0, 200])
ys_TTR = [np.NaN] * log_size
plt_TTR, = ax_TTR.plot(xs, ys_TTR, label="Tension-Torque Ratio")
ax_TTR.set_title('Tension-Torque Ratio')

# Plot Set Tension-Torque Ratio
ys_TTR_set = [np.NaN] * log_size
plt_TTR_set, = ax_TTR.plot(xs, ys_TTR_set, "--", label = "Set Tension-Torque Ratio")
ax_TTR.legend()

# Plot Speed Quotent
ax_Q = fig.add_subplot(2,4,8)
ax_Q.set_ylim([0, 1])
ax_Q.set_xlim([0, 200])
ys_SpdQ = [np.NaN] * log_size
plt_SpdQ, = ax_Q.plot(xs, ys_SpdQ, label = "Speed")
ax_Q.set_title('Quotients')

# Plot Tip Speed Ratio Quotent
ys_TSRQ = [np.NaN] * log_size
plt_TSRQ, = ax_Q.plot(xs, ys_TSRQ, "--", label = "Tip Speed Ratio")

# Plot Tension-Torque Ratio Quotent
ys_TTRQ = [np.NaN] * log_size
plt_TTRQ, = ax_Q.plot(xs, ys_TTRQ, ".-", label = "Tension-Torque Ratio")
ax_Q.legend()


plt.subplots_adjust(left=0.1,
                    bottom=0.1, 
                    right=0.9, 
                    top=0.9, 
                    wspace=0.4, 
                    hspace=0.4)

now = dt.datetime.now()

f = open("Data/" + now.strftime("%Y-%m-%d_%H:%M:%S") + " KiteTurbine.csv", "w", encoding = 'UTF8', newline = '')
header = ['DateTime', 'Status','Wind Speed', 'RPM', 'Tip Speed', 'Tension', 
            'Brake Current', 'Max Brake Current', 'TSR', 'TSR Set', 'TTR', 'TTR Set', 
            'Speed Q', 'TTR Q', 'TSR Q']
writer = csv.writer(f)
writer.writerow(header)

def get_readings(ser):
    # Function to get the reading from the serial port 
    # (string) and clean it into a numpy array.
    arr = np.array([np.NaN] * num_cols)
    if serial:
        line = ser.readline() # read bytes until line-ending
        line = line.decode(encoding='UTF-8') # convert to string    
        split_line = line.splitlines()[0].split(',')
        if len(split_line) == num_cols:
            try: 
                arr = np.array(split_line).astype(np.float)
            except:
                print(f"Error in conversion: {split_line}")
        else:
            print(f"Error in length: {split_line}")
    else:
        print('Serial library failed to load')
        arr = np.array([
            np.random.randint(1,7), # States
            np.random.randint(6, 12), # Wind Speed
            np.random.randint(100, 150), # RPM
            np.random.randint(20, 30), # Tip Speed
            np.random.randint(110, 120), # Tension
            
            np.random.randint(1, 5), # Brake Current
            7, # Max BRake
            
            np.random.randint(3, 7), # TSR
            7, # TSR Set
            
            np.random.randint(15, 20), #TTR
            18, # TTR Set
            
            np.random.randint(10, 25)/100, # SpeedQ
            np.random.randint(10, 40)/100, # TTRQ
            10/100, # TSRQ
        ])        
    return arr

def animate(i, ser, ys_wind, ys_turn, ys_bCurr, ys_bCurr_max, ys_tip, ys_TSR, ys_TSR_set, ys_ten, ys_TTR, ys_TTR_set, ys_SpdQ, ys_TTRQ, ys_TSRQ):
    
    # Get USB readings from function above
    pos = get_readings(ser)
    # print(pos)
    
    ys_wind.append(pos[1])
    ys_turn.append(pos[2]) 
    ys_tip.append(pos[3]) 
    ys_ten.append(pos[4]) 

    ys_bCurr.append(pos[5])
    ys_bCurr_max.append(pos[6]) 

    ys_TSR.append(pos[7]) 
    ys_TSR_set.append(pos[8]) 
    
    ys_TTR.append(pos[9]) 
    ys_TTR_set.append(pos[10]) 
    
    ys_SpdQ.append(pos[11]) 
    ys_TTRQ.append(pos[12]) 
    ys_TSRQ.append(pos[13])

    # Limit y lists to log_size
    ys_wind = ys_wind[-log_size:]
    ys_turn = ys_turn[-log_size:]
    ys_bCurr = ys_bCurr[-log_size:]
    ys_bCurr_max = ys_bCurr_max[-log_size:]
    ys_tip = ys_tip[-log_size:]
    ys_TSR = ys_TSR[-log_size:]
    ys_TSR_set = ys_TSR_set[-log_size:]
    ys_ten = ys_ten[-log_size:]
    ys_TTR = ys_TTR[-log_size:]
    ys_TTR_set = ys_TTR_set[-log_size:]
    ys_SpdQ = ys_SpdQ[-log_size:]
    ys_TTRQ = ys_TTRQ[-log_size:]
    ys_TSRQ = ys_TSRQ[-log_size:]


    # Update the y items
    plt_wind.set_ydata(ys_wind)
    plt_turn.set_ydata(ys_turn) 
    plt_bCurr.set_ydata(ys_bCurr)
    plt_bCurr_max.set_ydata(ys_bCurr_max) 
    plt_tip.set_ydata(ys_tip) 
    plt_TSR.set_ydata(ys_TSR) 
    plt_TSR_set.set_ydata(ys_TSR_set) 
    plt_ten.set_ydata(ys_ten) 
    plt_TTR.set_ydata(ys_TTR) 
    plt_TTR_set.set_ydata(ys_TTR_set) 
    plt_SpdQ.set_ydata(ys_SpdQ) 
    plt_TTRQ.set_ydata(ys_TTRQ) 
    plt_TSRQ.set_ydata(ys_TSRQ)

    writer.writerow([now] + list(pos))

    return plt_wind, plt_turn, plt_bCurr, plt_bCurr_max, plt_tip, plt_TSR, plt_TSR_set, plt_ten, plt_TTR, plt_TTR_set, plt_SpdQ, plt_TTRQ, plt_TSRQ

if serial:    
    # Set serial port location and connection speed
    port = '/dev/ttyUSB0'
    baud = 115200 # 9600
    
    ser = serial.Serial(port, baud, timeout=1)
    ser.flushInput()
    # setup plot to call animate() funciton periodically
    ani = animation.FuncAnimation(
        fig, 
        animate, 
        fargs=(
            ser, 
            ys_wind, 
            ys_turn, 
            ys_bCurr, 
            ys_bCurr_max, 
            ys_tip, 
            ys_TSR, 
            ys_TSR_set, 
            ys_ten, 
            ys_TTR, 
            ys_TTR_set, 
            ys_SpdQ, 
            ys_TTRQ, 
            ys_TSRQ
        ), 
        interval=1, 
        blit=True
    )
    plt.show()
else:
    ser = 0
    # setup plot to call animate() funciton periodically
    ani = animation.FuncAnimation(
        fig, 
        animate, 
        fargs=(
            ser, 
            ys_wind, 
            ys_turn, 
            ys_bCurr, 
            ys_bCurr_max, 
            ys_tip, 
            ys_TSR, 
            ys_TSR_set, 
            ys_ten, 
            ys_TTR, 
            ys_TTR_set, 
            ys_SpdQ, 
            ys_TTRQ, 
            ys_TSRQ
        ), 
        interval=1, 
        blit=True
    )
    plt.show()

f.close()