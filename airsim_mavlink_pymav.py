import os
import time
os.environ['MAVLINK20'] = '1'
from pymavlink import mavutil
import time
import signal
import threading
import pymavlink.dialects.v20.all as dialect
import keyboard
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from madgwickahrs import MadgwickAHRS
import numpy as np
import math

# Create an AHRS filter instance
ahrs = MadgwickAHRS()

gyro_data_ahrs = np.array([0.1, -0.2, 0.3])  # Angular velocities (rad/s)
accel_data_ahrs = np.array([0.0, 0.0, 9.81])  # Linear accelerations (m/s^2)
mag_data_ahrs = np.array([0.4, 0.0, 0.6])     # Magnetometer data (uT)

# Start a connection listening to a UDP port
connection = mavutil.mavlink_connection('tcp:127.0.0.1:4560', source_system=1, source_component=1) # conn udp_bridge
#connection = mavutil.mavlink_connection('udpout:127.0.0.1:5300') # conn udp_no bridge
#connection = mavutil.mavlink_connection('udpin:127.0.0.1:14540') # conn pymavlink
time.sleep(1)

print("sysid ", connection.target_system)
print("cmp ",connection.target_component)

is_running = True

# Initialize your variables
throttle_ctrl = 0.47
pitch_ctrl = 0
roll_ctrl = 0
yaw_ctrl = 0

m0 = 0
m1 = 0
m2 = 0
m3 = 0


data_file_ahrs = 'ahrs_data.txt'
with open(data_file_ahrs, 'w') as file:
    pass

# Define a function to increase the variable
def increase_thr():
    global throttle_ctrl
    throttle_ctrl += 0.005
    # print(f"Variable increased: {throttle_ctrl}")

# Define a function to decrease the variable
def decrease_thr():
    global throttle_ctrl
    throttle_ctrl -= 0.005
    # print(f"Variable decreased: {throttle_ctrl}")

def increase_pitch():
    global pitch_ctrl
    pitch_ctrl += 0.0005

def decrease_pitch():
    global pitch_ctrl
    pitch_ctrl -= 0.0005

def increase_roll():
    global roll_ctrl
    roll_ctrl += 0.0005

def decrease_roll():
    global roll_ctrl
    roll_ctrl -= 0.0005

def increase_yaw():
    global yaw_ctrl
    yaw_ctrl += 0.0005

def decrease_yaw():
    global yaw_ctrl
    yaw_ctrl -= 0.0005

# Register hotkeys for controlling the variable
keyboard.add_hotkey('up', increase_thr)  # Press the 'Up' arrow key
keyboard.add_hotkey('down', decrease_thr)  # Press the 'Down' arrow key

keyboard.add_hotkey('w', increase_pitch)  # Press the 'Up' arrow key
keyboard.add_hotkey('s', decrease_pitch)  # Press the 'Down' arrow key

keyboard.add_hotkey('a', increase_roll)  # Press the 'Up' arrow key
keyboard.add_hotkey('d', decrease_roll)  # Press the 'Down' arrow key

keyboard.add_hotkey('x', increase_yaw)  # Press the 'Up' arrow key
keyboard.add_hotkey('z', decrease_yaw)  # Press the 'Down' arrow key

class ImuAirsim:
    def __init__(self):
        self.time_usec = 0
        self.xacc = 0.0
        self.yacc = 0.0
        self.zacc = 0.0
        self.xgyro = 0.0
        self.ygyro = 0.0
        self.zgyro = 0.0
        self.pressure_alt = 0.0
        self.abs_pressure = 0.0
        self.diff_pressure = 0.0
        self.temperature = 0.0
        self.xmag = 0.0
        self.ymag = 0.0
        self.zmag = 0.0
        self.data_file = 'imu_data.txt'
        with open(self.data_file, 'w') as file:
            pass

    def update(self, msg):
        if msg.get_type() == 'HIL_SENSOR':
            imu_data = msg.to_dict()
            self.time_usec = imu_data['time_usec']
            self.xacc = imu_data['xacc']
            self.yacc = imu_data['yacc']
            self.zacc = imu_data['zacc']
            self.xgyro = imu_data['xgyro']
            self.ygyro = imu_data['ygyro']
            self.zgyro = imu_data['zgyro']
            self.pressure_alt = imu_data['pressure_alt']
            self.abs_pressure = imu_data['abs_pressure']
            self.diff_pressure = imu_data['diff_pressure']
            self.temperature = imu_data['temperature']
            self.xmag = imu_data['xmag']
            self.ymag = imu_data['ymag']
            self.zmag = imu_data['zmag']
            self.save_data()
    
    def save_data(self):
        with open(self.data_file, 'a') as file:
            file.write(f'IMU: Time_usec={self.time_usec}, XAcc={self.xacc}, YAcc={self.yacc}, ZAcc={self.zacc}, '
                       f'XGyro={self.xgyro}, YGyro={self.ygyro}, ZGyro={self.zgyro}, '
                       f'PressureAlt={self.pressure_alt}, AbsPressure={self.abs_pressure}, '
                       f'DiffPressure={self.diff_pressure}, Temperature={self.temperature}, '
                       f'XMag={self.xmag}, YMag={self.ymag}, ZMag={self.zmag}\n')

    def printInfo(self):
        print(f"Time (usec): {self.time_usec}, X Accel: {self.xacc}, Y Accel: {self.yacc}, Z Accel: {self.zacc}, X Gyro: {self.xgyro}, Y Gyro: {self.ygyro}, Z Gyro: {self.zgyro}, Pressure Alt: {self.pressure_alt}, Pressure: {self.abs_pressure}, Temperature: {self.temperature}, MagX: {self.xmag}, MagY: {self.ymag}, MagZ: {self.zmag}")

class GPSAirsim:
    def __init__(self):
        self.time_usec = 0
        self.fix_type = 0
        self.lat = 0
        self.lon = 0
        self.alt = 0
        self.eph = 0
        self.epv = 0
        self.vel = 0
        self.vn = 0
        self.ve = 0
        self.vd = 0
        self.cog = 0
        self.satellites_visible = 0
        self.data_file = 'gps_data.txt'
        with open(self.data_file, 'w') as file:
            pass

    def update(self, msg):
        if msg.get_type() == 'HIL_GPS':
            gps_data = msg.to_dict()
            self.time_usec = gps_data['time_usec']
            self.fix_type = gps_data['fix_type']
            self.lat = gps_data['lat']
            self.lon = gps_data['lon']
            self.alt = gps_data['alt']
            self.eph = gps_data['eph']
            self.epv = gps_data['epv']
            self.vel = gps_data['vel']
            self.vn = gps_data['vn']
            self.ve = gps_data['ve']
            self.vd = gps_data['vd']
            self.cog = gps_data['cog']
            self.satellites_visible = gps_data['satellites_visible']
            #print(gps_data['satellites_visible'])

            self.save_data()
    
    def save_data(self):
        with open(self.data_file, 'a') as file:
            file.write(f'GPS: Time_usec={self.time_usec}, FixType={self.fix_type}, '
                       f'Lat={self.lat}, Lon={self.lon}, Alt={self.alt}, '
                       f'EPH={self.eph}, EPV={self.epv}, Vel={self.vel}, '
                       f'VN={self.vn}, VE={self.ve}, VD={self.vd}, '
                       f'COG={self.cog}, SatellitesVisible={self.satellites_visible}\n')

    def printInfo(self):
        print(f"Time (usec): {self.time_usec}, Fix Type: {self.fix_type}, Latitude: {self.lat}, Longitude: {self.lon}, Altitude: {self.alt}, Satellites: {self.satellites_visible}")

imu_a = ImuAirsim()
gps_a = GPSAirsim()
imu_a.printInfo()
gps_a.printInfo()

def thread_rx_mavproxy():
    global is_running
    global imu_a
    global gps_a
    global ahrs
    global data_file_ahrs
    try:
        while(is_running):
            try:
                #print(connection.recv_match().to_dict())
                msg = connection.recv_match(blocking=False) #mavlink_conn.recv_msg()
                # type_msg = msg_decoded.get_type()
                if msg:
                    if msg.get_type() == 'HIL_GPS':
                        # Handle HIL_GPS message received from AirSim
                        # gps_data = msg.to_dict()
                        gps_a.update(msg)
                        #gps_a.printInfo()
                        #print(f"Received HIL_GPS: {gps_data}")

                    elif msg.get_type() == 'HIL_SENSOR':
                        # Handle HIL_SENSOR message received from AirSim
                        imu_a.update(msg)
                        # gyro_data = np.array([imu_a.xgyro, -imu_a.ygyro, -imu_a.zgyro])  # Angular velocities (rad/s)
                        # accel_data = np.array([imu_a.xacc, -imu_a.yacc, -imu_a.zacc])  # Linear accelerations (m/s^2)
                        # mag_data = np.array([imu_a.xmag, -imu_a.ymag, -imu_a.zmag])     # Magnetometer data (uT)
                        # ahrs.update(gyro_data, accel_data, mag_data)
                        # roll, pitch, yaw = ahrs.quaternion.to_euler_angles()
                        pitch = math.atan2(imu_a.xacc, math.sqrt(imu_a.yacc**2 + imu_a.zacc**2))
                        roll = math.atan2(-imu_a.yacc, -imu_a.zacc)
                        print(roll,pitch,0)
                        with open(data_file_ahrs, 'a') as file:
                            file.write(f'AHRS: Dunny={0}, Yaw={0}, Pitch={pitch}, Roll={roll}\n')
                        # imu_a.printInfo()

                        
            except:
                pass
        time.sleep(0.05)
    except KeyboardInterrupt:
        is_running = False
        print("Exiting...")
    finally:
        # Close the connection and the socket
        is_running = False
        connection.close()

def hb_send():
    global is_running
    try:
        while(is_running):
                connection.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_QUADROTOR,
                    mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                    mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
                    0,
                    mavutil.mavlink.MAV_STATE_ACTIVE,
                )
                #print("SentHB")
                time.sleep(1)
    except KeyboardInterrupt:
        is_running = False
        print("Exiting...")
    finally:
        is_running = False
        # Close the connection and the socket
        hb_thread.join()

def send_actuators(motor0, motor1, motor2, motor3):
    global imu_a
    global gps_a
    global is_running
    mode_flags = desired_flags = (
                mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED |
                mavutil.mavlink.MAV_MODE_FLAG_HIL_ENABLED |
                mavutil.mavlink.MAV_MODE_FLAG_STABILIZE_ENABLED |
                mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
                )
    try:
        # msg = mavutil.mavlink.MAVLink_hil_actuator_controls_message(
        #             time_usec=int(time.time() * 1e6),  # Use current time in microseconds
        #             controls = [motor0, motor1, motor2, motor3,
        #                         0.0, 0.0, 0.0, 0.0,
        #                         0.0, 0.0, 0.0, 0.0,
        #                         0.0, 0.0, 0.0, 0.0],
        #             mode=mode_flags,  # Use 0 for manual mode, adjust as needed
        #             flags=0  # Set the desired flags
        #         )

        # # Send the manually created HIL_ACTUATOR_CONTROLS message to AirSim
        # #msg.pack(connection)
        # print("Sending act")
        # connection.mav.send(msg)
        # print("Sent act")
        # connection.mav.hil_actuator_controls_send(
        #             time_usec=int(time.time() * 1e6),  # Use current time in microseconds
        #             controls = [motor0, motor1, motor2, motor3,
        #                         0.0, 0.0, 0.0, 0.0,
        #                         0.0, 0.0, 0.0, 0.0,
        #                         0.0, 0.0, 0.0, 0.0],
        #             mode=mode_flags,  # Use 0 for manual mode, adjust as needed
        #             flags=0  # Set the desired flags
        #         )
        actuator_controls_msg = connection.mav.hil_actuator_controls_encode(
            time_usec=int(time.time() * 1e6),  # Use current time in microseconds
            controls=[motor0, motor1, motor2, motor3,
                                0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0],
            mode=desired_flags,  # Use 0 for manual mode, adjust as needed
            flags=0  # Set the desired flags
        )
        #print(actuator_controls_msg)
        # Send the HIL_ACTUATOR_CONTROLS message to AirSim
        connection.mav.send(actuator_controls_msg)
        #print("sent actuators info")
    except KeyboardInterrupt:
        print("Error sending actuator...")
    finally:
        #print("Fail sendign actutaror")
        pass

def mixingMotors():
    global throttle_ctrl
    global pitch_ctrl
    global roll_ctrl
    global yaw_ctrl

    global m0
    global m1
    global m2
    global m3

    m0 = throttle_ctrl + yaw_ctrl - pitch_ctrl + roll_ctrl 
    m1 = throttle_ctrl + yaw_ctrl + pitch_ctrl - roll_ctrl
    m2 = throttle_ctrl - yaw_ctrl - pitch_ctrl - roll_ctrl
    m3 = throttle_ctrl - yaw_ctrl + pitch_ctrl + roll_ctrl

y = threading.Thread(target=thread_rx_mavproxy, args=())
y.start()
hb_thread = threading.Thread(target=hb_send, args=())
hb_thread.start()
try:
    while(is_running):
        mixingMotors()
        send_actuators(m0, m1, m2, m3)
        time.sleep(0.01)
except KeyboardInterrupt:
    is_running = False
    print("Exiting...")
finally:
    is_running = False
    # Close the connection and the socket
    y.join()
y.join()
