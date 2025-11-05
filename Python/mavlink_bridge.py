#!/usr/bin/env python3

from pymavlink import mavutil
from random import random
import time
import re
import socket
import threading
import string
import ast

global HOST
global SEND_PORT
global REC_PORT
global r
global s
global threads_should_stop
global _DATA
global s1

int_16_max = 65535
n = 0

# ---------------------------------------- #
# input byte value
# return array of signed float values
# Q = 127
# ---------------------------------------- #
def _byte_decode(x):
    Q = 127
    j = 1
    data = []
    for i in range(0, len(x)):
        if(~j & 1): # is even
            big = x[i] << 8 # first 8 bits
            little = x[i-1] # last 8 bits
            num = big + little
            if(big >> 15): # negative
                num = -1 * ((num ^ int_16_max) + 1) # twos compliment
            data.append(num/Q) # all data *127
        j += 1
    return data

# ---------------------------------------- #
# input array of signed float values
# input values range [-1,1]
# return array of signed 16bit integers
# Q = 10000
# ---------------------------------------- #
def _pack_bytes(y):
    b1 = "b\'"
    be = "\'"
    h1 = "\\x"
    packed_bytes = b1
    Q = 10000
    singedb = 1 << 15 # 0b1000000000000000
    for num in y: # -1 <= num <= 1
        num *= Q # float q value
        if(num<0):  # find singed eqivalent
            num = ((-num) ^ int_16_max) + 1
        hn = hex(int(num))[2:] # get rid of 0x
        while len(hn) < 4: # pad with zeros
            hn = "0"+hn
        big = hn[0:2]  # first two bytes
        little = hn[2:] # last two bytes
        big = ("0" + big) if len(big) < 2 else big
        little = ("0" + little) if len(little) < 2 else little
        packed_bytes += ((h1 + little) + (h1 + big)) # little endien
    byte_object = ast.literal_eval(packed_bytes + be)
    return byte_object

def _rec_data():
    global threads_should_stop
    global _DATA
    while not threads_should_stop:
        _DATA, addr = s1.recvfrom(1024) # recv(512)
        
        print("\n REC RAW:: ", _DATA)
        print("Convert:: ", _byte_decode(_DATA), "\n")

        if not s1.recvfrom(1024): # recv(1024):
            threads_should_stop = True

def update_hil_rc_inputs_raw(
    time_since_boot,    # uint64_t, us
    chan1_raw,          # uint16_t
    chan2_raw,          # uint16_t
    chan3_raw,          # uint16_t
    chan4_raw,          # uint16_t
    chan5_raw,          # uint16_t
    chan6_raw,          # uint16_t
    chan7_raw,          # uint16_t
    chan8_raw,          # uint16_t
    chan9_raw,          # uint16_t
    chan10_raw,         # uint16_t
    chan11_raw,         # uint16_t
    chan12_raw,         # uint16_t
    rssi = 225          # uint8_t, UINT8_MAX: invalid/unknown.
):
    vehicle.mav.hil_rc_inputs_raw_send(
        time_since_boot,
        chan1_raw,
        chan2_raw,
        chan3_raw,
        chan4_raw,
        chan5_raw,
        chan6_raw,
        chan7_raw,
        chan8_raw,
        chan9_raw,
        chan10_raw,
        chan11_raw,
        chan12_raw,
        rssi
    )


def update_hil_gps(
        time_since_boot, #uint64_t
        fix_type, #uint8_t
        lat, #int32_t
        lon, #int32_t
        alt, #int32_t
        eph, #uint16_t
        epv, #uint16_t
        vel, #uint16_t
        nv, #int16_t
        ve, #int16_t
        vd,#int16_t
        cog, #uint16_t
        sat_vis, #uint8_t
        id_ = 0, #uint8_t
        yaw = 0 #uint16_t
):
    vehicle.mav.hil_gps_send(
        time_since_boot,
        fix_type,
        lat,
        lon,
        alt,
        eph,
        epv,
        vel,
        nv,
        ve,
        vd,
        cog,
        sat_vis,
        id_,
        yaw
    )
    return

def update_hil(
        time_since_boot,   #uint64_t
        x_accel,           #float
        y_accel,           #float
        z_accel,           #float
        ang_speed_x,       #float
        ang_speed_y,       #float
        ang_speed_z,       #float
        mag_field_x,       #float
        mag_field_y,       #float
        mag_field_z,       #float
        abs_pressure,      #float
        airspeed_,         #float
        altitude_pressure, #float
        temp,              #float
        fields_updated = 7167,      #uint32_t  value taken from example code
        id_ = 0
):
    vehicle.mav.hil_sensor_send(
        time_since_boot,      #Timestamp (UNIX Epoch time or time since system boot), us
        x_accel,      #X acceleration, m/s/s
        y_accel,      #Y acceleration, m/s/s
        z_accel,      #Z acceleration, m/s/s
        ang_speed_x,      #Angular speed around X axis in body frame, rad/s
        ang_speed_y,      #Angular speed around Y axis in body frame, rad/s
        ang_speed_z,      #Angular speed around Z axis in body frame, rad/s
        mag_field_x,      #X Magnetic field, gauss
        mag_field_y,      #Y Magnetic field, gauss
        mag_field_z,      #Z Magnetic field, gauss
        abs_pressure,      #Absolute pressure, hPa
        airspeed_,      #Differential pressure (airspeed), hPa
        altitude_pressure,      #Altitude calculated from pressure
        temp,      #Temperature
        fields_updated,       #Bitmap for fields that have updated since last message
        id_
        )

def update_hil_state_quart(
        time_since_boot,    #uint64_t
        attitude_quat,  #float[4]
        rollspeed,  #float
        pitchspeed, #float
        yawspeed,   #float
        lat,    #int32_t
        lon,    #int32_t
        alt,    #int32_t
        vx, #int16_t
        vy, #int16_t
        vz, #int16_t
        ind_airspeed,   #uint16_t
        true_airspeed,  #uint16_t
        xacc,   #int16_t
        yacc,   #int16_t
        zacc    #int16_t
):
    
    vehicle.mav.hil_state_quaternion_send(
        time_since_boot,      #Timestamp (UNIX Epoch time or time since system boot), us
        attitude_quat,      #Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotation), float[4]
        rollspeed,      #Body frame roll / phi angular speed, rad/s
        pitchspeed,      #Body frame pitch / theta angular speed, rad/s
        yawspeed,      #Body frame yaw / psi angular speed, rad/s
        lat,      #Latitude, degE7
        lon,      #Longitude, degE7
        alt,      #Altitude, degE7
        vx,      #Ground X Speed (Latitude), cm/s
        vy,      #Ground Y Speed (Longitude), cm/s
        vz,      #Ground Z Speed (Altitude), cm/s
        ind_airspeed,      #Indicated airspeed, cm/s
        true_airspeed,      #True airspeed, cm/s
        xacc,      #X acceleration, mG
        yacc,      #Y acceleration, mG
        zacc      #Z acceleration, mG
        )

def crandom():
    return random()-0.5

if __name__ == "__main__":

    # set up plant ip and port
    HOST = '127.0.0.1'
    SEND_PORT = 65432
    REC_PORT = 65431
    _DATA = 1
    threads_should_stop = False

    print("Initializing connection to plant...")

    # recieve connection blip from plant
    s1 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # udp
    s1.bind((HOST, REC_PORT))
    r, addr = s1.recvfrom(512)
    print(f"Connected by {addr}")
    print("Reciever established.")

    # send connection confirmation to plant
    s2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # udp
    # MESSAGE = b"Hello, World!"
    # s2.sendto(MESSAGE, (HOST, SEND_PORT))
    print("Sender established.")

    # create threads
    t1 = threading.Thread(target=_rec_data, args=())
    # t2 = threading.Thread(target=_send_data, args=())

    # ---------- Begin Connection Sequence ----------

    # ---------- Create vehicle instance ----------
    vehicle = mavutil.mavlink_connection('tcpin:0.0.0.0:4564') #172.24.147.121

    print("Initializing Connection to PX4...")

    vehicle.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GENERIC,       #Vehicle or component type.
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,  #Autopilot type / class.
            0,                                      #System mode bitmap.
            0,                                      #A bitfield for use for autopilot-specific flags
            0                                       #System status flag.
            )


    # ---------- Wait for heartbeat ----------
    vehicle.wait_heartbeat()

    print("PX4 Connection Succesfull")

    print("Heartbeat from system (system %u component %u)" % (vehicle.target_system, vehicle.target_component))

    # ---------- End Connection Sequence ----------

    # current time mark
    current_time_us = round(time.time() * 1e6) #in microseconds

    # time at loop start
    time_loop_start_us = round(time.time() * 1e6 - 30e6) #in microseconds

    initializing = True # used to generate data for initia connecting to px4

    n = 0   # used to cound how many times we have recieved mav message from px4

    global actuator_cmd

    # begin plant connection relay
    print("Beginning threads...")
    _SEND = [0,0,0,0,0,0,0,0]
    t1.start()
    # t2.start()

    # ---------- Begin main connection loop ----------
    while vehicle != None and not threads_should_stop:

        # Update time
        time_since_loop_start_us = current_time_us - time_loop_start_us
        time_since_loop_start_ms = round(time_since_loop_start_us / 1e3)

        if initializing:
            # ---------- create dummy data to start px4 ----------
            h_lat = 40.073964
            h_lon = -83.079924
            h_alt = 276
            #hil
            x_accel =           float((   0           +crandom()*0.2      )*1)   #float
            y_accel =           float((   0           +crandom()*0.2      )*1)   #float
            z_accel =           float((   -9.81       +crandom()*0.2      )*1)   #float
            ang_speed_x =       float((   0           +crandom()*0.04     )*1)   #float
            ang_speed_y =       float((   0           +crandom()*0.04     )*1)   #float
            ang_speed_z =       float((   0           +crandom()*0.04     )*1)   #float
            mag_field_x =       float((   0.215       +crandom()*0.02     )*1)   #float
            mag_field_y =       float((   0.01        +crandom()*0.02     )*1)   #float
            mag_field_z =       float((   0.43        +crandom()*0.02     )*1)   #float
            abs_pressure =      float((   95598       +crandom()*4        )*0.01)  #float
            airspeed_ =         float((   95598       +crandom()*0        )*0.01) #float        Ser to 0 initaly, same as abs pressure if no speed, 95598
            altitude_pressure = float((   276         +crandom()*0.5      )*1) #float
            temp_ =             float((   0           +crandom()*0        )*1)  #float

            #hil_quat
            attitude_quat = [
                            float((   1           +crandom()*0        )*1), 
                            float((   0           +crandom()*0        )*1), 
                            float((   0           +crandom()*0        )*1), 
                            float((   0           +crandom()*0        )*1)
                ] #float[4]
            rollspeed =     float((   0           +crandom()*0        )*1)  #float
            pitchspeed =    float((   0           +crandom()*0        )*1) #float
            yawspeed =      float((   0           +crandom()*0        )*1)   #float
            lat =           round((   h_lat       +crandom()*5e-7     )*1e7)    #int32_t  same as gps
            lon =           round((   h_lon       +crandom()*5e-7     )*1e7)    #int32_t    same as gps
            alt =           round((   h_alt       +crandom()*0.05     )*1000)    #int32_t   same as gps
            vx =            round((   0           +crandom()*0.001    )*100) #int16_t
            vy =            round((   0           +crandom()*0.001    )*100) #int16_t
            vz =            round((   0           +crandom()*0.001    )*100) #int16_t
            ind_airspeed =  round((   0           + random()*0.001    )*100)   #uint16_t
            true_airspeed = round((   0           + random()*0.3      )*100)  #uint16_t
            xacc =          round((   0           +crandom()*0.001    )*100)   #int16_t
            yacc =          round((   0           +crandom()*0.001    )*100)   #int16_t
            zacc =          round((   0           +crandom()*0.001    )*100)    #int16_t

            #gps
            lat =       round((   h_lat       +crandom()*5e-7     )*1e7) #int32_t
            lon =       round((   h_lon       +crandom()*5e-7     )*1e7) #int32_t
            alt =       round((   h_alt       +crandom()*0.05     )*1000) #int32_t
            eph =       round((   0.3         + random()*0.001    )*100) #uint16_t
            epv =       round((   0.4         + random()*0.001    )*100) #uint16_t
            vel =       round((   0           + random()*0.001    )*100) #uint16_t
            nv =        round((   0           + random()*0.001    )*100) #int16_t
            ve =        round((   0           + random()*0.001    )*100) #int16_t
            vd =        round((   0           + random()*0.001    )*100)   #int16_t
            cog =       round((   0           +crandom()*0.001    )*100) #uint16_t
            fix_type =  3 #uint8_t
            sat_vis =   10 #uint8_t

        else:

            # ---------- update data from plant model ----------
            #get data
            data = plant_data(MODEL_SOCKET)

            #define variables

            #hil
            x_accel = data[0]   #float
            y_accel = data[1]   #float
            z_accel = data[2]   #float
            ang_speed_x = data[3]   #float
            ang_speed_y = data[4]   #float
            ang_speed_z = data[5]   #float
            mag_field_x = data[6]   #float
            mag_field_y = data[7]   #float
            mag_field_z = data[8]   #float
            abs_pressure = data[9]  #float
            airspeed_ = data[10] #float
            altitude_pressure = data[11] #float
            temp_ = data[12]  #float

            #hil_quat
            attitude_quat = [data[13], data[14], data[15], data[16]] #float[4]
            rollspeed = data[17]  #float
            pitchspeed = data[18] #float
            yawspeed = data[19]   #float
            lat = data[20]    #int32_t
            lon = data[21]    #int32_t
            alt = data[22]    #int32_t
            vx = data[23] #int16_t
            vy = data[24] #int16_t
            vz = data[25] #int16_t
            ind_airspeed = data[26]   #uint16_t
            true_airspeed = data[27]  #uint16_t
            xacc = data[28]   #int16_t
            yacc = data[29]   #int16_t
            zacc = data[30]    #int16_t

            #gps
            lat = data[31] #int32_t
            lon = data[32] #int32_t
            alt = data[33] #int32_t
            eph = data[34] #uint16_t
            epv = data[35] #uint16_t
            vel = data[36] #uint16_t
            nv = data[37] #int16_t
            ve = data[38] #int16_t
            vd = data[39]   #int16_t
            cog = data[40] #uint16_t
            fix_type = data[41] #uint8_t
            sat_vis = data[42] #uint8_t

        # Taken from JMavSim
        # 4ms between HIL_SENSOR messages, 4e3 us
        time_hil_sensor_us = 4e3
        # 8ms between HIL_STATE_QUATERNION messages, 8e3 us
        time_hil_quat_us = 8e3
        # 52ms between HIL_GPS messages, 5.2e4 us
        time_hil_gps_us = 5.2e4
        # 1000ms between each HEARTBEAT, 1e6 us
        time_heartbeat_us = 1e6
        # 4000ms between SYSTEM_TIME, 4e6 us
        time_system_time_us = 4e6

        # ---------- Send time stamp ----------
        if time_since_loop_start_us % time_system_time_us == 0:
            vehicle.mav.system_time_send(
                current_time_us,   #Timestamp (UNIX epoch time), us
                time_since_loop_start_ms  #Timestamp (time since system boot), ms
            )
            # print("Timestamp::\n")
            # print("     ", current_time_us, " us")
            # print("     ", time_since_loop_start_ms, " ms")
            # print("")

        # ---------- Send heartbeat ----------
        if time_since_loop_start_us % time_heartbeat_us == 0:
            the_type        = 0     # Vehicle or component type. For a flight controller component the vehicle type (quadrotor, helicopter, etc.). For other components the component type (e.g. camera, gimbal, etc.). This should be used in preference to component id for identifying the component type. (type:uint8_t, values:MAV_TYPE)
            autopilot       = 0     # Autopilot type / class. Use MAV_AUTOPILOT_INVALID for components that are not flight controllers. (type:uint8_t, values:MAV_AUTOPILOT)
            base_mode       = 0     # System mode bitmap. (type:uint8_t, values:MAV_MODE_FLAG)
            custom_mode     = 0     # A bitfield for use for autopilot-specific flags (type:uint32_t)
            system_status   = 0     # System status flag. (type:uint8_t, values:MAV_STATE)
            mavlink_version = 3     # MAVLink version, not writable by user, gets added by protocol because of magic data type          , # uint8_t_mavlink_version (type:uint8_t)

            vehicle.mav.heartbeat_send(
                the_type,       #Vehicle or component type.
                autopilot,  #Autopilot type / class.
                base_mode,                               #System mode bitmap.
                custom_mode,                               #A bitfield for use for autopilot-specific flags
                system_status,                             #System status flag.
                mavlink_version
                )
            # print("Heartbeat::")
            # print("     ", mavutil.mavlink.MAV_TYPE_GENERIC)
            # print("     ", mavutil.mavlink.MAV_AUTOPILOT_INVALID)
            # print("     ", sys_mode)
            # print("     ", bitfield)
            # print("     ", status_flag)
            

        # ---------- Send hil sensor ----------
        if time_since_loop_start_us % time_hil_sensor_us == 0:
            update_hil(
                time_since_loop_start_us,
                x_accel,
                y_accel,
                z_accel,
                ang_speed_x,
                ang_speed_y,
                ang_speed_z,
                mag_field_x,
                mag_field_y,
                mag_field_z,
                abs_pressure,
                airspeed_,
                altitude_pressure,
                temp_
            )
            # print("HIL Sensor::",
            #     "       x_accel:: ", x_accel, "\n"
            #     "       y_accel:: ", y_accel, "\n"
            #     "       z_accel:: ", z_accel
            #     )
        
        # ---------- Send hil quaternion ----------
        if time_since_loop_start_us %  time_hil_quat_us == 0:
            update_hil_state_quart(
                time_since_loop_start_us,
                attitude_quat,
                rollspeed,
                pitchspeed,
                yawspeed,
                lat,
                lon,
                alt,
                vx,
                vy,
                vz,
                ind_airspeed,
                true_airspeed,
                xacc,
                yacc,
                zacc
            )

        # ---------- Send gps ----------
        if time_since_loop_start_us % time_hil_gps_us == 0:
            update_hil_gps(
                time_since_loop_start_us,
                fix_type,
                lat,
                lon,
                alt,
                eph,
                epv,
                vel,
                nv,
                ve,
                vd,
                cog,
                sat_vis
            )

        msg = vehicle.recv_match(blocking = False)
        if msg != None:
            n += 1
            # print(f"N:: {n}")
            concat_msg = re.findall(r'\[.*?\]', str(msg))
            actuator_cmd = concat_msg[0].split(', ')
            actuator_cmd[0] = actuator_cmd[0][1:len(actuator_cmd[0])]
            actuator_cmd[15] = actuator_cmd[15][:len(actuator_cmd[15])-1] # magic number, known 16 index
            _SEND[0] = float(actuator_cmd[0])
            _SEND[1] = float(actuator_cmd[1])
            _SEND[2] = float(actuator_cmd[2])
            _SEND[3] = float(actuator_cmd[3])
            _SEND[4] = float(actuator_cmd[4])
            _SEND[5] = float(actuator_cmd[5])
            _SEND[6] = float(actuator_cmd[6])
            _SEND[7] = float(actuator_cmd[7])

# ---------------------- SEND DATA ----------------------------------------------------
            # packed = _pack_bytes([0.0010000000474974513, 0.0010000000474974513, 0.0010000000474974513, 0.0010000000474974513, 0, 0, 0, 0]) # sample data
            # s2.sendto(packed, (HOST, SEND_PORT))
            
            pack = _pack_bytes(_SEND)
            s2.sendto(pack, (HOST, SEND_PORT)) #_pack_byte_array(_DATA)

            # ---------- DEBUG ----------
            print(f"RAW: {_SEND}")
            print(f"BYTES SENT: {pack}") # SENT: [0.003000000026077032, 0.003000000026077032, 0.003000000026077032, 0.003000000026077032, 0.0, 0.0, 0.0, 0.0]
            unpacked = _byte_decode(pack)

            i = 0
            for item in unpacked:
                unpacked[i] = item * 127
                i+=1
            print(f"SENT: {unpacked}")
            # ---------- DEBUG ----------
# ---------------------- SEND DATA ----------------------------------------------------

        if n >= 1e5:
            threads_should_stop = True
        
        current_time_us += 100
        # time.sleep(1e-4) # coresponds to 100 us of real time

        if threads_should_stop:
            t1.join()
            # t2.join()
            s1.close()
            s2.close()
            vehicle = None
            
            print("============================================")
            print("=                                          =")
            print("=                  END                     =")
            print("=                                          =")
            print("============================================")