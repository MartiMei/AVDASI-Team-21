#Importing various libraries

from pymavlink import mavutil 
from datetime import datetime
from collections import deque
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import time
import csv
import threading
import tkinter as tk
from tkinter import *
import numpy as np
from numpy import sin, cos, radians
from mpl_toolkits.mplot3d.art3d import Line3D,Poly3DCollection
from queue import Queue
import customtkinter as ctk
from PIL import Image

# -----------------------------
# Command Logging
# -----------------------------
cmd_log = open("cmd_log.txt","w",buffering=1) #add redundancy to fail by record per line
#could use "a" if we want to save as entire log that contains every time's cmd, need discuss in next meeting
ui_log = Queue() #use for display cmd log in ui
ui_state = Queue()

def cmd_logging(action, **kwargs):  #what;how(keyword argument)
    timestamp = datetime.now().strftime("%H:%M:%S") #hour;min;sec
    info = ", ".join([f"{k}={v}" for k, v in kwargs.items()])  
    line = f"[{timestamp}] {action}" #time and event

    if info:
        line += f" | {info}"  #no | when no event
    print(line)
    cmd_log.write(line + "\n")

    ui_log.put(line) 

# -----------------------------
# Safety Switch
# -----------------------------
def toggle_safety_switch(mav, enable):
    try:
        mav.mav.set_mode_send(
            mav.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_DECODE_POSITION_SAFETY,
            1 if enable else 0
        )
        print(f"Safety {'Enabled' if enable else 'Disabled'}.")
        return True
    except Exception as e:
        print(f"Error toggling safety switch: {e}")
        return False

def ui_toggle_safety():
    global safety_enabled
    safety_enabled = not safety_enabled

    toggle_safety_switch(master, enable=safety_enabled)

    safety_button.configure(
        text=f"Safety: {'ON' if safety_enabled else 'OFF'}",
        fg_color="green" if safety_enabled else "red",
        hover_color="darkgreen" if safety_enabled else "darkred"
    )

    cmd_logging(
        "SAFETY_TOGGLE",
        state="SAFE" if safety_enabled else "UNSAFE"
    )

# -----------------------------
# Arming Switch
# -----------------------------
armed_state = False

def toggle_arming_switch(mav, arm):
    global armed_state
    try:
        mav.mav.command_long_send(
            mav.target_system,
            mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1 if arm else 0,
            0, 0, 0, 0, 0, 0
        )
        armed_state = arm
        cmd_logging(
            "ARM_STATE_CHANGE",
            state="ARMED" if arm else "DISARMED"
        )
        print(f"Vehicle {'ARMED' if arm else 'DISARMED'}")
        return True
    except Exception as e:
        print(f"Error toggling arming switch: {e}")
        return False

# -----------------------------
# MAVLink connection
# -----------------------------

#WIFI Connection
connection_string = "udp:0.0.0.0:14550"
master = mavutil.mavlink_connection(connection_string)
master.wait_heartbeat()
time.sleep(1.5)


"""
#USB Connection
connection_string = "COM4"   # <-- change to your actual COM port
baud = 115200

master = mavutil.mavlink_connection(connection_string, baud=baud)

"""


toggle_arming_switch(master, arm=False)
toggle_safety_switch(master, enable=True)

safety_enabled = True
python_control = False
override_active = False


master.mav.rc_channels_override_send(
    master.target_system,
    master.target_component,
    0, 0, 0, 0, 0, 0, 0, 0
)
cmd_logging(
    "MAVLINK_CONNECTED (WIFI)",
    system_id=master.target_system,
    component_id=master.target_component
)
"""

cmd_logging(
    "MAVLINK_CONNECTED (USB)",
    system_id=master.target_system,
    component_id=master.target_component
)
"""
print(f"Heartbeat from system {master.target_system}, component {master.target_component}")

# Track last commanded servo positions
servo_pwm = {
    1: 1495,  # Aileron 1
    2: 1495,  # Elevator 1
    3: 1495,  # Aileron 2
    4: 1495,  # Rudder
    5: 1495,  # Elevator 2
}

#------------------
#Open file with the last stored pwm value of flap servo
#------------------

global flap_pwm,flap_value
flap_value=0

FLAP_PWM_FILE = "flap_position.txt"

def load_flap_pwm():
    with open(FLAP_PWM_FILE, "r") as f:
        value = int(f.read().strip())
        print(f"[INFO] Loaded flap PWM from file: {value}")
        return value

def save_flap_pwm(value):
    with open(FLAP_PWM_FILE, "w") as f:
        f.write(str(int(value)))
    print(f"[INFO] Saved flap PWM: {value}")

flap_pwm = load_flap_pwm()


recording = False
exit_program = False

def enable_emergency():
    global emergency
    emergency=1
def disable_emergency():
    global emergency
    emergency=0

disable_emergency()


# -----------------------------
# Servo control functions
# -----------------------------
def set_servo(channel, pwm):

    # Safety clamp
    if pwm < 1000:
        pwm = 1000
    if pwm > 2000:
        pwm = 2000

    servo_pwm[channel] = pwm

    #pwm = int(pwm)
    """
    Update stored PWM value for a MAIN output controlled via RC channel override.

    channel: RC channel number (1–8)
    pwm: PWM value in range 1000–2000 microseconds
    """

def reset_positions():
    set_servo(1, 1495)
    set_servo(2, 1380)
    set_servo(3, 1495)
    set_servo(4, 1495)
    set_servo(5, 1600)

def centre_sliders():
    rudder.set(1568)
    elevator.set(1380)
    aileron.set(1495)

def update_elevator(value):
    set_servo(2, float(value))
    set_servo(5, float(value))

def update_rudder(value):
    set_servo(4, float(value))

def update_aileron(value):
    set_servo(1, float(value))
    set_servo(3, float(value))
    

# -----------------------------
# Angle input control
# -----------------------------

center_pwm = 1495

def set_servo_angle():
    global last_flap_pwm 
    #Rudder: +-40 degrees (3.3.1.h)
    #Aileron: +-40 degrees (3.3.2.g)
    #Elevator: +-45 degrees (3.3.1.a) 

    if emergency==0:
        try:
            angle_aileron = float(aileron_angle_entry.get())
            angle_elevator = float(elevator_angle_entry.get())
            angle_rudder = float(rudder_angle_entry.get())
        except ValueError:
            tk.messagebox.showerror("Invalid Input!")
            return
        
        #CREATE EQUATION FOR AILERON DEFLCTION
        #angle_aileron = max(-40, min(40, angle_aileron))
        pwm_aileron = angle_aileron
        set_servo(1, pwm_aileron)
        set_servo(3, (3000-pwm_aileron))
        aileron.set(pwm_aileron)
            
            
        #CREATE EQUATION FOR ELEVATOR DEFLCTION
        angle_elevator = max(-45, min(45, angle_elevator))
        pwm_elevator = int(0.0594*(angle_elevator)**2-11.167*(angle_elevator)+1380)
        set_servo(2, pwm_elevator)
        set_servo(5, 3000-pwm_elevator)
        elevator.set(pwm_elevator)

        #CREATE EQUATION FOR RUDDER DEFLCTION
        angle_rudder = max(-40, min(40, angle_rudder))
        pwm_rudder = int(11.56*angle_rudder+1568)
        set_servo(4, pwm_rudder)
        rudder.set(pwm_rudder)

        #log angle change in command log
        cmd_logging(
        "SET_SERVO_ANGLE",
        aileron_deg=angle_aileron,
        elevator_deg=angle_elevator,
        rudder_deg=angle_rudder)

    elif emergency==1:
        
        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            int(m1.get()),
            int(m2.get()),
            int(m3.get()),
            int(m4.get()),
            int(m5.get()), 0, 0, 0
        )

        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            11,   #AUX OUT 3
            int(a3.get()),
            0, 0, 0, 0, 0)

        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            13,   #AUX OUT 5
            int(a5.get()),
            0, 0, 0, 0, 0)

        last_flap_pwm=int(str(a3.get())+str(a5.get()))

        cmd_logging(
        "EMERGENCY_SET_PWM")
            


# -----------------------------
# Max/min servo angle test
# -----------------------------   

def servo_test(channel, min_pwm, max_pwm, count=3, delay=1):
    
    #Twist between MAX/MIN for 3 times

    cmd_logging(
    "SERVO_TEST_START",
    channel=channel,
    min_pwm=min_pwm,
    max_pwm=max_pwm,
    cycles=count)

    def run_test():
        for _ in range(count):
            set_servo(channel, max_pwm)
            if channel == 1: aileron.set(max_pwm)
            if channel == 2: elevator.set(max_pwm)
            if channel == 3: aileron.set(max_pwm)
            if channel == 4: rudder.set(max_pwm)
            if channel == 5: elevator.set(max_pwm)
            time.sleep(delay)
            set_servo(channel, min_pwm)
            if channel == 1: aileron.set(min_pwm)
            if channel == 2: elevator.set(min_pwm)
            if channel == 3: aileron.set(min_pwm)
            if channel == 4: rudder.set(min_pwm)
            if channel == 5: elevator.set(min_pwm)
            time.sleep(delay)
        #Back to center
        center = center_pwm
        set_servo(channel, center)
        if channel == 1: aileron.set(center)
        if channel == 2: elevator.set(center)
        if channel == 2: aileron.set(center)
        if channel == 4: rudder.set(center)
        if channel == 5: elevator.set(center)

        cmd_logging(
            "SERVO_TEST_END",
            channel=channel,
            center_pwm=center_pwm
        )
    if emergency==0:
        threading.Thread(target=run_test, daemon=True).start()

#-----------------------
#Defining flap positions
#-----------------------

def cruise_position():
    global flap_value
    flap_value=1

def takeoff_position():
    global flap_value
    flap_value=2

def landing_position():
    global flap_value
    flap_value=3

def set_custom_flap():
    global flap_value
    flap_value=4

#-----------------------
#Defining variables to store flight parameters
#-----------------------

time_data = deque(maxlen=300)
pitch_data = deque(maxlen=300)
start_time = time.time()

airspeed_data = deque(maxlen=300)
airspeed_time = deque(maxlen=300)

servo1_data = deque(maxlen=300)
servo2_data = deque(maxlen=300)
servo4_data = deque(maxlen=300)
servo_time = deque(maxlen=300)

# -----------------------------
# MAVLink reading - reads data from the Cube
# -----------------------------

def mavlink_thread():
    global recording, exit_program, python_control, yaw_offset,flap_pwm,flap_ontarget,airspeed,last_flap_mode
    global last_flap_pwm, sensor_data,flap_switch_value,pot_voltage,servo1_pwm, servo2_pwm, servo4_pwm,armed_state,safety_enabled,override_active
    yaw = roll = pitch = airspeed = 0.0
    yaw_offset = None

    # --- RC switch setup ---
    SWITCH_THRESHOLD = 1500      # midpoint threshold for mode switch toggle (Python/RC)
    
    # --- Initial Values ---
    SWITCH_CHANNEL = 7           # switch to toggle RC vs Python control
    FLAP_SWITCH_CHANNEL = 6      # 3-position flap switch
    VOLTAGE_TOLERANCE = 0.03     # acceptable volatge difference
    SERVO_STEP = 1               # PWM increment of flap servo per loop (small increments)

    
    flap_pwm = load_flap_pwm()   # loading the previous PWM of the flap servo, to prevent vibrations of flap
    flap_pwm3 = int(str(flap_pwm)[:4])
    flap_pwm5 = int(str(flap_pwm)[-4:])
    flap_target_adc = None
    current_adc = 0.0            # potentiometer reading
    
    #initialise boolean variables
    flap_ontarget = False #be true when PWM reach required mode
    last_flap_mode = None
    flap_target_voltage = None

    off_log_message=False
    on_log_message=False

    while not exit_program:
        #Defining the type of messages that is sent from the Cube
        msg = master.recv_match(
            type=["ATTITUDE", "RC_CHANNELS","SERVO_OUTPUT_RAW","NAMED_VALUE_FLOAT","HEARTBEAT"],
            blocking=True, timeout=1
        )
        if not msg:
            continue
        
        # --- Store flight parameters in arrays ---
        if recording:
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            
            t = time.time() - start_time
            time_data.append(t)
            pitch_data.append(pitch)
            roll_data.append(roll)
            yaw_data.append(yaw)
            pitchrate_data.append(pitch_rate)
            airspeed_data.append(pot_voltage)

            servo1_data.append(servo1_pwm)
            servo2_data.append(servo2_pwm)
            servo4_data.append(servo4_pwm)

        # --- Recieving Heartbeat message --- 
        if msg.get_type() == "HEARTBEAT":
            armed_state = bool(
            msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )
            
        # --- Handle attitude (yaw, pitch, roll) ---
        if msg.get_type() == "ATTITUDE":
            
            roll = msg.roll * 57.2958
            pitch = msg.pitch * 57.2958
            yaw_raw = msg.yaw * 57.2958
            pitch_rate = msg.pitchspeed * 57.2958
            
   
            if yaw_offset is None:
                yaw_offset = yaw_raw
            yaw = yaw_raw - yaw_offset

            if yaw > 180:
                yaw -= 360
            elif yaw < -180:
                yaw += 360
            
            # Update attitude label in UI, to 1 decimal place
            attitude_text.set(f"Pitch: {pitch:.1f} ° | Roll: {roll:.1f}° | Yaw: {yaw:.1f}")

        
        # --- Handle RC input (for control switch, Python/RC) ---
        if msg.get_type() == "RC_CHANNELS":
            switch_value = msg.chan7_raw
            flap_switch_value = msg.chan6_raw
            log_channel = msg.chan10_raw
            
            #Creating logging message in the command log
            if log_channel < 1400:
                if not off_log_message:
                    cmd_logging("LOGGING OFF")
                    status_text.set("Status: NOT LOGGING")
                    off_log_message = True
                    on_log_message = False
            elif log_channel > 1600:
                if not on_log_message:
                    cmd_logging("LOGGING ON")
                    status_text.set("Status: LOGGING")
                    on_log_message = True
                    off_log_message = False


            new_python_control = switch_value > 1500
            
            #Creating control mode message in the command log
            if new_python_control != python_control:
                old_mode = "PYTHON" if python_control else "RC"
                new_mode = "PYTHON" if new_python_control else "RC"
                print(f"[INFO] Control mode changing: {old_mode} -> {new_mode}")
                
                python_control = new_python_control

                if python_control:
                    cmd_logging("CONTROL_MODE_CHANGE", mode="PYTHON")
                    ui_state.put(("CONTROL_MODE", "Mode: PYTHON Control"))
                    print("[INFO] Sending initial neutral override")
                    #Resetting postitions of servos if there is a change in control mode
                    master.mav.rc_channels_override_send(
                        master.target_system,
                        master.target_component,
                        1495, 1380, 1495, 1495, 1600, 0, 0, 0
                    )
                    override_active = True
                    
                else:
                    cmd_logging("CONTROL_MODE_CHANGE", mode="RC")
                    ui_state.put(("CONTROL_MODE", "Mode: RC Control"))
                    print("[INFO] Releasing control to RC")
                    master.mav.rc_channels_override_send(
                        master.target_system,
                        master.target_component,
                        0, 0, 0, 0, 0, 0, 0, 0
                    )
                    override_active = False
                
                print(f"[INFO] Control mode changed to: {'PYTHON' if python_control else 'RC'}")

                
        # --- Recieving messages of the current PWM value of aileron/elevator servo ---
        elif msg.get_type() == "SERVO_OUTPUT_RAW":
            servo1_pwm = msg.servo1_raw
            servo2_pwm = msg.servo2_raw
            servo4_pwm = msg.servo4_raw

            #Displaying the live angles of each control surface on the UI
            aileron_text.set(f"Aileron angle: {servo1_pwm:.1f}°")
            elevator_text.set(f"Elevator angle: {servo2_pwm:.1f}°")
            #elevator_text.set(f"Elevator angle: {elevator_angle:.1f}°")
            rudder_text.set(f"Rudder angle: {servo4_pwm:.1f}°")
            #rudder_text.set(f"Rudder angle: {rudder_angle:.1f}°")

            #CHANGE THIS TO ANGLE!!! (NOT PWM)
            #CALIBRATION TABLE!

            
        # --- Recieving the current volatge of the potentiometer ---
        elif msg.get_type() == "NAMED_VALUE_FLOAT":
            name_raw = msg.name
            if isinstance(name_raw, bytes):
                name = name_raw.decode("ascii").rstrip("\x00")
            else:
                name = name_raw.rstrip("\x00")

            if name == "POT_VOLT":
                pot_voltage = msg.value
                #Display the angle of flap of flap
                voltage_text.set(f"Flap angle: {pot_voltage:.2f}V")
                #CHANGE THIS TO ANGLE
                #CALIBRATION TABLE!

            # --- Flap control ---
            
            if python_control:
                ui_state.put(("ANGLE_BUTTON", "normal"))
                if emergency==0:
                    if flap_value == 0:
                        flap_target_voltage=0
                    elif flap_value == 1:
                        #CHNAGE THESE VOLTAGE TARGETS, CALIBRATED TO WING
                        flap_mode = "UP"
                        flap_target_voltage = 1.48
                    elif flap_value == 2:
                        flap_mode = "TAKEOFF"
                        flap_target_voltage = 1.24
                    elif flap_value == 3:
                        flap_mode = "LANDING"
                        flap_target_voltage = 1.01
                    elif flap_value==4:
                        #CREATE EQUATION FOR FLAP DEFLCTION
                        flap_mode = "CUSTOM"
                        entered_flap_angle = float(flap_angle_entry.get())
                        flap_target_voltage = -0.0156*entered_flap_angle + 1.48

                    #Redundancy code to prevent errors
                    if flap_target_voltage==0:
                        continue 
                
                    #Flap logging to command log
                    if flap_mode != last_flap_mode:
                        cmd_logging(
                        "FLAP_MODE_CHANGE",
                        mode=flap_mode,
                        target_voltage=flap_target_voltage)
                    last_flap_mode = flap_mode
                    
                    #Moving the servo in small incraments until the desired voltage is reached
                    
                    #PORT
                    if pot_voltage < flap_target_voltage - VOLTAGE_TOLERANCE:
                        flap_pwm3 -= SERVO_STEP
                        flap_pwm5 += SERVO_STEP
                        flap_ontarget=False
                    elif pot_voltage > flap_target_voltage + VOLTAGE_TOLERANCE:
                        flap_pwm3 += SERVO_STEP
                        flap_pwm5 -= SERVO_STEP
                        flap_ontarget=False
                    else:
                        if flap_ontarget==False:
                            #Command logging to indicate the desired flap angle is reached
                            cmd_logging(
                            "FLAP_REACHED",
                            mode=flap_mode,
                            voltage=round(pot_voltage, 2))
                            last_flap_pwm=int(str(flap_pwm3)+str(flap_pwm5))
                        flap_ontarget=True
                    
                    flap_pwm3 = max(1000, min(2000, flap_pwm3))
                    flap_pwm5 = max(1000, min(2000, flap_pwm5))
                    
                    #Send command to 2 servos (as there are 2 flap servos)
                    master.mav.command_long_send(
                        master.target_system,
                        master.target_component,
                        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                        0,
                        11,   #AUX OUT 3
                        flap_pwm3,
                        0, 0, 0, 0, 0)

                    master.mav.command_long_send(
                        master.target_system,
                        master.target_component,
                        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                        0,
                        13,   #AUX OUT 5
                        flap_pwm5,
                        0, 0, 0, 0, 0)


                

            else:
                ui_state.put(("ANGLE_BUTTON", "disabled"))
                
        # --- Control of servos using the sliders on the UI ---
        if python_control and not safety_enabled:
            if emergency==0:
                master.mav.rc_channels_override_send(
                    master.target_system,
                    master.target_component,
                    int(aileron.get()),
                    (int(elevator.get()),
                    (int(3000-aileron.get())),
                    int(rudder.get()),
                    (int(3000-elevator.get()), 0, 0, 0
                )
            
            override_active = True
            
        elif safety_enabled:
            master.mav.rc_channels_override_send(
                master.target_system,
                master.target_component,
                1495, 1380, 1495, 1495, 1600
                , 0, 0, 0
            )
            override_active = True
            
        else:
            master.mav.rc_channels_override_send(
                master.target_system,
                master.target_component,
                0, 0, 0, 0, 0, 0, 0, 0
            )
            override_active = False

    #When exiting program, set all the port outputs to 0
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print("[INFO] Logging stopped, overrides released, file closed.")

# --- Function to quit program ---
def quit_program():
    #Command logging
    cmd_logging(
        "PROGRAM_QUIT",
        mode="PYTHON" if python_control else "RC",
        last_flap_pwm=last_flap_pwm
    )
    #Setting ports to be zero
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        0, 0, 0, 0, 0, 0, 0, 0
    )

    global exit_program
    exit_program = True

    #Save the last known flap PWM before quitting
    try: save_flap_pwm(last_flap_pwm)
    except: pass
    
    #Close the UI
    root.destroy()

# --- This function updates the state of buttons on the UI ---
def update_state():
    while not ui_state.empty():
        key, value = ui_state.get_nowait()
        if key == "CONTROL_MODE":
            control_mode_text.set(value)
        elif key == "ANGLE_BUTTON":
            try:
                if value == "disabled":
                    angle_set_button.config(state="disabled")
                elif value == "normal":
                    angle_set_button.config(state="normal")
            except NameError:
                pass
        elif key == "SAFETY_BUTTON_UPDATE":
            safety_button.configure(
                text=value,
                fg_color="green" if safety_enabled else "red",
                hover_color="darkgreen" if safety_enabled else "darkred"
            )
    
    if not exit_program:
        root.after(100, update_state) 

# -----------------------------
# Tkinter GUI setup
# -----------------------------

program = ctk.CTk()
program.title("Avionics")
program.geometry("1500x1000")

ctk.set_appearance_mode("light")
#ctk.set_default_color_theme("green")

root = ctk.CTkScrollableFrame(
    program,
    width=1500,
    height=1200,
    fg_color="white",
    border_width=2,
    border_color="black",
    
)
root.pack(fill="both", expand=True,padx=10,pady=10)

#--------
#CONTROL
#--------

info_frame=ctk.CTkFrame(root,fg_color="white",
                        border_width=2,
                        border_color="black")
info_frame.grid(row=0,column=1)

Label(info_frame, text="CONTROL", font="calibri 18 bold italic underline").grid(row=0, column=1, padx=10, pady=10,sticky="n")

safety_button = ctk.CTkButton(
    info_frame, 
    text="Safety: ON" if safety_enabled else "Safety: OFF",
    command=ui_toggle_safety,
    fg_color="green" if safety_enabled else "red",
    text_color="black",
    hover_color="darkgreen" if safety_enabled else "darkred",
    width=80
)
safety_button.grid(row=1, column=0, padx=10, pady=10, sticky="e")

quit_button = ctk.CTkButton(info_frame, text="Quit", command=quit_program,fg_color="red",text_color="black",hover_color="red",width=80)
quit_button.grid(row=1, column=0, padx=10, pady=10,sticky="w")

#--------
#LIVE INFORMATION
#--------

plot_frames=ctk.CTkFrame(info_frame)
plot_frames.grid(row=0,column=0,padx=10,pady=10,sticky="s")

Label(plot_frames, text="LIVE DATA", font="calibri 18 bold italic underline").grid(row=0, column=0, padx=10, pady=10)

attitude_text = StringVar(value="")
Label(plot_frames, textvariable=attitude_text, font=("Arial", 16)).grid(row=1, column=0, padx=10, pady=10)

voltage_text = StringVar(value="Flap angle: °")
Label(plot_frames, textvariable=voltage_text, font=("Arial", 16)).grid(row=2, column=0, padx=10, pady=10)

aileron_text = StringVar(value="Aileron angle: 0°")
Label(plot_frames, textvariable=aileron_text, font=("Arial", 16)).grid(row=3, column=0, padx=10, pady=10)

elevator_text = StringVar(value="Elevator angle: 0°")
Label(plot_frames, textvariable=elevator_text, font=("Arial", 16)).grid(row=4, column=0, padx=10, pady=10)

rudder_text = StringVar(value="Rudder angle: 0°")
Label(plot_frames, textvariable=rudder_text, font=("Arial", 16)).grid(row=5, column=0, padx=10, pady=10)

#--------
#USER INPUTS
#--------

#Angle input set_servo_angle
angle_frame = ctk.CTkFrame(info_frame)
angle_frame.grid(row=0, column=1, padx=10, pady=10) 

# Aileron
Label(angle_frame, text="Aileron (°):").grid(row=0, column=0, padx=5, pady=5, sticky="e")
aileron_angle_entry = Entry(angle_frame, width=5)
aileron_angle_entry.grid(row=0, column=1, padx=5, pady=5)
aileron_angle_entry.insert(0, "0")
Button(angle_frame, text="Test", command=lambda: servo_test(1, 1100, 1900)).grid(row=0, column=2, padx=10, pady=5, sticky="n") 

# Elevator
Label(angle_frame, text="Elevator (°):").grid(row=1, column=0, padx=5, pady=5, sticky="e")
elevator_angle_entry = Entry(angle_frame, width=5)
elevator_angle_entry.grid(row=1, column=1, padx=5, pady=5)
elevator_angle_entry.insert(0, "0")
Button(angle_frame, text="Test", command=lambda: servo_test(2, 1100, 1900)).grid(row=1, column=2, padx=10, pady=5)

# Rudder
Label(angle_frame, text="Rudder (°):").grid(row=2, column=0, padx=5, pady=5, sticky="e")
rudder_angle_entry = Entry(angle_frame, width=5)
rudder_angle_entry.grid(row=2, column=1, padx=5, pady=5)
rudder_angle_entry.insert(0, "0")
Button(angle_frame, text="Test", command=lambda: servo_test(4, 1100, 1900)).grid(row=2, column=2, padx=10, pady=5)

# Set angle button
angle_set_button = Button(angle_frame, text="Set Angles", command=set_servo_angle)
angle_set_button.grid(row=3, column=1, pady=10)

#Flap control frame
flap_frame=ctk.CTkFrame(info_frame)
flap_frame.grid(row=0, column=1, padx=10, pady=10,sticky="s",rowspan=2)

Label(flap_frame, text="Flap Control", font="calibri 14 bold italic underline").grid(row=0, column=1, padx=10, pady=10,sticky="n")

cruise=Button(flap_frame, text="CRUISE", command=cruise_position)
cruise.grid(row=1, column=0,padx=10, pady=10)

takeoff=Button(flap_frame, text="TAKEOFF", command=takeoff_position)
takeoff.grid(row=1, column=1,padx=10, pady=10)

landing=Button(flap_frame, text="LANDING", command=landing_position)
landing.grid(row=1, column=2,padx=10, pady=10)

Label(flap_frame, text="Custom angle (°):").grid(row=2,column=0,padx=5, pady=5)
flap_angle_entry=Entry(flap_frame,width=5)
flap_angle_entry.grid(row=2,column=1,padx=5, pady=5)
flap_angle_entry.insert(0, "0")
set_flap_angle=Button(flap_frame, text="Set Angle",command=set_custom_flap)
set_flap_angle.grid(row=2, column=2)


#RC Style Sliders

slider_frame=ctk.CTkFrame(info_frame)
slider_frame.grid(row=2,column=1,sticky="n")

rudder = tk.Scale(slider_frame, from_=950, to=2050, orient="horizontal", command=update_rudder, length=250, showvalue=False)
rudder.set(1500)
rudder.grid(row=1, column=0, padx=10, pady=10)

elevator = tk.Scale(slider_frame, from_=950, to=2050, orient="vertical", command=update_elevator, length=250, showvalue=False)
elevator.set(1500)
elevator.grid(row=0, column=1, padx=10, pady=10)

aileron = tk.Scale(slider_frame, from_=950, to=2050, orient="horizontal", command=update_aileron, length=250, showvalue=False)
aileron.set(1500)
aileron.grid(row=1, column=1, padx=10, pady=10)

recentre = ctk.CTkButton(slider_frame, text="Re-centre controls", command=centre_sliders,fg_color="black")
recentre.grid(row=0, column=0, padx=10, pady=10)

#--------
#COMMAND LOG DISPLAY
#--------

log_frame = Frame(info_frame)
log_frame.grid(row=2, column=2, columnspan=2,padx=10,pady=10,sticky="nsew")
Label(log_frame, text="Command Log").pack(anchor="w")

log_text = Text(
    log_frame,
    height=10,
    width=100,
    state="disabled",
    wrap="none"
)

log_text.pack(side="left",fill="both",expand=True)
log_scroll = Scrollbar(log_frame, command=log_text.yview) 
log_text.config(yscrollcommand=log_scroll.set) #combine text and scrolbar

#STATUS

Label(info_frame, text="STATUS", font="calibri 18 bold italic underline").grid(row=0, column=2, padx=10, pady=10,sticky="n")

status_text = StringVar(value="Status: NOT LOGGING")
Label(info_frame, textvariable=status_text, font=("Arial", 12)).grid(row=0, column=2, padx=10, pady=10)

control_mode_text = StringVar(value="Mode: RC Control")
Label(info_frame, textvariable=control_mode_text, font=("Arial", 12)).grid(row=0, column=2, padx=10, pady=10,sticky="s")


#COMPANY LOGO

image = Image.open("logo.jpg")
ctk_image = ctk.CTkImage(
    light_image=image,
    dark_image=image,
    size=(250, 250)   # resize here
)

# --------------------------------
# Display image in frame
# --------------------------------
image_label = ctk.CTkLabel(
    info_frame,
    image=ctk_image,
    text=""   # IMPORTANT: remove text spacing
)
image_label.grid(row=0,column=3)

#toggle_arming_switch(mav, arm=True)

#Port/Starboard wing selection
wing_frame=Frame(info_frame)
wing_frame.grid(row=1,column=3,padx=10, pady=10)

Label(wing_frame,text="WING SELECTION",font="calibri 14 bold italic underline").grid(row=0,column=0)
global wing
wing=IntVar()
port=Radiobutton(wing_frame,variable=wing,value=1,text="Port").grid(row=1,column=0)
starboard=Radiobutton(wing_frame,variable=wing,value=2,text="Starboard").grid(row=1,column=1)

# -----------------------------
# Emergency mode
# -----------------------------

def enable_emergency():
    global emergency
    emergency=1
    emergency_status.set("Active")
    
def disable_emergency():
    global emergency
    emergency=0
    emergency_status.set("")
        
Label(info_frame, text="").grid(row=3, column=0, padx=10, pady=10)

emergency_frame = Frame(info_frame)
emergency_frame.grid(row=4, column=0,columnspan=2,padx=10,pady=10,sticky="w")
Label(emergency_frame, text="Emergency mode", font="calibri 18 bold italic underline").grid(row=0, column=0, padx=10, pady=10)



enable_emergency_button = ctk.CTkButton(emergency_frame,command=enable_emergency,text="Enable",fg_color="green",text_color="black")
enable_emergency_button.grid(row=0, column=1, padx=10, pady=10,sticky="w")

disable_emergency_button = ctk.CTkButton(emergency_frame,command=disable_emergency,text="Disable",fg_color="red",text_color="black")
disable_emergency_button.grid(row=0, column=2, padx=10, pady=10,sticky="e")

Label(emergency_frame, text="MAIN OUT 1 (AILERON 1)").grid(row=1,column=0)
Label(emergency_frame, text="MAIN OUT 2 (ELEVATOR 1)").grid(row=2,column=0)
Label(emergency_frame, text="MAIN OUT 3 (AILERON 2)").grid(row=3,column=0)
Label(emergency_frame, text="MAIN OUT 4 (RUDDER)").grid(row=4,column=0)
Label(emergency_frame, text="MAIN OUT 5 (ELEVATOR 2)").grid(row=5,column=0)
Label(emergency_frame, text="AUX OUT 3 (FLAP 1)").grid(row=6,column=0)
Label(emergency_frame, text="AUX OUT 5 (FLAP 2)").grid(row=7,column=0)

m1=Entry(emergency_frame,width=10)
m1.grid(row=1,column=1,padx=5, pady=5)
m1.insert(0, "1500")
m2=Entry(emergency_frame,width=10)
m2.grid(row=2,column=1,padx=5, pady=5)
m2.insert(0, "1500")
m3=Entry(emergency_frame,width=10)
m3.grid(row=3,column=1,padx=5, pady=5)
m3.insert(0, "1500")
m4=Entry(emergency_frame,width=10)
m4.grid(row=4,column=1,padx=5, pady=5)
m4.insert(0, "1500")
m5=Entry(emergency_frame,width=10)
m5.grid(row=5,column=1,padx=5, pady=5)
m5.insert(0, "1500")
a3=Entry(emergency_frame,width=10)
a3.grid(row=6,column=1,padx=5, pady=5)
a3.insert(0, "1500")
a5=Entry(emergency_frame,width=10)
a5.grid(row=7,column=1,padx=5, pady=5)
a5.insert(0, "1500")

emergency_status=StringVar(value="")
emergency_status_label=Label(emergency_frame,textvariable=emergency_status,font=("Arial", 18),fg="red")
emergency_status_label.grid(row=2, column=2, padx=10, pady=10)

set_pwm = ctk.CTkButton(emergency_frame,command=set_servo_angle,text="SET PWM",fg_color="grey",text_color="black")
set_pwm.grid(row=3, column=2, padx=10, pady=10)



# -----------------------------
# 3D attitude visualization
# -----------------------------
fig3 = plt.figure(figsize=(3, 3))
ax3 = fig3.add_subplot(111, projection='3d')
ax3.set_title("3D Attitude")
ax3.set_xlim([-1, 1])
ax3.set_ylim([-1, 1])
ax3.set_zlim([-1, 1])
ax3.set_xlabel("X")
ax3.set_ylabel("Y")
ax3.set_zlabel("Z")
ax3.grid(True)  #add mash ---improvement to graph
ax3.view_init(elev=20, azim=-60)

arrow_len = 0.5  
ax3.quiver(0, 0, 0, arrow_len, 0, 0, color='r', arrow_length_ratio=0.1, linewidth=1.5)
ax3.quiver(0, 0, 0, 0, arrow_len, 0, color='g', arrow_length_ratio=0.1, linewidth=1.5)
ax3.quiver(0, 0, 0, 0, 0, arrow_len, color='b', arrow_length_ratio=0.1, linewidth=1.5)

ax3.text(arrow_len, 0, 0, 'Roll-axis', color='c')
ax3.text(0, arrow_len, 0, 'Pitch-axis', color='c')
ax3.text(0, 0, arrow_len, 'Yaw-axis', color='c')

plane_parts = {}

#new plane-like body
plane_parts['body'] = Poly3DCollection([], facecolor='red', edgecolor='black', linewidths=0.5)
ax3.add_collection3d(plane_parts['body'])

plane_parts['wing'] = Poly3DCollection([], facecolor='blue', edgecolor='black', linewidths=0.5)
ax3.add_collection3d(plane_parts['wing'])

plane_parts['tail_h'] = Poly3DCollection([], facecolor='green', edgecolor='black', linewidths=0.5)
ax3.add_collection3d(plane_parts['tail_h'])

plane_parts['tail_v'] = Poly3DCollection([], facecolor='green', edgecolor='black', linewidths=0.5)
ax3.add_collection3d(plane_parts['tail_v'])

def update_3d_view(roll, pitch, yaw):
    #Convert to radians
    r = radians(roll)
    p = -1*radians(pitch)
    y = radians(yaw)

    #Rotation
    Rz = np.array([
        [cos(y), -sin(y), 0],
        [sin(y),  cos(y), 0],
        [0,       0,      1]
    ])
    Ry = np.array([
        [cos(p), 0, sin(p)],
        [0,      1, 0],
        [-sin(p), 0, cos(p)]
    ])
    Rx = np.array([
        [1, 0, 0],
        [0, cos(r), -sin(r)],
        [0, sin(r),  cos(r)]
    ])

    #Yaw * Pitch * Roll
    R = Rz @ Ry @ Rx

    body_pts = np.array([
        [0, -0.05, -0.05],
        [1, -0.05, -0.05],
        [1, 0.05, -0.05],
        [0, 0.05, -0.05],
        [0, -0.05, 0.05],
        [1, -0.05, 0.05],
        [1, 0.05, 0.05],
        [0, 0.05, 0.05]
    ]).T
    body_rot = R @ body_pts
    faces_body = [
        [body_rot[:,0], body_rot[:,1], body_rot[:,2], body_rot[:,3]],
        [body_rot[:,4], body_rot[:,5], body_rot[:,6], body_rot[:,7]],
        [body_rot[:,0], body_rot[:,1], body_rot[:,5], body_rot[:,4]],
        [body_rot[:,2], body_rot[:,3], body_rot[:,7], body_rot[:,6]],
        [body_rot[:,1], body_rot[:,2], body_rot[:,6], body_rot[:,5]],
        [body_rot[:,4], body_rot[:,7], body_rot[:,3], body_rot[:,0]],
    ]
    plane_parts['body'].set_verts(faces_body)

    wing_pts = np.array([
        [0.3, -0.8, -0.025],
        [0.3, 0.8, -0.025],
        [0.3, 0.8, 0.025],
        [0.3, -0.6, 0.025]
    ]).T
    wing_rot = R @ wing_pts
    faces_wing = [[wing_rot[:,0], wing_rot[:,1], wing_rot[:,2], wing_rot[:,3]]]
    plane_parts['wing'].set_verts(faces_wing)

    tail_h_pts = np.array([
        [-0.2, -0.2, 0],
        [-0.2, 0.2, 0],
        [0, 0.2, 0],
        [0, -0.2, 0]
    ]).T
    tail_h_rot = R @ tail_h_pts
    plane_parts['tail_h'].set_verts([[tail_h_rot[:,0], tail_h_rot[:,1], tail_h_rot[:,2], tail_h_rot[:,3]]])

    tail_v_pts = np.array([
        [-0.2, 0, -0.05],
        [0, 0, -0.05],
        [0, 0, 0.05],
        [-0.2, 0, 0.05]
    ]).T
    tail_v_rot = R @ tail_v_pts
    plane_parts['tail_v'].set_verts([[tail_v_rot[:,0], tail_v_rot[:,1], tail_v_rot[:,2], tail_v_rot[:,3]]])

    canvas3.draw_idle()


canvas3 = FigureCanvasTkAgg(fig3, master=info_frame)
canvas3.get_tk_widget().grid(row=2,column=0,sticky="n")

#empty=ctk.CTkLabel(info_frame,text="",bg_color="white").grid(row=3, column=0)



# Create buffers for data
time_data = deque(maxlen=300)
pitch_data = deque(maxlen=300)
roll_data = deque(maxlen=300)
yaw_data = deque(maxlen=300)
pitchrate_data = deque(maxlen=300)
start_time = time.time()

# Scrolling window length (seconds)m

time_window = 10

# -----------------------------
# Start threads and GUI
# -----------------------------
reset_positions()
centre_sliders()
flap_pwm = load_flap_pwm()
set_servo(11, flap_pwm)

# Run calibration sweep ignoring potentiometer
thread = threading.Thread(target=mavlink_thread, daemon=True)
thread.start()
"""
def update_3d_view(roll, pitch, yaw):
    # Convert to radians
    r = radians(roll)
    p = -1*radians(pitch)
    y = radians(yaw)

    # Rotation matrices
    Rz = np.array([
        [cos(y), -sin(y), 0],
        [sin(y),  cos(y), 0],
        [0,       0,      1]
    ])
    Ry = np.array([
        [cos(p), 0, sin(p)],
        [0,      1, 0],
        [-sin(p), 0, cos(p)]
    ])
    Rx = np.array([
        [1, 0, 0],
        [0, cos(r), -sin(r)],
        [0, sin(r),  cos(r)]
    ])

    # Yaw * Pitch * Roll
    R = Rz @ Ry @ Rx

    # Transform each line endpoint
    nose = R @ np.array([1, 0, 0])
    wingL = R @ np.array([0, -1, 0])
    wingR = R @ np.array([0, 1, 0])
    tailT = R @ np.array([0, 0, 0.5])
    tailB = R @ np.array([0, 0, -0.5])

    # Update lines
    body_line.set_data([0, nose[0]], [0, nose[1]])
    body_line.set_3d_properties([0, nose[2]])

    wing_line.set_data([wingL[0], wingR[0]], [wingL[1], wingR[1]])
    wing_line.set_3d_properties([wingL[2], wingR[2]])

    tail_line.set_data([tailB[0], tailT[0]], [tailB[1], tailT[1]])
    tail_line.set_3d_properties([tailB[2], tailT[2]])

    canvas3.draw_idle()
"""
# GUI update plot function
def update_plot():
    if len(time_data) > 1:
        # --- Main time plot ---
        line_pitch.set_data(time_data, pitch_data)
        line_roll.set_data(time_data, roll_data)
        line_yaw.set_data(time_data, yaw_data)
        line_pitchrate.set_data(time_data, pitchrate_data)

        line_airspeed.set_data(time_data, airspeed_data)

        servo1_line.set_data(time_data, servo1_data)
        servo2_line.set_data(time_data, servo2_data)
        servo4_line.set_data(time_data, servo4_data)

        t_max = time_data[-1]
        t_min = max(0, t_max - time_window)
        ax.set_xlim(t_min, t_max)
        ax_airspeed.set_xlim(t_min, t_max)
        servo_ax.set_xlim(t_min, t_max)
        
        canvas.draw_idle()
        canvas_airspeed.draw_idle()
        servo_canvas.draw_idle()

    if not exit_program:
        root.after(100, update_plot)
    """
    update_3d_view(roll_data[-1] if roll_data else 0,
               pitch_data[-1] if pitch_data else 0,
               yaw_data[-1] if yaw_data else 0)
    """

def update_cmd_log():
    while not ui_log.empty():
        line = ui_log.get_nowait()
        log_text.config(state="normal")
        log_text.insert("end",line + "\n")
        log_text.see("end") #auto scroll
        log_text.config(state="disabled")

    if not exit_program:
            root.after(100,update_cmd_log)

update_plot()
update_cmd_log()
update_state()


program.mainloop()

#        ┏┓　　　┏┓+ +
#　　　┏┛┻━━━┛┻┓ + +
#　　　┃　　　　　　　┃ 　
#　　　┃　　　━　　　┃ ++ + + +
#　　 ████━████ ┃+
#　　　┃　　　　　　　┃ +
#　　　┃　　　┻　　　┃
#　　　┃　　　　　　　┃ + +
#　　　┗━┓　　　┏━┛
#　　　　　┃　　　┃　　　　　　　　　　　
#　　　　　┃　　　┃ + + + +
#　　　　　┃　　　┃　　　　Codes are far away from bugs with the animal protecting　　　
#　　　　　┃　　　┃ + 　　　
#　　　　　┃　　　┃
#　　　　　┃　　　┃　　+　　　　　　　　　
#　　　　　┃　 　 ┗━━━┓ + +
#　　　　　┃ 　　　　　　　┣┓
#　　　　　┃ 　　　　　　　┏┛
#　　　　　┗┓┓┏━┳┓┏┛ + + + +
#　　　　　　┃┫┫　┃┫┫
#　　　　　　┗┻┛　┗┻┛+ + + +
