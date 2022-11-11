import image, math, pyb, sensor, struct, time, utime,sys,tf
from pyb import Pin
import micropython

################################################################################
###                        MAVLINK PROTOCOL CONSTANTS                        ###
################################################################################

MAVLINK_MSG_ID_COMMAND_LONG = 76
MAVLINK_MSG_ID_COMMAND_LONG_confirmation = 0  # Just send one time
MAVLINK_MSG_ID_COMMAND_LONG_CRC = 152
MAV_CMD_COMPONENT_ARM_DISARM = 400

MAV_LANDING_TARGET_message_id = 149
MAV_LANDING_TARGET_frame = 2 # MAV_FRAME_BODY_NED
MAV_LANDING_TARGET_extra_crc = 200

MAV_CMD_DO_SET_MODE = 176

MAV_MODE_1= 217
MAV_MODE_2 = 209

MAV_PX4MODE_AUTO = 4
MAV_PX4MODE_OFFBOARD = 6

MAV_PX4SUBMODE_TAKEOFF = 2
MAV_PX4SUBMODE_LAND = 6

MAV_PIXHAWK_ID = 1 # SYSID_THISMAV for Pixhawk
MAV_OPENMV_ID = 255 # Viewed as GCS
MAV_COMPONENT_ID = 0 # 0 for broadcast; others see MAV_COMPONENT enum

MAVLINK_CRC_EXTRA_HEARTBEAT = 50

MAVLINK_CHANNEL_OVERRIDE_ID = 70
MAVLINK_CHANNEL_OVERRIDE_CRC = 124

################################################################################
###                     CAMERA SETTING AND MISCELLANEOUS                     ###
################################################################################

micropython.alloc_emergency_exception_buf(100) # For OpenMV Cam debug on the IDE
uart_baudrate = 115200
blue_led = pyb.LED(3)
green_led = pyb.LED(2)

# The timer is used to send keypoint commands to the drone with a predefined
# frequency. This frequency should not be at least 2 Hz.
tim = pyb.Timer(3, freq = 5)    # Timer 3 of the OpenMV Cam set to 5 Hz frequency clock          
arm = 0                         # Drone initialized to disarmed

# These are MAVLINK commands that have to be initialized to something in order
# to avoid errors
temp_key = b'\xfe\x1e\x02\x01\x00\x95\x00\x00\x00\x00\xf8\r\x00\x00\x00@\x00\x00\x00@\x00\x00\x00@\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00%\xf8'
temp_arm = b'\xfe!\x00\x01\x00L\x00\x00\x80?\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x90\x01\x01\x00\x00\x92\x92'

################# Camera settings #################
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)   
sensor.skip_frames(time = 100)
clock = time.clock()
img = sensor.snapshot()      # Acquisition of the image
extra_fb = sensor.alloc_extra_fb(sensor.width(), sensor.height(), sensor.RGB565)

model = tf.load("landing_quant.tflite", load_to_fb = True) # Importing the model

uart = pyb.UART(3, uart_baudrate, timeout_char = 10)  #Using UART3 of the OpenMV Cam

packet_sequence = 0

################################################################################
###                     MESSAGE CONSTRUCTION FUNCTIONS                       ###
################################################################################

########################################################################################
# Checksum definition for the bytearray transmission of the messages via UART

def checksum(data, extra):
    output = 0xFFFF
    for i in range(len(data)):
        tmp = data[i] ^ (output & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        output = ((output >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    tmp = extra ^ (output & 0xFF)
    tmp = (tmp ^ (tmp << 4)) & 0xFF
    output = ((output >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return output
########################################################################################

#Send the packet to arm the drone
def send_arm_packet(unused_input):
    global packet_sequence
    global arm
    global temp_arm
    #Sets up a scheduling for the arm function so that the function arm_packet_build 
    #is executed as soon as possible. It's necessary to pass an argument to the function
    #even if it's not used, otherwise the program raises an error.
    micropython.schedule(arm_packet_build,"_")  
    packet_sequence += 1
    uart.write(temp_arm)

def arm_packet_build(unused_input):
    global temp_arm
    global arm
    if not arm:     # If the drone is armed then it's disarmed in vice versa
        arm = 1
    else:
        arm = 0

    temp_arm = struct.pack("<fffffffhbbb",
                       arm, # 1 to arm, 0 to disarm
                       0,
                       0,
                       0,
                       0,
                       0,
                       0,
                       400, #MAV_CMD id
                       1, #SYSID_THISMAV for Pixhawk; 255 for GCS
                       0, # 0 for broadcast; others see MAV_COMPONENT enum
                       0)

    temp_arm = struct.pack("<bbbbb33s",
                       33,
                       packet_sequence & 0xFF,
                       1,
                       0,
                       76,
                       temp_arm)
    temp_arm = struct.pack("<b38sh",
                       0xFE,
                       temp_arm,
                       checksum(temp_arm, 152))

###################################################################################################

# Sets a keypoint that can be in poisition or in velocity. If the "pos" mode is active, then the
# drone will reach the keypoint (x,y,z) set by the global variable "keypoint". Otherwise, if the "vel" mode 
# is active, the drone will use the keypoint tuple to set the velocy of the drone to the (x,y,z) values of 
# the variable keypoint. 
def set_keypoint(unused_input):
    global packet_sequence
    global temp_key
    micropython.schedule(set_keypoint_build,"_")
    packet_sequence += 1
    uart.write(temp_key)

def set_keypoint_build(unused_input):
    global keypoint
    global temp_key
    global mode
    if mode == 'pos':
        temp_key = struct.pack("<lfffffffffffhbbb",
                           10,
                           keypoint[0],
                           keypoint[1],
                           keypoint[2],
                           0,
                           0,
                           0,
                           0,
                           0,
                           0,
                           0,
                           0,
                           3576,
                           MAV_PIXHAWK_ID,
                           MAV_COMPONENT_ID,
                           1)
        temp_key = struct.pack("<bbbbb53s",
                           53,
                           packet_sequence & 0xFF,
                           MAV_PIXHAWK_ID,
                           MAV_COMPONENT_ID,
                           84,
                           temp_key)
        temp_key = struct.pack("<b58sh",
                           0xFE,
                           temp_key,
                           checksum(temp_key, 143))
    elif mode == 'vel':
        temp_key = struct.pack("<lfffffffffffhbbb",
                       10,
                       0,
                       0,
                       0,
                       keypoint[0],
                       keypoint[1],
                       keypoint[2],
                       0,
                       0,
                       0,
                       0,
                       0,
                       3527,
                       MAV_PIXHAWK_ID,
                       MAV_COMPONENT_ID,
                       8)
        temp_key = struct.pack("<bbbbb53s",
                           53,
                           packet_sequence & 0xFF,
                           MAV_PIXHAWK_ID,
                           MAV_COMPONENT_ID,
                           84,
                           temp_key)
        temp_key = struct.pack("<b58sh",
                           0xFE,
                           temp_key,
                           checksum(temp_key, 143))


###################################################################################################

# Changes the flight mode of the drone 
def send_change_mode(Mode,PX4_mode,PX4_submode):
    global packet_sequence
    temp = struct.pack("<fffffffhbbb",
                       Mode,
                       PX4_mode,
                       PX4_submode,
                       0,
                       0,
                       0,
                       0,
                       MAV_CMD_DO_SET_MODE, #MAV_CMD id
                       MAV_PIXHAWK_ID, #SYSID_THISMAV for Pixhawk; 255 for GCS
                       MAV_COMPONENT_ID, # 0 for broadcast; others see MAV_COMPONENT enum
                       MAVLINK_MSG_ID_COMMAND_LONG_confirmation)
    temp = struct.pack("<bbbbb33s",
                       33,
                       packet_sequence & 0xFF,
                       MAV_PIXHAWK_ID,
                       MAV_COMPONENT_ID,
                       MAVLINK_MSG_ID_COMMAND_LONG,
                       temp)
    temp = struct.pack("<b38sh",
                       0xFE,
                       temp,
                       checksum(temp, MAVLINK_MSG_ID_COMMAND_LONG_CRC))
    packet_sequence += 1
    uart.write(temp)

####################################################################################

# Asks to the flight controller to return a tuple with the coordinates of the drone
def request_position():
    global packet_sequence
    temp = struct.pack("<fffffffhbbb",
                       32,
                       0,
                       0,
                       0,
                       0,
                       0,
                       0,
                       512, #MAV_CMD id
                       MAV_PIXHAWK_ID, #SYSID_THISMAV for Pixhawk; 255 for GCS
                       MAV_COMPONENT_ID, # 0 for broadcast; others see MAV_COMPONENT enum
                       MAVLINK_MSG_ID_COMMAND_LONG_confirmation)
    temp = struct.pack("<bbbbb33s",
                       33,
                       packet_sequence & 0xFF,
                       MAV_PIXHAWK_ID,
                       MAV_COMPONENT_ID,
                       MAVLINK_MSG_ID_COMMAND_LONG,
                       temp)
    temp = struct.pack("<b38sh",
                       0xFE,
                       temp,
                       checksum(temp, MAVLINK_MSG_ID_COMMAND_LONG_CRC))
    packet_sequence += 1
    uart.write(temp)

####################################################################################

#Asks the flight controller to return a tuple with the roll, pitch and yaw of the drone
def request_attitude():
    global packet_sequence
    temp = struct.pack("<fffffffhbbb",
                       30,
                       0,
                       0,
                       0,
                       0,
                       0,
                       0,
                       512, #MAV_CMD id
                       MAV_PIXHAWK_ID, #SYSID_THISMAV for Pixhawk; 255 for GCS
                       MAV_COMPONENT_ID, # 0 for broadcast; others see MAV_COMPONENT enum
                       MAVLINK_MSG_ID_COMMAND_LONG_confirmation)
    temp = struct.pack("<bbbbb33s",
                       33,
                       packet_sequence & 0xFF,
                       MAV_PIXHAWK_ID,
                       MAV_COMPONENT_ID,
                       MAVLINK_MSG_ID_COMMAND_LONG,
                       temp)
    temp = struct.pack("<b38sh",
                       0xFE,
                       temp,
                       checksum(temp, MAVLINK_MSG_ID_COMMAND_LONG_CRC))
    packet_sequence += 1
    uart.write(temp)


################################################################################
###                        MESSAGE FILTERING FUNCTIONS                       ###
################################################################################

# When a message is requested from the flight controller it's not the only one
# present on the serial port. As a matter of fact, even if the flight controller
# has been specifically told to do not send any spontaneus message, it sends a lot
# of message anyway. To overcome thi problem, the underlying functions filter the
# message by size and by ID number, in order to be sure that the read message is 
# the correct one.    
def read_position_message(message):
    if message is not None and message != bytes('','UTF-8'):
        mes = extract_msg(message,36)
        if mes is not None and mes!= bytes('','UTF-8'):
            cut_mes = cut_msg(mes,36)
            out =  struct.unpack('<bIffffff',cut_mes)
            if out[0] == 32:
                return (out[0],out[2],out[3],out[4])
            else:
                return None

def read_attitude_message(message):
    if message is not None and message != bytes('','UTF-8'):
        mes = extract_msg(message,36)
        if mes is not None and mes!= bytes('','UTF-8'):
            cut_mes = cut_msg(mes,36)
            out =  struct.unpack('<bIffffff',cut_mes)
            if out[0] == 30:
                return (out[0],out[2],out[3],out[4])
            else:
                return None

def cut_msg(message,length):
    message = message[0:length-2]
    return message[5:length-2]

def extract_msg(message,length):
    count = 1
    ind = 0
    pointer = 0
    while ind < len(message):
        if message[ind] == 254:
            if count != 1 and count <length:
                count = 0
                pointer = ind
        count += 1
        ind += 1
        if count == length:
            return message[pointer:pointer+length]

################################################################################
###                          DRONE MOVEMENT FUNCTIONS                        ###
################################################################################

# This function allows to move the drone in its NED local system using the 
# transformation between from global system using its roll, pitch and yaw

# Forward and backward
# Input: the distance in meters that the drone should travel
def go_for_back_ward(dist):
    pos = None
    att = None
    while pos is None:
        request_position()
        b = uart.read()
        pos = read_position_message(b)
    print(pos)
    while att is None:
        request_attitude()
        a = uart.read()
        att = read_attitude_message(a)
    yaw = att[3]
    print(yaw)

    return [pos[1]+dist*math.cos(yaw),pos[2]+dist*math.sin(yaw),pos[3]]

# Right and left
# Input: the distance in meters that the drone should travel
def go_right_left(dist):
    pos = None
    att = None
    while pos is None:
        request_position()
        b = uart.read()
        pos = read_position_message(b)
    #print(pos)
    while att is None:
        request_attitude()
        a = uart.read()
        att = read_attitude_message(a)
    yaw = att[3]
    #print(yaw)

    return [pos[1]+dist*math.cos(yaw+math.pi/2),pos[2]+dist*math.sin(yaw+math.pi/2),pos[3]]

####################################################################################
# Computes the distance between two points p1 and p2

def dist(p1,p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 + (p1[2]-p2[2])**2)

################################################################################
################################################################################
###                                  MAIN                                    ###   
################################################################################
################################################################################


################################################################################
###                               CONSTANTS                                  ###
################################################################################
thr = 4         # Threshold for the landing procedure
k_prop = 0.01   # Proportional gain of the controller
land_ct = 0     # Waiting index for the landing
i = 0
thr_pos = 0.5   # Threshold on the reaching of the position on the square path
l = 3           # Side of the drone search path square
w_ind = 0       # Waiting index if the circle is not a landing pad
lock_index = 0  # Waiting index if the drone breafly sees no circles it waits 
                # to see whether it's a processing error

# Position acknowledgments for the square path
p1_ack  = False
p2_ack  = False
p3_ack  = False
p4_ack  = False
p5_ack  = False
p6_ack  = False
p7_ack = False

target_lock = False

################################################################################
###                         TAKEOFF AND INITIAL SETUP                        ###
################################################################################

# Waiting for 30 seconds in order to allow the connection of the drone battery
# and to step away from the drone for safety reasons
time.sleep(30)
# Arming and takeoff

send_change_mode(MAV_MODE_1,MAV_PX4MODE_AUTO,MAV_PX4SUBMODE_TAKEOFF) # Sets takeoff mode
time.sleep_ms(2000)
send_arm_packet("_") 
#print("taking off")
time.sleep(20)       # Waiting for the drone to complete the takeoff procedure

# Move forward and wait 5 seconds
mode = "pos"                    # Set the "pos" mode for the keypoint reading               
keypoint = go_for_back_ward(3)  # Tells the drone to go forward by 3 meters

tim.callback(set_keypoint)      # Activates the callback for the set_keypoint function

print('Move forward')
time.sleep(2)
send_change_mode(209,6,0)     #Sets the offboard mode
time.sleep(5)

# Move right and wait 5 seconds
mode = "pos"
keypoint = go_right_left(3)     # Tells the drone to go right by 3 meters
print('Move right')
time.sleep(5)

# Retrives the home position of the drone 
home_pos = None
while home_pos is None:
    request_position()
    b = uart.read()
    home_pos = read_position_message(b)
print(home_pos)
time.sleep(5)

################################################################################
###                  MAIN WHILE: LOOKING FOR LANDING TARGET                  ###
################################################################################

while True:
    clock.tick()
    position = None
    # Retrives the drone position
    while position is None:
        request_position()
        b = uart.read()
        position = read_position_message(b)
    position = [position[1],position[2],position[3]]
    #print(position)
    max_circ = None 

    # Preprocessing the frame
    img = sensor.snapshot()
    extra_fb.replace(img)
    extra_fb.mean(2, threshold=True, offset=20, invert=True)

    max_r = 0
    max_circ = None

    circles = extra_fb.find_circles(threshold = 5000, x_margin = 100, y_margin = 100, r_margin = 100,
            r_min = 10, r_max = 50, r_step = 2)
    for circle in circle:
            if circle.r() > max_r:
                max_r = circle.r()
                max_circ = circle
    # Crop the image around the circle center and perform inference
    if max_circ:
        x = max_circ.x()-sensor.width()/2.0
        y = max_circ.y()-sensor.height()/2.0
        crop = img.crop(copy = True, roi = (int(max_circ.x()-1.5*max_circ.r()),
            int(max_circ.y()- 1.5*max_circ.r()), int(3*max_circ.r()),
            int(3*max_circ.r())))
        out = model.classify(crop)[0].output()
        
        # If the crop is a landing pad
        if out[0] < out[1]:
            print("allineamento")

            target_lock = True
            w_ind = 0
            dist_targ = math.sqrt((x)**2+(y)**2)   # Distance of the landing pad from the drone
            vel = k_prop * dist_targ               # Velocity proportional law control 
            if vel>0.5:                            # Saturates the velocity to 0.5 m/s
                vel = 0.5
            # Computes the absolute value of the angle between drone and landing pad
            alpha = abs(math.atan2(y,x))           

            # If the drone is farther that the treshold
            if dist_targ > thr:
                land_ct = 0
                mode = 'vel'   # Sets velocity control mode

                # Proportional controller
                if x >= 0 and y >= 0:
                    keypoint = [-vel*abs(math.sin(alpha)), +vel*abs(math.cos(alpha)),0]
                elif x < 0 and y > 0:
                    keypoint = [-vel*abs(math.sin(math.pi-alpha)), -vel*abs(math.cos(math.pi-alpha)),0]
                elif x < 0 and y < 0:
                    keypoint = [vel*abs(math.sin(math.pi-alpha)), -vel*abs(math.cos(math.pi-alpha)),0]
                elif x > 0 and y < 0:
                    keypoint = [vel*abs(math.sin(alpha)), vel*abs(math.cos(alpha)),0]
                else:
                    keypoint = [0,0,0]

            # If the drone is on the target it lands
            else:
                land_ct+=1
                print("On center: {}".format(land_ct))

            if land_ct > 0:
                mode = 'vel'
                keypoint = [0,0,0]
                print("Land")
                send_change_mode(MAV_MODE_1,MAV_PX4MODE_AUTO,MAV_PX4SUBMODE_LAND)  # Sets land mode
                time.sleep(15)
                break
        # If the circle is not a landing pad
        else:
            print("wait")
            mode = 'vel'
            keypoint = [0,0,0]
            # Waits to see if it's a false negative
            w_ind +=1
            if w_ind > 10:
                target_lock = False
                max_circ = None
    # Search path of the drone. The drone travels a square of side l and if it doesn't find 
    # a landing pad it lands
    if max_circ is None and not target_lock and position is not None and home_pos is not None:
        #print("Ricerca")
        mode = 'pos'
        position1 = [home_pos[1],home_pos[2],-3]
        position2 = [position1[0]+l/2,position1[1],position1[2]]
        position3 = [position2[0],position2[1]+l,position2[2]]
        position4 = [position3[0]-l,position3[1],position3[2]]
        position5 = [position4[0],position4[1]-l,position4[2]]
        position6 = [position5[0]+l/2*math.cos(math.pi/4),position5[1]+l/2*math.sin(math.pi/4),position5[2]]
        position7 = [position1[0],position1[1],position1[2]] 

        if not p1_ack and dist(position,position1)>thr_pos:
            keypoint = position1
            print("going to p1")

        else:

            p1_ack = True

            if not p2_ack and dist(position,position2)>thr_pos:
                keypoint = position2
                print("going to p2")
            else:
                p2_ack = True

                if not p3_ack and dist(position,position3)>thr_pos:
                    keypoint = position3
                    print("going to p3")
                else:
                    p3_ack = True

                    if not p4_ack and dist(position,position4)>thr_pos:
                        keypoint = position4
                        print("going to p4")
                    else:
                        p4_ack = True

                        if not p5_ack and dist(position,position5)>thr_pos:
                            keypoint = position5
                            print("going to p5")
                        else:
                            p5_ack = True

                            if not p6_ack and dist(position,position6)>thr_pos:
                                keypoint = position6
                                print("going to p6")
                            else:
                                p6_ack = True

                                if not p7_ack and dist(position,position7)>thr_pos:
                                    keypoint = position7
                                    print("going to p7")
                                else:
                                    p7_ack = True
                                    print("No landing pad found!!!")
                                    p1_ack  = False
                                    p2_ack  = False
                                    p3_ack  = False
                                    p4_ack  = False
                                    p5_ack  = False
                                    p6_ack  = False
                                    p7_ack = False
                                    send_change_mode(MAV_MODE_1,MAV_PX4MODE_AUTO,MAV_PX4SUBMODE_LAND)  # Sets land mode
                                    #time.sleep(25)
                                    #send_arm_packet(4)
                                    break
    # If the drone breafly sees no circles it waits to see whether it's a processing error
    if max_circ is None and target_lock:
        lock_index +=1
        print("Target lock,but non visible")
        '''
        mode = 'vel'
        keypoint = [0,0,0]
        '''
        if lock_index > 20:
            target_lock = False
            lock_index  = 0

        continue
