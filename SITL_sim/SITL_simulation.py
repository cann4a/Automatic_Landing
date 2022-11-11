from __future__ import division
from statistics import mean
from pymavlink import mavutil
import time

from tensorflow.keras.preprocessing.image import img_to_array
from tensorflow.keras.models import load_model
import numpy as np
import cv2 as cv
import gi
from threading import Timer
import math

gi.require_version('Gst', '1.0')
from gi.repository import Gst


# Enstablishes the connection to the drone simulation via localhost 
the_connection = mavutil.mavlink_connection('udpin:localhost:14540')

################################################################################
###                                 CLASSES                                  ###
################################################################################

################################################################################
# The RepeatedTimer class is needed to instantiate a callback to a function. 
# This function will be executed every given amount of seconds.
class RepeatedTimer(object):
    """Callback function caller class constructor
    Attributes:
        interval (int): Period of the function execution
        function (string): Name of the function that is called
    """
    def __init__(self, interval, function, *args, **kwargs):
        self._timer     = None
        self.interval   = interval
        self.function   = function
        self.args       = args
        self.kwargs     = kwargs
        self.is_running = False
        self.start()

    def _run(self):
        self.is_running = False
        self.start()
        self.function(keypoint, **self.kwargs)

    def start(self):
        if not self.is_running:
            self._timer = Timer(self.interval, self._run)
            self._timer.start()
            self.is_running = True

    def stop(self):
        self._timer.cancel()
        self.is_running = False

def hello(name):
    print ("Hello %s!" % name)

##################################################################################################
# The Video class serves to read the video stream in the simulation and utilize it in the script
class Video():
    """BlueRov video capture class constructor
    Attributes:
        port (int): Video UDP port
        video_codec (string): Source h264 parser
        video_decode (string): Transform YUV (12bits) to BGR (24bits)
        video_pipe (object): GStreamer top-level pipeline
        video_sink (object): Gstreamer sink element
        video_sink_conf (string): Sink configuration
        video_source (string): Udp source ip and port
    """

    def __init__(self, port=5600):
        """Summary
        Args:
            port (int, optional): UDP port
        """

        Gst.init(None)

        self.port = port
        self._frame = None

        # [Software component diagram](https://www.ardusub.com/software/components.html)
        # UDP video stream (:5600)
        self.video_source = 'udpsrc port={}'.format(self.port)
        # [Rasp raw image](http://picamera.readthedocs.io/en/release-0.7/recipes2.html#raw-image-capture-yuv-format)
        # Cam -> CSI-2 -> H264 Raw (YUV 4-4-4 (12bits) I420)
        self.video_codec = '! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264'
        # Python don't have nibble, convert YUV nibbles (4-4-4) to OpenCV standard BGR bytes (8-8-8)
        self.video_decode = \
            '! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert'
        # Create a sink to get data
        self.video_sink_conf = \
            '! appsink emit-signals=true sync=false max-buffers=2 drop=true'

        self.video_pipe = None
        self.video_sink = None

        self.run()

    def start_gst(self, config=None):
        """ Start gstreamer pipeline and sink
        Pipeline description list e.g:
            [
                'videotestsrc ! decodebin', \
                '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                '! appsink'
            ]
        Args:
            config (list, optional): Gstreamer pileline description list
        """

        if not config:
            config = \
                [
                    'videotestsrc ! decodebin',
                    '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                    '! appsink'
                ]

        command = ' '.join(config)
        self.video_pipe = Gst.parse_launch(command)
        self.video_pipe.set_state(Gst.State.PLAYING)
        self.video_sink = self.video_pipe.get_by_name('appsink0')

    @staticmethod
    def gst_to_opencv(sample):
        """Transform byte array into np array
        Args:
            sample (TYPE): Description
        Returns:
            TYPE: Description
        """
        buf = sample.get_buffer()
        caps = sample.get_caps()
        array = np.ndarray(
            (
                caps.get_structure(0).get_value('height'),
                caps.get_structure(0).get_value('width'),
                3
            ),
            buffer=buf.extract_dup(0, buf.get_size()), dtype=np.uint8)
        return array

    def frame(self):
        """ Get Frame
        Returns:
            iterable: bool and image frame, cap.read() output
        """
        return self._frame

    def frame_available(self):
        """Check if frame is available
        Returns:
            bool: true if frame is available
        """
        return type(self._frame) != type(None)

    def run(self):
        """ Get frame to update _frame
        """

        self.start_gst(
            [
                self.video_source,
                self.video_codec,
                self.video_decode,
                self.video_sink_conf
            ])

        self.video_sink.connect('new-sample', self.callback)

    def callback(self, sink):
        sample = sink.emit('pull-sample')
        new_frame = self.gst_to_opencv(sample)
        self._frame = new_frame

        return Gst.FlowReturn.OK
################################################################################
###                             UTILITIES FUNCTION                           ###
################################################################################

##################################################################################################
# This function serves to move the gimbal of the drone to a specified angle. This is needed
# because, by defalut, the gimbal is oriented parallel to the ground. For this application 
# the camera must point down, perpendicularly to the ground. 
def look_at(tilt, roll=0, pan=0):
    """
    Moves gimbal to given position
    Args:
        tilt (float): tilt angle in centidegrees (0 is forward)
        roll (float, optional): pan angle in centidegrees (0 is forward)
        pan  (float, optional): pan angle in centidegrees (0 is forward)
    """
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
        1,
        tilt,
        roll,
        pan,
        0, 0, 0,
        mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING)

##################################################################################################
# Sets a keypoint that can be both in position or velocity. If the pos mode is active, the drone 
# will head to the point (x,y,z) defined by the keypoint variable. Otherwise, if the vel mode is
# active, the drone velocity will be set to (x,y,z).   
def set_keypoint(keypoint):
    if mode == 'pos':
        the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
            the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, 3576,
            keypoint[0], keypoint[1], keypoint[2],0,0,0,0,0,0,0,0))
    elif mode == 'vel':
        the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
            the_connection.target_component, mavutil.mavlink.MAV_FRAME_BODY_NED, 3527,
            0, 0, 0, keypoint[0],keypoint[1],keypoint[2],0,0,0,0,0))

# Sets the offboard mode 
def set_offboard_mode():
    the_connection.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                            PX4_MAV_MODE,
                            PX4_CUSTOM_SUB_MODE_OFFBOARD, 0, 0, 0, 0, 0)
# Sets the land mode 
def set_land_mode():
    the_connection.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                            217,
                            4, 6, 0, 0, 0, 0)
# Sets the takeoff mode 
def set_takeoff_mode():
    the_connection.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                            217,
                            4, 2, 0, 0, 0, 0)
    
################################################################################
###                              INITIAL SETUP                               ###
################################################################################
PX4_MAV_MODE = 209
PX4_CUSTOM_SUB_MODE_OFFBOARD = 6

# Loads the neural network to perform inference
model_path = '/home/matteo/Landing_pad_20e.h5'
print("[INFO] loading CNN...")
model = load_model(model_path)
print("Opened") 

# Enstablishes the connection with the drone in the simulation
the_connection.wait_heartbeat()
print("Heartbeat from system: (system %u component %u)" %(the_connection.target_system, the_connection.target_component))

# Instantiates the Video class to read the video streaming in the simulation
print("[INFO] Preparing Video")
video = Video()
print("[INFO] Video Ready")

# Rotates the gimbal
look_at(-90)

# Takeoff the drone
set_takeoff_mode()
time.sleep(15) 

# Arming the drone
the_connection.mav.command_long_send(the_connection.target_system, 
        the_connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0,0,0,0,0,0)
time.sleep(10)

#msg = the_connection.recv_match(type = 'LOCAL_POSITION_NED',blocking = True)
#position = [msg.x,msg.y,msg.z]

mode = 'vel'
keypoint = [0,0,0]

# Starts the timer for the position/velocity set points at a frequency of 2 Hz
rt = RepeatedTimer(1/2,set_keypoint,keypoint)
time.sleep(2)

# Setting offboard mode to abilitate the drone control by keypoints
set_offboard_mode()

################################################################################
###                               CONSTANTS                                  ###
################################################################################    
delta = 5       # Pixel threshold to avoid problems with the crop 
thr = 5         # Threshold for the landing procedure
k_prop = 0.01   # Proportional gain of the controller
land_ct = 0     # Waiting index for the landing
index_2 = 0
i = 0
thr_pos = 0.1   # Threshold on the reaching of the position on the square path
l = 6           # Side of the drone search path square
wind = 0        # Waiting index if the circle is not a landing pad

# Position acknowledgments for the square path
p1_ack  = False
p2_ack  = False
p3_ack  = False
p4_ack  = False
p5_ack  = False
p6_ack  = False
p7_ack = False
target_lock = False

alpha_v = []
x_vec = []
y_vec = []
dir_aq = False
vel = 0.5
################################################################################
###                  MAIN WHILE: LOOKING FOR LANDING TARGET                  ###
################################################################################

while True:
    i +=1
    # Receiving the drone position
    msg = the_connection.recv_match(type = 'LOCAL_POSITION_NED',blocking = True)
    position = [msg.x,msg.y,msg.z]
    if i == 1:
        # If it's the first position it's set to be the home position
        home_pos = [msg.x,msg.y,msg.z]

    if not video.frame_available():
        continue
    frame = video.frame()
    #  Resize resolution equal to the OpenMv Cam H7P resolution (320x240)
    frame = cv.resize(frame, (320,240), interpolation=cv.INTER_AREA)
    frame_orj = frame
    # Applies the preprocessing 
    frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    frame = cv.blur(frame,(6,6))
    frame = cv.adaptiveThreshold(frame,255,cv.ADAPTIVE_THRESH_GAUSSIAN_C,cv.THRESH_BINARY_INV,11,2)

    circles = cv.HoughCircles(frame,cv.HOUGH_GRADIENT_ALT,2,50,param1=1,param2=0.4,minRadius=10,maxRadius=20)

    # If some circle is detected it takes the one with bigger radius
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        max_circ = []
        max_r = 0
        seen_count = 0
        for c in circles:
            if c[2] > max_r:
                max_circ = c
        # If the crop is too close to the edge of the image, the program crashes. Hence a threshold
        # delta is used, in order to don't avaluate circles that are close to the image edge. 
        if max_circ[2]<= max_circ[1]-delta and max_circ[2]<= max_circ[0]-delta:
            tmp_roi = frame_orj[(max_circ[1]-max_circ[2]-delta):(max_circ[1]+max_circ[2]+delta),
                                (max_circ[0]-max_circ[2]-delta):(max_circ[0]+max_circ[2]+delta)]
            # Prepares the image for feeding the neural network
            tmp_roi = cv.resize(tmp_roi, (64,64), interpolation=cv.INTER_AREA)
            tmp_roi = cv.cvtColor(tmp_roi, cv.COLOR_BGR2GRAY)
            tmp_roi = tmp_roi.astype("float") / 255.0
            tmp_roi = img_to_array(tmp_roi)
            tmp_roi = np.expand_dims(tmp_roi, axis=0)
            (notLandingpad, landingpad) = model.predict(tmp_roi)[0]

            # If the circle is a landing pad
            if landingpad > 0.8:
                seen_count += 1
                target_lock = True
                index_2 = 0

                # Computes the x,y of the landing pad and nalso the angle from the center of the
                # image and the landing pad itself
                x = max_circ[0]-320/2.0
                y = max_circ[1]-240/2.0
                alpha = abs(math.atan2(y,x))

                dist_targ = math.sqrt((x)**2+(y)**2)
                vel = k_prop * dist_targ         # Velocity proportional law control 

                if vel>0.5:         # Saturates the velocity to 0.5 m/s
                    vel = 0.5

                # If the drone is farther that the treshold
                if dist_targ>thr:

                    mode = 'vel'    # Sets velocity control mode
                    land_ct = 0

                    # Proportional controller
                    if x >= 0 and y >= 0:    
                        keypoint = [-vel*abs(math.sin(alpha)), +vel*abs(math.cos(alpha)),0]               
                    elif x < 0 and y > 0:
                        keypoint = [-vel*abs(math.sin(alpha)), -vel*abs(math.cos(alpha)),0]
                    elif x < 0 and y < 0:
                        keypoint = [+vel*abs(math.sin(alpha)), -vel*abs(math.cos(alpha)),0]
                    elif x > 0 and y < 0:
                        keypoint = [vel*abs(math.sin(alpha)), vel*abs(math.cos(alpha)),0]
                    else:
                        keypoint = [0,0,0]
                    print(keypoint)

                # If the drone is on the target it lands
                else:
                    land_ct+=1
                    print("On center: {}".format(land_ct))

                if  land_ct>0:
                    mode = 'vel'
                    keypoint = [0,0,0]
                    print("Land")
                    set_land_mode()
                    rt.stop()
                    time.sleep(30)
                    the_connection.mav.command_long_send(the_connection.target_system,
                            the_connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0,0,0,0,0,0)
                    break
            # If the circle is not a landing pad
            else: 
                print("wait")
                continue
        else:
            print("No crop")
            continue
    # Search path of the drone. The drone travels a square of side l and if it doesn't find 
    # a landing pad it lands
    elif circles is None and not target_lock:
        alpha_v = []
        x_vec = []
        y_vec = []
        dir_aq = False
        mode = 'pos'
        #print("Ricerca")
        position1 = [home_pos[0],home_pos[1],-3]
        position2 = [position1[0]+l/2,position1[1],position1[2]]
        position3 = [position2[0],position2[1]+l,position2[2]]
        position4 = [position3[0]-l,position3[1],position3[2]] 
        position5 = [position4[0],position4[1]-l,position4[2]]
        position6 = [position5[0]+l/2*math.cos(math.pi/4),position5[1]+l/2*math.sin(math.pi/4),position5[2]]
        position7 = [position1[0],position1[1],position1[2]] # per fare qualche pattern esteso che si ripete
        
        if  not p1_ack and math.dist(position,position1)>thr_pos:
            keypoint = position1
            print("going to p1")
        else:
            p1_ack = True

        if not p2_ack and p1_ack and math.dist(position,position2)>thr_pos:
            keypoint = position2
            print("going to p2")
        elif p1_ack and math.dist(position,position2)<thr_pos: 
            p2_ack = True

        if not p3_ack and p1_ack and p2_ack and math.dist(position,position3)>thr_pos:
            keypoint = position3
            print("going to p3")
        elif p1_ack and p2_ack and math.dist(position,position3)<thr_pos:
            p3_ack = True

        if not p4_ack and p1_ack and p2_ack and p3_ack and math.dist(position,position4)>thr_pos:
            keypoint = position4
            print("going to p4")
        elif p1_ack and p2_ack and p3_ack and math.dist(position,position4)<thr_pos:
            p4_ack = True

        if not p5_ack and p1_ack and p2_ack and p3_ack and p4_ack and math.dist(position,position5)>thr_pos:
            keypoint = position5
            print("going to p5")
        elif p1_ack and p2_ack and p3_ack and p4_ack and math.dist(position,position5)<thr_pos:
            p5_ack = True

        if not p6_ack and p1_ack and p2_ack and p3_ack and p4_ack and p5_ack and math.dist(position,position6)>thr_pos:
            keypoint = position6
            print("going to p6")
        elif p1_ack and p2_ack and p3_ack and p4_ack and p5_ack and math.dist(position,position6)<thr_pos:
            p6_ack = True

        if not p7_ack and p1_ack and p2_ack and p3_ack and p4_ack and p5_ack and p6_ack and math.dist(position,position7)>thr_pos:
            keypoint = position7
            print("going to p7")
        elif p1_ack and p2_ack and p3_ack and p4_ack and p5_ack and p6_ack and math.dist(position,position7)<thr_pos:
            p7_ack = True
            print("No landing pad found!!!")
            #break
            p1_ack  = False
            p2_ack  = False
            p3_ack  = False
            p4_ack  = False
            p5_ack  = False
            p6_ack  = False
            p7_ack = False
            continue
    else:
        print("wait No target visible")
        index_2 +=1
        if index_2>100:
            target_lock = False
            index_2 = 0
        continue
    #print(keypoint)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break
