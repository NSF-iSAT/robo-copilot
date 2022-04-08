import rospy
from misty_wrapper.msg import MoveHead
from gaze_tracking_ros import GazeState

# idk about some of these parameters, I just based it on https://github.com/MistySampleSkills/Misty-Concierge-Template/blob/master/JavaScript/conciergeBaseTemplate/conciergeBaseTemplate.js

# movement limits, in degrees
PITCH_UP = -40
PITCH_DOWN = 26
YAW_LEFT = 81
YAW_RIGHT = -81
ROLL_LEFT = -40
ROLL_RIGHT = 40

BEAR_RANGE = 13 # -13 right and +13 left
ELEVATION_RANGE = 13 # -13 up and +13 down

class GazeFollower:
    def __init__(self, img_w, img_h):
        self.img_dims = (img_w, img_h)
        gaze_sub = rospy.Subscriber("/gaze_tracking/gaze_state", GazeState, self.gaze_callback)
        self.head_pub = rospy.Publisher("/misty/id_0/head", MoveHead, queue_size=1)

        self.head_roll  = 0
        self.head_pitch = 0
        self.head_yaw   = 0

        rospy.init_node("gaze_follower", anonymous=True)
        # reset head position
        self.head_pub.publish(MoveHead(self.head_roll, self.head_pitch, self.head_yaw))

        rospy.spin()

    def gaze_callback(self, msg):
        avg_pupil_x = (msg.pupil_left_coords.x + msg.pupil_right_coords.x) / 2
        avg_pupil_y = (msg.pupil_left_coords.y + msg.pupil_right_coords.y) / 2
        
        avg_pupil_x = avg_pupil_x / self.img_dims[0] - 0.5
        avg_pupil_y = avg_pupil_y / self.img_dims[1] - 0.5

        # calculate head movement
        pitch = self.head_pitch
        yaw = self.head_yaw

        if (avg_pupil_y > 0.1 or avg_pupil_y < -0.1):
            pitch = self.head_pitch + (((PITCH_DOWN - PITCH_UP) / 33) * (avg_pupil_y*13))
        if (avg_pupil_x > 0.1 or avg_pupil_x < -0.1):
            yaw   = self.head_yaw +   (((YAW_RIGHT - YAW_LEFT) / 66) * (avg_pupil_x*13))
        
        # limit movement
        if (pitch > PITCH_UP):
            pitch = PITCH_UP
        elif (pitch < PITCH_DOWN):
            pitch = PITCH_DOWN
        if (yaw > YAW_RIGHT):
            yaw = YAW_RIGHT
        elif (yaw < YAW_LEFT):
            yaw = YAW_LEFT
            
        # publish movement
        if(yaw != self.head_yaw or pitch != self.head_pitch):
            self.head_pub.publish(MoveHead(roll=self.head_roll, pitch=pitch, yaw=yaw, velocity=0.5, units="degrees"))

