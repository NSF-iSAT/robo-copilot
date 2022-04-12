import time
import rospy
from misty_wrapper.msg import MoveHead
from gaze_tracking_ros.msg import GazeState

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

DELTA_MIN = 0.05

class GazeFollower:
    def __init__(self, misty_id, img_w, img_h):
        self.img_dims = (img_w, img_h)
        gaze_sub = rospy.Subscriber("/gaze_state", GazeState, self.gaze_callback)
        self.head_pub = rospy.Publisher("/misty/id_0/head", MoveHead, queue_size=1)

        self.head_roll  = 0
        self.head_pitch = 0
        self.head_yaw   = 0

        rospy.init_node("gaze_follower", anonymous=True)
        # reset head position
        rospy.sleep(5)
        self.head_pub.publish(MoveHead(roll=self.head_roll, pitch=self.head_pitch, yaw=self.head_yaw,
            velocity=100, units="degrees"))
        
        self.timeout = rospy.Duration(1.5)
        self.reset_timeout = rospy.Duration(10)
        self.movement_start = rospy.Time.now()

        while not rospy.is_shutdown():
            if rospy.Time.now() - self.movement_start > self.reset_timeout:
                self.head_pub.publish(MoveHead(roll=0, pitch=0, yaw=0,
                    velocity=100, units="degrees"))
                self.movement_start = rospy.Time.now()

    def gaze_callback(self, msg):

        if rospy.Time.now() - self.movement_start > self.timeout:
            avg_pupil_x = (msg.pupil_left_coords.x + msg.pupil_right_coords.x) / 2.0
            avg_pupil_y = (msg.pupil_left_coords.y + msg.pupil_right_coords.y) / 2.0
            
            print(msg)

            avg_pupil_x = avg_pupil_x / self.img_dims[0] - 0.5
            avg_pupil_y = avg_pupil_y / self.img_dims[1] - 0.5

            # calculate head movement
            pitch = self.head_pitch
            yaw = self.head_yaw

            if (abs(avg_pupil_x) + abs(avg_pupil_y)) > DELTA_MIN:
                self.head_yaw -= ((YAW_LEFT - YAW_RIGHT)/33 * (avg_pupil_x*13))
                self.head_pitch += ((PITCH_DOWN - PITCH_UP)/33 * (avg_pupil_y*13))
                # self.head_pub.publish(MoveHead(roll = self.head_roll, pitch = self.head_pitch + ((PITCH_DOWN - PITCH_UP)/33 * (avg_pupil_y*13)), velocity=10.0,
                    # yaw = self.head_yaw - ((YAW_LEFT - YAW_RIGHT)/33 * (avg_pupil_x*13)), duration=1, units="degrees"))

                if self.head_yaw < YAW_RIGHT:
                    self.head_yaw = YAW_RIGHT
                elif self.head_yaw > YAW_LEFT:
                    self.head_yaw = YAW_LEFT

                if self.head_pitch > PITCH_DOWN:
                    self.head_pitch = PITCH_DOWN
                elif self.head_pitch < PITCH_UP:
                    self.head_pitch = PITCH_UP

                self.movement_start = rospy.Time.now()
                self.head_pub.publish(MoveHead(roll = self.head_roll, pitch = self.head_pitch, yaw = self.head_yaw,
                    velocity=100, units="degrees"))
            
if __name__ == "__main__":
    GazeFollower(0, 480, 640)