 #!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image

class LightDetector(object):
    def __init__(self):

        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(10)

        # Subscribers
        self.image_subscriber = rospy.Subscriber("/unity_ros/Quadrotor/Sensors/SemanticCamera/image_raw",Image, self.semantic_callback)

    def semantic_callback(self, msg):
        rospy.loginfo('Image received...')
            
if __name__ == '__main__':
    rospy.init_node("light_detector_node", anonymous=True)
    my_node = LightDetector()
    rospy.spin()