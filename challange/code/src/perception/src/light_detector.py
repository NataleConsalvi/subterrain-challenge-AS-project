 #!/usr/bin/env python
import cv2
import numpy as np
import ros_numpy
import rospy
import tf2_ros
import tf2_geometry_msgs
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
from image_geometry import PinholeCameraModel

class LightDetector(object):
    def __init__(self):
        # Initialize the CvBridge class
        self.bridge = CvBridge()

        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(10)

        # Subscribers
        self.semantic_camera_subscriber = rospy.Subscriber("/unity_ros/Quadrotor/Sensors/SemanticCamera/image_raw", Image, self.sem_image_callback)
        self.sem_img = None

        self.semantic_camera_info_subscriber = rospy.Subscriber("/unity_ros/Quadrotor/Sensors/SemanticCamera/camera_info", CameraInfo, self.camera_info_callback)
        self.point_cloud_subscriber = rospy.Subscriber("/perception/pcl/out_colored", PointCloud2, self.point_cloud_callback)

        # self.rgb_camera_subscriber = rospy.Subscriber("/realsense/rgb/left/image_raw", Image, self.rgb_image_callback)
        # self.rgb_img = None

        self._image_pub = rospy.Publisher("/processed_image", Image, queue_size=10)

        self.overlayed_image_pub = rospy.Publisher("/overlayed_image", Image, queue_size=10)

        self.tf_buffer = tf2_ros.Buffer() 
        tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Placeholder for camera intrinsic parameters
        self.K = None
        self.P = None

    def camera_info_callback(self, msg):
        rospy.loginfo('Camera Info received...')
        self.K = np.array(msg.K).reshape(3, 3)
        self.P = np.array(msg.P).reshape(3, 3)

    def find_unique_colors(self, image):
        # Convert the image to a list of pixels and find unique colors
        unique_colors = np.unique(image.reshape(-1, 3), axis=0)
        
        # Log the unique colors
        rospy.loginfo("Unique colors found (in BGR format):")
        for color in unique_colors:
            rospy.loginfo(str(color))
        
    def sem_image_callback(self, msg):
        try:
            # Convert the image to a OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.sem_img = cv_image
            #self.find_unique_colors(cv_image)

            # # Process the image
            self.process_image(cv_image)

        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")


    def rgb_image_callback(self, msg):
        try:
            # Convert the image to a OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.rgb_img = cv_image
            #self.find_unique_colors(cv_image)

            if self.sem_img is not None:
                dst = cv2.addWeighted(cv_image, 0.7, self.sem_img, 0.3, 0.0)
                self.overlayed_image_pub.publish(self.bridge.cv2_to_imgmsg(dst, "bgr8"))

        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")

    def process_image(self, image):
        # Example of processing the image:
        # Define lower and upper bounds for the color you're looking for
        lower_bound = np.array([4, 235, 255])  
        upper_bound = np.array([4, 235, 255]) 

        # Find pixels within the bounds and create a mask
        mask = cv2.inRange(image, lower_bound, upper_bound)

        # Optionally, apply the mask to get the segmented object
        segmented_image = cv2.bitwise_and(image, image, mask=mask)
        # Find contours from the binary image
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw bounding box
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)  # Blue bounding box

        # Display the result
        try:
            ros_image = self.bridge.cv2_to_imgmsg(image, "bgr8")
            self._image_pub.publish(ros_image)
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error in Publishing: {e}")

    
    
    def point_cloud_callback(self, data):
        rospy.loginfo('Point Cloud received...')
        try:
            frame_id = data.header.frame_id

            transform = tf_buffer.lookup_transform('Quadrotor/Sensors/DepthCamera', frame_id, rospy.Time(), rospy.Duration(1.0))

            cloud_transformed = tf2_ros.do_transform_cloud(data, transform)

            pcl_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data)

            u = (self.K[0, 0] * pcl_array[:, 0] / pcl_array[:, 2]) + self.K[0, 2]
            v = (self.K[1, 1] * pcl_array[:, 1] / pcl_array[:, 2]) + self.K[1, 2]

        except tf2_ros.LookupException as e:
            rospy.logerr(e)

    def calculate_3d_coordinates(u, v, depth, camera_intrinsics):
        fx = camera_intrinsics['fx']
        fy = camera_intrinsics['fy']
        cx = camera_intrinsics['cx']
        cy = camera_intrinsics['cy']

        Z = depth
        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy
        return X, Y, Z

if __name__ == '__main__':
    rospy.init_node("light_detector_node", anonymous=True)
    LightDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    