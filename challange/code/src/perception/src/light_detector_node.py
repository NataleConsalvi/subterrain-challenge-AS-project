#!/usr/bin/env python3

# Import necessary libraries
import cv2 # OpenCV for image processing
import numpy as np # NumPy for numerical operations
import pcl # Point Cloud Library for 3D data processing
import ros_numpy # Utilities for converting between ROS messages and NumPy arrays
import rospy # Python library for ROS
import tf2_ros # TF2 ROS library for transformations
import tf2_geometry_msgs # TF2 Geometry messages
from math import sqrt
from sensor_msgs.msg import Image, CameraInfo, PointCloud2 # ROS message types
from vision_msgs.msg import Detection3DArray, Detection3D, BoundingBox3D # ROS message types for 3D detections
from visualization_msgs.msg import MarkerArray, Marker # ROS message types for visualization
from cv_bridge import CvBridge, CvBridgeError # Library for converting between ROS Image messages and OpenCV images
from image_geometry import PinholeCameraModel # Library for  for interpreting images geometrically
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud # Utility for transforming sensor_msgs with TF2 

class LightDetector(object):
    def __init__(self):
        """
        Initializing the LightDetector ROS node. Setting up publishers, subscribers,
        and other necessary variables for the node's operation.
        """
        self.bridge = CvBridge() # Initialize the CvBridge class
        self.loop_rate = rospy.Rate(10) # Node cycle rate (in Hz)

        self.init_params() # Initialize parameters from ROS parameter server


        # Subscribers to input topics for semantic images, camera info, point clouds, and RGB images
        self.semantic_camera_subscriber = rospy.Subscriber(self.semantic_img_input_topic,
            Image, self.sem_image_callback)
        self.sem_img = None

        self.semantic_camera_info_subscriber = rospy.Subscriber(self.semantic_camera_info_input_topic,
            CameraInfo, self.camera_info_callback)

        self.point_cloud_subscriber = rospy.Subscriber(self.pcl_input_topic,
            PointCloud2, self.point_cloud_callback)

        self.rgb_camera_subscriber = rospy.Subscriber(self.rgb_img_input_topic,
            Image, self.rgb_image_callback)
        self.rgb_img = None


        # Initializing publishers for processed images, overlaid images, object point clouds, bounding boxes, and visualization markers
        # based on configuration parameters
        if self.publish_proc_sem_img:
            self.proc_sem_img_pub = rospy.Publisher(self.proc_sem_img_out_topic,
                Image, queue_size=10)


        if self.publish_overlayed_rgb_img:
            self.overlayed_image_pub = rospy.Publisher(self.overlayed_rgb_img_out_topic,
                Image, queue_size=10)
        
        if self.publish_object_pcl:
            self.object_pcl_pub = rospy.Publisher(self.object_pcl_out_topic,
                PointCloud2, queue_size=10)

        if self.publish_object_bb:
            self.object_bb_pub = rospy.Publisher(self.object_bb_out_topic,
                Detection3DArray, queue_size=10)

        if self.publish_object_vis_markers:
            self.object_marker_pub = rospy.Publisher(self.object_vis_out_topic,
                MarkerArray, queue_size=10)

        self.tf_buffer = tf2_ros.Buffer() 
        tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.K = None # Placeholder for camera intrinsic parameters
        self.mask = None # Placeholder for mask image for detected light
        self.detected_lights = None # Placeholder for detected lights
 
    def init_params(self):
        """
        Initialize parameters from the ROS parameter server. These parameters configure the
        topics to subscribe to and publish to, as well as other needed parameters for
        the node.
        """
        self.semantic_img_input_topic = rospy.get_param(
            "/perception_params/semantic_camera_img_input_topic")
        self.semantic_camera_info_input_topic = rospy.get_param(
            "/perception_params/semantic_camera_info_input_topic")

        self.pcl_input_topic = rospy.get_param(
            "/perception_params/pcl_input_topic")

        self.rgb_img_input_topic = rospy.get_param(
            "/perception_params/rgb_img_input_topic")

        self.publish_proc_sem_img = rospy.get_param(
            "/perception_params/publish_2d_processed_semantic_img")
        self.proc_sem_img_out_topic = rospy.get_param(
            "/perception_params/2d_processed_semantic_img_out_topic")

        self.publish_overlayed_rgb_img = rospy.get_param(
            "/perception_params/publish_overlayed_rgb_img")
        self.overlayed_rgb_img_out_topic = rospy.get_param(
            "/perception_params/overlayed_rgb_img_out_topic")
        
        self.publish_object_pcl = rospy.get_param(
            "/perception_params/publish_object_pcl")
        self.object_pcl_out_topic = rospy.get_param(
            "/perception_params/object_pcl_out_topic")

        self.publish_object_bb = rospy.get_param(
            "/perception_params/publish_object_bb")
        self.object_bb_out_topic = rospy.get_param(
            "/perception_params/object_bb_out_topic")
        
        self.publish_object_vis_markers = rospy.get_param(
            "/perception_params/publish_object_vis_markers")
        self.object_vis_out_topic = rospy.get_param(
            "/perception_params/object_vis_out_topic")
        
        self.pub_in_world_coords = rospy.get_param(
            "/perception_params/publish_in_world_coords")

        self.mean_k = rospy.get_param(
            "/perception_params/mean_k")
        self.thresh = rospy.get_param(
            "/perception_params/thresh")

    # Defining camera info callback
    def camera_info_callback(self, msg):
        if self.K is None:
            rospy.loginfo('Camera Info received...')
            self.K = np.array(msg.K).reshape(3, 3) # Intrinsic matrix from camera info message
    
    # Convert the image to a list of pixels and find unique colors
    def find_unique_colors(self, image):
        unique_colors = np.unique(image.reshape(-1, 3), axis=0)
        rospy.loginfo("Unique colors found (in BGR format):") # Log the unique colors
        for color in unique_colors:
            rospy.loginfo(str(color))

    # Defining semantic image callback   
    def sem_image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8") # Convert the image to a OpenCV image
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
        
        self.sem_img = cv_image

        #self.find_unique_colors(cv_image) # Used for finding unique colors in semantic image

        self.process_image(cv_image) #  Process the image

    # Defining rgb image callback  
    def rgb_image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8") # Convert the image to a OpenCV image
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
        
        self.rgb_img = cv_image

        # if self.sem_img is not None:
        #     dst = cv2.addWeighted(cv_image, 0.7, self.sem_img, 0.3, 0.0)
        #     self.overlayed_image_pub.publish(self.bridge.cv2_to_imgmsg(dst, "bgr8"))


    def process_image(self, image):
        # Processing the image
        # Detecting the light in the image
        # Define lower and upper bounds for the color of the light
        lower_bound = np.array([4, 235, 255])  
        upper_bound = np.array([4, 235, 255]) 

        mask = cv2.inRange(image, lower_bound, upper_bound) # Find pixels within the bounds and create a mask
        self.mask = mask

        if np.sum(self.mask) == 0:
            if self.publish_proc_sem_img:
                try:
                    ros_image = self.bridge.cv2_to_imgmsg(image, "bgr8")
                    self.proc_sem_img_pub.publish(ros_image) # Publishing processed image
                except CvBridgeError as e:
                    rospy.logerr(f"CvBridge Error in Publishing: {e}")
            return

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) # Find contours from the binary image
        if self.publish_proc_sem_img:
            for contour in contours:
                x, y, w, h = cv2.boundingRect(contour) 
                cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)  # Draw bounding box
            try:
                ros_image = self.bridge.cv2_to_imgmsg(image, "bgr8")
                self.proc_sem_img_pub.publish(ros_image) # Publishing processed image
            except CvBridgeError as e:
                rospy.logerr(f"CvBridge Error in Publishing: {e}")

    
    # Defining point cloud callback 
    def point_cloud_callback(self, data):
        rospy.loginfo('Point Cloud received...')
        try:
            frame_id = data.header.frame_id # Gets frame id from data header
            ## Assume the semantic image and left rgb image are in the same frame so no need for transformation
            # transform = self.tf_buffer.lookup_transform('/Quadrotor/Sensors/RGBCameraLeft', frame_id, rospy.Time(), rospy.Duration(1.0))
            # cloud_transformed = do_transform_cloud(data.cloud, transform)
            pcl_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data) # Convert msg to numpy

            if self.K is None or self.rgb_img is None or self.sem_img is None:
                return
        
            u, v = self.calculate_pixel_coordinates(pcl_array) # Calculate the pixel coords of the point cloud points
             # Draw circles around pixel coordinates.
             # This visualizes where points in the point cloud correspond to locations in the image.

            for u_, v_ in zip(u,v):
                image = cv2.circle(self.rgb_img, (int(u_),int(v_)), radius=0, color=(0, 0, 255), thickness=-1)
            if self.publish_overlayed_rgb_img:
                self.overlayed_image_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))


            if self.mask is not None and np.sum(self)!=0:
  
                u = np.array(u, dtype=int)
                v = np.array(v, dtype=int)

                res_mask = (self.mask[v,u] > 0) & (v > 0) & (u > 0) 

                object_pcl_arr = pcl_array[res_mask]

                pcl_cloud = pcl.PointCloud(object_pcl_arr.astype(np.float32))
                # Applying outlier removal method
                filtered_pcl_cloud = self.do_statistical_outlier_filtering(pcl_cloud, 
                    self.mean_k,self.thresh)                                            
                filtered_np_cloud = np.asarray(filtered_pcl_cloud)

                # Convert pcl_array to a structured array with named fields
                structured_array = np.zeros((filtered_np_cloud.shape[0],), dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32)])
                structured_array['x'] = filtered_np_cloud[:, 0]
                structured_array['y'] = filtered_np_cloud[:, 1]
                structured_array['z'] = filtered_np_cloud[:, 2]

                object_pcl_msg = ros_numpy.point_cloud2.array_to_pointcloud2(structured_array, stamp = data.header.stamp, frame_id = data.header.frame_id)

                pcl_header = data.header
                # Transforms the filtered point cloud to a world coordinate frame
                if self.pub_in_world_coords:
                    try:
                        transform = self.tf_buffer.lookup_transform('world', data.header.frame_id, rospy.Time(), rospy.Duration(1.0))
                    except tf2_ros.LookupException as e:
                        rospy.logerr(e)
                        return

                    object_pcl_cloud = do_transform_cloud(object_pcl_msg, transform)
                    filtered_np_cloud = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(object_pcl_cloud)
                    pcl_header.frame_id = 'world'

                if self.publish_object_pcl:
                    if self.pub_in_world_coords:
                        object_pcl_msg = object_pcl_cloud
                        object_pcl_msg.header = pcl_header
                    self.object_pcl_pub.publish(object_pcl_msg)

                if self.publish_object_bb or self.publish_object_vis_markers:   
                    bb_array = self.create_3d_bb_from_pcl(filtered_np_cloud)

                    if self.pub_in_world_coords:
                        bb_array.detections = self.compare_add_bbs(bb_array)

                    bb_array.header = pcl_header
                    self.object_bb_pub.publish(bb_array)

                if self.publish_object_vis_markers:
                    markers_array = self.create_marker_from_bb(bb_array)
                    self.object_marker_pub.publish(markers_array)
            else:
                bb_array = Detection3DArray()
                pcl_header.frame_id = 'world'
                bb_array.header = pcl_header
                bb_array.detections = self.detected_lights
                if self.publish_object_bb or self.publish_object_vis_markers:  
                    self.object_bb_pub.publish(bb_array)

                if self.publish_object_vis_markers:
                    markers_array = self.create_marker_from_bb(bb_array)
                    self.object_marker_pub.publish(markers_array)

        except Exception as e:
            rospy.logerr(e)


    def compare_add_bbs(self, bb_array):
        # TODO add and compare with self.detected_lights
        for new_detection in bb_array.detections:
            if self.detected_lights is None:
                self.detected_lights = [new_detection]
            else:
                far = True
                for old_detection in self.detected_lights:
                    if self.calculate_distance(new_detection.bbox, old_detection.bbox) < 1:
                        far = False
                        break
                if far:
                    self.detected_lights.append(new_detection)
        return self.detected_lights

    def calculate_distance(self, bb1, bb2):
        pos1 = bb1.center.position
        pos2 = bb2.center.position
        return sqrt((pos1.x - pos2.x)**2 + (pos1.y - pos2.y)**2 + (pos1.z - pos2.z)**2 )


    # Calculating pixel coordinates of point cloud points
    def calculate_pixel_coordinates(self, pcl):
        u = (self.K[0, 0] * pcl[:, 0] / pcl[:, 2]) + self.K[0, 2]
        v = (self.K[1, 1] * pcl[:, 1] / pcl[:, 2]) + self.K[1, 2]
        return u, v
    # Statistical outlier removal method
    def do_statistical_outlier_filtering(self, pcl_data,mean_k,tresh):
        '''
        :param pcl_data: point could data subscriber
        :param mean_k: number of neighboring points to analyze for any given point
        :param tresh: Any point with a mean distance larger than global will be considered outlier
        :return: Statistical outlier filtered point cloud data
        eg) cloud = do_statistical_outlier_filtering(cloud,10,0.001)
        : https://github.com/fouliex/RoboticPerception
        '''
        outlier_filter = pcl_data.make_statistical_outlier_filter()
        outlier_filter.set_mean_k(mean_k)
        outlier_filter.set_std_dev_mul_thresh(tresh)
        return outlier_filter.filter()
    #Creating 3d bounding box around the detected light
    def create_3d_bb_from_pcl(self, pcl):

        x_extent = np.max(pcl[:,0]) - np.min(pcl[:,0])
        y_extent = np.max(pcl[:,1]) - np.min(pcl[:,1])
        z_extent = np.max(pcl[:,2]) - np.min(pcl[:,2])
        
        center_x = np.min(pcl[:,0]) + 0.5*x_extent
        center_y = np.min(pcl[:,1]) + 0.5*y_extent
        center_z = np.min(pcl[:,2]) + 0.5*z_extent

        bb = BoundingBox3D()
        bb.size.x = x_extent
        bb.size.y = y_extent
        bb.size.z = z_extent
        bb.center.position.x = center_x
        bb.center.position.y = center_y
        bb.center.position.z = center_z
        bb.center.orientation.w = 1
  

        detection = Detection3D()
        detection.bbox = bb
        
        detections_arr = Detection3DArray()
        detections_arr.detections.append(detection)

        return detections_arr

    def create_marker_from_bb(self, detections_arr):

        markers_arr = MarkerArray()
        for detection in detections_arr.detections:
            marker = Marker()
            marker.pose = detection.bbox.center
            marker.scale = detection.bbox.size
            marker.type = 1
            marker.header = detections_arr.header
            marker.color.a = 0.5

            markers_arr.markers.append(marker)

        return markers_arr



if __name__ == '__main__':
    rospy.init_node("light_detector_node", anonymous=True)
    LightDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    