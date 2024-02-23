 #!/usr/bin/env python
import cv2
import numpy as np
import pcl
import ros_numpy
import rospy
import tf2_ros
import tf2_geometry_msgs
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from vision_msgs.msg import Detection3DArray, Detection3D, BoundingBox3D
from visualization_msgs.msg import MarkerArray, Marker
from cv_bridge import CvBridge, CvBridgeError
from image_geometry import PinholeCameraModel

class LightDetector(object):
    def __init__(self):
        # Initialize the CvBridge class
        self.bridge = CvBridge()

        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(10)


        self.init_params()


        # Subscribers
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


        # Publishers
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

        # Placeholder for camera intrinsic parameters
        self.K = None

        self.mask = None


    def init_params(self):
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

        self.mean_k = rospy.get_param(
            "/perception_params/mean_k")
        self.thresh = rospy.get_param(
            "/perception_params/thresh")


    def camera_info_callback(self, msg):
        rospy.loginfo('Camera Info received...')
        self.K = np.array(msg.K).reshape(3, 3)
        self.P = np.array(msg.P).reshape(4, 3)

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

            # if self.sem_img is not None:
            #     dst = cv2.addWeighted(cv_image, 0.7, self.sem_img, 0.3, 0.0)
            #     self.overlayed_image_pub.publish(self.bridge.cv2_to_imgmsg(dst, "bgr8"))

        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")

    def process_image(self, image):
        # Example of processing the image:
        # Define lower and upper bounds for the color you're looking for
        lower_bound = np.array([4, 235, 255])  
        upper_bound = np.array([4, 235, 255]) 

        # Find pixels within the bounds and create a mask
        mask = cv2.inRange(image, lower_bound, upper_bound)
        self.mask = mask
        # Optionally, apply the mask to get the segmented object
        segmented_image = cv2.bitwise_and(image, image, mask=mask)

        # Find contours from the binary image
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


        # Display the result
        if self.publish_proc_sem_img:
            # Draw bounding box
            for contour in contours:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)  # Blue bounding box
            try:
                ros_image = self.bridge.cv2_to_imgmsg(image, "bgr8")
                self.proc_sem_img_pub.publish(ros_image)
            except CvBridgeError as e:
                rospy.logerr(f"CvBridge Error in Publishing: {e}")

    
    
    def point_cloud_callback(self, data):
        rospy.loginfo('Point Cloud received...')
        try:
            frame_id = data.header.frame_id

            ## Assume the semantic image and left rgb image are in the same frame so no need for transformation
            # transform = tf_buffer.lookup_transform('/Quadrotor/Sensors/RGBCameraLeft', frame_id, rospy.Time(), rospy.Duration(1.0))
            # cloud_transformed = tf2_ros.do_transform_cloud(data, transform)

            # convert msg to numpy
            pcl_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data)

            # calculate the pixel coords
            u, v = self.calculate_pixel_coordinates(pcl_array)

            for u_, v_ in zip(u,v):
                image = cv2.circle(self.rgb_img, (int(u_),int(v_)), radius=0, color=(0, 0, 255), thickness=-1)
            if self.publish_overlayed_rgb_img:
                self.overlayed_image_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))


            if self.mask is not None:
  
                u = np.array(u, dtype=int)
                v = np.array(v, dtype=int)

                res_mask = (self.mask[v,u] > 0) & (v > 0) & (u>0) 
                valid_mask = [v>0, u>0]

                object_pcl_arr = pcl_array[res_mask]

                pcl_cloud = pcl.PointCloud(object_pcl_arr.astype(np.float32))
    
                filtered_pcl_cloud = self.do_statistical_outlier_filtering(pcl_cloud, 
                    self.mean_k,self.thresh)
                filtered_np_cloud = np.asarray(filtered_pcl_cloud)

                # Convert pcl_array to a structured array with named fields
                structured_array = np.zeros((filtered_np_cloud.shape[0],), dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32)])
                structured_array['x'] = filtered_np_cloud[:, 0]
                structured_array['y'] = filtered_np_cloud[:, 1]
                structured_array['z'] = filtered_np_cloud[:, 2]

                if self.publish_object_pcl:
                    object_pcl_msg = ros_numpy.point_cloud2.array_to_pointcloud2(structured_array, stamp = data.header.stamp, frame_id = data.header.frame_id)
                    self.object_pcl_pub.publish(object_pcl_msg)

                if self.publish_object_bb or self.publish_object_vis_markers:   
                    bb_array = self.create_3d_bb_from_pcl(filtered_np_cloud)
                    bb_array.header = data.header
                    self.object_bb_pub.publish(bb_array)

                if self.publish_object_vis_markers:
                    markers_array = self.create_marker_from_bb(bb_array)
                    self.object_marker_pub.publish(markers_array)

        except tf2_ros.LookupException as e:
            rospy.logerr(e)

    def calculate_pixel_coordinates(self, pcl):
        u = (self.K[0, 0] * pcl[:, 0] / pcl[:, 2]) + self.K[0, 2]
        v = (self.K[1, 1] * pcl[:, 1] / pcl[:, 2]) + self.K[1, 2]
        return u, v
    
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
    