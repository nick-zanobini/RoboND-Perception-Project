#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster


# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"] = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict


# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)


# Helpder function to merge two point clouds
def merge_clouds(cloud1, cloud2):
    merged_cloud_list = []

    for point in cloud1:
        merged_cloud_list.append(point)
    for point in cloud2:
        merged_cloud_list.append(point)

    merged_cloud = pcl.PointCloud_PointXYZRGB()
    merged_cloud.from_list(merged_cloud_list)
    return merged_cloud


# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

    # Exercise-2 TODOs:

    # TODO: Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)

    # Much like the previous filters, we start by creating a filter object:
    outlier_filter = cloud.make_statistical_outlier_filter()

    # Set the number of neighboring points to analyze for any given point
    outlier_filter.set_mean_k(50)

    # Set threshold scale factor
    x = 1.0

    # Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
    outlier_filter.set_std_dev_mul_thresh(x)

    # Finally call the filter function for magic
    cloud_filtered = outlier_filter.filter()

    # TODO: Voxel Grid Downsampling
    vox = cloud_filtered.make_voxel_grid_filter()

    # Choose a voxel (also known as leaf) size
    LEAF_SIZE = 0.01

    # Set the voxel (or leaf) size
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)

    # Call the filter function to obtain the resultant downsampled point cloud
    cloud_filtered = vox.filter()

    # TODO: PassThrough Filter in z axis
    # Create a PassThrough filter object.
    passthrough_z = cloud_filtered.make_passthrough_filter()

    # Assign axis and range to the passthrough filter object.
    filter_axis = 'z'
    passthrough_z.set_filter_field_name(filter_axis)
    axis_min = 0.609
    axis_max = 1.4
    passthrough_z.set_filter_limits(axis_min, axis_max)

    # Finally use the filter function to obtain the resultant point cloud.
    cloud_filtered_1 = passthrough_z.filter()

    # TODO: PassThrough Filter in y axis
    # Create a PassThrough filter object.
    passthrough_y = cloud_filtered_1.make_passthrough_filter()

    # Assign axis and range to the passthrough filter object.
    filter_axis = 'y'
    passthrough_y.set_filter_field_name(filter_axis)
    axis_min = -0.50
    axis_max = 0.50
    passthrough_y.set_filter_limits(axis_min, axis_max)

    # Finally use the filter function to obtain the resultant point cloud.
    cloud_filtered = passthrough_y.filter()

    # TODO: RANSAC Plane Segmentation
    # Create the segmentation object
    seg = cloud_filtered.make_segmenter()

    # Set the model you wish to fit
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)

    # Max distance for a point to be considered fitting the model
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)

    # TODO: Extract inliers and outliers
    # Call the segment function to obtain set of inlier indices and model coefficients
    inliers, coefficients = seg.segment()
    # Extract inliers
    table_cloud = cloud_filtered.extract(inliers, negative=False)
    # Extract outliers
    objects_cloud = cloud_filtered.extract(inliers, negative=True)

    # TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(objects_cloud)
    tree = white_cloud.make_kdtree()

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()

    # Set tolerances for distance threshold
    # as well as minimum and maximum cluster size (in points)
    # NOTE: These are poor choices of clustering parameters
    # Your task is to experiment and find values that work for segmenting objects.
    # 0.01, 350, 50000 - See 3 middle objects
    ec.set_ClusterTolerance(0.014)
    ec.set_MinClusterSize(20)
    ec.set_MaxClusterSize(2000)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()

    # Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    # Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # TODO: Convert PCL data to ROS messages
    ros_objects_cloud = pcl_to_ros(objects_cloud)
    ros_table_cloud = pcl_to_ros(table_cloud)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

    # TODO: Publish ROS messages
    pcl_objects_pub.publish(ros_objects_cloud)
    pcl_table_pub.publish(ros_table_cloud)
    pcl_cluster_pub.publish(ros_cluster_cloud)

    # Exercise-3 TODOs:

    detected_objects_labels = []
    detected_objects = []

    # Classify the clusters! (loop through each detected cluster one at a time)
    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster
        pcl_cluster = objects_cloud.extract(pts_list)
        pcl_cloud = pcl_to_ros(pcl_cluster)

        # Compute the associated feature vector
        chists = compute_color_histograms(pcl_cloud, using_hsv=True)
        normals = get_normals(pcl_cloud)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))

        # Make the prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1, -1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label, label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = pcl_cloud
        detected_objects.append(do)

        # Publish the list of detected objects
    detected_objects_pub.publish(detected_objects)

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        pr2_mover(detected_objects, ros_table_cloud)
    except rospy.ROSInterruptException:
        pass


# function to load parameters and request PickPlace service
def pr2_mover(object_list, table_cloud):

    # TODO: Initialize variables
    test_scene_num = Int32()
    object_name = String()
    arm_name = String()
    pick_pose = Pose()
    place_pose = Pose()

    dict_list = []
    object_name_list = []
    object_group_list = []
    dropbox_name_list = []
    dropbox_group_list = []
    dropbox_position_list = []

    # store items that have been picked up.
    avoidance_pick_list = object_list

    # TODO: Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    dropbox_list_param = rospy.get_param('/dropbox')

    # TODO: Parse parameters into individual variables
    for i, _ in enumerate(object_list_param):
        object_name_list.append(object_list_param[i]['name'])
        object_group_list.append(object_list_param[i]['group'])

    for k, _ in enumerate(dropbox_list_param):
        dropbox_name_list.append(dropbox_list_param[k]['name'])
        dropbox_group_list.append(dropbox_list_param[k]['group'])
        dropbox_position_list.append(dropbox_list_param[k]['position'])

    if len(object_name_list) == 3:
        test_scene_num.data = 1
    elif len(object_name_list) == 5:
        test_scene_num.data = 2
    elif len(object_name_list) == 8:
        test_scene_num.data = 3

    yaml_filename = "output_" + str(test_scene_num.data)

    # TODO: Rotate PR2 in place to capture side tables for the collision map
    pr2_move_pub.publish(Float64(np.pi / 2))
    rospy.sleep(15)
    pr2_move_pub.publish(Float64(0.0))
    rospy.sleep(15)
    pr2_move_pub.publish(Float64(-np.pi / 2))
    rospy.sleep(15)
    pr2_move_pub.publish(Float64(0.0))
    rospy.sleep(15)

    # TODO: Loop through the pick list
    for obj in object_list_param:
        for detected_obj in object_list:
            if obj['name'] == detected_obj.label:
                # TODO: Get the PointCloud for a given object and obtain it's centroid
                object_name.data = obj['name']
                points_arr = ros_to_pcl(detected_obj.cloud).to_array()
                center = np.mean(points_arr, axis=0)[:3]
                pick_pose.position.x = np.asscalar(center[0])
                pick_pose.position.y = np.asscalar(center[1])
                pick_pose.position.z = np.asscalar(center[2])

                # Get cloud with table data and objects not trying to pick up.
                avoidance_pick_list.remove(detected_obj)
                avoidance_cloud = ros_to_pcl(table_cloud)
                for obstacle in object_list:
                    avoidance_cloud = merge_clouds(avoidance_cloud, ros_to_pcl(obstacle.cloud))

                # Publish PointCloud2 for Collision Avoidance
                pr2_obj_detect_pub.publish(pcl_to_ros(avoidance_cloud))

                break

        # TODO: Create 'place_pose' for the object
        dropbox_index = dropbox_group_list.index(obj['group'])
        place_pose.position.x = dropbox_position_list[dropbox_index][0]
        place_pose.position.y = dropbox_position_list[dropbox_index][1]
        place_pose.position.z = dropbox_position_list[dropbox_index][2]

        # TODO: Assign the arm to be used for pick_place
        arm_name.data = dropbox_name_list[dropbox_index]

        # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
        yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
        dict_list.append(yaml_dict)

    # Wait for 'pick_place_routine' service to come up
    rospy.wait_for_service('pick_place_routine')

    try:
        pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

        # TODO: Insert your message variables to be sent as a service request
        resp = pick_place_routine(test_scene_num, arm_name, object_name, pick_pose, place_pose)

        print ("Response: ", resp.success)

    except rospy.ServiceException, e:
        print("Service call failed: %s" % e)

    # TODO: Output your request parameters into output yaml file
    send_to_yaml(yaml_filename, dict_list)


if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('clustering', anonymous=True)

    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)

    # TODO: Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)
    pr2_move_pub = rospy.Publisher('/pr2/world_joint_controller/command', Float64, queue_size=3)
    pr2_obj_detect_pub = rospy.Publisher('/pr2/3d_map/points', PointCloud2, queue_size=1)

    # TODO: Load Model From disk
    model = pickle.load(open('model_5000.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
