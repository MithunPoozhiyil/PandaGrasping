#!/usr/bin/env python

import numpy as np
import cv2

from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import rospy
from grasp_algo_class import GraspAlgo
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from std_msgs.msg import Float64MultiArray

class GraspModule:
    def __init__(self):
        # initialize a node
        # rospy.init_node("grasp_node")
        self.bridge = CvBridge()
        image_topic = "/camera/color/image_raw"
        depth_topic = "/camera/aligned_depth_to_color/image_raw"
        camera_info_topic = "/camera/aligned_depth_to_color/camera_info"
        seg_map_topic = "/seg_label"
        self.color_frame_id = ""
        rospy.Subscriber(image_topic, Image, self.img_rgb_callback)
        rospy.Subscriber(depth_topic, Image, self.img_depth_callback)
        rospy.Subscriber(camera_info_topic, CameraInfo, self.camera_info_callback)
        rospy.Subscriber(seg_map_topic, Image, self.seg_map_info_callback)
        self.grasp_pub = rospy.Publisher('grasp_points', PoseArray, queue_size=10)
        self.img_rgb = None
        self.image_depth = None
        self.camera_matrix = None
        self.img_xyz = None
        self.segmap = None
        self.grasp_generation = GraspAlgo()
        self.handle_loop()
        rospy.spin()

    def handle_loop(self):

        while not rospy.is_shutdown():
            try:
                b = np.count_nonzero(self.segmap)
                # if not (isinstance(self.image_depth, type(None)))   and not (isinstance(self.img_rgb, type(None)))  \
                #         and not (isinstance(self.camera_matrix, type(None))) and b != 0 and not (isinstance(self.img_xyz, type(None))):
                if not (isinstance(self.image_depth, type(None)))   and not (isinstance(self.img_rgb, type(None)))  \
                        and not (isinstance(self.camera_matrix, type(None))) and b != 0:
                #     print ( self.img_rgb.shape, self.image_depth.shape)

                    # cv2.imwrite("/home/mithun/Desktop/image.PNG", self.img_rgb)
                    # cv2.imshow( 'Image', self.img_rgb)
                    # k = cv2.waitKey(1)
                    # print(self.img_xyz)

                    pred_grasps_cam, scores = self.grasp_generation.inference(self.segmap, self.img_rgb, self.image_depth, self.camera_matrix,
                                               self.img_xyz)
                    # print(pred_grasps_cam.keys())
                    # print(scores.keys())
                    # grasp_msg = Float64MultiArray()

                    grasp_msg = PoseArray()
                    grasp_msg.header.frame_id = self.color_frame_id
                    grasp_msg.header.stamp = rospy.Time().now()

                    # total_grasps = []
                    for x in pred_grasps_cam:
                        grasps = pred_grasps_cam.get(x)
                        score_val = scores.get(x)
                        # print('grasps', grasps)
                        # print('score', score_val)
                        if len(score_val) > 0:
                            max_score_index = np.argmax(score_val)
                            optimal_grasp = grasps[max_score_index]

                            grasp_position = np.array(optimal_grasp[:3, 3], dtype=float)

                            # print('type_grasp_position', type(grasp_position))

                            grasp_rot = np.array([optimal_grasp[0][:3], optimal_grasp[1][:3], optimal_grasp[2][:3]], dtype=float)

                            grasp_rot_matrix = R.from_matrix(grasp_rot)
                            grasp_quat = grasp_rot_matrix.as_quat()
                            # print('type_grasp_quat', type(grasp_quat))
                            grasp_pos = np.concatenate((grasp_position, grasp_quat))
                            grasp_pos = grasp_pos.tolist()

                            pose_grasp = Pose()
                            pose_grasp.position.x = grasp_pos[0]
                            pose_grasp.position.y = grasp_pos[1]
                            pose_grasp.position.z = grasp_pos[2]
                            pose_grasp.orientation.x = grasp_pos[3]
                            pose_grasp.orientation.y = grasp_pos[4]
                            pose_grasp.orientation.z = grasp_pos[5]
                            pose_grasp.orientation.w = grasp_pos[6]

                            grasp_msg.poses.append( pose_grasp )

                            # print('type_grasp_pos',type(grasp_pos))
                            # print('grasp_pos',grasp_pos)

                            # total_grasps.append(grasp_pos)
                    # print('total_grasps',total_grasps)
                    # print('type', type(total_grasps[0][6]))

                    # grasp_msg.data = total_grasps
                    self.grasp_pub.publish(grasp_msg)
            except KeyboardInterrupt:
                cv2.destroyAllWindows()
                break


    def img_rgb_callback(self,msg):
        # rospy.loginfo("RGB CALLBACK")
        self.img_rgb = self.bridge.imgmsg_to_cv2(msg)
        self.img_rgb = cv2.cvtColor(self.img_rgb, cv2.COLOR_RGB2BGR)
        self.color_frame_id = msg.header.frame_id

    def img_depth_callback(self, msg):
        # rospy.loginfo("DEPTH CALLBACK")
        self.image_depth = self.bridge.imgmsg_to_cv2(msg)

    def camera_info_callback(self,msg):
        # rospy.loginfo("Cmaera info CALLBACK")
        self.camera_matrix = np.array(msg.K).reshape(3, 3)

        # self.fx = intrinsics[0, 0]
        # self.fy = intrinsics[1, 1]
        # self.px = intrinsics[0, 2]
        # self.py = intrinsics[1, 2]
        # print("receiving cam_info")
    def seg_map_info_callback(self, msg):
        # rospy.loginfo("DEPTH CALLBACK")
        self.segmap = self.bridge.imgmsg_to_cv2(msg)
        # print('segmap_size',self.segmap.shape)


# -----------------Not used now-------------------------------#
    def compute_xyz(self,depth_img, fx, fy, px, py, height, width):
        indices = np.indices((height, width), dtype=np.float32).transpose(1, 2, 0)
        z_e = depth_img
        x_e = (indices[..., 1] - px) * z_e / fx
        y_e = (indices[..., 0] - py) * z_e / fy
        self.img_xyz = np.stack([x_e, y_e, z_e], axis=-1)  # Shape: [H x W x 3]


    def test(self):
        hh = 5



if __name__ == "__main__":

    rospy.init_node("graph_generator_node")
    grasp_module = GraspModule()
    grasp_module.handle_loop()

    # grasp_generation = GraspAlgo()
    # grasp_generation.inference(camera_input.segmap, camera_input.img_rgb, camera_input.image_depth, camera_input.camera_matrix, camera_input.img_xyz, camera_input.pc_color)
    # while not rospy.is_shutdown():
    #     grasp_generation_obj.test()
        # grasp_generation_obj.inference(globalConfig, ckptDir, dataFile)
    # inference(globalConfig, ckptDir, dataFile)

