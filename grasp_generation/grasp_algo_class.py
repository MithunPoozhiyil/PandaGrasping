import os
import numpy as np
import glob
import os
import sys
import config_utils
from contact_grasp_estimator import GraspEstimator
from visualization_utils import visualize_grasps, show_image
from data import regularize_pc_point_count, depth2pc, load_available_input_data

import tensorflow.compat.v1 as tf
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
from tensorflow.python.util import deprecation
deprecation._PRINT_DEPRECATION_WARNINGS = False

class GraspAlgo:
    def __init__(self):
        tf.disable_eager_execution()
        physical_devices = tf.config.experimental.list_physical_devices('GPU')
        tf.config.experimental.set_memory_growth(physical_devices[0], True)
        BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        sys.path.append(os.path.join(BASE_DIR))

        self.load_variables()
        self.load_model()

    def load_variables(self):
        self.checkpoint_dir = "/home/mithun/ws_graspnet/src/contact_graspnet_ros/contact_graspnet/checkpoints/scene_test_2048_bs3_hor_sigma_001"
        self.dataFile = "/home/mithun/Desktop/4.npy"
        self.input_paths = self.dataFile
        self.forwardPasses = 1
        self.globalConfig = config_utils.load_config(self.checkpoint_dir, batch_size=self.forwardPasses)
        self.local_regions = False
        self.skip_border_objects = False
        self.filter_grasps = True
        self.z_range = [0.2, 1.5]


    def load_model(self):
        # Build the model
        self.grasp_estimator = GraspEstimator(self.globalConfig)
        self.grasp_estimator.build_network()

        # Add ops to save and restore all the variables.
        saver = tf.train.Saver(save_relative_paths=True)

        # Create a session
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        config.allow_soft_placement = True
        self.sess = tf.Session(config=config)

        # Load weights
        self.grasp_estimator.load_weights(self.sess, saver, self.checkpoint_dir, mode='test')

    def inference(self, segmap, rgb, depth, cam_K, pc_full):

        """
        Predict 6-DoF grasp distribution for given model and input data

        :param global_config: config.yaml from checkpoint directory
        :param checkpoint_dir: checkpoint directory
        :param input_paths: .png/.npz/.npy file paths that contain depth/pointcloud and optionally intrinsics/segmentation/rgb
        :param K: Camera Matrix with intrinsics to convert depth to point cloud
        :param local_regions: Crop 3D local regions around given segments.
        :param skip_border_objects: When extracting local_regions, ignore segments at depth map boundary.
        :param filter_grasps: Filter and assign grasp contacts according to segmap.
        :param segmap_id: only return grasps from specified segmap_id.
        :param z_range: crop point cloud at a minimum/maximum z distance from camera to filter out outlier points. Default: [0.2, 1.8] m
        :param forward_passes: Number of forward passes to run on each point cloud. Default: 1
        """
        os.makedirs('results', exist_ok=True)
        pc_segments = {}
        depth = depth / 1000
        print("Image size: ", rgb.shape, "Depth size: ", depth.shape)

        if segmap is None and (self.local_regions or self.filter_grasps):
            raise ValueError('Need segmentation map to extract local regions or filter grasps')

        if pc_full is None:
            print('Converting depth to point cloud(s)...')
            # segmap = None
            pc_full, pc_segments, pc_colors = self.grasp_estimator.extract_point_clouds(depth, cam_K, segmap=segmap,
                                                                                   rgb=rgb,
                                                                                   skip_border_objects=self.skip_border_objects,
                                                                                   z_range=self.z_range)
        print('Generating Grasps...')
        # print("PC_REGION: ", pc_segments.keys())
        pred_grasps_cam, scores, contact_pts, _ = self.grasp_estimator.predict_scene_grasps(self.sess, pc_full,
                                                                                       pc_segments=pc_segments,
                                                                                       local_regions=self.local_regions,
                                                                                       filter_grasps=self.filter_grasps,
                                                                                       forward_passes=self.forwardPasses)

        # # Save results
        # np.savez('results/predictions_{}'.format(os.path.basename(p.replace('png', 'npz').replace('npy', 'npz'))),
        #          pred_grasps_cam=pred_grasps_cam, scores=scores, contact_pts=contact_pts)
        # print(pred_grasps_cam, scores, contact_pts, pc_full.shape)
        print('Grasps Generated...')

        # Visualize results
        # show_image(rgb, segmap)
        # visualize_grasps(pc_full, pred_grasps_cam, scores, plot_opencv_cam=True, pc_colors=pc_colors)
        return pred_grasps_cam, scores

