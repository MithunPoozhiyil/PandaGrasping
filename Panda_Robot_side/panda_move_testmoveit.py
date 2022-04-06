#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import tf
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
import math
import time
import numpy as np
from franka_control.msg import ErrorRecoveryActionGoal

class Robot(object):
    def __init__(self):
        print("============ Initialising ROS nodes...")
        self.listener = tf.TransformListener()

        #Instantiate a RobotCommander object. This object is an interface to the robot as a whole.
        self.robot = moveit_commander.RobotCommander()

        #Instantiate a PlanningSceneInterface object. This object is an interface to the world surrounding the robot.
        self.scene = moveit_commander.PlanningSceneInterface()

        #Instantiate a MoveGroupCommander object. This object is an interface to one group of joints
        self.arm = moveit_commander.MoveGroupCommander("panda_arm")
        # self.arm = moveit_commander.MoveGroupCommander("panda_arm_hand")
    

        #Set the name of the link to be considered as an end effector for above group
        # self.arm.set_end_effector_link("panda_hand")
        self.arm.set_end_effector_link("panda_gripper")

        self.hand = moveit_commander.MoveGroupCommander("hand")


        self.arm.set_max_velocity_scaling_factor(0.50)  # scaling down velocity
        self.arm.set_max_acceleration_scaling_factor(0.5)  # scaling down velocity
        self.arm.allow_replanning(True)
        self.arm.set_num_planning_attempts(10)
        self.arm.set_goal_position_tolerance(0.0005)
        self.arm.set_goal_orientation_tolerance(0.001)
        self.arm.set_planning_time(5)

        self.arm.set_planner_id("FMTkConfigDefault")
        # self.arm.set_planner_id("RRTstarkConfigDefault")

        #DisplayTrajectory publisher which is used below to publish trajectories for RVIZ to visualize.
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)
        self.FrankaRecoveryPub = rospy.Publisher('/franka_control/error_recovery/goal', ErrorRecoveryActionGoal,
                                                       queue_size=10)

        self.marker_trans = []
        self.marker_rot = []
        self.aruco_marker_trans = []
        self.aruco_marker_rot = []
        self.grasp_pos_array = PoseArray
        self.initial_pos = PoseStamped
        self.lift_way_point = np.array([0.41, -0.01, 0.35])
        self.drop_point = np.array([0.49, 0.40, 0.25])
        # self.marker_sub = rospy.Subscriber('/our/test/pose_stamped', PoseStamped, self.get_whycon_pos)
        # self.marker_sub = rospy.Subscriber('/aruco_single/pose', PoseStamped, self.get_aruco_pos)
        self.grasp_sub = rospy.Subscriber('/grasp_points', PoseArray, self.get_grasp_pos_array)

        # Pet a list of all the groups in the robot:
        self.group_names = self.robot.get_group_names()
        print("============ Robot Groups:", self.group_names)

        # Pet the name of the reference frame for this robot:
        print("============ Reference frame: %s" % self.arm.get_planning_frame())

        # Print the name of the end-effector link for this group
        print ("============ end-effector frame: %s" % self.arm.get_end_effector_link())

        # Print entire state of the robot
        print("============ Printing robot state")
        print (self.robot.get_current_state())

        # Print current pose of the robot
        print("============ Printing robot pose")
        print(self.arm.get_current_pose())
        print("")
        self.br = tf.TransformBroadcaster()

    def get_robot_initial_pose(self):
        print("============Saving Initial pose============")
        arm = self.arm
        self.initial_pos = arm.get_current_pose()

    def get_grasp_pos_array(self, msg):
        self.grasp_pos_array = msg
      

    def get_transform(self, from_tf, to_tf):
        self.listener.waitForTransform(from_tf, to_tf, rospy.Time().now(), rospy.Duration(10.0))
        return self.listener.lookupTransform(from_tf, to_tf, rospy.Time(0))

    def execute_way_point(self):
        arm = self.arm
        waypoints = []
        scale = 2
        wpose = arm.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
        (plan, fraction) = arm.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold
        arm.execute(plan, wait=True)

    def go_to_position(self,point):
        print('Moving for image capture')
        arm = self.arm
       
        p = PoseStamped()
        p.header.frame_id = '/panda_link0'

        p.pose.position.x = point[0]
        p.pose.position.y = point[1]
        p.pose.position.z = point[2]

        p.pose.orientation.x = self.initial_pos.pose.orientation.x
        p.pose.orientation.y = self.initial_pos.pose.orientation.y
        p.pose.orientation.z = self.initial_pos.pose.orientation.z
        p.pose.orientation.w = self.initial_pos.pose.orientation.w
        #----------------------------------------------
        self.br.sendTransform((p.pose.position.x, p.pose.position.y, p.pose.position.z), ( p.pose.orientation.x,
                                                                                           p.pose.orientation.y,
                                                                      p.pose.orientation.z,  p.pose.orientation.w),
                              rospy.Time.now(), "Way point", "panda_link0")

        target = arm.set_pose_target(p)
        # plan_grasp = self.arm.plan()
        # print("============ Waiting while RVIZ displays plan1...")
        # rospy.sleep(5)
        # print("============ Visualizing plan_home")
        # display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        #
        # display_trajectory.trajectory_start = self.robot.get_current_state()
        # display_trajectory.trajectory.append(plan_grasp)
        # self.display_trajectory_publisher.publish(display_trajectory)
        #
        # print("============ Waiting while plan1 is visualized (again)...")
        # rospy.sleep(5)
        arm.go(target, wait=True)
        arm.stop()
        arm.clear_pose_targets()

    def open_gripper(self):
        print('Opening Gripper')
        hand = self.hand
        gripper_state = hand.get_current_joint_values()
        gripper_state[0] = 0.03
        gripper_state[1] = 0.03
        hand.go(gripper_state, wait=True)
        hand.stop()

    def close_gripper(self):
        print('Closing Gripper')
        hand = self.hand
        gripper_state = hand.get_current_joint_values()
        gripper_state[0] = 0.00
        gripper_state[1] = 0.00
        hand.go(gripper_state, wait=True)
        hand.stop()
 

    def initiate_grasp(self,trans,rot):
        arm = self.arm
        T_link0_camera = self.listener.fromTranslationRotation(trans, rot)
        orient_end_eff = tf.transformations.quaternion_from_matrix(T_link0_camera)
        str_i = 0
        # print("no. of. objects to pick",len(self.grasp_pos_array.poses))
        for objects in self.grasp_pos_array.poses:
            T_point_camera = self.listener.fromTranslationRotation([objects.position.x, objects.position.y,
                                                                    objects.position.z], [objects.orientation.x,
                                                                                         objects.orientation.y,
                                                                                         objects.orientation.z,
                                                                                         objects.orientation.w])
            # T_point_camera = self.listener.fromTranslationRotation([0.46995, -0.0828, 0.0549], [objects.orientation.x,
            #                                                                              objects.orientation.y,
            #                                                                              objects.orientation.z,
            #                                                                              objects.orientation.w])

            #-----------------------------------------------------------------#
            tt_quat = quaternion_from_euler(0, 0, math.radians(90))
            rotation_temp = self.listener.fromTranslationRotation([0, 0, 0], [tt_quat[0],
                                                                                         tt_quat[1],
                                                                                         tt_quat[2],
                                                                                         tt_quat[3]])
            #-----------------------------------------------------------------#
            T_link0_point = np.dot(T_link0_camera, T_point_camera)
            # T_link0_point = np.dot(T_link0_point, rotation_temp)
            pos_i = T_link0_point[0:3, 3]
            orient_i = tf.transformations.quaternion_from_matrix(T_link0_point)
            p = PoseStamped()
            p.header.frame_id = '/panda_link0'

            # p.pose.position.x = pos_i[0]-0.05
            # p.pose.position.y = pos_i[1]
            # # p.pose.position.z = pos_i[2] + 0.15
            # # p.pose.position.z = pos_i[2]-0.05
            # p.pose.position.z = pos_i[2]+0.005-0.03

            p.pose.position.x = pos_i[0]-0.025
            p.pose.position.y = pos_i[1]+0.01
            # p.pose.position.z = pos_i[2] + 0.15
            p.pose.position.z = pos_i[2]-0.07
            # p.pose.position.z = pos_i[2]


            # 0.46995768599329274, -0.082892089799845886, 0.054932158385593788


            # print('going to pose', p.pose.position.x, p.pose.position.y, p.pose.position.z)
            p.pose.orientation.x = orient_i[0]
            p.pose.orientation.y = orient_i[1]
            p.pose.orientation.z = orient_i[2]
            p.pose.orientation.w = orient_i[3]

            # p.pose.orientation.x = self.initial_pos.pose.orientation.x
            # p.pose.orientation.y = self.initial_pos.pose.orientation.y
            # p.pose.orientation.z = self.initial_pos.pose.orientation.z
            # p.pose.orientation.w = self.initial_pos.pose.orientation.w
            #-------------------------------------------------------------------------
            obj_orien = euler_from_quaternion([p.pose.orientation.x, p.pose.orientation.y,
                                              p.pose.orientation.z, p.pose.orientation.w])
            end_orien = euler_from_quaternion([self.initial_pos.pose.orientation.x, self.initial_pos.pose.orientation.y,
                                              self.initial_pos.pose.orientation.z, self.initial_pos.pose.orientation.w])

            #-------------------------------------------------------------------------

            self.br.sendTransform((p.pose.position.x, p.pose.position.y, p.pose.position.z),
                                  (p.pose.orientation.x, p.pose.orientation.y,
                                   p.pose.orientation.z, p.pose.orientation.w),
                                  rospy.Time.now(), "new_object"+str(str_i), "panda_link0")
            if abs(math.degrees(obj_orien[2]) - math.degrees(end_orien[2])) < 81 and abs(math.degrees(obj_orien[1]) - math.degrees(end_orien[1])) < 40 and p.pose.position.x < 0.5:
            # if abs(math.degrees(obj_orien[2]) - math.degrees(end_orien[2])) < 120 and abs(math.degrees(obj_orien[1]) - math.degrees(end_orien[1])) < 90 and p.pose.position.x < 0.7:
                print("grasping"+str(str_i))

                wpose = arm.get_current_pose().pose
                wpose.position.x = p.pose.position.x
                target = arm.set_pose_target(wpose)
                plan_grasp = self.arm.plan()
                arm.go(target, wait=True)
                arm.stop()
                arm.clear_pose_targets()

                wpose = arm.get_current_pose().pose
                wpose.position.y = p.pose.position.y
                target = arm.set_pose_target(wpose)
                plan_grasp = self.arm.plan()
                arm.go(target, wait=True)
                arm.stop()
                arm.clear_pose_targets()

                wpose = arm.get_current_pose().pose
                wpose.orientation.x = p.pose.orientation.x
                wpose.orientation.y = p.pose.orientation.y
                wpose.orientation.z = p.pose.orientation.z
                wpose.orientation.w = p.pose.orientation.w
                target = arm.set_pose_target(wpose)
                plan_grasp = self.arm.plan()
                arm.go(target, wait=True)
                arm.stop()
                arm.clear_pose_targets()

                wpose = arm.get_current_pose().pose
                wpose.position.z = p.pose.position.z
                target = arm.set_pose_target(wpose)
                plan_grasp = self.arm.plan()
                arm.go(target, wait=True)
                arm.stop()
                arm.clear_pose_targets()

                #-------------------------------------------------------
                # target = arm.set_pose_target(p)
                # plan_grasp = self.arm.plan()
                # print("============ Waiting while RVIZ displays plan1...")
                # rospy.sleep(5)
                # print("============ Visualizing plan_home")
                # display_trajectory = moveit_msgs.msg.DisplayTrajectory()
                #
                # display_trajectory.trajectory_start = self.robot.get_current_state()
                # display_trajectory.trajectory.append(plan_grasp)
                # self.display_trajectory_publisher.publish(display_trajectory)
                #
                # print("============ Waiting while plan1 is visualized (again)...")
                # rospy.sleep(5)

                # arm.go(target, wait=True)
                # arm.stop()
                # arm.clear_pose_targets()
                error_msg = ErrorRecoveryActionGoal()
                self.FrankaRecoveryPub.publish(error_msg)
                time.sleep(2)
                self.close_gripper()
                error_msg = ErrorRecoveryActionGoal()
                self.FrankaRecoveryPub.publish(error_msg)
                time.sleep(2)

                wpose = arm.get_current_pose().pose
                wpose.position.z = wpose.position.z + 0.25
                target = arm.set_pose_target(wpose)
                plan_grasp = self.arm.plan()
                arm.go(target, wait=True)
                arm.stop()
                arm.clear_pose_targets()

                # self.go_to_position(self.lift_way_point)
                self.go_to_position(self.drop_point)
                self.open_gripper()
                self.go_to_position(self.lift_way_point)
            else:
                print("more angle for object"+str(str_i), abs(math.degrees(obj_orien[2]) - math.degrees(end_orien[2])))
            str_i = str_i + 1

def main():
    rospy.init_node('Panda_move_test', anonymous=True)
    panda = Robot()
    rate = rospy.Rate(100)
    panda.get_robot_initial_pose()
    time.sleep(1)
    # image_capture_point = np.array([0.40, -0.08, 0.53])
    image_capture_point = np.array([0.321, 0.0388, 0.3475])
    test_pose = np.array([0.5300, 0.0072, 0.0704-0.03])

    while not rospy.is_shutdown():
        # #-----Go to image capture point-------#
        panda.go_to_position(image_capture_point)
        panda.open_gripper()
        time.sleep(10)
        (trans, rot) = panda.get_transform('panda_link0', '_camera_color_optical_frame')
        trans_tmp = (trans[0],trans[1],trans[2])
        rot_tmp = (rot[0],rot[1],rot[2],rot[3])
        panda.initiate_grasp(trans_tmp, rot_tmp)
        rate.sleep()
        # break



if __name__ == '__main__':
    try:
        moveit_commander.roscpp_initialize(sys.argv)
        main()
    except Exception as e:
        print(e)
    finally:
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


