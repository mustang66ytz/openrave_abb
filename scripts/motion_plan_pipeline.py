#!/usr/bin/env python
import openravepy as orpy
import time
import numpy as np
import tf.transformations as tr
import rospy
import roslib
import actionlib
import math

from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from geometry_msgs.msg import (Point, Quaternion, Pose, Vector3, Transform, Wrench)
from sensor_msgs.msg import CameraInfo, Image, RegionOfInterest, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import test_irb1200_model

class MotionPlan(object):
    def __init__(self):
        self.env = orpy.Environment()
        self.env.SetViewer('qtcoin')
        self.env.SetDefaultViewer()
        self.robot = None
        self.manipulator = None
        self.links = None
        self.curstate = []
        self.init_config = []
        self.goal_config = []
        self.optimal_path = JointTrajectory()

    '''
    this function defines a planning environment environment: a pile of boxes
    '''
    def planning_env(self):
        env_obj = orpy.RaveCreateKinBody(self.env, '')
        env_obj.SetName('over_hang_obstacles')
        env_obj.InitFromBoxes(np.array(
            [[0.6, 0.3, 0.86, 0.1, 0.1, 0.1], [0.6, -0.2, 0.86, 0.1, 0.1, 0.1], [0.6, 0.3, 1.06, 0.1, 0.1, 0.1],
             [0.6, -0.2, 1.06, 0.1, 0.1, 0.1], [0.6, 0.3, 1.26, 0.1, 0.1, 0.1], [0.6, -0.2, 1.26, 0.1, 0.1, 0.1],
             [0.61, 0.05, 1.53, 0.11, 0.21, 0.17]]), True)
        return env_obj

    '''
    this function loads an environment and add the defined environment in the planning_env function
    '''
    def set_env(self):
        #self.env.Load('../worlds/boxes_irb4600_task.env.xml')
        self.env.Load('../worlds/boxes_irb1200_task.env.xml')
        self.env.AddKinBody(self.planning_env())
        self.robot = self.env.GetRobot('robot')
        self.manipulator = self.robot.SetActiveManipulator('gripper')
        self.robot.SetActiveDOFs(self.manipulator.GetArmIndices())
        self.links = self.robot.GetLinks()

    '''
    this function returns the jacobian matrix of the robot
    '''
    def calculate_jacobian_translation(self, link_name):
        jacobian = []
        link_idx = [l.GetName() for l in self.robot.GetLinks()].index(link_name)
        link_origin = self.robot.GetLink(link_name).GetTransform()[:3, 3]
        np.set_printoptions(precision=6, suppress=True)
        jacobian_translation = self.robot.ComputeJacobianTranslation(link_idx, link_origin)
        jacobian_angular = self.robot.ComputeJacobianAxisAngle(link_idx)
        jacobian.append(jacobian_translation)
        jacobian.append(jacobian_angular)
        return jacobian

    '''
    this function calculates the inverse kinematics based on a cartesian pose
    @param cartesian_pose: pose in format of xyzwxyz
    '''
    def ik_calculation(self, cartesian_pose):
        ikmodel = orpy.databases.inversekinematics.InverseKinematicsModel(self.robot,
                                                                          iktype=orpy.IkParameterization.Type.Transform6D)
        if not ikmodel.load():
            print "ik model loading failed"
            ikmodel.autogenerate()
            print "auto generating ik model"
        cartesian_pose_quaternion = cartesian_pose[3:]
        cartesian_pose_translation = cartesian_pose[:3]
        pose3D = tr.quaternion_matrix(cartesian_pose_quaternion)
        pose3D[:3, 3] = np.array(cartesian_pose_translation)
        ik_config_list = self.manipulator.FindIKSolutions(pose3D, orpy.IkFilterOptions.CheckEnvCollisions)

        return ik_config_list

    '''
    this function updates the self.curstate if detected state changes
    '''
    def callback(self, data):
        if len(self.curstate) == 0 or (data.position != self.curstate):
            self.curstate = []
            for item in data.position:
                self.curstate.append(item)
    '''
    this function gets the robot's current configuration by subscribing to the topic /joint_states
    '''
    def get_current_joint_states(self):
        r = rospy.Rate(10)
        count = 0
        while count < 10:
            sub_once = rospy.Subscriber("/joint_states", JointState, self.callback)
            count = count + 1
            r.sleep()
        sub_once.unregister()

    '''
    this function initialize the start and goal configurations
    @param init_config: list of initial joints angles in radians
           goal_config: list of goal joints angles in radians
    '''
    def init_poses(self, init_config, goal_config):
        self.init_config = init_config
        self.goal_config = goal_config

    '''
    this function calculate all the paths corresponding to all the goal configurations
    @param goal_config_list: list of goal configurations
    returns both the openrave path and the ros trjectory
    '''
    def calculate_all_paths(self, goal_config_list):
        result = []
        path_list = []
        ros_traj_list = []
        sum_plan_time = 0

        for goal_config in goal_config_list:
            planner = orpy.RaveCreatePlanner(self.env, 'birrt')
            params = orpy.Planner.PlannerParameters()
            params.SetRobotActiveJoints(self.robot)
            params.SetInitialConfig(self.init_config)
            params.SetGoalConfig(goal_config)
            params.SetPostProcessing('ParabolicSmoother', '<_nmaxiterations>40</_nmaxiterations>')
            #params.SetPostProcessing('shortcut_linear', '<_nmaxiterations>40</_nmaxiterations>')
            planner.InitPlan(self.robot, params)
            traj = orpy.RaveCreateTrajectory(self.env, '')
            curr_start_time = time.time()
            planner.PlanPath(traj)
            curr_end_time = time.time()
            sum_plan_time = sum_plan_time + curr_end_time - curr_start_time
            ros_traj = test_irb1200_model.ros_trajectory_from_openrave(self.robot.GetName(), traj)
            print "duration of the path is: ", ros_traj.points[-1].time_from_start
            path_list.append(traj)
            ros_traj_list.append(ros_traj)
        average_plan_time = sum_plan_time/len(goal_config_list)
        print "average motion planning time is: ", average_plan_time
        result.append(path_list)
        result.append(ros_traj_list)
        return result

    '''
    this function chooses the best trajectory based on shortest time
    @param path_list: list of candidating ros trajectory
    '''
    def calculate_best_path(self, path_list):
        # sort all the feasible paths according to their durations
        ros_traj_list = path_list[1]
        ros_traj_list.sort(key=test_irb1200_model.sort_time)
        self.optimal_path = ros_traj_list[0]
    '''
    this function visualizes all paths in the openrave visualizer
    @param path_list: list of paths to visualize
    '''
    def visualize_all_paths(self, path_list):
        # iterate through the openrave path list
        for path in path_list[0]:
            print "visualizing a feasible path!"
            self.robot.SetActiveDOFValues(self.init_config)
            controller = self.robot.GetController()
            controller.SetPath(path)
            time.sleep(2)


    '''
    this function sends the ros_traj to joint_trajectory_action server
    @param path: the ros trajectory to be executed
    '''
    def execute_path(self, path):
        # use the ros trajectory
        raw_input("Press enter to move the robot! Be careful!")
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = path
        goal.trajectory.header.stamp = rospy.Time.now()
        client.send_goal(goal)
        client.wait_for_result()

if __name__ == "__main__":
    print "Running the motion planning pipeline"
    rospy.init_node("openrave_planning_client")
    client = actionlib.SimpleActionClient('/joint_trajectory_action', FollowJointTrajectoryAction)
    client.wait_for_server()
    print "the jointTrajectoryAction server is connected"

    # initialize a MotionPlan object
    motion_planner = MotionPlan()
    # setup the environment
    motion_planner.set_env()
    # calculate the goal ik
    ik_start_time = time.time()
    goal_config_list = motion_planner.ik_calculation([0.6, 0, 0.9, 0.0, 1.0, 0.0, 0.0])
    ik_end_time = time.time()
    print "ik took ", ik_end_time-ik_start_time, " to caculate!"
    # define initial pose and goal pose
    motion_planner.get_current_joint_states()
    motion_planner.init_poses(motion_planner.curstate, [0.6, 0, 0.9, 0.0, 1.0, 0.0, 0.0])
    # calculate all paths from start pose to target pose
    path_list = motion_planner.calculate_all_paths(goal_config_list)
    # calculate the shortest path
    motion_planner.calculate_best_path(path_list)
    # visualize the best path
    motion_planner.visualize_all_paths(path_list)
    # execute the best path
    motion_planner.execute_path(motion_planner.optimal_path)

