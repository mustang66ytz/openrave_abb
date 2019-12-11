#!/usr/bin/env python
import openravepy as orpy
import time
import numpy as np
import tf.transformations as tr
import rospy
import roslib
import actionlib

from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from geometry_msgs.msg import (Point, Quaternion, Pose, Vector3, Transform, Wrench)
from sensor_msgs.msg import CameraInfo, Image, RegionOfInterest, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def ros_trajectory_from_openrave(robot_name, traj):
  """
  Converts an OpenRAVE trajectory into a ROS JointTrajectory message.
  @type  robot_name: str
  @param robot_name: The robot name
  @type  traj: orpy.Trajectory
  @param traj: The input OpenRAVE trajectory
  @rtype: trajectory_msgs/JointTrajectory
  @return: The equivalent ROS JointTrajectory message
  """
  ros_traj = JointTrajectory()
  # Specification groups
  spec = traj.GetConfigurationSpecification()
  try:
    values_group = spec.GetGroupFromName('joint_values {0}'.format(robot_name))
  except orpy.openrave_exception:
    orpy.RaveLogError('Corrupted trajectory. Failed to find group: joint_values')
    return None
  try:
    velocities_group = spec.GetGroupFromName('joint_velocities {0}'.format(robot_name))
  except orpy.openrave_exception:
    orpy.RaveLogError('Corrupted trajectory. Failed to find group: joint_velocities')
    return None
  try:
    deltatime_group = spec.GetGroupFromName('deltatime')
  except orpy.openrave_exception:
    orpy.RaveLogError('Corrupted trajectory. Failed to find group: deltatime')
    return None
  # Copy waypoints
  time_from_start = 0
  for i in range(traj.GetNumWaypoints()):
    waypoint = traj.GetWaypoint(i).tolist()
    deltatime = waypoint[deltatime_group.offset]
    # OpenRAVE trajectory sometimes comes with repeated waypoints. DO NOT append them
    if np.isclose(deltatime, 0) and i > 0:
      continue
    # Append waypoint
    ros_point = JointTrajectoryPoint()
    ros_point.positions = waypoint[values_group.offset:values_group.offset+values_group.dof]
    ros_point.velocities = waypoint[velocities_group.offset:velocities_group.offset+velocities_group.dof]
    time_from_start += deltatime
    ros_point.time_from_start = rospy.Duration(time_from_start)
    ros_traj.points.append(ros_point)
    ros_traj.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
  return ros_traj

class PickPlace(object):

	def __init__(self):
		self.env = orpy.Environment()
		self.env.SetViewer('qtcoin')
#		self.env.Load('../robots/ur10_robotiq_85_gripper.robot.xml')
		self.env.Load('../worlds/cubes_irb1200_task.env.xml')
		self.env.SetDefaultViewer()
		self.robot = self.env.GetRobot('robot')
		self.manipulator = self.robot.SetActiveManipulator('gripper')
		self.robot.SetActiveDOFs(self.manipulator.GetArmIndices())
		self.links = self.robot.GetLinks()
		self.box = orpy.RaveCreateKinBody(self.env, '')
		self.box_centroid = []
		self.solutions = []
		self.flag = 0
		self.initConfig = []

	def get_links(self):
		print self.links
	
	def set_env(self):
		with self.env:
			self.box.SetName('box')
			self.box.InitFromBoxes(np.array([[0.5, 0.3, 1, 0.01, 0.04, 0.22]]), True)
			self.env.AddKinBody(self.box)
		self.box_centroid = self.box.ComputeAABB().pos()
		print self.box_centroid

	def calculate_jacobian_translation(self, link_name):
		jacobian = []
		link_idx = [l.GetName() for l in self.robot.GetLinks()].index(link_name) 
		link_origin = self.robot.GetLink(link_name).GetTransform()[:3,3]
		np.set_printoptions(precision=6, suppress=True)
		jacobian_translation = self.robot.ComputeJacobianTranslation(link_idx, link_origin)
		jacobian_angular = self.robot.ComputeJacobianAxisAngle(link_idx)
		jacobian.append(jacobian_translation)
		jacobian.append(jacobian_angular)
		return jacobian

	def ik_calculation(self):
		ikmodel = orpy.databases.inversekinematics.InverseKinematicsModel(self.robot, iktype=orpy.IkParameterization.Type.Transform6D)
		if not ikmodel.load():
			print "ik model loading failed"
  			ikmodel.autogenerate()
			print "auto generating ik model"
		Tgrasp = tr.quaternion_matrix([ 0.5,  0.5,  0.5, -0.5])
		Tgrasp[:3,3] = self.box_centroid
		print "finding the ik solutions"
		#self.solutions = self.manipulator.FindIKSolutions(Tgrasp, 0)
		self.solutions = self.manipulator.FindIKSolutions(Tgrasp, orpy.IkFilterOptions.CheckEnvCollisions)

		print self.solutions

	def set_pose(self):
		for config in self.solutions:
			self.robot.SetActiveDOFValues(config)
			time.sleep(1)

	def grasp(self):
		taskmanip = orpy.interfaces.TaskManipulation(self.robot)
		taskmanip.CloseFingers()
		self.robot.WaitForController(0)
		self.robot.Grab(self.box)

	def diff_ik(self, twist):
		link_idx = [l.GetName() for l in self.robot.GetLinks()].index('robotiq_85_base_link')
		link_origin = self.robot.GetLink('robotiq_85_base_link').GetTransform()[:3,3]
		J = np.zeros((6,6))
		q = self.robot.GetActiveDOFValues()
		for i in range(10):
  			J[:3,:] = self.robot.ComputeJacobianTranslation(link_idx, link_origin)[:,:6]
  			J[3:,:] = self.robot.ComputeJacobianAxisAngle(link_idx)[:,:6]
  			qdot = np.linalg.solve(J, twist)
  			q[:6] += qdot
  			self.robot.SetActiveDOFValues(q)
  			raw_input('Press Enter for next differential IK step')

	def callback(self, data):
		if len(self.initConfig) == 0:
			print "getting the current joint states"
			for item in data.position:
				self.initConfig.append(item)

	def get_current_joint_states(self):
		r = rospy.Rate(10)
		# this function subscribe to the topic /joint_states and update the self.init_config with the current value
		
		count = 0
		while count<10:
			sub_once = rospy.Subscriber("/joint_states", JointState, self.callback)
			#rospy.spin()
			count = count + 1
			r.sleep()
		sub_once.unregister()
		

	def motion_plan(self):
		planner = orpy.RaveCreatePlanner(self.env, 'birrt')
		params = orpy.Planner.PlannerParameters()
		params.SetRobotActiveJoints(self.robot)
		self.get_current_joint_states()
		params.SetInitialConfig(self.initConfig)
		params.SetGoalConfig(self.solutions[0])
		params.SetPostProcessing('ParabolicSmoother', '<_nmaxiterations>40</_nmaxiterations>')
		planner.InitPlan(self.robot, params)
		# Plan a trajectory
		traj = orpy.RaveCreateTrajectory(self.env, '')
		planner.PlanPath(traj)
		controller = self.robot.GetController()
		controller.SetPath(traj)
		# print the sequence of joint angles
		times = np.arange(0, traj.GetDuration(), 0.01)
		qvect = np.zeros((len(times), self.robot.GetActiveDOF()))
		spec = traj.GetConfigurationSpecification()
		for i in range(len(times)):
  			trajdata = traj.Sample(times[i])
			qvect[i,:] = spec.ExtractJointValues(trajdata, self.robot, self.manipulator.GetArmIndices(), 0)
		print qvect
		ros_traj = ros_trajectory_from_openrave(self.robot.GetName(), traj)
		print "trajectory in ros format is:"
		print ros_traj
		return ros_traj


if __name__ == "__main__":
	print "running openrave in ros"
	rospy.init_node("openrave_planning_client")
        client = actionlib.SimpleActionClient('/joint_trajectory_action', FollowJointTrajectoryAction)
        client.wait_for_server()
	print "the jointTrajectoryAction server is connected"
	scene = PickPlace()
	scene.set_env()
	scene.ik_calculation()
	#scene.set_pose()
	#scene.grasp()
	#twist = np.array([0, 0, 0.01, 0, 0, 0])
	#scene.diff_ik(twist)
	#jacobian = scene.calculate_jacobian_translation('robotiq_85_base_link')
	#print "translation jacobian: ", jacobian[0]
	#print "angular jacobian: ", jacobian[1]
	raw_input("Press enter to execute motion planning")
	res_traj = scene.motion_plan()
        goal = FollowJointTrajectoryGoal()
	goal.trajectory = res_traj
	
	goal.trajectory.header.stamp = rospy.Time.now()
	client.send_goal(goal)
	client.wait_for_result()


	raw_input("Press any key to exit")

