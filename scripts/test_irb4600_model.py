#!/usr/bin/env python
import openravepy as orpy
import time
import tf.transformations as tr
import rospy
import math
import numpy as np

if __name__ == "__main__":
    env = orpy.Environment()
    env.SetViewer('qtcoin')
    env.Load('../worlds/boxes_irb4600_task.env.xml')
    robot = env.GetRobot('robot')
    manipulator = robot.SetActiveManipulator('gripper')
    robot.SetActiveDOFs(manipulator.GetArmIndices())
    np.set_printoptions(precision=6, suppress=True)

    ikmodel = orpy.databases.inversekinematics.InverseKinematicsModel(robot,
                                                                      iktype=orpy.IkParameterization.Type.Transform6D)
    if not ikmodel.load():
        print "no ik model found"
        ikmodel.autogenerate()
        print "auto generating ik model"
    target1 = tr.quaternion_matrix([0.0, 1.0, 0.0, 0.0])
    target1[:3, 3] = np.array([0.6, 0, 0.9])
    roll1, pitch1, yaw1 = tr.euler_from_matrix(target1, 'syxz')
    print "target orientation is: ", roll1, pitch1, yaw1
    print "target pose is:", target1
    solutions = manipulator.FindIKSolutions(target1, orpy.IkFilterOptions.CheckEnvCollisions)
    print solutions

    planner = orpy.RaveCreatePlanner(env, 'birrt')  # Using bidirectional RRT
    params = orpy.Planner.PlannerParameters()
    params.SetRobotActiveJoints(robot)
    params.SetGoalConfig(solutions[0])
    params.SetPostProcessing('ParabolicSmoother', '<_nmaxiterations>10</_nmaxiterations>')
    planner.InitPlan(robot, params)
    # Plan a trajectory
    traj = orpy.RaveCreateTrajectory(env, '')
    planner.PlanPath(traj)
    # Execute the trajectory
    controller = robot.GetController()
    controller.SetPath(traj)
    # set the poses
    #robot.SetActiveDOFValues(solutions[0])

    link_idx = [l.GetName() for l in robot.GetLinks()].index('tool0')
    link_origin = robot.GetLink('tool0').GetTransform()[:3, 3]
    J = np.zeros((6, 6))
    q = robot.GetActiveDOFValues()

    #move linear
    J[:3, :] = robot.ComputeJacobianTranslation(link_idx, link_origin)[:, :6]
    J[3:, :] = robot.ComputeJacobianAxisAngle(link_idx)[:, :6]
    target2 = tr.quaternion_matrix([ 0.5,  0.5,  0.5, -0.5])
    target2[:3, 3] = np.array([0.6, 0, 1.2])
    roll2, pitch2, yaw2 = tr.euler_from_matrix(target2, 'syxz')
    print "target orientation is: ", roll2, pitch2, yaw2
    solutions2 = manipulator.FindIKSolutions(target2, orpy.IkFilterOptions.CheckEnvCollisions)
    print "setting to the second target"
    time.sleep(2)
    robot.SetActiveDOFValues(solutions2[0])
    time.sleep(2)
    robot.SetActiveDOFValues(solutions[0])
    dx = target2[0, 3]-target1[0, 3]
    dy = target2[1, 3]-target1[1, 3]
    dz = target2[2, 3]-target1[2, 3]
    droll = roll2 - roll1
    dpitch = pitch2 - pitch1
    dyaw = yaw2 - yaw1
    print "changes in orientation is: ", droll, dpitch, dyaw

    for i in range(100):
        print "moving in linear motion"
        J[:3, :] = robot.ComputeJacobianTranslation(link_idx, link_origin)[:, :6]
        J[3:, :] = robot.ComputeJacobianAxisAngle(link_idx)[:, :6]
        twist = np.array([dx/100, dy/100, dz/100, droll/100, dpitch/100, dyaw/100])
        qdot = np.linalg.solve(J, twist)
        q[:6] += qdot
        robot.SetActiveDOFValues(q)
        time.sleep(0.1)
        #twist = np.array([0, 0, 0.01, 0, 0, 0])

    raw_input('Press Enter to exit')
