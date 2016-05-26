#!/usr/bin/env python
"""
Coverage router:
this node translates the 2d-velocity commands that come from coverage planner
into twist stamped commands for a controller
and translates the pose stamped measurements that come from a quad or simulator
into 2d-pose information for the coverage planner.
"""



import utilities.coverage_utilities as cov
import geometry_msgs.msg as gms
import std_msgs.msg as sms
import quad_control.msg as qms
import quad_control.srv as qsv
import threading as thd
import numpy as np
import rospy as rp
import transforms3d.euler as t3de
import trajectory_msgs.msg as tms



rp.init_node('coverage_router')
__NAME = rp.get_param('name')


__HEIGHT = 1.3


__lock = thd.Lock()
__coverage_cmd_vel = gms.Pose2D()

ip = cov.INITIAL_POSES[__NAME]
__pose_2d = gms.Pose2D(x=ip[0], y=ip[1], theta=ip[2])


def __twist_from_vel_2d(vel):
    msg = gms.Twist()
    msg.linear = gms.Vector3(x=vel.x, y=vel.y, z=0.0)
    msg.angular = gms.Vector3(x=0.0, y=0.0, z=vel.theta)
    return msg
    
    
def __multi_dof_joint_trajectory_from_vel_2d_pose_2d(vel_2d, pose_2d):
    msg = tms.MultiDOFJointTrajectory()
    pose = __transform_from_pose_2d(pose_2d)
    twist = __twist_from_vel_2d(vel_2d)
    point = tms.MultiDOFJointTrajectoryPoint()
    point.transforms.append(pose)
    point.velocities.append(twist)
    msg.points.append(point)
    return msg
    
    
def __pose_2d_from_pose_stamped(ps):
    quat = ps.pose.orientation
    quat_array = [quat.w, quat.x, quat.y, quat.z]
    euler = t3de.quat2euler(quat_array)
    pose_2d = gms.Pose2D(x=ps.pose.position.x, y=ps.pose.position.y, theta=euler[2])
    return pose_2d


def __transform_from_pose_2d(ps):
    msg = gms.Transform()
    msg.translation = gms.Vector3(x=ps.x, y=ps.y, z=__HEIGHT)
    euler = (0.0, 0.0, ps.theta)
    qa = t3de.euler2quat(*euler)
    quat = gms.Quaternion(w=qa[0], x=qa[1], y=qa[2], z=qa[3])
    msg.rotation = quat
    return msg


def __coverage_cmd_vel_callback(msg):
    global __lock, __coverage_cmd_vel
    __lock.acquire()
    __coverage_cmd_vel = msg
    __lock.release()


def __pose_stamped_callback(msg):
    global __lock
    global __pose_2d
    __lock.acquire()
    __pose_2d = __pose_2d_from_pose_stamped(msg)
    __lock.release()


rp.Subscriber('coverage_cmd_vel', gms.Pose2D, __coverage_cmd_vel_callback)
rp.Subscriber('ground_truth/pose', gms.PoseStamped, __pose_stamped_callback)
__pose_2d_pub = rp.Publisher('coverage_pose_2d', gms.Pose2D, queue_size=10)
#__cmd_pose_stamped_pub = rp.Publisher('command/pose', gms.PoseStamped, queue_size=10)
#__cmd_twist_stamped_pub = rp.Publisher('cmd_twist_stamped', gms.TwistStamped, queue_size=10)
__cmd_trajectory_pub = rp.Publisher('command/trajectory', tms.MultiDOFJointTrajectory, queue_size=10)



def __work():
    global __lock
    global __coverage_cmd_vel
    global __pose_2d
    global __pose_2d_pub
    global __cmd_trajectory_pub
    __lock.acquire()
    __pose_2d_pub.publish(__pose_2d)
    #twist_stamped = __twist_stamped_from_vel_2d(__coverage_cmd_vel)
    #__cmd_twist_stamped_pub.publish(twist_stamped)
    mdofjt = __multi_dof_joint_trajectory_from_vel_2d_pose_2d(__coverage_cmd_vel, __pose_2d)
    __cmd_trajectory_pub.publish(mdofjt)
    #old_pos = np.array([__pose_2d.x, __pose_2d.y])
    #old_ang = __pose_2d.theta
    #lin_vel = np.array([__coverage_cmd_vel.x, __coverage_cmd_vel.y])
    #ang_vel = __coverage_cmd_vel.theta
    #now = rp.get_rostime()
    #time = now.secs + now.nsecs*1e-9
    #dt = 0.01
    #new_pos = old_pos + lin_vel*dt
    #new_ang = old_ang + ang_vel*dt
    #rp.logwarn(dt)
    #new_ang = np.arctan2(np.sin(new_ang), np.cos(new_ang))
    #new_pose_2d = gms.Pose2D(x=new_pos[0], y=new_pos[1], theta=new_ang)
    #new_pose_stamped = __pose_stamped_from_pose_2d(new_pose_2d)
    #__cmd_pose_stamped_pub.publish(new_pose_stamped)
    __lock.release()
    
    
__RATE = rp.Rate(1e2)
    
    
__lock.acquire()
while not rp.is_shutdown():
    __lock.release()
    __work()
    __RATE.sleep()
    __lock.acquire()
    
__lock.release()
