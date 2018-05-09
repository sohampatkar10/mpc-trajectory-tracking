#!/usr/bin/env python

import rospy
from quad_arm_trajectory_tracking.msg import FlatState
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
from gazebo_aerial_manipulation_plugin.msg import RPYPose
import numpy as np
from tf import transformations

class Filter():
  def __init__(self):
    self.twistSub = rospy.Subscriber('/base_twist', TwistStamped, self.callback, queue_size=1)
    self.poseSub = rospy.Subscriber('/base_pose', RPYPose, self.pose_callback, queue_size=1)
    self.accn_pub = rospy.Publisher('/base_accn', Vector3, queue_size=1)
    self.yr_pub = rospy.Publisher('/yaw_rate', Float64, queue_size=1)
    self.measurements = list()
    self.yaw_measurements = list()

  def callback(self, msg):
    self.measurements.append(msg)
    if len(self.measurements) < 101:
      return

    bx = np.zeros(100)
    by = np.zeros(100)
    bz = np.zeros(100)

    for t in range(1, 101):
      dt = (self.measurements[t].header.stamp - self.measurements[t-1].header.stamp).to_sec()
      # print "dt = ", dt
      dt = 1e-2
      bx[t-1] = (self.measurements[t].twist.linear.x - self.measurements[t-1].twist.linear.x)/dt
      by[t-1] = (self.measurements[t].twist.linear.y - self.measurements[t-1].twist.linear.y)/dt
      bz[t-1] = (self.measurements[t].twist.linear.z - self.measurements[t-1].twist.linear.z)/dt

    accn = Vector3()

    accn.x = np.average(bx);
    accn.y = np.average(by);
    accn.z = np.average(bz);

    self.accn_pub.publish(accn)
    self.measurements.pop(0)

  def pose_callback(self, msg):
    self.yaw_measurements.append(msg)
    if len(self.yaw_measurements) < 101:
      return

    y = np.zeros(100)
    for t in range(1, 101):
      dt = (self.yaw_measurements[t].header.stamp - self.yaw_measurements[t-1].header.stamp).to_sec()
      # print "dt = ", dt
      dt = 1e-2
      y[t-1] = (self.yaw_measurements[t].rpy.z - self.yaw_measurements[t-1].rpy.z)/dt

    yr = Float64()
    yr.data = np.average(y)

    self.yr_pub.publish(yr)
    self.yaw_measurements.pop(0)

if __name__ == '__main__':
  rospy.init_node('filter')
  joy_transport = Filter()
  try:
    rospy.spin()
  except rospy.ROSInterruptException:
    print "Received Interrupt"
    pass
