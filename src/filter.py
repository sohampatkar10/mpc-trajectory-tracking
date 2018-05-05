#!/usr/bin/env python

import rospy
from quad_arm_trajectory_tracking.msg import FlatState
from gazebo_aerial_manipulation_plugin.msg import RPYPose
import numpy as np

class Filter():
  def __init__(self):
    self.pose_sub = rospy.Subscriber('/base_pose', RPYPose, self.callback, queue_size=1)
    self.state_pub = rospy.Publisher('/flat_state', FlatState, queue_size=1)
    self.measurements = list()

  def callback(self, msg):
    self.measurements.append(msg)
    if len(self.measurements) < 50:
      return

    A = np.zeros((50,4))
    bx = np.zeros(50)
    by = np.zeros(50)
    bz = np.zeros(50)

    t0 = self.measurements[0].header.stamp
    for t in range(50):
      dt = (self.measurements[t].header.stamp-t0).to_sec()
      A[t][0] = 1.0
      A[t][1] = dt
      A[t][2] = dt*dt
      A[t][3] = dt*dt*dt
      bx[t] = self.measurements[t].position.x
      by[t] = self.measurements[t].position.y
      bz[t] = self.measurements[t].position.z

    ax = np.dot(np.linalg.pinv(A), bx)
    ay = np.dot(np.linalg.pinv(A), by)
    az = np.dot(np.linalg.pinv(A), bz)

    tc = (msg.header.stamp - self.measurements[0].header.stamp).to_sec()

    flat_state = FlatState()
    flat_state.header.stamp = msg.header.stamp
    flat_state.pose.position.x = msg.position.x
    flat_state.pose.position.y = msg.position.y
    flat_state.pose.position.z = msg.position.z

    flat_state.velocity.linear.x = ax[1] + 2.0*ax[2]*tc + 3.0*ax[3]*tc*tc
    flat_state.velocity.linear.y = ay[1] + 2.0*ay[2]*tc + 3.0*ay[3]*tc*tc
    flat_state.velocity.linear.z = az[1] + 2.0*az[2]*tc + 3.0*az[3]*tc*tc

    flat_state.acceleration.x = 2.0*ax[2] + 6.0*ax[3]*tc
    flat_state.acceleration.y = 2.0*ay[2] + 6.0*ay[3]*tc
    flat_state.acceleration.z = 2.0*az[2] + 6.0*az[3]*tc

    flat_state.jerk.x = 6.0*ax[3]
    flat_state.jerk.y = 6.0*ay[3]
    flat_state.jerk.z = 6.0*az[3]

    self.state_pub.publish(flat_state)

    self.measurements.pop(0)

if __name__ == '__main__':
  rospy.init_node('filter')
  joy_transport = Filter()
  try:
    rospy.spin()
  except rospy.ROSInterruptException:
    print "Received Interrupt"
    pass
