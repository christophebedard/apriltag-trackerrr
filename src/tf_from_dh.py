#!/usr/bin/env python
## \file tf_from_dh.py
# \brief Helper node that subscribes to a JointState topic and publishes
# the corresponding transforms based on DH parameters.
# \author christophebedard
import math
import rospy
import tf
import numpy
from sensor_msgs.msg import JointState

NODE_NAME = 'tf_from_dh'
END_EFFECTOR_TF_NAME = '/camera'
FRAME_TF_NAME_PREFIX = '/joint'
COL_OFFSET = 0
COL_D = 1
COL_A = 2
COL_ALPHA = 3
COL_TOTAL = 4

child_frames = []
frames = []
offset = []
d = []
a = []
alpha = []
end_effector_rpy = {}
end_effector_xyz = {}

br = tf.TransformBroadcaster()

def jointstate_callback(data):
    """
    callback function for current jointstate message
    """
    # publish tf for each dof
    for i in range(1, dof+1):
        idx = i - 1
        theta = offset[idx] + data.position[idx]
        rot_matrix = numpy.array([[math.cos(theta),
                                   -math.sin(theta)*math.cos(alpha[idx]),
                                   math.sin(theta)*math.sin(alpha[idx]),
                                   0.],
                                  [math.sin(theta),
                                   math.cos(theta)*math.cos(alpha[idx]),
                                   -math.cos(theta)*math.sin(alpha[idx]),
                                   0.],
                                  [0.,
                                   math.sin(alpha[idx]),
                                   math.cos(alpha[idx]),
                                   0.],
                                  [0., 0., 0., 1.]])
        rot_quaternion = tf.transformations.quaternion_from_matrix(rot_matrix)
        trans_matrix = numpy.array([a[idx]*math.cos(theta), a[idx]*math.sin(theta), d[idx]])
        br.sendTransform(trans_matrix,
                         rot_quaternion,
                         rospy.Time().now(),
                         frames[idx],
                         child_frames[idx])

    # also publish end effector tf
    end_effector_trans_matrix = numpy.array([end_effector_xyz['x'],
                                             end_effector_xyz['y'],
                                             end_effector_xyz['z']])
    end_effector_rot_quaternion = tf.transformations.quaternion_from_euler(end_effector_rpy['r'],
                                                                           end_effector_rpy['p'],
                                                                           end_effector_rpy['y'])
    br.sendTransform(end_effector_trans_matrix,
                     end_effector_rot_quaternion,
                     rospy.Time().now(),
                     END_EFFECTOR_TF_NAME,
                     frames[dof-1])

def main():
    """
    tf_from_dh main
    """
    global child_frames, frames, offset, d, a, alpha
    global dof, dh_matrix, end_effector_xyz, end_effector_rpy

    rospy.init_node(NODE_NAME, anonymous=True)

    # get params
    dof = rospy.get_param('~dof')
    present_jointstate_topic = rospy.get_param('~present_jointstate_topic')
    dh_matrix = rospy.get_param('~/dh_matrix')
    end_effector_xyz['x'] = rospy.get_param('~/end_effector/x')
    end_effector_xyz['y'] = rospy.get_param('~/end_effector/y')
    end_effector_xyz['z'] = rospy.get_param('~/end_effector/z')
    end_effector_rpy['r'] = rospy.get_param('~/end_effector/roll')
    end_effector_rpy['p'] = rospy.get_param('~/end_effector/pitch')
    end_effector_rpy['y'] = rospy.get_param('~/end_effector/yaw')

    # set up dof-dependent info
    for i in range(1, dof+1):
        child_frames.append(FRAME_TF_NAME_PREFIX + str(i-1))
        frames.append(FRAME_TF_NAME_PREFIX + str(i))
        offset.append(dh_matrix[COL_TOTAL * (i - 1) + COL_OFFSET])
        d.append(dh_matrix[COL_TOTAL * (i - 1) + COL_D])
        a.append(dh_matrix[COL_TOTAL * (i - 1) + COL_A])
        alpha.append(dh_matrix[COL_TOTAL * (i - 1) + COL_ALPHA])

    rospy.Subscriber(present_jointstate_topic, JointState, jointstate_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
