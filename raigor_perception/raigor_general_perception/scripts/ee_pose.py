#!/usr/bin/env python
# this program has obivious delay, deprecated since 210813
import rospy
import tf2_ros
import geometry_msgs.msg
rospy.init_node('ee_pose_listener')
 
# create a tf2_ros type buffer
tfBuffer = tf2_ros.Buffer()
# create a TF2 transform listener object. Save data into tfBuffer
listener = tf2_ros.TransformListener(tfBuffer)
end_effector_pos = rospy.Publisher('/end_effector_pos', geometry_msgs.msg.TransformStamped, queue_size=1)
 
rate = rospy.Rate(20) # rate at 5 hz
 
while not rospy.is_shutdown():
 
    try:
        # Get last ( Time(0) )transform from the world frame to frame m1n6s200_end_effector
        frame_info = tfBuffer.lookup_transform('base_link', 'ee_link', rospy.Time(0))
        # print ("end-effector print out",frame_info)
        cmd = geometry_msgs.msg.TransformStamped()
        cmd = frame_info
        end_effector_pos.publish(cmd)
 
    except (tf2_ros.TransformException):
        rate.sleep()
        continue
 
rate.sleep()