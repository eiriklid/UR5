#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import geometry_msgs.msg

import csv

if __name__ == '__main__':
    rospy.init_node('eiriklid_ur5_tf_listener')

    listener = tf.TransformListener()


    with open('testrun.csv','w') as logFile:
    	logFileWriter = csv.writer(logFile)
    	logFileWriter.writerow(['time_ns','x','y','z','q_x','q_y','q_z','q_w'])
    	rate = rospy.Rate(125.0) 
    	#rate = rospy.Rate(10.0)
    	print "====== TIME ====="
    	print rospy.Time.now()

    	listener.waitForTransform("/world", "/ee_link", rospy.Time(), rospy.Duration(5.0))
    	while not rospy.is_shutdown():
    		try:
    			time = listener.getLatestCommonTime("/world", "/ee_link")
    			(trans,rot) = listener.lookupTransform("/world", "/ee_link", time)
    		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    			continue


    		logFileWriter.writerow([time, trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3]])
    		rate.sleep()

