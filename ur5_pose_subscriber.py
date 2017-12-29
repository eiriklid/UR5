def ur5_pose_subscriber():
  print "============ Starting ur5_pose_subscriber setup"
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('eiriklid_ur5_pose_subscriber',
                  anonymous=True)

  group = moveit_commander.MoveGroupCommander("manipulator")
  print "============ Show pose"
  print group.get_current_pose().pose

  ## When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()

