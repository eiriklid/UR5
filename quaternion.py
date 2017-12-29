import geometry_msgs.msg

INIT = geometry_msgs.msg.Quaternion() #	x: 135	y:  0	z:180
INIT.w 	= 0
INIT.x 	= 0
INIT.y 	= -0.924
INIT.z 	= 0.383

ROLL = geometry_msgs.msg.Quaternion() #	x:  45	y:  0	z:180
ROLL.w 	= 0
ROLL.x 	= 0
ROLL.y 	= -0.383
ROLL.z 	= 0.924

PITCH = geometry_msgs.msg.Quaternion()# x:  90 	y:-45	z:90	
PITCH.w 	= 0.653
PITCH.x 	= 0.271
PITCH.y 	= -0.653
PITCH.z 	= 0.271

YAW = geometry_msgs.msg.Quaternion() # 	x:-180 	y:-45 	z:-90
YAW.w 	= 0.271
YAW.x 	= -0.653
YAW.y 	= -0.653
YAW.z 	= 0.271


ROLL_YAW = geometry_msgs.msg.Quaternion()	
ROLL_YAW.w 	= -0.653
ROLL_YAW.x 	= 0.271
ROLL_YAW.y 	= 0.271
ROLL_YAW.z 	= -0.653

ROLL_PITCH = geometry_msgs.msg.Quaternion()	
ROLL_PITCH.w 	= 0.271
ROLL_PITCH.x 	= 0.653
ROLL_PITCH.y 	= -0.271
ROLL_PITCH.z 	= 0.653



PITCH_YAW = geometry_msgs.msg.Quaternion()	
PITCH_YAW.w 	= -0.271
PITCH_YAW.x 	= -0.653
PITCH_YAW.y 	= 0.271
PITCH_YAW.z 	= -0.653