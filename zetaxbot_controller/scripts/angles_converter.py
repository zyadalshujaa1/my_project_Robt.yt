#!/usr/bin/env python3
import rospy
import math
from zetaxbot_controller.srv import AnglesConverter, AnglesConverterResponse

"""
  zetaxbot - angles_converter
  This script implements two services on the topics
    - radians_to_degrees
    - degrees_to_radians

  Both of them receives a request with the format:
    float64 headPitch
    float64 rshoulder
    float64 rshoulderRot
    float64 rbicep
    float64 relbow
    float64 headYaw
    float64 rwrist 
    float64 rthumb
    float64 rring
    float64 rarc
    float64 rpind 
    float64 rmaj
    float64 lshoulder
    float64 lshoulderRot
    float64 lbicep
    float64 lelbow
    float64 lwrist 
    float64 lthumb
    float64 lring
    float64 larc
    float64 lpind 
    float64 lmaj

    rshoulder
    rshoulderRot
    rbicep
    relbow
    rwrist 
    rthumb
    rpind 
    rmaj
    rarc
    rring
    lshoulder
    lshoulderRot
    lbicep
    lelbow
    lwrist 
    lthumb
    lring
    larc
    lpind 
    lmaj
    

    rshoulder
    rshoulderRot
    rbicep
    relbow
    rwrist 
    rthumb
    rpind 
    rmaj
    rarc
    rring
    lshoulder
    lshoulderRot
    lbicep
    lelbow
    lwrist 
    lthumb
    lpind 
    lmaj
    lring
    larc
    headPitch
    headYaw

  and sends a response in the same format to the client

  The first service (radians_to_degrees) receives the angles in radians and convert
  those in degrees according to the boundaries defined inthe URDF file

  The second service (degrees_to_radians) receives the angles in degrees and convert
  those in radians according to the boundaries defined inthe URDF file

  This conversion is needed for the control of the real robot in order to convert the radians angle of each joint
  as used in ROS in degrees angles as used in Arduino for the actuation of the Servo motors
  

  Copyright (c) 2021 Antonio Brandi.  All right reserved.
"""

def convert_radians_to_degrees(req):
    # Function that is called every time the service radians_to_degrees is called
    # It receives the Request message as input with the angles in radians
    # and returns the Result message as output with the angles in degrees
    res = AnglesConverterResponse()
    # res.base = int(((req.base+(math.pi/2))*180)/math.pi)
    # res.shoulder = 180-int(((req.shoulder+(math.pi/2))*180)/math.pi)
    # res.elbow = int(((req.elbow+(math.pi/2))*180)/math.pi)
    # res.gripper = int(((-req.gripper)*180)/(math.pi/2))
    res.headYaw  =90+float(req.headYaw *180/(math.pi))
    res.headPitch  =90+float(req.headPitch *180/(math.pi))
    res.rshoulder =90+float(req.rshoulder *180/(math.pi))
    res.rshoulderRot =90+float(req.rshoulderRot *180/(math.pi))
    res.rbicep =90+float(req.rbicep *180/(math.pi))
    res.relbow =90+float(req.relbow *180/(math.pi))
    res.rwrist  =90+float(req.rwrist *180/(math.pi))
    res.rthumb =90+float(req.rthumb *180/(math.pi))
    res.rring =90+float(req.rring *180/(math.pi))
    res.rarc =90+float(req.rarc *180/(math.pi))
    res.rpind  =90+float(req.rpind *180/(math.pi))
    res.rmaj =90+float(req.rmaj *180/(math.pi))
    res.lshoulder =90+float(req.lshoulder *180/(math.pi))
    res.lshoulderRot =90+float(req.lshoulderRot *180/(math.pi))
    res.lbicep =90+float(req.lbicep *180/(math.pi))
    res.lelbow =90+float(req.lelbow *180/(math.pi))
    res.lwrist  =90+float(req.lwrist *180/(math.pi))
    res.lthumb =90+float(req.lthumb *180/(math.pi))
    res.lring =90+float(req.lring *180/(math.pi))
    res.larc =90+float(req.larc *180/(math.pi))
    res.lpind  =90+float(req.lpind *180/(math.pi))
    res.lmaj =90+float(req.lmaj *180/(math.pi))
    return res

def convert_degrees_to_radians(req):
    # Function that is called every time the service radians_to_degrees is called
    # It receives the Request message as input with the angles in degrees
    # and returns the Result message as output with the angles in radians
    res = AnglesConverterResponse()
    # res.base = ((math.pi*req.base) - ((math.pi/2)*180))/180
    # res.shoulder = (((180-req.shoulder)*math.pi)-((math.pi/2)*180))/180
    # res.elbow = ((math.pi*req.elbow) - ((math.pi/2)*180))/180
    # res.gripper = -((math.pi/2)*req.gripper)/180

    res.headYaw  = float(req.headYaw *(math.pi)/180)
    res.headPitch  = float(req.headPitch *(math.pi)/180)
    res.rshoulder = float(req.rshoulder *(math.pi)/180)
    res.rshoulderRot = float(req.rshoulderRot *(math.pi)/180)
    res.rbicep = float(req.rbicep *(math.pi)/180)
    res.relbow = float(req.relbow *(math.pi)/180)
    res.rwrist  = float(req.rwrist *(math.pi)/180)
    res.rthumb = float(req.rthumb *(math.pi)/180)
    res.rring = float(req.rring *(math.pi)/180)
    res.rarc = float(req.rarc *(math.pi)/180)
    res.rpind  = float(req.rpind *(math.pi)/180)
    res.rmaj = float(req.rmaj *(math.pi)/180)
    res.lshoulder = float(req.lshoulder *(math.pi)/180)
    res.lshoulderRot = float(req.lshoulderRot *(math.pi)/180)
    res.lbicep = float(req.lbicep *(math.pi)/180)
    res.lelbow = float(req.lelbow *(math.pi)/180)
    res.lwrist  = float(req.lwrist *(math.pi)/180)
    res.lthumb = float(req.lthumb *(math.pi)/180)
    res.lring = float(req.lring *(math.pi)/180)
    res.larc = float(req.larc *(math.pi)/180)
    res.lpind  = float(req.lpind *(math.pi)/180)
    res.lmaj = float(req.lmaj *(math.pi)/180)
    return res


if __name__ == "__main__":
    # Inizialize a ROS node called angles_converter
    rospy.init_node('angles_converter')

    # Inizialize two services for the angle conversions 
    radians_to_degrees = rospy.Service('radians_to_degrees', AnglesConverter, convert_radians_to_degrees)
    degrees_to_radians = rospy.Service('degrees_to_radians', AnglesConverter, convert_degrees_to_radians)

    # keeps the node up and running
    rospy.spin()