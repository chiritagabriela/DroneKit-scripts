#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2
import cv2.aruco as aruco
import sys
import time
import math
import numpy as np
import ros_numpy as rnp
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil
from array import array

#VARIABLES
velocity=-.5
takeoffHeight=4

vehicle = connect('tcp:127.0.0.1:5763',wait_ready=True)
vehicle.parameters['PLND_ENABLED']=1
vehicle.parameters['PLND_TYPE']=1
vehicle.parameters['PLND_EST_TYPE']=0
vehicle.parameters['LAND_SPEED']=30 #cm/s

newImgPublisher = rospy.Publisher('/camera/color/newImage', Image, queue_size=10)

arucoID = 72
arucoSize = 20

arucosDictionary = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()

horizontalRes = 640
verticalRes = 480

horizontalFov = 62.2 * (math.pi / 180)
verticalFov = 48.8 * (math.pi / 180)

found=0
notFound=0

distCoeff = [0.0, 0.0, 0.0, 0.0, 0.0]
cameraMatrix = [[530.8269276712998, 0.0, 320.5],[0.0, 530.8269276712998, 240.5],[0.0, 0.0, 1.0]]
npCameraMatrix = np.array(cameraMatrix)
npDistCoeff = np.array(distCoeff)


timeLast=0
timeWaiting = .1

#FUNCTIONS

def armAndTakeoff(targetHeight):
    while vehicle.is_armable !=True:
        print('[?]Waiting for vehicle to become armable.')
        time.sleep(1)
    print('[+]Vehicle armable.')

    vehicle.mode = VehicleMode('GUIDED')

    while vehicle.mode !='GUIDED':
        print('[?]Waiting for drone to enter GUIDED flight mode.')
        time.sleep(1)
    print('[+]Vehicle now in GUIDED mode.')

    vehicle.armed = True
    while vehicle.armed ==False:
        print('[+]Waiting for vehicle to become armed.')
        time.sleep(1)
    print('[+]Props are spinning!')

    vehicle.simple_takeoff(targetHeight)

    while True:
        print('[+]Current altitude: %d'%vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >=.95*targetHeight:
            break
        time.sleep(1)
    print('[+]Target altitude reached!')

    return None

def sendVelocity(x,y,z):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0,
        0,
        0,
        x,
        y,
        z,
        0,0,0,0,0)
    vehicle.send_mavlink(msg)
    vehicle.flush()


def sendLandingOption(x,y):
    msg = vehicle.message_factory.landing_target_encode(
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        x,
        y,
        0,0,0
        )
    vehicle.send_mavlink(msg)
    vehicle.flush()


def msgReceiver(message):
    global notFound, found, timeLast, timeWaiting, arucoID

    height=message.height
    width=message.width


    if time.time() - timeLast > timeWaiting:
        data = rnp.numpify(message)
        gray = cv2.cvtColor(data, cv2.COLOR_BGR2GRAY)

        ids = ''
        (corners, ids, rejected) = aruco.detectMarkers(image=gray,dictionary=arucosDictionary,parameters=parameters)

        try:
            if ids is not None:
                if ids[0]==arucoID:
                    ret = aruco.estimatePoseSingleMarkers(corners,arucoSize,cameraMatrix=npCameraMatrix,distCoeffs=npDistCoeff)

                    (rvec, tvec) = (ret[0][0,0,:],ret[1][0,0,:])
                    x = '{:.2f}'.format(tvec[0])
                    y = '{:.2f}'.format(tvec[1])
                    z = '{:.2f}'.format(tvec[2])

                    sumForX = corners[0][0][0][0] + corners[0][0][1][0] + corners[0][0][2][0] +  corners[0][0][3][0]
                    sumForY = corners[0][0][0][1] + corners[0][0][1][1] + corners[0][0][2][1] +  corners[0][0][3][1]

                    averageX = sumForX / 4
                    averageY = sumForY / 4

                    angleX = (averageX - horizontalRes*.5)*horizontalFov / horizontalRes
                    angleY = (averageY - verticalRes*.5)*verticalFov / verticalRes

                    if vehicle.mode!='LAND':
                        vehicle.mode = VehicleMode('LAND')
                        while vehicle.mode!='LAND':
                            time.sleep(1)
                        print("[+]Vehicle is LAND mode.")
                        sendLandingOption(angleX,angleY)
                    else:
                        sendLandingOption(angleX,angleY)

                    position = 'MARKER POSITION: x=' + x +' y=' + y +' z=' + z

                    aruco.drawDetectedMarkers(data,corners)
                    aruco.drawAxis(data,npCameraMatrix,npDistCoeff,rvec,tvec,10)
                    cv2.putText(data,position,(10,50),0,.7,(255,0,0),thickness=2)
                    print(position)
                    found = float(found + 1)
                else:
                    notFound= float(notFound+1)
            else:
                notFound=float(notFound+1)

        except Exception as e:
            print('Target likely not found')
            print(e)
            notFound=notFound+1
        newMsg = rnp.msgify(Image, data,encoding='rgb8')
        newImgPublisher.publish(newMsg)
        timeLast = time.time()
    else:
        return None


def subscriber():
    rospy.init_node('droneNode',anonymous=False)
    sub = rospy.Subscriber('/camera/color/image_raw', Image, msgReceiver)
    rospy.spin()


if __name__=='__main__':
    try:
        armAndTakeoff(takeoffHeight)
        time.sleep(1)
        sendVelocity(0,velocity,0)
        time.sleep(10)
        subscriber()
    except rospy.ROSInterruptException:
        pass
