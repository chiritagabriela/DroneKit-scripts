import cv2
import cv2.aruco as aruco
import numpy as np
import time
import os
import platform
import sys
import socket
import struct
import math

vcap = cv2.VideoCapture('http://192.168.1.5:8080/?action=stream')
viewVideo=True

if len(sys.argv)>1:
	viewVideo=sys.argv[1]
	if viewVideo=='0' or viewVideo=='False' or viewVideo=='false':
		viewVideo=False

id_to_find=72
marker_size=18

horizontal_res = 640
vertical_res = 480

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()

cameraMatrix = np.loadtxt('cameraMatrix.txt', delimiter=',')
cameraDistortion = np.loadtxt('cameraDistortion.txt', delimiter=',')

horizontal_fov = 62.2 * (math.pi / 180 )
vertical_fov = 48.8 * (math.pi / 180)


def create_socket():
	try:
		global host
		global port
		global sock
		host = ""
		port = 9999
		sock = socket.socket()

	except socket.error as msg:
		print("Socket creation error: " + str(msg))
	print("socket created")
def bind_socket():
	try:
		global host
		global port
		global sock
		print("Binding the Port: " + str(port))

		sock.bind((host, port))
		sock.listen(5)

	except socket.error as msg:
		print("Socket Binding error" + str(msg) + "\n" + "Retrying...")
	print("socket binded")

def socket_accept():
	global client_adress
	conn, address = sock.accept()
	print("Connection has been established! |" + " IP " + address[0] + " | Port" + str(address[1]))
	conn.close()

def send_commands(conn):
	global client_adress
	while True:
		cap = cv2.VideoCapture(video_file)
		while True:
			flag, frame = cap.read()
			if flag:
				frame = cPickle.dumps(frame)
				size = len(frame)
				payload = struct.pack('L', size)
				frame = payload + frame
				conn.sendall(frame)

def lander():
	frame = vcap.read()
	frame = cv2.resize(frame,(horizontal_res,vertical_res))
	frame_np = np.array(frame[1])
	gray_img = cv2.cvtColor(frame_np, cv2.COLOR_BGR2GRAY)
	ids = ''
	corners, ids, rejected = aruco.detectMarkers(image=gray_img, dictionary=aruco_dict, parameters=parameters)
	try:
		if ids is not None and ids[0] == id_to_find:
			ret = aruco.estimatePoseSingleMarkers(corners,marker_size,cameraMatrix=cameraMatrix,distCoeffs=cameraDistortion)
			(rvec, tvec) = (ret[0][0, 0, :], ret[1][0, 0, :])
			x = '{:.2f}'.format(tvec[0])
			y = '{:.2f}'.format(tvec[1])
			z = '{:.2f}'.format(tvec[2])

			y_sum = 0
			x_sum = 0

			x_sum = corners[0][0][0][0]+ corners[0][0][1][0]+ corners[0][0][2][0]+ corners[0][0][3][0]
			y_sum = corners[0][0][0][1]+ corners[0][0][1][1]+ corners[0][0][2][1]+ corners[0][0][3][1]

			x_avg = x_sum*.25
			y_avg = y_sum*.25

			x_ang = (x_avg - horizontal_res*.5)*(horizontal_fov/horizontal_res)
			y_ang = (y_avg - vertical_res*.5)*(vertical_fov/vertical_res)
	except Exception as e:
		print('Target likely not found. Error: ' + str(e))

def main():
	create_socket()
	bind_socket()
	socket_accept()
	#conn.sendall("gabi")
main()
