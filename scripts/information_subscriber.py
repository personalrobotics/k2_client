#!/usr/bin/env python
import rospy
from Person import Person
from seat import Seat
from k2_client.msg import PersonMessage
from k2_client.msg import PersonArray
import csv
import time
from time import gmtime, strftime

seats = ['A','B','C','D','E','F']
message_type = "PersonArray"
topic = "Information"

def callback(data):
	for person in data.people:
		time_string = strftime("%a, %d %b %Y %H:%M:%S", gmtime(person.header.stamp.secs))
		seat = '0'
		l_seat = '0'
		if person.seat > -1:
			seat = seats[person.seat]
		if person.looking_at_seat > -1:
			l_seat = seats[person.looking_at_seat]
		writer.writerow([time_string, message_type, topic, (int)(person.trackingId/100000000), person.trackingId%100000000.0, seat, person.position_x, person.position_y, person.position_z, person.pitch, 
			person.yaw, person.roll, person.is_talking, person.volume, person.leaning, person.arms_crossed, person.hand_touch, person.face_touch, l_seat])

def listener():
	rospy.init_node('listener', anonymous = True)
	rospy.Subscriber("Information", PersonArray, callback)
	rospy.spin()

if __name__ == '__main__':
	filename = raw_input("Please enter a file name: ")
	path = "src/k2_client/scripts/%s.csv"%filename
	csvfile = open(path, 'w')
	writer = csv.writer(csvfile)
	row = ["Time", "Message Type", "Topic", "Tracking Id Part 1", "Tracking Id Part 2", "Seat", "X Position:", "Y Position:", "Z Position:", "Pitch", "Yaw", "Roll", "Talking", "Volume", 
	"Leaning", "Arms Crossed", "Hand Touch", "Face Touch", "Looking At"]
	writer.writerow(row)
	listener()
	csvfile.close()
