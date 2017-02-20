#!/usr/bin/env python
import rospy
from Person import Person
from seat import Seat
from k2_client.msg import BodyArray
from k2_client.msg import Body
from k2_client.msg import Audio
from k2_client.msg import Face
from k2_client.msg import JointPositionAndState
from k2_client.msg import JointOrientationAndType
import math
import sys
import os
from k2_client.msg import PersonMessage
from k2_client.msg import PersonArray

person_dict = dict([])
seat_list = []
seat_names = ['A','B','C','D','E','F']
count_messages_to_ten = 0
distance = .3
people = 1

def callback_audio(data):
	for key in person_dict:
		if key in data.correlations:
			person_dict[key].update_audio_data(True, data.volume)
		else:
			person_dict[key].update_audio_data(False, 0.0)
	publish(data.header.stamp)

def callback_face(data):
	key = data.trackingId
	if key in person_dict:
		person_dict[key].update_face_data(data.faceOrientation.x, data.faceOrientation.y, data.faceOrientation.z)
	publish(data.header.stamp)

def callback_body(data):
	global count_messages_to_ten
	#if the three bodies have been correctly initialized
	if count_messages_to_ten > 10: 
		tracked_people = []
		untracked_bodies = []
		#go through each incoming body
		for b in data.bodies:
			if b.trackingId == 0:
				pass
			else:
				key = b.trackingId
				#if the trackingID matches with currently tracked bodies, update the data normally
				if key in person_dict:
					update_all_body_info(b, key)
					tracked_people.append(key)
				else:
					untracked_bodies.append(b)
		print person_dict.keys()

		publish(data.bodies[0].header.stamp)

		#first attempt to match body, based on last updated position of the untracked people
		for body in untracked_bodies:
			print "attempting to match by position"
			pos = body.jointPositions[3].position
			for p in person_dict.keys():
				if p not in tracked_people:
					if math.fabs(person_dict[p].x - pos.x) < distance and math.fabs(person_dict[p].y - pos.y) < distance and math.fabs(person_dict[p].z - pos.z) < distance:
						update_all_body_info(body, p)
						person_dict[body.trackingId] = person_dict.pop(p)
						untracked_bodies.remove(body)
						tracked_people.append(p)

		#second attempt to match body, based on position of seat of untracked people
		for body in untracked_bodies:
			print "attempting to match by seat"
			for key in person_dict.keys():
				if key not in tracked_people:
					pos = body.jointPositions[3].position
					temp_seat =  person_dict[key].seat
					if math.fabs(temp_seat.x - pos.x) < distance and math.fabs(temp_seat.y - pos.y) < distance and math.fabs(temp_seat.z - pos.z) < distance:
						update_all_body_info(body, key)
						person_dict[body.trackingId] = person_dict.pop(key)
						untracked_bodies.remove(body)

		if len(untracked_bodies) > 0:
			print "ERROR: Unidentified Body"

	else:
		if count_messages_to_ten == 10 and not len(person_dict) == people:
			print "ERROR: %s Bodies Not Identified" % people
			os._exit(0)
		#if all bodies have been correctly identified
		if count_messages_to_ten == 10:
			ids = []
			x_pos = []
			for p_id in person_dict:
				ids.append(p_id)
				x_pos.append(person_dict[p_id].x)

			#sort both lists based on increasing x position
			unzipped_sorted = zip(*sorted(zip(x_pos, ids), key=lambda pair: pair[0]))
			x_pos = list(unzipped_sorted[0])
			ids = list(unzipped_sorted[1])
			
			#seat with the smallest x position is seat A
			for i in ids:
				seat_list[ids.index(i)].x = person_dict[ids[ids.index(i)]].x
				seat_list[ids.index(i)].y = person_dict[ids[ids.index(i)]].y
				seat_list[ids.index(i)].z = person_dict[ids[ids.index(i)]].z
				person_dict[ids[ids.index(i)]].seat = seat_list[ids.index(i)]

			count_messages_to_ten = count_messages_to_ten + 1

		#if it hasn't received 10 messages yet, update info with average position or add a new person to the dictionary
 		else:
			for body in data.bodies:
				key = body.trackingId
				if key == 0:
					pass
				else:
					p = body.jointPositions[3].position
					if key in person_dict:
						person_dict[key].update_body_data((p.x + person_dict[key].x)/2, (p.y + person_dict[key].y)/2, (p.z + person_dict[key].z)/2)
					else:
						person_dict[key] = Person(x=p.x, y=p.y, z=p.z)
			count_messages_to_ten = count_messages_to_ten + 1 

	for key1 in person_dict:
		for key2 in person_dict:
			if not key1 == key2:
				person_dict[key1].looking_at(person_dict[key2])	

	for key in person_dict:
		person_dict[key].print_person_info()

#Calculate information about leaning, armscrossed, facetouch, and hand touch and store that information
def update_all_body_info(body, key):
	p = body.jointPositions
	person_dict[key].update_body_data(p[3].position.x, p[3].position.y, p[3].position.z)
	person_dict[key].is_lean(body.jointOrientations[1].orientation.z)
	person_dict[key].is_arms_crossed(p[23].position.x, p[23].position.y, p[23].position.z, p[21].position.x, p[21].position.y, p[21].position.z,
		p[9].position.x, p[9].position.y, p[9].position.z, p[5].position.x, p[5].position.y, p[5].position.z)
	person_dict[key].is_hand_touch(p[23].position.x, p[23].position.y, p[23].position.z, p[21].position.x, p[21].position.y, p[21].position.z)
	person_dict[key].is_face_touch(p[3].position.x, p[3].position.y, p[3].position.z, p[23].position.x, p[23].position.y, p[23].position.z, 
		p[21].position.x, p[21].position.y, p[21].position.z)

#Publish calculated information on a new topic
def publish(time):
	person_a = PersonArray()
	person_array = person_a.people
	pub = rospy.Publisher("Information", PersonArray, queue_size = 10)
	for person in person_dict:
		temp_person = PersonMessage()
		temp_person.header.stamp = time
		temp_person.trackingId = person
		temp_person.position_x = person_dict[person].x
		temp_person.position_y = person_dict[person].y
		temp_person.position_z = person_dict[person].z
		temp_person.is_talking = person_dict[person].is_talking
		temp_person.pitch = person_dict[person].pitch
		temp_person.yaw = person_dict[person].yaw
		temp_person.roll = person_dict[person].roll
		temp_person.leaning = person_dict[person].leaning
		temp_person.hand_touch = person_dict[person].hand_touch
		temp_person.face_touch = person_dict[person].face_touch
		temp_person.arms_crossed = person_dict[person].arms_crossed
		temp_person.volume = person_dict[person].volume
		if not person_dict[person].seat == None:
			temp_person.seat = seat_names.index(person_dict[person].seat.label)
		else:
			temp_person.seat = -1
		if not person_dict[person].looking_at_seat == None:
			temp_person.looking_at_seat = seat_names.index(person_dict[person].looking_at_seat.label)
		else:
			temp_person.looking_at_seat = -1
		person_array.append(temp_person)

	pub.publish(person_array)

def listener():
	rospy.init_node('listener', anonymous = True)
	rospy.Subscriber("head/kinect2/k2_faces/faces", Face, callback_face)
	rospy.Subscriber("head/kinect2/k2_bodies/bodies", BodyArray, callback_body)
	rospy.Subscriber("head/kinect2/k2_audio/audio", Audio, callback_audio)
	rospy.spin()

if __name__ == '__main__':
	people = (int)(raw_input("Number of people: "))
	for c in seat_names:
		seat_list.append(Seat(0.0,0.0,0.0,c))
	count_messages_to_ten = 0
	listener()
	#For debugging purposes 
	#print person_dict.keys()

