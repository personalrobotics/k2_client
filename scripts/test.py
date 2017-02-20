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
import time

person_dict = dict([])
seat_list = []
seat_names = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
count_messages_to_ten = 0
distance = .3
people = 1
timeStart = 0.0
file_name = None
testing = ""
correct = ""
total_correct = 0.0
total_sent = 0.0
seat_needed = ""

#All methods run for 5 seconds
def callback_audio(data):
	if time.time() - timeStart > 5:
		exit_all()
	else:
		for key in person_dict:
			if key in data.correlations:
				person_dict[key].update_audio_data(True, data.volume)
			else:
				person_dict[key].update_audio_data(False, 0.0)

def callback_face(data):
	if time.time() - timeStart > 5:
		exit_all()
	else:
		key = data.trackingId
		if key in person_dict:
			person_dict[key].update_face_data(data.faceOrientation.x, data.faceOrientation.y, data.faceOrientation.z)

#Testing body information only
def callback_body(data):
	global total_correct
	global total_sent
	if time.time() - timeStart > 5:
		exit_all()
	else:
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

			#first attempt to match body, based on last updated position of the untracked people
			for body in untracked_bodies:
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
				print "ERROR: Three Bodies Not Identified"
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

		#Run Tests
		for key in person_dict:
			if "%s"%person_dict[key].seat == seat_needed:
				if testing == "lean":
					file_name.write("%s\n"%person_dict[key].leaning)
					if "%s"%person_dict[key].leaning == correct:
						total_correct += 1.0
					total_sent += 1.0
				elif testing == "looking":
					file_name.write("%s\n"%person_dict[key].looking_at_seat)
					if "%s"%person_dict[key].looking_at_seat == correct:
						total_correct += 1.0
					total_sent += 1.0
				elif testing == "facetouch":
					file_name.write("%s\n"%person_dict[key].face_touch)
					if "%s"%person_dict[key].face_touch == correct:
						total_correct += 1.0
					total_sent += 1.0
				elif testing == "handtouch":
					file_name.write("%s\n"%person_dict[key].hand_touch)
					if "%s"%person_dict[key].hand_touch == correct:
						total_correct += 1.0
					total_sent += 1.0
				elif testing == "armscrossed":
					file_name.write("%s\n"%person_dict[key].arms_crossed)
					if "%s"%person_dict[key].arms_crossed == correct:
						total_correct += 1.0
					total_sent += 1.0
			#file_name.write(person_dict[key].print_person_info())

def update_all_body_info(body, key):
	p = body.jointPositions
	person_dict[key].update_body_data(p[3].position.x, p[3].position.y, p[3].position.z)
	person_dict[key].is_lean(body.jointOrientations[1].orientation.z)
	person_dict[key].is_arms_crossed(p[23].position.x, p[23].position.y, p[23].position.z, p[21].position.x, p[21].position.y, p[21].position.z,
		p[9].position.x, p[9].position.y, p[9].position.z, p[5].position.x, p[5].position.y, p[5].position.z)
	person_dict[key].is_hand_touch(p[23].position.x, p[23].position.y, p[23].position.z, p[21].position.x, p[21].position.y, p[21].position.z)
	person_dict[key].is_face_touch(p[3].position.x, p[3].position.y, p[3].position.z, p[23].position.x, p[23].position.y, p[23].position.z, 
		p[21].position.x, p[21].position.y, p[21].position.z)

def exit_all():
	print "total correct: %s total sent: %s"%(total_correct, total_sent)
	print "%s"%(total_correct/total_sent)
	file_name.write("%s\n"%(total_correct/total_sent))
	file_name.close()
	os._exit(0)

def listener(time):
	rospy.init_node('listener', anonymous = True)
	rospy.Subscriber("head/kinect2/k2_faces/faces", Face, callback_face)
	rospy.Subscriber("head/kinect2/k2_bodies/bodies", BodyArray, callback_body)
	rospy.Subscriber("head/kinect2/k2_audio/audio", Audio, callback_audio)
	rospy.spin()

if __name__ == '__main__':
	total_correct = 0
	total_sent = 0
	people = (int)(raw_input("How many people? "))
	for c in seat_names:
		seat_list.append(Seat(0.0,0.0,0.0,c))
	count_messages_to_ten = 0

	testing = raw_input("What are we testing? ")
	seat_needed = raw_input("Which seat is being tested? ")
	correct = raw_input("What should be received? ")
	info = raw_input("Additional file info? ")
	name = raw_input("Name(s)? ")

	path = "/root/workspace/src/k2_client/scripts/testing_data/test_%s_%s_%s.txt"%(testing, info, name)

	file_name = open(path, "w")
	file_name.write("%s\n"%testing)
	file_name.write("%s\n"%info)
	file_name.write("%s\n"%name)

	begin = raw_input("Please type 'begin': ")

	if begin == "begin":
		timeStart = time.time()
		listener(time.time() - timeStart)
		print person_dict.keys()




