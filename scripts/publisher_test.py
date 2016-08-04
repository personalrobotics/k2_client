#!/usr/bin/env python
import rospy
from Person import Person
from seat import Seat
from k2_client.msg import PersonMessage
from k2_client.msg import PersonArray
from building_trust_nao_controller.msg import GameEvent
from building_trust_nao_controller.msg import RobotBehavior
import csv
import time
from time import gmtime, strftime
import json
import os

type_info = 0

def talker():
	pub = rospy.Publisher("Game", GameEvent, queue_size=10)
	rospy.init_node('publisher', anonymous=True)
	game_event = GameEvent()
	if type_info == 1:
		game_event.gameEvent = 0
	elif type_info == 2:
		game_event.gameEvent = 1
		game_event.properties = '{"Outcome":1}'
	elif type_info == 3:
		game_event.gameEvent = 1
		game_event.properties = '{"Outcome":0,"Mistakers":["Person1","Person2"]}'
	elif type_info == 4:
		game_event.gameEvent = 2
		game_event.properties = '{"Admitter":["People"]}'
	elif type_info == 5:
		game_event.gameEvent = 3
		game_event.properties = '{"People":["Person1","Person2"]}'
	elif type_info == 6:
		game_event.gameEvent = 4
	pub.publish(game_event)

if __name__ == '__main__':
	while True:
		type_input = raw_input("Please input type: ")
		if type_input == 'x':
			os._exit(0)
		else:
			type_info = (int)(type_input)
			talker()