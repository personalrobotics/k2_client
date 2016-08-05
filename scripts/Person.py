import math
from seat import Seat
class Person:
	def __init__(self, x=0.0, y=0.0, z=0.0, talk=False, time=0.0, p=0.0, yaw=0.0, r=0.0, lean="None", hand_touch=False, face_touch=False, arms_crossed=False, v=0.0, seat=None, look=None):
		self.x = x
		self.y = y
		self.z = z
		self.is_talking = talk
		self.total_talking_time = time 		#total talking time in milliseconds
		self.pitch = p
		self.yaw = yaw
		self.roll = r
		self.leaning = lean
		self.hand_touch = hand_touch
		self.face_touch = face_touch
		self.arms_crossed = arms_crossed
		self.volume = v
		self.seat = None
		self.looking_at_seat = look


	def __str__(self):
		return "name"

	def update_body_data(self, x=0.0, y=0.0, z=0.0):
		self.x = x
		self.y = y
		self.z = z

	def update_face_data(self, p, y, r):
		self.pitch = p
		self.yaw = y
		self.roll = r

	def update_audio_data(self, talk, volume):
		self.is_talking = talk
		if talk:
			self.total_talking_time += 16
		self.volume = volume

	#Calculate Leaning
	def is_lean(self, spine):
		if spine < -.04:
			self.leaning = "Forward"
		elif spine >= .04:
			self.leaning = "Backward"
		else:
			self.leaning = "None"

	#Calculate arms crossed
	def is_arms_crossed(self, hand_right_x, hand_right_y, hand_right_z, hand_left_x, hand_left_y, hand_left_z, elbow_right_x, elbow_right_y, elbow_right_z, elbow_left_x, elbow_left_y, elbow_left_z):
		if math.fabs(hand_right_x - elbow_left_x) < .3 and math.fabs(hand_left_x - elbow_right_x) < .3 and math.fabs(hand_right_y - elbow_left_y) < .3 and math.fabs(hand_left_y - elbow_right_y) < .3 and math.fabs(hand_right_z - elbow_left_z) < .3 and hand_left_z - elbow_right_z < .3:
			self.arms_crossed = True
		else:
			self.arms_crossed = False

	#Calculate hand touch
	def is_hand_touch(self, hand_right_x, hand_right_y, hand_right_z, hand_left_x, hand_left_y, hand_left_z):
		if math.fabs(hand_right_x - hand_left_x) < .15 and math.fabs(hand_right_y - hand_left_y) < .15 and math.fabs(hand_right_z - hand_left_z) < .15:
			self.hand_touch = True
		else:
			self.hand_touch = False

	#Calcuate face touch
	def is_face_touch(self, face_x, face_y, face_z, hand_right_x, hand_right_y, hand_right_z, hand_left_x, hand_left_y, hand_left_z):
		if (math.fabs(face_x - hand_right_x) < .2 or math.fabs(face_x - hand_left_x) < .2) and (math.fabs(face_y - hand_right_y) < .2 or math.fabs(face_y - hand_left_y) < .2) and (math.fabs(face_z - hand_right_z) < .2 or math.fabs(face_z - hand_left_z) < .2):
			self.face_touch = True
		else:
			self.face_touch = False

	#Calculate looking at
	def looking_at(self, person):

		pitch_orientation_correct = False
		yaw_orientation_correct = False

		xz_dist = math.sqrt(math.pow((person.x - self.x), 2) + math.pow((person.z - self.z), 2))
		y_dist = person.y - self.y
		ideal_pitch = math.atan(y_dist/xz_dist)
		if (math.fabs(ideal_pitch - (self.pitch*math.pi+.3))) < 0.5:
			pitch_orientation_correct = True
		else:
			pitch_orientation_correct = False

		x_dist = -1 * (person.x - self.x)
		z_dist = person.z - self.z
		theta_1 = math.atan(z_dist/x_dist)
		theta_2 = math.atan(self.z/self.x)

		if person.x > 0 and self.x > 0 and (person.x - self.x) > 0:
			ideal_y = (theta_1 + theta_2) - math.pi
		elif person.x < 0 and self.x < 0 and (person.x - self.x) < 0:
			ideal_y = (theta_1 + theta_2) + math.pi
		else:
			ideal_y = theta_1 + theta_2

		if (math.fabs(ideal_y - (self.yaw * math.pi)))  < 0.5:
			yaw_orientation_correct = True
		else:
			yaw_orientation_correct = False

		print "%s looking at %s" % (self.seat, person.seat)
		print "Self ideal x: %s Self ideal y: %s Self actual x: %s Self actual y: %s" %(ideal_pitch, ideal_y, self.pitch*math.pi+.2, self.yaw*math.pi)
		print "person ideal x: %s person ideal y: %s person actual x: %s person actual y: %s" %(ideal_pitch, ideal_y, person.pitch*math.pi+.2, person.yaw*math.pi)

		if (pitch_orientation_correct and yaw_orientation_correct):
			self.looking_at_seat = person.seat
		else:
			self.looking_at_seat = None

	#for debugging purposes
	def print_person_info(self):
		print "\n"
		print "x: %s y: %s z: %s" % (self.x, self.y, self.z)
		print "talking: %s total time talking: %s volume: %s" % (self.is_talking, self.total_talking_time, self.volume)
		print "pitch: %s yaw: %s roll: %s" % (self.pitch, self.yaw*math.pi, self.roll*math.pi)
		print "leaning: %s arms crossed: %s hand touch: %s face touch: %s" % (self.leaning, self.arms_crossed, self.hand_touch, self.face_touch)
		print "seat %s" % (self.seat)
		print "looking at: %s" % (self.looking_at_seat)
		print "\n"


