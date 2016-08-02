class Seat:
	def __init__(self, x=0.0, y=0.0, z=0.0, label='A'):
		self.x = x
		self.y = y
		self.z = z
		self.label = label

	def __str__(self):
		return self.label
