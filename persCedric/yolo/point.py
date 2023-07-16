class Point:
	def __init__(self, x, y):
		self.x = x
		self.y = y

	def get(self):
		return (self.x, self.y)

	def get_int(self):
		return (int(self.x), int(self.y))

