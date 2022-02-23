class DataSaver:
	def __init__(self, filename):
		self.fd = open(filename, "w")

	def save_data(self, a, g, m):
		self.fd.write(f"Acc:{a} Gyr:{g} Mag:{m}\n")

	def close(self):
		self.fd.close()

