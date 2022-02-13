import socket

class Communicator():
	def __init__(self, host, port):
		self.sck = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.sck.connect((host, port))
	def send_text(self, text):
		print(text)
		self.sck.sendall(bytes(text, 'ascii'))
	def receive_text(self):
		return self.sck.recv(1024)
