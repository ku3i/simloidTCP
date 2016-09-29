#!/usr/bin/python

import sys, socket, subprocess, time, random
from os import listdir
from os.path import isfile, join

# globals
sim_path = "./bin/Release/simloid"
address  = "127.0.0.1"
port     = random.randint(7000, 9000)
bufsize  = 4096
robot_id = 31
scene_id = 3

MSG = '\033[93m' #orange terminal color

class Joint:
	def __init__(self, params):
		#traits
		self.id       =   int(params[0])
		self.type     =       params[1]
		self.sym      =       params[2]
		self.stop_lo  = float(params[3])
		self.stop_hi  = float(params[4])
		self.def_pos  = float(params[5])
		self.name     =       params[6]

		#state
		self.position = 0.0
		self.velocity = 0.0
		self.voltage  = random.uniform(-0.01, +0.01)

	def printf(self):
		print("\t{0} = {1} (def: {2}".format(self.id, self.name, self.def_pos))


class Simloid:
	def __init__(self, address, port):
		self.sock = socket.socket()
		self.sock.connect((address, port))
		msg = self.sock.recv(bufsize)
		robot_info = msg.split()
		
		self.num_bodies = int(robot_info.pop(0))
		self.num_joints = int(robot_info.pop(0))
		self.num_accels = int(robot_info.pop(0))
		self.joints = []		

		for i in range(self.num_joints):
			j = Joint(robot_info[0:7])
			self.joints.append(j)
			robot_info = robot_info[7:]

		print(MSG + "Robot has {0} joints:".format(self.num_joints))
		for j in self.joints: j.printf()

		print(MSG + "and {0} bodies and {1} acceleration sensors".format(self.num_bodies, self.num_accels))
		self.sock.send("ACK\n")
		self.recv_sensor_status()
		self.sock.send("UA 0\nGRAVITY ON\nDONE\n")
		self.recv_sensor_status()

		print(MSG + "Robot initialized.\n\n")
		

	def send_motor_controls(self):
		cmd = "UX " + " ".join(str(j.voltage) for j in self.joints) + "\nDONE\n"
		self.sock.send(cmd)
		return self.sock

	def recv_sensor_status(self):
		msg = self.sock.recv(bufsize)
		result = self.sock and (len(msg) > 0)
		if (result):
			status = msg.split()
			#print status
			time = float(status.pop(0))
			for j in self.joints: j.position = float(status.pop(0))
			for j in self.joints: j.velocity = float(status.pop(0))

		return result

	def loop(self):
		return self.send_motor_controls() and self.recv_sensor_status()

		

def control_loop(joints):
	# simple p-ctrl holding the default position
	for j in joints:
		j.voltage = 5.0 * (j.def_pos - j.position)
	


def main(argv):
	random.seed()
	subprocess.Popen([sim_path, "--port", str(port), "--robot", str(robot_id), "--scene", str(scene_id)])
	time.sleep(0.5)
	simloid = Simloid(address, port)
	result = True

	while (result):
    		try:
        		result = simloid.loop()
			control_loop(simloid.joints)

    		except KeyboardInterrupt: # press CTRL + C to exit
        		print(MSG + "Bye___")
	        	sys.exit()
		
	
	


if __name__ == "__main__": main(sys.argv)

