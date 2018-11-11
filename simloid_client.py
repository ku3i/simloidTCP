#!/usr/bin/python

import sys, socket, subprocess, time, random
from os import listdir
from os.path import isfile, join

# globals
sim_path = "./bin/Release/simloid"
address  = "127.0.0.1"
port     = random.randint(7000, 9000)
bufsize  = 4096
scene_id = 3
robot_ids = [ [32,  0, 0.00 ] # normal hannah
            , [37,  0, 0.10 ] # random hannah
            , [37, 42, 1.00 ]
            , [37, 43, 1.00 ]
            , [37, 44, 1.00 ]
            , [37, 45, 1.00 ]
            , [37, 46, 1.00 ]
            , [37, 23, 0.10 ]
            , [37, 23, 0.25 ]
            , [37, 23, 0.50 ]
            , [37, 23, 1.00 ]
            , [31,  0, 0.00 ] # other robots
            , [11,  0, 0.00 ]
            , [20,  0, 0.00 ]
            , [40,  0, 0.00 ]
            , [50,  0, 0.00 ]
            , [60,  0, 0.00 ]
            , [80,  0, 0.00 ]
            , [90,  0, 0.00 ]
            ]



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
		self.recv_robot_info()


	def recv_robot_info(self):
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


	def change_model(self, model_id):
		cmd = "MODEL {0}\nDONE\n".format(model_id)
		self.sock.send(cmd)
		self.recv_robot_info()

	def change_to_random_model(self, model_id, instance, amlitude):
		cmd = "MODEL {0} {1} {2}\nDONE\n".format(model_id, instance, amlitude)
		self.sock.send(cmd)
		self.recv_robot_info()


def control_loop(joints):
	# simple p-ctrl holding the default position
	for j in joints:
		j.voltage = 5.0 * (j.def_pos - j.position)
	


def main(argv):
	random.seed()
	cur_model_id = 0
	subprocess.Popen([sim_path, "--port", str(port), "--robot", str(robot_ids[cur_model_id][0]), "--scene", str(scene_id)])
	time.sleep(0.5)
	simloid = Simloid(address, port)
	result = True

	cycles = 0

	while (result):
			try:
				cycles += 1
				result = simloid.loop()
				control_loop(simloid.joints)
				if cycles % 200 == 0:
					cur_model_id += 1
					if cur_model_id == len(robot_ids):
						cur_model_id = 0
					simloid.change_to_random_model(robot_ids[cur_model_id][0],robot_ids[cur_model_id][1],robot_ids[cur_model_id][2])

			except KeyboardInterrupt: # press CTRL + C to exit
				print(MSG + "Bye___")
				sys.exit()
		


if __name__ == "__main__": main(sys.argv)

