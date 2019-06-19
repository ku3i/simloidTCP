+-----------------------------------------------------------------------------+
|
|   SIMLOID Documentation Version 0.1.0
|
+-----------------------------------------------------------------------------+
|
|   Contents
|       Installation
|       Starting the Server
|       Robots
|       Scenes
|       TCP Status and Control Interface
|       Traits Message
|       Sensor Status Message
|       Motor Control Message
|           Voltage Control
|           PID Control
|           Max. Torque
|           Add Impulse
|           Other
|
|
+--------------+--------------------------------------------------------------+
| Installation |
+--------------+
|
|   Simloid uses the version 0.13 of ODE, be sure you have the provided file
|   'libode.a' installed in '/usr/local/lib/libode.a
|
|   Build the lib and install headers:
|   $ ./configure --enable-double-precision
|   $ make
|   $ make install
|
|
+---------------------+-------------------------------------------------------+
| Starting the Server |
+---------------------+
|
|   To start simloid type:
|
|   $ ./simloid --port <port> --robot <robot_id> --scene <scene_id>
|
|   Command line parameters can be omitted. Instead you can edit the
|   configuration file 'simloid.conf'.
|
|
+--------+--------------------------------------------------------------------+
| Robots |
+--------+
|
|   10: karlsims    : tadpole_0
|   11: karlsims    : tadpole_1
|   20: crawler     : crawler
|   30: fourlegged  : wildcat0
|   31: fourlegged  : wildcat1
|   40: biped       : ostrich
|   50: biped       : humanoid
|   60: nolegs      : worm
|
|   90: standard    : pendulum
|   91: standard    : double_pendulum
|   92: standard    : rotor_horizontal
|   93: standard    : rotor_vertical
|   94: standard    : rotor_axial
|   95: standard    : pendulum without accel
|
|
+--------+--------------------------------------------------------------------+
| Scenes |
+--------+
|
|    0: empty world     : flat, even, ground
|    1: hurdles         : bars with successively increasing step height
|    2: hills           : cosine hills with increasing slope and height
|    3: shaky ground    : small rocks all over the ground
|
|
+----------------------------------+------------------------------------------+
| TCP Status and Control Interface |
+----------------------------------+
|
|   The communication to simloid (server) starts with a traits handshake.
|
|       1.server sends traits
|       2.client sends "ACK\n"
|
|       3.server sends status message
|       4.client sends control message
|       (repeat 3+4)
|
|
+----------------+------------------------------------------------------------+
| Traits Message |
+----------------+
|
|   Description of the robot traits message:
|
|   Num_Bodies Num_Joints Num_Accels \n
|   joint_id_0   type symmetric_joint stop_lo stop_hi def_pos name \n
|   joint_id_1   type symmetric_joint stop_lo stop_hi def_pos name \n
|   ...
|   joint_id_N-1 type symmetric_joint stop_lo stop_hi def_pos name \n
|
|
+-----------------------+-----------------------------------------------------+
| Sensor Status Message |
+-----------------------+
|
|   The status message consists of floats only, separated by blanks,
|   terminated by '\n' and has always the same number of float elements.
|   However the message length varies with respect to the numbers being send.
|
|   time
|   joint position (pos)        1..Num_Joints
|   joint velocity (vel)        1..Num_Joints
|   acceleration sensors (xyz)  1..Num_Accels
|   body pos (xyz) + vel (xyz)  1..Num_Bodies
|
|
+-----------------------+-----------------------------------------------------+
| Motor Control Message |
+-----------------------+
|
|   A control message can carry multiple motor control commands and must be
|   terminated with the command "DONE\n".
|   Each motor control command must be terminated by a newline ('\n').
|
|
+-- VOLTAGE CONTROL ----------------------------------------------------------+
|
|   Applicable motor voltage must be in range [-1,+1].
|   If you set a motor voltage, the joint stays in VOLTAGE CONTROL mode
|   until a position (PID CONTROL mode) is being set.
|
|   1.) Set the same voltage to every joint
|
|       Command: UA <voltage>
|       Example: "UA 0.5\n"
|
|   2.) Set an individual voltage to each joint [0...N-1]
|
|       Command: UX <voltage_0> <voltage_1> ... <voltage_N-1>
|       Example: "UX 0.5 0.2 -0.4 1.0 -0.34\n"
|
|   3.) Set only voltage to a single joint with joint_ID
|
|       Command: UI <joint_ID> <voltage>
|       Example: "UI 2 0.1\n"
|
|
+-- PID CONTROL --------------------------------------------------------------+
|
|   Applicable positions must be from range [-1,+1] = [-pi,+pi].
|   Reachable range can be smaller if joint stops are set.
|   If you set a joint position, the joint stays in PID CONTROL mode
|   until a control voltage is being set.
|
|   1.) Set the same position to every joint
|
|       Command: PA <position>
|       Example: "PA 0.3\n"
|
|   2.) Set an individual position to each joint [0...N-1]
|
|       Command: PX <position_0> <position_1> ... <position_N-1>
|       Example: "PX 0.2 0.4 0.5 -1.0 -0.22\n"
|
|   3.) Set only position to joint with joint_ID
|
|       Command: PI <joint_ID> <position>
|       Example: "PI 1 1.0\n"
|
|
+-- MAX TORQUE ---------------------------------------------------------------+
|
|   This command is used in combination with the PID control mode to set
|   the maximal available torque for the position controller which can be
|   used to enforce the given set-point position.
|
|   1.) Set the same max. torque to every joint
|
|       Command: TA <maxtorque>
|       Example: "TA 0.5\n"
|
|   2.) Set an individual max. torque to each joint [0...N-1]
|
|       Command: TX <maxtorque_0> <maxtorque_1> ... <maxtorque_N-1>
|       Example: "TX 0.5 0.2 -0.4 1.0 -0.34\n"
|
|   3.) Set only max. torque to joint with joint_ID
|
|       Command: TI <joint_ID> <maxtorque>
|       Example: "TI 2 0.1\n"
|
|
+-- ADD IMPULSE --------------------------------------------------------------+
|
|   This command is used to set forces to the body segments of the robot.
|
|   1.) Set a force with given vector to the center of mass to the body
|       with body_ID.
|
|       Command: IM <body_ID> <Force_X> <Force_Y> <Force_Z>
|       Example: "IM 3 0.2 0.4 0.6\n"
|
|
+-- OTHER --------------------------------------------------------------------+
|
|   1.) Switch gravity on/off.
|
|       Command: GRAVITY <ON|OFF>
|       Example: "GRAVITY ON\n"
|
|   2.) Reset the simulation.
|
|       Command: RESET
|
|   3.) Save and simulation state.
|
|       Command: SAVE
|
|   4.) Restore previously saved simulation state.
|
|       Command: RESTORE
|
|   5.) Reset the clock (simulation time).
|
|       Command: NEWTIME
|
|   6.) Terminate a control message.
|
|       Command: DONE
|
|   7.) Exit the simulation and close the socket.
|
|       Command: EXIT
|
|
+-----------------------------------------------------------------------------+
