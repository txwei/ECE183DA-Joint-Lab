"""lab3_controller controller."""
import numpy as np
import matplotlib.pyplot as plt
from controller import Supervisor
import control_algorithm 
#include <stdlib.h>

# create the Supervisor instance.
supervisor = Supervisor()
robot = supervisor.getFromDef('robot')
trans_field = robot.getField('translation')
rotation_field = robot.getField('rotation')

# get the time step of the current world.
timestep = int(supervisor.getBasicTimeStep())

# get instances of devices of the supervisor
motor_L = supervisor.getDevice('motor_L')
motor_R = supervisor.getDevice('motor_R')

compass = supervisor.getDevice('compass')
lidar_R = supervisor.getDevice('lidar_R')
lidar_F = supervisor.getDevice('lidar_F')
gyro = supervisor.getDevice('gyro')

compass.enable(timestep)
lidar_R.enable(timestep)
lidar_F.enable(timestep)
gyro.enable(timestep)

# initialize the motors
motor_L.setPosition(float('inf'))
motor_L.setVelocity(0)
motor_R.setPosition(float('inf'))
motor_R.setVelocity(0)

# initialize arrays to store states and sensor outputs
compass_data = np.zeros((10*1000,2))
lidar_R_data = np.zeros((10*1000,1))
lidar_F_data = np.zeros((10*1000,1))
gyro_data = np.zeros((10*1000,1))

position_data = np.zeros((10*1000,2))
orientation_data = np.zeros((10*1000,1))

# get control signals from the control algorithm
# for j in range(3):
trajectory_num = 18 # change this number to test out new trajectories
control_signals = control_algorithm.get_control_signals(trajectory_num)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
i = 0
while supervisor.step(timestep) != -1:
	v_l = control_signals[i][0]
	v_r = control_signals[i][1]
	motor_L.setVelocity(-v_r)
	motor_R.setVelocity(-v_l)
	lidar_R_data[i] = lidar_R.getValue()/4096*10
	lidar_F_data[i] = lidar_F.getValue()/4096*10
	gyro_data[i] = gyro.getValues()[1]
	compass_data[i] = [compass.getValues()[2],compass.getValues()[0]]

	trans = trans_field.getSFVec3f()
	position_data[i] = [-trans[2],-trans[0]]
	rotat = rotation_field.getSFRotation()
	orientation_data[i] = rotat[3]
	i = i+1
	if i == 10*1000:
  		break

output = np.hstack((position_data,orientation_data,lidar_F_data,lidar_R_data,gyro_data,compass_data))
output = np.transpose(output)
np.savetxt("data" + str(trajectory_num) + ".csv", output, delimiter=",")
# supervisor.simulationReset()
# supervisor.simulationResetPhysics()