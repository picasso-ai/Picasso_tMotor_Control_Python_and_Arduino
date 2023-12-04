import argparse
import tMotor as tMotor
import time
import numpy as np

# Parse command-line arguments
parser = argparse.ArgumentParser()
parser.add_argument('--file_name', type=str, default='C:/Users/taylo/OneDrive/Documents/Desktop/GradSchool/Lab/tMotor Controllers/Data/tMotor.csv', help='Name of the CSV file')
parser.add_argument('--amplitude', type=float, default=1.0, help='Amplitude value')
args = parser.parse_args()

# ComPort Setup
ComPort = 'com4'
# ComPort Setup

# Motor Setup
motor = tMotor.TMOTOR(ComPort)
# Motor Setup

# Time Control
now = 0
t_pr1 = 0
Delta_T1 = 0.04
start = time.time()
# Time Control

while True:
    # Receive Data
    motor.read()
    motor.decode()
    # Receive Data

    now = time.time() - start

    if now - t_pr1 > Delta_T1:
        t_pr1 = now

        # Send Data
        torque = args.amplitude * np.sin(now)
        motor.sendTorque(torque)
        # Send Data

        # Store Data in CSV
        with open(args.file_name, 'a') as file:
            file.write(f'{now},{torque},{motor.torque}\n')
        # Store Data in CSV
