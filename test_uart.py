
import serial
from time import sleep
import random

# Can be Downloaded from this Link
# https://pypi.python.org/pypi/pyserial

# Global Variables
ser = 0


# Function to Initialize the Serial Port
def init_serial():
    COMPORT = 3  # Enter Your COM Port Number Here.
    global ser  # Must be declared in Each Function
    ser = serial.Serial()
    ser.baudrate = 9600
    ser.port = "COM{}".format(COMPORT)  # COM Port Name Start from 0

    # ser.port = '/dev/ttyUSB0' #If Using Linux

    # Specify the TimeOut in seconds, so that SerialPort
    # Doesn't hangs
    ser.timeout = 10
    ser.open()  # Opens SerialPort

    # print port open or closed
    if ser.isOpen():
        print('Open: ' + ser.portstr)


# Function Ends Here


# Call the Serial Initilization Function, Main Program Starts from here
init_serial()

# temp = input('Type what you want to send, hit enter:\r\n')

# Writes to the SerialPort

while 1:
    # bytes = ser.readline()  #Read from Serial Port
    # print('You sent: ' + bytes)      #Print What is Read from Port
    temp1 = random.randint(0, 10)
    temp2 = random.randint(0, 10)
    ser.write(str.encode(format(temp1, '.1f')) + b' ')
    ser.write(str.encode(format(temp2, '.1f')) + b'\n')
    sleep(1)
# Ctrl+C to Close Python Window

