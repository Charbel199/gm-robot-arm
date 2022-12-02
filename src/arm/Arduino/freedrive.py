import serial
import time
arduino = serial.Serial(port='/dev/ttyACM0', baudrate=9600)
arduino.close()
arduino.open()

def write(x):                  # defining a function for sending and receiving a message
    arduino.write(bytes(x, 'utf-8'))
    return

while True:
    instruction = input("What do you want to do? x: exit, r: read, w: write").lower()
    if(instruction == "x"):
        break
    elif(instruction == "r"):
        write("Read")
        values = arduino.readline()
        print(values)
    elif(instruction == "w"):
        write("Write")
        servo = input("Enter servo number to move (1-6, 1 being the gripper): ")
        while(servo not in range(1,7)):
            servo = input("Invalid servo. Enter servo number to move (1-6, 1 being the gripper): ")
        write(servo)
        time.sleep(0.5)
        servoMoving = arduino.readline()
        print(servoMoving)

        value = input("Enter angle you wish to go to between 0 and 180 (90 and 180 for gripper): ")
        while(value not in range(0,181)):
            value = input("Invalid angle. Enter angle you wish to go to between 0 and 180 (90 and 180 for gripper): ")
        write(value)
        time.sleep(0.5)
        goingTo = arduino.readline()
        print(goingTo)
        while(True):
            done = arduino.readline()
            if (done == "Done"):
                break
    elif(instruction == "f"):
        write("Write")
        servo = input("Enter letter: (q,a / w,s / e,d / r,f / t,g / y,h) ")
        # Check if we need to read to empty buffer
        while (True):
            try:
                done = arduino.readline()
                if (done == "Done"):
                    break
            except Exception:
                pass

    else:
        print("Invalid instruction. Please try again.")

