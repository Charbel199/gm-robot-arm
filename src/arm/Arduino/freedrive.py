import serial
import time
from pynput import keyboard
arduino = serial.Serial(port='/dev/ttyACM0', baudrate=9600)
arduino.close()
arduino.open()

def write(x):                  # defining a function for sending and receiving a message
    arduino.write(bytes(str(x), 'utf-8'))
    time.sleep(0.3)
    return

def on_key_press(key):

    if 'char' in dir(key):
            print(f'Key {key.char} was pressed')
            if key.char in ['q','a','w','s','e','d','r','f','t','g','y','h']:
                listener.stop()
                write(key.char)
    return

while True:
    instruction = input("What do you want to do? (x: exit, r: read, w: write, f: freedrive): ").lower()
    if(instruction == "x"):
        break
    elif(instruction == "r"):
        write("R")
        values = arduino.readline().strip().decode("utf-8")
        print(values)
    elif(instruction == "w"):
        write("W")
        servo = int(input("Enter servo number to move (1-6, 1 being the gripper): "))
        while(servo not in range(1,7)):
            servo = int(input("Invalid servo. Enter servo number to move (1-6, 1 being the gripper): "))
        write(servo)
        time.sleep(0.5)
        servoMoving = arduino.readline().strip().decode("utf-8")
        print(servoMoving)

        value = int(input("Enter angle you wish to go to between 0 and 180 (90 and 180 for gripper): "))
        while(value not in range(0,181)):
            value = int(input("Invalid angle. Enter angle you wish to go to between 0 and 180 (90 and 180 for gripper): "))
        write(value)
        time.sleep(0.5)
        goingTo = arduino.readline().strip().decode("utf-8")
        print(goingTo)
        while(True):
            done = arduino.readline().strip().decode("utf-8")
            if (done == "Done"):
                break
    elif(instruction == "f"):
        listener = keyboard.Listener(on_press=on_key_press)
        listener.start()
        while(True):
            values = arduino.readline().strip().decode("utf-8")
            print(values)
            #servo = input("Enter letter: (q,a / w,s / e,d / r,f / t,g / y,h) ")
            #write(servo)
            # Check if we need to read to empty buffer
            while (True):
                try:
                    done = arduino.readline().strip().decode("utf-8")
                    if (done == "Done"):
                        print("DONE")
                        listener = keyboard.Listener(on_press=on_key_press)
                        listener.start()
                        break
                except Exception:
                    pass

    else:
        print("Invalid instruction. Please try again.")

