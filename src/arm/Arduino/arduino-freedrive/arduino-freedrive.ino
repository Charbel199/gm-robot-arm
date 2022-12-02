#include <Servo.h>
#define servoPin1 7
#define servoPin2 2
#define servoPin3 3
#define servoPin4 4
#define servoPin5 5
#define servoPin6 6

Servo servo1;   //Range: 90 to 180
Servo servo2;   //Range: 0 to 180
Servo servo3;   //Range: 0 to 180 
Servo servo4;   //Range: 0 to 180
Servo servo5;   //Range: 0 to 180
Servo servo6;   //Range: 0 to 180

#define DELAY 10
#define SERVO_STEP 2

int x;
String instruction;
float servo1_pos;
float servo2_pos;
float servo3_pos;
float servo4_pos;
float servo5_pos;
float servo6_pos;
int servo;
String servo_positions = "";
int value;


void setup() {
  Serial.begin(9600);
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  servo3.attach(servoPin3);
  servo4.attach(servoPin4);
  servo5.attach(servoPin5);
  servo6.attach(servoPin6);
  Serial.setTimeout(1);
}

void loop() {
  manual_control();
}


void manual_control(){
 while (!Serial.available());      // Loop till arduino receives a message
  instruction = Serial.readString();
  servo1_pos = servo1.read();
  servo2_pos = servo2.read();
  servo3_pos = servo3.read();
  servo4_pos = servo4.read();
  servo5_pos = servo5.read();
  servo6_pos = servo6.read();

  if(instruction == "Read"){
      servo_positions = "";
      servo_positions = servo_positions + "Servo 1: " + servo1_pos + "\tServo2: " + servo2_pos + "\tServo3: " + servo3_pos + "\tServo4: " + servo4_pos + "\tServo5: " + servo5_pos + "\tServo6: " + servo6_pos; //stupid arduino string concatenation
      Serial.print(servo_positions);
    }
  else if(instruction == "Write"){
      while (!Serial.available()); //waiting for servo
      servo = Serial.readString().toInt();
      Serial.print("Moving servo " + servo);
      while (!Serial.available()); //waiting for angle
      value = Serial.readString().toInt();
      Serial.print("Going to angle " + value);
      switch(servo) {
        case 1: moveServo(servo1, servo1_pos, value, DELAY);
        break;
        case 2: moveServo(servo2, servo2_pos, value, DELAY);
        break;
        case 3: moveServo(servo3, servo3_pos, value, DELAY);
        break;
        case 4: moveServo(servo4, servo4_pos, value, DELAY);
        break;
        case 5: moveServo(servo5, servo5_pos, value, DELAY);
        break;
        case 6: moveServo(servo6, servo6_pos, value, DELAY);
        break;
        default:
        break;
        }
      Serial.print("Done");
    }
   }

void free_drive_control(){
 while (!Serial.available());      // Loop till arduino receives a message
  instruction = Serial.readString();
  servo1_pos = servo1.read();
  servo2_pos = servo2.read();
  servo3_pos = servo3.read();
  servo4_pos = servo4.read();
  servo5_pos = servo5.read();
  servo6_pos = servo6.read();

  servo_positions = servo_positions + "Servo 1: " + servo1_pos + "\tServo2: " + servo2_pos + "\tServo3: " + servo3_pos + "\tServo4: " + servo4_pos + "\tServo5: " + servo5_pos + "\tServo6: " + servo6_pos; //stupid arduino string concatenation
  Serial.print(servo_positions);

  switch(instruction){
    case 'q': moveServo(servo1, servo1_pos, servo1_pos+SERVO_STEP, DELAY);
    break;
    case 'a': moveServo(servo1, servo1_pos, servo1_pos-SERVO_STEP, DELAY);
    break;

    case 'w': moveServo(servo2, servo2_pos, servo2_pos+SERVO_STEP, DELAY);
    break;
    case 's': moveServo(servo2, servo2_pos, servo2_pos-SERVO_STEP, DELAY);
    break;

    case 'e': moveServo(servo3, servo3_pos, servo3_pos+SERVO_STEP, DELAY);
    break;
    case 'd': moveServo(servo3, servo3_pos, servo3_pos-SERVO_STEP, DELAY);
    break;

    case 'r': moveServo(servo4, servo4_pos, servo4_pos+SERVO_STEP, DELAY);
    break;
    case 'f': moveServo(servo4, servo4_pos, servo4_pos-SERVO_STEP, DELAY);
    break;

    case 't': moveServo(servo5, servo5_pos, servo5_pos+SERVO_STEP, DELAY);
    break;
    case 'g': moveServo(servo5, servo5_pos, servo5_pos-SERVO_STEP, DELAY);
    break;

    case 'y': moveServo(servo6, servo6_pos, servo6_pos+SERVO_STEP, DELAY);
    break;
    case 'h': moveServo(servo6, servo6_pos, servo6_pos-SERVO_STEP, DELAY);
    break;
  }
  Serial.print("Done");
    }
   }
void moveServo(Servo servoX, int from, int to, int delayValue){
  if(to>90 && to<180){
    if(from > to){
        for(int i=from; i>to; i--){
            servoX.write(i);
            delay(delayValue);
            }
           }


  if(from < to){
  for(int i=from; i<to; i++){
    servoX.write(i);
    delay(delayValue);
  }
  }
  }
}
