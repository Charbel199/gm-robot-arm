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

#define DELAY 5
#define SERVO_STEP 5
#define SERVO_STEP_SMALL 2

#define servo1_safe 146
#define servo2_safe 0
#define servo3_safe 90
#define servo4_safe 0
#define servo5_safe 180
#define servo6_safe 90

int x;
String instruction;
String str;
char move_instruction;
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
  pinMode(13, OUTPUT);
  servo1.attach(servoPin1);
  servo1.write(servo1_safe);
  servo2.attach(servoPin2);
  servo2.write(servo2_safe);
  servo3.attach(servoPin3);
  servo3.write(servo3_safe);
  servo4.attach(servoPin4);
  servo4.write(servo4_safe);
  servo5.attach(servoPin5);
  servo5.write(servo5_safe);
  servo6.attach(servoPin6);
  servo6.write(servo6_safe);
  Serial.setTimeout(1);
}

void loop() {
  free_drive_control();
}

void moveServo(Servo servoX, int from, int to, int delayValue){
  if(to>=0 && to<=180){
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


void manual_control(){
 while (!Serial.available());      // Loop till arduino receives a message
  instruction = Serial.readString();
  servo1_pos = servo1.read();
  servo2_pos = servo2.read();
  servo3_pos = servo3.read();
  servo4_pos = servo4.read();
  servo5_pos = servo5.read();
  servo6_pos = servo6.read();

  if(instruction == "R"){
      digitalWrite(13,HIGH);
      delay(250);
      digitalWrite(13,LOW);
      delay(250);
      servo_positions = "";
      servo_positions = servo_positions + "Servo 1: " + servo1_pos + "\t[" + servo2_pos + ", " + servo3_pos + ", " + servo4_pos + ", " + servo5_pos + ", " + servo6_pos +"]"; //stupid arduino string concatenation
      Serial.println(servo_positions);
    }
  else if(instruction == "W"){
      digitalWrite(13,HIGH);
      delay(250);
      digitalWrite(13,LOW);
      delay(250);
      while (!Serial.available()); //waiting for servo
      str = Serial.readString();
      //servo = str.toInt();
      servo = 1;
      Serial.println("Moving servo " + str);
      while (!Serial.available()); //waiting for angle
      str = Serial.readString();
      //value = str.toInt();
      value = 120;
      Serial.println("Going to angle " + str);
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
      Serial.println("Done");
    }
   }

void free_drive_control(){
  
  servo1_pos = servo1.read();
  servo2_pos = servo2.read();
  servo3_pos = servo3.read();
  servo4_pos = servo4.read();
  servo5_pos = servo5.read();
  servo6_pos = servo6.read();
  servo_positions = "";
  servo_positions = servo_positions + "Servo 1: " + servo1_pos + "\t[" + servo2_pos + ", " + servo3_pos + ", " + servo4_pos + ", " + servo5_pos + ", " + servo6_pos +"]"; //stupid arduino string concatenation
  Serial.println(servo_positions);
  while (!Serial.available());      // Loop till arduino receives a message
  move_instruction = Serial.read(); // String to char stupid 
  Serial.println(move_instruction);

  switch(move_instruction){
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

     case '1': moveServo(servo1, servo1_pos, servo1_pos+SERVO_STEP_SMALL, DELAY);
    break;
    case 'z': moveServo(servo1, servo1_pos, servo1_pos-SERVO_STEP_SMALL, DELAY);
    break;

    case '2': moveServo(servo2, servo2_pos, servo2_pos+SERVO_STEP_SMALL, DELAY);
    break;
    case 'x': moveServo(servo2, servo2_pos, servo2_pos-SERVO_STEP_SMALL, DELAY);
    break;

    case '3': moveServo(servo3, servo3_pos, servo3_pos+SERVO_STEP_SMALL, DELAY);
    break;
    case 'c': moveServo(servo3, servo3_pos, servo3_pos-SERVO_STEP_SMALL, DELAY);
    break;

    case '4': moveServo(servo4, servo4_pos, servo4_pos+SERVO_STEP_SMALL, DELAY);
    break;
    case 'v': moveServo(servo4, servo4_pos, servo4_pos-SERVO_STEP_SMALL, DELAY);
    break;

    case '5': moveServo(servo5, servo5_pos, servo5_pos+SERVO_STEP_SMALL, DELAY);
    break;
    case 'b': moveServo(servo5, servo5_pos, servo5_pos-SERVO_STEP_SMALL, DELAY);
    break;

    case '6': moveServo(servo6, servo6_pos, servo6_pos+SERVO_STEP_SMALL, DELAY);
    break;
    case 'n': moveServo(servo6, servo6_pos, servo6_pos-SERVO_STEP_SMALL, DELAY);
    break;
  }
  
  Serial.println("Done");
  }
   
