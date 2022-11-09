#include <Servo.h>

#define servoPin1 7
#define servoPin2 2
#define servoPin3 3
#define servoPin4 4
#define servoPin5 8
#define servoPin6 6

Servo servo1;   //Range: 92 to 180
Servo servo2; 
Servo servo3; 
Servo servo4;
Servo servo5;
Servo servo6;

//SafePoseValues:
#define servo6_safe 90
#define servo5_safe 90
#define servo4_safe 90
#define servo3_safe 90
#define servo2_safe 90
#define servo1_safe 180

 
void setup() { 
  Serial.begin(9600);
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  servo3.attach(servoPin3);
  servo4.attach(servoPin4);
  servo5.attach(servoPin5);
  servo6.attach(servoPin6);
  
} 
 
 
void loop() {
  //Setting the safe pose
  goToSafePose();

  //Go to the initial movement pose
  goToInitialPose();

  //Begin the loops for random pattern movement
  for( int i =180; i > 0; i--){
    servo6.write(i);
    delay(25);
  }

  servo1.write(1500);
  delay(500);
  servo1.write(2800);

  for( int i =0; i < 180; i++){
    servo6.write(i);
    delay(25);
  }

  //Gripping code
  servo1.write(1500);
  delay(500);
  servo1.write(2800);
  
}

//Safe pose of the robot that is out of view of the camera
void goToSafePose(){
  moveServo(servo6,servo6.read(),
  delay(1000);
}

//Initial pose to begin the movement
void goToInitialPose(){
  servo6.write(180);
  servo5.write(80);
  servo4.write(90);
  servo3.write(90);
  servo2.write(90);
  servo1.write(180);
  delay(1000);
}

void moveServo(Servo servoX, int from, int to, int delayValue){
  for( int i=from; i>to; i--){
    servoX.write(i);
    delay(delayValue);
  }
}
