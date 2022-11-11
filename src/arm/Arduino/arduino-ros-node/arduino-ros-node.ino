#include <Servo.h>
#include <ros.h>
#include <rosserial_msgs/ServoPositions.h>

#define servoPin1 7
#define servoPin2 2
#define servoPin3 3
#define servoPin4 4
#define servoPin5 5
#define servoPin6 6

Servo servo1;   //Range: 90 to 180
Servo servo2;   //Range: 90 to 180
Servo servo3;   //Range: 90 to 180 
Servo servo4;   //Range: 90 to 180
Servo servo5;   //Range: 90 to 180
Servo servo6;   //Range: 90 to 180

//SafePoseValues:
#define servo1_safe 90
#define servo2_safe 90
#define servo3_safe 90
#define servo4_safe 90
#define servo5_safe 90
#define servo6_safe 180


ros::NodeHandle node;

void message(const rosserial_msgs::ServoPositions& servo_positions){
  int servo1_pos = servo_positions.servo1;
  //node.loginfo("servo1" + servo1_pos);
  }
ros::Subscriber<rosserial_msgs::ServoPositions> controller("/control/arm", &message);
 
void setup() { 
  Serial.begin(9600);
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  servo3.attach(servoPin3);
  servo4.attach(servoPin4);
  servo5.attach(servoPin5);
  servo6.attach(servoPin6);
  node.initNode();
  node.subscribe(controller);
} 
 
 
void loop() {
  while(!node.connected()){
    node.spinOnce();
    }
  node.loginfo("hello");
  /*
  //Setting the safe pose
  goToSafePose();

  //Go to the initial movement pose
  goToInitialPose();
  */
  delay(500);
}

//Safe pose of the robot that is out of view of the camera
void goToSafePose(){
  //TODO: 
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
