#include <Servo.h>
#include <ros.h>
#include <rosserial_msgs/ServoPositions.h>
#include <std_msgs/Bool.h>

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

#define EPSILON 1
#define DELAY 3

//SafePoseValues:
#define servo1_safe 142
#define servo2_safe 0
#define servo3_safe 90
#define servo4_safe 0
#define servo5_safe 180
#define servo6_safe 90


ros::NodeHandle nh;
std_msgs::Bool bool_msg;

ros::Publisher pub("control/move_done", &bool_msg);

void message(const rosserial_msgs::ServoPositions& servo_positions){
  nh.loginfo("RECEIVED MOVE");

  //Desired positions
  int servo1_des_pos = servo_positions.servo1;
  int servo2_des_pos = servo_positions.servo2;
  int servo3_des_pos = servo_positions.servo3;
  int servo4_des_pos = servo_positions.servo4;
  int servo5_des_pos = servo_positions.servo5;
  int servo6_des_pos = servo_positions.servo6;

  // Current positions
  float servo1_pos = servo1.read();

  char servo1_pos_str[6];
  char servo1_des_pos_str[4];
  char servo2_des_pos_str[4];
  char servo3_des_pos_str[4];
  char servo4_des_pos_str[4];
  char servo5_des_pos_str[4];
  char servo6_des_pos_str[4];
  //dtostrf(servo1_pos,2,2,servo1_pos_str);
  //nh.loginfo(servo1_pos_str);

  
  float servo2_pos = servo2.read();
  float servo3_pos = servo3.read();
  float servo4_pos = servo4.read();
  float servo5_pos = servo5.read();
  float servo6_pos = servo6.read();

  //flash LED
  digitalWrite(13, HIGH);   
  delay(250);             
  
  if(abs(servo6_des_pos - servo6_pos) > EPSILON){
  moveServo(servo6, servo6_pos, servo6_des_pos, DELAY*4);
  String(servo6_des_pos).toCharArray(servo6_des_pos_str,4);
  nh.loginfo(servo6_des_pos_str);
  }
  
  if(abs(servo5_des_pos - servo5_pos) > EPSILON){
  moveServo(servo5, servo5_pos, servo5_des_pos, DELAY*6);
  String(servo5_des_pos).toCharArray(servo5_des_pos_str,4);
  nh.loginfo(servo5_des_pos_str);
  }
  
  if(abs(servo4_des_pos - servo4_pos) > EPSILON){
  moveServo(servo4, servo4_pos, servo4_des_pos, DELAY*2);
  String(servo4_des_pos).toCharArray(servo4_des_pos_str,4);
  nh.loginfo(servo4_des_pos_str);
  }
  
  if(abs(servo3_des_pos - servo3_pos) > EPSILON){
  moveServo(servo3, servo3_pos, servo3_des_pos, DELAY*2);
  String(servo3_des_pos).toCharArray(servo3_des_pos_str,4);
  nh.loginfo(servo3_des_pos_str);
  }
    
  if(abs(servo2_des_pos - servo2_pos) > EPSILON){
  moveServo(servo2, servo2_pos, servo2_des_pos, DELAY);
  String(servo2_des_pos).toCharArray(servo2_des_pos_str,4);;
  nh.loginfo(servo2_des_pos_str);
  }
  
  if(abs(servo1_des_pos - servo1_pos) > EPSILON){
  moveServo(servo1, servo1_pos, servo1_des_pos, DELAY*5);
  String(servo1_des_pos).toCharArray(servo1_des_pos_str,4);
  nh.loginfo(servo1_des_pos_str);
  }
    
       
  digitalWrite(13, LOW); 
  pub.publish(&bool_msg);
  nh.loginfo("COMPLETED MOVE");
  }

 ros::Subscriber<rosserial_msgs::ServoPositions> controller("control/arm", &message);
 
void setup() { 
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
  
  nh.loginfo("1");
  nh.initNode();
  nh.loginfo("2");
  nh.advertise(pub);
  nh.logdebug("3");
  bool_msg.data = true;
  nh.subscribe(controller);
  nh.logdebug("INITIALIZED NODE");
  //goToInitialPose();
} 
 
 
void loop() {
   nh.spinOnce();
   delay(50);
   
}

void moveServo(Servo servoX, int from, int to, int delayValue){
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
