#include <Servo.h>

#define servoPin1 7
#define servoPin2 2
#define servoPin3 3
#define servoPin4 4
#define servoPin5 8
#define servoPin6 6

Servo servo1;
Servo servo2; 
Servo servo3; 
Servo servo4;
Servo servo5;
Servo servo6;

float servo1_pos, servo2_pos, servo3_pos, servo4_pos, servo5_pos, servo6_pos;

 
void setup() { 
  Serial.begin(9600);
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  servo3.attach(servoPin3);
  servo4.attach(servoPin4);
  servo5.attach(servoPin5);
  servo6.attach(servoPin6);
  servo6.write(180);
  servo1.write(92);
  
} 
 
 
void loop() {
  servo1_pos = servo1.read();
  Serial.print(servo1_pos);
  Serial.print("\t");
  servo2_pos = servo2.read();
  Serial.print(servo2_pos);
  Serial.print("\t");
  servo3_pos = servo3.read();
  Serial.print(servo3_pos);
  Serial.print("\t");
  servo4_pos = servo4.read();
  Serial.print(servo4_pos);
  Serial.print("\t");
  servo5_pos = servo5.read();
  Serial.print(servo5_pos);
  Serial.print("\t");
  servo6_pos = servo6.read();
  Serial.print(servo6_pos);
  Serial.print("\t");
  delay(1500);
  Serial.print("\n");
}
