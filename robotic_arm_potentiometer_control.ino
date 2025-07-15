// C++ code for Arm-old Schwarzenegger
//
#include <Servo.h>

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

//Minimum and Maximum pulse length count
#define Base_min 150
#define Base_max 600 
#define Shoulder_min 150
#define Shoulder_max 500 
#define Elbow_min 150
#define Elbow_max 500 
#define Wrist_min 150
#define Wrist_max 480
#define Gripper_min 150
#define Gripper_max 250

//Set the motors to their channels 
const int P1=0; //Base
const int P2=1; //Shoulder
const int P3=2; //Elbow
const int P4=3; //Wrist
const int P5=4; //Gripper

void setup()
{
  //A4 and A5 are internally connected to SDA and SCL for I2C
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A6, INPUT);
  pinMode(A7, INPUT);

     
  Serial.begin(9600);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
}

void loop()
{
  
  //Read from Potentiometer 1 (Base) and adjust the servo
   moveServo(P1, Base_min, Base_max);

  //Read from Potentiometer 2 (Shoulder) and adjust the servo
   moveServo(P2, Shoulder_min, Shoulder_max);
   
  //Read from Potentiometer 3 (Elbow) and adjust the servo
   moveServo(P3, Elbow_min, Elbow_max);
   
  //Read from Potentiometer 4 (Wrist) and adjust the servo
   moveServo(P4, Wrist_min, Wrist_max);
   
  //Read from Potentiometer 5 (Gripper) and adjust the servo
   moveServo(P5, Gripper_min, Gripper_max);

   //Add delay
   delay(20);

}


void moveServo(int P, int min, int max)
{
  int pot; 
  float deg, val;
  switch (P)
  {
    case P1: pot = analogRead(A0);break;
    case P2: pot = analogRead(A1);break;
    case P3: pot = analogRead(A2);break;
    case P4: pot = analogRead(A6);break;
    case P5: pot = analogRead(A7);
  }
    
  Serial.print("Potentiometer "); Serial.print(P); Serial.print(" = "); Serial.print(pot); //Print the value from the potentiometer
  deg = float(map(pot,0,1023,0,180.0));  //Find the degree equivalent of the value
  Serial.print(",\t Potentiometer "); Serial.print(P); Serial.print(" mapped = "); Serial.println(deg); //print the degree value
  val = float(map(deg, 0, 180.0, min, max));     // scale it for use with the servo
  pwm.setPWM(P,0,val);
}
