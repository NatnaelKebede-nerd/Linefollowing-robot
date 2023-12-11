
// /// Define the arduino
#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

/// Include

#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
using namespace std;

// Create the node handle object
ros::NodeHandle  nh;

// Define the Ultrasonic Sensor input pins
static const int CH_Ultrasonic_trig = 4;
static const int CH_Ultrasonic_echo = 3;
float duration; 
float obstacle_distance = 10;

static float left_mtr_ctr_pwm;
static float right_mtr_ctr_pwm;

// Define the IR input channel pins
static const int CH_IRSENSOR_LEFT = 5;
static const int CH_IRSENSOR_RIGHT = 6;
static const int CH_IRSENSOR_CENTER = 7;

// Define the motor output ports
static const int CH_LEFTMOTR_OUT_A = 12;
static const int CH_LEFTMOTR_OUT_B = 13;
static const int CH_LEFTMOTR_OUT_EN = 11;
static const int CH_RIGHTMOTR_OUT_A = 8;
static const int CH_RIGHTMOTR_OUT_B = 9;
static const int CH_RIGHTMOTR_OUT_EN = 10;

// Define sensor Array for Three IR sensors and intialize them
int IR_sensors[3] = {0, 0, 0};

// Define Error for line Follwing
float error = 0;

// Define the message and the publishers, Str 
std_msgs::Int32 agv_IR_msg;
std_msgs::Int32 agv_Ultrasonic_msg;

ros::Publisher publisher_IRSensor("IR_Sensor_Feedback", &agv_IR_msg);
ros::Publisher publisher_Ultrasonic("Ultrasonic_Input", &agv_Ultrasonic_msg);

void cb_left_motor( const std_msgs::Float32& cmd_leftmotor_pwm){
   // THIS SHOULD BE PUBLSE OUT FOR PWM OUTPUT OF THE DC MOTOR

  // Spilit the command message and intor direction and speed
  // float left_mtr_ctr_pwm;
 left_mtr_ctr_pwm = cmd_leftmotor_pwm.data;

  // if (left_mtr_ctr_pwm > 0) {
  //    digitalWrite(CH_LEFTMOTR_OUT_A, HIGH);
  //    digitalWrite(CH_LEFTMOTR_OUT_B, LOW);
  //  } else if (left_mtr_ctr_pwm < 0) {
  //    digitalWrite(CH_LEFTMOTR_OUT_A, LOW);
  //    digitalWrite(CH_LEFTMOTR_OUT_B, HIGH);
  //  } else if (left_mtr_ctr_pwm == 0) {
  //    digitalWrite(CH_LEFTMOTR_OUT_A, LOW);
  //    digitalWrite(CH_LEFTMOTR_OUT_B, LOW);
  //  }

  // left_mtr_ctr_pwm = constrain(abs(left_mtr_ctr_pwm), 0 , 180);
  // digitalWrite(CH_LEFTMOTR_OUT_EN, left_mtr_ctr_pwm);
  // // delay(500);
  
}

// void cb_left_motor_direction( const std_msgs::String& cmd_leftmotor_dir){

//   //The cmd_msg.data holds in a string that information on the EN-value and for motor spin direction
//   // Spilit the command message and intor direction and speed

//   static String left_mtr_ctr_dir = cmd_leftmotor_dir.data;

//   // For direction use the frd
//   if (left_mtr_ctr_dir == "frd") {
//     digitalWrite(CH_LEFTMOTR_OUT_A, HIGH);
//     digitalWrite(CH_LEFTMOTR_OUT_B, LOW);
//   } else if (left_mtr_ctr_dir == "rev") {
//     digitalWrite(CH_LEFTMOTR_OUT_A, LOW);
//     digitalWrite(CH_LEFTMOTR_OUT_B, HIGH);
//   } else if (left_mtr_ctr_dir == "STOP") {
//     digitalWrite(CH_LEFTMOTR_OUT_A, LOW);
//     digitalWrite(CH_LEFTMOTR_OUT_B, LOW);
//   }
   
// }

void cb_right_motor( const std_msgs::Float32& cmd_rightmotor_pwm){

  //The cmd_msg.data holds in a string that information on the EN-value and for motor spin direction
  // Spilit the command message and intor direction and speed
  // float right_mtr_ctr_pwm;
   right_mtr_ctr_pwm = cmd_rightmotor_pwm.data;

  // if (right_mtr_ctr_pwm > 0) {
  //    digitalWrite(CH_RIGHTMOTR_OUT_A, HIGH);
  //    digitalWrite(CH_RIGHTMOTR_OUT_B, LOW);
  //  } else if (right_mtr_ctr_pwm < 0) {
  //    digitalWrite(CH_RIGHTMOTR_OUT_A, LOW);
  //    digitalWrite(CH_RIGHTMOTR_OUT_B, HIGH);
  //  } else if (right_mtr_ctr_pwm == 0) {
  //    digitalWrite(CH_RIGHTMOTR_OUT_A, LOW);
  //    digitalWrite(CH_RIGHTMOTR_OUT_B, LOW);
  //  }

  // // right_mtr_ctr_pwm = right_mtr_ctr_pwm + PID_value;
  // right_mtr_ctr_pwm = constrain(abs(right_mtr_ctr_pwm), 0, 255);
  // digitalWrite(CH_RIGHTMOTR_OUT_EN, right_mtr_ctr_pwm);
  // // delay(500);
  
}

// void cb_right_motor_direction( const std_msgs::String& cmd_rightmotor_dir){

//   //The cmd_msg.data holds in a string that information on the EN-value and for motor spin direction
//   // Spilit the command message and intor direction and speed

//   static String right_mtr_ctr_dir = cmd_rightmotor_dir.data;
//   // Serial.print(mtr_ctr_dir);

//   // For direction use the frd

//   if (right_mtr_ctr_dir == "frd") {
//     digitalWrite(CH_RIGHTMOTR_OUT_A, HIGH);
//     digitalWrite(CH_RIGHTMOTR_OUT_B, LOW);
//   } else if (right_mtr_ctr_dir == "rev") {
//     digitalWrite(CH_RIGHTMOTR_OUT_A, LOW);
//     digitalWrite(CH_RIGHTMOTR_OUT_B, HIGH);
//   } else if (right_mtr_ctr_dir == "STOP") {
//     digitalWrite(CH_RIGHTMOTR_OUT_A, LOW);
//     digitalWrite(CH_RIGHTMOTR_OUT_B, LOW);
//   } 
  // delay(500);                 
// }

// intialize the subscribers for motor spped and directiona 
ros::Subscriber<std_msgs::Float32> subscriber_leftmotor("cmd_left_motor", cb_left_motor);
ros::Subscriber<std_msgs::Float32> subscriber_rightmotor("cmd_right_motor", cb_right_motor);

// ros::Subscriber<std_msgs::String> subscriber_leftmotor_direction("cmd_left_motor_direction", cb_left_motor_direction);
// ros::Subscriber<std_msgs::String> subscriber_rightmotor_direction("cmd_right_motor_direction", cb_right_motor_direction);

void setup() {

  /// Initialize the node
  nh.initNode();

  //Declar ultrasonic sensor input output
  pinMode(CH_Ultrasonic_trig, OUTPUT);  
	pinMode(CH_Ultrasonic_echo, INPUT); 


  //Declar IR sensor input
  pinMode(CH_IRSENSOR_LEFT, INPUT);
  pinMode(CH_IRSENSOR_RIGHT, INPUT);
  pinMode(CH_IRSENSOR_CENTER, INPUT);

  //Declar DC motor output
  pinMode (CH_LEFTMOTR_OUT_A, OUTPUT);
  pinMode (CH_LEFTMOTR_OUT_B, OUTPUT);
  pinMode (CH_LEFTMOTR_OUT_EN, OUTPUT);
  pinMode (CH_RIGHTMOTR_OUT_A, OUTPUT);
  pinMode (CH_RIGHTMOTR_OUT_B, OUTPUT);
  pinMode (CH_RIGHTMOTR_OUT_EN, OUTPUT);

  /// create the publishers
  nh.advertise(publisher_IRSensor);
  nh.advertise(publisher_Ultrasonic);

  /// Subscribe to motors speed topic
  nh.subscribe(subscriber_leftmotor);
  nh.subscribe(subscriber_rightmotor);


  // nh.negotiateTopics();

  /// Subscribe to motors dir topic
  // nh.subscribe(subscriber_leftmotor_direction);
  // nh.subscribe(subscriber_rightmotor_direction);

 //intialize the motors to start in Zero position
  digitalWrite(CH_RIGHTMOTR_OUT_A, LOW);
	digitalWrite(CH_RIGHTMOTR_OUT_B, LOW);
	digitalWrite(CH_LEFTMOTR_OUT_A, LOW);
	digitalWrite(CH_LEFTMOTR_OUT_B, LOW);
  
  //setting serial monitor at a default baund rate of 9600/115200
  Serial.begin(57600);                     
  delay(500);
  Serial.println("Started !!");
  delay(1000);

}
 
void loop() {
  

  // Get Obstacle distance from Ultrasonic sensor
  read_Ultrasonic_sensor_values();

  //Publish obstacle data to ROS Topic
  agv_Ultrasonic_msg.data = obstacle_distance;
  publisher_Ultrasonic.publish(&agv_Ultrasonic_msg);

  //Get Gained Error fro IR Sensors
  read_IR_sensor_values();

  //Publish Gained Error to ROS topic
  agv_IR_msg.data = error;
  publisher_IRSensor.publish(&agv_IR_msg);

  // digitalWrite(CH_RIGHTMOTR_OUT_A, LOW);
  // digitalWrite(CH_RIGHTMOTR_OUT_B, HIGH);

  // digitalWrite(CH_RIGHTMOTR_OUT_EN, 20);

  //  digitalWrite(CH_LEFTMOTR_OUT_A, HIGH);
  // digitalWrite(CH_LEFTMOTR_OUT_B, LOW);

  // digitalWrite(CH_LEFTMOTR_OUT_EN, 20);

  //Spin this node Once
  if (right_mtr_ctr_pwm > 0) {
     digitalWrite(CH_RIGHTMOTR_OUT_A, HIGH);
     digitalWrite(CH_RIGHTMOTR_OUT_B, LOW);
   } else if (right_mtr_ctr_pwm < 0) {
     digitalWrite(CH_RIGHTMOTR_OUT_A, LOW);
     digitalWrite(CH_RIGHTMOTR_OUT_B, HIGH);
   } else if (right_mtr_ctr_pwm == 0) {
     digitalWrite(CH_RIGHTMOTR_OUT_A, LOW);
     digitalWrite(CH_RIGHTMOTR_OUT_B, LOW);
   }

  // right_mtr_ctr_pwm = right_mtr_ctr_pwm + PID_value;
  right_mtr_ctr_pwm = constrain(abs(right_mtr_ctr_pwm), 0, 255);
  digitalWrite(CH_RIGHTMOTR_OUT_EN, right_mtr_ctr_pwm);

  // delay(500);

    if (left_mtr_ctr_pwm > 0) {
     digitalWrite(CH_LEFTMOTR_OUT_A, HIGH);
     digitalWrite(CH_LEFTMOTR_OUT_B, LOW);
   } else if (left_mtr_ctr_pwm < 0) {
     digitalWrite(CH_LEFTMOTR_OUT_A, LOW);
     digitalWrite(CH_LEFTMOTR_OUT_B, HIGH);
   } else if (left_mtr_ctr_pwm == 0) {
     digitalWrite(CH_LEFTMOTR_OUT_A, LOW);
     digitalWrite(CH_LEFTMOTR_OUT_B, LOW);
   }

  left_mtr_ctr_pwm = constrain(abs(left_mtr_ctr_pwm), 0 , 180);
  digitalWrite(CH_LEFTMOTR_OUT_EN, left_mtr_ctr_pwm);
  


  nh.spinOnce();

  delay(500);

}

void read_Ultrasonic_sensor_values() {

  digitalWrite(CH_Ultrasonic_trig, LOW);  
	delayMicroseconds(2);  
	digitalWrite(CH_Ultrasonic_trig, HIGH);  
	delayMicroseconds(10);  
	digitalWrite(CH_Ultrasonic_trig, LOW);

  duration = pulseIn(CH_Ultrasonic_echo, HIGH);

  obstacle_distance = (duration*.0343)/2;

}

void read_IR_sensor_values()

{

  IR_sensors[0] = !digitalRead(CH_IRSENSOR_LEFT);
  IR_sensors[1] = !digitalRead(CH_IRSENSOR_CENTER);
  IR_sensors[2] = !digitalRead(CH_IRSENSOR_RIGHT);
 

  if ((IR_sensors[0] == 1) && (IR_sensors[1] == 0) && (IR_sensors[2] == 0))
    error = 100;
  else if ((IR_sensors[0] == 0) && (IR_sensors[1] == 1) && (IR_sensors[2] == 0))
    error = 0;
  else if ((IR_sensors[0] == 0) && (IR_sensors[1] == 0) && (IR_sensors[2] == 1))
    error = -100;
  else if ((IR_sensors[0] == 1) && (IR_sensors[1] == 1) && (IR_sensors[3] == 0)) // Turn robot left side
    error = 50;
  else if ((IR_sensors[0] == 0) && (IR_sensors[1] == 1) && (IR_sensors[2] == 1)) // Turn robot right side
    error = -50;
  else if ((IR_sensors[0] == 0) && (IR_sensors[1] == 0) && (IR_sensors[2] == 0)) // Make U turn
    error = 200;
  else if ((IR_sensors[0] == 1) && (IR_sensors[1] == 1) && (IR_sensors[2] == 1)) // Turn left side or stop
    error = 300;
}









