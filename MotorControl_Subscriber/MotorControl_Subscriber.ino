#include <ros.h>
#include <motor_control/MotorCommand.h>

// Motor pin definitions
const int enaAPin = 9;  // Enable pin for motor A
const int in1APin = 10; // Input pin 1 for motor A
const int in2APin = 11; // Input pin 2 for motor A
const int in1BPin = 12; // Input pin 1 for motor B
const int in2BPin = 13; // Input pin 2 for motor B

ros::NodeHandle nh;

void motorCommandCallback(const motor_control::MotorCommand& msg)
{
  // Extract the values from the motor command message
  int enaAValue = msg.speed;
  int directionB = msg.steer;

  // Set the PWM value for motor A
  analogWrite(enaAPin, enaAValue);

  // Set the direction of rotation for motor B
  if (directionB > 0)
  {
    digitalWrite(in1BPin, HIGH);
    digitalWrite(in2BPin, LOW);
  }
  else if (directionB < 0)
  {
    digitalWrite(in1BPin, LOW);
    digitalWrite(in2BPin, HIGH);
  }
}

ros::Subscriber<motor_control::MotorCommand> motorCommandSub("motor_command", motorCommandCallback);

void setup()
{
  // Initialize the motor control pins as outputs
  pinMode(enaAPin, OUTPUT);
  pinMode(in1APin, OUTPUT);
  pinMode(in2APin, OUTPUT);
  pinMode(in1BPin, OUTPUT);
  pinMode(in2BPin, OUTPUT);

  // Initialize the ROS node
  nh.initNode();
  
  // Subscribe to the motor_command topic
  nh.subscribe(motorCommandSub);

  // Wait for the node to be ready
  while (!nh.connected())
  {
    nh.spinOnce();
  }
}

void loop()
{
  // Process any incoming ROS messages
  nh.spinOnce();
}
