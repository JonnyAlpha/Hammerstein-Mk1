// Program to set an Arduino as a slave for an i2c connection with a Raspberry Pi
// Controlling a mobile platform (robot) with manual and autonomous modes  
// Connections are Arduino Pin A5 to RPi SDA Arduino Pin A6 to RPi SCL
// Inspiration for i2c from Oscar Laing's Blog http://blog.oscarliang.net/raspberry-pi-arduino-connected-i2c

//Setup libraries
#include <Wire.h>
#include <Servo.h>

//Set Arduino as i2c slave and define the Serial bus address
#define SLAVE_ADDRESS 0x04

//Set variables for serial communication
int number = 0; // Number determines which command to run 
int state = 0; // Was used to check High and Low (not yet used)

//Setup Ultrasonic Sensor Pins
#define trigPin 13
#define echoPin 12

//Defines for reading distances and collision avoidance when in autonomous mode (when setup) 
#define microsecondsToCentimeters(microseconds) (unsigned long)microseconds / 29.1 / 2.0
#define Min_Action_Distance 40 // Set maximum allowaable distance to obstacle

//Time delays for motors and servos if required
#define ServoMoveDelay 2000

//Setup motor controller pins for L298N
//Right Motor
int enA = 11;
int in1 = 8;
int in2 = 7;

//Left Motor
int enB = 10;
int in3 = 6;
int in4 = 5;

//Setup LED test pin
int Led = 3;

//Give servos a logical designation
Servo sensor_servo;

//Setup variables
byte scan_pos = 0;
byte sweep_pos = 0;
byte pos_index = 90;
unsigned long left_distance, right_distance, forward_distance;
unsigned int duration;
unsigned int distance;
//unsigned int FrontDistance;
//unsigned int LeftDistance;
//unsigned int RightDistance;
unsigned long lastDataReceivedMillis; 
unsigned long maxGap = 10000;
unsigned int mode;

void setup() {
  delay(1000); // Sets a short delay after switching on
  Serial.begin(9600); // Start serial for output debugging
  Wire.begin(SLAVE_ADDRESS); // Initialise i2c as slave
  Wire.onReceive(receiveData); //Define callbacks for i2c communication
  Wire.onRequest(sendData); //Define callbacks for i2c communication
  
  //Setup the servo
  sensor_servo.attach(2); //Attaches the servo to pin 2 of the Nano
  sensor_servo.write(90); //Set the servo to face front
  scan_pos = 90;
  
  //Below set all the pin modes as OUTPUT as they will all be outputting data
  //Set modes for distance sensor pins 
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  //Set modes for motor controller pins
  pinMode(enA, OUTPUT);   
  pinMode(enB, OUTPUT);   
  pinMode(in1, OUTPUT);   
  pinMode(in2, OUTPUT);   
  pinMode(in3, OUTPUT);   
  pinMode(in4, OUTPUT);

  //Set mode for Led test pin
  pinMode(Led, OUTPUT);

  // Message to serial monitor declaring that robot is ready
  Serial.println("Hammerstein Ready!");
  mode = 0;
}

void loop() { 
  ping();
  //scan(); 
  //get_battery(); 
  while(true)
  {
    if (Wire.available())
    {
    mode = 0;
    Serial.println("Manual Mode");  
    continue;
    }
    else if(millis() - lastDataReceivedMillis >= maxGap)
    {
      if (mode == 0) Serial.println("Switching to autonomous mode in 5 seconds");
      delay(5000);
      mode = 1;
      AutonomousMode();
    }
   
  }
}


// Callback for received data
void receiveData(int byteCount) {
  
  while(Wire.available()) {
    number = Wire.read();
    Serial.println("data received: ");
    Serial.println(number);
    
    if (number == 0) {
      Stop();
    } 
    if (number == 1) {
      Forward();
    }
    if (number == 2) {
      Left();
    }
    if (number == 3) {
      Right();
    }
    if (number == 4) {
      Backward();
    }
    if (number == 5) {
      Servo_Centre();
    }
    if (number == 6) {
      Servo_Left();
    }
    if (number == 7) {
      Servo_Right();
    }
    if (number == 8) {
      led_on();
    }
    if (number == 9) {
      led_off();
    }
    //if (number == 10) { //Need to create a function in the python code to send command 10
    //  Auto_navigate();
    //}
  }
}

// Callback for sending data
void sendData() {
  Wire.write(number);
}

void AutonomousMode() //08/09/2015 - autonomous mode set to detect movement 
{
  Serial.println("Now in autonomous mode");
  unsigned long motion_detect_left, motion_detect_right, motion_detect_forward;
  if (mode == 1) //Only execute if in autonomous mode
  {
    Scan();
    motion_detect_left = left_distance;
    motion_detect_right = right_distance;
    motion_detect_forward = forward_distance;
    Scan();
    if (motion_detect_left == left_distance && motion_detect_right == right_distance && motion_detect_forward == forward_distance)
    {
      Stop(); 
    }
    else
    {
      Serial.println("Obstacle detected");
      number = 1;
      Auto_navigate();
      sendData(number); //Send content of variable 'number' to inform the Raspberry Pi that an obstacle has been detected and to say something
    }
  }
}

void Auto_navigate()
{
  do
  {
    Serial.println("Auto navigating environment");
    Scan();
    if (forward_distance > Min_Action_Distance)
    {
      Forward(); 
    }
    else
    {
      Stop();
      if (left_distance > right_distance && left_distance > Min_Action_Distance)
      {
        Left();
        Forward();
      }
      else
      {
        Right();
        if (right_distance > Min_Action_Distance)
        {
          Forward();
        }
      }
    }
   } while (mode == 1);
}

 
void get_battery()
//{
//  int BatteryValue = analogRead(0);
//  float voltage = BatteryValue * (7.2 / 1023.0);
//  Serial.println(voltage);
//}

{
  int raw = analogRead(0);
  float val = fmap(raw, 0, 1023, 0.00, 7.20);
  Serial.println(val);
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void Scan() //Print routines included for debugging 
{
  //Position sensor servo to the right 
  if (scan_pos != 20) 
  { //Only move the servo if it is not already in position
    sensor_servo.write (20);
  }
  delay (ServoMoveDelay);
  Serial.println("Sensor at 20 degrees..."); 
  ping(); //Read the sensor
  left_distance = distance;
  Serial.println("Clear right for  "); 
  Serial.print(left_distance); 
  Serial.println(" cm"); 
  
  //Position sensor servo to the front
  if (scan_pos != 90) //Only move the servo if it is not already in position
  { 
    sensor_servo.write(90);
  }
  delay (ServoMoveDelay);
  Serial.println("Servo at 90 degrees front...");
  ping(); //Read the sensor
  forward_distance = distance;
  Serial.println("Clear forward for  "); 
  Serial.print(forward_distance); 
  Serial.println(" cm"); 
  
  //Position sensor servo to the left 
  if (scan_pos != 160) //Only move the servo if it is not already in position
  { 
    sensor_servo.write (160);
  }
  delay (ServoMoveDelay);
  Serial.println("Sensor at 160 degrees...");
  ping(); //Read the sensor
  Serial.println("Clear left for  ");
  Serial.print(left_distance);  
  Serial.println(" cm"); 

  if (scan_pos != 90) //Only move the servo if it is not already in position
  { 
    sensor_servo.write(90);
  }
}

void Servo_Sweep() //This function tells the robot to scan for obstacles - redundant for time being  
{
  if (sweep_pos <=0)
  {
    pos_index = 20;
  }
  else if (sweep_pos >=180) 
  {
    pos_index = -20;
  }
  Serial.print ("pos_index = ");
  Serial.println(pos_index);
  sweep_pos += pos_index;
  Serial.print("sweep_pos = ");
  Serial.println(sweep_pos);
  sensor_servo.write (sweep_pos);
}

void Servo_Centre() //Manual control of sensor servo 
{
  Serial.println("");
  Serial.println("Scanning ahead");
  if (sweep_pos != 90)
  {
    sensor_servo.write(90);
    sweep_pos = 90;
    ping();
    //distance = ping();
    Serial.println("Distance ahead = ");
    Serial.print(distance);
    Serial.print(" cm");
  }
}

void Servo_Right() //Manual control of sensor servo 
{
  Serial.println("");
  Serial.println("Scanning right");
  if (sweep_pos != 20) 
  {
    sensor_servo.write(20);
    sweep_pos = 20;
    ping();
    //distance = ping();
    Serial.println("Distance right = ");
    Serial.print(distance);
    Serial.print(" cm"); 
  }
}

void Servo_Left() //Manual control of sensor servo 
{
  Serial.println("");
  Serial.println("Scanning Left");
  if (sweep_pos != 160) 
  {
    sensor_servo.write(160);
    sweep_pos = 160;
    ping();
    //distance = ping();
    Serial.println("Distance left = ");
    Serial.print(distance);
    Serial.print(" cm");
  }
}

void led_on() //Turn the LED on 
{
  Serial.println("");
  Serial.println("Led On");
  digitalWrite(Led, HIGH); 
}

void led_off() //Turn the LED off
{
  Serial.println("");
  Serial.println("Led Off");
  digitalWrite(Led, LOW);
}


unsigned long ping() //Read the HC-SR04 sensor
{
  //Trigger the sensor to send out a ping
 digitalWrite(trigPin, HIGH);
 delayMicroseconds(1000);
 digitalWrite(trigPin, LOW);
 duration = pulseIn(echoPin, HIGH);
 distance = (duration/2) /29.1;
 //Serial.print(distance);
 //Serial.println(" cm");
 //delay(500);
 //distance = ping();
}

void Forward() //This function tells the robot to go forward 
{
  Serial.println("");
  Serial.println("Moving forward");
  // turn on left motor   
  digitalWrite(in1, HIGH);   
  digitalWrite(in2, LOW);   
  // set speed out of possible range 0~255
  analogWrite(enA, 255);   
  // turn on right motor   
  digitalWrite(in3, LOW);   
  digitalWrite(in4, HIGH);   
  // set speed out of possible range 0~255   
  analogWrite(enB, 255);   
  //delay(100);
}

void Backward() //This function tells the robot to move backward
{
  Serial.println("");
  Serial.println("Moving backward");
  // turn on left motor   
  digitalWrite(in1, LOW);   
  digitalWrite(in2, HIGH);   
  // set speed out of possible range 0~255
  analogWrite(enA, 255);   
  // turn on right motor   
  digitalWrite(in3, HIGH);   
  digitalWrite(in4, LOW);   
  // set speed out of possible range 0~255   
  analogWrite(enB, 255);   
  //delay(100);
}

void Left() //This function tells the robot to turn left
{
  Serial.println("");
  Serial.println("Moving left");
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH); 
// set speed out of possible range 0~255
  analogWrite(enA, 255); 
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
// set speed out of possible range 0~255   
  analogWrite(enB, 255);   
  //delay(100);  
}

void Right() //This function tells the robot to turn right
{
  Serial.println("");
  Serial.println("Moving right");
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
// set speed out of possible range 0~255
  analogWrite(enA, 255); 
  digitalWrite(in3, HIGH); //Was High
  digitalWrite(in4, LOW);
  analogWrite(enB, 255);
  //delay(100);
}

void Stop() //This function tells the robot to stop moving
{
  Serial.println("");
  Serial.println("Stopping");
// now turn off motors   
  digitalWrite(in1, LOW);   
  digitalWrite(in2, LOW);
  //analogWrite(enA, 0);  
  digitalWrite(in3, LOW);   
  digitalWrite(in4, LOW);
  //analogWrite(enB, 0);
}
