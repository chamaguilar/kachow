#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <AFMotor.h>

ros::NodeHandle nh;

#define ENC_IN_LEFT_A 18
#define ENC_IN_RIGHT_A 19
#define ENC_IN_LEFT_B 22
#define ENC_IN_RIGHT_B 23

boolean Direction_left = true;
boolean Direction_right = true;
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;
 
std_msgs::Int16 right_wheel_tick_count;
ros::Publisher rightPub("right_ticks", &right_wheel_tick_count);
std_msgs::Int16 left_wheel_tick_count;
ros::Publisher leftPub("left_ticks", &left_wheel_tick_count);
 
const int interval = 30;
long previousMillis = 0;
long currentMillis = 0;

#define leftTrig 24
#define leftEcho 25
#define centerTrig 26
#define centerEcho 27
#define rightTrig 28
#define rightEcho 29
static float duration, distance;
const int sonar_timeout = 5000;

float angleMin;
float angleMax;
float angleIncrement;
float timeIncrement;
float rangeMin;
float rangeMax;
float valRanges[3];

sensor_msgs::LaserScan scan;
ros::Publisher sonarPub("scan", &scan);

AF_DCMotor tlMotor(1); // top left, M1
AF_DCMotor trMotor(4); // top right, M4
AF_DCMotor blMotor(2); // bottom left, M2
AF_DCMotor brMotor(3); // bottom right, M3

const int PWM_INCREMENT = 1;
const int TICKS_PER_REVOLUTION = 3000;
const double WHEEL_RADIUS = 0.0575;
const double WHEEL_BASE = 0.285;
const double TICKS_PER_METER = 8000;

const int K_P = 231;
const int b = 62;

const int DRIFT_MULTIPLIER = 120;
const int PWM_TURN = 240;
const int PWM_MIN = 100;
const int PWM_MAX = 240;

double velLeftWheel = 0;
double velRightWheel = 0;
double pwmLeftReq = 0;
double pwmRightReq = 0;

double lastCmdVelReceived = 0;

void right_wheel_tick() {
  int val = digitalRead(ENC_IN_RIGHT_B); 
  if (val == LOW) {
    Direction_right = true; // Reverse
  }
  else {
    Direction_right = false; // Forward
  }
  if (Direction_right) {  
    if (right_wheel_tick_count.data == encoder_maximum) {
      right_wheel_tick_count.data = encoder_minimum;
    }
    else {
      right_wheel_tick_count.data++;  
    }    
  }
  else {
    if (right_wheel_tick_count.data == encoder_minimum) {
      right_wheel_tick_count.data = encoder_maximum;
    }
    else {
      right_wheel_tick_count.data--;  
    }
  }
}

void left_wheel_tick() {
  int val = digitalRead(ENC_IN_LEFT_B);
  if (val == LOW) {
    Direction_left = false; // Reverse
  }
  else {
    Direction_left = true; // Forward
  }
  if (Direction_left) {
    if (left_wheel_tick_count.data == encoder_maximum) {
      left_wheel_tick_count.data = encoder_minimum;
    }
    else {
      left_wheel_tick_count.data++;  
    }  
  }
  else {
    if (left_wheel_tick_count.data == encoder_minimum) {
      left_wheel_tick_count.data = encoder_maximum;
    }
    else {
      left_wheel_tick_count.data--;  
    }   
  }
}
 
void calc_vel_left_wheel(){
  static double prevTime = 0;
  static int prevLeftCount = 0;
  int numOfTicks = (65535 + left_wheel_tick_count.data - prevLeftCount) % 65535;
  if (numOfTicks > 10000) {
        numOfTicks = 0 - (65535 - numOfTicks);
  }
  velLeftWheel = numOfTicks/TICKS_PER_METER/((millis()/1000)-prevTime);
  prevLeftCount = left_wheel_tick_count.data;
  prevTime = (millis()/1000);
}

void calc_vel_right_wheel(){
  static double prevTime = 0;
  static int prevRightCount = 0;
  int numOfTicks = (65535 + right_wheel_tick_count.data - prevRightCount) % 65535;
  if (numOfTicks > 10000) {
        numOfTicks = 0 - (65535 - numOfTicks);
  }
  velRightWheel = numOfTicks/TICKS_PER_METER/((millis()/1000)-prevTime);
  prevRightCount = right_wheel_tick_count.data;
  prevTime = (millis()/1000); 
}

void calc_pwm_values(const geometry_msgs::Twist& cmdVel) {
  lastCmdVelReceived = (millis()/1000);
  pwmLeftReq = K_P * cmdVel.linear.x + b;
  pwmRightReq = K_P * cmdVel.linear.x + b;
 
  // Check if we need to turn 
  if (cmdVel.angular.z != 0.0) {
    // Turn left
    if (cmdVel.angular.z > 0.0) {
      pwmLeftReq = -PWM_TURN;
      pwmRightReq = PWM_TURN;
    }
    // Turn right    
    else {
      pwmLeftReq = PWM_TURN;
      pwmRightReq = -PWM_TURN;
    }
  }
  // Go straight
  else {
    static double prevDiff = 0;
    static double prevPrevDiff = 0;
    double currDifference = velLeftWheel - velRightWheel; 
    double avgDifference = (prevDiff+prevPrevDiff+currDifference)/3;
    prevPrevDiff = prevDiff;
    prevDiff = currDifference;

    pwmLeftReq -= (int)(avgDifference * DRIFT_MULTIPLIER);
    pwmRightReq += (int)(avgDifference * DRIFT_MULTIPLIER);
  }
  
  // Handle low PWM values
  if (abs(pwmLeftReq) < PWM_MIN) {
    pwmLeftReq = 0;
  }
  if (abs(pwmRightReq) < PWM_MIN) {
    pwmRightReq = 0;  
  }  
}

void set_pwm_values() {
  static int pwmLeftOut = 0;
  static int pwmRightOut = 0;
  static bool stopped = false;
  if ((pwmLeftReq * velLeftWheel < 0 && pwmLeftOut != 0) ||
      (pwmRightReq * velRightWheel < 0 && pwmRightOut != 0)) {
    pwmLeftReq = 0;
    pwmRightReq = 0;
  }
 
  // Set the direction of the motors
  if (pwmLeftReq > 0) { // Left wheel forward
    tlMotor.run(FORWARD);
    blMotor.run(FORWARD);
  }
  else if (pwmLeftReq < 0) { // Left wheel reverse
    tlMotor.run(BACKWARD);
    blMotor.run(BACKWARD);
  }
  else if (pwmLeftReq == 0 && pwmLeftOut == 0 ) { // Left wheel stop
    tlMotor.run(RELEASE);
    blMotor.run(RELEASE);
  }
  else { // Left wheel stop
    tlMotor.run(RELEASE);
    blMotor.run(RELEASE); 
  }
 
  if (pwmRightReq > 0) { // Right wheel forward
    trMotor.run(BACKWARD);
    brMotor.run(BACKWARD);
  }
  else if(pwmRightReq < 0) { // Right wheel reverse
    trMotor.run(FORWARD);
    brMotor.run(FORWARD);
  }
  else if (pwmRightReq == 0 && pwmRightOut == 0) { // Right wheel stop
    trMotor.run(RELEASE);
    brMotor.run(RELEASE);
  }
  else { // Right wheel stop
    trMotor.run(RELEASE);
    brMotor.run(RELEASE);
  }
 
  // Increase the required PWM if the robot is not moving
  if (pwmLeftReq != 0 && velLeftWheel == 0) {
    pwmLeftReq *= 1.5;
  }
  if (pwmRightReq != 0 && velRightWheel == 0) {
    pwmRightReq *= 1.5;
  }
 
  // Calculate the output PWM value by making slow changes to the current value
  if (abs(pwmLeftReq) > pwmLeftOut) {
    pwmLeftOut += PWM_INCREMENT;
  }
  else if (abs(pwmLeftReq) < pwmLeftOut) {
    pwmLeftOut -= PWM_INCREMENT;
  }
  else{}
   
  if (abs(pwmRightReq) > pwmRightOut) {
    pwmRightOut += PWM_INCREMENT;
  }
  else if(abs(pwmRightReq) < pwmRightOut) {
    pwmRightOut -= PWM_INCREMENT;
  }
  else{}
 
  // Conditional operator to limit PWM output at the maximum 
  pwmLeftOut = (pwmLeftOut > PWM_MAX) ? PWM_MAX : pwmLeftOut;
  pwmRightOut = (pwmRightOut > PWM_MAX) ? PWM_MAX : pwmRightOut;
 
  // PWM output cannot be less than 0
  pwmLeftOut = (pwmLeftOut < 0) ? 0 : pwmLeftOut;
  pwmRightOut = (pwmRightOut < 0) ? 0 : pwmRightOut;

  tlMotor.setSpeed(pwmLeftOut);
  blMotor.setSpeed(pwmLeftOut);
  trMotor.setSpeed(pwmRightOut);
  brMotor.setSpeed(pwmRightOut);
}
 
ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", &calc_pwm_values );
 
void setup() {
  pinMode(ENC_IN_LEFT_A , INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B , INPUT);
  pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B , INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_tick, RISING);

  pinMode(leftTrig, OUTPUT);
  pinMode(leftEcho, INPUT);
  pinMode(centerTrig, OUTPUT);
  pinMode(centerEcho, INPUT);
  pinMode(rightTrig, OUTPUT);
  pinMode(rightEcho, INPUT);
  
  tlMotor.setSpeed(0);
  trMotor.setSpeed(0);
  blMotor.setSpeed(0);
  brMotor.setSpeed(0);

  angleMin = -1.57;
  angleMax = 1.57;
  angleIncrement = 3.14 / 3;
  timeIncrement = 1 / 6;
  rangeMin = 0.03;
  rangeMax = 1.0;
  scan.ranges_length = 3;
  for (int i=0; i<=2; i++) {
    valRanges[i] = i;
  }

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(rightPub);
  nh.advertise(leftPub);
  nh.advertise(sonarPub);
  nh.subscribe(subCmdVel);
}
 
void loop() {
  nh.spinOnce();

  currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;

    sonarSensor(rightTrig, rightEcho);
    valRanges[0] = distance / 100.0;
    sonarSensor(centerTrig, centerEcho);
    valRanges[1] = distance / 100.0;
    sonarSensor(leftTrig, leftEcho);
    valRanges[2] = distance / 100.0;

    scan.header.stamp = nh.now();
    scan.header.frame_id = "ultrasound";
    scan.angle_min = angleMin;
    scan.angle_max = angleMax;
    scan.angle_increment = angleIncrement;
    scan.time_increment = timeIncrement;
    scan.range_min = rangeMin;
    scan.range_max = rangeMax;
    scan.ranges = valRanges;
    sonarPub.publish( &scan );
    
    leftPub.publish( &left_wheel_tick_count );
    rightPub.publish( &right_wheel_tick_count );

    calc_vel_right_wheel();
    calc_vel_left_wheel();
  }
  
  set_pwm_values();
}

void sonarSensor(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH, sonar_timeout);
  distance = 0.034 * duration / 2;
}
