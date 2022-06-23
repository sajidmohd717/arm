/*
 * rosserial Publisher Example
 * Prints "hello world!"
 * This intend to connect to an Arduino Ethernet Shield
 * and a rosserial socket server.
 * You can launch the rosserial socket server with
 * roslaunch rosserial_server socket.launch
 * The default port is 11411
 *
 */
#include <SPI.h>
#include <Ethernet.h>
#include <WiFi.h>
#define SENSORPIN 7
 
// To use the TCP version of rosserial_arduino
#define ROSSERIAL_ARDUINO_TCP


int sensorState = 2;         // variable for reading the pushbutton status

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <avr/dtostrf.h>


// Shield settings
byte mac[] = { 0xA8, 0x61, 0x0A, 0xAE, 0x11, 0x7B };//
IPAddress ip(192, 168, 1, 131);//

// Server settings
IPAddress server(192,168,1,6);
// Set the rosserial socket server port
const uint16_t serverPort = 11411;

ros::NodeHandle nh;
const int IN1 = 2;
const int IN2 = 3;
const int PWM = 1;
const int SensorDOWN = 6; // input pin for Photomicrosensor
const int SensorUP = 4;
int move_state = 0;
int position_state = 0;
int dock;
int UP, DOWN; // a variable to read the encoder state
float var;
char result[32];
char log_msg[32];
char state_char[8];
uint16_t period = 1000;
uint32_t last_time = 0;
short int arm_status_value[3] = {};

// Make a arm_moving_state publisher
std_msgs::Int16MultiArray arm_status_msg;
ros::Publisher arm_status("arm_status", &arm_status_msg);
char dim0_label[] = "arm_status";

void messageDOWN(const std_msgs::Int32 &msg) {
  rosinfo(msg);
  check_sensors();
  while (DOWN == 1) { // move until front sensor triggered
    Serial.print("SensorDOWN: ");
    Serial.println(DOWN);
    delay(50);
    Motor_Backward(200);
    check_sensors();

     //print state on ros
    dtostrf(move_state, 6, 2, state_char);
    nh.loginfo(state_char);
    state_char[0]= '\0';
    
    //publish to rostopic
    move_state = 1; // moving down
    arm_status_value[0] = move_state;
    arm_status_value[1] = position_state;
    arm_status_value[2] = sensorState;
    arm_status_msg.data = arm_status_value;
    arm_status.publish(&arm_status_msg);
    nh.spinOnce();
  }
  messageBR(msg);
}

void messageUP(const std_msgs::Int32 &msg) {
  rosinfo(msg);
  check_sensors();
  
  while (UP == 1) { // move until front sensor triggered
    Serial.print("up");
    Serial.print("SensorUP: ");
    Serial.println(UP);    
    delay(50);
    Motor_Forward(200);
    check_sensors();
    
    //print state on ros
    dtostrf(move_state, 6, 2, state_char);
    nh.loginfo(state_char);
    state_char[0]= '\0';

    //publish to rostopic
    move_state = 2; // moving down
    arm_status_value[0] = move_state;
    arm_status_value[1] = position_state;
    arm_status_value[2] = sensorState;
    arm_status_msg.data = arm_status_value;
    arm_status.publish(&arm_status_msg);
    nh.spinOnce();
  }
  messageBR(msg);
}

void messageBR(const std_msgs::Int32 &msg) {
  rosinfo(msg);
  check_sensors();
  move_state = 0;

  while (move_state == 0) { // move until front sensor triggered
    Motor_Brake();
    //print state on ros
    dtostrf(move_state, 6, 2, state_char);
    nh.loginfo(state_char);
    state_char[0]= '\0';
    arm_status_value[0] = move_state;
    arm_status_value[1] = position_state;
    arm_status_value[2] = sensorState;
    arm_status_msg.data = arm_status_value;
    arm_status.publish(&arm_status_msg);
    nh.spinOnce();
  }
}

void rosinfo(const std_msgs::Int32 &msg){
  var=msg.data;
  dtostrf(var, 6, 2, result);
  sprintf(log_msg,"Message Published =%s", result);
  nh.loginfo(log_msg);
  log_msg[0]='\0';
  result[0]='\0';
}

void check_sensors(){
  UP = digitalRead(SensorUP);
  DOWN = digitalRead(SensorDOWN);
  if (UP == 1 && DOWN ==1){
    position_state = 1; // in between
    sensorState = 2;
  }
  else if (UP == 1 && DOWN == 0){
    position_state = 0; // arm at bottom position
    sensorState = 2;
  }
  else if (UP == 0 && DOWN == 1){
    position_state = 2; // arm at top position
    sensorState = digitalRead(SENSORPIN); //1 if not broken (not docked), 0 if broken (docked)
    arm_status_value[2] = sensorState;
  }
  else{
    position_state = -1; //unknown position
  }
}

ros::Subscriber<std_msgs::Int32> down("arm_down", &messageDOWN );
ros::Subscriber<std_msgs::Int32> up("arm_up", &messageUP );
ros::Subscriber<std_msgs::Int32> brake("arm_brake", &messageBR );


void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(SensorDOWN, INPUT); // set pin 2 as input
  pinMode(SensorUP, INPUT); // set pin 2 as input
  
  // Use serial to monitor the process
  Serial.begin(115200);

  // Connect the Ethernet
  Ethernet.begin(mac, ip);

  
  // Let some time for the Ethernet Shield to be initialized
  delay(1000);

  Serial.println("");
  Serial.println("Ethernet connected");
  Serial.println("IP address: ");
  Serial.println(Ethernet.localIP());

  // Set the connection to rosserial socket server
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
    // Start to be polite
  arm_status_msg.data_length = 3;
  nh.advertise(arm_status);
  nh.subscribe(down);
  nh.subscribe(up);
  nh.subscribe(brake);
  
  // Another way to get IP
  Serial.print("IP = ");
  Serial.println(nh.getHardware()->getLocalIP());


}

void loop()
{
  if(millis() - last_time >= period)
  {
    last_time = millis();
    if (nh.connected())
    {
      Serial.println("Connected");
    check_sensors();
    arm_status_value[0] = move_state;
    arm_status_value[1] = position_state;
    arm_status_value[2] = {};
    arm_status_msg.data = arm_status_value;
    arm_status.publish(&arm_status_msg); // publishing to arm_server
    } else {
      Serial.println("Not Connected");
    }
  }

  nh.spinOnce();
  delay(1);
}

void Motor_Forward(int Speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(PWM, Speed);
}

void Motor_Backward(int Speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(PWM, Speed);
}

void Motor_Brake() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}
