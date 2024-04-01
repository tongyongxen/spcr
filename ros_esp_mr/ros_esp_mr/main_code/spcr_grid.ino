#define ROSSERIAL_ARDUINO_TCP
#include "WiFi.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <ESP32Encoder.h>

#define IN1 25 // Left wheel
#define IN2 33
#define PWM_L_PIN 26
#define IN3 2 // Right Wheel
#define IN4 4
#define PWM_R_PIN 15 
#define B_IN1 22 // Brush 
#define B_IN2 23
#define Brush_PWM 19 

#define C1_L 13 // Left encoder
#define C2_L 14 
#define C1_R 18 // Right encoder
#define C2_R 5

#define IR_SENSOR_PIN 34 // Infrared sensor

float left_wheel;
float right_wheel;

ESP32Encoder encoderLeft;
ESP32Encoder encoderRight;

bool brush_motor_state = false; // Variable to hold brush motor state

IPAddress server(172, 20, 10, 2);
uint16_t serverPort = 11411;
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

ros::NodeHandle nh; // Create ROS node
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &cmdVel_to_pwm_cb);
ros::Subscriber<std_msgs::Empty> brush_sub("/brush_cmd", &brush_cmd_cb);
std_msgs::Int16 encoderLeftMsg;
ros::Publisher encoderLeftPub("enc_l_data", &encoderLeftMsg);
std_msgs::Int16 encoderRightMsg;
ros::Publisher encoderRightPub("enc_r_data", &encoderRightMsg);
std_msgs::Bool ir_sensorMsg;
ros::Publisher ir_sensorPub("ir_sensor_status", &ir_sensorMsg);

void setup() {
  Serial.begin(115200);
  setupWiFi();
  nh.getHardware()->setConnection(server, serverPort);

  pinMode(IN3, OUTPUT); // Left
  pinMode(IN4, OUTPUT);
  pinMode(PWM_L_PIN, OUTPUT);
  pinMode(IN1, OUTPUT); // Right
  pinMode(IN2, OUTPUT);
  pinMode(PWM_R_PIN, OUTPUT);

  pinMode(IR_SENSOR_PIN, INPUT); // IR sensor

  pinMode(B_IN1, OUTPUT); // Brush
  pinMode(B_IN2, OUTPUT);
  pinMode(Brush_PWM, OUTPUT);

  pinMode(C1_L, INPUT_PULLUP); 
  pinMode(C2_L, INPUT_PULLUP);
  pinMode(C1_R, INPUT_PULLUP);
  pinMode(C2_R, INPUT_PULLUP);

  encoderLeft.attachHalfQuad(C2_L, C1_L);
  encoderRight.attachHalfQuad(C1_R, C2_R);

  ledcSetup(0, 5000, 8);
  ledcAttachPin(PWM_L_PIN, 0);

  ledcSetup(1, 5000, 8);
  ledcAttachPin(PWM_R_PIN, 1);

  ledcSetup(2, 5000, 8);
  ledcAttachPin(Brush_PWM, 2);

  Stop();
  Brush_ON();

  nh.initNode();
  nh.advertise(encoderLeftPub);
  nh.advertise(encoderRightPub);
  nh.advertise(ir_sensorPub);
  nh.subscribe(brush_sub);
  nh.subscribe(sub);
}

void loop() {
  long encoderLeftValue = encoderLeft.getCount();
  long encoderRightValue = encoderRight.getCount();

  encoderLeftMsg.data = encoderLeftValue;
  encoderLeftPub.publish(&encoderLeftMsg);

  encoderRightMsg.data = encoderRightValue;
  encoderRightPub.publish(&encoderRightMsg);

  bool irSensorStatus = digitalRead(IR_SENSOR_PIN) == LOW;
  ir_sensorMsg.data = irSensorStatus;
  ir_sensorPub.publish(&ir_sensorMsg);

  nh.spinOnce();
  delay(300);
}

void setupWiFi() {  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  Serial.print("IP:   ");
  Serial.println(WiFi.localIP());
}

void cmdVel_to_pwm_cb(const geometry_msgs::Twist& velocity_msg) {
  float target_x = 1.0;
  float target_y = 1.0;
  float grid_spacing_mm = 900.0 / 10.0; // Assuming 10x10 grid for a 900mm x 900mm panel

  for (int i = 0; i < 10; i++) {
    for (int j = 0; j < 10; j++) {
      target_x = 1.0 + i * grid_spacing_mm + (grid_spacing_mm / 2);
      target_y = 1.0 + j * grid_spacing_mm + (grid_spacing_mm / 2);

      go_to_goal(target_x, target_y);
      delay(1000); // Adjust delay duration as needed
    }
  }
}

void direction() {
  digitalWrite(IN1, left_wheel > 0);
  digitalWrite(IN2, left_wheel < 0);
  digitalWrite(IN3, right_wheel > 0);
  digitalWrite(IN4, right_wheel < 0);
}

void speed() {
  ledcWrite(0, 200);  
  ledcWrite(1, 200);
}

void Stop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  ledcWrite(0, 0);
  ledcWrite(1, 0);
}

void brush_cmd_cb(const std_msgs::Empty&) {
  brush_motor_state = !brush_motor_state;
  if (brush_motor_state) {
    digitalWrite(B_IN1, HIGH);
    digitalWrite(B_IN2, LOW);
    ledcWrite(2, 200);
 
