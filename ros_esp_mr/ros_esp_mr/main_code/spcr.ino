#define ROSSERIAL_ARDUINO_TCP
#include "WiFi.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>
#include <ESP32Encoder.h>

#define IN1 25 // Left wheel
#define IN2 33
#define PWM_L_PIN 26
#define IN3 2 //Right Wheel
#define IN4 4
#define PWM_R_PIN 15 

#define C1_L 13 //Left encoder
#define C2_L 14 
#define C1_R 18 //Right encoder
#define C2_R 5

float left_wheel;
float right_wheel;

ESP32Encoder encoderLeft;
ESP32Encoder encoderRight;

IPAddress server(172, 20, 10, 2);
uint16_t serverPort = 11411;
const char*  ssid = "iPhone B";
const char*  password = "123123123";

void cmdVel_to_pwm_cb( const geometry_msgs::Twist& velocity_msg);
ros::NodeHandle  nh; //Create ros node
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &cmdVel_to_pwm_cb );

std_msgs::Int16 encoderLeftMsg;
ros::Publisher encoderLeftPub("enc_l_data",&encoderLeftMsg);

std_msgs::Int16 encoderRightMsg;
ros::Publisher encoderRightPub("enc_r_data",&encoderRightMsg);


void setup()
{
 Serial.begin(115200);
 setupWiFi();
 nh.getHardware()->setConnection(server, serverPort);
//Set motor pin as output
 pinMode(IN3,OUTPUT); //Left
 pinMode(IN4,OUTPUT);
 pinMode(PWM_L_PIN,OUTPUT);
 pinMode(IN1,OUTPUT); //Right
 pinMode(IN2,OUTPUT);
 pinMode(PWM_R_PIN,OUTPUT);

 //Encoder
 pinMode(C1_L,INPUT_PULLUP); 
 pinMode(C2_L,INPUT_PULLUP);
 pinMode(C1_R,INPUT_PULLUP);
 pinMode(C2_R,INPUT_PULLUP);

 encoderLeft.attachHalfQuad(C2_L,C1_L); //Set as left encoder
 encoderRight.attachHalfQuad(C1_R,C2_R); //Set as right encoder


 ledcSetup(0,5000,8); //(pwm_channel,freq,resolution)
 ledcAttachPin(PWM_L_PIN,0);//set channel to EN

 ledcSetup(1,5000,8); // Set channel 1
 ledcAttachPin(PWM_R_PIN,1);

 Stop(); //Initialize motor in stop condition

 nh.initNode(); //Initialize the node
 nh.advertise(encoderLeftPub); //Publish the data
 nh.advertise(encoderRightPub);
 nh.subscribe(sub);
}

void loop()
{
 long encoderLeftValue= encoderLeft.getCount(); //Lib to count++
 long encoderRightValue= encoderRight.getCount();

 encoderLeftMsg.data = encoderLeftValue;
 encoderLeftPub.publish(&encoderLeftMsg);

 encoderRightMsg.data=encoderRightValue;
 encoderRightPub.publish(&encoderRightMsg);

 nh.spinOnce();//Run the node once only
 delay(300);
}

void setupWiFi()
{  
   WiFi.begin(ssid, password);
   while (WiFi.status() != WL_CONNECTED) { delay(500);Serial.print("."); }
   Serial.print("SSID: ");
   Serial.println(WiFi.SSID());
   Serial.print("IP:   ");
   Serial.println(WiFi.localIP());

}

void cmdVel_to_pwm_cb( const geometry_msgs::Twist& velocity_msg){

    right_wheel = (velocity_msg.linear.x + velocity_msg.angular.z ) / 2 ;
    left_wheel = (velocity_msg.linear.x - velocity_msg.angular.z ) /2 ;
    direction();
    speed();
    if ( velocity_msg.linear.x ==0.0 & velocity_msg.angular.z ==0.0){
        Stop();
    }
    Serial.print(left_wheel);Serial.print(" / ");Serial.println(right_wheel);

}

void direction(){
    digitalWrite(IN1, left_wheel >0 );
    digitalWrite(IN2,left_wheel < 0);
    digitalWrite(IN3,right_wheel > 0 );
    digitalWrite(IN4,right_wheel < 0);
}

void speed (){
    ledcWrite(0, 200);  
    ledcWrite(1, 200);
}

void Stop()
{
  digitalWrite(IN1,LOW); //Set as 0
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,LOW);

  ledcWrite(0,0);//(Channel,PWM)
  ledcWrite(1,0);
}