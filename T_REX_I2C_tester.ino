
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt16MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <Wire.h>

// PID Library used: AutoPID by <Ryan Downing>
#include <AutoPID.h>


#define BUTTON 8
#define LED 13

#define startbyte 0x0F
#define I2Caddress 0x07

int sv[6]={1500,1500,1500,1500,0,0};                 // servo positions: 0 = Not Used
int sd[6]={5,10,-5,-15,20,-20};                      // servo sweep speed/direction
double lmspeed,rmspeed;                                 // left and right motor speed from -255 to +255 (negative value = reverse)
int ldir=5;                                          // how much to change left  motor speed each loop (use for motor testing)
int rdir=5;                                          // how much to change right motor speed each loop (use for motor testing)
byte lmbrake,rmbrake;                                // left and right motor brake (non zero value = brake)
byte devibrate=50;                                   // time delay after impact to prevent false re-triggering due to chassis vibration
int sensitivity=50;                                  // threshold of acceleration / deceleration required to register as an impact
int lowbat=550;                                      // adjust to suit your battery: 550 = 5.50V
byte i2caddr=7;                                      // default I2C address of T'REX is 7. If this is changed, the T'REX will automatically store new address in EEPROM
byte i2cfreq=0;                                      // I2C clock frequency. Default is 0=100kHz. Set to 1 for 400kHz

// Variables for encoder
int lmenc=0;
int rmenc=0;


//pid settings and gains
#define OUTPUT_MIN 0
#define OUTPUT_MAX 150
#define KP 5
#define KI 3
#define KD 2

double desiredRateLeftEncoder, desiredRateRightEncoder, rateLeftEncoder, rateRightEncoder; // output is missing from here as they have been defined above as lmspeed and rmspeed

//input/output variables passed by reference, so they are updated automatically
AutoPID leftPID(&rateLeftEncoder, &desiredRateLeftEncoder, &lmspeed, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);
AutoPID rightPID(&rateRightEncoder, &desiredRateRightEncoder, &rmspeed, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);



ros::NodeHandle node_handle;

std_msgs::UInt16 encoder_data_msg;
geometry_msgs::Twist cmd_vel_msg;

void subscriberCallback(const geometry_msgs::Twist& cmd_vel_msg) {
  // define all possible situations here

  if (cmd_vel_msg.angular.z == 0 && cmd_vel_msg.linear.x > 0) {
    desiredRateLeftEncoder = 100;
    desiredRateRightEncoder = 100;
  } else if (cmd_vel_msg.angular.z == 0 && cmd_vel_msg.linear.x == 0) {
    lmspeed = 0;
    rmspeed = 0;
    desiredRateLeftEncoder = 0;
    desiredRateRightEncoder = 0;
  } else if (cmd_vel_msg.angular.z != 0 && cmd_vel_msg.linear.x == 0) {
    lmspeed = 0;
    rmspeed = 0;
    if (cmd_vel_msg.angular.z > 0) {
      desiredRateLeftEncoder = 70;
      desiredRateRightEncoder = -70;
    } else {
      desiredRateLeftEncoder = -70;
      desiredRateRightEncoder = 70;
    }
  } else {
    lmspeed = 0;
    rmspeed = 0;
    desiredRateLeftEncoder = 0;
    desiredRateRightEncoder = 0;
    node_handle.logerror("Stopping the robot.");
  }
}

ros::Publisher encoder_rate_publisher("encoder_rate", &encoder_data_msg);
ros::Subscriber<geometry_msgs::Twist> cmd_vel_subscriber("/cmd_vel", &subscriberCallback);


void setup()
{ 
  node_handle.getHardware()->setBaud(57600);
  node_handle.initNode();
  node_handle.advertise(encoder_rate_publisher);
  node_handle.subscribe(cmd_vel_subscriber);
  Wire.begin();

  // sets to minimum or maximum output when the deviation in desired output is less than or greater than the specified value below
  leftPID.setBangBang(40);
  rightPID.setBangBang(40);
  // tune output every 0.5s
  leftPID.setTimeStep(500);
  rightPID.setTimeStep(500);
}


void loop()
{
 
  
  MasterSend(startbyte,2,lmspeed,lmbrake,rmspeed,rmbrake,sv[0],sv[1],sv[2],sv[3],sv[4],sv[5],devibrate,sensitivity,lowbat,i2caddr,i2cfreq);
  delay(50);

  double oldlmenc = lmenc;
  double oldrmenc = rmenc;

  
  MasterReceive();                                   // receive data packet from T'REX controller
  delay(50);


  // Calculates the encoder rate of change. Think about if we really need to divide the rate variable by time. I dont think we do.
  rateRightEncoder = rmenc - oldrmenc;
  rateLeftEncoder = lmenc - oldlmenc;
//  String abc = "hello";
//  node_handle.loginfo("Right encoder rate " + toCharArray(String(rateRightEncoder)));
//  node_handle.loginfo(rateRightEncoder);
//  node_handle.loginfo(Prepend("abc", String(rateRightEncoder)));
//  node_handle.loginfo(rateLeftEncoder);
  
  // PID will tune output.
  leftPID.run();
  rightPID.run();
  
  encoder_data_msg.data = rateRightEncoder;
  encoder_rate_publisher.publish( &encoder_data_msg );
  node_handle.spinOnce();
  delay(1);

}
