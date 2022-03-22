
#include <SerialTransfer.h>
#include <Wire.h>


// PID Library used: AutoPID by <Ryan Downing>
#include <AutoPID.h>



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

SerialTransfer myTransfer;

struct STRUCT {
  double dre;
  double dle;
  bool pid_params;
  double kp_l;
  double ki_l;
  double kd_l;
  double kp_r;
  double ki_r;
  double kd_r;
} tuningPacket;

//pid settings and gains
#define OUTPUT_MIN 0
#define OUTPUT_MAX 180
#define KP_L 2
#define KI_L 1.3
#define KD_L 0.2
#define KP_R 2
#define KI_R 1.3
#define KD_R 0.1
bool sync = false;
bool resetIntegral = false;

double desiredRateRightEncoder, rateLeftEncoder, rateRightEncoder; // output is missing from here as they have been defined above as lmspeed and rmspeed
double desiredRateLeftEncoder;

//input/output variables passed by reference, so they are updated automatically
AutoPID leftPID(&rateLeftEncoder, &desiredRateLeftEncoder, &lmspeed, OUTPUT_MIN, OUTPUT_MAX, KP_L, KI_L, KD_L);
AutoPID rightPID(&rateRightEncoder, &desiredRateRightEncoder, &rmspeed, OUTPUT_MIN, OUTPUT_MAX, KP_R, KI_R, KD_R);


double clamp(double x, double a, double b) {
  if (x > a) {
    return a;
  } else if (x < b) {
    return b;
  } else {
    return x;
  }
}

void setup()
{ 
  Serial.begin(115200);
//  myTransfer.begin(Serial);
  desiredRateLeftEncoder = 0;
  desiredRateRightEncoder = 0;
  Wire.begin();

  // sets to minimum or maximum output when the deviation in desired output is less than or greater than the specified value below
  
  // tune output every 0.1s
  leftPID.setTimeStep(100);
  rightPID.setTimeStep(100);
}


void loop()
{

//  if(myTransfer.available())
//  {
////    desiredRateLeftEncoder = 30;
//    uint16_t recSize = 0;
////    recSize = myTransfer.rxObj(tuningPacket, recSize);
//    tune();
//  }

  if (Serial.available() > 0) {
    // recieved data packet
    int rcvdData[2];
    // ready to flash the buffer
    bool flushBuff = false;
    // what index it is
    int index = 0;
    // character that marks the end of 
    char endmarker = '\n';
    // delimator
    char delim = ',';
    // string read from the end of , to the next ,
    String interimString;
    // recieved character
    char rc;
    // current integer
    int currentInt;
    while (Serial.available() > 0) {
      rc = Serial.read();
      
      if ((rc == delim || rc == endmarker) && flushBuff == false) {
        rcvdData[index] = interimString.toInt();
        interimString = "";
        index += 1;
        if (rc == endmarker) {
          flushBuff = true;
        }
      }
      else {
        interimString += rc;
      }
  }
    if (rcvdData[0] >= 0 && rcvdData[0] < 255) {
      desiredRateLeftEncoder = rcvdData[0];
      desiredRateRightEncoder = rcvdData[1];
    }
    

  }

  if (lmbrake == 1) {
    lmbrake = 0;
  }

  if (desiredRateLeftEncoder == 0) {
    lmbrake = 1;
  }

  if (rmbrake == 1) {
    rmbrake = 0;
  }

  if (desiredRateRightEncoder == 0) {
    rmbrake = 1;
  }
  
// if(Serial.available () != 0){
////  Serial.println(420);
//  int tmp = Serial.parseInt();
//  Serial.read();
//  if (tmp > 0){
//    desiredRateLeftEncoder = tmp;
//    desiredRateRightEncoder = tmp;
//  }
//  else if (tmp == -1){
//    lmspeed = 0;
//    rmspeed = 0;
//    desiredRateLeftEncoder = 0;
//    desiredRateRightEncoder = 0;
//  }
//  
// }
  
  MasterSend(startbyte,2,(int)lmspeed,lmbrake,(int)rmspeed,rmbrake,sv[0],sv[1],sv[2],sv[3],sv[4],sv[5],devibrate,sensitivity,lowbat,i2caddr,i2cfreq);
  delay(50);

  double oldlmenc = lmenc;
  double oldrmenc = rmenc;

  
  MasterReceive();                                   // receive data packet from T'REX controller
  delay(50);


  // Calculates the encoder rate of change. Think about if we really need to divide the rate variable by time. I dont think we do.
  double oldRateRightEncoder = rateRightEncoder;
  double oldRateLeftEncoder = rateLeftEncoder;
  rateRightEncoder = rmenc - oldrmenc;
  rateLeftEncoder = lmenc - oldlmenc;
//  if (rateLeftEncoder > 0) {
//    rateLeftEncoder += 0;
//  }
    if (rateLeftEncoder > 255 || rateLeftEncoder < -255) {
    rateLeftEncoder = oldRateLeftEncoder;
  }
    if (rateRightEncoder > 255 || rateRightEncoder < -255) {
    rateRightEncoder = oldRateRightEncoder;
  }

  if (desiredRateRightEncoder != desiredRateLeftEncoder) {
    resetIntegral = true;
  }
  if (desiredRateRightEncoder == desiredRateLeftEncoder && resetIntegral == true) {
    resetIntegral = false;
    leftPID.setIntegral(0);
    rightPID.setIntegral(0);
  }
  
  
  // PID will tune output.
  leftPID.setIntegral(clamp(leftPID.getIntegral(), 255, -255));
  leftPID.run();
  rightPID.setIntegral(clamp(rightPID.getIntegral(), 255, -255));
  rightPID.run();

//  if (rightPID.atSetPoint(10) && leftPID.atSetPoint(10) && sync == false){
//    sync = true;
//    desiredRateLeftEncoder = 0;
//    desiredRateRightEncoder = 0;
//  }

//  
  Serial.print(rateLeftEncoder);
  Serial.print(", "); // a space ' ' or  tab '\t' character is printed between the two values.
  Serial.print(rateRightEncoder);
  Serial.print(", "); // a space ' ' or  tab '\t' character is printed between the two values.
  
//  Serial.print(desiredRateLeftEncoder);
//  Serial.print(", ");
  Serial.print(lmspeed);
   Serial.print(", ");
  Serial.println(rmspeed);
//  Serial.print(", ");
//  Serial.println(leftPID.getIntegral());

}
