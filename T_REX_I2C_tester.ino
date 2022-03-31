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


//pid settings and gains
#define OUTPUT_MIN_FORWARDS 0                                 
#define OUTPUT_MAX_FORWARDS 180
#define OUTPUT_MIN_BACKWARDS -180                                 
#define OUTPUT_MAX_BACKWARDS 0

#define KP_L 1.2
#define KI_L 1
#define KD_L 0.1
#define KP_R 1.2
#define KI_R 1
#define KD_R 0.1

double moveForwardsIntegralRight, moveForwardsDerivativeRight, moveBackwardsIntegralRight, moveBackwardsDerivativeRight, turnRightIntegralRight, turnRightDerivativeRight, turnLeftIntegralRight, turnLeftDerivativeRight;
double moveForwardsIntegralLeft, moveForwardsDerivativeLeft, moveBackwardsIntegralLeft, moveBackwardsDerivativeLeft, turnRightIntegralLeft, turnRightDerivativeLeft, turnLeftIntegralLeft, turnLeftDerivativeLeft;

bool sync = false;
bool resetIntegral = false;

double desiredRateRightEncoder, desiredRateLeftEncoder, rateLeftEncoder, rateRightEncoder; // output is missing from here as they have been defined above as lmspeed and rmspeed

//input/output variables passed by reference, so they are updated automatically
AutoPID leftPID(&rateLeftEncoder, &desiredRateLeftEncoder, &lmspeed, OUTPUT_MIN_FORWARDS, OUTPUT_MAX_FORWARDS, KP_L, KI_L, KD_L);
AutoPID rightPID(&rateRightEncoder, &desiredRateRightEncoder, &rmspeed, OUTPUT_MIN_FORWARDS, OUTPUT_MAX_FORWARDS, KP_R, KI_R, KD_R);


double clamp(double x, double a, double b) {
  if (x > a) {
    return a;
  } else if (x < b) {
    return b;
  } else {
    return x;
  }
}

void tune(double drenc, double dlenc, String dir) {
  desiredRateRightEncoder = drenc;
  desiredRateLeftEncoder = dlenc;

  long unsigned t1 = millis();
  long unsigned t2 = millis();

  while((t2 - t1) < 7000) {
    t2 = millis();
    MasterSend(startbyte,2,(int)lmspeed,lmbrake,(int)rmspeed,rmbrake,sv[0],sv[1],sv[2],sv[3],sv[4],sv[5],devibrate,sensitivity,lowbat,i2caddr,i2cfreq);
    delay(50);
    double oldlmenc = lmenc;
    double oldrmenc = rmenc;
    MasterReceive();
    delay(50);

    if ((desiredRateRightEncoder > 0 && rmenc < 0) || (desiredRateRightEncoder < 0 && rmenc > 0)) {
      rmenc = -rmenc;
    }
    
    double oldRateRightEncoder = rateRightEncoder;
    double oldRateLeftEncoder = rateLeftEncoder;
    rateRightEncoder = rmenc - oldrmenc;
    rateLeftEncoder = lmenc - oldlmenc;
    leftPID.run();
    rightPID.run();

    if (lmbrake == 1) {
      lmbrake = 0;
      leftPID.reset();
    }
    if (desiredRateLeftEncoder == 0) {
      lmbrake = 1;
      leftPID.reset();
    }
    if (rmbrake == 1) {
      rmbrake = 0;
      rightPID.reset();
    }
    if (desiredRateRightEncoder == 0) {
      rmbrake = 1;
      rightPID.reset();
    }

    Serial.print(rateLeftEncoder);
    Serial.print(", "); // a space ' ' or  tab '\t' character is printed between the two values.
    Serial.print(rateRightEncoder);
    Serial.print(", "); // a space ' ' or  tab '\t' character is printed between the two values.
  
//  Serial.print(desiredRateLeftEncoder);
//  Serial.print(", ");
    Serial.print(lmspeed);
    Serial.print(", ");
    Serial.println(rmspeed);
  }

  if (dir == "FORWARDS") {
    moveForwardsIntegralRight = rightPID.getIntegral();
    moveForwardsIntegralLeft = leftPID.getIntegral();
  } else if (dir == "BACKWARDS") {
    moveBackwardsIntegralRight = rightPID.getIntegral();
    moveBackwardsIntegralLeft = leftPID.getIntegral();
  } else if (dir == "LEFT") {
    turnLeftIntegralRight = rightPID.getIntegral();
    turnLeftIntegralLeft = leftPID.getIntegral();
  } else if (dir == "RIGHT") {
    turnRightIntegralRight = rightPID.getIntegral();
    turnRightIntegralLeft = leftPID.getIntegral();
  }

  leftPID.reset();
  rightPID.reset();
  
}


void tuneForwards() {
  tune(30,30,"FORWARDS");
  tune(0,0,"NOTHING");
  leftPID.setOutputRange(OUTPUT_MIN_BACKWARDS, OUTPUT_MAX_BACKWARDS);
  rightPID.setOutputRange(OUTPUT_MIN_BACKWARDS, OUTPUT_MAX_BACKWARDS);
  tune(-30,-30, "BACKWARDS");
  tune(0,0,"NOTHING");
  leftPID.setOutputRange(OUTPUT_MIN_BACKWARDS, OUTPUT_MAX_BACKWARDS);
  rightPID.setOutputRange(OUTPUT_MIN_FORWARDS, OUTPUT_MAX_FORWARDS);
  tune(20,-20, "LEFT");
  tune(0,0,"NOTHING");
  leftPID.setOutputRange(OUTPUT_MIN_FORWARDS, OUTPUT_MAX_FORWARDS);
  rightPID.setOutputRange(OUTPUT_MIN_BACKWARDS, OUTPUT_MAX_BACKWARDS);
  tune(-20,20, "RIGHT");
  tune(0,0,"NOTHING");
  
  leftPID.setOutputRange(OUTPUT_MIN_FORWARDS, OUTPUT_MAX_FORWARDS);
  rightPID.setOutputRange(OUTPUT_MIN_FORWARDS, OUTPUT_MAX_FORWARDS);
}

void setup()
{ 
  Wire.begin();
  Serial.begin(115200);
  desiredRateLeftEncoder = 0;
  desiredRateRightEncoder = 0;

  // tune output every 0.1s
  leftPID.setTimeStep(100);
  rightPID.setTimeStep(100);

  tuneForwards();
}


void loop()
{

  double oldDesiredRateLeftEncoder = desiredRateLeftEncoder;
  double oldDesiredRateRightEncoder = desiredRateRightEncoder;

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
    if (rcvdData[0] >= -255 && rcvdData[0] <= 255) {
      desiredRateLeftEncoder = rcvdData[0];
      desiredRateRightEncoder = rcvdData[1];
    }
  }
  

  if (lmbrake == 1) {
    lmbrake = 0;
    leftPID.reset();
  }
  if (desiredRateLeftEncoder == 0) {
    lmbrake = 1;
    leftPID.reset();
  }
  if (rmbrake == 1) {
    rmbrake = 0;
    rightPID.reset();
  }
  if (desiredRateRightEncoder == 0) {
    rmbrake = 1;
    rightPID.reset();
  }

  if ((desiredRateRightEncoder == desiredRateLeftEncoder && desiredRateRightEncoder > 0) && oldDesiredRateRightEncoder != desiredRateRightEncoder) {
    rightPID.setIntegral(moveForwardsIntegralRight);
    leftPID.setIntegral(moveForwardsIntegralLeft);
  } else if ((desiredRateRightEncoder == desiredRateLeftEncoder && desiredRateRightEncoder < 0) && oldDesiredRateRightEncoder != desiredRateRightEncoder) {
    rightPID.setIntegral(moveBackwardsIntegralRight);
    leftPID.setIntegral(moveBackwardsIntegralLeft);
  } else if ((desiredRateLeftEncoder < 0 && oldDesiredRateLeftEncoder >= 0)) {
    rightPID.setIntegral(turnLeftIntegralRight);
    leftPID.setIntegral(turnLeftIntegralLeft);
  } else if (desiredRateRightEncoder < 0 && oldDesiredRateRightEncoder >= 0) {
    rightPID.setIntegral(turnRightIntegralRight);
    leftPID.setIntegral(turnRightIntegralLeft);
  }
  
  MasterSend(startbyte,2,(int)lmspeed,lmbrake,(int)rmspeed,rmbrake,sv[0],sv[1],sv[2],sv[3],sv[4],sv[5],devibrate,sensitivity,lowbat,i2caddr,i2cfreq);
  delay(50);

  double oldlmenc = lmenc;
  double oldrmenc = rmenc;

  MasterReceive();                                   // receive data packet from T'REX controller
  delay(50);

  if ((desiredRateRightEncoder > 0 && rmenc < 0) || (desiredRateRightEncoder < 0 && rmenc > 0)) {
    rmenc = -rmenc;
  }


  // Calculates the encoder rate of change. Think about if we really need to divide the rate variable by time. I dont think we do.
  double oldRateRightEncoder = rateRightEncoder;
  double oldRateLeftEncoder = rateLeftEncoder;
  rateRightEncoder = rmenc - oldrmenc;
  rateLeftEncoder = lmenc - oldlmenc;

  // PREVENTS INTEGER OVERFLOW COMING FROM MOTOR BOARD
  if (rateLeftEncoder > 255 || rateLeftEncoder < -255) {
    rateLeftEncoder = oldRateLeftEncoder;
  }
  if (rateRightEncoder > 255 || rateRightEncoder < -255) {
    rateRightEncoder = oldRateRightEncoder;
  }

  // reset integral after every rotation/move. decide between the two
  if (desiredRateRightEncoder != desiredRateLeftEncoder) {
    resetIntegral = true;
  }
  if (desiredRateRightEncoder == desiredRateLeftEncoder && resetIntegral == true) {
    resetIntegral = false;
    leftPID.setIntegral(0);
    rightPID.setIntegral(0);
  }

  // change max and min out of pid based on direction of motors. Added for motor protection
  if (desiredRateRightEncoder < 0 && oldDesiredRateRightEncoder >= 0) {
    rightPID.setOutputRange(OUTPUT_MIN_BACKWARDS, OUTPUT_MAX_BACKWARDS);
  } else if (desiredRateRightEncoder >= 0 && oldDesiredRateRightEncoder < 0) {
    rightPID.setOutputRange(OUTPUT_MIN_FORWARDS, OUTPUT_MAX_FORWARDS);
  }
  if (desiredRateLeftEncoder < 0 && oldDesiredRateLeftEncoder >= 0) {
    leftPID.setOutputRange(OUTPUT_MIN_BACKWARDS, OUTPUT_MAX_BACKWARDS);
  } else if (desiredRateLeftEncoder >= 0 && oldDesiredRateLeftEncoder < 0) {
    leftPID.setOutputRange(OUTPUT_MIN_FORWARDS, OUTPUT_MAX_FORWARDS);
  }
  
  
  
  // PID integral clamps
  leftPID.setIntegral(clamp(leftPID.getIntegral(), 255, -255));
  leftPID.run();
  rightPID.setIntegral(clamp(rightPID.getIntegral(), 255, -255));
  rightPID.run();

  
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
