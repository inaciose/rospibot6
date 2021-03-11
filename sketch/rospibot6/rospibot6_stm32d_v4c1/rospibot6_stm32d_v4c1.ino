//#define USE_I2C

int leftHzPulses = 0;
int leftHzPulsesLast = 0;
int rightHzPulses = 0;
int rightHzPulsesLast = 0;

#define HZ_TIME 1000
unsigned long hzTimer = 0;

#define ROTATE_DEGREES (1.0 * 360.0)

#define PID_TARGET 5

#define ENCODER_PULSES 512.0
#define WHEEL_RADIUS 0.0331
#define WHEEL_AXIS 0.183

#define CMD_RESET_ENCODERS 1
#define CMD_SET_PWM 2           // params: pwmLeft, pwmRight
#define CMD_LIDAR_MOTOR_ON 3
#define CMD_LIDAR_MOTOR_OFF 4
#define CMD_SET_MOTION 5        // params: leftSetPoint, rightSetpoint, distance
#define CMD_SET_VEL 6

#define CMD_SET_PID_KL  81
#define CMD_SET_PID_KR  82
#define CMD_SET_PID_KS  83

//
// MOTORS
//

// motors pins
#define IN1 PA0
#define IN2 PA1
#define IN3 PA2
#define IN4 PA3
#define ENA PB0
#define ENB PB1

// LeftMotor
#define leftMotorEn ENA
#define leftMotorForward IN1
#define leftMotorBackward IN2

// RightMotor
#define rightMotorEn ENB
#define rightMotorForward IN3
#define rightMotorBackward IN4

signed int leftMotorPwmOut = 0;
signed int rightMotorPwmOut = 0;
signed int leftMotorPwmOutCmd = 0;
signed int rightMotorPwmOutCmd = 0;
bool setPwmStatus = false;

//
// ENCODERS
//

#define ENCODER_LEFT_FUNCTION encoderLeftCounter
#define ENCODER_LEFT_PINA PA4
#define ENCODER_LEFT_PINB PA5
#define ENCODER_LEFT_SIGNAL CHANGE
int encoderLeftPinLast; // control
volatile long encoderLeftPulses; // the number of pulses
volatile long encoderLeftPulsesSpeedPID; // the number of pulses for PID
volatile long encoderLeftPulsesSteeringPID; // the number of pulses for PID, must be reset at same time

#define ENCODER_RIGHT_FUNCTION encoderRightCounter
#define ENCODER_RIGHT_PINA PA6
#define ENCODER_RIGHT_PINB PA7
#define ENCODER_RIGHT_SIGNAL CHANGE
int encoderRightPinLast; // control
volatile long encoderRightPulses; // the number of pulses
volatile long encoderRightPulsesSpeedPID; // the number of pulses PID
volatile long encoderRightPulsesSteeringPID; // the number of pulses for PID, must be reset at same time

long encoderLeftPulsesTarget = 0;
long encoderRightPulsesTarget = 0;
long encoderLeftPulsesTargetStart = 0;
long encoderRightPulsesTargetStart = 0;
bool encoderLeftPulsesOnTarget = false;
bool encoderRightPulsesOnTarget = false;
bool encoderPulsesTargetEnabled = false;
int encoderLeftPulsesTargetStopOffset = 0;
int encoderRightPulsesTargetStopOffset = 0;

long pulsesTarget;

#include "pid.h"

#ifdef USE_I2C
  #define I2C_NODE 11
  #include <Wire_slave.h>
#endif

#define LED_PIN PC13
#define LED_TIME 1000
unsigned long ledTimer = 0;
bool blinkState = false;

#define MOTOR_TIME 200
unsigned long motorTimer = 0;
int motorPwm = 0;
boolean motorPwmUp = true;

#define LIDAR_MOTOR_PIN PB12
int lidarMotorPower = 0;

void encoderLeftCounter() {
  int Lstate = digitalRead(ENCODER_LEFT_PINA);
  if((encoderLeftPinLast == LOW) && Lstate==HIGH) {
    if(digitalRead(ENCODER_LEFT_PINB) == LOW) {
      encoderLeftPulses--;
      // pid
      encoderLeftPulsesSpeedPID--;
      encoderLeftPulsesSteeringPID--;
    } else {
      encoderLeftPulses++;
      // pid
      encoderLeftPulsesSpeedPID++;
      encoderLeftPulsesSteeringPID++;
    }
  }  
  leftHzPulses++; //DEBUG
  encoderLeftPinLast = Lstate; 
}

void encoderRightCounter() {
  int Lstate = digitalRead(ENCODER_RIGHT_PINA);
  if((encoderRightPinLast == LOW) && Lstate==HIGH) {
    if(digitalRead(ENCODER_RIGHT_PINB) == LOW) {
      encoderRightPulses++;
      encoderRightPulsesSpeedPID++;
      encoderRightPulsesSteeringPID++;    
    } else {
      encoderRightPulses--;
      encoderRightPulsesSpeedPID--;
      encoderRightPulsesSteeringPID--;
    }
  }
  rightHzPulses++; //DEBUG
  encoderRightPinLast = Lstate;
}

void setup() {

  Serial.begin(115200);

  // set direction pins mode as output
  pinMode(leftMotorForward, OUTPUT);
  pinMode(leftMotorBackward, OUTPUT);
  pinMode(rightMotorForward, OUTPUT);
  pinMode(rightMotorBackward, OUTPUT);

  // set velocity pins mode as PWM output
  pinMode(leftMotorEn, PWM);
  pinMode(rightMotorEn, PWM);


  // set motors brake
  digitalWrite(leftMotorForward, HIGH);
  digitalWrite(leftMotorBackward, HIGH);
  digitalWrite(rightMotorForward, HIGH);
  digitalWrite(rightMotorBackward, HIGH);
  delay(250);

  // set motors freewheel
  digitalWrite(leftMotorForward, LOW);
  digitalWrite(leftMotorBackward, LOW);
  digitalWrite(rightMotorForward, LOW);
  digitalWrite(rightMotorBackward, LOW);
  delay(250);

  // encoders
  pinMode(ENCODER_LEFT_PINA,INPUT);
  pinMode(ENCODER_LEFT_PINB,INPUT);
  attachInterrupt(ENCODER_LEFT_PINA, ENCODER_LEFT_FUNCTION, ENCODER_LEFT_SIGNAL);
  pinMode(ENCODER_RIGHT_PINA,INPUT);
  pinMode(ENCODER_RIGHT_PINB,INPUT);
  attachInterrupt(ENCODER_RIGHT_PINA, ENCODER_RIGHT_FUNCTION, ENCODER_RIGHT_SIGNAL);

  // pid
  setup_PID();

  #ifdef USE_I2C
    // i2c slave
    Wire.begin(I2C_NODE);
    Wire.onRequest(requestEvent);
    Wire.onReceive(receiveEvent);    
  #else


    // set pid's & pulse targets
    float pulsesPerMeter = (WHEEL_RADIUS * 2 * PI) / ENCODER_PULSES;
    Serial.print(pulsesPerMeter); Serial.print("\t"); Serial.print(WHEEL_RADIUS); Serial.print("\t"); Serial.print(WHEEL_AXIS); Serial.print("\t"); Serial.print(ENCODER_PULSES); Serial.println();          

    // for rotation
    float distance = ROTATE_DEGREES * PI /180.0 * (WHEEL_AXIS / 2);

    // for forward    
    //
    
    // set pulses for distance
    pulsesTarget = distance / pulsesPerMeter; 

    // pid speed/direction targets
    leftSpeedPidSetPointTmp = PID_TARGET;
    rightSpeedPidSetPointTmp = -PID_TARGET;

    Serial.print(leftSpeedPidSetPointTmp); Serial.print("\t"); Serial.print(rightSpeedPidSetPointTmp); Serial.print("\t"); Serial.print(pulsesTarget); Serial.println();          
    setPidTarget();
    setPulsesTarget(pulsesTarget);
  
  #endif
  
  // set motors forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN3, HIGH);
  motorTimer = millis() + MOTOR_TIME;

  // lidar Power
  pinMode(LIDAR_MOTOR_PIN, OUTPUT);
  digitalWrite(LIDAR_MOTOR_PIN, lidarMotorPower);
  
  // configure LED pin for output
  pinMode(LED_PIN, OUTPUT);
  ledTimer = millis() + LED_TIME;

}

void loop() {

  #ifndef USE_I2C
  static int counter = 0;
  if(counter > 1) {
    Serial.print("end");
    while(true) {}
  }  
  #endif
  
  update_PID();

  // bof: debug
  // hz check
  int leftHz = 0;
  int rightHz = 0;
  if(millis() > hzTimer) {
    hzTimer = millis() + HZ_TIME;
    leftHz = leftHzPulses - leftHzPulsesLast;
    leftHzPulsesLast = leftHzPulses;
    rightHz = rightHzPulses - rightHzPulsesLast;
    rightHzPulsesLast = rightHzPulses;
  }
  
  static double encoderLeftPulsesLast = 0;
  static double encoderRightPulsesLast = 0;

/*
  if(encoderLeftPulses != encoderLeftPulsesLast || encoderRightPulses != encoderRightPulsesLast) {
    if(encoderLeftPulses != encoderLeftPulsesLast) encoderLeftPulsesLast = encoderLeftPulses;
    if(encoderRightPulses != encoderRightPulsesLast) encoderRightPulsesLast = encoderRightPulses;
*/
  if(leftHz || rightHz) {

    Serial.print("piderr ("); 
    Serial.print(leftSpeedPidInputLast - leftSpeedPidSetPoint); Serial.print("\t"); 
    Serial.print(rightSpeedPidInputLast - rightSpeedPidSetPoint); Serial.print("\t"); 
    Serial.print(steeringPidInputLast); Serial.print(")\t hz (");

    Serial.print(leftHz / 2); Serial.print("\t"); Serial.print(rightHz /2 ); Serial.print(")\tpulses(");  Serial.print(encoderLeftPulses); Serial.print("\t"); Serial.print(encoderRightPulses);Serial.print(")\t target("); Serial.print(encoderLeftPulsesTarget); Serial.print("\t"); Serial.print(encoderRightPulsesTarget); Serial.println(")\t");
    //Serial.print(leftMotorPwmOut); Serial.print("\t"); Serial.print(rightMotorPwmOut); Serial.print("\t"); Serial.print(encoderLeftPulses); Serial.print("\t"); Serial.print(encoderRightPulses); Serial.println("\t");
  }
  // eof debug
  

  // check encoder targets
  if(encoderPulsesTargetEnabled) {
    //Serial.print("CKENC: "); Serial.print(leftSpeedPidSetPointDirection); Serial.print("\t"); Serial.print(rightSpeedPidSetPointDirection); Serial.print("\t"); 
    //Serial.print(encoderLeftPulses);Serial.print("\t"); Serial.print(encoderLeftPulsesTarget); Serial.print("\t"); 
    //Serial.print(encoderRightPulses);Serial.print("\t"); Serial.print(encoderRightPulsesTarget); Serial.println();
    //
    // check left encoder target
    //
    if(!encoderLeftPulsesOnTarget) {
      if(leftSpeedPidSetPointDirection >= 0) {
        if(encoderLeftPulses >= encoderLeftPulsesTarget - encoderLeftPulsesTargetStopOffset) {
          encoderLeftPulsesOnTarget = true;  
        }
      } else {
        if(encoderLeftPulses < encoderLeftPulsesTarget + encoderLeftPulsesTargetStopOffset) {
          encoderLeftPulsesOnTarget = true;
        }      
      }
      // left stop on encoder target
      if(encoderLeftPulsesOnTarget) {
        Serial.println("L ON TARGET");
        leftMotorPwmOut = 0;
        leftSpeedPidSetPoint = 0;
        leftSpeedPidSetPointDirection = 0; 
      }
    }
    //
    // check right encoder target
    //
    if(!encoderRightPulsesOnTarget) {
      if(rightSpeedPidSetPointDirection >= 0) {
        if(encoderRightPulses >= encoderRightPulsesTarget - encoderRightPulsesTargetStopOffset) {
          encoderRightPulsesOnTarget = true;  
        }
      } else {
        if(encoderRightPulses < encoderRightPulsesTarget + encoderRightPulsesTargetStopOffset) {
          encoderRightPulsesOnTarget = true;
        }      
      }
      // left stop on encoder target
      if(encoderRightPulsesOnTarget) {
        Serial.println("R ON TARGET");
        rightMotorPwmOut = 0;
        rightSpeedPidSetPoint = 0;
        rightSpeedPidSetPointDirection = 0;
      }
    }
    
    //
    // encoders on target
    //
    if(encoderLeftPulsesOnTarget && encoderRightPulsesOnTarget) {
      //Serial.println("BOTH ON TARGET");

/*
      Serial.print(useSteeringPid); Serial.print("\t"); Serial.print(steeringPidInputLast); Serial.print("\t");
      Serial.print(encoderLeftPulses); Serial.print("\t"); Serial.print(encoderRightPulses); Serial.print("\t");
      Serial.print(encoderLeftPulsesTarget); Serial.print("\t"); Serial.print(encoderRightPulsesTarget); Serial.print("\t");
      Serial.print(encoderLeftPulses - encoderLeftPulsesTargetStart); Serial.print("\t"); Serial.print(encoderRightPulses - encoderRightPulsesTargetStart); Serial.print("\t");
      Serial.print(leftSpeedPidSetPoint); Serial.print("\t"); Serial.print(rightSpeedPidSetPoint); Serial.print("\t");
      Serial.print(leftSpeedPidInputLast - leftSpeedPidSetPoint); Serial.print("\t"); 
      Serial.print(rightSpeedPidInputLast - rightSpeedPidSetPoint); Serial.print("\t"); 
      Serial.print(leftSpeedPidOutput); Serial.print("\t"); Serial.print(rightSpeedPidOutput); Serial.print("\t");
      Serial.print("\n");
      
      delay(100);
  
      //Serial.print(useSteeringPid); Serial.print("\t"); Serial.print(steeringPidInputLast); Serial.print("\t");
      Serial.print(encoderLeftPulses); Serial.print("\t"); Serial.print(encoderRightPulses); Serial.print("\t");
      Serial.print(leftMotorPwmOut); Serial.print("\t"); Serial.print(rightMotorPwmOut); Serial.print("\t");

      Serial.print(encoderLeftPulsesTarget); Serial.print("\t"); Serial.print(encoderRightPulsesTarget); Serial.print("\t");    
      Serial.print(encoderLeftPulses - encoderLeftPulsesTargetStart); Serial.print("\t"); Serial.print(encoderRightPulses - encoderRightPulsesTargetStart); Serial.print("\t");
      Serial.print(leftSpeedPidSetPoint); Serial.print("\t"); Serial.print(rightSpeedPidSetPoint); Serial.print("\t");
      Serial.print(leftSpeedPidInputLast - leftSpeedPidSetPoint); Serial.print("\t"); 
      Serial.print(rightSpeedPidInputLast - rightSpeedPidSetPoint); Serial.print("\t"); 
      Serial.print(leftSpeedPidOutput); Serial.print("\t"); Serial.print(rightSpeedPidOutput); Serial.print("\t");
      Serial.print("\n");

      delay(100);
      Serial.print(encoderLeftPulses); Serial.print("\t"); Serial.print(encoderRightPulses); Serial.print("\t");
      Serial.print(leftMotorPwmOut); Serial.print("\t"); Serial.print(rightMotorPwmOut); Serial.print("\t");
      Serial.print("\n");

      delay(100);
      Serial.print(encoderLeftPulses); Serial.print("\t"); Serial.print(encoderRightPulses); Serial.print("\t");
      Serial.print(leftMotorPwmOut); Serial.print("\t"); Serial.print(rightMotorPwmOut); Serial.print("\t");
      Serial.print("\n");

      delay(100);
      Serial.print(encoderLeftPulses); Serial.print("\t"); Serial.print(encoderRightPulses); Serial.print("\t");
      Serial.print(leftMotorPwmOut); Serial.print("\t"); Serial.print(rightMotorPwmOut); Serial.print("\t");
      Serial.print("\n");
      
      delay(100);
      Serial.print(encoderLeftPulses); Serial.print("\t"); Serial.print(encoderRightPulses); Serial.print("\t");
      Serial.print(leftMotorPwmOut); Serial.print("\t"); Serial.print(rightMotorPwmOut); Serial.print("\t");
      Serial.print("\n");
      */

      encoderLeftPulsesTargetStart = 0;
      encoderRightPulsesTargetStart = 0;
      encoderLeftPulsesTarget = 0;
      encoderLeftPulsesOnTarget = false;
      encoderRightPulsesTarget = 0;
      encoderRightPulsesOnTarget = false;
      encoderPulsesTargetEnabled = false;
      if(useSteeringPid) {
        steeringPid.SetMode(MANUAL);
        useSteeringPid = false;        
      }
    }
  }
  
  // set speed  
  if(setPwmStatus) {    
    leftMotorPwmOut = leftMotorPwmOutCmd;
    rightMotorPwmOut = rightMotorPwmOutCmd;
    setPwmStatus = false;
  } else {
    leftMotorPwmOut = 0;
    rightMotorPwmOut = 0;
    if(leftSpeedPidSetPoint != 0) {
      leftMotorPwmOut = leftSpeedPidOutput * leftSpeedPidSetPointDirection; // - steeringPidOutput;
    }
    if(rightSpeedPidSetPoint != 0) {
      rightMotorPwmOut = rightSpeedPidOutput * rightSpeedPidSetPointDirection; // + steeringPidOutput;
    }
  }
  
  bodyMotorsControl();

  // activity led
  if(millis() > ledTimer) {
    ledTimer = millis() + LED_TIME;
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);    
  }

  #ifndef USE_I2C

  if(!encoderPulsesTargetEnabled) {
    // end of motion
    delay(1000);
    
    // pid speed/direction targets
    leftSpeedPidSetPointTmp = -leftSpeedPidSetPointTmp;
    rightSpeedPidSetPointTmp = -rightSpeedPidSetPointTmp;  
    Serial.print("Restart motion: ("); Serial.print(leftSpeedPidSetPointTmp); Serial.print("\t"); Serial.print(rightSpeedPidSetPointTmp); Serial.print(")\t"); Serial.print(pulsesTarget); Serial.print("\t("); Serial.print(encoderLeftPulses); Serial.print("\t"); Serial.print(encoderRightPulses);Serial.println(")");          
    setPidTarget();
    setPulsesTarget(pulsesTarget);
    counter++;
  }
    
  #endif
}

#ifdef USE_I2C
  
  void requestEvent() {
    union u_tag {
      byte b[4];
      int i;
    } i2c_data[32];
  
    i2c_data[0].i = 0;
    i2c_data[1].i = encoderLeftPulses;
    i2c_data[2].i = encoderRightPulses;
    i2c_data[3].i = leftSpeedPidSetPoint - leftSpeedPidInputLast;
    i2c_data[4].i = rightSpeedPidSetPoint - rightSpeedPidInputLast;
    i2c_data[5].i = steeringPidInputLast;
  
    //Wire.write((byte*)&i2c_data, 12);
    Wire.write((byte*)&i2c_data, 24);
  }
  
  void receiveEvent(int howMany){
    
    int encoderPulsesTarget = 0;
    double Kp, Ki, Kd;
    
    union u_tag {
      byte b[4];
      int i;
      float f;
    } i2c_data[32];
    
    // set receive buffer elements to zero
    for(int f = 0; f < 4; f++) i2c_data[f].i = 0;
  
    // collect incoming data
    byte *ptr = (byte*)&i2c_data;
    while(Wire.available()) {
      byte b = Wire.read();
      *ptr = b;
      ptr++;
    }
  
    if(i2c_data[0].b[0]) {
      Serial.println(i2c_data[0].b[0]);
    }
  
    // do cmd on first byte
    switch(i2c_data[0].b[0]) {
      
      case CMD_RESET_ENCODERS:
        // reset encoders
        //Serial.println("reset encoders");
        encoderLeftPulses = 0;
        encoderRightPulses = 0;
        break;
        
      case CMD_SET_PWM:
        // set pwm
        //Serial.println("set pwm");
        leftMotorPwmOutCmd = i2c_data[1].i;
        rightMotorPwmOutCmd = i2c_data[2].i;
        setPwmStatus = true;
        break;
        
      case CMD_LIDAR_MOTOR_ON:
        //Serial.println("set lidar on");
        // lidar motor on
        lidarMotorPower = HIGH;
        digitalWrite(LIDAR_MOTOR_PIN, lidarMotorPower);
        break;
        
      case CMD_LIDAR_MOTOR_OFF:
        // lidar motor off
        //Serial.println("set lidar off");
        lidarMotorPower = LOW;
        digitalWrite(LIDAR_MOTOR_PIN, lidarMotorPower);
        break;
        
      case CMD_SET_MOTION:
      case CMD_SET_VEL:
        // get i2c data to set pid params and encoder target
        //Serial.println("set motion");
        
        leftSpeedPidSetPointTmp = i2c_data[1].i;
        rightSpeedPidSetPointTmp = i2c_data[2].i;
        if(i2c_data[0].b[0] == CMD_SET_MOTION) {
          encoderPulsesTarget = i2c_data[3].i;
        } else {
          encoderPulsesTarget = 0;
        }
  
        //Serial.print(leftSpeedPidSetPointTmp); Serial.print("\t"); Serial.print(rightSpeedPidSetPointTmp); Serial.print("\t"); Serial.print(encoderPulsesTarget); Serial.println();      
        setPidTarget();
        setPulsesTarget(encoderPulsesTarget);
        break;
  
  /*
      case CMD_SET_VEL:
        // set pidSetpoint
        //Serial.println("set pwm");
        leftMotorPwmOut = i2c_data[1].i;
        rightMotorPwmOut = i2c_data[2].i;
        break;
  */
  
      case CMD_SET_PID_KL:
        Kp = i2c_data[1].i / 1000;
        Ki = i2c_data[2].i / 1000;
        Kd = i2c_data[3].i / 1000;
        leftSpeedPid.SetTunings(Kp, Ki, Kd);      
        Serial.print(leftSpeedPid.GetKp()); Serial.print("\t"); Serial.print(leftSpeedPid.GetKi()); Serial.print("\t"); Serial.print(leftSpeedPid.GetKd()); Serial.println();
        break;
        
      case CMD_SET_PID_KR:
        Kp = i2c_data[1].i / 1000;
        Ki = i2c_data[2].i / 1000;
        Kd = i2c_data[3].i / 1000;
        rightSpeedPid.SetTunings(Kp, Ki, Kd);
        Serial.print(leftSpeedPid.GetKp()); Serial.print("\t"); Serial.print(leftSpeedPid.GetKi()); Serial.print("\t"); Serial.print(leftSpeedPid.GetKd()); Serial.println();
        break;
        
      case CMD_SET_PID_KS:
        Kp = i2c_data[1].i / 1000;
        Ki = i2c_data[2].i / 1000;
        Kd = i2c_data[3].i / 1000;
        steeringPid.SetTunings(Kp, Ki, Kd);
        Serial.print(leftSpeedPid.GetKp()); Serial.print("\t"); Serial.print(leftSpeedPid.GetKi()); Serial.print("\t"); Serial.print(leftSpeedPid.GetKd()); Serial.println();
        break;      
    }
  }

#endif

void setPulsesTarget( int encoderPulsesTarget) {

  // set the target for encoders if any
  if(encoderPulsesTarget) {
    // set left encoder target
    if(leftSpeedPidSetPointDirection >= 0) {
      encoderLeftPulsesTarget = encoderLeftPulses + abs(encoderPulsesTarget);    
    } else {
      encoderLeftPulsesTarget = encoderLeftPulses - abs(encoderPulsesTarget);
    }
    // set right encoder target
    if(rightSpeedPidSetPointDirection >= 0) {
      encoderRightPulsesTarget = encoderRightPulses + abs(encoderPulsesTarget);
    } else {      
      encoderRightPulsesTarget = encoderRightPulses - abs(encoderPulsesTarget);
    }
  
    // set control to check
    encoderLeftPulsesOnTarget = false;
    encoderRightPulsesOnTarget = false;
    encoderPulsesTargetEnabled = true;
    // debug only
    encoderLeftPulsesTargetStart = encoderLeftPulses;
    encoderRightPulsesTargetStart = encoderRightPulses;
  } else {
    // reset control
    encoderLeftPulsesTarget = 0;
    encoderRightPulsesTarget = 0;
    encoderLeftPulsesOnTarget = false;
    encoderRightPulsesOnTarget = false;
    encoderPulsesTargetEnabled = false;
    // debug only
    encoderLeftPulsesTargetStart = 0;
    encoderRightPulsesTargetStart = 0;
  }
  Serial.print("pulsesTarget");  Serial.print("\t"); Serial.print(encoderPulsesTargetEnabled); Serial.print("\t"); Serial.print(encoderLeftPulsesTarget); Serial.print("\t"); Serial.print(encoderRightPulsesTarget); Serial.println();
}

//
// motor control
//

void bodyMotorsControl() {

  // brake control vars
  static bool leftBrake = false;
  static bool rightBrake = false;

/*
  static int leftZeroCounter = 0;
  static int rightZeroCounter = 0;
  const int zeroCounterLimit = 100;

  // left brake control
  if(leftMotorPwmOut == 0) {
    if(!leftBrake && leftZeroCounter == 0) leftBrake = true;
    if(leftZeroCounter <= zeroCounterLimit) leftZeroCounter++;
    if(leftZeroCounter > zeroCounterLimit) leftBrake = false;    
  } else {
    leftZeroCounter = 0;
    leftBrake = false;
  }

  // left brake control
  if(rightMotorPwmOut == 0) {
    if(!rightBrake && rightZeroCounter == 0) rightBrake = true;
    if(rightZeroCounter <= zeroCounterLimit) rightZeroCounter++;
    if(rightZeroCounter > zeroCounterLimit) rightBrake = false;    
  } else {
    rightZeroCounter = 0;
    rightBrake = false;
  }
*/
  
  // set motor left direction & speed
  if(leftMotorPwmOut == 0) {

    if(leftBrake) {
      setBodyMotorLeftBrake();
      //setBodyMotorLeftStop();
      pwmWrite(leftMotorEn,64000);            
    } else {
      //setBodyMotorLeftBrake();
      setBodyMotorLeftStop();
      pwmWrite(leftMotorEn, 0);
    }
    
  } else if(leftMotorPwmOut > 0) {
    setBodyMotorLeftForward();
    pwmWrite(leftMotorEn, leftMotorPwmOut);
  } else {
    setBodyMotorLeftBackward();
    pwmWrite(leftMotorEn, abs(leftMotorPwmOut));    
  }
  
  // set motor right direction & speed
  if(rightMotorPwmOut == 0) {

    if(rightBrake) {
      setBodyMotorRightStop();
      //setBodyMotorRightBrake();
      pwmWrite(rightMotorEn, 64000);
    } else {
      setBodyMotorRightStop();
      //setBodyMotorRightBrake();
      pwmWrite(rightMotorEn, 0);
    }

  } else if(rightMotorPwmOut > 0) {
    setBodyMotorRightForward();
    pwmWrite(rightMotorEn, rightMotorPwmOut);
  } else {
    setBodyMotorRightBackward();    
    pwmWrite(rightMotorEn, abs(rightMotorPwmOut));
  }
}

void setBodyMotorLeftForward() {
  digitalWrite(leftMotorForward, HIGH);
  digitalWrite(leftMotorBackward, LOW);
}

void setBodyMotorLeftBackward() {
  digitalWrite(leftMotorForward, LOW);
  digitalWrite(leftMotorBackward, HIGH);
}

void setBodyMotorRightForward() {
  digitalWrite(rightMotorForward, HIGH);
  digitalWrite(rightMotorBackward, LOW);
}

void setBodyMotorRightBackward() {
  digitalWrite(rightMotorForward, LOW);
  digitalWrite(rightMotorBackward, HIGH);
}

void setBodyMotorLeftStop() {
  digitalWrite(leftMotorForward, LOW);
  digitalWrite(leftMotorBackward, LOW);
}

void setBodyMotorRightStop() {
  digitalWrite(rightMotorForward, LOW);
  digitalWrite(rightMotorBackward, LOW);
}

void setBodyMotorLeftBrake() {
  digitalWrite(leftMotorForward, HIGH);
  digitalWrite(leftMotorBackward, HIGH);
}

void setBodyMotorRightBrake() {
  digitalWrite(rightMotorForward, HIGH);
  digitalWrite(rightMotorBackward, HIGH);
}
