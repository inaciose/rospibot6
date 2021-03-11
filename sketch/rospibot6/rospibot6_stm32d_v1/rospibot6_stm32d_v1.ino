#define MCU_CMD_RESET_ENCODERS 1
#define MCU_CMD_SET_PWM 2
#define MCU_CMD_LIDAR_MOTOR_ON 3
#define MCU_CMD_LIDAR_MOTOR_OFF 4

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

//
// ENCODERS
//

#define ENCODER_LEFT_FUNCTION encoderLeftCounter
#define ENCODER_LEFT_PINA PA4
#define ENCODER_LEFT_PINB PA5
#define ENCODER_LEFT_SIGNAL CHANGE
byte encoderLeftPinLast; // control
volatile signed int encoderLeftPulses; // the number of pulses
boolean encoderLeftDirection; // the rotation direction 

#define ENCODER_RIGHT_FUNCTION encoderRightCounter
#define ENCODER_RIGHT_PINA PA6
#define ENCODER_RIGHT_PINB PA7
#define ENCODER_RIGHT_SIGNAL CHANGE
byte encoderRightPinLast; // control
volatile signed int encoderRightPulses; // the number of pulses
boolean encoderRightDirection; // the rotation direction 

signed int encoderLeftPulsesI2c = 0;
signed int encoderRightPulsesI2c = 0;
signed int encodersByte = -1;
byte *encodersBytePtr;

// serial data manipulation
union u_tag {
  byte b[4];
  int i;
  float f;
};

#define I2C_NODE 11
#include <Wire_slave.h>

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
//unsigned long lidarMotorPowerTimer;


void encoderLeftCounter() {
  int Lstate = digitalRead(ENCODER_LEFT_PINB);
  if((encoderLeftPinLast == LOW) && Lstate==HIGH) {
    int val = digitalRead(ENCODER_LEFT_PINA);
    if(val == LOW && encoderLeftDirection) {
      encoderLeftDirection = false;
    } else if(val == HIGH && !encoderLeftDirection) {
      encoderLeftDirection = true;
    }
  }
  encoderLeftPinLast = Lstate; 
  if(!encoderLeftDirection) {
    encoderLeftPulses++;
  } else {
    encoderLeftPulses--;
  }
}

void encoderRightCounter() {
  int Lstate = digitalRead(ENCODER_RIGHT_PINB);
  if((encoderRightPinLast == LOW) && Lstate==HIGH) {
    int val = digitalRead(ENCODER_RIGHT_PINA);
    if(val == LOW && encoderRightDirection) {
      encoderRightDirection = false; //Reverse
    } else if(val == HIGH && !encoderRightDirection) {
      encoderRightDirection = true;  //Forward
    }
  }
  encoderRightPinLast = Lstate; 
  if(!encoderRightDirection) {
    encoderRightPulses--;
  } else {
    encoderRightPulses++;
  }
  //Serial.println("X");
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
  encoderLeftDirection = false; 
  pinMode(ENCODER_LEFT_PINA,INPUT);
  pinMode(ENCODER_LEFT_PINB,INPUT);
  attachInterrupt(ENCODER_LEFT_PINB, ENCODER_LEFT_FUNCTION, ENCODER_LEFT_SIGNAL);
  encoderRightDirection = true;
  pinMode(ENCODER_RIGHT_PINA,INPUT);
  pinMode(ENCODER_RIGHT_PINB,INPUT);
  attachInterrupt(ENCODER_RIGHT_PINB, ENCODER_RIGHT_FUNCTION, ENCODER_RIGHT_SIGNAL);

  // i2c slave
  Wire.begin(I2C_NODE);                // join i2c bus with address #8
  Wire.onRequest(requestEvent); // register event
  Wire.onReceive(receiveEvent); // register event

  // set motors forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN3, HIGH);
  motorTimer = millis() + MOTOR_TIME;

  // lidar Power
  pinMode(LIDAR_MOTOR_PIN, OUTPUT);
  digitalWrite(LIDAR_MOTOR_PIN, lidarMotorPower);
  //lidarMotorPowerTimer = millis();
  
  // configure LED pin for output
  pinMode(LED_PIN, OUTPUT);
  ledTimer = millis() + LED_TIME;

}

void loop() {
  
  /*
  // motor speed change
  if(millis() > motorTimer) {
    motorTimer = millis() + MOTOR_TIME;
    if(motorPwmUp) {
      if(motorPwm < 64535) {
        motorPwm += 1000;
      } else {
        motorPwmUp = false;
      }
    } else {
      if(motorPwm > 1000) {
        motorPwm -= 1000;
      } else {
        motorPwmUp = true;
      }
    }    
    pwmWrite(ENA, motorPwm);
    pwmWrite(ENB, motorPwm);
  }
  */

  // debug
  //*
  static double encoderLeftPulsesLast = 999;
  static double encoderRightPulsesLast = 999;
  
  if(encoderLeftPulses != encoderLeftPulsesLast || encoderRightPulses != encoderRightPulsesLast) {
    if(encoderLeftPulses != encoderLeftPulsesLast) encoderLeftPulsesLast = encoderLeftPulses;
    if(encoderRightPulses != encoderRightPulsesLast) encoderRightPulsesLast = encoderRightPulses;
    Serial.print(leftMotorPwmOut); Serial.print("\t"); Serial.print(rightMotorPwmOut); Serial.print("\t"); Serial.print(encoderLeftPulses); Serial.print("\t"); Serial.print(encoderRightPulses); Serial.println("\t");
  }

  bodyMotorsControl();

/*
  if(millis() >= (lidarMotorPowerTimer + 20000)){
    lidarMotorPowerTimer = millis();
    digitalWrite(LIDAR_MOTOR_PIN, lidarMotorPower);
    if (lidarMotorPower == 0) lidarMotorPower = 1; else lidarMotorPower = 0;
  }
*/

  // activity led
  if(millis() > ledTimer) {
    ledTimer = millis() + LED_TIME;
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);    
  }
}

void requestEvent() {
  union u_tag {
    byte b[4];
    signed int i;
  } i2c_data[32];
  
  i2c_data[0].i = 0;
  i2c_data[1].i = encoderLeftPulses;
  i2c_data[2].i = encoderRightPulses;
  
  // debug
  /*
  i2c_data[0].i = 0;
  i2c_data[1].i = 256;
  i2c_data[2].i = -256;
  */

  Wire.write((byte*)&i2c_data, 12);
}

void receiveEvent(int howMany){
  union u_tag {
    byte b[4];
    signed int i;
  } i2c_data[32];

  //union u_tag cmd_data[32];
  
  // set receive buffer elements to zero
  for(int f = 0; f < 4; f++) i2c_data[f].i = 0;
  //for(int f = 0; f < 4; f++) cmd_data[f].i = 0;

  // collect incoming data
  byte *ptr = (byte*)&i2c_data;
  while(Wire.available()) {
    byte b = Wire.read();
    *ptr = b;
    ptr++;
  }

  //leftMotorPwmOut = i2c_data[1].i;
  //rightMotorPwmOut = i2c_data[2].i;

  switch(i2c_data[0].b[0]) {
    case 1:
      // reset encoders
      encoderLeftPulses = 0;
      encoderRightPulses = 0;
      break;
    case 2:
      // set pwm
      leftMotorPwmOut = i2c_data[1].i;
      rightMotorPwmOut = i2c_data[2].i;
      break;
    case 3:
      // lidar motor on
      lidarMotorPower = HIGH;
      digitalWrite(LIDAR_MOTOR_PIN, lidarMotorPower);
      break;
    case 4:
      // lidar motor off
      lidarMotorPower = LOW;
      digitalWrite(LIDAR_MOTOR_PIN, lidarMotorPower);
      break;     
  }
}

//
// motor control
//

void bodyMotorsControl() {  
  // set motor left direction & speed
  if(leftMotorPwmOut == 0) {
    setBodyMotorLeftStop();
    pwmWrite(leftMotorEn, leftMotorPwmOut);
  } else if(leftMotorPwmOut > 0) {
    setBodyMotorLeftForward();
    pwmWrite(leftMotorEn, leftMotorPwmOut);
  } else {
    setBodyMotorLeftBackward();
    pwmWrite(leftMotorEn, abs(leftMotorPwmOut));    
  }
  
  // set motor right direction & speed
  if(rightMotorPwmOut == 0) {
    setBodyMotorRightStop();
    pwmWrite(rightMotorEn, rightMotorPwmOut);
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
