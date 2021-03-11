// units are m, m/s, radian/s
extern double wheel_encoder_pulses;
extern double wheel_rad;
extern double wheel_sep;

// stored for fast calculation
extern double wheel_per;
extern double distancePerPulse;
extern double pulses_per_m;

// encoder variables
extern long encoderLeftPulses;
extern long encoderRightPulses;

extern long encoderLeftPulsesTarget;
extern long encoderRightPulsesTarget;
extern long encoderLeftPulsesTargetStart;
extern long encoderRightPulsesTargetStart;

extern bool encoderLeftPulsesOnTarget;
extern bool encoderRightPulsesOnTarget;
extern bool encoderPulsesTargetEnabled;

extern int encoderLeftTargetDirection;
extern int encoderRightTargetDirection;

extern long encoderLeftPulsesLast;
extern long encoderRightPulsesLast;

// odometry
extern double bodyX;
extern double bodyY;
extern double bodyTheta;

// velocity
extern double odomXvel;
extern double odomZvel;

// debug
extern bool displayDebugOdom;

// debug
extern int mcuSteeringPidError;
extern int mcuLeftPidError;
extern int mcuRightPidError;

//extern int leftTmpPulses; // last displacement = (encoderLeftPulses - encoderLeftPulsesTargetStart);
//extern int rightTmpPulses // last displacemmen = (encoderRightPulses - encoderRightPulsesTargetStart);

