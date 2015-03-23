/*
$URL: svn+ssh://aaron@birenboim.com/home/svn/arduino/sketchbook/DalekDrive/MotorDriveC.h $
$Id: MotorDriveC.h 143 2015-03-23 00:24:53Z aaron $

Motor driver for Vicky Tan Motor controler (aliexpress).
This controller has a direction input, and PWM speed control logic inputs,
for each of two motors.

The PWM speed cannot be set to 255 (full-time on).  They recommend
a 99% max PWM rate, perhaps 252 on arduino analogWrite 8-bit PWM.

I think it tends to brake when set to 0 speed, which might
be a good thing for a simple robot (like the Dalek electric wheelchair base)
where we don't want to coast.

H-bridges on left and right rear wheel motor drives.
*/

#define MAX_PWM   252    // don't go over this PWM level.  driver can't do it
#define MIN_PWM   20     // Motors won't move below this level
#define STARTUP_TIME  50 // when starting from brake, go to MAX_PWM for this many ms
#define DEADMAN_TIME 250 // transition to emergency stop if no command update in this time interval

// Controls how long to wait for motor to slow down/stop
// before trying to change direction.
// larger value==more delay
#define EMERGENCY_STOP_TIME 2000  // time after emergency stop before can drive again

// Don't allow immediate change of direction.
// let it "brake" for a little bit before starting in other direction
#define MOTOR_READY 0
#define MOTOR_FWD   1
#define MOTOR_REV   2
#define MOTOR_STOP  3
#define MOTOR_START_FWD 5
#define MOTOR_START_REV 6

#ifndef ABS
#define ABS(x)  (((x)<0)?(-(x)):(x))
#endif

// ----------- Global state classes:

class MotorDrive 
{
protected:
  inline static int getPWM(int pwm)
  {
    pwm = ABS(pwm);
    if (pwm > MAX_PWM) pwm = MAX_PWM;
    if (pwm < MIN_PWM) pwm = 0;
    return(pwm);
  }
  
public:
  struct {
    int REV;  // reverse direction indicator, 1==Reverse
    int PWM;  // Enable/speed pin. 0..MAX_PWM
  } Pin;
  int speed;  // current speed
  float decel; // time to allow to stop, in ms / PWM count
  byte mode;
  unsigned long modeDoneTime;

  void stop()
  {
    digitalWrite(Pin.PWM , 0);
    modeDoneTime = millis() + (long)(speed * decel);
    speed=0;
    mode = MOTOR_STOP;
  }

  void emergencyStop()
  {
    stop();
    modeDoneTime += EMERGENCY_STOP_TIME;
  }

  void begin(const int rev, const int pwm, const float de=4.0)
  {
    Pin.REV=rev;
    Pin.PWM=pwm;
    decel=de;

    pinMode(rev,OUTPUT);
    pinMode(pwm,OUTPUT);

    emergencyStop();
  }

  // Set speed -MAX_PWM for max reverse, MAX_PWM for max forward
  void setSpeed(const int spdReq)
  {
    long t;

    t = millis();
    switch(mode)
      {
      case MOTOR_STOP :
        if (t < modeDoneTime) return;
	if (ABS(spdReq) < MIN_PWM) return;
	mode = (spdReq < 0) ? MOTOR_START_REV : MOTOR_START_FWD;
	digitalWrite(Pin.REV, (mode == MOTOR_START_REV) ? HIGH : LOW);
	analogWrite(Pin.PWM, MAX_PWM);
	modeDoneTime = t + STARTUP_TIME;
	speed = spdReq;
      case MOTOR_FWD :
      case MOTOR_REV :
	if (t > modeDoneTime) { emergencyStop(); return; } // deadman expired
	if ( (ABS(spdReq) < MIN_PWM)  ||  // stop or change direction
	     ((spdReq < 0) && (mode == MOTOR_FWD)) ||
	     ((spdReq > 0) && (mode == MOTOR_REV)) )
	  {
	    stop();
	    return;
	  }
	speed = spdReq;
	analogWrite(Pin.PWM,getPWM(speed));
	modeDoneTime = t + DEADMAN_TIME;
	return;
      case MOTOR_START_REV :
      case MOTOR_START_FWD :
	if ( (ABS(spdReq) < MIN_PWM) ||
	     ((spdReq < 0) && (mode == MOTOR_START_FWD)) ||
	     ((spdReq > 0) && (mode == MOTOR_START_REV)) )
	  {
	    stop();
	    return;
	  }
	// same direction, but speed request change
	speed = spdReq;
	if (t >= modeDoneTime)
	  {
	    mode = (speed > 0) ? MOTOR_FWD : MOTOR_REV;
	    modeDoneTime = t + DEADMAN_TIME;
	    setSpeed(speed);
	  }
	return;
      }
  }

  // update state, but no new command (no deadman reset)
  // Checks if previous command is complete, and an automatic state transition
  // is needed
  void update(long t)  // current time, from millis()
  {
    switch(mode)
      {
      case MOTOR_STOP : return;
      case MOTOR_FWD :
      case MOTOR_REV :
	if (t > modeDoneTime) emergencyStop(); // deadman expired
	return;
      case MOTOR_START_REV :
      case MOTOR_START_FWD :
	if (t > modeDoneTime) setSpeed(speed);
	return;
      }
  }

} MotR, MotL;
