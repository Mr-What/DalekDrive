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
#define MIN_PWM   15     // Motors won't move below this level
#define STARTUP_TIME  50 // when starting from brake, go to MAX_PWM for this many ms
#define DEADMAN_TIME 250//250 // transition to emergency stop if no command update in this time interval

// Controls how long to wait for motor to slow down/stop
// before trying to change direction.
// larger value==more delay
#define EMERGENCY_STOP_TIME 3000  // time after emergency stop before can drive again

// Don't allow immediate change of direction.
// let it "brake" for a little bit before starting in other direction
#define MOTOR_STOP  0
#define MOTOR_2STOP 1     // 1's is transition state indicator bit
#define MOTOR_FWD   4
#define MOTOR_REV   8
#define MOTOR_2FWD  5
#define MOTOR_2REV  9

#ifndef ABS
#define ABS(x)  (((x)<0)?(-(x)):(x))
#endif

// ----------- Global state classes:

class MotorDrive 
{
protected:
  inline int getPWM(int pwm)
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
    analogWrite(Pin.PWM , 0);
    int stoppingTime = (int)ABS(speed * decel);
Serial.print(stoppingTime);Serial.println(" ms to stop.");
    modeDoneTime = millis() + stoppingTime;
    //speed=0;  don't clobber command in case of direction change
    mode = MOTOR_2STOP;
  }

  void emergencyStop()
  {
    Serial.print("Emergency ");
    stop();
    speed=0;
    modeDoneTime += EMERGENCY_STOP_TIME;
  }

  void begin(const int rev, const int pwm, const float de=4.0)
  {
    Pin.REV=rev;
    Pin.PWM=pwm;
    decel=de;

    pinMode(rev,OUTPUT);
    pinMode(pwm,OUTPUT);

    analogWrite(Pin.PWM,0);
    digitalWrite(Pin.REV,0);
    
    emergencyStop();
  }

  // Set speed -MAX_PWM for max reverse, MAX_PWM for max forward
  void setSpeed(const int spdReq, long t)
  {
    byte prevMode = mode;
    switch(prevMode)
      {
      case MOTOR_2STOP :
        if (t < modeDoneTime)
          {  // make sure things are stopped
            analogWrite(Pin.PWM,0);
            return;
          }
        // done stoping, continue to STOP mode
        mode = MOTOR_STOP;
Serial.println(F("stopped."));
      case MOTOR_STOP :
	if (ABS(spdReq) < MIN_PWM) return;
	mode = (spdReq < 0) ? MOTOR_2REV : MOTOR_2FWD;
	digitalWrite(Pin.REV, (mode == MOTOR_2REV) ? HIGH : LOW);
	analogWrite(Pin.PWM, MAX_PWM); // hard kick to get started
	modeDoneTime = t + STARTUP_TIME;
	speed = spdReq;
Serial.print(F("Start "));
Serial.println((mode == MOTOR_2REV) ? F("REV") : F("FWD"));
        return;
      case MOTOR_FWD :
      case MOTOR_REV :
        digitalWrite(Pin.REV,(prevMode == MOTOR_REV) ? HIGH : LOW);
	if (t > modeDoneTime) { emergencyStop(); return; } // deadman expired
	if ( (ABS(spdReq) < MIN_PWM)  ||  // stop or change direction
	     ((spdReq < 0) && (prevMode == MOTOR_FWD)) ||
	     ((spdReq > 0) && (prevMode == MOTOR_REV)) )
	  {
	    stop();
            // set speed so that it goes to this speed after coast-down
            speed =  (ABS(spdReq) < MIN_PWM) ? 0 : spdReq;
	    return;
	  }
	speed = spdReq;
	analogWrite(Pin.PWM,getPWM(speed));
	modeDoneTime = t + DEADMAN_TIME;
Serial.println(speed);
	return;
      case MOTOR_2REV :
      case MOTOR_2FWD :
	if (ABS(spdReq) < MIN_PWM)
          {
            speed = 100;  // give it some time to decel, although just starting
            stop();
            speed = 0;
            return;
          }
	if ( ((spdReq < 0) && (mode == MOTOR_2FWD)) ||
	     ((spdReq > 0) && (mode == MOTOR_2REV)) )
	  { // direction change
            speed = 100;  // give it some time to decel, although just starting
	    stop();
            speed = spdReq;  // go to this speed after coast-down period
	    return;
	  }
	// same direction, but speed request change
	speed = spdReq;
	if (t >= modeDoneTime)
	  {
	    mode = (speed > 0) ? MOTOR_FWD : MOTOR_REV;
	    modeDoneTime = t + DEADMAN_TIME;
            digitalWrite(Pin.REV,(mode == MOTOR_REV)?1:0);  // make sure DIR pin is correct
            analogWrite(Pin.PWM,getPWM(speed));
Serial.println("Started");
	  }
	return;
      }
  }

  // Set speed -MAX_PWM for max reverse, MAX_PWM for max forward
  inline void setSpeed(const int spdReq)
  {
    setSpeed(spdReq,millis());
  }
  
  // update state, but no new command (no deadman reset)
  // Checks if previous command is complete, and an automatic state transition
  // is needed
  void update(long t)  // current time, from millis()
  {
//Serial.print(F("Update "));  Serial.println(t);
    if ((modeDoneTime > 0xfffff000) && (t < 999))
      {  // time counter must have wrapped around
        modeDoneTime = 0;
        Serial.println("Clock wrap-around");
      }

    byte prevMode = mode;
    switch(prevMode)
      {
      case MOTOR_2STOP : 
      case MOTOR_STOP :
        if ((t > modeDoneTime) && speed)
          { // this was a temp stop in a direction change.  Command desired speed.
Serial.print(F("Restart "));Serial.println(speed);
            setSpeed(speed,t);
          }
//else Serial.println("stopped.");
        return;
      case MOTOR_FWD :
      case MOTOR_REV :
	if (t > modeDoneTime) emergencyStop(); // deadman expired
	return;
      case MOTOR_2REV :
      case MOTOR_2FWD :
	if (t > modeDoneTime)
          {
            //mode = (prevMode == MOTOR_2REV) ? MOTOR_REV : MOTOR_FWD;
Serial.println(F("moving"));
            setSpeed(speed,t);
          }
	return;
      }
  }

};

