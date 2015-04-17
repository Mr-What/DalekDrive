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
    if (pwm > _maxPWM) pwm = _maxPWM;
    if (pwm < _minPWM) pwm = 0;
    return(pwm);
  }
  
public:
  struct {
    int REV;  // reverse direction indicator, 1==Reverse
    int PWM;  // Enable/speed pin. 0..MAX_PWM
  } Pin;
  int _speed;  // current speed
  float _decel; // time to allow to stop, in ms / PWM count
  byte _mode;
  unsigned long _modeDoneTime;
  int _deadTime, _minPWM, _maxPWM, _startupTime, _stopTime;
  int _msgCount;
  
  MotorDrive(const int deadTime=250,
             const int startupTime=5,
             const int stopTime=3000,
             const int minPWM=9,
             const int maxPWM=252)
  {
    _deadTime = deadTime; // emergency stop if no command update in this time interval
    _minPWM = minPWM;     // Motors won't move below this level
    _maxPWM = maxPWM;     // don't go over this PWM level.  driver can't do it
    _startupTime = startupTime; // when starting from still, issue full power pulse this long to get motors started
    _stopTime = stopTime;  // Pause at least this long after emergency stop before restarting
    _msgCount = 11;  // issue this many diagnostic messages before going quiet
  }

  void stop()
  {
    analogWrite(Pin.PWM , 0);
    int stoppingTime = (int)ABS(_speed * _decel);
if(_msgCount>0){_msgCount--;Serial.print(stoppingTime);Serial.println(" ms to stop.");}
    _modeDoneTime = millis() + stoppingTime;
    //speed=0;  don't clobber command in case of direction change
    _mode = MOTOR_2STOP;
  }

  void emergencyStop()
  {
    Serial.print("Emergency ");
    _msgCount = 11;  // turn on diagnostics for a few commands
    stop();
    _speed=0;
    _modeDoneTime += _stopTime;
  }

  void begin(const int rev, const int pwm, const float de=4.0)
  {
    Pin.REV=rev;
    Pin.PWM=pwm;
    _decel=de;

    pinMode(rev,OUTPUT);
    pinMode(pwm,OUTPUT);

    analogWrite(Pin.PWM,0);
    digitalWrite(Pin.REV,0);
    
    emergencyStop();
  }

  // Set speed -MAX_PWM for max reverse, MAX_PWM for max forward
  void setSpeed(const int spdReq, long t)
  {
    byte prevMode = _mode;
    switch(prevMode)
      {
      case MOTOR_2STOP :
        if (t < _modeDoneTime)
          {  // make sure things are stopped
            analogWrite(Pin.PWM,0);
            return;
          }
        // done stoping, continue to STOP mode
        _mode = MOTOR_STOP;
if(_msgCount>0){_msgCount--;Serial.println(F("stopped."));}
      case MOTOR_STOP :
	if (ABS(spdReq) < _minPWM) return;
	_mode = (spdReq < 0) ? MOTOR_2REV : MOTOR_2FWD;
	digitalWrite(Pin.REV, (_mode == MOTOR_2REV) ? HIGH : LOW);
	analogWrite(Pin.PWM, _maxPWM); // hard kick to get started
	_modeDoneTime = t + _startupTime;
	_speed = spdReq;
if(_msgCount>0){_msgCount--;
Serial.print(F("Start "));
Serial.println((_mode == MOTOR_2REV) ? F("REV") : F("FWD"));}
        return;
      case MOTOR_FWD :
      case MOTOR_REV :
        digitalWrite(Pin.REV,(prevMode == MOTOR_REV) ? HIGH : LOW);
	if (t > _modeDoneTime) { emergencyStop(); return; } // deadman expired
	if ( (ABS(spdReq) < _minPWM)  ||  // stop or change direction
	     ((spdReq < 0) && (prevMode == MOTOR_FWD)) ||
	     ((spdReq > 0) && (prevMode == MOTOR_REV)) )
	  {
	    stop();
            // set speed so that it goes to this speed after coast-down
            _speed = (ABS(spdReq) < _minPWM) ? 0 : spdReq;
	    return;
	  }
	_speed = spdReq;
	analogWrite(Pin.PWM,getPWM(_speed));
	_modeDoneTime = t + _deadTime;
//if(_msgCount>0){_msgCount--;Serial.println(_speed);}
	return;
      case MOTOR_2REV :
      case MOTOR_2FWD :
	if (ABS(spdReq) < _minPWM)
          {
            _speed = 100;  // give it some time to decel, although just starting
            stop();
            _speed = 0;
            return;
          }
	if ( ((spdReq < 0) && (_mode == MOTOR_2FWD)) ||
	     ((spdReq > 0) && (_mode == MOTOR_2REV)) )
	  { // direction change
            _speed = 100;  // give it some time to decel, although just starting
	    stop();
            _speed = spdReq;  // go to this speed after coast-down period
	    return;
	  }
	// same direction, but speed request change
	_speed = spdReq;
	if (t >= _modeDoneTime)
	  {
	    _mode = (_speed > 0) ? MOTOR_FWD : MOTOR_REV;
	    _modeDoneTime = t + _deadTime;
            digitalWrite(Pin.REV,(_mode == MOTOR_REV)?1:0);  // make sure DIR pin is correct
            analogWrite(Pin.PWM,getPWM(_speed));
if(_msgCount>0){_msgCount--;Serial.println("Started");}
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
    if ((_modeDoneTime > 0xfffff000) && (t < 999))
      {  // time counter must have wrapped around
        _modeDoneTime = 0;
        Serial.println("Clock wrap-around");
      }

    byte prevMode = _mode;
    switch(prevMode)
      {
      case MOTOR_2STOP : 
      case MOTOR_STOP :
        if ((t > _modeDoneTime) && _speed)
          { // this was a temp stop in a direction change.  Command desired speed.
if(_msgCount>0){_msgCount--;Serial.print(F("Restart "));Serial.println(_speed);}
            setSpeed(_speed,t);
          }
//else Serial.println("stopped.");
        return;
      case MOTOR_FWD :
      case MOTOR_REV :
	if (t > _modeDoneTime) emergencyStop(); // deadman expired
	return;
      case MOTOR_2REV :
      case MOTOR_2FWD :
	if (t > _modeDoneTime)
          {
            //mode = (prevMode == MOTOR_2REV) ? MOTOR_REV : MOTOR_FWD;
if(_msgCount>0){_msgCount--;Serial.println(F("moving"));}
            setSpeed(_speed,t);
          }
	return;
      }
  }

};

