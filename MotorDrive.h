/*
$URL: svn+ssh://aaron@birenboim.com/home/svn/arduino/sketchbook/DalekDrive/MotorDrive.h $
$Id: MotorDrive.h 130 2013-01-17 02:51:03Z aaron $

Motor driver for Dalek based on electric wheelchair base.

H-bridges on left and right rear wheel motor drives.

These are novel H-bridges with a DIR Reverse indicator
(hi for reverse), then a PWM pin for speed.
The PWM speed
     (0=stop, 255=max forward) when in forward mode,
but inverted in reverse mode:
     (255=stop, 0=max reverse) in reverse mode.

See setup() funtion for pin assignments.

*/

// Controls how long to wait for motor to slow down/stop
// before trying to change direction.
// larger value==more delay
#define EMERGENCY_STOP_TIME 2000  // time after emergency stop before can drive again

#define MOTOR_READY 0
#define MOTOR_COAST 1
#define MOTOR_STOP  2
#define MOTOR_EMERGENCY_STOP 3

// ----------- Global state classes:

class MotorDrive 
{
public:
  struct {
    int REV;  // reverse direction indicator, 1==Reverse
    int EN;   // Enable/speed pin. 255 is fast forward, but stopped reverse
  } Pin;
  int speed;  // current speed
  float decel;  // deceleration rate for pause
  byte mode;
  unsigned long modeStartTime;

  void emergencyStop()
  {
    digitalWrite(Pin.REV,LOW);
    digitalWrite(Pin.EN ,LOW);
    mode = MOTOR_EMERGENCY_STOP;
    speed=0;
    modeStartTime = millis();
  }

  void begin(const int rev, const int en, const float de=4.0)
  {
    Pin.REV=rev;
    Pin.EN=en;
    decel=de;

    pinMode(rev,OUTPUT);
    pinMode( en,OUTPUT);

    emergencyStop();
    mode = MOTOR_READY;
  }

  // Set speed -255 for max reverse, 255 for max forward
  void setSpeed(const int spdReq)
  {
    if (mode == MOTOR_EMERGENCY_STOP)
      {
        if (millis() - modeStartTime < EMERGENCY_STOP_TIME)
          return;
        else mode = MOTOR_READY;
      }
    int spd = spdReq;
    if (spd < -255) spd = -255;
    if (spd >  255) spd =  255;

    if ((long)spd * (long)speed > 0)
      { // no change in direction.  just change EN value
        if (spd > 0)
          {
            if (spd ==  255) digitalWrite(Pin.EN,HIGH);
            else              analogWrite(Pin.EN,spd);
          }
        else
          {
            if (spd == -255) digitalWrite(Pin.EN,LOW);
            else              analogWrite(Pin.EN,255+spd);
          }
        speed = spd;
        //Serial.print(F("\t\tsame direction\t"));Serial.println(speed);
        return;
      }

    // check current REV pin state
    int revState = digitalRead(Pin.REV);

    // Weather we are stopped, or have a change in direction,
    // shut motor down first.
    if (revState)  digitalWrite(Pin.EN,HIGH); // HIGH to stop, rev mode
    else           digitalWrite(Pin.EN, LOW); //  LOW to stop, forward mode
    mode = MOTOR_COAST;

    int prevVel = (speed > 0) ? speed : -speed;
    if (prevVel>10)
      { // kind of fast.
        // wait a while to slow down before allowing more commands
        Serial.println(F("\t\tDirection change... wait a bit"));
        delay(prevVel*decel);
        // ideally, return here, and resume at next command for coast
      }

    if (spd != 0)
      { // should do these simultaneous, but I think it
        // is safe to allow sub-microseconds of reverse direction drive
        if (spd < 0)
          {
            analogWrite(Pin.EN,spd+255);
            digitalWrite(Pin.REV,HIGH);
          }
        else
          {
            analogWrite(Pin.EN,spd);
            digitalWrite(Pin.REV,LOW);
          }
        mode = MOTOR_READY;
      }
    else mode = MOTOR_STOP;
    speed = spd;
    //Serial.println(speed);
    return;
  }
} MotR, MotL;
