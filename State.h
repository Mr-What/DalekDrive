/*
$URL: svn+ssh://aaron@birenboim.com/home/svn/arduino/sketchbook/DalekDrive/State.h $
$Id: State.h 142 2015-03-23 00:16:34Z aaron $

Main Dalek state control
*/

#include "Command.h"

#define MODE_SLAVE 0
#define MODE_MANUAL 1
#define MODE_AUTONOMOUS 2
#define MODE_EMERGENCY_STOP 0x80

#define COMMAND_TIMEOUT 250   // allowable ms without a command before emergency shutdown

//---------------- These are the persistent parameters, saved to EEPROM :
const char *VersID = "Dlk1"; // CHANGE THIS whenever the below struct changes!!!
typedef struct {
  char ID[4];  // version and ID code
  int SonarPeriod, CommandTimeout;  // as in State
  AutonomousModeParameters ap;
} PersistentState;
bool isValid(const PersistentState &s);


// convert analog speed reading into a motor speed command
int getAnalogSpeedCommand(const int pin)
{
  // returns analog value from 0..1023.
  // use low values for reverse, dead zone in middle (stop)
  // high end for range for forward speed
  int val = analogRead(pin);
  //Serial.print(val);

#if 1
  // let min non-zero command be around 40ish
  if (val < 400)
    { // reverse
      val = ((long)((val - 500))*514)/1000;
      if (val < -255) val = -255;
      return val;
    }
  else if (val > 500)
    { // forward
      val = (((long)(val-400))*412)/1000;
      if (val > 255) val = 255;
      return val;
    }
  return 0;


#else
  // let speed commands be exponential, for fine control at low speed

  float vel;
  if (val < 405)
    {
      // let mapped value go a little outside command range
      // to allow for a little dead zone around "pegged" area.

      vel = 404-val;  // reverse velocity
      if (vel < 2)
        {
          Serial.println("\t0 .. very slow, rev");
          return(0);  // close enough to stop.  just stop
        }
      vel = exp(vel/200) / 7.4;  // normalize to 0..1, exp curve,  log(7.4)=2
      val = -255*vel;  // speed command
      //Serial.print('\t');Serial.print(val);
      if (val > -3 ) val=0; // don't do very slow speed.  just heats coils
      if (val < -255) val=-255;
      Serial.print('\t');Serial.println(val);
      return(val);
    }

  // let stop dead zone be from around 405 to around 505
  val -= 500;
  //Serial.print("\t");Serial.print(val);

  // low speed commands will just heat motor.
  // just return 0 for very slow commands
  if (val < 6)
    {
      Serial.println("\t0 .. very low");
      return(0);
    }

  // normalize exp scale.  Note val is around 1..523.
  // if range to exp is 0..2, note exp(2)=7.4 for normalization
  vel = exp((float)val/260) / 7.4;
  val = vel * 255;
  //Serial.print("\t");Serial.print(val);
  if (val < 4) val=0;  // don't use very slow speeds.  just heat motor
  if (val > 255) val=255;
  Serial.print("\t");Serial.println(val);
  return(val);
#endif
}

bool updatePrint(const int cmd, int &prev)
{
  bool doit = false;
  if ((cmd == 0) || (cmd == -255) || (cmd == 255))
    {
      doit = prev != cmd;
      prev=cmd;
      return(doit);
    }
  int d = cmd - prev;
  if (d < 0) d = -d;
  doit = (d > 6);  // only print significant moves
  if (doit)
    {
      prev += cmd;
      prev /= 2;   // move half-way to actual command so prev tracks actual command
    }
  return doit;
}

void printUsage();

//-----------------------------------------------------------

class DalekDriveState
{
public:
  struct {
    byte manualOverride;  // Deadman to enable manual override commands
    byte autonomous;
    byte speedR, speedL;  // pots to manually control motor speed
  } Pin;
  unsigned long prevCmdTime;  // time at previous speed command
  unsigned long cmdTimeout;  // ms to wait with no command before emergency shutdown
  volatile byte mode;
  int prevR, prevL;  // for computing change in speed command
  int SonarPeriod;  // ms

  void begin(const byte pinAutonomousDeadman,
             const byte pinManualOverride,
             const byte pinPotL, const byte pinPotR)
  {
    Pin.autonomous = pinAutonomousDeadman;
    Pin.manualOverride = pinManualOverride;
    Pin.speedL = pinPotL;
    Pin.speedR = pinPotR;
    prevCmdTime = 0;
    cmdTimeout = COMMAND_TIMEOUT;
    SonarPeriod = 2000; // ms
    prevR = prevL = 0;

    pinMode(Pin.manualOverride, INPUT);
    digitalWrite(Pin.manualOverride,HIGH);  // enable pull-up on deadman Manual override pin
    pinMode(Pin.autonomous, INPUT);
    digitalWrite(Pin.autonomous,HIGH);  // enable pull-up on autonomous deadman pin
    Autonomous.begin();  // initialize autonomous mode state
    Command.begin();  // initialize command reader
  }

  void ImmediateStop()
  {
    MotR.speed = (MotR.speed<0)?-1:1; // skip decel period, if any
    MotL.speed = (MotL.speed<0)?-1:1; // skip decel period, if any
    MotR.setSpeed(0);
    MotL.setSpeed(0);
    //Serial.println(F("STOP"));
    mode = MODE_SLAVE;
  }

  void updateManualOverride()
  { // manual-override mode, get commands from pots
    int xR = getAnalogSpeedCommand(Pin.speedR);
    int xL = getAnalogSpeedCommand(Pin.speedL);
    if (updatePrint(xR,prevR) || updatePrint(xL,prevL))
      {Serial.print(xL);Serial.print('\t');Serial.println(xR);}
    MotR.setSpeed(xR);
    MotL.setSpeed(xL);
  }


  void printUsage() const {
    Serial.println(F("?) print this message\n"
                     "!) Save current operational parameters\n"
                     "^) Restore previously saved parameters"));
    Serial.print(F("L) left motor speed (-255..255)\nR) right motor speed\n"
                   "p) "));
    Serial.print(SonarPeriod);
    Serial.print(F(" sonar Period (ms)\nt) "));
    Serial.print(cmdTimeout);
    Serial.print(F(" command Timeout (ms)\nm) "));
    Serial.print(Autonomous.prm.minCmd);
    Serial.print(F(" minimum motor command (3..100)\nC) "));
    Serial.print(Autonomous.prm.tooClose);
    Serial.print(F(" too Close range (cm)\nc) "));
    Serial.print(Autonomous.prm.close);
    Serial.print(F(" close range (cm)\nS) "));
    Serial.print((int)(Autonomous.prm.maxSpeed*100));
    Serial.print(F(" max Speed\t(% of full)\nT) "));
    Serial.print((int)(Autonomous.prm.maxTurn*100));
    Serial.print(F(" max Turn rate\t(% of full)\nG) "));
    Serial.print((int)(Autonomous.prm.increaseSpeed*100));Serial.print(F(" ("));
    Serial.print(      Autonomous.prm.increaseSpeed);
    Serial.print(F(") speed increase Gain\ng) "));
    Serial.print((int)(Autonomous.prm.decreaseSpeed*100));Serial.print(F(" ("));
    Serial.print(      Autonomous.prm.decreaseSpeed);
    Serial.print(F(") speed decrease gain\nr) "));
    Serial.print((int)(Autonomous.prm.decreaseTurn*100));Serial.print(F(" ("));
    Serial.print(      Autonomous.prm.decreaseTurn);
    Serial.print(F(") turn rate decrease gain\nd) "));
    Serial.print((int)(Autonomous.prm.deltaTurn*100));
    Serial.println(F(" turn change delta (% of full)"));
  }

  void checkCommand(unsigned long t)
  {
    int val;
    char code;
    if (!Command.get(code,val)) return;
    Serial.print('*');Serial.print(code);Serial.println(val);

    if (mode & MODE_MANUAL)
      {
        if ((code=='L') || (code=='R'))
          {
            Serial.println(F("In Manual control mode: Motor speed command ignored."));
            return;
          }
      }
    switch(code)
      {
      case 'L': MotL.setSpeed(val); break;
      case 'R': MotR.setSpeed(val); break;
      case 'A': mode |= MODE_AUTONOMOUS; Serial.println(F("Autonomous Mode")); break;
      case 'a': mode &= (~MODE_AUTONOMOUS); Serial.println(F("Manual Override Enabled")); break;
      case '!': save(); break;
      case '^': load(); break;
      case 'p': SonarPeriod=val; break;
      case 't': cmdTimeout=val;  break;
      case 'm': Autonomous.prm.minCmd=val; break;
      case 'C': Autonomous.prm.tooClose=val; break;
      case 'c': Autonomous.prm.close=val; break;
      case 'S': Autonomous.prm.maxSpeed=val*0.01; break;
      case 'T': Autonomous.prm.maxTurn=val*0.01; break;
      case 'G': Autonomous.prm.increaseSpeed=val*0.01; break;
      case 'g': Autonomous.prm.decreaseSpeed=val*0.01; break;
      case 'r': Autonomous.prm.decreaseTurn=val*0.01; break;
      case 'd': Autonomous.prm.deltaTurn=val*0.01; break;
      case '?': printUsage(); break;
      default:
        Serial.print(F("Unrecognized command : "));
        Serial.print(code);
        //Serial.print("\t");
        Serial.println(val);
      }
    prevCmdTime = t;
  }

  int save() // save some parameters to EEPROM
  {
    PersistentState s;
    int i;

    s.SonarPeriod = SonarPeriod;
    s.CommandTimeout = cmdTimeout;
    for(i=0; i < 4; i++) s.ID[i] = VersID[i];
    s.ap = Autonomous.prm;
    if (!isValid(s)) return(1);
    int n = sizeof(s);
    byte *b = (byte *)(&s);
    for (i=0; i<n; i++, b++) EEPROM.write(i,*b);
    return(0);
  }

  void set(const PersistentState &s)  // set state from a struct
  {
    SonarPeriod = s.SonarPeriod;
    cmdTimeout = s.CommandTimeout;
    Autonomous.prm = s.ap;
  }

  // load (some) parameters from EEPROM
  // returns 0 or error code
  //    Call after an initial State.begin(), which sets defaults
  //    this will override defaults if a valid state was saved
  int load()  // load state parameters from EEPROM
  {
    PersistentState s;
    int n = sizeof(s);
    byte *b = (byte *)(&s);
    for (int i=0; i<n; i++, b++) *b = EEPROM.read(i);
    if (isValid(s))
      {
        set(s);
        return(0);
      }
    return(1);
  }

} State;

// check if contents of a PersistentState struct are reasonable
bool isValid(const PersistentState &s)
{
  for (int i=0; i < 4; i++) if (s.ID[i] != VersID[i]) return(false);
  if (s.SonarPeriod < 20) return(false); // too fast for sonar
  if (s.SonarPeriod > 32000) return(false);
  if (s.CommandTimeout < 20) return(false);
  if (s.CommandTimeout > 10000) return(false);
  if (s.ap.minCmd < 3) return(false);
  if (s.ap.minCmd > 100) return(false);
  if (s.ap.tooClose < 10) return(false);
  if (s.ap.tooClose > 150) return(false);
  if (s.ap.close <= s.ap.tooClose) return(false);
  if (s.ap.close > 200) return(false);
  if ((s.ap.maxSpeed > 1.0) || (s.ap.maxSpeed < 0.1)) return(false);
  if ((s.ap.maxTurn  > 1.0) || (s.ap.maxTurn  < 0.1)) return(false);
  if ((s.ap.increaseSpeed <= 1.0) || (s.ap.increaseSpeed > 10.0)) return(false);
  if ((s.ap.decreaseSpeed >= 1.0) || (s.ap.decreaseSpeed < 0.05)) return(false);
  if ((s.ap.decreaseTurn  >= 1.0) || (s.ap.decreaseTurn  < 0.03)) return(false);
  if ((s.ap.deltaTurn < 0.001) || (s.ap.deltaTurn > 0.9)) return(false);
  return(true);
}

