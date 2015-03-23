/* $URL: svn+ssh://aaron@birenboim.com/home/svn/arduino/sketchbook/DalekDrive/Autonomous.h $
$Id: Autonomous.h 132 2013-02-12 02:19:03Z aaron $

A simple autonomous motion mode for the Dalek.
Uses sonar for collision avoidance, but tried to roam
about randomly.

*/

// check motor command limits, shift to fit.  try to retain turn rate
void checkLimits(int &a, int &b)
{
  if (a > 255)
    {
      int d = a-255;
      b -= d;
      if (b < -255) b = -255;
      a = 255;
      return;
    }
  if (a < -255)
    {
      int d = -255-a;
      b += d;
      if (b > 255) b = 255;
      a = -255;
      return;
    }
}

typedef struct {
  float maxSpeed, maxTurn;  // maxumum speed and turn commands

  // "gain" factors for commands to increase/decrease speed and turning
  float increaseSpeed, decreaseSpeed, decreaseTurn, deltaTurn;

  int minCmd;  // min motor command, before just calling it zero
  int close, tooClose;  // cm range to start to try and avoid; STOP and avoid immediately
} AutonomousModeParameters;

class AutonomousController {
 public:
  AutonomousModeParameters prm;
  float _speed, _turn;  // current speed and turn rates.
  unsigned long _prevUpdate;
  int  _prevClosest;
  byte _prevClosestSensor;


  void begin()
  {
    prm.minCmd = 22;  // slowest command speed to send to a motor
    prm.maxSpeed = 0.8;
    prm.maxTurn = 0.7;
    _speed = _turn = 0;
    prm.increaseSpeed = 1.05;
    prm.decreaseSpeed = 0.9;
    prm.decreaseTurn  = 0.8;
    prm.deltaTurn = 0.05;
    prm.tooClose = 30;
    prm.close = 80;
    _prevUpdate = 0;
    _prevClosest = 999;
    _prevClosestSensor = 222;
  }

  bool isLeft(const byte sensorIndex)
  {
    switch(sensorIndex)
      {
      case 0:  // update which are left indices as needed...
      case 1:
        return(true);
      }
    return(false);
  }

  void updateTurn(const byte which)
  {
    bool left = isLeft(which);
    if (left) // make sure it is turning AWAY from sensed obsticle, at some rate.
      {
        float lim = 2 * prm.deltaTurn;
        if (_turn < lim) _turn = lim;
        else _turn += prm.deltaTurn;
      }
    else
      {
        float lim = -2 * prm.deltaTurn;
        if (_turn > lim) _turn = lim;
        else _turn -= prm.deltaTurn;
      }
  }

  void checkLimits()  // check _speed/_turn limits
  {
    if (_speed > prm.maxSpeed) _speed = prm.maxSpeed;
    if (_speed <      0      ) _speed = 0;
    if (_turn  > prm.maxTurn ) _turn  = prm.maxTurn;
    if (_turn < -prm.maxTurn ) _turn  =-prm.maxTurn;
  }

  void update(const unsigned long t)
  {
    //long dt = t - _prevUpdate;

    byte which;
    int closest = Sonar.closest(which);
    Serial.print("Closest: ");Serial.println(closest);
    if      (closest >    prm.close)
      { _speed *= prm.increaseSpeed;
        _turn  *= prm.decreaseTurn;
Serial.println("Not close : Speeding up, turning less");
        // make sure we do go forward SOMEWHAT
        // when there is no nearby obsticle
        if (_speed < .1) _speed = .1;
      }
    else if (closest > prm.tooClose)
      {
Serial.println("Close: slow down, turn more");
        _speed *= prm.decreaseSpeed;
        updateTurn(which);
      }
    else
      {
Serial.println("Too Close : STOP!");
        _speed  = 0;  // very close... STOP!
        updateTurn(which); // try to turn away
      }

    checkLimits();  // check _speed/_turn limits
Serial.print("*** Speed=");Serial.print(_speed);
Serial.print("\tTurn=");Serial.println(_turn);
  }

  // convert speed and turn on normalized scale to 0..255 motor commands
  void getMotorCommands(int &cmdL, int &cmdR)
  {
    cmdL = (_speed + _turn) * 256.0;
    cmdR = (_speed - _turn) * 256.0;

    int oldL, oldR;
    oldL = oldR = 999;
    while (!((oldL==cmdL) && (oldR==cmdR)))
      {
        oldL = cmdL; oldR = cmdR;
        ::checkLimits(cmdL,cmdR);
        ::checkLimits(cmdR,cmdL);
      }

    if ( (cmdL < prm.minCmd) && (cmdL > -prm.minCmd) )
      {
        if ( (cmdR < prm.minCmd) && (cmdR > -prm.minCmd))
          { cmdR = cmdL = 0; } // too slow, just stop
      }
  }

} Autonomous;

