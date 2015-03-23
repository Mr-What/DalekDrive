/*
$URL: svn+ssh://aaron@birenboim.com/home/svn/arduino/sketchbook/DalekDrive/DalekDrive.ino $
$Id: DalekDrive.ino 143 2015-03-23 00:24:53Z aaron $

Main Dalek control loop.

Current model has two H-bridge motor drives, MotL and MotR,
and 4 sonar sensors

Motor drives take speed commands from -255..255,
with negative numbers for reverse.
If commands are not updated reguarly, the
motors will be commanded to stop.

See setup() funtion for pin assignments.
*/
#include <EEPROM.h>

#define SONAR_PERIOD     1000 // 50  // (ms) sonar sample period.  min about 30

const char TAB = '\t';  // forces #include<Arduino.h> here, needed for following #include's

#include "Sonar.h"

#define VICKY_DRIVE
#ifdef VICKY_DRIVE
#include "MotorDriveC.h"
#else
#include "MotorDrive.h"
#endif

#include "Autonomous.h"
#include "State.h"

void setup()
{
  // Config for Arduino Mega:
  // on mega, PWM motor pins are 44,45 on timer 5,
  // the direction pins (REV) will be 42,43
  MotR.begin(42,44);  // REV, EN, [dec]
  MotL.begin(43,45);  // PWM on timer5 (B)

  // set motor PWM freq    1==31250Hz, 2==3926, 3=977, default is 4==480ish Hz
  //TCCR5B = (TCCR5B & 0xF8) | 2; // 3926 Hz
  TCCR5B = (TCCR5B & 0xF8) | 3; // 977 Hz

  //// 20 is ext IRQ 3 ; manual speed override pot input pins
  //// 21 is ext IRQ 2 ; autonomous mode deadman
  //  2 is ext IRQ 0 ; manual override
  //  3 is ext IRQ 1 ; Automomous (deadman)
  State.begin(3,2,6,7);  // loads default state

  Serial.begin(115200);

  if (State.load()!=0)  // load state parameters from EEPROM
    {
      // there was a problem, save default state
      // (as set in State.begin() above) for next boot
      State.save();
    }

  cli();
  Sonar.begin();
  //attachInterrupt(3,deadmanReleased,RISING);  // Int3 is for mega pin20
  //attachInterrupt(2,deadmanReleased,RISING);  // Int2 is for mega pin21
  attachInterrupt(0,deadmanReleased,RISING);  // Int0 is for DIO pin2 (manual override)
  attachInterrupt(1,deadmanReleased,RISING);  // Int1 is for DIO pin3 (autonomous)
  sei();
}


void loop()
{
  unsigned long t = millis();

  // check for mode transition on Motor Controllers
  MotR.update(t);
  MotL.update(t);
  
  State.checkCommand(t);
  Sonar.checkTimeout();

  bool manualMode = !digitalRead(State.Pin.manualOverride);
  if (manualMode)
    {
      State.mode |=  MODE_MANUAL;  // means deadman is enabled.
      // manual-override mode, get commands from pots
      State.updateManualOverride();
      return;
    }

  if (State.mode & MODE_MANUAL)
    { // transition from manual to remote-commanded, shutdown immediately.
      State.ImmediateStop();
      State.mode &= ~MODE_MANUAL;
      return;
    }

  // For now, ignore Autonomous mode command.  let pin control mode.
  bool autonomous = !digitalRead(State.Pin.autonomous);
  if (autonomous) State.mode |= MODE_AUTONOMOUS;
  else
    {
      if (t-State.prevCmdTime > State.cmdTimeout)
        { // loss of command... SHUTDOWN!
          State.ImmediateStop();
          State.prevCmdTime = t;
        }
      State.mode &= ~(MODE_AUTONOMOUS);
    }
  
  //Serial.print("prev ");Serial.print(State.prevCmdTime);Serial.print("\t");Serial.println(t);

  if (Sonar.done)
    {
      static bool sonarProcessed = false;

      if (sonarProcessed)
        { // check if it is time to send next pulse
          static unsigned long prevSonar = 0;

          t = millis();  // update time, incase above code took a long time
          if (t - prevSonar >= State.SonarPeriod)
            {
              prevSonar = t;
              Sonar.sendPing();
              sonarProcessed = false;
            }
        }
      else
        {
          if (autonomous)
            { // new sonar reading is ready
              Autonomous.update(t);  // update internal state
              int cmdL,cmdR;
              Autonomous.getMotorCommands(cmdL,cmdR);
              Serial.print('L');Serial.print(cmdL);
              Serial.print("\tR");Serial.println(cmdR);
              MotL.setSpeed(cmdL);
              MotR.setSpeed(cmdR);
            }
          Sonar.printCurrentCM();
          //Sonar.printCurrent();
          sonarProcessed = true;
        }
    }
}


// make sure deadman release stops immediately
void deadmanReleased()
{
  MotR.emergencyStop();
  MotL.emergencyStop();
}
