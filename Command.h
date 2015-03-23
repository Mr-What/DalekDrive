/*
$URL: svn+ssh://aaron@birenboim.com/home/svn/arduino/sketchbook/DalekDrive/Command.h $
$Id: Command.h 142 2015-03-23 00:16:34Z aaron $

Command interpreter for serial port
*/

class CommandReader
{
public:
  int nDig,val;
  bool neg;
  char code;

  void begin()
  {
    nDig=val=0;
    code=0;
    neg = false;
  }
  bool get(char &cmdCode, int &cmdVal)
  {
    int i = Serial.read();
    if (i < 0) return(false);  // no command yet
    char c = i;
Serial.print('[');Serial.print(i);Serial.print(',');Serial.print(c);Serial.println(']');
    switch(c)
      {
      case '~' :
        Serial.println(F("Command Stream RESET!"));
        begin();
        return(false);

      case '0':
      case '1':
      case '2':
      case '3':
      case '4':
      case '5':
      case '6':
      case '7':
      case '8':
      case '9':
        val = val*10 + (i-((int)('0')));
        nDig++;
        //Serial.print(nDig);Serial.print(")");Serial.println(val);
        return(false);
      case '-':
        if ((nDig == 0) && ((int)code>0))
          {
            //Serial.println(F("negative command value follows:"));
            neg = true; // value is negative
          }
        else
          {
            Serial.println(F("Not expecting a value.  '-' char ignored."));
            begin();  // clear bad entry
          }
        return(false);
      // commands without values
      case '!':
      case '^':
      case '?':
      case 'a':  // command to set Autonomous in manual mode
      case 'A':
        cmdCode = c;  // return prev command code (if any)
        cmdVal = 0;
        return(true);

        // codes with values follow : 
      case 'L':
      case 'R':
      case 'p':
      case 't':
      case 'm':
      case 'C':
      case 'c':
      case 'S':
      case 'T':
      case 'G':
      case 'g':
      case 'r':
      case 'd':
        begin();  // clear old command, if any
        code = c; // remember command for wich the following value applies
        return(false);  // wait for value
        // seperator
      case ' ':
      case '\t':
      case '\r':
      case '\n':
      case 0:
      case ',':
      case ';':
        if ( code > (char)0 )
          { // command was in progress, close it out
            cmdCode = code;
            cmdVal = neg ? -val : val;
            begin();  // clear for next command
            return(true);  // had a complete command
          }
      default: // treat any other character as a seperator
        begin();  // clear any partial command
        return(false);  // prev command not complete
      }
  }

} Command;
