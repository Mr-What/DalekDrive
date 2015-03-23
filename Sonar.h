// $URL: svn+ssh://aaron@birenboim.com/home/svn/arduino/sketchbook/DalekDrive/Sonar.h $
// $Id: Sonar.h 131 2013-01-18 17:55:43Z aaron $
// Arduino Mega controlling an array of HC-SFR0 sonar sensors
//    Uses use Pin-Change interrupts to get sonar timings
// Instantiates global "Sonar"

#define PIN_TRIGGER 49
#define PIN_ECHO0 50
#define PIN_ECHO1 51
#define PIN_ECHO2 52
#define PIN_ECHO3 53

#define NUM_SONAR_SENSORS 4  // assume starting pins 50-53, then 13..10
#define FILTER_LEN  5    // taps for median filter on soundings, must be odd
#define TIMEOUT_US 25000
//----------------------------------------------- SonarState
//should be declared before including this, partilly to force <Arduino.h> defs: const char TAB = '\t';

// validated calibration for HC-SR04 sonar sensors:  (us-464)/58 == cm, distance to tip of sonar cans
int us2cm(const int us) {
  if ((us<=0)||(us>=TIMEOUT_US)) return(999);
  return( (int)((us-464)/58) );
}

// insert value into a sorted list
void insert(const int val, int *a, byte &aLen)
{ // bubble-sort
  byte i = aLen;
  a[aLen++]=val; // stick new entry on end
  while(i>0) // bubble up
  {
    int t = a[i-1];
    if (a[i]>=t) return;
    a[i-1] = a[i];
    a[i--] = t;
  }
}


class SonarState {
public:
  byte pinEcho[NUM_SONAR_SENSORS];
  volatile byte nDone;
  volatile bool done;
  // hopefully, with a long enough sounding ring-buffer we can get
  // away with NOT making it volatile
  volatile int dt[FILTER_LEN+1][NUM_SONAR_SENSORS];
  unsigned long ping, start;
  long nPulses;

  void begin()
  {
    start = ping = 0;
    pinEcho[0]=PIN_ECHO0;
    pinEcho[1]=PIN_ECHO1;
    pinEcho[2]=PIN_ECHO2;
    pinEcho[3]=PIN_ECHO3;
    nPulses = 0;
    nDone = 0;
    done = false;
    for (int i=0; i < NUM_SONAR_SENSORS; i++)
      {
        for (int j=0; j <= FILTER_LEN; j++) dt[j][i]=0;
        pinMode(pinEcho[0],INPUT);
      }
    pinMode(PIN_TRIGGER,OUTPUT);
    digitalWrite(PIN_TRIGGER,LOW);

    PCMSK0 = 0b1111; // enable PC interrupts on arduino pins 50..53
  }

  inline byte activeBufferIndex() const
  { // if nPulses points to NEXT buffer to use, nPulses-1 is active buffer
    return((byte)((nPulses + FILTER_LEN) % (FILTER_LEN+1))); 
  }
  inline byte currentBufferIndex() const
  { // if nPulses points to NEXT buffer to use, nPulses-1 is active,
    // and nPulses-2 is most-recent COMPLETE reading
    return((byte)((nPulses + FILTER_LEN-1) % (FILTER_LEN+1))); 
  }

  void sendPing()
  {
    byte idx = 0;//activeBufferIndex();
    digitalWrite(PIN_TRIGGER,HIGH);
    start = micros();
    nPulses++;
    for (byte i=0; i < NUM_SONAR_SENSORS; i++) dt[idx][i]=0;
    nDone = 0;
    done = false;
    //delayMicroseconds(2);  // whole pulse must be at least 10us, but delay can be less because of overhead and above code
    digitalWrite(PIN_TRIGGER,LOW);
    ping = micros();

    // consider a delay here, to allow echo pins to go high?
    PCICR |= 1;  // enable PCI0 pin group interrupt
  }

  // too much time passed.  reset even if not done, and set timeout vals
  void timeout(int longTime)
  {
    PCICR &= 0xfe;  // disable PCI0 pin group interrupt
    done = true;
    nDone = NUM_SONAR_SENSORS;
    // if nPulses points to NEXT buffer to use, nPulses-1 is current buffer
    byte idx = 0;//activeBufferIndex();
    for (byte i=0; i < NUM_SONAR_SENSORS; i++)
      if (dt[idx][i] == 0) dt[idx][i] = longTime;
  }

  void checkTimeout()
  {
    if (done) return;
    int et = (int)(micros() - ping);
    if (et > TIMEOUT_US) timeout(et);
  }

  /// blocks until reading is complete
  void getReadingBLOCK()
  {
    sendPing();
    while (!done) checkTimeout();
  }



  // get most recent sounding for sensor i
  int us(byte i) const
  {
    // use morst recent, COMPLETE reading
    byte idx = 0;//currentBufferIndex()+FILTER_LEN;
    return(dt[idx][i]);
  }
  // convert current dt[idx] to cm
  int cm(byte i) const { return us2cm(us(i)); }

  /// return median of recent readings from sensor i
  int median(byte j) const
  {
    byte i = (activeBufferIndex()+1) % (FILTER_LEN+1);
    byte iLast = (i + FILTER_LEN-1)  % (FILTER_LEN+1);
    int lo[FILTER_LEN/2], hi[FILTER_LEN/2],mid;
    byte nLo=0;
    byte nHi=0;
    mid = dt[i][j];
    while (i != iLast)
      {
        i = (i+1) % (FILTER_LEN+1);
        int x = dt[i][j];
        if (x < mid)
          {
            if (nLo < FILTER_LEN/2) insert(x,lo,nLo);
            else
              { // too many below mid.  change mid
                insert(mid,hi,nHi);
                if (x > lo[FILTER_LEN/2-1]) mid=x;
                else
                  {
                    mid = lo[FILTER_LEN/2-1];
                    nLo--;
                    insert(x,lo,nLo);
                  }
              }
          }
        else
          { // inverse of above case logic:
            if (nHi < FILTER_LEN/2) insert(x,hi,nHi);
            else
              { // too many above mid.  change mid
                insert(mid,lo,nLo);
                if (x < hi[0]) mid=x;
                else
                  {
                    mid = hi[0];
                    for (byte k = 0; k<FILTER_LEN/2-1; k++) hi[k]=hi[k+1];
                    nHi--;
                    insert(x,hi,nHi);
                  }
              }
          }
      }
    return(mid);
  }
  int medianCM(byte idx) const { return us2cm(median(idx)); }


  /// diagnostic method to do a sonar reading, and print results
  void printReading()
  {
    getReadingBLOCK();

    Serial.print(nPulses); Serial.print(") ");
    Serial.print(start);  Serial.print(TAB);
    Serial.print(ping-start);
    int i;
    for (i=0; i < NUM_SONAR_SENSORS; i++)
      {
        Serial.print(TAB); Serial.print(us(i));
        Serial.print("(");Serial.print(cm(i));Serial.print(")");
      }
    Serial.println();

/*
    Serial.print(F("\t\t"));    // print median, too
    for (i=0; i < NUM_SONAR_SENSORS; i++)
      {
        Serial.print(TAB); Serial.print(median(i));
        Serial.print("(");Serial.print(medianCM(i));Serial.print(")");
      }
    Serial.println();
*/
  }

  void printCurrentCM()
  {
    if (!done) return;  // only print when most recent readings are simultaneous
    byte idx = 0;//currentBufferIndex();
    Serial.print('s');
    for (byte i=0; i < NUM_SONAR_SENSORS; i++)
      {
        Serial.print(TAB);
        //Serial.print((char)('A'+i));
        Serial.print(us2cm(dt[idx][i]));
      }
    Serial.println();
  }

  void printCurrent()
  {
    if (!done) return;  // only print when most recent readings are simultaneous
    byte idx = 0;//currentBufferIndex();
    Serial.print('s');
    for (byte i=0; i < NUM_SONAR_SENSORS; i++)
      {
        Serial.print(TAB);
        //Serial.print((char)('A'+i));
        Serial.print(dt[idx][i]);
      }
    Serial.println();
  }

  int closest(byte &which)
  {
    byte idx = 0;//currentBufferIndex();
    byte iLo = 0;
    int lo = dt[idx][0];
    for (byte i=1; i < NUM_SONAR_SENSORS; i++)
      {
        int d = dt[idx][i];
        if (d < lo)
          {
            lo = d;
            iLo = i;
          }
      }
    which = iLo;
    return(us2cm(lo));
  }

} Sonar;

/// Pin-Change interrupt, for arduino pins 50..53, and 13..10
ISR(PCINT0_vect) {
  unsigned long t = micros();
  int dt = t - Sonar.ping;

  // It has been measured as taking 464us from falling edge of trigger
  // to rising edge of echo duration pulse
  if (dt < 480) return;  // still waiting for leading edge

  byte idx = 0;//Sonar.activeBufferIndex();
  // check which pin fell here, and note time
  for (int i=0; i < NUM_SONAR_SENSORS; i++)
    {
      if (Sonar.dt[idx][i] == 0)
        {
          if (digitalRead(Sonar.pinEcho[i]) == LOW)
            {
              Sonar.dt[idx][i] = dt;
              Sonar.nDone++;
            }
        }
    }
  if (Sonar.nDone == NUM_SONAR_SENSORS)
    {
      Sonar.done = true;
      PCICR &= 0xfe;  // disable PCI0 pin group interrupt
    }
}

// sound 34029 cm/s at sea level
// 34321 at +20C  
// 331.4 + 0.6 Tc (ms/)  where Tc is temp in C (dominated temp, not alt)
//   34482 by above at 22C
#define SPEED_OF_SOUND 34000   // cm/s

