#include <Servo.h>

//  Mango Eye Mechanism Code
//  Utilizes Adafruit PCA9685 servo driver
//  Slow servo control seems to require the current and desired servo position
//  Function fn_gomove increments each eye servo position starting with
//  the largest distance to travel

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca= Adafruit_PWMServoDriver(0x40);

// Servo travel limits must be set specifically for Hitec servos at 60 hz
// The mechanism has hard limits considerably less than the servo max travel
// The mechanism will likely break if driven to the servo maximums 
// The eye and eye lid mechanism have unique limits, see below
// Another sketch called Mango_Eyes_blink facilitates setting these limits

// Initialization

const int size = 9;  //  size of array that postions servos randomly
const int size2 = 5;  //  size of array for sensor readings

int moveto[size];

int ledPin = 13; // PIR SR-501 initialization
int pirPin = 8;
int pirState = LOW;
int pirtimer = 0;
int motion = 0;
double cur = 0;  //  servo current intialization


int relayPin = 7;  //  servo power relay initialization
int relayState = LOW;

int fanPin = 6;  // brown wire connects D6 and relay 2
int fanOn = 0;

// Note: Analog I/O pins A3 and A4 are used for the servo driver board

int curPin = A2;  //  ACS712ELC-20A head current sensor initialization
int tempPinh = A3;  //  TMP136 head temperature sensor initialization
int tempPinp = A6;  //  TMP136 power supply temperature sensor initialization
int voltPin = A7;  //  voltage sensor initialization

void setup()
{
  pinMode(ledPin, OUTPUT);// LED on/off indicator
  pinMode(curPin, INPUT);// current sensor
  pinMode(pirPin, INPUT);// HC-SR501 temperature sensor
  pinMode(tempPinh, INPUT); // TMP136 temperature sensor
  pinMode(tempPinp, INPUT); // TMP136 temperature sensor
  pinMode(voltPin, INPUT); // voltage sensor
  pinMode(relayPin, OUTPUT);  // servo power relay module
  pinMode(fanPin, OUTPUT);  // fan on/off relay module

  pca.begin();
  pca.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  // Hitec 5565 digital servos can be controlled at higher frequencies
  // but the inproved accuracy is not needed for this eye mechanism

  Serial.begin(9600); // Adafruit PCA9685 servo driver
  Serial.println(F("Initialize System"));  
  delay(10);
}

// Servo control loop

void loop()
{
  // Do not drive the servos beyond the following limits else the mechanism could break

  int elo = 295; // full min travel of the hitec 5065 at 60hz is 205
  int ehi = 420; // full max travel of the hitec 5065 at 60hz is 510
  int emid = 357; // mid travel is 357

  // The Feetech 360 servo is controlled via PWM to a position, not speed
  // The rotation is limited to +/- 180 deg by PWM of 630 and 130, respectively

  int npmid = 380;  // neck position middle
  int npmin = 130;  // min servo PWM signal, any smaller and the AtoD converter does so badly
  int npmax = 630;
  
  int uplo = 275; // upper lid min position, changed from 250 to allow wide open lids
  int uplc = 359; // upper lid max position
  int lwlo = 275; // lower lid min position
  int lwlc = 359; // lower lid max position

  int uploo = uplo * 0.9;  // wide open lid positions to express suprise
  int lwloo = uplo * 0.9;

  int servocnt = 9;
  int servoNum = 1;

  int cycles = 0;
  int l = 0;
  int slowd = 100;  //  time delay sleepy eyelids servo slew rate

  //  When motion is detected, or motion ceases to be detected,
  //  the eyelids slowly open or close.
  //  Functions fn_lidsopen and fn_lidsclose are called.
  //  After sleepy eyelids are open, random eye movement occurs for "cycle"
  //  iterations, then sleep resumes.

  motion = digitalRead(pirPin); // read input value of PIR
  if (motion == HIGH)  // check if the input is HIGH
  {
    Serial.println("motion detected, x cycles");
    digitalWrite(ledPin, HIGH);  // turn LED on
    digitalWrite(relayPin, HIGH);  //  turn on servo power through relay
    relayState = HIGH;
    digitalWrite(fanPin, HIGH);  //  turn fan on through relay
    fanOn = HIGH;
    Serial.println("servo power on");
    Serial.println("cooling fan on");
            
    cycles = 20;

    //  Upon startup, prints of the head and power supply temperature and voltage are made
    // Need to take an average of many samples rather than a single check
    // See Maker guides ACL712 for an example of 1000 sample average
    
    Serial.print("Initial Head Temperature");   //  print temperature
    fn_temp(tempPinh); // temperature calc

    Serial.print("Initial Power Supply Temperature");
    fn_temp(tempPinp); // temperature calc

    Serial.print("Initial Power Supply Voltage");
    fn_volt(voltPin); // voltage calc

    Serial.print("Initial Head Current");
    fn_cur(curPin); // current calc function

    delay(10); // wait before waking

    if (pirState == LOW)
    {
    //  elements 7 and 8, npfm and npto are reset below
    // array elements
    //  0     1      2      3     4      5     6   7    8
    // exfm, elyfm, eryfm, exto, elyto, eryto, d, npfm, npto

    int moveto[size] = {357,357,357,357,345,370,1,npmid,npmid};

    Serial.println("initializing moveto");
    Serial.println("exfm  elyfm eryfm exto elyto eryto  d   npfm   npto");
    Serial.print(moveto[0]);
    Serial.print("   ");
    Serial.print(moveto[1]);
    Serial.print("   ");
    Serial.print(moveto[2]);
    Serial.print("   ");
    Serial.print(moveto[3]);
    Serial.print("   ");
    Serial.print(moveto[4]);
    Serial.print("   ");
    Serial.print(moveto[5]);
    Serial.print("   ");
    Serial.print(moveto[6]);
    Serial.print("   ");
    Serial.print(moveto[7]);
    Serial.print("   ");
    Serial.println(moveto[8]);
    //    0       1      2       3        4
    // counter, temph, tempp, voltage, current

    int sensor[size2] = {0,0,0,0,0};  // array elements
    
    delay(5);
     
      Serial.println("motion detected and waking!");
      pirState = HIGH;
      delay(50);
      fn_lidsopen(uplo, uploo, uplc, slowd); // sleepy eyelids open
    }
  }
  else
  {
    digitalWrite(ledPin, LOW); //  turn LED off
    if (pirState == HIGH)  // go to sleep and turn servo power off after motion ceases
    {
      Serial.println("Motion end!");
      cycles = 0.;
      pirState = LOW;

      moveto[3] = emid;  //  center eyes
      moveto[4] = emid;
      moveto[5] = emid;
      moveto[6] = 30;
      moveto[8] = npmid;  // center neck
      fn_gomove(moveto[0], moveto[1], moveto[2], moveto[3], moveto[4], moveto[5], moveto[6], moveto[7], moveto[8]);
      Serial.println("Going to sleep");
      fn_lidsclose(uplo, uploo, uplc, slowd); // sleepy eyelids closed

      //  Upon shutdown, prints of the head and power supply temperature and voltage are made

      Serial.print("Final Head Temperature");   //  print temperature
      fn_temp(tempPinh); // temperature calc

      Serial.print("Final Power Supply Temperature");
      fn_temp(tempPinp); // temperature calc

      Serial.print("Final Power Supply Voltage");
      fn_volt(voltPin); // voltage calc

      delay(1000); // wait a second before shutting down

      digitalWrite(relayPin, LOW);
      relayState = LOW;
      Serial.println("servo power off");
      digitalWrite(fanPin, LOW);  //  turn fan off through relay
      fanOn = LOW;
      Serial.println("cooling fan off");
      delay(500);
    }
  }
  // If pirState is HIGH, cycle through random eye motion for "cycles" times
  if (pirState == HIGH)
  {
    Serial.println("Start loop");
    delay(2000);

    for (int l=0; l<=cycles; l++) // repeat "cycles" times
    {
      moveto[3] = random(1,8);
      moveto[4] = random(1,8);
      moveto[5] = moveto[4];
      moveto[6] = random(1,8);

    Serial.println("Top of loop, random numbers");
    Serial.println("exfm  elyfm eryfm exto elyto eryto  d   npfm   npto");
    Serial.print(moveto[0]);
    Serial.print("   ");
    Serial.print(moveto[1]);
    Serial.print("   ");
    Serial.print(moveto[2]);
    Serial.print("   ");
    Serial.print(moveto[3]);
    Serial.print("   ");
    Serial.print(moveto[4]);
    Serial.print("   ");
    Serial.print(moveto[5]);
    Serial.print("   ");
    Serial.print(moveto[6]);
    Serial.print("   ");
    Serial.print(moveto[7]);
    Serial.print("   ");
    Serial.println(moveto[8]);

      // array elements
      //  0     1      2      3     4      5     6   7    8
      // exfm, elyfm, eryfm, exto, elyto, eryto, d, npfm, npto
  
      // calc eye servo position between elo and ehi based on the random number between 1 and 7

      moveto[3] = elo + (ehi - elo)*(moveto[3] - 1)/6;  // eye left/right motion

      moveto[4] = elo + (ehi - elo)*(moveto[4] - 1)/6; // left eye up/down motion
      moveto[5] = ehi - (ehi - elo)*(moveto[5] - 1)/6; // right eye up/down motion, opposite due to servo orientation

      moveto[6] = moveto[6]*50/7;  // calc rate, or motion delay between 7 and 50ms based on random no
    //  moveto[7] = uplo + (uplc - uplo)*(moveto[7] - 1)/6; // calc lid servo position between uplo and uplc

    //  moveto[8] = moveto[7]; // copy upper lid position to lower lid

      // neck position determination

      moveto[7] = moveto[8];  // copy previous neck to position to from position
      moveto[8] = npmid;  // set next neck position to mid, unless the eye position is near max left or right, see if's below
      if (moveto[3] < (elo + 10)) // turn neck position left when eye position is left
      {
        moveto[8] = npmax;
      }
      if (moveto[3] > (ehi - 10)) // turn neck position right when eye position is right
      {
        moveto[8] = npmin;
      }
       
      delay(5);

      if (l < 1)
      {
        (moveto[0]) = 357;
        (moveto[1]) = 357;
        (moveto[2]) = 357;
        (moveto[7]) = npmid;
      }

      Serial.println("Bottom of loop, servo pwm values");

      fn_gomove(moveto[0], moveto[1], moveto[2], moveto[3], moveto[4], moveto[5], moveto[6], moveto[7], moveto[8]);
  
      // reset exfm, elyfm, and eryfm to the final exto, elyto, and eryto positions
      moveto[0] = moveto[3];
      moveto[1] = moveto[4];
      moveto[2] = moveto[5];  

      if (moveto[6]>49)  // blink occasionally, dependent on the time delay - d
      {
        int blnkNum = 1;
        int blnkd = 5;
        fn_blnk(uploo, uplc, blnkNum, blnkd); // blinks "blnkNum" times with "blnkd" delay after lids closed
      }

      if (moveto[6]<8 && moveto[1]>390)  // wink occasionally, dependent on the time delay - d
      {
        int blnkNum = 1;
        int blnkd = 5;
        fn_wink(uploo, uplc, blnkNum, blnkd); // winks "blnkNum" times with "blnkd" delay after lids closed
      }

      if (l>=cycles)  // last eye movement, center eyes before closing lids
      {
        moveto[0] = moveto[3];
        moveto[1] = moveto[4];
        moveto[2] = moveto[5];
        moveto[3] = emid;
        moveto[4] = emid;
        moveto[5] = emid;
        moveto[6] = 4;

        Serial.println("initializing last loop");
   
        fn_gomove(moveto[0], moveto[1], moveto[2], moveto[3], moveto[4], moveto[5], moveto[6], moveto[7], moveto[8]);
      }
      delay(5);
    }
    //  end of loop
  }
}

// _______________________________________
// _______________________________________
// Functions

// fn_gomove allows servo transition at variable rates determined by delay "d"
// the array element positions are:
//  0     1      2      3      4      5    6   7     8
// exfm, elyfm, eryfm, exto, elyto, eryto, d, npfm, npto

int fn_gomove(int xfm, int lyfm, int ryfm, int xto, int lyto, int ryto, int d, int npfm, int npto)
{
  Serial.print(d);   //  print servp motion time delay
  Serial.print(" ms "); // labeled in milliseconds, ms
  Serial.println("delay");

  // The Feetech 360 servo is controlled via PWM to a position, not speed
  // The rotation is limited to +/- 180 deg by PWM of 630 and 130, respectively
  
  int np = npfm;  // initial neck position
  int npmin = 130;  // min servo PWM signal, any smaller and the AtoD converter does so badly
  int npmax = 630;
  int npd = 0;  // delta neck position, length of ramp up / down
  int ndelay = 100;  // neck motion time delay
  int ndelayd = 1;  // neck motion time delay increment
  int ndelaymax = 100;  // neck motion maximum time delay
  int ndelaymin = 20;  // neck motion minimum time delay
  int decel = 0;  // toggle to switch from ramp up to down
  int accel = 0;  // toggle to avoid ramp up when ramping down

  int adx = (xto - xfm);
  int adly = (lyto - lyfm);
  int adry = (ryto - ryfm);
  int i = xfm;
  int j = lyfm;
  int k = ryfm;
  int mid = 357;

// The compiler doesn't like "abs()" in this context, returned the error:
// "unterminated argument list invoking macro "abs"
// when attempted: int adx = abs(dx);
// Therefore, these if statements are the substitute:

  if (adx < 0) {adx = -adx;}
  if (adly < 0) {adly = -adly;}
  if (adry < 0) {adry = -adry;}

  // three alternatives to which is the larger transion: dx > dy, dy > dx, or dx = dy
  // dx = left/right position change for both eyes
  // dly - left eye up/down change
  // the right eye up/down motion is to be inverted as the left eye position is incremented
  // three cases:
  //  1     2      3
  //  dx   dly   dx = dy
  //  dly  dx
// *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *
  if (adx > adly)  //  case 1, adx > adly
  {
    Serial.println("case 1 begin");
    if (xto > xfm)  //  x increasing
    {
      if (npto < npfm) // neck position decreasing
      {
        decel = 1;
        while (ndelay > ndelaymin)  // accel neck move from npfm
        {
          delay(ndelay);
          ndelay = ndelay - ndelayd;
          npd = npfm - np;
          np = np - 1;
          pca.setPWM(8,0,np);
        }
      }
      for (i = xfm; i < xto; i++)
      {
        // servo connection sequence - 0 = lx, 1 = ly, 2 = rx, 3 = ry, where lx is left eye in left/right motion
        pca.setPWM(0, 0, i);
        pca.setPWM(2, 0, i);
        if (np > (npto + npd))  // finish neck move towards npto
        {
          pca.setPWM(8,0,np);
          np = np - 1;
        }
        else if (ndelay < ndelaymax)  // decel neck to npto
        {
          while (ndelay < ndelaymax)
          {
            pca.setPWM(8,0,np);
            ndelay = ndelay + ndelayd;
            np = np - 1;
            delay(ndelay);
          }
        }
        if (lyto > j)  // case 1, ly increasing
        {
          pca.setPWM(1, 0, j);
          pca.setPWM(3, 0, (2*mid - j));
          j = j + 1;
        }
        else if (lyto < j) //  case 1, ly decreasing
        {
          pca.setPWM(1, 0, j);
          pca.setPWM(3, 0, (2*mid - j));
          j = j - 1;
        }
        else
        {
        pca.setPWM(1, 0, lyto);
        pca.setPWM(3, 0, (2*mid - j));
        }
        delay(d);
      }
      while (np > (npto + npd))  // finish neck move towards npto
        {
          pca.setPWM(8,0,np);
          np = np - 1;
          delay(ndelay);
        }
      while (ndelay < ndelaymax)  // decel neck to npto
        {
          pca.setPWM(8,0,np);
          ndelay = ndelay + ndelayd;
          np = np - 1;
          delay(ndelay);
        }
      Serial.println("case 1 increasing end");
      return;
    }
    else if (xto < xfm) //  x decreasing
    {
      if (npto > npfm) // neck position increasing
      {
        while (ndelay > ndelaymin)  // accel np from npfm
        {
          pca.setPWM(8,0,np);
          delay(ndelay);
          ndelay = ndelay - ndelayd;
          npd = np - npfm;
          np = np + 1;
        }
      }
      for (i = xfm; i > xto; i--)  //  case 1, adx > adly, x decreasing
      {
        // servo connection sequence - 0 = lx, 1 = ly, 2 = rx, 3 = ry, where lx is left eye in left/right motion
        pca.setPWM(0, 0, i);
        pca.setPWM(2, 0, i);

        if (np < (npto - npd))  // constant rate np w eye x
        {
          pca.setPWM(8,0,np);
          np = np + 1;
        }
        else if (ndelay < ndelaymax)  // decel np
        {
          while (ndelay < ndelaymax)
          {
            pca.setPWM(8,0,np);
            ndelay = ndelay + ndelayd;
            np = np + 1;
            delay(ndelay);
          }
        }
        if (lyto > j)  // case 1, ly increasing
        {
          pca.setPWM(1, 0, j);
          pca.setPWM(3, 0, (2*mid - j));
          j = j + 1;
        }   
        else if (lyto < j) //  case 1, ly decreasing
        {
          pca.setPWM(1, 0, j);
          pca.setPWM(3, 0, (2*mid - j));
          j = j - 1;
        }
        else
        {
          pca.setPWM(1, 0, lyto);
          pca.setPWM(3, 0, (2*mid - j));
        }
        delay(d);
      }
      while (np < (npto - npd))  // finish neck move towards npto
      {
        pca.setPWM(8,0,np);
        np = np + 1;
        delay(ndelay);
      }
      while (ndelay < ndelaymax)  // decel neck to npto
      {
        pca.setPWM(8,0,np);
        ndelay = ndelay + ndelayd;
        np = np + 1;
        delay(ndelay);
      }
      Serial.println("case 1 decreasing end");
      return;
    }
    else  // i = xto
    {
      return;
    }
  }
  else if (adx < adly)  //  case 2, adly > adx 
  {
    if (lyto > lyfm)  //  ly increasing 
    {
      Serial.println("case 2 begin, increasing");
      for (j = lyfm; j < lyto; j++)
      {
        // servo connection sequence - 0 = lx, 1 = ly, 2 = rx, 3 = ry, where lx is left eye in left/right motion
        pca.setPWM(1, 0, j);
        pca.setPWM(3, 0, (2*mid - j));
        if (xto > i)  // case 2, x increasing
        {
          if (npto < np) // neck position decreasing
          {
            decel = 1;
            while (ndelay > ndelaymin && accel < 1)  // accel neck move from npfm
            {
              pca.setPWM(8,0,np);
              delay(ndelay);
              ndelay = ndelay - ndelayd;
              npd = npfm - np;
              np = np - 1;
            }
            if (np > (npto + npd))  // finish neck move towards npto following eye movement
            {
              pca.setPWM(0, 0, i);
              pca.setPWM(2, 0, i);
              pca.setPWM(8,0,np);
              np = np - 1;
              i = i + 1;
            }
            else if (ndelay < ndelaymax)  // decel neck to npto
            {
              accel = 1;
              pca.setPWM(0, 0, i);  // no np move, eyes moving
              pca.setPWM(2, 0, i);
              pca.setPWM(8,0,np);
              ndelay = ndelay + ndelayd;
              np = np - 1;
              i = i + 1;
            }
          }
          else
          {
            pca.setPWM(0, 0, i);  // no np move, eyes moving
            pca.setPWM(2, 0, i);
            i = i + 1;
          }
        }
        else if (xto < i) //  case 2, x decreasing
        {
          if (npto > np) // neck position increasing
          {
            decel = 1;
            while (ndelay > ndelaymin)  // accel neck move from npfm
            {
              pca.setPWM(8,0,np);
              delay(ndelay);
              ndelay = ndelay - ndelayd;
              npd = np - npfm;
              np = np + 1;
            }
            if (np < (npto - npd))  // finish neck move towards npto
            {
              pca.setPWM(0, 0, i);
              pca.setPWM(2, 0, i);
              pca.setPWM(8,0,np);
              np = np + 1;
              i = i - 1;
            }
            else
            {
              while (ndelay < ndelaymax) // decel neck to npto
              {
                pca.setPWM(8,0,np);
                ndelay = ndelay + ndelayd;
                np = np + 1;
                i = i - 1;
              }
            }
          }
          else
          {
            pca.setPWM(0, 0, i); // if neck position doesn't change, continue to move eyes
            pca.setPWM(2, 0, i);
            i = i - 1;
          }
        }
        else  // i = xto, either eye x didn't move or they finished moving but np hasn't finished
        {
          if (decel > 0 && np > (npto + npd))  // finish neck move following x eye movement
          {
            pca.setPWM(8,0,np);
            np = np - 1;
          }
          else if (decel > 0 && (npto < npfm) && (ndelay < ndelaymax))
          {
            while (ndelay < ndelaymax)  // decel neck to npto
            {
              pca.setPWM(8,0,np);
              delay(ndelay);
              ndelay = ndelay + ndelayd;
              np = np - 1;
            }
          }
          if (decel > 0 && np < (npto - npd))  // finish neck move towards npto
          {
            pca.setPWM(8,0,np);
            np = np + 1;
          }
          else if (decel > 0 && (npto > npfm) && (ndelay < ndelaymax))  // decel neck to npto
          {
            while (ndelay < ndelaymax) // decel neck to npto
            {
              pca.setPWM(8,0,np);
              delay(ndelay);
              ndelay = ndelay + ndelayd;
              np = np + 1;
            }
          }
        }
        if ((lyto - lyfm) <= 1 && decel > 0) // eye l move is nearly complete, but np move isn't
        {
          while ((np - npto) > 1 && decel > 0)  // neck position decreasing on the last eye y move
          {
            if (np > (npto + npd))
            {
            pca.setPWM(8,0,np);
            np = np - 1;
            delay (ndelay);
            }
            else if (ndelay < ndelaymax)  // decel neck to npto
            {
              pca.setPWM(8,0,np);
              np = np - 1;
              ndelay = ndelay + ndelayd;
              delay (ndelay);
            }
          }
          while ((npto - np) > 1 && decel > 0)  // neck position increasing on the last eye y move
          {
            if (np < (npto - npd))
            {
            pca.setPWM(8,0,np);
            np = np + 1;
            delay (ndelay);
            }
            else if (ndelay < ndelaymax)  // decel neck to npto
            {
              pca.setPWM(8,0,np);
              np = np + 1;
              ndelay = ndelay + ndelayd;
              delay (ndelay);
            }
          }
        }
        delay(d);
      }
      Serial.println("case 2 increasing end");
      return;
    }
    // *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *
    else if (lyto < lyfm) //  ly decreasing
    {
      Serial.println("case 2 begin, decreasing");
      for (j = lyfm; j > lyto; j--)  //  case 2, adly > adx, ly decreasing
      {
        Serial.println("lto  xto  npto  j  i  np  nptm  npd");
        // servo connection sequence - 0 = lx, 1 = ly, 2 = rx, 3 = ry, where lx is left eye in left/right motion
        pca.setPWM(1, 0, j);
        pca.setPWM(3, 0, (2*mid - j));
        if (xto > i)  // case 2, x increasing
        {
          if (npto < np) // neck position decreasing
          {
            decel = 1;
            while (ndelay > ndelaymin)  // accel neck move from npfm
            {
              pca.setPWM(8,0,np);
              delay(ndelay);
              ndelay = ndelay - ndelayd;
              npd = npfm - np;
              np = np - 1;
            }
            if (np > (npto + npd))  // finish neck move towards npto following eye movement
            {
              pca.setPWM(0, 0, i); // constant "d" delay times for all servos
              pca.setPWM(2, 0, i);
              pca.setPWM(8,0,np);
              np = np - 1;
              i = i + 1;
            }
            else if (ndelay < ndelaymax)  // decel neck to npto
            {
              accel = 1;
              pca.setPWM(0, 0, i);
              pca.setPWM(2, 0, i);
              pca.setPWM(8,0,np);
              ndelay = ndelay + ndelayd;
              np = np - 1;
              i = i + 1;
            }
          }
          else
          {
            pca.setPWM(0, 0, i);  // if neck position doesn't change, continue to move eyes
            pca.setPWM(2, 0, i);
            i = i + 1;
          }
        }
        else if (xto < i) //  case 2, x decreasing
        {
          if (npto > np) // neck position increasing
          {
            decel = 1;
            while (ndelay > ndelaymin)  // accel neck move from npfm
            {
              pca.setPWM(8,0,np);
              delay(ndelay);
              ndelay = ndelay - ndelayd;
              npd = np - npfm;
              np = np + 1;
            }
            if (np < (npto - npd))  // finish neck move towards npto
            {
              pca.setPWM(0, 0, i);
              pca.setPWM(2, 0, i);
              pca.setPWM(8,0,np);
              np = np + 1;
              i = i - 1;
            }
            else if (ndelay < ndelaymax) // decel neck to npto
            {
              while (ndelay < ndelaymax) // decel neck to npto
              {
                pca.setPWM(8,0,np);
                ndelay = ndelay + ndelayd;
                np = np + 1;
                i = i - 1;
              }
            }
          }
          else
          {
            pca.setPWM(0, 0, i); // if neck position doesn't change, continue to move eyes
            pca.setPWM(2, 0, i);
            i = i - 1;
          }
        }
        else  // i = xto, either eye x didn't move or they finished moving but np hasn't finished
        {
          if (decel > 0 && np > (npto + npd))  // finish neck move towards npto following eye movement
          {
            pca.setPWM(8,0,np);
            np = np - 1;
          }
          else if (decel > 0 && (npto < npfm) && (ndelay < ndelaymax)) // decel neck to npto
          {
            while (ndelay < ndelaymax)  // decel neck to npto
            {
              pca.setPWM(8,0,np);
              delay(ndelay);
              ndelay = ndelay + ndelayd;
              np = np - 1;
            }
          }
          if (decel > 0 && np < (npto - npd))  // finish neck move towards npto
          {
            pca.setPWM(8,0,np);
            np = np + 1;
          }
          else if (decel > 0 && (npto > npfm) && (ndelay < ndelaymax))  // decel neck to npto
          {
            while (ndelay < ndelaymax) // decel neck to npto
            {
              pca.setPWM(8,0,np);
              delay(ndelay);
              ndelay = ndelay + ndelayd;
              np = np + 1;
            }
          }
        }
        if ((lyfm - lyto) <= 1 && decel > 0)
        {
          if (decel > 0 && np > (npto + npd))  // neck position decreasing on the last eye y move
          {
            while (np > (npto + npd))
            {
            pca.setPWM(8,0,np);
            np = np - 1;
            delay (ndelay);
            }
            while (ndelay < ndelaymax)  // decel neck to npto
            {
              pca.setPWM(8,0,np);
              np = np - 1;
              ndelay = ndelay + ndelayd;
              delay (ndelay);
            }
          }
          else if (decel > 0 && np < (npto - npd))  // neck position increasing on the last eye y move
          {
            while (np < (npto - npd))
            {
              pca.setPWM(8,0,np);
              np = np + 1;
              delay (ndelay);
            }
            while (ndelay < ndelaymax)  // decel neck to npto
            {
              pca.setPWM(8,0,np);
              np = np + 1;
              ndelay = ndelay + ndelayd;
              delay (ndelay);
            }
          }
        }
        delay(d);
      }
      Serial.println("case 2 decreasing end");
      return;
    }
    else  // j = lto
    {
      return;
    }
  }
  else //  case 3, adx = adly 
  {
    if (lyto > lyfm)  //  ly increasing 
    {
      Serial.println("case 3 begin, increasing");
      for (j = lyfm; j < lyto; j++)
      {
        // servo connection sequence - 0 = lx, 1 = ly, 2 = rx, 3 = ry, where lx is left eye in left/right motion
        pca.setPWM(1, 0, j);
        pca.setPWM(3, 0, (2*mid - j)); 
        if (xto > i)  // case 3, x increasing
        {
          if (npto < np) // neck position decreasing
          {
            decel = 1;
            while (ndelay > ndelaymin)  // accel neck move from npfm
            {
              pca.setPWM(8,0,np);
              delay(ndelay);
              ndelay = ndelay - ndelayd;
              npd = npfm - np;
              np = np - 1;
            }
            if (np > (npto + npd))  // finish neck move towards npto with eye move
            {
              pca.setPWM(0, 0, i);
              pca.setPWM(2, 0, i);
              pca.setPWM(8,0,np);
              np = np - 1;
              i = i + 1;
            }
            else
            {
              while (ndelay < ndelaymax)  // decel neck to npto
              {
                pca.setPWM(8,0,np);
                ndelay = ndelay + ndelayd;
                np = np - 1;
                i = i + 1;
              }
            }
          }
          else
          {        
            pca.setPWM(0, 0, i);  // no np move, eyes moving
            pca.setPWM(2, 0, i);
            i = i + 1;
          }
        }
        else if (xto < i) //  case 3, x decreasing
        {
          if (npto > np) // neck position increasing
          {
            decel = 1;
            while (ndelay > ndelaymin)  // accel neck move from npfm
            {
              pca.setPWM(8,0,np);
              delay(ndelay);
              ndelay = ndelay - ndelayd;
              npd = np - npfm;
              np = np + 1;
            }
            if (npto < (npto - npd))  // finish neck move towards npto
            {
              pca.setPWM(0, 0, i);
              pca.setPWM(2, 0, i);
              pca.setPWM(8,0,np);
              np = np + 1;
              i = i - 1;
            }
            else if (ndelay < ndelaymax)  // decel neck to npto
            {
              while (ndelay < ndelaymax)  // decel neck to npto
              {
                pca.setPWM(8,0,np);
                ndelay = ndelay + ndelayd;
                np = np + 1;
                i = i - 1;
              }
            }
          }
          else
          {
            pca.setPWM(0, 0, i);  // if neck position doesn't change, continue to move eyes
            pca.setPWM(2, 0, i);
            i = i - 1;
          }
        }
        else  // i = xto, either neck position doesn't change, or they finished moving but np hasn't
        {
          if (decel > 0 && np > (npto + npd))  // finish neck move following x eye movement
          {
            pca.setPWM(8,0,np);
            np = np - 1;
          }
          else if (decel > 0 && (npto < npfm) && (ndelay < ndelaymax))
          {
            while (ndelay < ndelaymax) // decel neck to npto
            {
              pca.setPWM(8,0,np);
              delay(ndelay);
              ndelay = ndelay + ndelayd;
              np = np - 1;
            }
          }
          if (decel > 0 && np < (npto - npd))  // finish neck move towards npto
          {
            pca.setPWM(8,0,np);
            np = np + 1;
          }
          else if (decel > 0 && (npto > npfm) && (ndelay < ndelaymax))  // decel neck to npto
          {
            while (ndelay < ndelaymax)  //decel neck to npto
            {
              pca.setPWM(8,0,np);
              delay(ndelay);
              ndelay = ndelay + ndelayd;
              np = np + 1;
            }
          }
        }
        if ((lyto - lyfm) <= 1 && decel > 0)
        {
          if (decel > 0 && np > (npto + npd))  // neck position decreasing on the last eye y move
          {
            while (np > (npto + npd))
            {
              pca.setPWM(8,0,np);
              np = np - 1;
              delay (ndelay);
            }
            while (ndelay < ndelaymax)  // decel neck to npto
            {
              pca.setPWM(8,0,np);
              np = np - 1;
              ndelay = ndelay + ndelayd;
              delay (ndelay);
            }
          }
          else if (decel > 0 && np > (npto - npd))  // neck position decreasing on the last eye y move
          {
            while (np > (npto - npd))
            {
              pca.setPWM(8,0,np);
              np = np + 1;
              delay (ndelay);
            }
            while (ndelay < ndelaymax)  // decel neck to npto
            {
              pca.setPWM(8,0,np);
              np = np + 1;
              ndelay = ndelay + ndelayd;
              delay (ndelay);
            }
          }
        }
        delay(d);
      }
      Serial.println("case 3 increasing end");
      return;
    }
    else if (lyto < lyfm) //  ly decreasing
    {
      Serial.println("case 3 begin decreasing");
      for (j = lyfm; j > lyto; j--)  //  case 3, adx = adly, ly decreasing
      {
        // servo connection sequence - 0 = lx, 1 = ly, 2 = rx, 3 = ry, where lx is left eye in left/right motion
        pca.setPWM(1, 0, j);
        pca.setPWM(3, 0, (2*mid - j));
        if (xto > i)  // case 3, x increasing
        {
          if (npto < np) // neck position decreasing
          {
            decel = 1;
            while (ndelay > ndelaymin)  // accel neck move from npfm
            {
              pca.setPWM(8,0,np);
              delay(ndelay);
              ndelay = ndelay - ndelayd;
              npd = npfm - np;
              np = np - 1;
            }
            if (np > (npto + npd))  // finish neck move with eye move
            {
              pca.setPWM(0, 0, i);  // constant "d" delay times for all servos
              pca.setPWM(2, 0, i);
              pca.setPWM(8,0,np);
              np = np - 1;
              i = i + 1;
            }
            else
            {
              while (ndelay < ndelaymax)
              {
                pca.setPWM(8, 0, np);
                ndelay = ndelay + ndelayd;
                np = np - 1;
                i = i + 1;
              }
            }
          }
          else
          {
            pca.setPWM(0, 0, i);  // if neck position doesn't change continue to move eyes
            pca.setPWM(2, 0, i);
            i = i + 1;
          }
        }
        else if (xto < i) //  case 3, x decreasing
        {
          if (npto > np) // neck position increasing
          {
            decel = 1;
            while (ndelay > ndelaymin)  // accel neck move from npfm
            {
              pca.setPWM(8,0,np);
              delay(ndelay);
              ndelay = ndelay - ndelayd;
              npd = np - npfm;
              np = np + 1;
            }
            if (np < (npto - npd))  // finish neck move toward npto
            {
              pca.setPWM(0, 0, i);
              pca.setPWM(2, 0, i);
              pca.setPWM(8,0,np);
              np = np + 1;
              i = i - 1;
            }
            else
            {
              while (ndelay < ndelaymax)  // decel neck to npto
              {
                pca.setPWM(8,0,np);
                ndelay = ndelay + ndelayd;
                np = np + 1;
                i = i - 1;
              }
            }
          }
          else
          {
            pca.setPWM(0, 0, i);  // if neck position doesn't change, continue to move eyes
            pca.setPWM(2, 0, i);
            i = i - 1;
          }
        }
        else  // i = xto, either eye x didn't move or they finished moving but np hasn't
        {
          if (decel > 0 && np > (npto + npd))  // finish neck move towards npto following eye movement
          {
            pca.setPWM(8,0,np);
            np = np - 1;
          }
          else if (decel > 0 && (npto < npfm) && (ndelay < ndelaymax))  // decel neck to npto
          {
            while (ndelay < ndelaymax)  // decel neck to npto
            {
              pca.setPWM(8,0,np);
              delay(ndelay);
              ndelay = ndelay + ndelayd;
              np = np - 1;
            }
          }
          if (decel > 0 && np < (npto - npd))  // finish neck move towards npto
          {
            pca.setPWM(8,0,np);
            np = np + 1;
          }
          else if (decel > 0 && (npto > npfm) && (ndelay < ndelaymax))  // decel neck to npto
          {
            while (ndelay < ndelaymax)  // decel neck to npto
            {
              pca.setPWM(8,0,np);
              delay(ndelay);
              ndelay = ndelay + ndelayd;
              np = np + 1;
            }
          }
        }
        if ((lyfm - lyto) <= 1 && decel > 0)
        {
          while ((np - npto) > 1 && decel > 0)
          {
            if (np > (npto + npd))
            {
              pca.setPWM(8,0,np);
              np = np - 1;
              delay (ndelay);
            }
            else if (ndelay < ndelaymax)  // decel neck to npto
            {
              pca.setPWM(8,0,np);
              np = np - 1;
              ndelay = ndelay + ndelayd;
              delay (ndelay);
            }
          }
          while ((npto - np) > 1 && decel > 0)  // neck position increasing on the last eye y move
          {
            if (np < (npto - npd))
            {
              pca.setPWM(8,0,np);
              np = np + 1;
              delay (ndelay);
            }
            else if (ndelay < ndelaymax)  // decel neck to npto
            {
              pca.setPWM(8,0,np);
              np = np + 1;
              ndelay = ndelay + ndelayd;
              delay (ndelay);
            }
          }
        }
        delay(d);
      }
      Serial.println("case 3 decreasing end");
      return;
    }
    else  // lyto = lyfm
    {
      return;
    }
  }
}
//_____________________________________________________________
int fn_lidsclose(int o, int oo, int c, int d) // sleepy eyelids closed
{
  for (int p = oo; p < ((c-o)/3 + o); p++)
  {
    pca.setPWM(4,0,p);
    pca.setPWM(5,0,p);
    pca.setPWM(6,0,(c+o-p));  //  position reversed due to oppposte servo orientation
    pca.setPWM(7,0,(c+o-p));
    delay (d);
  }
  delay(d);
  for (int p = ((c-o)/3 + o); p > o; p--)
  {
    pca.setPWM(4,0,p);
    pca.setPWM(5,0,p);
    pca.setPWM(6,0,(c+o-p));  //  position reversed due to oppposte servo orientation
    pca.setPWM(7,0,(c+o-p));
    delay (d/20);
  }
  delay(d);
  for (int p = o; p < (2*(c-o)/3 + o); p++)
  {
    pca.setPWM(4,0,p);
    pca.setPWM(5,0,p);
    pca.setPWM(6,0,(c+o-p));  //  position reversed due to oppposte servo orientation
    pca.setPWM(7,0,(c+o-p));
    delay (d);
  }
  delay(d);
  for (int p = (2*(c-o)/3 + o); p > o; p--)
  {
    pca.setPWM(4,0,p);
    pca.setPWM(5,0,p);
    pca.setPWM(6,0,(c+o-p));  //  position reversed due to oppposte servo orientation
    pca.setPWM(7,0,(c+o-p));
    delay (d/20);
  }
  for (int p = o; p < c; p++)
  {
    pca.setPWM(4,0,p);
    pca.setPWM(5,0,p);
    pca.setPWM(6,0,(c+o-p));  //  position reversed due to oppposte servo orientation
    pca.setPWM(7,0,(c+o-p));
    delay (d);
  }
  return;
}

int fn_lidsopen(int o, int oo, int c, int d) // sleepy eyelids closed
{
  for (int p = c; p > (c - (c-o)/3); p--)
  {
    pca.setPWM(4,0,p);
    pca.setPWM(5,0,p);
    pca.setPWM(6,0,(c+o-p));  //  position reversed due to oppposte servo orientation
    pca.setPWM(7,0,(c+o-p));
    delay (d);
  }
  delay(d);
  for (int p = (c - (c-o)/3); p < c; p++)
  {
    pca.setPWM(4,0,p);
    pca.setPWM(5,0,p);
    pca.setPWM(6,0,(c+o-p));  //  position reversed due to oppposte servo orientation
    pca.setPWM(7,0,(c+o-p));
    delay (d/20);
  }
  delay(d);
  for (int p = c; p > (c - 2*(c-o)/3); p--)
  {
    pca.setPWM(4,0,p);
    pca.setPWM(5,0,p);
    pca.setPWM(6,0,(c+o-p));  //  position reversed due to oppposte servo orientation
    pca.setPWM(7,0,(c+o-p));
    delay (d);
  }
  delay(d);
  for (int p = (c - 2*(c-o)/3); p < c; p++)
  {
    pca.setPWM(4,0,p);
    pca.setPWM(5,0,p);
    pca.setPWM(6,0,(c+o-p));  //  position reversed due to oppposte servo orientation
    pca.setPWM(7,0,(c+o-p));
    delay (d/20);
  }
  delay(d);
  for (int p = c; p > o; p--)
  {
    pca.setPWM(4,0,p);
    pca.setPWM(5,0,p);
    pca.setPWM(6,0,(c+o-p));  //  position reversed due to oppposte servo orientation
    pca.setPWM(7,0,(c+o-p));
    delay (d);
  }
  delay(d);
  for (int p = o; p > oo; p--)
  {
    pca.setPWM(4,0,p);
    pca.setPWM(5,0,p);
    pca.setPWM(6,0,(c+o-p));  //  position reversed due to oppposte servo orientation
    pca.setPWM(7,0,(c+o-p));
    delay (d/20);
  }
  return;
}

int fn_blnk(int o, int c, int no, int d)
{
  pca.setPWM(4,0,c);
  pca.setPWM(5,0,c);
  pca.setPWM(6,0,(o + 25));  //  position reversed due to oppposte servo orientation
  pca.setPWM(7,0,(o + 25));

  delay(100);

  for (int p = c; p > o; p--)
  {
    pca.setPWM(4,0,p);
    pca.setPWM(5,0,p);
    pca.setPWM(6,0,(c+o+25-p));  //  position reversed due to oppposte servo orientation
    pca.setPWM(7,0,(c+o+25-p));
    delay (d);
  }
    delay(5);

  return;
}

int fn_wink(int o, int c, int no, int d)
{
  pca.setPWM(4,0,c);
  pca.setPWM(5,0,c);
  
  delay(100);

  for (int p = c; p > o; p--)
  {
    pca.setPWM(4,0,p);
    pca.setPWM(5,0,p);
    delay (d);
  }
    delay(5);

  return;
}


int fn_blnkWide(int oo, int c, int no, int d)
{
  pca.setPWM(4,0,c);
  pca.setPWM(5,0,c);
  pca.setPWM(6,0,oo);
  pca.setPWM(7,0,oo);


  delay(d);

  for (int p = c; p > oo; p--)
  {
    pca.setPWM(4,0,p);
    pca.setPWM(5,0,p);
    pca.setPWM(6,0,(c+oo-p));  //  position reversed due to oppposte servo orientation
    pca.setPWM(7,0,(c+oo-p));
    delay(5);
  }

  pca.setPWM(4,0,c);
  pca.setPWM(5,0,c);
  pca.setPWM(6,0,oo);
  pca.setPWM(7,0,oo);

  delay(d);

for (int p = c; p > oo; p--)
  {
    pca.setPWM(4,0,p);
    pca.setPWM(5,0,p);
    pca.setPWM(6,0,(c+oo-p));
    pca.setPWM(7,0,(c+oo-p));
    delay(5);
  }
  
  pca.setPWM(4,0,c);
  pca.setPWM(5,0,c);
  pca.setPWM(6,0,oo);
  pca.setPWM(7,0,oo);

  delay(d);
  return;
}

int fn_temp(int Pin)
{
int tdig = analogRead(Pin);  //  take a temperature reading
float tvolt = tdig * (5.0 / 1024.0);  //  convert digital to voltage
float tempC = (tvolt - 0.5) * 100;  //  convert voltage to deg C
float tempF = (tempC * 9.0 / 5.0) + 32.0;  //  convert deg C to F
Serial.print(tempF);   //  print temperature
Serial.print("\xC2\xB0"); // shows degree symbol
Serial.println("F");
return;
}

int fn_volt(int Pin)
{
int vdig = analogRead(Pin);  //  take a voltage reading
float volt = vdig * (5.0 / 1024.0 / .62);  //  convert digital to voltage
//  5/.62 is the calibration factor a voltage divider comprised of 47k and 7.8 k resistors
Serial.print(volt);   //  print voltage
Serial.println("volts");
return;
}

int fn_cur(int Pin)
{
  const double sf = 0.1;  //  scale factor for 20A current module
  int curdig = analogRead(Pin);  //  take a current reading
  float curv(curdig*5.00/1024);  //  convert digital to voltage mv
  cur = (curv - 5.00/2) * sf;
  Serial.print(cur, 1);  //  print current
  Serial.print("\t");
  Serial.println("amps");
  return;
}


      // blink once, then lids wide open, blink twice

      //  int blnkNum = 1;
      //  int blnkd = 500;
      //  fn_blnk(uplo, uplc, blnkNum, blnkd); // blinks "blnkNum" times with "blnkd" delay after lids closed
      //  blnkNum = 2;
      //  fn_blnkWide(uploo, uplc, blnkNum, blnkd); // blinks "blnkNum" times with "blnkd" delay after lids closed

      // Close eyelids
      // delay(5);

  
//    Serial.println("gomove return");
//    Serial.print("exfm\telyfm\teryfm\texto\telyto\teryto\td");
//    Serial.println();
//    Serial.print(moveto[0]);
//   Serial.print("\t");
//    Serial.print(moveto[1]);
//    Serial.print("\t");
//    Serial.print(moveto[2]);
//    Serial.print("\t");
//    Serial.print(moveto[3]);
//    Serial.print("\t");
//    Serial.print(moveto[4]);
//    Serial.print("\t");
//    Serial.print(moveto[5]);
//    Serial.print("\t");
//    Serial.print(moveto[6]);
//    Serial.println();
//    Serial.println();
//    Serial.p