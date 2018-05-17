/*
    CHOKEBOT (v2.69b)
    Justin Lee -- GNU GPLv3 or above-- 
    Copyright 2016-2018 < justin at taiz dot me >

  >> WARNING: ROBOT CLAWS ARE TO BE USED <<
  >> SOLELY FOR THE PURPOSE OF CHOKING,  <<
  >> STRANGLING, AND PLEASURE.           <<

*/


///
// contants

#define serialSpeed 19200
#define serialBlockDelay 50
#define serialReadDelay 15

#define controlClockPin 3
#define controlLatchPin 4
#define controlDataPin 2
#define controlDataSize 8
#define controlDelay 2 // microseconds

#define controlMaskUp     B00001000
#define controlMaskDown   B00000100
#define controlMaskLeft   B00000010
#define controlMaskRight  B00000001
#define controlMaskStart  B00010000
#define controlMaskSelect B00100000
#define controlMaskB      B01000000
#define controlMaskA      B10000000

#define servoMoveDelay 3 // in milliseconds (for now shared by all servos)
#define servoLowerLimit 0 // these are absolute min / max
#define servoUpperLimit 180

#define servoHeadPin 8
#define servoHeadLowerLimit 15
#define servoHeadUpperLimit 165

#define servoClawLPin 9
#define servoClawRPin 10
#define servoClawLowerLimit 20
#define servoClawUpperLimit 75

#define servoArmPin 11
#define servoArmStop 95
#define servoArmLowerLimit servoArmStop - 15
#define servoArmUpperLimit servoArmStop + 15

#define servoBasePin 12
#define servoBaseStop 90
#define servoBaseLowerLimit servoBaseStop - 25
#define servoBaseUpperLimit servoBaseStop + 25

// strings in flash
#define _string_Enable F("enabled")
#define _string_Disable F("disable")
#define _string_Unknown F("unknown")
#define _string_Intro F(" ~*~~~~~~~~~~~~~~~~~~~~*~\n ~*~ CHOKE-BOT v2.69b ~*~\n ~*~~~~~~~~~~~~~~~~~~~~*~\n")

#define _string_SetupSerial F("serial initialized")
#define _string_SetupControl F("control initialized")
#define _string_SetupServo F("servos initialized")
#define _string_SetupClaw F("claw initialized")
#define _string_SetupHead F("head initialized")
#define _string_SetupArm F("arm initialized")
#define _string_SetupBase F("base initialized")
#define _string_SetupComplete F("setup complete\n")

#define _string_ServoState F("servo state : ")
#define _string_ServoHead F("head : ")
#define _string_ServoClawL F("claw left : ")
#define _string_ServoClawR F("claw right : ")
#define _string_ServoArm F("arm : ")
#define _string_ServoBase F("base : ")

#define _string_SetupParamDelay F("\tdelay (ms) -> ")
//#define _string_SetupParamControlDelay F("\tdelay (µs) -> ")
#define _string_SetupParamControlDelay F("\tdelay (us) -> ") // no mu
#define _string_SetupParamServoMax F("\tservo max (deg) -> ")
#define _string_SetupParamServoMin F("\tservo min (deg) -> ")
#define _string_SetupParamServoPin F("\tservo pin -> ")


///
// debug stuff

//#define _string_DebugControlByte F("debug : 0b")

//#define controlByteDebugEnable
//#define serialReadDebugEnable
//#define updateServoArmDebugEnable
//#define updateServoDebugEnable


///
// libraries

#include <Servo.h>


///
// globals

byte inByte, controlByte;
byte servoMovement = 0;

Servo servoHead;
byte servoHeadPos = servoHeadLowerLimit;
boolean servoHeadEnable = true;

Servo servoClawL, servoClawR;
byte servoClawPos = servoClawLowerLimit;
boolean servoClawLEnable = true;
boolean servoClawREnable = true;

Servo servoArm;
byte servoArmPos = servoArmStop;
boolean servoArmEnable = true;


Servo servoBase;
byte servoBasePos = servoBaseStop;
boolean servoBaseEnable = true;


///
// helpers

byte servoMirror(byte in)
{
  return (servoUpperLimit - in);
}

void servoClamp(byte *pos, byte upperLimit, byte lowerLimit)
{
  if (*pos > upperLimit)
  {
    *pos = upperLimit;
  }
  else if (*pos < lowerLimit)
  {
    *pos = lowerLimit;
  }
}

void servoPrintName(byte servo)
{
  switch (servo)
  {
    case 1:
      Serial.print(_string_ServoHead);
      break;

    case 2:
      Serial.print(_string_ServoClawL);
      break;

    case 3:
      Serial.print(_string_ServoClawR);
      break;

    case 4:
      Serial.print(_string_ServoArm);
      break;

    case 5:
      Serial.print(_string_ServoBase);
      break;

    default:
      Serial.print(_string_Unknown);
      break;
  }
}

void servoPrintEnable(byte servo, boolean state)
{
  Serial.print(_string_ServoState);
  servoPrintName(servo);
  Serial.println(state ? _string_Enable : _string_Disable);
}

void servoPrintPos(byte servo, byte pos)
{
  Serial.print(_string_ServoState);
  servoPrintName(servo);
  Serial.println(pos);
}

void servoPrintParams(byte d, byte u, byte l, byte p)
{
  Serial.print(_string_SetupParamDelay);
  Serial.println(d);
  Serial.print(_string_SetupParamServoMin);
  Serial.println(l);
  Serial.print(_string_SetupParamServoMax);
  Serial.println(u);
  Serial.print(_string_SetupParamServoPin);
  Serial.println(p);
}

byte getSerialByte(boolean block)
{
  inByte = 0; // reset temp space

  Serial.flush(); // empty the serial buffer for safety

  while (block && !Serial.available())
  {
    delay(serialBlockDelay);
  }

  if (Serial.available() > 0) // we have data
  {
    inByte = Serial.read(); // get the data

#ifdef serialReadDebugEnable
    delay(serialReadDelay); // if you need to slow things down
#endif
  }

  return (inByte);
}


///
// update (loop) routines

void updateSerial()
{
  if (getSerialByte(false))
  {
    switch (inByte)
    {
      case 't':
        switch (getSerialByte(true))
        {
          case 'h':
            servoHeadEnable = !servoHeadEnable;
            servoPrintEnable(1, servoHeadEnable);
            break;

          case 'c':
            switch (getSerialByte(true))
            {
              case 'l':
                servoClawLEnable = !servoClawLEnable;
                servoPrintEnable(2, servoClawLEnable);
                break;

              case 'r':
                servoClawREnable = !servoClawREnable;
                servoPrintEnable(3, servoClawREnable);
                break;

              default:
                break;
            }
            break;

          case 'a':
            servoArmEnable = !servoArmEnable;
            servoPrintEnable(4, servoArmEnable);
            break;

          case 'b':
            servoBaseEnable = !servoBaseEnable;
            servoPrintEnable(5, servoBaseEnable);
            break;

          default:
            break;
        }
        break;

      case 's':
        switch (getSerialByte(true))
        {
          case 'a':
            servoArmPos = Serial.parseInt();
            servoPrintPos(4, servoArmPos);
            servoArm.write(servoArmPos);
            break;

          default:
            break;
        }
        break;

      default:
        break;
    }
  }
}

void updateControl()
{
  // reset buffer and prep to latch
  controlByte = 0;

  digitalWrite(controlLatchPin, LOW);
  digitalWrite(controlClockPin, LOW);

  // latch current state
  digitalWrite(controlLatchPin, HIGH);
  delayMicroseconds(controlDelay);
  digitalWrite(controlLatchPin, LOW);

  // read the first bit before we clock
  controlByte = digitalRead(controlDataPin);

  for (int i = 1; i < controlDataSize; i++)
  {
    digitalWrite(controlClockPin, HIGH); // clock in next bit
    delayMicroseconds(controlDelay); // let the old shift register catch up

    controlByte = controlByte << 1; // move data
    controlByte |= digitalRead(controlDataPin); // add new bit

    delayMicroseconds(controlDelay); // waiting to clock again...
    digitalWrite(controlClockPin, LOW);
  }

#ifdef controlByteDebugEnable
  if (controlByte != 0xFF)
  {
    Serial.print(_string_DebugControlByte);
    Serial.println(controlByte, BIN);
  }
#endif

  controlByte = ~controlByte; // NES control has inverted outputs
}

// servo stuff
void updateServoHead()
{
  // update control state
  boolean twistLeft = controlByte & controlMaskSelect;
  boolean twistRight = controlByte & controlMaskStart;

  if (twistLeft || twistRight)
  {
    // calculate move direction
    servoHeadPos += twistLeft - twistRight;

    // clamp between limits
    servoClamp(&servoHeadPos, servoHeadUpperLimit, servoHeadLowerLimit);

    // update position
    if (servoHeadEnable)
      servoHead.write(servoHeadPos);

    // increment movement tracker
    servoMovement++;
  }
}

void updateServoClaw()
{
  // update control state
  boolean openClaw = controlByte & controlMaskB;
  boolean closeClaw = controlByte & controlMaskA;

  if (openClaw || closeClaw)
  {
    // calculate move direction
    servoClawPos += openClaw - closeClaw;

    // clamp between limits
    servoClamp(&servoClawPos, servoClawUpperLimit, servoClawLowerLimit);

    // update position
    if (servoClawLEnable)
      servoClawL.write(servoClawPos);
    if (servoClawREnable)
      servoClawR.write(servoMirror(servoClawPos));

    // increment movement tracker
    servoMovement++;
  }
}

void updateServoArm() // continuous rotation server
{
  // update control state
  boolean raiseArm = controlByte & controlMaskUp;
  boolean lowerArm = controlByte & controlMaskDown;

  // if      (we are moving)        or (either button is pressed)
  if ((servoArmPos != servoArmStop) || raiseArm || lowerArm)
  {
    servoArmPos = servoArmStop; // reset to stop pos

    if (raiseArm && !lowerArm)
      servoArmPos = servoArmUpperLimit; // turn one way

    if (lowerArm && !raiseArm)
      servoArmPos = servoArmLowerLimit; // turn the other way

    // update the direction (or stop)
    if (servoArmEnable)
      servoArm.write(servoArmPos);
  }
}

void updateServoBase()
{
  // update control state
  boolean twistLeft = controlByte & controlMaskLeft;
  boolean twistRight = controlByte & controlMaskRight;

  // if      (we are moving)        or (either button is pressed)
  if ((servoBasePos != servoBaseStop) || twistLeft || twistRight)
  {
    servoBasePos = servoBaseStop; // reset to stop pos, default state

    if (twistLeft && !twistRight)
      servoBasePos = servoBaseUpperLimit; // turn one way

    if (twistRight && !twistLeft)
      servoBasePos = servoBaseLowerLimit; // turn the other way

    // update the direction (or stop)
    if (servoBaseEnable)
      servoBase.write(servoBasePos);
  }
}

void updateServo()
{
  servoMovement = 0; // reset movement flag

  updateServoHead();
  updateServoClaw();
  updateServoArm();
  updateServoBase();

  if (servoMovement)
  {
    delay(servoMoveDelay);
  }
}


/*void updateDebug()
{
  if (servoArmPos < 10)
    Serial.print('0');

  if (servoArmPos < 100)
    Serial.print('0');

  Serial.println(servoArmPos);
}*/


///
// setup routines

void setupSerial()
{
  Serial.begin(serialSpeed);

  Serial.println(_string_Intro);
  Serial.println(_string_SetupSerial);
  Serial.print(_string_SetupParamDelay);
  Serial.println(serialBlockDelay);
}

void setupControl()
{
  pinMode(controlLatchPin, OUTPUT);
  pinMode(controlClockPin, OUTPUT);
  pinMode(controlDataPin,   INPUT);

  digitalWrite(controlLatchPin, HIGH); // waiting states
  digitalWrite(controlClockPin, HIGH); // waiting states

  Serial.println(_string_SetupControl);
  Serial.print(_string_SetupParamControlDelay);
  Serial.println(controlDelay);
}

void setupServoHead()
{
  servoHead.attach(servoHeadPin);
  servoHead.write(servoHeadLowerLimit);

  Serial.println(_string_SetupHead);
  servoPrintParams(servoMoveDelay, servoHeadUpperLimit, servoHeadLowerLimit, servoHeadPin);
}

void setupServoClaw()
{
  servoClawL.attach(servoClawLPin);
  servoClawR.attach(servoClawRPin);

  servoClawL.write(servoClawLowerLimit);
  servoClawR.write(servoMirror(servoClawLowerLimit));

  Serial.println(_string_SetupClaw);
  servoPrintParams(servoMoveDelay, servoClawUpperLimit, servoClawLowerLimit, servoClawLPin);
}

void setupServoArm()
{
  servoArm.attach(servoArmPin);
  servoArm.write(servoArmPos);

  Serial.println(_string_SetupArm);
  servoPrintParams(servoMoveDelay, servoArmUpperLimit, servoArmLowerLimit, servoArmPin);
}

void setupServoBase()
{
  servoBase.attach(servoBasePin);
  servoBase.write(servoBasePos);

  Serial.println(_string_SetupBase);
  servoPrintParams(servoMoveDelay, servoBaseUpperLimit, servoBaseLowerLimit, servoBasePin);
}

void setupServo()
{
  setupServoHead();
  setupServoClaw();
  setupServoArm();
  setupServoBase();

  Serial.println(_string_SetupServo);
}


///
// arduino main stuff

void setup()
{
  setupSerial();  // get talking
  setupControl(); // prep NES controller
  setupServo();   // enable the joints

  Serial.println(_string_SetupComplete);
}


void loop()
{
  updateControl();
  updateSerial();
  updateServo();

  //updateDebug();
}
