// 
#include<Arduino.h>
#include<PS4BT.h>
#include<usbhub.h>
#include<Wire.h>
#ifdef dobogusinclude
#include<spi4teensy3.h>
#endif
#include<SPI.h>
#define SlaveUno 1

USB Usb;
BTD Btd(&Usb);
PS4BT PS4(&Btd,PAIR);


void RequestEvent();

uint16_t psbt = 0;
uint16_t psbt1 = 0;
uint8_t analogPsbt[5] = {127, 127, 127, 0, 0}; //(x,y,w);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); // Halt
  }
  Serial.print(F("\r\nPS4 Bluetooth Library Started"));
  Wire.begin(SlaveUno);
  Wire.onRequest(RequestEvent);

}

void loop() {
  Usb.Task();

  if ((PS4.connected())) {
    //Serial.println("Connected");

    if (PS4.getAnalogHat(LeftHatX) > 140 || PS4.getAnalogHat(LeftHatX) < 115 || PS4.getAnalogHat(LeftHatY) > 140 || PS4.getAnalogHat(LeftHatY) < 115)
    {
      analogPsbt[0] = PS4.getAnalogHat(LeftHatX);
      analogPsbt[1] = 255 - PS4.getAnalogHat(LeftHatY);
      Serial.print(F("\r\nLeftHatX: "));
      Serial.print(analogPsbt[0]);
      Serial.print(F("\tLeftHatY: "));
      Serial.println(analogPsbt[1]);
    }
    else
    {
      analogPsbt[0] = 127;
      analogPsbt[1] = 127;
    }
    if (PS4.getAnalogButton(L2) || PS4.getAnalogButton(R2)) {
      analogPsbt[2] = map((PS4.getAnalogButton(L2) - PS4.getAnalogButton(R2)), -255, 255, 0, 255);
      Serial.print(F("\r\nL2: "));
      Serial.print(PS4.getAnalogButton(L2));
      Serial.print(F("\tR2: "));
      Serial.print(PS4.getAnalogButton(R2));
      Serial.print(F("\tW: "));
      Serial.println(analogPsbt[2]);
    }
    if (!(PS4.getAnalogButton(L2) || PS4.getAnalogButton(R2)))
    {
      analogPsbt[2] = 127;
    }
    
    if (PS4.getButtonClick(TRIANGLE)) {
      //            Serial.println("\r\nTriangle");
      psbt |= 1 << 0;
    }
    else if (PS4.getButtonClick(CIRCLE)) {
      //            Serial.println("\r\nCircle");
      psbt |= 1 << 1;
    }
    else if (PS4.getButtonClick(CROSS)) {
      //            Serial.println("\r\nCross");
      psbt |= 1 << 2;
    }
    else if (PS4.getButtonClick(SQUARE)) {
      //            Serial.println("\r\nSquare");
      psbt |= 1 << 3;
    }
    else if (PS4.getButtonClick(L1)) {
      //            Serial.println("\r\nL1");
      psbt |= 1 << 4;
    }
    else if (PS4.getButtonClick(R1)) {
      //            Serial.println("\r\nR1");
      psbt |= 1 << 5;
    }
    else if (PS4.getButtonClick(SHARE)) {
      //            Serial.println("\r\nShare");
      psbt |= 1 << 6;
    }
    else if (PS4.getButtonClick(OPTIONS)) {
      //            Serial.println("\r\nOptions");
      psbt |= 1 << 7;
    }
    else if (PS4.getButtonClick(UP)) {
      //            Serial.print(F("\r\nUP"));
      psbt |= 1 << 8;
    }
    else if (PS4.getButtonClick(RIGHT)) {
//      Serial.print(F("\r\nRight"));
      psbt |= 1 << 9;
    }
    else if (PS4.getButtonClick(DOWN)) {
      //            Serial.print(F("\r\nDown"));
      psbt |= 1 << 10;
    }
    else if (PS4.getButtonClick(LEFT)) {
      //            Serial.print(F("\r\nLeft"));
      psbt |= 1 << 11;
    }
    else if (PS4.getButtonClick(L3)) {
      //            Serial.print(F("\r\nL3"));
      psbt |= 1 << 12;
    }
    else if (PS4.getButtonClick(R3)) {
      //            Serial.print(F("\r\nR3"));
      psbt |= 1 << 13;
    }
    else if (PS4.getButtonClick(TOUCHPAD)) {
      //            Serial.print(F("\r\nTouchPad"));
      psbt |= 1 << 14;
    }
    else if (PS4.getButtonClick(PS)) {
      //            Serial.print(F("\r\nPS"));
      psbt |= 1 << 15;
    }
  }
  else {
    psbt = 0;
    analogPsbt[0] = 127;
    analogPsbt[1] = 127;
    analogPsbt[2] = 127;
  }

}
void RequestEvent()
{
  analogPsbt[3] = highByte(psbt);
  analogPsbt[4] = lowByte(psbt);
  Wire.write(analogPsbt, 5);
  if (psbt != 0)
    Serial.println(psbt);
  psbt = 0;

}