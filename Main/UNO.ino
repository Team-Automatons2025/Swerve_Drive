/*
 Example sketch for the PS4 Bluetooth library - developed by Kristian Lauszus
 For more information visit my blog: http://blog.tkjelectronics.dk/ or
 send me an e-mail:  kristianl@tkjelectronics.com
 */
#include <Wire.h>
#include <PS4BT.h>
#include <usbhub.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

USB Usb;
//USBHub Hub1(&Usb); // Some dongles have a hub inside
BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so

/* You can create the instance of the PS4BT class in two ways */
// This will start an inquiry and then pair with the PS4 controller - you only have to do this once
// You will need to hold down the PS and Share button at the same time, the PS4 controller will then start to blink rapidly indicating that it is in pairing mode
PS4BT PS4(&Btd, PAIR);

// After that you can simply create the instance like so and then press the PS button on the device
//PS4BT PS4(&Btd);

// bool printAngle, printTouch;
// uint8_t oldL2Value, oldR2Value;

byte  psbt[3] = {127,127,127}; 
void onRequest(){
  Wire.write(psbt,3);
}

void setup() {
  Wire.begin(8);
  Serial.begin(115200);
  #if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); // Halt
  }
  Serial.print(F("\r\nPS4 Bluetooth Library Started"));
  Wire.onRequest(onRequest);
}
void loop() {
  Usb.Task();

  if (PS4.connected()) {
    if (PS4.getAnalogHat(LeftHatX) > 137 || PS4.getAnalogHat(LeftHatX) < 117 || PS4.getAnalogHat(LeftHatY) > 137 || PS4.getAnalogHat(LeftHatY) < 117 ) {//|| PS4.getAnalogHat(RightHatX) > 137 || PS4.getAnalogHat(RightHatX) < 117 || PS4.getAnalogHat(RightHatY) > 137 || PS4.getAnalogHat(RightHatY) < 117
      
      // Serial.print(PS4.getAnalogHat(LeftHatX));
      psbt[0] = PS4.getAnalogHat(LeftHatX);
     
      
      
      // Serial.print(PS4.getAnalogHat(LeftHatY));
      psbt[1] = PS4.getAnalogHat(LeftHatY);
     

      // Serial.print(F("\tRightHatX: "));
      // Serial.print(PS4.getAnalogHat(RightHatX));
      // Serial.print(F("\tRightHatY: "));
      // Serial.print(PS4.getAnalogHat(RightHatY));
    }
    else {
      psbt[0]=127;
      psbt[1]=127;
    }

    if ((PS4.getAnalogButton(L2) || PS4.getAnalogButton(R2))) { // These are the only analog buttons on the PS4 controller
      
      psbt[2] = map(PS4.getAnalogButton(L2) - PS4.getAnalogButton(R2),-255,255,255,0);
      
      
      
     
    }
    else{
      psbt[2] = 127;
    }
   
    // if (PS4.getAnalogButton(L2) != oldL2Value || PS4.getAnalogButton(R2) != oldR2Value) // Only write value if it's different
    //   PS4.setRumbleOn(PS4.getAnalogButton(L2), PS4.getAnalogButton(R2));
    // oldL2Value = PS4.getAnalogButton(L2);
    // oldR2Value = PS4.getAnalogButton(R2);

    // if (PS4.getButtonClick(PS)) {
    //   Serial.print(F("\r\nPS"));
    //   PS4.disconnect();
    // }
    // else {
    //   if (PS4.getButtonClick(TRIANGLE)) {
    //     Serial.print(F("\r\nTriangle"));
    //     PS4.setRumbleOn(RumbleLow);
    //   }
    //   if (PS4.getButtonClick(CIRCLE)) {
    //     Serial.print(F("\r\nCircle"));
    //     PS4.setRumbleOn(RumbleHigh);
    //   }
    //   if (PS4.getButtonClick(CROSS)) {
    //     Serial.print(F("\r\nCross"));
    //     PS4.setLedFlash(10, 10); // Set it to blink rapidly
    //   }
    //   if (PS4.getButtonClick(SQUARE)) {
    //     Serial.print(F("\r\nSquare"));
    //     PS4.setLedFlash(0, 0); // Turn off blinking
    //   }

    //   if (PS4.getButtonClick(UP)) {
    //     Serial.print(F("\r\nUp"));
    //     PS4.setLed(Red);
    //   } if (PS4.getButtonClick(RIGHT)) {
    //     Serial.print(F("\r\nRight"));
    //     PS4.setLed(Blue);
    //   } if (PS4.getButtonClick(DOWN)) {
    //     Serial.print(F("\r\nDown"));
    //     PS4.setLed(Yellow);
    //   } if (PS4.getButtonClick(LEFT)) {
    //     Serial.print(F("\r\nLeft"));
    //     PS4.setLed(Green);
    //   }

    //   if (PS4.getButtonClick(L1))
    //     Serial.print(F("\r\nL1"));
    //   if (PS4.getButtonClick(L3))
    //     Serial.print(F("\r\nL3"));
    //   if (PS4.getButtonClick(R1))
    //     Serial.print(F("\r\nR1"));
    //   if (PS4.getButtonClick(R3))
    //     Serial.print(F("\r\nR3"));

    //   if (PS4.getButtonClick(SHARE))
    //     Serial.print(F("\r\nShare"));
    //   if (PS4.getButtonClick(OPTIONS)) {
    //     Serial.print(F("\r\nOptions"));
    //     printAngle = !printAngle;
    //   }
    //   if (PS4.getButtonClick(TOUCHPAD)) {
    //     Serial.print(F("\r\nTouchpad"));
    //     printTouch = !printTouch;
    //   }

    //   if (printAngle) { // Print angle calculated using the accelerometer only
    //     Serial.print(F("\r\nPitch: "));
    //     Serial.print(PS4.getAngle(Pitch));
    //     Serial.print(F("\tRoll: "));
    //     Serial.print(PS4.getAngle(Roll));
    //   }

    //   if (printTouch) { // Print the x, y coordinates of the touchpad
    //     if (PS4.isTouching(0) || PS4.isTouching(1)) // Print newline and carriage return if any of the fingers are touching the touchpad
    //       Serial.print(F("\r\n"));
    //     for (uint8_t i = 0; i < 2; i++) { // The touchpad track two fingers
    //       if (PS4.isTouching(i)) { // Print the position of the finger if it is touching the touchpad
    //         Serial.print(F("X")); Serial.print(i + 1); Serial.print(F(": "));
    //         Serial.print(PS4.getX(i));
    //         Serial.print(F("\tY")); Serial.print(i + 1); Serial.print(F(": "));
    //         Serial.print(PS4.getY(i));
    //         Serial.print(F("\t"));
    //       }
    //     }
    //   }
    // }
  }
  else{
    psbt[0]=127;
    psbt[1]=127;
    psbt[2]=127;
  }
  Serial.print(F("\tLeftHatX: "));
  Serial.print(psbt[0]);
  Serial.print(F("\tLeftHatY: "));
  Serial.print(psbt[1]);
  Serial.print("\toMega: ");
  Serial.println(psbt[2]);

}
