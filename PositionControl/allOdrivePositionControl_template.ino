#include <ODriveUART.h>

ODriveUART odrive1(Serial1);
ODriveUART odrive2(Serial5);
ODriveUART odrive3(Serial3);
ODriveUART odrive4(Serial8);

int pos = 20;

void setup() {

  Serial.begin(115200);
  Serial1.begin(115200);
  Serial5.begin(115200);
  Serial3.begin(115200);
  Serial8.begin(115200);

  odrive3.closedLoopState();
  odrive4.closedLoopState();

  delay(2000);

}

void loop() {

  odrive1.setPosition(pos);
  odrive2.setPosition(pos);
  odrive3.setPosition(pos);
  odrive4.setPosition(pos);

}


