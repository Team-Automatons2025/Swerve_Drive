#include <Wire.h>
#include <ODriveUART.h>

ODriveUART odrive1(Serial1);
ODriveUART odrive2(Serial5);
ODriveUART odrive3(Serial3);
ODriveUART odrive4(Serial8);

int pos = 0;
int buf = 20;

// up 256
// down 1024
// left 2048
// right 512

#define SlaveUno 1
uint8_t highB;
uint8_t lowB;
uint8_t psbt[5] = {0};
uint16_t digitalPsbt = 0;
int x, y, w;
int a;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial5.begin(115200);
  Serial3.begin(115200);
  Serial8.begin(115200);
  Wire1.begin();

  odrive3.closedLoopState();
  odrive4.closedLoopState();

  delay(2000);
}

void loop() {
  getdata();
  // Serial.println(digitalPsbt);
  if(digitalPsbt == 256){
    Serial.println("UP");
    a += buf;
    odrive1.setPosition(a);
    odrive2.setPosition(a);
    odrive3.setPosition(a);
    odrive4.setPosition(a);
  }

  if(digitalPsbt == 1024){
    Serial.println("DOWN");
    a -= buf;
    odrive1.setPosition(a);
    odrive2.setPosition(a);
    odrive3.setPosition(a);
    odrive4.setPosition(a);
  }

  if(digitalPsbt == 2048){
    Serial.println("LEFT");
  }

  if(digitalPsbt == 512){
    Serial.println("RIGHT");
  }

}



void getdata()
{
  Wire1.requestFrom(SlaveUno, 5);
  while ((Wire1.available() < 5));
  Wire1.readBytes(psbt, 5);
  x = map(psbt[0], 0, 255, -127, 128);
  y = map(psbt[1], 0, 255, 127, -128);
  w = map(psbt[2], 0, 255, -40, 40);


  digitalPsbt = psbt[4] | psbt[3] << 8;
  if (digitalPsbt != 0)
    Serial.println(digitalPsbt);


  if (x != 0 || y != 0) {
       Serial.print(String("x: ") + x);
       Serial.println(String("\t\ty: ") + y);
  }
   if (w != 0) Serial.println(String("\t\tw: ") + w);
  //    else  Serial.println(String("w: ") + w);
}
