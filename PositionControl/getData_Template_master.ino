#include <Wire.h>

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

void setup() {
  Serial.begin(115200);
  Wire1.begin();
}

void loop() {
  getdata();
  // Serial.println(digitalPsbt);
  if(digitalPsbt == 256){
    Serial.println("UP");
  }

  if(digitalPsbt == 1024){
    Serial.println("DOWN");
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