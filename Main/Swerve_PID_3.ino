#include <Wire.h>
#include <Encoder.h>
#include <ODriveArduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire2);

#define r_pwm1 9
#define l_pwm1 8

#define l_pwm2 7
#define r_pwm2 6
#define l_pwm3 5
#define r_pwm3 4
#define l_pwm4 3
#define r_pwm4 2


int velocity;
int axis = 0;


//Encoder Pins
int enco1A = 26;
int enco1B = 27;
//Encoder Pins
int enco2A = 33;
int enco2B = 32;
//Encoder Pins
int enco3A = 38;
int enco3B = 37;
//Encoder Pins
int enco4A = 31;
int enco4B = 30;


Encoder enco1(enco1A, enco1B);
Encoder enco2(enco2A, enco2B);
Encoder enco3(enco3A, enco3B);
Encoder enco4(enco4A, enco4B);

ODriveArduino odrive1(Serial1);
ODriveArduino odrive2(Serial5);
ODriveArduino odrive3(Serial3);
ODriveArduino odrive4(Serial8);

long cpr1 = 465098;
long cpr2 = 467189;
long cpr3 = 468765;
long cpr4 = 466072;

int pwm = 110;

byte psData[3];
int leftX;
int leftY;
int oMega;
int o;
int mg;
int theta;

int oMegaP;

int flag = 0;
int pidFlag = 0;
int sPidFlag = 0;

float angle1, velocity1;
float angle2, velocity2;
float angle3, velocity3;
float angle4, velocity4;

int maxV = 35;

float botAngle = 0;
float botTargetAngle = 0;
float botErrorAngle;
float diffError;
int integral;
int prevError;
long prevT, currT;

int angular_limit = 60;
float sMultiplier = 2;

void getData() {
  // Serial.println("hi");
  Wire1.requestFrom(8, 3);
  
  while (Wire1.available() > 2) {

    Wire1.readBytes(psData, 3);
    leftX = psData[0];
    leftY = psData[1];
    oMega = psData[2];
    leftX = map(leftX, -1, 255, -127, 127);
    leftY = map(leftY, -1, 255, 127, -127);
    oMega = map(oMega, 0, 255, 127, -127);
  }
  // Serial.print(leftX);
  // Serial.print(" ");
  // Serial.print(leftY);
  // Serial.print(" ");
  // Serial.println(oMega);
  mg = sqrt(pow(leftX, 2) + pow(leftY, 2));
  theta = atan2(leftY,leftX)*180/M_PI;
  theta = (theta +360)%360;
   
  o = oMega;
  if (mg > 30 || abs(o) > 10) {
    if(mg>30){
      mg = map(mg,30,127,0,80);
    }
    flag = 1;
    
  } else {
    mg = 0;
    flag = 0;
  }
  if (abs(o) > 10) {
    botTargetAngle = botAngle;
  }
  else{
    oMega = 0;
  }

  // Serial.print(mg);
  // Serial.print(" ");
  // Serial.println(theta);
  // o = map(o, 0, 126, 0, angular_limit);
  // o = constrain(o, -angular_limit, angular_limit);
  // Serial.println(o);
  // oMega *= o/o;
  // Serial.println(oMega);
}
void angularPID(double kp, double ki, double kd) {

  botErrorAngle = botTargetAngle - botAngle;  // 0 // 350 // -350

  if (botErrorAngle < -180) {
    botErrorAngle += 360;  // 10
  } else if (botErrorAngle > 180) {
    botErrorAngle -= 360;  //-10
  }
  // Serial.println(botErrorAngle);
  currT = micros();  // 0 // 9
  diffError = abs(botErrorAngle) - abs(prevError);
  // dt = currT - prevT;
  integral += botErrorAngle;
  oMega = -(botErrorAngle * kp + ((diffError * kd)) + integral * ki);
  prevError = botErrorAngle;
  prevT = micros();  // 1

  if (botErrorAngle < 5 && botErrorAngle > -5) {
    pidFlag = 0;
    sPidFlag = 1;
    oMegaP = oMega;
    if (abs(o) < 10 && flag ==0) {
      oMega = 0;
      pidFlag = 0;
    } 
    else{
      oMegaP = 0;
      // pidFlag = 1;
      sPidFlag = 0;
    }
    // if(flag == 1){
    //   oMega = oMegaP;
    //   oMegaP = 0;
    // }
  }
  else{
    pidFlag = 1;
    sPidFlag = 0;
    oMegaP = 0;
  }

  if(abs(o) > 10){
      oMega = map(o, 0, 126, 0, angular_limit);
  }
  // } else if (botErrorAngle < -180) {
  //   pidFlag = 1;
  //   sPidFlag = 0;
  //   oMegaP = 0;
  // } else if (botErrorAngle > 180) {
  //   pidFlag = 1;
  //   sPidFlag = 0;
  //   oMegaP = 0;
  // }
  
  oMega = constrain(oMega, -angular_limit, angular_limit);
  oMegaP = constrain(oMegaP, -angular_limit, angular_limit);

  Serial.print((String) "bAng:" + botAngle + " bAngEr:" + botErrorAngle + " pT:" + (botErrorAngle * kp) + " iT:" + (integral * ki) + " dT:" + ((diffError * kd)) + " oMega" + oMega
  + " omegaP:" + oMegaP + " sP:" + sPidFlag + " pf:" + pidFlag + " f:" + flag);
}
class vctor {
private:
  int magnitude;
  double angle;
public:
  float x_component;
  float y_component;

  void update(int magnitudex, int anglex) {
    magnitude = magnitudex;
    angle = anglex;
  }
  void resolve() {
    angle = angle * M_PI / 180;
    x_component = magnitude * cos(angle);
    y_component = magnitude * sin(angle);
    // Serial.println((String)"Resolve:\tangle: "+angle+"\tx_component: "+x_component+"\ty_component: "+y_component);
  }
};
byte x;


class Module {
private:
  uint8_t index;
  int currentAngle;
  long ticks;
  long cpr;
  uint8_t axis = 0;
  uint8_t pwm;
  uint8_t r_pwm;
  uint8_t l_pwm;
  float X, Y;
  int mag;
  int ang;
  int velocity;
  int shortestAngle = 0;
  int8_t dir = 1;
  vctor h,w;
  Encoder &enco;
  ODriveArduino &odrive;

public:
  Module(uint8_t indexx, uint8_t r_pwmx, uint8_t l_pwmx, Encoder &encox, ODriveArduino &odrivex, int pwmx, long cprx)
    : enco(encox), odrive(odrivex) {
    index = indexx;
    cpr = cprx;
    pwm = pwmx;
    r_pwm = r_pwmx;
    l_pwm = l_pwmx;
  }

  void calc() {
    // Serial.println(mg);
    h.update(mg, theta + botAngle);       //      
    w.update(oMega, 45 + (90 * index));  // 135/225/315/45 with respect to X axis
    // Serial.print("X ");
    h.resolve();
    w.resolve();

    X = h.x_component + w.x_component;
    Y = h.y_component + w.y_component;
    // Serial.println((String)"X"+index+": "+X+"\tY"+index+": "+Y);
    mag = sqrt(pow(X, 2) + pow(Y, 2));
    ang = atan2(X, Y) * 180 / M_PI;
    ang = (ang + 360) % 360;
    // Serial.println((String)"mag:"+mag+"\t ang"+ang);
  }

  float statutoryPID(float velocity) {
    if ((shortestAngle <= 45) || (shortestAngle >= 135 && shortestAngle <= 225) || (shortestAngle >= 315)) {

      if (index == 1 || index == 4) {
        velocity = sMultiplier*(oMegaP * cos(shortestAngle * M_PI / 180));  // Convert angle to radians
      } 
      else if (index == 2 || index == 3) {
        velocity = -sMultiplier*(oMegaP * cos(shortestAngle * M_PI / 180));
      }
    }

    else if ((shortestAngle > 45 && shortestAngle < 135)||(shortestAngle >225 && shortestAngle < 315)) {
        if (index == 1 || index == 2) {
          velocity = -sMultiplier*(oMegaP * sin(shortestAngle * M_PI / 180));  // Convert angle to radians
        } 
        else if (index == 3 || index == 4) {
         velocity = sMultiplier*(oMegaP * sin(shortestAngle * M_PI / 180));
        }
      }
    return velocity; 
  }

  void shortAngDir() {

    int CA, TAO, pArc, nArc, pArcO, nArcO, outputAngle, TA = ang;
    //180 // 180
    if ((flag == 1 || pidFlag == 1 )) {
      CA = shortestAngle;                    //40 // 0
      TAO = TA < 180 ? TA + 180 : TA - 180;  //0 // 0
      pArc = (TA - CA);                      //140 180
      pArc = pArc < 0 ? pArc + 360 : pArc;
      nArc = (TAO - CA);                    // -40 // 0
      nArc = nArc < 0 ? nArc + 360 : nArc;  // 320 // 0
      pArcO = (360 - abs(pArc)) % 360;      // 220 // 180
      nArcO = (360 - abs(nArc)) % 360;      // 40  //

      // Serial.print("pArc:");
      // Serial.println(pArc);
      // Serial.print("nArc:");
      // Serial.println(nArc);
      // Serial.print("pARcO:");
      // Serial.println(pArcO);
      // Serial.print("nArcO:");
      // Serial.println(nArcO);

      if (abs(pArc) <= abs(nArc) && abs(pArc) <= abs(pArcO) && abs(pArc) <= abs(nArcO)) {
        outputAngle = pArc;
        dir = 1;
      } else if (abs(nArc) < abs(pArc) && abs(nArc) <= abs(pArcO) && abs(nArc) <= abs(nArcO)) {
        outputAngle = nArc;
        dir = -1;
      } else if (abs(pArcO) <= abs(nArc) && abs(pArcO) < abs(pArc) && abs(pArcO) < abs(nArcO)) {
        outputAngle = -pArcO;
        dir = 1;
      } else if (abs(nArcO) <= abs(pArc) && abs(nArcO) < abs(pArcO) && abs(nArcO) < abs(nArc)) {
        outputAngle = -nArcO;
        dir = -1;
      } else {
        outputAngle = 0;
      }

      shortestAngle = CA + outputAngle;             // 0
      shortestAngle = (shortestAngle + 360) % 360;  // 0
    }
    ang = shortestAngle;

    // Serial.print("shortestAngle:");
    // Serial.print(shortestAngle);//0
    // // Serial.print("\t\t\tvAngle:");
    // // Serial.print(vAngle);
    // // Serial.print("\t");
    // Serial.print("\t\t\tdir:");
    // Serial.println(dir);// -1
  }


  void compute(float *anglex, float *velocity) {
    calc();
    shortAngDir();
    *anglex = ang;
    *velocity = mag * 0.4 * dir;
    if(sPidFlag == 1 && flag==0){
      *velocity = statutoryPID(*velocity);
    }
    // Serial.println(*velocity);
  }

  int getAngle() {
    ticks = enco.read();
    ticks = ticks % cpr;
    ticks = ticks < 0 ? ticks + cpr : ticks;
    currentAngle = map(ticks, 0, cpr, 0, 360);
    currentAngle = currentAngle % 360;
    // Serial.println(currentAngle);
    return currentAngle;
  }

  void setAngle(int ta) {
    int P_Arc, N_Arc;
    P_Arc = ta - getAngle();
    P_Arc = P_Arc < 0 ? P_Arc + 360 : P_Arc;
    N_Arc = 360 - P_Arc;

    if (N_Arc == 360) {
      analogWrite(r_pwm, 0);
      analogWrite(l_pwm, 0);
      // velocity_flag |= 1<<(index-1);
    } else if ((P_Arc < N_Arc) && ((N_Arc - P_Arc) < (360 - 3))) {
      analogWrite(l_pwm, 0);
      analogWrite(r_pwm, pwm);
      // velocity_flag &= ~1<<(index-1);
    } else if ((N_Arc < P_Arc) && ((P_Arc - N_Arc) < (360 - 3))) {
      analogWrite(r_pwm, 0);
      analogWrite(l_pwm, pwm);
      // velocity flag &= ~1<<(index - 1);
    } else {
      analogWrite(r_pwm, 0);
      analogWrite(l_pwm, 0);

      // velocity_flag |= 1<<(index-1);
    }
  }
  void checkOdrive() {
    Serial.println("Waiting for ODrive");
    Serial.print(index);
    while (odrive.getState() == AXIS_STATE_UNDEFINED) {
      delay(10);
    }

    Serial.print("DC voltage: ");
    Serial.println(odrive.getParameterAsFloat("vbus_voltage"));

    Serial.println("Enabling closed loop control...");
    while (odrive.getState() != AXIS_STATE_CLOSED_LOOP_CONTROL) {
      odrive.clearErrors();
      odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
      delay(10);
    }
  }
  void setVelocity(int vel) {
    if (flag == 0 && pidFlag == 0 && sPidFlag == 0) {
      odrive.SetVelocity(axis, 0);
    } else if (flag == 1 || pidFlag == 1 || sPidFlag == 1 ) {
      odrive.SetVelocity(axis, vel);
    }
  }
  void actuate(int targetAngle, int velocity) {
    setAngle(targetAngle);
    setVelocity(velocity);
  }
};

Module Module1(1, r_pwm1, l_pwm1, enco1, odrive1, pwm, cpr1);
Module Module2(2, r_pwm2, l_pwm2, enco2, odrive2, pwm, cpr2);
Module Module3(3, r_pwm3, l_pwm3, enco3, odrive3, pwm, cpr3);
Module Module4(4, r_pwm4, l_pwm4, enco4, odrive4, pwm, cpr4);

void setup() {
  Wire1.begin();

  Serial.begin(115200);

  Serial1.begin(115200);
  Serial5.begin(115200);
  Serial3.begin(115200);
  Serial8.begin(115200);
  // while (!Serial) delay(10);  // wait for serial port to open!

  /* Initialise the sensor */
  bno.begin();
  if (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }

  delay(1000);

  bno.setExtCrystalUse(true);


  // Module1.checkOdrive();
  // Module2.checkOdrive();
  Module3.checkOdrive();
  Module4.checkOdrive();

  pinMode(l_pwm1, OUTPUT);
  pinMode(r_pwm1, OUTPUT);
  pinMode(l_pwm2, OUTPUT);
  pinMode(r_pwm2, OUTPUT);
  pinMode(l_pwm3, OUTPUT);
  pinMode(r_pwm3, OUTPUT);
  pinMode(l_pwm4, OUTPUT);
  pinMode(r_pwm4, OUTPUT);
  // delay(500);
}
int angle;
void loop() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  botAngle = euler.x();

  // Serial.println(botAngle);
  getData();
  

  angularPID(1.25, 0.0000,1);

  Module1.compute(&angle1, &velocity1);
  Module2.compute(&angle2, &velocity2);
  Module3.compute(&angle3, &velocity3);
  Module4.compute(&angle4, &velocity4);

  velocity1 = constrain(velocity1, -maxV, maxV);
  velocity2 = constrain(velocity2, -maxV, maxV);
  velocity3 = constrain(velocity3, -maxV, maxV);
  velocity4 = constrain(velocity4, -maxV, maxV);

  Serial.println((String) " Angle1:" + angle1 + " velocity1:" + velocity1 + "\tAngle2:" + angle2 + " velocity2:" + velocity2 
  + "\tAngle3:" + angle3 + " velocity3:" + velocity3 + "\tAngle4:" + angle4 + " velocity:" + velocity4);

  Module1.actuate(angle1, velocity1);
  Module2.actuate(angle2, velocity2);
  Module3.actuate(angle3, velocity3);
  Module4.actuate(angle4, velocity4); 

  leftX = 0;
  leftY = 0;
  oMega = 0;
  // Serial.println(enco4.read());
}
