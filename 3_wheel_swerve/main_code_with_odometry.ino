#include "bts.h"
#include <Wire.h>
#include <Encoder.h>
#include <ODriveSwerve.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire2);

#define pwm 90
#define vel_const 0.4
#define dia 7

#define r_pwm1 9
#define l_pwm1 8
#define l_pwm2 7
#define r_pwm2 6
#define l_pwm3 5
#define r_pwm3 4

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

bts bts1(r_pwm1,l_pwm1);
bts bts2(r_pwm2,l_pwm2);
bts bts3(r_pwm3,l_pwm3);

Encoder enco1(enco1A, enco1B);
Encoder enco2(enco2A, enco2B);
Encoder enco3(enco3A, enco3B);

ODriveSwerve odrive1(Serial1);
ODriveSwerve odrive2(Serial5);
ODriveSwerve odrive3(Serial3);

long cpr1 = 465098;
long cpr2 = 467189;
long cpr3 = 468765;

int ordive_cpr = 1024;
float wheel_circumfernce = PI * dia;

float Rx,Ry;

float Px1,Py1;
float Px2,Py2;
float Px3,Py3;

float o1_x = 0     ,o1_y = 25;
float o2_x = 12.5  ,o2_y = -21.65;
float o3_x = -12.5 ,o3_y = -21.65;

byte psData[3];
int leftX;
int leftY;
int oMega;
int o;
int mg;
int theta;
int oMegaP;

int flag = 0;
int PidFlag = 0;
int sPidFlag = 0;
int oMegaFlag = 0;

float angle1, velocity1;
float angle2, velocity2;
float angle3, velocity3;

int maxV = 35;
double vel;

float botAngle = 0;
float botTargetAngle = 0;
float botErrorAngle;
float derivative_error;
int integral_error;
int prevError;
long prevT, currT;

int angular_limit = 60;
float sMultiplier = 1.5;

void getData() {

// reciving data from PS4 ---(Bluetooth)---> UNO(HOSTSHIELD)(SPI) ---(I2C)---> Teensy ;
  Wire1.requestFrom(8, 3);
  while (Wire1.available()>2) {
    
    Wire1.readBytes(psData, 3);
    leftX = psData[0];
    leftY = psData[1];
    oMega = psData[2];
    // leftX = map(leftX, -1, 255, -127, 127);
    // leftY = map(leftY, -1, 255, 127, -127);
    // oMega = map(oMega,  0, 255, -127, 127);
  }
    // calculating angle of ps4 joystick using its X and Y co-ordinate;
  theta = (atan2(leftX,leftY) * RAD_TO_DEG);
  theta = (theta + 360)%360;
  // Serial.println(theta);
  // Serial.println(oMega);
  // calculating magnitude of ps4 joystick using its X and Y co-ordinate;
  mg = sqrt(pow(leftX,2)+pow(leftY,2));
  o = oMega;
  if (mg > 30 || abs(o) > 10) {
    if(mg>30){
      flag = mg>30?1:0;
      mg*=flag;
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
  
}

void angularPID(double kp, double ki, double kd) {

  botErrorAngle = botTargetAngle - botAngle;  // 0 // 350 // -350

  if (botErrorAngle < -180) {
    botErrorAngle += 360;  // 10
  } else if (botErrorAngle > 180) {
    botErrorAngle -= 360;  //-10
  }
  
  currT = millis();

  derivative_error = (botErrorAngle - prevError)/(currT - prevT);
  integral_error += botErrorAngle; 
  oMega = -(kp * botErrorAngle + ki * integral_error + kd * derivative_error);
  prevError = botErrorAngle;

  prevT = currT;
  
  if (botErrorAngle < 10 && botErrorAngle > -10) {
    PidFlag = 0;
    sPidFlag = 1;
    oMegaP = oMega;
    if (abs(o) < 10 && flag ==0) {
      oMega = 0;
    } 
    else{
      oMega = map(o, 0, 126, 0, angular_limit);
      oMegaP = 0;
    }
    // if(flag == 1){
    //   oMega = oMegaP;
    //   oMegaP = 0;
    // }
  }
  else{
    PidFlag = 1;
    sPidFlag = 0;
    oMegaP = 0;
  }

  oMega = constrain(oMega, -angular_limit, angular_limit);
  oMegaP = constrain(oMegaP, -angular_limit, angular_limit);
}

void res_co_ordinate(){
  Rx = Px1 + Px2 + Px3;
  Ry = Py1 + Py2 + Py3;
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

class Module{
private:
  int CA = 0;
  float X, Y;
  int TA = 0;
  int mag;
  int ang;
  int dir;
  int shortestAngle = 0;
  vctor h,w;
  int  dis;
  
  int index;
  bts& motor;
  Encoder& enco;
  ODriveSwerve& odrive;
  long cpr;

  float origin_x, origin_y;
  float x = 0, y = 0;
  double user_view_ang = 0;


public:
  int target_angle;
  int angtemp;
  // Module(int index, int Rpwm, int Lpwm, Encoder& enco, ODriveSwerve& odrive, long cpr) : enco(enco), odrive(odrive){
  Module(int index, bts& motor, Encoder& enco, ODriveSwerve& odrive, long cpr, float origin_x, float origin_y) :
         index(index), motor(motor), enco(enco), odrive(odrive), cpr(cpr), origin_x(origin_x), origin_y(origin_y) {
    // this-> Rpwm = Rpwm;
    // this-> Lpwm = Lpwm;
      // this-> cpr = cpr;
  }

  int distance(){
    dis = odrive.getPosition();
    dis = wheel_circumfernce/ordive_cpr * dis;

    return dis;
  }

  void co_ordinate(int bot_angle, int module_angle, float* x2, float* y2){
    user_view_ang = (bot_angle + module_angle) % 360;
    dis = distance();
    
    float sin_a = sin(user_view_ang * M_PI / 180);
    float cos_a = cos(user_view_ang * M_PI / 180);

    *x2 = (dir * dis * sin_a) + x;
    *y2 = (dir * dis * cos_a) + y;

    x = *x2;
    y = *y2;

    *x2 += origin_x;
    *y2 += origin_y;
  }


  void calc() {
    // Serial.println(mg);
    // Serial.print(theta - botAngle);
    h.update(mg, theta - botAngle);       //      
    w.update(oMega, 270 + (60 * (index - 1) ));  // 270/330/390(30)
    h.resolve();
    w.resolve();
    // Serial.println();
    X = h.x_component + w.x_component;
    Y = h.y_component + w.y_component;
    // Serial.println((String)"X"+index+": "+X+"\tY"+index+": "+Y);
    mag = sqrt(pow(X, 2) + pow(Y, 2));
    ang = atan2(Y, X) * 180 / M_PI;
    ang = (ang + 360) % 360;
    // Serial.println((String)"      mag:"+mag+"\t ang"+ang);
  }

  int statutoryPID(float velocity) {
      if ((target_angle <= 45) || (target_angle >= 135 && target_angle <= 225) || (target_angle >= 315)) {
        if (index == 1 || index == 4) {
          velocity = sMultiplier*(oMegaP * cos(target_angle * M_PI / 180));  // Convert angle to radians
        } 
        else if (index == 2 || index == 3) {
          velocity = -sMultiplier*(oMegaP * cos(target_angle * M_PI / 180));
        }
      }

      else if ((target_angle > 45 && target_angle < 135)||(target_angle >225 && target_angle < 315)) {
          if (index == 1 || index == 2) {
            velocity = -sMultiplier*(oMegaP * sin(target_angle * M_PI / 180));  // Convert angle to radians
          } 
          else if (index == 3 || index == 4) {
            velocity = sMultiplier*(oMegaP * sin(target_angle * M_PI / 180));
          }
        } 
    return velocity;
  }

  void compute(float* angle, float* velocity){
    calc();
    short_angle();
    *angle = ang;    
    *velocity = mag * vel_const * dir;

    if (sPidFlag == 1 && flag == 0) {
      *velocity = statutoryPID(*velocity);
    }

    // Serial.println((String)*velocity + " " + *angle);
  }

  // void motor(int r, int l){
  //   analogWrite(Rpwm, r);
  //   analogWrite(Lpwm, l);
  // }

  int t2a(){
    long tick = enco.read();
    tick = tick % cpr;
    tick = tick < 0 ? tick + cpr : tick;
    CA = map(tick, 0, cpr, 0, 360);
    return CA % 360;
  }

  void short_angle() {
    int CA, TA = ang;
    if ((flag == 1 || PidFlag == 1 )){
      CA = target_angle;

      if(abs(TA - CA) <=90 || 270 <= abs(TA - CA)){
        target_angle = TA;
        dir = 1;
      }
      else{ 
        target_angle = TA - 180;
        target_angle = target_angle < 0 ? target_angle + 360 : target_angle;
        dir = -1;
      }
    // shortestAngle = CA + target_angle;           // 0
    target_angle = (target_angle + 360) % 360;  // 0
    }
    ang = target_angle;
  }

  // setting the shortest angle by breaking the 360 barrier;
  void set_angle(int target_angle){
    int pArc, nArc;
    pArc = target_angle - t2a();
    pArc = pArc >= 0 ? pArc : 360 + pArc;

    nArc = 360 - pArc;

    if(nArc == 360){
      // motor(0, 0);     
      motor.rotate(0);     
    }
    else if (pArc < nArc && ((nArc - pArc) < (360 - 7))) {
      // motor(pwm, 0);  
      motor.rotate(pwm);  
    }
    else if(nArc < pArc && ((pArc - nArc) < (360 - 7))){
      // motor(0, pwm);  
      motor.rotate(-pwm);  
    }
    else {
      // motor(0, 0);  
      motor.rotate(0);  
    }
  }

  void setVelocity(int vel) {
    if (flag == 0 && PidFlag == 0 && sPidFlag == 0) {
      odrive.setVelocity(axis, 0);
    } else if (flag == 1 || PidFlag == 1 || sPidFlag == 1) {
      odrive.setVelocity(axis, vel);
    }
  }

  void actuate(int angle, int velocity){
    set_angle(angle);
    odrive.setVelocity(axis, velocity);
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

};

// Module Module1(1, r_pwm1, l_pwm1, enco1, odrive1, cpr1);
// Module Module2(2, r_pwm2, l_pwm2, enco2, odrive2, cpr2);
// Module Module3(3, r_pwm3, l_pwm3, enco3, odrive3, cpr3);

Module Module1(1, bts1, enco1, odrive1, cpr1, o1_x, o1_y);
Module Module2(2, bts2, enco2, odrive2, cpr2, o2_x, o2_y);
Module Module3(3, bts3, enco3, odrive3, cpr3, o3_x, o3_y);

void setup() {
  Wire1.begin();

  Serial.begin(115200);

  Serial1.begin(115200);
  Serial5.begin(115200);
  Serial3.begin(115200);
  // while (!Serial) delay(10);  // wait for serial port to open!

  /* Initialise the sensor */
  bno.begin();
  if (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);

  bno.setExtCrystalUse(true);

  // pinMode(l_pwm1, OUTPUT);
  // pinMode(r_pwm1, OUTPUT);
  // pinMode(l_pwm2, OUTPUT);
  // pinMode(r_pwm2, OUTPUT);
  // pinMode(l_pwm3, OUTPUT);
  // pinMode(r_pwm3, OUTPUT);

  // delay(500);
}

void loop() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  botAngle = euler.x();

  // Serial.println(botAngle);
  getData();
  
  angularPID(0.4, 0.0001,1);

  Module1.compute(&angle1, &velocity1);
  Module2.compute(&angle2, &velocity2);
  Module3.compute(&angle3, &velocity3);

  Module1.actuate(angle1, velocity1);
  Module2.actuate(angle2, velocity2);
  Module3.actuate(angle3, velocity3);

  Module1.co_ordinate(botAngle, angle1, &Px1, &Py1);
  Module2.co_ordinate(botAngle, angle2, &Px2, &Py2);
  Module3.co_ordinate(botAngle, angle3, &Px3, &Py3);

  res_co_ordinate();

  Serial.print((String)"Rx: " + Rx + "  Ry: " + Ry);
  // Serial.print((String)"target_angle  " + Module1.target_angle + "     BotErrorAngle : " + botErrorAngle + "    Spid" + sPidFlag);

  leftX = 0;
  leftY = 0;
  oMega = 0;
  Serial.println();
}
