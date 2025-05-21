// #include "bts.h"
#include <Wire.h>
#include <Encoder.h>
#include <ODriveSwerve.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);

#define pwm 90
#define vel_const 0.5
#define dia 7

#define l_pwm1 2
#define r_pwm1 3

#define l_pwm2 4
#define r_pwm2 5

#define l_pwm3 10
#define r_pwm3 9

int velocity;
int axis = 0;

//Encoder Pins
int enco1A = 0;
int enco1B = 1;
//Encoder Pins
int enco2A = 15;
int enco2B = 14;
//Encoder Pins
int enco3A = 31;
int enco3B = 30;

int a = 1;

// bts bts1(r_pwm1,l_pwm1);
// bts bts2(r_pwm2,l_pwm2);
// bts bts3(r_pwm3,l_pwm3);

Encoder enco1(enco1A, enco1B);
Encoder enco2(enco2A, enco2B);
Encoder enco3(enco3A, enco3B);

ODriveSwerve odrive1(Serial5);
ODriveSwerve odrive2(Serial7);
ODriveSwerve odrive3(Serial8);

long cpr1 = 465623;
long cpr2 = 466169;
long cpr3 = 465765;

int ordive_cpr = 4;
float wheel_circumfernce = PI * dia;

int Rx, Ry;

int Px1, Py1;
int Px2, Py2;
int Px3, Py3;

float o1_x = 0, o1_y = 25.08;
float o2_x = -21.73, o2_y = -12.54;
float o3_x = 21.73, o3_y = -12.54;

byte psData[5];
byte dir_buttons;
byte oth_buttons;
int8_t leftX = 0;
int8_t leftY = 0;
int8_t oMegaR;
int oMegaS;
int mg;
int theta;
int oMegaP;
int oMega;
int Ps_TOUCH;

int flag = 0;
int PidFlag = 0;
int sPidFlag = 0;
int oMegaFlag = 0;
int noPid = 0;
bool reset_flag = 0;

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
unsigned int bAngSR = 500;
unsigned long pST;

int angular_limit = 60;
float sMultiplier = 0.7;

long check_time_curr = 0;
long check_time_prev = 0;

void getData() {

  Wire2.requestFrom(8, 5);
  while (Wire2.available() == 5) {

    Wire2.readBytes(psData, 5);
    dir_buttons = psData[0];
    oth_buttons = psData[1];
    leftX = psData[2];
    leftY = psData[3];
    oMegaR = psData[4];
    if (oMegaR == -128) {
      oMegaR = 127;
    }
  }
  // calculating angle of ps4 joystick using its X and Y co-ordinate;
  theta = (atan2(leftX, leftY) * RAD_TO_DEG);
  theta = (theta + 360) % 360;
  mg = sqrt(pow(leftX, 2) + pow(leftY, 2));
  // Serial.println(theta);
  // Serial.println(oMegaR);

  Ps_TOUCH = oth_buttons & 1 << 6;

  if (Ps_TOUCH) reset_flag = 1;
  else reset_flag = 0;

  if (dir_buttons && dir_buttons != 255) {
    if (dir_buttons & 1 << 4 && dir_buttons & 1 << 5) {
      theta = 45;
      // Serial.println("up - right");
    } else if (dir_buttons & 1 << 5 && dir_buttons & 1 << 6) {
      theta = 135;
      // Serial.println("down - right");
    } else if (dir_buttons & 1 << 6 && dir_buttons & 1 << 7) {
      theta = 225;
      // Serial.println("down - left");
    } else if (dir_buttons & 1 << 7 && dir_buttons & 1 << 4) {
      theta = 315;
      // Serial.println("up - left");
    }

    else if (dir_buttons & 1 << 4) {
      theta = 0;
      // Serial.println("up");
    } else if (dir_buttons & 1 << 5) {
      theta = 90;
      // Serial.println("right");
    } else if (dir_buttons & 1 << 6) {
      theta = 180;
      // Serial.println("down");
    } else if (dir_buttons & 1 << 7) {
      theta = 270;
      // Serial.println("left");
    }

    leftX = 0;
    leftY = 0;
    oMega = 0;
    mg = 61;
    // move_flag = 0;
  }

  // calculating magnitude of ps4 joystick using its X and Y co-ordinate;

  if (mg > 30) {
    if (mg > 30) {
      mg = map(mg, 30, 127, 0, 127);
    } else {
      mg = 0;
    }
    flag = 1;

  } else {
    mg = 0;
    flag = 0;
  }

  if (abs(oMegaR) > 20) {

    oMegaFlag = 1;
    oMega = map(oMegaR, 0, 127, 0, 64);
    oMegaR = oMega;
  } else {
    oMegaFlag = 0;
    oMega = 0;
  }
}

void angularPID(double kp, double ki, double kd) {

  if (oMegaFlag == 0 && noPid == 1) {
    if (abs(odrive1.getVelocity()) <= 2 && abs(odrive2.getVelocity()) <= 2 && abs(odrive3.getVelocity()) <= 2) {
      noPid = 0;
    } else {
      botTargetAngle = botAngle;
    }
  }
  if (oMegaFlag == 1) {
    botTargetAngle = botAngle;
  }
  botErrorAngle = botTargetAngle - botAngle;  // 0 // 350 // -350

  if (botErrorAngle < -180) {
    botErrorAngle += 360;  // 10
  } else if (botErrorAngle > 180) {
    botErrorAngle -= 360;  //-10
  }


  currT = micros();
  derivative_error = (botErrorAngle - prevError) / (currT - prevT);
  integral_error += botErrorAngle;
  oMegaP = (kp * botErrorAngle + (derivative_error * kd));
  prevError = botErrorAngle;
  prevT = currT;

  if (abs(botErrorAngle) < 7.5) {
    oMegaS = oMegaP;
    sPidFlag = 1;
    PidFlag = 0;
  } else {
    PidFlag = 1;
    sPidFlag = 0;
    oMegaS = 0;
  }

  if (abs(botErrorAngle) < 1.5) {
    oMegaP = 0;
    oMegaS = 0;
    PidFlag = 0;
    sPidFlag = 0;
  }
  if (oMegaFlag == 1) {
    PidFlag = 0;
    sPidFlag = 0;
    oMegaP = 0;
    oMegaS = 0;
  }

  if (flag == 1 && oMegaFlag == 0) {
    PidFlag = 1;
    sPidFlag = 0;
  }
  oMega = sPidFlag == 1 ? 0 : (PidFlag == 1 ? -oMegaP : oMegaR);

  if (oMegaFlag == 1 && noPid == 0) {
    noPid = 1;
  }

  oMega = constrain(oMega, -angular_limit, angular_limit);
  oMegaS = constrain(oMegaP, -angular_limit, angular_limit);


  // Serial.print((String) "bAng:" + botAngle + " bAngEr:" + botErrorAngle + " pT:" + (botErrorAngle * kp) + "dT:"+(derivative_error*kd)+ " oMega:" + oMega
  // +" oR:"+oMegaR+ " omegaP:" + -oMegaP +" oS:"+oMegaS+" of:"+oMegaFlag+" nF:"+noPid+" sP:" + sPidFlag + " pf:" + PidFlag + " f:" + flag);
  //     Serial.print((String)"  " + odrive1.getVelocity() + "   " + odrive2.getVelocity() + "   " + odrive3.getVelocity()+"   ");
}

void res_co_ordinate() {
  // Rx = (Px1 + Px2 + Px3) / 3;
  // Ry = (Py1 + Py2 + Py3) / 3;

  Rx += (Px1 + Px2 + Px3) / 3;
  Ry += (Py1 + Py2 + Py3) / 3;
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
    x_component = magnitude * cos(angle);  //0 //+0.86 //-0.86
    y_component = magnitude * sin(angle);  //1 // 0.5 // -0.5
    // Serial.println((String)"Resolve:\tangle: "+angle+"\tx_component: "+x_component+"\ty_component: "+y_component);
  }
};
byte x;

class Module {
public:
  int CA = 0;
  float X, Y;
  int TA = 0;
  float mag;
  int ang;
  int shortestAngle = 0;
  vctor h, w;
  float dis;
  float prev_dis;

  int index;
  int Rpwm, Lpwm;
  Encoder& enco;
  ODriveSwerve& odrive;
  long cpr;

  float origin_x, origin_y;
  float x = 0, y = 0;
  double user_view_ang = 0;
  int rot_ang = (90 + (240 * (index - 1))) % 360;

  int ANGLE = 0;
  float VELOCITY = 0;
  float sPidConst;

  int dir;
  int target_angle;
  int angtemp;

  long int modules_ticks;
  long ang_reset;

  Module(int index, int Rpwm, int Lpwm, Encoder& enco, ODriveSwerve& odrive, long cpr, float origin_x, float origin_y)
    : index(index), enco(enco), odrive(odrive), cpr(cpr), origin_x(origin_x), origin_y(origin_y) {
    this->Rpwm = Rpwm;
    this->Lpwm = Lpwm;
  }

  void motor(int r, int l) {
    analogWrite(Rpwm, r);
    analogWrite(Lpwm, l);
  }

  float distance() {
    dis = odrive.getPosition();
    // Serial.print(dis);
    dis = (wheel_circumfernce / ordive_cpr) * dis;

    // Serial.println(dis);
    // Serial.print("    ");

    float diff_dis = dis - prev_dis;

    return diff_dis;
  }

  void co_ordinate(int bot_angle, int module_angle, int* x2, int* y2) {
    user_view_ang = (bot_angle + (dir * module_angle)) % 360;
    float diff_dis = distance();
    Serial.print((String) "prev: " + prev_dis + "   dis:" + diff_dis);

    float sin_a = sin(user_view_ang * M_PI / 180);
    float cos_a = cos(user_view_ang * M_PI / 180);

    if (abs(diff_dis) > 0.0001) {
      // *x2 = (dir*diff_dis * sin_a) + x;
      // *y2 = (diff_dis * cos_a) + y;

      *x2 = round(dir*diff_dis * sin_a);
      *y2 = round(diff_dis * cos_a);

      // x = *x2;
      // y = *y2;

      // *x2 += origin_x;
      // *y2 += origin_y;
      prev_dis = dis;
    }
    Serial.println((String) "   index: " + index + "   sin: " + sin_a + "    x: " + *x2 + "   y: " + *y2);
  }


  void calc() {
    // Serial.println(mg);
    // Serial.print(theta - botAngle);
    h.update(mg, theta - botAngle);              //
    w.update(oMega, 270 - (120 * (index - 1)));  // 270/330/390(30)
    h.resolve();
    w.resolve();
    // Serial.println();
    X = h.x_component + w.x_component;  //0 //0.86 // -0.5
    Y = h.y_component + w.y_component;  //1 //0.5 // -0.86
    // Serial.println((String)"X"+index+": "+X+"\tY"+index+": "+Y);
    mag = sqrt(pow(X, 2) + pow(Y, 2));  //1 //1 // 1
    ang = atan2(Y, X) * 180 / M_PI;     // 90 // 30 // 210
    ang = (ang + 360) % 360;            //90 // 330 /210
    if (flag == 1 || oMegaFlag == 1) {
      ANGLE = ang;
    }
    // Serial.println((String)"      mag:"+mag+"\t ang"+ang);
  }

  float statutoryPID(float velocity) {


    velocity = sMultiplier * sPidMod() * oMegaS * cos((abs(rot_ang - target_angle)) * PI / 180);


    return velocity;
  }

  void compute(float* angle, float* velocity) {
    calc();
    short_angle();
    *angle = ang;                       //30 330 210
    *velocity = mag * vel_const * dir;  //1 1 1

    if (sPidFlag == 1 && flag == 0 && oMegaFlag == 0) {
      *velocity = statutoryPID(*velocity);
    }
    VELOCITY = *velocity;

    // Serial.println((String)*velocity + " " + *angle);
  }

  int t2a() {
    long tick = enco.read();
    tick = tick % cpr;
    tick = tick < 0 ? tick + cpr : tick;
    CA = map(tick, 0, cpr, 0, 360);
    return CA % 360;
  }

  void short_angle() {
    int CA, TA = ang;
    if ((flag == 1 || PidFlag == 1 || oMegaFlag == 1) && sPidFlag == 0) {
      CA = target_angle;

      if (abs(TA - CA) <= 90 || 270 <= abs(TA - CA)) {
        target_angle = TA;
        dir = 1;
      } else {
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
  void set_angle(int target_angle) {
    int pArc, nArc;
    pArc = target_angle - t2a();
    pArc = pArc >= 0 ? pArc : 360 + pArc;

    nArc = 360 - pArc;

    if (nArc == 360) {
      // motor(0, 0);
      motor(0, 0);
    } else if (pArc < nArc && ((nArc - pArc) < (360 - 7))) {
      // motor(pwm, 0);
      motor(pwm, 0);
    } else if (nArc < pArc && ((pArc - nArc) < (360 - 7))) {
      // motor(0, pwm);
      motor(0, pwm);
    } else {
      // motor(0, 0);
      motor(0, 0);
    }
    // Serial.print((String)dir + "  ");
  }

  void setVelocity(int vel) {
    if (flag == 0 && PidFlag == 0 && sPidFlag == 0) {
      odrive.setVelocity(axis, 0);
    } else if (flag == 1 || PidFlag == 1 || sPidFlag == 1 || oMegaFlag) {
      odrive.setVelocity(axis, vel);
    }
  }

  void actuate(int angle, int velocity) {
    set_angle(angle);
    odrive.setVelocity(axis, velocity);
  }

  void checkOdrive() {
    if (
      odrive.getProcedureResult() == PROCEDURE_RESULT_DISARMED
      || odrive.getError() == ODRIVE_ERROR_CURRENT_LIMIT_VIOLATION
      || odrive.getError() == ODRIVE_ERROR_TIMING_ERROR) {
      odrive.clearErrors();
      odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
      // Serial.println("disarm");
    }
  }

  // void checkOdrive() {
  //   Serial.println("Waiting for ODrive");
  //   Serial.print(index);
  //   while (odrive.getState() == AXIS_STATE_UNDEFINED) {
  //     delay(10);
  //   }

  //   Serial.print("DC voltage: ");
  //   Serial.println(odrive.getParameterAsFloat("vbus_voltage"));

  //   Serial.println("Enabling closed loop control...");
  // while (odrive.getState() != AXIS_STATE_CLOSED_LOOP_CONTROL) {
  //   odrive.clearErrors();
  //   odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
  //   delay(10);
  // }
  // }

  void reset_module() {

    modules_ticks = enco.read();
    ang_reset = map(modules_ticks, 0, cpr, 0, 360);

    // if (PS) toggle_module = toggle_module ? 0 : 1;
    // if(PS) toggle_module ^= 1;

    if (Ps_TOUCH) {
      if (ang_reset > 0) {
        motor(0, pwm);
        target_angle = ang_reset;
      } else if (ang_reset < 0) {
        motor(pwm, 0);
        target_angle = ang_reset;
      } else {
        motor(0, 0);
        target_angle = ang_reset;
      }
    }
  }

  void printf() {
    Serial.print((String) "\t" + index + " Vel:" + VELOCITY);
    Serial.print((String) " DAng:" + ANGLE);
    Serial.print((String) " MAng:" + target_angle);
    Serial.print((String) " RotAngle:" + rot_ang);
    Serial.print((String) " Cos:" + cos((abs(rot_ang - target_angle)) * PI / 180));
  }
};

Module Module1(1, r_pwm1, l_pwm1, enco1, odrive1, cpr1, o1_x, o1_y);
Module Module2(2, r_pwm2, l_pwm2, enco2, odrive2, cpr2, o2_x, o2_y);
Module Module3(3, r_pwm3, l_pwm3, enco3, odrive3, cpr3, o3_x, o3_y);

float sPidMod() {
  float x = abs(cos(abs(Module1.target_angle - Module1.rot_ang) * DEG_TO_RAD)) + abs(cos(abs(Module2.target_angle - Module2.rot_ang) * DEG_TO_RAD)) + abs(cos(abs(Module3.target_angle - Module3.rot_ang) * DEG_TO_RAD));
  x = 3 / x;
  return x;
}

void setup() {

  Serial.begin(115200);

  Serial5.begin(115200);
  Serial7.begin(115200);
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

  // delay(1000);
  Serial.println("Started...");
  // delay(1000);
  Wire2.begin();

  bno.setExtCrystalUse(true);

  pinMode(l_pwm1, OUTPUT);
  pinMode(r_pwm1, OUTPUT);
  pinMode(l_pwm2, OUTPUT);
  pinMode(r_pwm2, OUTPUT);
  pinMode(l_pwm3, OUTPUT);
  pinMode(r_pwm3, OUTPUT);

  while (odrive1.getState() == AXIS_STATE_ENCODER_INDEX_SEARCH
         && odrive2.getState() == AXIS_STATE_ENCODER_INDEX_SEARCH
         && odrive3.getState() == AXIS_STATE_ENCODER_INDEX_SEARCH) {
    ;
  }

  odrive1.setPosition(0);
  odrive2.setPosition(0);
  odrive3.setPosition(0);

  // delay(1000);
  // delay(500);
  // Module1.checkOdrive();
  // Module2.checkOdrive();
  // Module3.checkOdrive();
}

void loop() {
  check_time_curr = millis();
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  botAngle = euler.x();

  // Serial.println(botAngle);
  getData();

  //         kp     ki    kd
  angularPID(0.6, 0.0001, 50);

  // if(check_time_curr - check_time_prev >= 1000){
  //   Module1.checkOdrive();
  //   Module2.checkOdrive();
  //   Module3.checkOdrive();
  //   check_time_prev = check_time_curr;
  // }

  if (reset_flag) {
    Module1.reset_module();
    Module2.reset_module();
    Module3.reset_module();
  }

  if (!reset_flag) {
    Module1.compute(&angle1, &velocity1);
    Module2.compute(&angle2, &velocity2);
    Module3.compute(&angle3, &velocity3);

    Module1.actuate(angle1, velocity1);
    Module2.actuate(angle2, velocity2);
    Module3.actuate(angle3, velocity3);
  }
  // Module1.printf();
  // Module2.printf();
  // Module3.printf();
  // Serial.print(Module1.t2a());
  // Serial.print(" ");
  // Serial.print(Module2.t2a());
  // Serial.print(" ");
  // Serial.println(Module3.t2a());

  Module1.co_ordinate(botAngle, angle1, &Px1, &Py1);
  Module2.co_ordinate(botAngle, angle2, &Px2, &Py2);
  Module3.co_ordinate(botAngle, angle3, &Px3, &Py3);

  res_co_ordinate();

  Serial.println((String)"Rx: " + Rx + "  Ry: " + Ry);
  // Serial.print((String)"target_angle  " + Module3.target_angle + "     BotErrorAngle : " + botErrorAngle + "    Spid" + sPidFlag);
  // Serial.println((String)leftX + "    " + leftY + "   " + oMegaR + "    " + dir_buttons + "   " + oth_buttons);
  // Serial.println((String)velocity1 + "    " + velocity2 + "   " + velocity3);
  // Serial.print(leftX);
  // Serial.print("\t");
  // Serial.print(leftY);
  // Serial.print("\t");

  leftX = 0;
  leftY = 0;
  oMega = 0;
  // Serial.print(millis());
  Serial.println();
  // delay(10);
}
