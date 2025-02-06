#include <Wire.h>
#include <Encoder.h>
#include <ODriveSwerve.h>

#define PWM_VALUE 70

#define RPWM_PINS {9, 7, 5, 3}
#define LPWM_PINS {8, 6, 4, 2}

const int ENCODER_PINS[4][2] = {
  {26, 27},
  {33, 32},
  {38, 37},
  {31, 30}
};

const long CPR_VALUES[4] = {466104, 464239, 463951, 466490};

ODriveSwerve odriveInstances[] = {
  ODriveSwerve(Serial1),
  ODriveSwerve(Serial5),
  ODriveSwerve(Serial3),
  ODriveSwerve(Serial8)
};

Encoder encoders[] = {
  Encoder(ENCODER_PINS[0][0], ENCODER_PINS[0][1]),
  Encoder(ENCODER_PINS[1][0], ENCODER_PINS[1][1]),
  Encoder(ENCODER_PINS[2][0], ENCODER_PINS[2][1]),
  Encoder(ENCODER_PINS[3][0], ENCODER_PINS[3][1])
};

#define VELOCITY_CONSTANT 0.8
int globalAngle;
float magnitude;
int direction;

class Module {
private:
  int index;
  int Rpwm, Lpwm;
  Encoder& encoder;
  ODriveSwerve& odrive;
  long cpr;
  int currentAngle;

public:
  Module(int index, int Rpwm, int Lpwm, Encoder& encoder, ODriveSwerve& odrive, long cpr) :
    index(index), Rpwm(Rpwm), Lpwm(Lpwm), encoder(encoder), odrive(odrive), cpr(cpr) {}

  void motor(int forward, int reverse) {
    analogWrite(Rpwm, forward);
    analogWrite(Lpwm, reverse);
  }

  int getCurrentAngle() {
    long tick = encoder.read() % cpr;
    tick = tick < 0 ? tick + cpr : tick;
    currentAngle = map(tick, 0, cpr, 0, 360);
    return currentAngle;
  }

  int computeShortestAngle(int targetAngle) {
    int angleDifference = targetAngle - getCurrentAngle();
    angleDifference = (angleDifference + 360) % 360;

    if (angleDifference > 180) {
      direction = -1;
      return (targetAngle - 180 + 360) % 360;
    }
    direction = 1;
    return targetAngle;
  }

  void setAngle(int targetAngle) {
    int angleDifference = abs(targetAngle - currentAngle);
    if (angleDifference <= 5 || angleDifference >= 355) {
      motor(0, 0);
    } else if (angleDifference < 180) {
      motor(PWM_VALUE, 0);
    } else {
      motor(0, PWM_VALUE);
    }
  }

  void moveToPosition(float position) {
    odrive.goTo(position, 30);
  }
};

Module modules[] = {
  Module(1, RPWM_PINS[0], LPWM_PINS[0], encoders[0], odriveInstances[0], CPR_VALUES[0]),
  Module(2, RPWM_PINS[1], LPWM_PINS[1], encoders[1], odriveInstances[1], CPR_VALUES[1]),
  Module(3, RPWM_PINS[2], LPWM_PINS[2], encoders[2], odriveInstances[2], CPR_VALUES[2]),
  Module(4, RPWM_PINS[3], LPWM_PINS[3], encoders[3], odriveInstances[3], CPR_VALUES[3])
};

void setup() {
  Wire.begin();
  Serial.begin(115200);

  for (int i = 0; i < 4; i++) {
    pinMode(RPWM_PINS[i], OUTPUT);
    pinMode(LPWM_PINS[i], OUTPUT);
  }

  for (auto& odrive : odriveInstances) {
    odrive.closedLoopState();
  }
  delay(2000);
}

void gotoPoint(int targetAngle, float position) {
  int buffer = 3;

  do {
    globalAngle = targetAngle;
    for (auto& module : modules) {
      module.setAngle(module.computeShortestAngle(globalAngle));
    }
  } while (!isAngleAligned(buffer));

  // Stop all motors
  for (auto& module : modules) {
    module.motor(0, 0);
  }

  do {
    for (auto& module : modules) {
      module.moveToPosition(position);
    }
  } while (!isPositionAligned(position, 0.2));
}

bool isAngleAligned(int buffer) {
  for (auto& module : modules) {
    if (abs(module.getCurrentAngle() - globalAngle) > buffer) {
      return false;
    }
  }
  return true;
}

bool isPositionAligned(float position, float tolerance) {
  for (auto& odrive : odriveInstances) {
    if (abs(position - odrive.getPosition()) > tolerance) {
      return false;
    }
  }
  return true;
}

void loop() {
  gotoPoint(30, 10);
  delay(1000);
  gotoPoint(60, 20);
  delay(1000);
}
