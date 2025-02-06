#include <Wire.h>
#include <Encoder.h>
#include <ODriveSwerve.h>

#define pwm 70
#define vel_const 0.8

#define r_pwm1 9
#define l_pwm1 8

#define l_pwm2 7
#define r_pwm2 6

#define l_pwm3 5
#define r_pwm3 4

#define l_pwm4 3
#define r_pwm4 2

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

int velocity;
int axis = 0;

Encoder enco1(enco1A, enco1B);
Encoder enco2(enco2A, enco2B);
Encoder enco3(enco3A, enco3B);
Encoder enco4(enco4A, enco4B);

ODriveSwerve odrive1(Serial1);
ODriveSwerve odrive2(Serial5);
ODriveSwerve odrive3(Serial3);
ODriveSwerve odrive4(Serial8);

long cpr1 = 466104;
long cpr2 = 464239;
long cpr3 = 463951;
long cpr4 = 466490;

int mg;

int TA;
int dir = 0;
int flag = 0;
int vel = 0;
int x = 0,y = 0; 
int px = 0, py = 0;

int tflag = 0;
int oflag = 15;

struct data{
  int x, y;
};

class queue{
private:
  data* arr;
  int front;
  int rear;
  int capacity;
  int count;

public:
  queue(int size){
    arr = new data[size];
    front = 0;
    rear = -1;
    capacity = size;
    count = 0;
  }

  void insert(int x = 0, int y = 0){
    if (isFull()) {
        return;
    }

    rear = (rear + 1) % capacity; // Circular increment
    arr[rear].x = x;
    arr[rear].y = y;
    count++;
  }

  void pop(){
    if (isEmpty()) {
        return;
    }

    front = (front + 1) % capacity; // Circular increment
    count--;
  }

  std::pair<int, int> top(){
    if (isEmpty()) {
        return {0,0}; // Return a default value if empty
    }
    return {arr[front].x, arr[front].y};
  }

  bool isEmpty() {
      return count == 0;
  }

  // Function to check if the queue is full
  bool isFull() {
      return count == capacity;
  }
};

queue q(10);

void cal(int X, int Y) {
  int difx = X - px;
  int dify = Y - py;

  TA = (atan2(difx, dify) * RAD_TO_DEG);
  TA = (TA + 360) % 360;

  mg = sqrt(pow(X - px, 2) + pow(Y - py, 2));
}

class Module {
private:
  int index;
  int Rpwm, Lpwm;
  int cpr;
  Encoder& enco;
  ODriveSwerve& odrive;

public:
  int CA = 0;
  int total_mg = 0;
  int curr_p = 0;
  int flag, f;

  Module(int index, int Rpwm, int Lpwm, Encoder& enco, ODriveSwerve& odrive, long cpr)
    : enco(enco), odrive(odrive) {
    this->index = index;
    this->Rpwm = Rpwm;
    this->Lpwm = Lpwm;
    this->cpr = cpr;
  }

  void motor(int r, int l) {
    analogWrite(Rpwm, r);
    analogWrite(Lpwm, l);
  }

  int t2a() {
    long tick = enco.read();
    tick = tick % cpr;
    tick = tick < 0 ? tick + cpr : tick;
    CA = map(tick, 0, cpr, 0, 360);
    return CA % 360;
  }

  // calculating velocity for bldc;
  int velocity() {
    return mg * vel_const * dir;
  }

  int short_angle(int TA) {
    int target_angle;
    CA = t2a();
    if (abs(TA - CA) <= 90 || 270 <= abs(TA - CA)) {
      target_angle = TA;
      dir = 1;
      return target_angle;
    } else {
      target_angle = TA - 180;
      target_angle = target_angle < 0 ? target_angle + 360 : target_angle;
      dir = -1;
      return target_angle;
    }

    // flags();
  }

  // setting the shortest angle by breaking the 360 barrier;
  void set_angle(int target_angle) {
    int pArc, nArc;
    pArc = target_angle - CA;
    pArc = pArc >= 0 ? pArc : 360 + pArc;

    nArc = 360 - pArc;
    if (target_angle == CA) {
      motor(0, 0);
      tflag |= 1<< (index -1);

    }
     else if (pArc < nArc && ((nArc - pArc) < (360 - 5))) {
      motor(pwm, 0);
      tflag &=~(1 << (index - 1));
    } else if (nArc < pArc && ((pArc - nArc) < (360 - 5))) {
      motor(0, pwm);
      tflag &=~(1 << (index - 1));
    } 
    
    else {
      motor(0, 0);
      tflag |= 1<< (index -1);
    }
  }

  void flags(){
    if(!q.isEmpty()){
      curr_p = odrive.getPosition();
      f = (abs(TA - CA) <= 90 || 270 <= abs(TA - CA)) ? 1 : 0;

      Serial.println((String) TA + "\t" + total_mg + "\t" + curr_p + "\t" + oflag + "\t");

      if( oflag >> (index - 1) & 1 ){
        if ( f == 1) {
          Serial.println((String)"a");
          total_mg += mg;
        }
        if ( f == 0 ) {
          Serial.println((String)"b");
          total_mg -= mg;
        }

        oflag &= ~(1 << (index - 1));
      }

    }
  }

  // making dc and bldc work togathe  r;
  void actuate() {
    if (q.isEmpty()) {
      return;
    }
    int target_angle = short_angle(TA);
    // if(oflag == 0){
      set_angle(target_angle); 
    // }
    if(oflag == 0 && tflag == 15){
      odrive.goTo(total_mg, 10);
    }
  }

  
};

Module Module1(1, r_pwm1, l_pwm1, enco1, odrive1, cpr1);
Module Module2(2, r_pwm2, l_pwm2, enco2, odrive2, cpr2);
Module Module3(3, r_pwm3, l_pwm3, enco3, odrive3, cpr3);
Module Module4(4, r_pwm4, l_pwm4, enco4, odrive4, cpr4);


void setup() {
  Wire1.begin();
  Serial.begin(115200);

  Serial1.begin(115200);
  Serial5.begin(115200);
  Serial3.begin(115200);
  Serial8.begin(115200);

  odrive3.closedLoopState();
  odrive4.closedLoopState();

  pinMode(l_pwm1, OUTPUT);
  pinMode(r_pwm1, OUTPUT);
  pinMode(l_pwm2, OUTPUT);
  pinMode(r_pwm2, OUTPUT);
  pinMode(l_pwm3, OUTPUT);
  pinMode(r_pwm3, OUTPUT);
  pinMode(l_pwm4, OUTPUT);
  pinMode(r_pwm4, OUTPUT);

  q.insert(0, 0);
  q.insert(60, 0);
  q.insert(60, 40);
  q.insert(0,40);
  q.insert(0, 0);

  // q.insert(0, 0);
  // q.insert(30, 40);
  // q.insert(60, 0);
  // q.insert(0,20);
  // q.insert(60, 20);
  // q.insert(0, 0);

  delay(500);
}

void loop() {
  std::pair<int, int> co = q.top();
  x = co.first;
  y = co.second;


  cal(x, y);
  Module1.flags();
  Module2.flags();
  Module3.flags();
  Module4.flags();

  Module1.actuate();
  Module2.actuate();
  Module3.actuate();
  Module4.actuate();      

  if (abs(Module1.curr_p - Module1.total_mg) <= 1 &&
      abs(Module2.curr_p - Module2.total_mg) <= 1 && 
      abs(Module3.curr_p - Module3.total_mg) <= 1 && 
      abs(Module4.curr_p - Module4.total_mg) <= 1){
    oflag = 15;
    Serial.println("aa");
  }

  if(oflag == 15){
    q.pop();
    // Serial.println((String)px + " " + py + " " + x + " "+ y + " " + TA + " POP");

    px = x;
    py = y;
    delay(1000);
  }   
  
  Serial.println();
  // if(q.isEmpty()){
  //   Serial.print("done");
  // }
}
