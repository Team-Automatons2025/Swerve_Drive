/*This code is for receiving ps4 controller data on the esp32. The data to be received can be programmed from the "chooseButton" macros.
  It also has the "buttonMode" selection option to select the behoviour of button (press or click). For click button released of button pressed can be selected.
  Also the data to be printed can be selected from the respective macros.*/
//make datasheet of the code
//pin21: SDA, pin22: SCL

#include <PS4Controller.h>
#include <Wire.h>

#define size 9
byte ps4Data[size] =   {0};
byte sendData[size] =  {0};       //use pointers
#define chooseButton_0 B00000000  //choose the button which needs to be used;  0: not to be used; 1: to be used
#define chooseButton_1 B00000000  //choose the button which needs to be used;  0: not to be used; 1: to be used
#define chooseButton_2 B01001100  //choose the button which needs to be used;  0: not to be used; 1: to be used
#define buttonMode_0   B11111111  // 0: on press; 1: on click
#define buttonMode_1   B11111111  // 0: on press; 1: on click
#define buttonDown_0   B11111111  // 0: button released; 1: button pressed
#define buttonDown_1   B11111111  // 0: button released; 1: button pressed
#define printData_0    B00000000        // choose which button data to be printed
#define printData_1    B00000000        // choose which button data to be printed
#define printData_2    B01001100         // choose which button data to be printed
int bytesToSend =      0;
int l2, r2, diff;
int val;

void psData(){

  //Buttons data
  if(buttonMode_0 & 1 && chooseButton_0 & 1){                //up button
    ps4Data[0] |= buttonDown_0 & 1 ? PS4.event.button_down.up : PS4.event.button_up.up;
    if(printData_0 & 1 && ps4Data[0] & 1) Serial.println("Up click");
  }
  else if(chooseButton_0 & 1){
    ps4Data[0] |=  PS4.Up();
    if(printData_0 & 1 && ps4Data[0] & 1) Serial.println("Up press");
  }
  
  if(buttonMode_0 & 1 << 1 && chooseButton_0 & 1 << 1){      //right button
    ps4Data[0] |= (buttonDown_0 & 1 << 1 ? PS4.event.button_down.right : PS4.event.button_up.right) << 1;
    if(printData_0 & 1 << 1 && ps4Data[0] & 1 << 1) Serial.println("Right click");
  }
  else if(chooseButton_0 & 1 << 1){
    ps4Data[0] |=  PS4.Right() << 1;
    if(printData_0 & 1 << 1 && ps4Data[0] & 1 << 1) Serial.println("Right press");
  }
  if(buttonMode_0 & 1 << 2 && chooseButton_0 & 1 << 2){      //down button
    ps4Data[0] |= (buttonDown_0 & 1 << 2 ? PS4.event.button_down.down : PS4.event.button_up.down) << 2;
    if(printData_0 & 1 << 2 && ps4Data[0] & 1 << 2) Serial.println("Down click");
  }else if(chooseButton_0 & 1 << 2){
    ps4Data[0] |=  PS4.Down() << 2;
    if(printData_0 & 1 << 2 && ps4Data[0] & 1 << 2) Serial.println("Down press");
  }
  if(buttonMode_0 & 1 << 3 && chooseButton_0 & 1 << 3){      //left button
    ps4Data[0] |= (buttonDown_0 & 1 << 3 ? PS4.event.button_down.left : PS4.event.button_up.left) << 3;
    if(printData_0 & 1 << 3 && ps4Data[0] & 1 << 3) Serial.println("Left click");
  }
  else if(chooseButton_0 & 1 << 3){
    ps4Data[0] |=  PS4.Left() << 3;
    if(printData_0 & 1 << 3 && ps4Data[0] & 1 << 3) Serial.println("Left press");
  }
  if(buttonMode_0 & 1 << 4 && chooseButton_0 & 1 << 4){      //triangle button
    ps4Data[0] |= (buttonDown_0 & 1 << 4 ? PS4.event.button_down.triangle : PS4.event.button_up.triangle) << 4;
    if(printData_0 & 1 << 4 && ps4Data[0] & 1 << 4) Serial.println("Triangle click");
  }
  else if(chooseButton_0 & 1 << 4){
    ps4Data[0] |=  PS4.Triangle() << 4;
    if(printData_0 & 1 << 4 && ps4Data[0] & 1 << 4) Serial.println("Triangle press");
  }
  if(buttonMode_0 & 1 << 5 && chooseButton_0 & 1 << 5){      //circle button
    ps4Data[0] |= (buttonDown_0 & 1 << 5 ? PS4.event.button_down.circle : PS4.event.button_up.circle) << 5;
    if(printData_0 & 1 << 5 && ps4Data[0] & 1 << 5) Serial.println("Circle click");
  }
  else if(chooseButton_0 & 1 << 5){
    ps4Data[0] |=  PS4.Circle() << 5;
    if(printData_0 & 1 << 5 && ps4Data[0] & 1 << 5) Serial.println("Circle press");
  }
  if(buttonMode_0 & 1 << 6 && chooseButton_0 & 1 << 6){      //cross button
    ps4Data[0] |= (buttonDown_0 & 1 << 6 ? PS4.event.button_down.cross : PS4.event.button_up.cross) << 6;
    if(printData_0 & 1 << 6 && ps4Data[0] & 1 << 6) Serial.println("Cross click");
  }
  else if(chooseButton_0 & 1 << 6){
    ps4Data[0] |=  PS4.Cross() << 6;
    if(printData_0 & 1 << 6 && ps4Data[0] & 1 << 6) Serial.println("Cross press");
  }
  if(buttonMode_0 & 1 << 7 && chooseButton_0 & 1 << 7){        //square button
    ps4Data[0] |= (buttonDown_0 & 1 << 7 ? PS4.event.button_down.square : PS4.event.button_up.square) << 7;
    if(printData_0 & 1 << 7 && ps4Data[0] & 1 << 7) Serial.println("Square click");
  }
  else if(chooseButton_0 & 1 << 7){
    ps4Data[0] |=  PS4.Square() << 7;
    if(printData_0 & 1 << 7 && ps4Data[0] & 1 << 7) Serial.println("Square press");
  }
  if(buttonMode_1 & 1 && chooseButton_1 & 1){                //L1 button
    ps4Data[1] |= (buttonDown_1 & 1? PS4.event.button_down.l1 : PS4.event.button_up.l1);
    if(printData_1 & 1 && ps4Data[1] & 1) Serial.println("L1 click");
  }
  else if(chooseButton_1 & 1){
    ps4Data[1] |=  PS4.L1();
    if(printData_1 & 1 && ps4Data[1] & 1) Serial.println("L1 press");
  }
  if(buttonMode_1 & 1 << 1 && chooseButton_1 & 1 << 1){      //R1 button
    ps4Data[1] |= (buttonDown_1 & 1 << 1 ? PS4.event.button_down.r1 : PS4.event.button_up.r1) << 1;
    if(printData_1 & 1 << 1 && ps4Data[1] & 1 << 1) Serial.println("R1 click");
  }
  else if(chooseButton_1 & 1 << 1){
    ps4Data[1] |=  PS4.R1() << 1;
    if(printData_1 & 1 << 1 && ps4Data[1] & 1 << 1) Serial.println("R1 press");
  }
  if(buttonMode_1 & 1 << 2 && chooseButton_1 & 1 << 2){      //L3 button
    ps4Data[1] |= (buttonDown_1 & 1 << 2 ? PS4.event.button_down.l3 : PS4.event.button_up.l3) << 2;
    if(printData_1 & 1 << 2 && ps4Data[1] & 1 << 2) Serial.println("L3 click");
  }
  else if(chooseButton_1 & 1 << 2){
    ps4Data[1] |=  PS4.L3() << 2;
    if(printData_1 & 1 << 2 && ps4Data[1] & 1 << 2) Serial.println("L3 press");
  }
  if(buttonMode_1 & 1 << 3 && chooseButton_1 & 1 << 3){      //R3 button
    ps4Data[1] |= (buttonDown_1 & 1 << 3 ? PS4.event.button_down.r3 : PS4.event.button_up.r3) << 3;
    if(printData_1 & 1 << 3 && ps4Data[1] & 1 << 3) Serial.println("R3 click");
  }
  else if(chooseButton_1 & 1 << 3){
    ps4Data[1] |=  PS4.R3() << 3;
    if(printData_1 & 1 << 3 && ps4Data[1] & 1 << 3) Serial.println("R3 press");
  }
  if(buttonMode_1 & 1 << 4 && chooseButton_1 & 1 << 4){      //share button
    ps4Data[1] |= (buttonDown_1 & 1 << 4 ? PS4.event.button_down.share : PS4.event.button_up.share) << 4;
    if(printData_1 & 1 << 4 && ps4Data[1] & 1 << 4) Serial.println("Share click");
  }
  else if(chooseButton_1 & 1 << 4){
    ps4Data[1] |=  PS4.Share() << 4;
    if(printData_1 & 1 << 4 && ps4Data[1] & 1 << 4) Serial.println("Share press");
  }
  if(buttonMode_1 & 1 << 5 && chooseButton_1 & 1 << 5){      //options button
    ps4Data[1] |= (buttonDown_1 & 1 << 5 ? PS4.event.button_down.options : PS4.event.button_up.options) << 5;
    if(printData_1 & 1 << 5 && ps4Data[1] & 1 << 5) Serial.println("Option click");
  }
  else if(chooseButton_1 & 1 << 5){
    ps4Data[1] |=  PS4.Options() << 5;
    if(printData_1 & 1 << 5 && ps4Data[1] & 1 << 5) Serial.println("Option press");
  }
  if(buttonMode_1 & 1 << 6 && chooseButton_1 & 1 << 6){      //touchpad button
    ps4Data[1] |= (buttonDown_1 & 1 << 6 ? PS4.event.button_down.touchpad : PS4.event.button_up.touchpad) << 6;
    if(printData_1 & 1 << 6 && ps4Data[1] & 1 << 6) Serial.println("Touchpad click");
  }
  else if(chooseButton_1 & 1 << 6){
    ps4Data[1] |=  PS4.Touchpad() << 6;
    if(printData_1 & 1 << 6 && ps4Data[1] & 1 << 6) Serial.println("Touchpad press");
  }
  if(buttonMode_1 & 1 << 7 && chooseButton_1 & 1 << 7){      //ps button
    ps4Data[1] |= (buttonDown_1 & 1 << 7 ? PS4.event.button_down.ps : PS4.event.button_up.ps) << 7;
    if(printData_1 & 1 << 7 && ps4Data[1] & 1 << 7) Serial.println("PS click");
  }
  else if(chooseButton_1 & 1 << 7){
    ps4Data[1] |=  PS4.PSButton() << 7;
    if(printData_1 & 1 << 7 && ps4Data[1] & 1 << 7) Serial.println("PS press");
  }
  // if(buttonMode_1 & 1 << 7 && chooseButton_1 & 1 << 7){      //battery option
  //   ps4Data[1] |= (buttonDown_1 & 1 << 7 ? PS4.event.button_down.ps : PS4.event.button_up.ps) << 7;
  //   if(printData_1 & 1 << 7 && ps4Data[1] & 1 << 7) Serial.println("PS click");
  // }
  // else if(chooseButton_1 & 1 << 7){
  //   ps4Data[1] |=  PS4.PSButton() << 7;
  //   if(printData_1 & 1 << 7 && ps4Data[1] & 1 << 7) Serial.println("PS press");
  // }

  if(chooseButton_2 & 1){
    ps4Data[2] = PS4.L2Value();
    if(printData_2 & 1) Serial.print(ps4Data[2] + String("  "));
  }
  if(chooseButton_2 & 1 << 1){
    ps4Data[3] = PS4.R2Value();
    if(printData_2 & 1 << 1) Serial.print(ps4Data[3] + String("  "));
  }

  if(chooseButton_2 & 1 << 2){
    val = PS4.LStickX();
    if(abs(val) > 10) ps4Data[4] = val;
    else ps4Data[4] = 0;

    if(printData_2 & 1 << 2) Serial.print(ps4Data[4] + String("  "));
  }
  if(chooseButton_2 & 1 << 3){
    val = PS4.LStickY();
    if(abs(val) > 10) ps4Data[5] = val;
    else ps4Data[5] = 0;

    if(printData_2 & 1 << 3) Serial.print(ps4Data[5] + String("  "));
  }

  if(chooseButton_2 & 1 << 4){
    ps4Data[6] = PS4.RStickX();
    if(printData_2 & 1 << 4) Serial.print(ps4Data[6] + String("  "));
  }
  if(chooseButton_2 & 1 << 5){
    ps4Data[7] = PS4.RStickY();
    if(printData_2 & 1 << 5) Serial.println(ps4Data[7] + String("  "));
  }
  if(chooseButton_2 & 1 << 6){
    r2 = PS4.R2Value();
    l2 = PS4.L2Value();
    diff = r2 - l2;
    diff = map(diff, -255, 255, 128, -127);
    if(abs(diff) < 10)
      diff = 0;

    ps4Data[8] = diff; 
    if(printData_2 & 1 << 6) Serial.println(ps4Data[8] + String("  "));
  }
  Serial.println();
}

int dataToSend(){
  int i = 0;
  if(chooseButton_0){
    sendData[i] = ps4Data[0];
    i++;
  }
  if(chooseButton_1){
    sendData[i] = ps4Data[1];
    i++;
  }
  sendData[1] = ps4Data[1];
  if(chooseButton_2 & 1){
    sendData[i] = ps4Data[2];
    i++;
  }
  if(chooseButton_2 & 1 << 1){
    sendData[i] = ps4Data[3];
    i++;
  }
  if(chooseButton_2 & 1 << 2){
    sendData[i] = ps4Data[4];
    i++;
  }
  if(chooseButton_2 & 1 << 3){
    sendData[i] = ps4Data[5];
    i++;
  }
  if(chooseButton_2 & 1 << 4){
    sendData[i] = ps4Data[6];
    i++;
  }
  if(chooseButton_2 & 1 << 5){
    sendData[i] = ps4Data[7];
    i++;
  }
  if(chooseButton_2 & 1 << 6){
    sendData[i] = ps4Data[8];
    i++;
  }
  return i;
}

void requestEvent(){
  Wire.write(sendData, bytesToSend);
  for(int i = 0; i < size; i++){
    ps4Data[i] = 0;
  }
}

void setup() {
  Serial.begin(115200);
  // PS4.attachOnConnect(onConnect);
  // PS4.attachOnDisconnect(onDisConnect);
  PS4.begin();
  PS4.attach(psData);
  Serial.println("Start");
  Wire.begin(8);
  Wire.onRequest(requestEvent);
}

void loop() {
  bytesToSend = dataToSend();
}
