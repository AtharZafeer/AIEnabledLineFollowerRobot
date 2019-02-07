#include <L298N.h>
#define EN1 9  //right motor enable pwm
#define EN2 11 // left motor enable pwm
#define in1 3 // right motor
#define in2 4 //
#define in3 5 // left motor
#define in4 6 //
const int ir[]={A0,A1,A2,A3,A4}; //infrared array sensor
const int irL= 1,irR = 2 ; //left and right sensor
int data[4], MODE;
L298N motorR(EN1, in1, in2); //initializes a right motor
L298N motorL(EN2, in3, in4);
void setup() {
  // put your setup code here, to run once:
    pinMode(EN1,OUTPUT);pinMode(13,OUTPUT);
    pinMode(EN2, OUTPUT);
    pinMode(in4, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2,OUTPUT);pinMode(in3,OUTPUT);
    pinMode(irR, INPUT);pinMode(irL, INPUT);
    for(int i=0;i<5;i++)
    {
       pinMode(ir[i],INPUT); 
    }
    motorR.setSpeed(80); //initial speed of the motors
    motorL.setSpeed(80);
    Serial.begin(9600);
    delay(2000);
}   
   
int error ,RSvalue, LSvalue,lasterror=0;// error, right and left sensor value
void loop(){     
  ReadError();
  switch(MODE){
    case 1:{ 
      ReadError();
      calcPID();
      break;}
    case 2:{
      runExtra();
      ReadError();
      turn(2,700);
      ReadError();
      break;}
    case 3:{ 
      runExtra(200);
      ReadError();
      turn(1,700);
      ReadError();
      break;}
    case 4:{
      runExtra(200);
      ReadError();
      if(MODE != 1){
        turn(1,700);
       }else {
        ReadError();
        calcPID();
       }break;}
    case 5:{
      runExtra(600);
      //delay(1000);
      ReadError();
      if(MODE == 1){
          calcPID();    
        }else {
          turn(3, 1000);   
        }break;
      }        
    }
}

void ReadError(){
    RSvalue = digitalRead(irR);
    LSvalue = digitalRead(irL);
    for(int i=0;i<5;i++){
      data[i]=analogRead(ir[i]);
      data[i]= correct(data[i]);
      Serial.print(data[i]);Serial.print(' ');
    }Serial.println();// get the data from the sensor array
    // find the error based on the sensor array
    //mode 1,2,3,4,5 is for line follow, right, left,continuous line  and stop respectively
     if((data[0]==1) && (data[1]==1) && (data[2]==1) && (data[3]==1) && (data[4]==1)){MODE = 4; error = 0;}
     else if(data[4]==0 && (LSvalue == 1)){MODE = 2;error = 0;}//right}
     else if(LSvalue == 1 && data[0] == 0){MODE = 3;error = 0; }//left }
     else if((data[0]== 0) && (data[1]== 0) && (data[2]== 0) && (data[3]== 0) && (data[4]== 0)){error = 0; MODE = 5;}
     else if((data[0]== 0) && (data[1]== 0) && (data[2]== 1) && (data[3]== 0) && (data[4]== 0)){error = 0; MODE = 1; }
     else if((data[0]== 0) && (data[1]== 0) && (data[2]== 0) && (data[3]== 0) && (data[4]== 1)){error = 4; MODE = 1;}
     else if((data[0]== 0) && (data[1]== 0) && (data[2]== 0) && (data[3]== 1) && (data[4]== 1)){error = 3; MODE = 1;}
     else if((data[0]== 0) && (data[1]== 0) && (data[2]== 0) && (data[3]== 1) && (data[4]== 0)){error = 2; MODE = 1;} 
     else if((data[0]== 0) && (data[1]== 0) && (data[2]== 1) && (data[3]== 1) && (data[4]== 0)){error = 1; MODE = 1;}
     else if((data[0]== 0) && (data[1]== 1) && (data[2]== 1) && (data[3]== 0) && (data[4]== 0)){error = -1; MODE = 1;}
     else if((data[0]== 0) && (data[1]== 1) && (data[2]== 0) && (data[3]== 0) && (data[4]== 0)){error = -2; MODE = 1;}
     else if((data[0]== 1) && (data[1]== 1) && (data[2]== 0) && (data[3]== 0) && (data[4]== 0)){error = -3; MODE = 1;}
     else if((data[0]== 1) && (data[1]== 0) && (data[2]== 0) && (data[3]== 0) && (data[4]== 0)){error = -4; MODE = 1;}  
}  
int correct(int value){
  if(value < 100){
    return 0;}
  else {
    return 1;
    }
 }
void runExtra(int d){
  calcPID();
  delay(d);
  //motorR.stop();
  //motorL.stop();
}
void turn(int way, int degree){
  //calcPID();
  delay(500);
  if(way == 1){ // way = 1 is left
    motorR.forwardFor(degree);
    motorL.backwardFor(degree);
  }else if(way == 2){ //way = 2 for right
    motorL.forwardFor(degree);
    motorR.backwardFor(degree);
  }else if(way == 3){
    motorL.forwardFor(degree);
    motorR.backwardFor(degree);
  }
}
int p=0,i=0,d=0;
void calcPID() {
   int i=0,ki=1,kp=7,kd=10;
   int rightmotorspeed, leftmotorspeed;  
   //int irf = analogRead(irfread);
   //irf = irfcorrect(irf);
   int motorspeed;
   p=error;
   i=i+error;
   d=error-lasterror;
   motorspeed= (kp*p) + (kd*d) + (ki*i) ; //calculate the pid value
   lasterror = error; 
   Serial.print("motorspeed= ");Serial.print(motorspeed);Serial.println();
   rightmotorspeed = 80+motorspeed; //set motor speeds. 80 is the base speed
   leftmotorspeed = 80-motorspeed;
   if(rightmotorspeed > 255)rightmotorspeed = 255;motorspeed =0; // prevent the value from exceeding 225
   if(leftmotorspeed > 255)leftmotorspeed = 255;motorspeed =0; 
   if(rightmotorspeed <0 )rightmotorspeed = 0;motorspeed =0; // prevent the value from going negative
   if(leftmotorspeed <0 ) leftmotorspeed = 0;motorspeed =0;
   motorR.setSpeed(rightmotorspeed);// set the speed of the motors based on that
   motorL.setSpeed(leftmotorspeed);
   motorR.forward();
   motorL.forward();  
   Serial.print(leftmotorspeed);Serial.print(' ');Serial.print(rightmotorspeed);
   Serial.println();
}
