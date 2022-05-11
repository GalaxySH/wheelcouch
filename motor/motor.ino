//slave
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <math.h>

SoftwareSerial mySerial(5,6);  //rx pin,tx pin
char recvchars[32];
boolean newdata = false;
int dataarray[3];
int speeds[2] {NEUTRAL, NEUTRAL};// left, right

void recvdata(){
  static boolean recvinprogress=false;
  static byte ndx = 0;
  char startmarker = '<';
  char comma = ',';
  char endmarker = '>';
  char c;

  while(mySerial.available() > 0 && newdata == false) {
    c=mySerial.read();

    if(c == startmarker) recvinprogress = true;

    else if(c == endmarker) {
      recvchars[ndx] = c;
      recvinprogress = false;
      ndx = 0;
      newdata = true;
    }
    if(recvinprogress == true) {
      recvchars[ndx] = c;
      ndx++;  
    }
    
    
  }

  Serial.print(recvchars);
  Serial.print("\n");
  delay(100);
  
  readintonum(recvchars,dataarray);
  int i;
  for(i=0;i<3;i++){
    Serial.println(dataarray[i]);
  }
  delay(100);
}

void readintonum(char* recvchars,int* dataarray){
  int i,count=0,num=0,arraycount=0;
  boolean started=false;
  for(i=0;i<32;i++){
     if(recvchars[i]=='<')started=true;
       
     if((started==true)&&(recvchars[i]==',')){
      count=0;
      dataarray[arraycount]=num;
      arraycount++;
      num=0;
     }
     if((started==true)&&(recvchars[i]>=48)&&(recvchars[i]<=57)){
         num=num+pow(10,count)*(recvchars[i]-48);
         count++;
     }
     if((started==true)&&(recvchars[i]=='>'))break;
  }
  return;
}

void left() {
  if (fpow == -1) {
    speeds[0] = NEUTRAL;
  } else {
    speeds[1] = FORWARD;
  }
}

void right() {
  if (fpow == -1) {
    speeds[1] = NEUTRAL;
  } else {
    speeds[0] = FORWARD;
  }
}

void setup() {
  // put your setup code here, to run once:
  mySerial.begin(28800);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(newdata==false){
    recvdata();
  }

  switch (fpow) {
    case -1: speeds[0] = FORWARD; speeds[1] = FORWARD; break;
    case 0: speeds[0] = NEUTRAL; speeds[1] = NEUTRAL; break;
    case 1: speeds[0] = REVERSE; speeds[1] = REVERSE; break;
  }
  switch (lpow) {
    case -1: left(); break;
    case 1: right(); break;
    default: break;
  }
}
