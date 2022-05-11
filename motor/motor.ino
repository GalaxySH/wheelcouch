//slave
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <math.h>
#include <Servo.h>

SoftwareSerial mySerial(5,6);  //rx pin,tx pin
int motors[2][2] {{9, 10}, {5, 6}};// left forward, left rear, right forward, right rear
int fpow = 0;// initial forward power  (vector with values -1, 0, and 1)
int lpow = 0;// initial lateral turning power (vector with values -1, 0, and 1)
int REVERSE = 0;
int NEUTRAL = 90;//90// confirm that this is actually a neutral value
int FORWARD = 180;
char recvchars[32];
boolean newdata = false;
int dataarray[3];
int speeds[2] {NEUTRAL, NEUTRAL};// left, right
int sounding = 0;// speaker boolean
Servo lfDrive;
Servo lrDrive;
Servo rfDrive;
Servo rrDrive;

void recvdata() {
  static boolean recvinprogress = false;
  static byte ndx = 0;
  char startmarker = '<';
  char comma = ',';
  char endmarker = '>';
  char c;

  while (mySerial.available() > 0 && newdata == false) {
    c = mySerial.read();

    if (c == startmarker) {
      recvinprogress = true;
    } else if(c == endmarker) {
      recvchars[ndx] = c;
      recvinprogress = false;
      ndx = 0;
      newdata = true;
    }
    if (recvinprogress == true) {
      recvchars[ndx] = c;
      ndx++;  
    }
  }

  Serial.print(recvchars);
  Serial.print("\n");
  delay(100);
  
  readintonum(recvchars,dataarray);
  int i;
  for(i = 0; i < 3; i++) {
    Serial.println(dataarray[i]);
  }
  delay(100);
}

void readintonum(char* recvchars,int* dataarray){
  int i, count = 0 , num = 0, arraycount = 0;
  boolean started=false;
  for (i = 0; i < 32; i++) {
    if (recvchars[i]=='<') started = true;
      
    if ((started == true) && (recvchars[i] == ',')) {
      count = 0;
      dataarray[arraycount] = num;
      arraycount++;
      num = 0;
    }
    if ((started == true) && (recvchars[i] >= 48) && (recvchars[i] <= 57)) {
      num = num + pow(10,count) * (recvchars[i] - 48);
      count++;
    }
    if ((started == true) && (recvchars[i] == '>')) break;
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

  //SERVO
  lfDrive.attach(motors[0][0]);
  lrDrive.attach(motors[0][1]);
  rfDrive.attach(motors[1][0]);
  rrDrive.attach(motors[1][1]);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(newdata==false){
    recvdata();
    fpow = dataarray[0];
    lpow = dataarray[1];
    sounding = dataarray[2];
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

  lfDrive.write(speeds[0]);
  lrDrive.write(speeds[0]);
  rfDrive.write(speeds[1]);
  rrDrive.write(speeds[1]);
}
