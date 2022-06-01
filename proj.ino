#include <SoftwareSerial.h>
SoftwareSerial mySerial(12, 13); // RX, TX
  
char BT_input = 'o'; //Bluetooth input
int left_motorF = 10; //left motors forward
int left_motorR = 9; //left motors reverse
int right_motorR = 8; //right motors reverse
int right_motorF = 7; //right motors forward
int motorA = 6; //enable motor A For Left Wheels
int motorB = 11; //enable motor B For Right Wheels

int R_IR = 3; //IR sensor Right
int M_IR = A1; //IR sensor Middle
int L_IR = 2; //IR sensor Left

void readBT(){    //Read from bluetooth module
  if(mySerial.available()){ 
    BT_input = mySerial.read();
  //  Serial.println(BT_input);
  }
}

void forward(int speed1 = 150, int speed2 = 150){    //move forward(all motors rotate in forward direction)
  digitalWrite(left_motorF, HIGH);
  digitalWrite(right_motorF, HIGH);
  digitalWrite(left_motorR, LOW);
  digitalWrite(right_motorR, LOW);
  analogWrite(motorA, speed1);// control speed of motor A
  analogWrite(motorB, speed2);// control speed of motor B
}

void backward(){    //move reverse (all motors rotate in reverse direction)
  digitalWrite(left_motorR, HIGH);
  digitalWrite(right_motorR, HIGH);
  digitalWrite(left_motorF, LOW);
  digitalWrite(right_motorF, LOW);
}

void turnLeft(int Speed1 = 150, int Speed2 = 150){    //turn left (right side motors rotate in forward direction, left side motors doesn't rotate)
    
  digitalWrite(right_motorF, HIGH);
  digitalWrite(left_motorF, LOW);
  digitalWrite(right_motorR, LOW);
  digitalWrite(left_motorR, HIGH);
  analogWrite(motorB,Speed1);
  analogWrite(motorA,Speed2);
}

void turnRight(int Speed1 = 150, int Speed2 = 150){   //turn right (left side motors rotate in forward direction, right side motors doesn't rotate)
  digitalWrite(left_motorF, HIGH);
  digitalWrite(right_motorF, LOW);
  digitalWrite(right_motorR, HIGH);
  digitalWrite(left_motorR, LOW);

  analogWrite(motorA,Speed1);
  analogWrite(motorB,Speed2);
}

void stopMotors(){  //STOP (all motors stop)
  digitalWrite(left_motorF, LOW);
  digitalWrite(left_motorR, LOW);
  digitalWrite(right_motorF, LOW);
  digitalWrite(right_motorR, LOW);
}
void setup() {
  pinMode(left_motorF, OUTPUT);   //left motors forward
  pinMode(left_motorR, OUTPUT);   //left motors reverse
  pinMode(right_motorF, OUTPUT);   //right motors forward
  pinMode(right_motorR, OUTPUT);   //right motors reverse
  mySerial.begin(9600);
  Serial.begin(9600);
//  Serial.println(analogRead(R_IR));
}
 void leftinterrupt()
 {
  stopMotors();
  delay(30);
  while (digitalRead(2)==1)
     turnLeft(170,170);
 }
 void rightinterrupt()
 {
  stopMotors();
  delay(30);
  while (digitalRead(3)==1)
     turnRight(170,170);
 }
void loop() {
  readBT();                               //Read from bluetooth module
//   mySerial.println("jiu");
  if(BT_input == 'F') forward();          //move forward  
          
  else if(BT_input == 'B') backward();    //move reverse
   
  else if(BT_input == 'L') turnLeft();    //turn left
  
  else if(BT_input == 'R') turnRight();   //turn right   
  
  else if(BT_input == 'I'){
    forward();
    turnRight();
  }
  else if(BT_input == 'G'){
    forward();
    turnLeft();
  }
  
  else if(BT_input == 'J'){
    backward();
    turnLeft();
  }

  else if(BT_input == 'H'){
    backward();
    turnRight();
  }
  
  else if(BT_input == 'S') stopMotors();  //STOP   

  else if (BT_input == 'X') {            // IR line follower
    
    while(BT_input != 'x'){
      if(digitalRead(L_IR)==0||digitalRead(R_IR)==0){
       attachInterrupt(0,leftinterrupt,RISING);
       attachInterrupt(1,rightinterrupt,RISING);
              forward(15);

      }else if (digitalRead(L_IR)==1&&digitalRead(R_IR)==1){
        forward(15);
        }
       else forward(15);
//      mySerial.println(L_IR);
//      if ((digitalRead(L_IR) == 0)&&(digitalRead(M_IR) == 1)&&(digitalRead(R_IR) == 0)) forward();
//  
//      if ((digitalRead(L_IR) == 1)&&(digitalRead(M_IR) == 1)&&(digitalRead(R_IR) == 0)) turnLeft();
//      if ((digitalRead(L_IR) == 1)&&(digitalRead(M_IR) ==0)&&(digitalRead(R_IR) == 0))  turnLeft();
//  
//      if ((digitalRead(L_IR) == 0)&&(digitalRead(M_IR) == 1)&&(digitalRead(R_IR) == 1)) turnRight();
//      if ((digitalRead(L_IR) == 0)&&(digitalRead(M_IR) == 0)&&(digitalRead(R_IR) == 1)) turnRight();
//      if ((digitalRead(L_IR) == 1)&&(digitalRead(M_IR) == 1)&&(digitalRead(R_IR) == 1)) stopMotors();
//      if ((digitalRead(L_IR) == 0)&&(digitalRead(M_IR) == 0)&&(digitalRead(R_IR) == 0)) stopMotors();
    //  if ((digitalRead(L_IR) == 0)&&(digitalRead(R_IR) == 0)) forward();
  //    if ((digitalRead(L_IR) == 1)&&(digitalRead(R_IR) == 0)) turnRight();
//      if ((digitalRead(L_IR) == 0)&&(digitalRead(R_IR) == 1)) turnLeft();

//      if ((digitalRead(L_IR) == 1)&&(digitalRead(R_IR) == 1)) forward();
      readBT();
    }
    detachInterrupt(0);
    detachInterrupt(1);
    stopMotors();

  }
  delay(10);
}

//int isBlack(int IR){
//  if(analogRead(IR)>500){
//    return 1;
//  }else if(analogRead(IR)<500){
//    return 0;
//  }
//}
