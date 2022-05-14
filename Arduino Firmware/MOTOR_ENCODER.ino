#define motor1_fr 9 //22,23
#define motor1_1 10
#define motor1_2 11
volatile int count = 0;
int prev = 0;
float vel = 0.0;
float vfilt = 0;
float v_prev = 0;
unsigned long pt = 0l;
unsigned long pt_1 = 0l;
unsigned long ct = 0l;
boolean dir = true;
int n =0;
void setup() {
  Serial.begin(9600);
  pinMode(motor1_fr,OUTPUT);
  pinMode(motor1_2,OUTPUT);
  pinMode(motor1_1,OUTPUT);
  pinMode(2,INPUT);
  pinMode(3,INPUT);
  attachInterrupt(digitalPinToInterrupt(2), IntA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), IntB, CHANGE);
  Serial.println(0);
  digitalWrite(motor1_fr,HIGH);
  digitalWrite(motor1_1,HIGH);
  digitalWrite(motor1_2,LOW);
}

void loop() {
  ct = millis();
  if(ct - pt>20){
    vel = (60*count)/(44*4.4*0.02);
    count = 0;
    //vfilt = -0.3067*vfilt + 0.6533*(vel+v_prev);//40ms-15hz
    vfilt = 0.0297*vfilt + 0.4852*(vel+v_prev);//20ms-15hz
    //vfilt = 0.3593*vfilt + 0.3203*(vel+v_prev);//10ms-15hz
    v_prev = vel;
    //Serial.print(vel);
    //Serial.print(",");
    Serial.println(vfilt);
    pt = ct;
  }
  if(ct-pt_1>=2000 && n<6){
    dir = !dir;
    reverse(dir);
    pt_1 = ct;
    n++;
    if(n==6){
       digitalWrite(motor1_fr,LOW);
    }
  }
}
void reverse(boolean b){
  if(b){
    digitalWrite(motor1_fr,HIGH);
    digitalWrite(motor1_1,HIGH);
    digitalWrite(motor1_2,LOW);
  }else{
   digitalWrite(motor1_1,LOW);
   digitalWrite(motor1_2,LOW);
  }
}
void IntA(){
  if(digitalRead(2)){//AH
    if(digitalRead(3)){//B=1
      count--;
    }else{
      count++;
    }
  }else{//AL
    if(digitalRead(3)){//B=1
      count++;
    }else{
      count--;
    }
  }
}

void IntB(){
  if(digitalRead(3)){//BH
    if(digitalRead(2)){//A=1
      count++;
    }else{
      count--;
    }
  }else{//BL
    if(digitalRead(2)){//A=1
      count--;
    }else{
      count++;
    }
  }
}
