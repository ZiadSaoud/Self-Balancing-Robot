#include <SPI.h>
byte dataToReceive[4];
byte dataToSend[] = {0x00, 0x00, 0x00, 0x00};
volatile byte index = 0;
volatile int pwm_r = 0;
volatile int pwm_l = 0;

#define motor_l_pwm 6
#define motor_r_pwm 3
#define motor_l_1   7
#define motor_l_2   8
#define motor_r_1   4
#define motor_r_2   5

void setup() {
  SPI_SetUp();
  setupMotors();
}

void loop() {
  // put your main code here, to run repeatedly:
  if(pwm_l >= 0){
    digitalWrite(motor_l_1, HIGH);
    digitalWrite(motor_l_2, LOW);
    analogWrite(motor_l_pwm,pwm_l);
  }else{
    digitalWrite(motor_l_1, LOW);
    digitalWrite(motor_l_2, HIGH);
    analogWrite(motor_l_pwm,abs(pwm_l));
  }
  if(pwm_r >= 0){
    digitalWrite(motor_r_1, HIGH);
    digitalWrite(motor_r_2, LOW);
    analogWrite(motor_r_pwm,pwm_r);
  }else{
    digitalWrite(motor_r_1, LOW);
    digitalWrite(motor_r_2, HIGH);
    analogWrite(motor_r_pwm,abs(pwm_r));
  }
}

// SPI interrupt routine
ISR (SPI_STC_vect){
  uint8_t oldsrg = SREG;
  cli();
  SPDR = dataToSend[index]; // Load the SPI data register with data to shift out
  dataToReceive[index++] = SPDR;
  if(index == 4){
    index=0;
    pwm_r = (int16_t)((dataToReceive[0]<<8) | dataToReceive[1]);
    pwm_l = (int16_t)((dataToReceive[2]<<8) | dataToReceive[3]);
  }
  SREG = oldsrg;
}

void SPI_SetUp(){
  SPCR |= bit(SPE);//Enable SPI
  pinMode(MISO, OUTPUT);//Make MISO pin as OUTPUT
  SPI.attachInterrupt();//Attach SPI interrupt
}
void setupMotors(){
  pinMode(motor_l_pwm, OUTPUT);
  pinMode(motor_r_pwm, OUTPUT);
  pinMode(motor_l_1, OUTPUT);
  pinMode(motor_l_2, OUTPUT);
  pinMode(motor_r_1, OUTPUT);
  pinMode(motor_r_2, OUTPUT);
}
