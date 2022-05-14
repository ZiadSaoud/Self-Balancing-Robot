#include <Fuzzy.h>
#include <Wire.h>
#include <Kalman.h>
#include<SPI.h>

const uint8_t IMUAddress = 0x68; 
const uint16_t I2C_TIMEOUT = 1000; 

Kalman kalmanY;
double kalAngleY; // Calculated angle using a Kalman filter

double accX, accY, accZ;//Acc data
double gyroY;//Gyro data

uint32_t timer;
uint8_t i2cData[14]; // Buffer for Acc_Gyro data

Fuzzy *controller;

int encoder_count[2];
void setup() {
  //Serial.begin(9600);
  SPI.begin();
  setUp_Acc_Gyro();
  initialize_filter();
  controller = fuzzySetUP();
}

void loop() {
  update_Acc_Gyro();
  double dt = (double)(micros() - timer) / 1000000; // Calculate delta t
  timer = micros();
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  double gyroYrate = gyroY / 131.0; // Convert raw data to deg/s
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
  //Serial.print(kalAngleY);Serial.print("\t");
  controller->setInput(1, kalAngleY * DEG_TO_RAD);
  controller->setInput(2, gyroYrate * DEG_TO_RAD);
  controller->fuzzify();
  float out = controller->defuzzify(1);
  if(abs(out) > 7.2){
    out = (out > 0)?7.2:-7.2; 
  }
  int pwm = round(mapf(out,-7.2, 7.2, -255.0, 255.0));
  send_receive_Ints(pwm+40, pwm);
}

Fuzzy* fuzzySetUP(){
  Fuzzy *fuzzy = new Fuzzy();
  //Instantiating a FuzzyInput object Theta
  FuzzyInput *Theta = new FuzzyInput(1);
  // Instantiating a FuzzySet object Negative 
  FuzzySet *Negative = new FuzzySet(-PI, -PI, -1, -0.02);
  //add the fuzzy set negative to the fuzzy input theta
  Theta->addFuzzySet(Negative);
  FuzzySet *Zero = new FuzzySet(-1, -0.02, -0.02, 1);
  //add the fuzzy set negative to the fuzzy input theta
  Theta->addFuzzySet(Zero);
  //Positive
  FuzzySet *Positive = new FuzzySet(-0.02, 1, PI, PI);
  //add the fuzzy set Positive to the fuzzy input theta
  Theta->addFuzzySet(Positive);
  // Including the FuzzyInput into Fuzzy
  fuzzy->addFuzzyInput(Theta);

  
  //Instantiating a FuzzyInput object Theta_dot
  FuzzyInput *Theta_dot = new FuzzyInput(2);
  // Instantiating a FuzzySet object Negative 
  FuzzySet *Negative_v = new FuzzySet(-5, -5, -5, 5);
  //add the fuzzy set negative to the fuzzy input theta_dot
  Theta_dot->addFuzzySet(Negative_v);
  //Positive
  FuzzySet *Positive_v = new FuzzySet(-5, 5, 5, 5);
  //add the fuzzy set Positive to the fuzzy input theta
  Theta_dot->addFuzzySet(Positive_v);
  // Including the FuzzyInput into Fuzzy
  fuzzy->addFuzzyInput(Theta_dot);

  // Instantiating a FuzzyOutput objects
  FuzzyOutput *Voltage = new FuzzyOutput(1);
  // Instantiating a FuzzySet object Negative Large
  FuzzySet *NLarge = new FuzzySet(-80, -80, -80, -16);
  //add the fuzzy set negative to the fuzzy output voltage
  Voltage->addFuzzySet(NLarge);
  // Instantiating a FuzzySet object Negative Medium
  FuzzySet *NMedium = new FuzzySet(-32, -16, -16, 0);
  //add the fuzzy set Negative Medium to the fuzzy output voltage
  Voltage->addFuzzySet(NMedium);
  FuzzySet *Zero_v = new FuzzySet(-7.5, -1.5, 1.5, 7.5);
  //add the fuzzy set Negative Medium to the fuzzy output voltage
  Voltage->addFuzzySet(Zero_v);
  // Instantiating a FuzzySet object Positive Medium
  FuzzySet *PMedium = new FuzzySet(0, 16, 16, 32);
  //add the fuzzy set Positive Medium to the fuzzy output voltage
  Voltage->addFuzzySet(PMedium);
  // Instantiating a FuzzySet object Positive Large
  FuzzySet *PLarge = new FuzzySet(16, 80, 80, 80);
  //add the fuzzy set Positive Medium to the fuzzy output voltage
  Voltage->addFuzzySet(PLarge);
  // Including the FuzzyInput into Fuzzy
  fuzzy->addFuzzyOutput(Voltage);

  // Building FuzzyRule "IF Theta = Negative AND IF Theta_dot = Negative THEN Voltage = NL"
  // Instantiating a FuzzyRuleAntecedent objects
  FuzzyRuleAntecedent *ThetaNegative_Theta_DotNegative = new FuzzyRuleAntecedent();
  // Creating a FuzzyRuleAntecedent with just a single FuzzySet
  ThetaNegative_Theta_DotNegative->joinWithAND(Negative, Negative_v);
  // Instantiating a FuzzyRuleConsequent objects
  FuzzyRuleConsequent *thenVoltage_NL = new FuzzyRuleConsequent();
  // Including a FuzzySet to this FuzzyRuleConsequent
  thenVoltage_NL->addOutput(NLarge);
  // Instantiating a FuzzyRule objects
  FuzzyRule *fuzzyRule01 = new FuzzyRule(1, ThetaNegative_Theta_DotNegative, thenVoltage_NL);
  // Including the FuzzyRule into Fuzzy
  fuzzy->addFuzzyRule(fuzzyRule01);

  // Building FuzzyRule "IF Theta = Negative AND IF Theta_dot = Positive THEN Voltage = PM"
  // Instantiating a FuzzyRuleAntecedent objects
  FuzzyRuleAntecedent *ThetaNegative_Theta_DotPositive = new FuzzyRuleAntecedent();
  // Creating a FuzzyRuleAntecedent with just a single FuzzySet
  ThetaNegative_Theta_DotPositive->joinWithAND(Negative, Positive_v);
  // Instantiating a FuzzyRuleConsequent objects
  FuzzyRuleConsequent *thenVoltage_PM = new FuzzyRuleConsequent();
  // Including a FuzzySet to this FuzzyRuleConsequent
  thenVoltage_PM->addOutput(PMedium);
  // Instantiating a FuzzyRule objects
  FuzzyRule *fuzzyRule02 = new FuzzyRule(2, ThetaNegative_Theta_DotPositive, thenVoltage_PM);
  // Including the FuzzyRule into Fuzzy
  fuzzy->addFuzzyRule(fuzzyRule02);


  // Building FuzzyRule "IF Theta = Positive AND IF Theta_dot = Positive THEN Voltage = PL"
  // Instantiating a FuzzyRuleAntecedent objects
  FuzzyRuleAntecedent *ThetaPositive_Theta_DotPositive = new FuzzyRuleAntecedent();
  // Creating a FuzzyRuleAntecedent with just a single FuzzySet
  ThetaPositive_Theta_DotPositive->joinWithAND(Positive, Positive_v);
  // Instantiating a FuzzyRuleConsequent objects
  FuzzyRuleConsequent *thenVoltage_PL = new FuzzyRuleConsequent();
  // Including a FuzzySet to this FuzzyRuleConsequent
  thenVoltage_PL->addOutput(PLarge);
  // Instantiating a FuzzyRule objects
  FuzzyRule *fuzzyRule03 = new FuzzyRule(3, ThetaPositive_Theta_DotPositive, thenVoltage_PL);
  // Including the FuzzyRule into Fuzzy
  fuzzy->addFuzzyRule(fuzzyRule03);

  // Building FuzzyRule "IF Theta = Positive AND IF Theta_dot = Negative THEN Voltage = NM"
  // Instantiating a FuzzyRuleAntecedent objects
  FuzzyRuleAntecedent *ThetaPositive_Theta_DotNegative = new FuzzyRuleAntecedent();
  // Creating a FuzzyRuleAntecedent with just a single FuzzySet
  ThetaPositive_Theta_DotNegative->joinWithAND(Positive, Negative_v);
  // Instantiating a FuzzyRuleConsequent objects
  FuzzyRuleConsequent *thenVoltage_NM = new FuzzyRuleConsequent();
  // Including a FuzzySet to this FuzzyRuleConsequent
  thenVoltage_NM->addOutput(NMedium);
  // Instantiating a FuzzyRule objects
  FuzzyRule *fuzzyRule04 = new FuzzyRule(4, ThetaPositive_Theta_DotNegative, thenVoltage_NM);
  // Including the FuzzyRule into Fuzzy
  fuzzy->addFuzzyRule(fuzzyRule04);

  // Building FuzzyRule "IF Theta = Zero AND IF Theta_dot = Negative THEN Voltage = Zero"
  // Instantiating a FuzzyRuleAntecedent objects
  FuzzyRuleAntecedent *ThetaZero_Theta_DotNegative = new FuzzyRuleAntecedent();
  // Creating a FuzzyRuleAntecedent with just a single FuzzySet
  ThetaZero_Theta_DotNegative->joinWithAND(Zero, Negative_v);
  // Instantiating a FuzzyRuleConsequent objects
  FuzzyRuleConsequent *thenVoltage_Z = new FuzzyRuleConsequent();
  // Including a FuzzySet to this FuzzyRuleConsequent
  thenVoltage_Z->addOutput(Zero_v);
  // Instantiating a FuzzyRule objects
  FuzzyRule *fuzzyRule05 = new FuzzyRule(5, ThetaZero_Theta_DotNegative, thenVoltage_Z);
  // Including the FuzzyRule into Fuzzy
  fuzzy->addFuzzyRule(fuzzyRule05);

  // Building FuzzyRule "IF Theta = Zero AND IF Theta_dot = Positive THEN Voltage = Zero"
  // Instantiating a FuzzyRuleAntecedent objects
  FuzzyRuleAntecedent *ThetaZero_Theta_DotPositive = new FuzzyRuleAntecedent();
  // Creating a FuzzyRuleAntecedent with just a single FuzzySet
  ThetaZero_Theta_DotPositive->joinWithAND(Zero, Positive_v);
  // Instantiating a FuzzyRuleConsequent objects
  FuzzyRuleConsequent *thenVoltage_Z_z = new FuzzyRuleConsequent();
  // Including a FuzzySet to this FuzzyRuleConsequent
  thenVoltage_Z_z->addOutput(Zero_v);
  // Instantiating a FuzzyRule objects
  FuzzyRule *fuzzyRule06 = new FuzzyRule(6, ThetaZero_Theta_DotPositive, thenVoltage_Z_z);
  // Including the FuzzyRule into Fuzzy
  fuzzy->addFuzzyRule(fuzzyRule06);  
  return fuzzy;
}

void setUp_Acc_Gyro(){
  Wire.begin();
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode
  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    //Serial.print(F("Error reading sensor"));
    while (1);
  }
  delay(100); // Wait for sensor to stabilize
}

void update_Acc_Gyro(){
  while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  //tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  //gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  //gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;
}

void initialize_filter(){
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  kalmanY.setAngle(pitch);
  timer = micros();
}


uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop) {
  return i2cWrite(registerAddress, &data, 1, sendStop);
}

uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop) {
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data, length);
  uint8_t rcode = Wire.endTransmission(sendStop); 
  if (rcode) {
    //Serial.print(F("i2cWrite failed: "));
    //Serial.println(rcode);
  }
  return rcode;
}

uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes) {
  uint32_t timeOutTimer;
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  uint8_t rcode = Wire.endTransmission(false); 
  if (rcode) {
    //Serial.print(F("i2cRead failed: "));
    //Serial.println(rcode);
    return rcode; 
  }
  Wire.requestFrom(IMUAddress, nbytes, (uint8_t)true);
  for (uint8_t i = 0; i < nbytes; i++) {
    if (Wire.available())
      data[i] = Wire.read();
    else {
      timeOutTimer = micros();
      while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
      if (Wire.available())
        data[i] = Wire.read();
      else {
        //Serial.println(F("i2cRead timeout"));
        return 5; 
      }
    }
  }
  return 0; 
}

void send_receive_Ints(int x, int y){
  byte dataToSend[] = {highByte(x), lowByte(x), highByte(y), lowByte(y)};
  byte dataToReceive[4];
  for(int i=0; i<4; i++){
    dataToReceive[i] = SPI.transfer(dataToSend[i]);
  }
  encoder_count[0] = (int16_t)((dataToReceive[0]<<8) | dataToReceive[1]);
  encoder_count[1] = (int16_t)((dataToReceive[2]<<8) | dataToReceive[3]);
}

float mapf(float x,float min_1,float max_1,float min_2,float max_2){
  float z = (x-min_1)*(max_2-min_2)/(max_1-min_1)+min_2;
  return z;
}
