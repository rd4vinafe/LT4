#include <Wire.h>

// 🧭 I2C Communication
#define SLAVE_ADDR        0x12  // STM32 I2C slave address

// 📡 Sensor Commands
#define SS1_ADDR          0x01  // Sensor 1: returns 4 bytes (float)
#define SS2_ADDR          0x02  // Sensor 2: returns 4 bytes (float)
#define SSZR_ADDR         0x03  // Zero detect: returns 1 byte (status)

// ⚙️ Control Commands
#define BTA1_ADDR         0x04  // Triac 1 control: 2 bytes (duty || on/off)
#define BTA2_ADDR         0x05  // Triac 2 control: 2 bytes (duty || on/off)
#define POWER_SWITCH_ADDR 0x06  // Power switch: 1 byte (on/off)

union {
  byte b[4];
  float f;
} data;

// 📡 Read float from Sensor 1
float readSensor1() {
  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(SS1_ADDR);
  Wire.endTransmission();
  delay(50);
  Wire.requestFrom(SLAVE_ADDR, 4);
  if (Wire.available() >= 4) {
    data.b[0] = Wire.read();
    data.b[1] = Wire.read();
    data.b[2] = Wire.read();
    data.b[3] = Wire.read();
  }
  return data.f;
}

// 📡 Read float from Sensor 2
float readSensor2() {
  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(SS2_ADDR);
  Wire.endTransmission();
  delay(50);
  Wire.requestFrom(SLAVE_ADDR, 4);
  if (Wire.available() >= 4) {
    data.b[0] = Wire.read();
    data.b[1] = Wire.read();
    data.b[2] = Wire.read();
    data.b[3] = Wire.read();
  }
  return data.f;
}

// 📡 Read zero detect status (1 byte)
uint8_t readZeroDetect() {
  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(SSZR_ADDR);
  Wire.endTransmission();
  delay(50);
  Wire.requestFrom(SLAVE_ADDR, 1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0;
}
void setPowerSwitch(bool on) {
  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(POWER_SWITCH_ADDR);     // Command address for power switch
  Wire.write(on ? 1 : 0);            // Send 1 for ON, 0 for OFF
  Wire.endTransmission();
}

void togglePowerSwitch() {
  static bool powerState = false;  // Initial state: OFF
  powerState = !powerState;  // Flip the state
  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(POWER_SWITCH_ADDR);
  Wire.write(powerState ? 1 : 0);  // Send 1 for ON, 0 for OFF
  Wire.endTransmission();

  Serial.print("Power Switch toggled to: ");
  Serial.println(powerState ? "ON" : "OFF");
}



void setTriac1(bool on) {
  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(BTA1_ADDR);             // Triac 1 command
  Wire.write(127);                  // duty ex: 100% =0xff(255)
  Wire.write(on ? 1 : 0);            
  Wire.endTransmission();

  Serial.print("Triac1: ");
  Serial.println(on ? "ON" : "OFF");
}

void setTriac2(bool on) {
  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(BTA2_ADDR);             // Triac 2 command
  Wire.write(127);                  // duty ex: 100% =0xff(255)
  Wire.write(on ? 1 : 0);            // 1 = ON, 0 = OFF
  Wire.endTransmission();

  Serial.print("Triac2: ");
  Serial.println(on ? "ON" : "OFF");
}

void toggleTriac1() {
  static bool triac1State = false;
  triac1State = !triac1State;
  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(BTA1_ADDR);               // Command for Triac 1
  Wire.write(127);                    // duty ex: 100% =0xff(255) 
  Wire.write(triac1State ? 1 : 0);     // ON = 1, OFF = 0
  Wire.endTransmission();

  Serial.print("Triac1 toggled to: ");
  Serial.println(triac1State ? "ON" : "OFF");
}

void toggleTriac2() {
  static bool triac2State = false;
  triac2State = !triac2State;
  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(BTA2_ADDR);               // Command for Triac 2
  Wire.write(127);                    // duty ex: 100% =0xff(255)
  Wire.write(triac2State ? 1 : 0);     // ON = 1, OFF = 0
  Wire.endTransmission();

  Serial.print("Triac2 toggled to: ");
  Serial.println(triac2State ? "ON" : "OFF");
}


void setup() {
  Wire.begin();
  Serial.begin(9600);
}

void loop() {
  float temp1 = readSensor1();
  delay(100);
  float temp2 = readSensor2();
  delay(100);
  uint8_t zero = readZeroDetect();
  delay(100);
  // togglePowerSwitch(); // always on to test triac
  // delay(100);
  toggleTriac1();
  delay(100);
  toggleTriac2();
  delay(100);
  Serial.print("Sensor1 = ");
  Serial.println(temp1, 6);

  Serial.print("Sensor2 = ");
  Serial.println(temp2, 6);

  Serial.print("Zero Detect = ");
  Serial.println(zero);

  delay(100);
}