#include <Wire.h>

#define MAG3110_ADDRESS 0x0E
#define MAG3110_REGISTER_STATUS 0x00
#define MAG3110_REGISTER_X_MSB 0x01
#define MAG3110_REGISTER_X_LSB 0x02
#define MAG3110_REGISTER_Y_MSB 0x03
#define MAG3110_REGISTER_Y_LSB 0x04
#define MAG3110_REGISTER_Z_MSB 0x05
#define MAG3110_REGISTER_Z_LSB 0x06
#define MAG3110_REGISTER_CTRL_REG1 0x10

unsigned long previousMicros = 0;


void setup() {
  Wire.begin();
  Serial.begin(9600);

  Iniciar_MAG3110(); 
}

void loop() {
 
  

  short x, y, z;
  Leer_MAG3110(x, y, z);  

  // Output the readings
  Serial.print("X: ");
  Serial.print(x);
  Serial.print("   Y: ");
  Serial.print(y);
  Serial.print("   Z: ");
  Serial.println(z);

  unsigned long loopTime = micros() - previousMicros;
  previousMicros = micros();
  Serial.print("Loop time: ");
  Serial.println(loopTime);
  
}

void Iniciar_MAG3110() {
  // Configure MAG3110
  Wire.beginTransmission(MAG3110_ADDRESS);
  Wire.write(MAG3110_REGISTER_CTRL_REG1);
  Wire.write(0b00000001); //80Hz max
  Wire.endTransmission();
}

void Leer_MAG3110(short& x, short& y, short& z) {
  // Read data from MAG3110
  Wire.beginTransmission(MAG3110_ADDRESS);
  Wire.write(MAG3110_REGISTER_X_MSB);
  Wire.endTransmission();
  Wire.requestFrom(MAG3110_ADDRESS, 6);

  // Read X, Y, and Z axis data
  x = (Wire.read() << 8) | Wire.read();
  y = (Wire.read() << 8) | Wire.read();
  z = (Wire.read() << 8) | Wire.read();
}
