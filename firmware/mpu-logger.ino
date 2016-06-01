#include <Wire.h>

uint32_t meas_time;
volatile bool data_ready;
int16_t mag_sens_adj_x, mag_sens_adj_y, mag_sens_adj_z;


// CRC-8-CCITT lookup table for the polynomial x^8 + x^2 + x^1 + x^0 (0x07)
static const PROGMEM uint8_t crc8_table[] = {
  0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31,
  0x24, 0x23, 0x2a, 0x2d, 0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65,
  0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d, 0xe0, 0xe7, 0xee, 0xe9,
  0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
  0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1,
  0xb4, 0xb3, 0xba, 0xbd, 0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2,
  0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea, 0xb7, 0xb0, 0xb9, 0xbe,
  0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
  0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16,
  0x03, 0x04, 0x0d, 0x0a, 0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42,
  0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a, 0x89, 0x8e, 0x87, 0x80,
  0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
  0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6, 0xcf, 0xc8,
  0xdd, 0xda, 0xd3, 0xd4, 0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c,
  0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44, 0x19, 0x1e, 0x17, 0x10,
  0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
  0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f,
  0x6a, 0x6d, 0x64, 0x63, 0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b,
  0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13, 0xae, 0xa9, 0xa0, 0xa7,
  0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
  0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef,
  0xfa, 0xfd, 0xf4, 0xf3
};

static inline uint8_t crc8_byte(uint8_t crc, uint8_t data) {
  return pgm_read_byte(crc8_table + (crc ^ data));
}

static inline int16_t mag_adjust(int16_t value, int16_t sens_adj) {
  if (value >= 4095)
    return 32767;
  else if (value <= -4095)
    return -32768;
  else  
    return value + ((value * sens_adj) >> 8);
}

void setup() {
  Serial.begin(250000);
  Wire.begin();
  pinMode(13, OUTPUT);
  delay(100);

  Serial.print("mpu-logger");

  // Reset all registers
  Wire.beginTransmission(0x68);
  Wire.write(107);
  Wire.write(0x80);
  Wire.endTransmission();

  delay(100);
  
  // Enable Slave I2C bypass
  Wire.beginTransmission(0x68);
  Wire.write(55);
  Wire.write(0x22);
  Wire.endTransmission();

  // Configure interrupts
  Wire.beginTransmission(0x68);
  Wire.write(56);
  Wire.write(0x01);
  Wire.endTransmission();  

  // Set MPU clock source to X axis gyroscope
  Wire.beginTransmission(0x68);
  Wire.write(107);
  Wire.write(1);
  Wire.endTransmission();
  
  // Trigger AK8975 measurement
  Wire.beginTransmission(0x0C);
  Wire.write(0x0A);
  Wire.write(0x01);
  Wire.endTransmission();

  // Wait for conversion to finish
  delay(10);

  // Set AK8975 fuse ROM access mode
  Wire.beginTransmission(0x0C);
  Wire.write(0x0A);
  Wire.write(0x0F);
  Wire.endTransmission();

  // Read AK8975 sensitivity adjustment data
  Wire.beginTransmission(0x0C);
  Wire.write(0x10);
  Wire.endTransmission(true);
  Wire.requestFrom(0x0C, 3);

  uint8_t asax, asay, asaz;
  asax = Wire.read();
  asay = Wire.read();
  asaz = Wire.read();
  
  mag_sens_adj_x = asax - 128;
  mag_sens_adj_y = asay - 128;
  mag_sens_adj_z = asaz - 128;

  // Set AK8975 back to power-down mode
  Wire.beginTransmission(0x0C);
  Wire.write(0x0A);
  Wire.write(0x0F);
  Wire.endTransmission();
  
  // Set MPU sample rate
  Wire.beginTransmission(0x68);
  Wire.write(25);
  Wire.write(79);
  Wire.endTransmission();

  attachInterrupt(digitalPinToInterrupt(2), mpu_interrupt, RISING);
}

void loop() {
  // Copy interrupt data
  noInterrupts();
  bool data_ready_copy = data_ready;
  uint32_t meas_time_copy = meas_time;
  data_ready = false;
  interrupts();

  // Barrier waiting for interrupt
  if (!data_ready_copy)
    return;
  
  Wire.beginTransmission(0x0C);
  Wire.write(0x03);
  Wire.endTransmission(false);
  Wire.requestFrom(0x0C, 6);

  int16_t ax, ay, az, gx, gy, gz, temp, mx, my, mz;

  // Get the magnetometer data
  mx = Wire.read() | Wire.read() << 8;
  my = Wire.read() | Wire.read() << 8;
  mz = Wire.read() | Wire.read() << 8;

  // Adjust the magnetometer sensitivity
  mx = mag_adjust(mx, mag_sens_adj_x);
  my = mag_adjust(my, mag_sens_adj_y);
  mz = mag_adjust(mz, mag_sens_adj_z);

  // Trigger AK8975 measurement
  Wire.beginTransmission(0x0C);
  Wire.write(0x0A);
  Wire.write(0x01);
  Wire.endTransmission();  

  Wire.beginTransmission(0x68);
  Wire.write(58);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 15);
  
  uint8_t int_status = Wire.read();
  ax = Wire.read() << 8 | Wire.read();
  ay = Wire.read() << 8 | Wire.read();
  az = Wire.read() << 8 | Wire.read();
  temp = Wire.read() << 8 | Wire.read();
  gx = Wire.read() << 8 | Wire.read();
  gy = Wire.read() << 8 | Wire.read();
  gz = Wire.read() << 8 | Wire.read();

  Serial.write('A');
  Serial.write(meas_time_copy >> 24);
  Serial.write(meas_time_copy >> 16);
  Serial.write(meas_time_copy >> 8);
  Serial.write(meas_time_copy);
  Serial.write(ax >> 8);
  Serial.write(ax);
  Serial.write(ay >> 8);
  Serial.write(ay);
  Serial.write(az >> 8);
  Serial.write(az);
  Serial.write(temp >> 8);
  Serial.write(temp);
  Serial.write(gx >> 8);
  Serial.write(gx);
  Serial.write(gy >> 8);
  Serial.write(gy);
  Serial.write(gz >> 8);
  Serial.write(gz);
  Serial.write(mx >> 8);
  Serial.write(mx);
  Serial.write(my >> 8);
  Serial.write(my);
  Serial.write(mz >> 8);
  Serial.write(mz);

  uint8_t crc;
  crc = crc8_byte(0, meas_time_copy >> 24);
  crc = crc8_byte(crc, meas_time_copy >> 16);
  crc = crc8_byte(crc, meas_time_copy >> 8);
  crc = crc8_byte(crc, meas_time_copy);
  crc = crc8_byte(crc, ax >> 8);
  crc = crc8_byte(crc, ax);
  crc = crc8_byte(crc, ay >> 8);
  crc = crc8_byte(crc, ay);
  crc = crc8_byte(crc, az >> 8);
  crc = crc8_byte(crc, az);
  crc = crc8_byte(crc, temp >> 8);
  crc = crc8_byte(crc, temp);
  crc = crc8_byte(crc, gx >> 8);
  crc = crc8_byte(crc, gx);
  crc = crc8_byte(crc, gy >> 8);
  crc = crc8_byte(crc, gy);
  crc = crc8_byte(crc, gz >> 8);
  crc = crc8_byte(crc, gz);
  crc = crc8_byte(crc, mx >> 8);
  crc = crc8_byte(crc, mx);
  crc = crc8_byte(crc, my >> 8);
  crc = crc8_byte(crc, my);
  crc = crc8_byte(crc, mz >> 8);
  crc = crc8_byte(crc, mz);
  Serial.write(crc);  
}

void mpu_interrupt() {
  meas_time = millis();
  data_ready = true;
}

