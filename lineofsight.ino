#include <Wire.h>
#include <TimerOne.h>

#define MPU9250_ADDR 0x68

#define GYRO_FULL_SCALE_250_DPS 0x00  
#define GYRO_FULL_SCALE_500_DPS 0x08
#define GYRO_FULL_SCALE_1000_DPS 0x10
#define GYRO_FULL_SCALE_2000_DPS 0x18

#define L_MOTOR_PIN 10
#define R_MOTOR_PIN 15

#define BAUD 115200
#define MIN_DV -7000
#define MAX_DV 7000

int32_t dv = 0;         /* Change in velocity */
int16_t dc = -29;           /* Resting state offset */
int16_t gz = 0;           /* Current gyroscope-y sample */
int16_t gz_prev = 0;        /* Previous gyroscope-y sample */
int16_t t = 0;
int16_t t_prev = 0;
int16_t dt = 0;
volatile bool waiting = true;   /* Interrupt flag */
long int ti;          /* Initial time */
int16_t drift_counter = 100;

/* TODO:
 *  [x] Capturing dc value (average of first 5 samples
 *  [ ] Handling start/stop indication by button input
 *    [ ] Configure pin for button input (need additional info from Derek)
 *    [ ] Create interrupt from button press (toggle boolean flag)
 *  [x] Compile code (library dependencies currently unhandled)
 */

void i2c_read(uint8_t addr, uint8_t reg, uint8_t nbytes, uint8_t *data) {
  /* Set register addresses */
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission();
  
  /* Read bytes */
  Wire.requestFrom(addr, nbytes);
  uint8_t i = 0;
  while (Wire.available())
    data[i++] = Wire.read();
}

void i2c_write_byte(uint8_t addr, uint8_t reg, uint8_t data) {
  /* Set register address */
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

void setup() {
  /* Arduino initializations */
  Wire.begin();
  Serial.begin(BAUD); /* TODO: Is this baud okay? */
  
  /* Set gyroscope low pass filter at 5 Hz */
  //i2c_write_byte(MPU9250_ADDR, 26, 0.06);
  
  /* Configure gyroscope range */
  i2c_write_byte(MPU9250_ADDR, 27, GYRO_FULL_SCALE_1000_DPS);
  
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  pinMode(L_MOTOR_PIN, OUTPUT);
  pinMode(R_MOTOR_PIN, OUTPUT);
  
  /* Initialize timer */
  Timer1.initialize(10); 
  Timer1.attachInterrupt(callback);
  
  /* Store initial time */
  ti = millis();  

  
  for (int i = 0; i < 10; i++) {
    sample();
    delay(10);
  } 
  
  /*
  int x = 0;
  for (int i = 0; i < 10; i++) {
    sample();
    x += gz;
      delay(100); //CHANGED FROM 100
    }
  dc = x / 10;
  */
}

void sample() {
  /* Read accelerometer and gyroscope */
  uint8_t buf[14];
  i2c_read(MPU9250_ADDR, 0x3B, 14, buf);

  /* Save previous time sample, read new sample */
  t_prev = t;
  t = (millis() - ti) >> 7;
  dt = t - t_prev;
  
  /* Save previous gyroscope-y sample, read new sample */
  gz_prev = gz; 
  gz = (buf[12] << 8 | buf[13]) - dc;
  
  /* Add current sample to running integration */
  //dv += dt * (gz + ((gz - gz_prev) >> 1));
  dv += (dt * gz) + (dt * ((gz - gz_prev) / 2));
  Serial.print(dv);
  Serial.print('\t');
  Serial.println(gz);

  /*
  drift_counter--;
  if (drift_counter == 0) {
    dv += 1;
    drift_counter = 100;
  }
  */
}

void callback() {
  waiting = false;
}

/* Signal motors to spin */
void spin_left() {
  digitalWrite(L_MOTOR_PIN, HIGH);
}

void spin_right() {
  digitalWrite(R_MOTOR_PIN, HIGH);
}

void stop_motors() {
  digitalWrite(L_MOTOR_PIN, LOW);
  digitalWrite(R_MOTOR_PIN, LOW);
}

/* Main loop */
void loop() {
  while(waiting);
  
  sample();
  waiting = true;
 
  if (dv < MIN_DV) {
    //Serial.println('L');
    spin_left();
    //spin_right();
  } else if (dv > MAX_DV) {
    //Serial.println('R');
    spin_right();
  } else {
    stop_motors();
  }
}

