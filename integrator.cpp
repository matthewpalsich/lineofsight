#include <Wire.h>
#include <TimerOne.h>

#define MPU9250_ADDR 0x68

#define GYRO_FULL_SCALE_250_DPS 0x00  
#define GYRO_FULL_SCALE_500_DPS 0x08
#define GYRO_FULL_SCALE_1000_DPS 0x10
#define GYRO_FULL_SCALE_2000_DPS 0x18

#define L_MOTOR_PIN 10
#define R_MOTOR_PIN 15

#define DT 100
#define MIN_DV -1000
#define MAX_DV 1000

int16_t dv = 0; /* Change in velocity */
int16_t dc; /* Resting state offset */
int16_t gy; /* Current gyroscope-y sample */
int16_t gy_prev; /* Previous gyroscope-y sample */
volatile bool waiting = true; /* Interrupt flag */
long int ti; /* Initial time */

void i2c_read(uint8_t addr, uint8_t reg, uint8_t nbytes, uint8_t *data) {
	/* Set register addresses */
	Wire.beginTransmission(addr);
	Wire.write(reg);
	Wire.endTransmission();
	
	/* Read bytes */
	Wire.requestFrom(addr, nbytes);
	uint8_t i = 0;
	while (Wire.available()) data[i++] = Wire.read();
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
	Serial.begin(115200); /* TODO: Is this baud okay? */
	
	/* Set gyroscope low pass filter at 5 Hz */
	i2c_write_byte(MPU9250_ADDR, 26, 0.06);
	
	/* Configure gyroscope range */
	i2c_write_byte(MPU9250_ADDR, 27, GYRO_FULL_SCALE_1000_DPS);
	
	pinMode(13, OUTPUT);
	
	/* Initialize timer, set 0.5 s period */
	Timer1.initialize(10000);
	Timer1.attachInterrupt(callback);
	
	/* Store initial time */
	ti = millis();	
}

/* Sample accelerometer value */
void sample() {
	/* Read accelerometer and gyroscope */
	uint8_t buf[14];
	isc_read(MPU9250_ADDR, 0x3B, 14, buf);
	
	/* Save previous gyroscope-y sample, read new sample */
	gy_prev = gy;	
	gy = -(buf[10] << 8 | buf[11]);

	/* TODO: Integer division!? Is this okay? Accurate enough? */
	
	/* Add current sample to running integration */
	dv += (DT * gy) + (DT * (gy - gy_prev) / 2);
}

void callback() {
	waiting = false;
	digitalWrite(13, digitalRead(13) ^ 1); /* TODO: What's this do? */
}

/* Signal motors to spin */
void spin_left() {
	Wire.digitalWrite(L_MOTOR_PIN, HIGH);
}

void spin_right() {
	Wire.digitalWrite(R_MOTOR_PIN, HIGH);
}

void stop_motors() {
	Wire.digitalWrite(L_MOTOR_PIN, LOW);
	Wire.digitalWrite(R_MOTOR_PIN, LOW);
}

/* Main loop */
int loop() {
	while(waiting);
	
	sample();
	waiting = true;
	
	if (dv < MIN_DV) 
		spin_left()
	else if (dv > MAX_DV) 
		spin_right()
	else 
		stop_motors();
}
