#include "Particle.h"

#include "LIS3DH.h"

// Sample to print accelerometer samples to serial.

SYSTEM_THREAD(ENABLED);

SYSTEM_MODE(MANUAL);

SerialLogHandler logHandler;

// Print 4 samples per second to serial
const unsigned long PRINT_SAMPLE_PERIOD = 250;

// LIS3DH is to secondary SPI (SPI1) on the D pins, with D5 as the CS pin
LIS3DHSPI accel(SPI1, D5);

unsigned long lastPrintSample = 0;

void setup() {
	SPI1.begin();

	reconfigureSpi();

	// Initialize sensors
	LIS3DHConfig config;
	config.setAccelMode(LIS3DH::RATE_100_HZ);

	bool setupSuccess = accel.setup(config);
	Log.info("setupSuccess=%d", setupSuccess);
}

void loop() {

	if (millis() - lastPrintSample >= PRINT_SAMPLE_PERIOD) {
		lastPrintSample = millis();

		LIS3DHSample sample;
		if (accel.getSample(sample)) {
			Log.info("%d,%d,%d", sample.x, sample.y, sample.z);
		}
		else {
			Log.info("no sample");
		}
	}
}
