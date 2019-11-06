#include "Particle.h"

#include "LIS3DH.h"

// This code only works on Gen 3 devices (Argon, Boron, Xenon, B Series SoM)
// It expects and Ethernet FeatherWing. It was tested with a Xenon that was not part of a mesh network.
STARTUP(System.enableFeature(FEATURE_ETHERNET_DETECTION));

SYSTEM_THREAD(ENABLED);

// You must use SEMI_AUTOMATIC or MANUAL mode, not automatic (explained below)
SYSTEM_MODE(SEMI_AUTOMATIC);

SerialLogHandler logHandler;

// Print 4 samples per second to serial
const unsigned long PRINT_SAMPLE_PERIOD = 250;

// LIS3DH is to secondary SPI (SPI1) on reconfigured pins with A3 as the CS pin
LIS3DHSPI accel(SPI1, A3);

unsigned long lastPrintSample = 0;

void reconfigureSpi();

void setup() {

	// Optional, for testing: Wait 10 seconds or until serial is connect before proceeding.
	waitFor(Serial.isConnected, 10000);

	// Because the SPI interface needs to be brought up briefly on the old pins, we need to
	// do this before bringing Ethernet up.
	reconfigureSpi();

	// Now bring up the Ethernet that we've freed the D pins from SPI1.
	Ethernet.on();
	Ethernet.connect();
	Particle.connect();

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

// port: 0 or 1
// pin: 0 - 31
uint32_t pselConfig(bool connect, int port, int pin) {
	uint32_t result = 0;

	if (connect) {
		result |= 0x8000000;
	}
	result |= port << 5;
	result |= pin;

	return result;
}

void reconfigureSpi() {
	// This is the magic for reconfiguring SPI.
	// This code only works on Gen 3 devices (Argon, Boron, Xenon, B Series SoM)

	// This is what we want to remap to, but you can use different pins.
	// SPI   Pin  Hardware Pin
	// SCK   A0   P0.3
	// MOSI  A1   P0.4
	// MISO  A2   P0.28

	// The pin mapping table is handy for finding the hardware pin numbers:
	// https://docs.particle.io/reference/hardware/pin-info/?m=table&sort=num

	// You must bring up SPI1 on the original pins first, because otherwise SPI1.begin() will
	// overwrite the reconfiguration and revert it back to the old pins.
	SPI1.begin();


	// SCK and MOSI need to be configured as OUTPUT. MISO is INPUT.
	pinMode(A0, OUTPUT); // SCK
	pinMode(A1, OUTPUT); // MOSI
	pinMode(A2, INPUT);  // MISO

	// CS is configured above in the LIS3DHSPI object construction. It doesn't affect pin reconfiguration.

	// We reconfigure SPI1, which is nRF52 SPIM2. The addresses are in the nRF52 Product Specification.
	uint8_t *pBase = (uint8_t *)0x40023000;

	// ENABLE offset 0x500
	uint32_t *pENABLE = (uint32_t *)&pBase[0x500];

	// PSEL.SCK offset 0x508
	uint32_t *pPSEL_SCK = (uint32_t *)&pBase[0x508];

	// PSEL.MOSI offset 0x50c
	uint32_t *pPSEL_MOSI = (uint32_t *)&pBase[0x50c];

	// PSEL.MISO OFFSET 0x510
	uint32_t *pPSEL_MISO = (uint32_t *)&pBase[0x510];

	// Standard pin config for SPI1
	// SCK   D2   P1.01
	// MOSI  D3   P1.02
	// MISO  D4   P1.08

	// Disconnect the old pins
	*pPSEL_SCK = pselConfig(false, 1, 1);	// D2
	*pPSEL_MOSI = pselConfig(false, 1, 2);	// D3
	*pPSEL_MISO = pselConfig(false, 1, 8);	// D4

	// Restore the old pins back to INPUT mode (MISO was already input)
	pinMode(D2, INPUT); // SCK
	pinMode(D3, INPUT); // MOSI

	// Disable SPIM
	*pENABLE = 0;

	// Reconnect to the new pins
	*pPSEL_SCK = pselConfig(true, 0, 3); 	// A0
	*pPSEL_MOSI = pselConfig(true, 0, 4); 	// A1
	*pPSEL_MISO = pselConfig(true, 0, 28);	// A2

	// Reenable SPIM
	*pENABLE = 0x7;

	// Log.info("SCK=%lx MOSI=%lx MISO=%lx", *pPSEL_SCL, *pPSEL_MOSI, *pPSEL_MISO);

}
