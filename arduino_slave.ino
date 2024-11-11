#include <SPI.h>
#include <Adafruit_LSM6DSOX.h>
#include <stdint.h>

#define BUFFER_SIZE 48  // 12 floats * 4 bytes per float
#define SS_PIN 10

// Sensor instances and addresses
Adafruit_LSM6DSOX sensor1;
uint8_t sensor1_addr = 0x6A;
Adafruit_LSM6DSOX sensor2;
uint8_t sensor2_addr = 0x6B;

// Variables to hold sensor data
float sensorData[12];  // Array to hold sensor data

volatile uint8_t pos = 0;
volatile bool dataReady = false;
volatile bool transferComplete = false;

uint8_t sendStorage[BUFFER_SIZE];    // Buffer to store bytes to send to master

// SPI interrupt number for the SAM3X8E chip:
#define SPI0_INTERRUPT_NUMBER (IRQn_Type)24

void setup() {
  Serial.begin(115200);
  slaveBegin(SS_PIN);  // Initialize SPI as slave on pin 10
}

void loop() {
  if (transferComplete) {
    transferComplete = false;  // Reset flag

    // Update sensor data
    updateSensorData();

    // Prepare data to send in next SPI transaction
    memcpy(sendStorage, sensorData, sizeof(sensorData));


    // Serial.println(sensorData[11]);
    // Preload TDR with the first byte to send
    REG_SPI0_TDR = sendStorage[0];
    pos = 1;  // Reset position for next transfer
  }
}

void slaveBegin(uint8_t _pin) {
  // Setup the SPI Interrupt registers
  NVIC_ClearPendingIRQ(SPI0_INTERRUPT_NUMBER);
  NVIC_EnableIRQ(SPI0_INTERRUPT_NUMBER);

  // Initialize the SPI device with Arduino default values
  SPI.begin(_pin);
  REG_SPI0_CR = SPI_CR_SWRST;     // Reset SPI

  // Setup interrupt
  REG_SPI0_IDR = SPI_IDR_TDRE | SPI_IDR_MODF | SPI_IDR_OVRES |
                 SPI_IDR_NSSR | SPI_IDR_TXEMPTY | SPI_IDR_UNDES;
  REG_SPI0_IER = SPI_IER_RDRF;

  // Setup the SPI registers
  REG_SPI0_CR = SPI_CR_SPIEN;     // Enable SPI
  REG_SPI0_MR = SPI_MR_MODFDIS;   // Slave and no modefault
  REG_SPI0_CSR = SPI_MODE0;       // DLYBCT=0, DLYBS=0, SCBR=0, 8-bit transfer

  // Initialize sensors
  while (!sensor1.begin_I2C(sensor1_addr)) {
    Serial.println("Failed to initialize sensor1!");
    delay(10);
  }

  while (!sensor2.begin_I2C(sensor2_addr)) {
    Serial.println("Failed to initialize sensor2!");
    delay(10);
  }

  sensor1.setAccelDataRate(LSM6DS_RATE_416_HZ);
  sensor2.setAccelDataRate(LSM6DS_RATE_416_HZ);

  // Update sensor data for the first transfer
  updateSensorData();
  memcpy(sendStorage, sensorData, sizeof(sensorData));

  // Preload TDR with the first byte to send
  REG_SPI0_TDR = sendStorage[0];
  pos = 1;  // Start position at 1 since first byte is already loaded
}

void updateSensorData() {
  // Read sensor1 data
  sensor1.readAcceleration(sensorData[0], sensorData[1], sensorData[2]);
  sensor1.readGyroscope(sensorData[3], sensorData[4], sensorData[5]);

  // Read sensor2 data
  sensor2.readAcceleration(sensorData[6], sensorData[7], sensorData[8]);
  sensor2.readGyroscope(sensorData[9], sensorData[10], sensorData[11]);
}

void SPI0_Handler(void) {
  uint32_t status = REG_SPI0_SR;

  // Check if data has been received
  if (status & SPI_SR_RDRF) {
    // Read byte from SPI data register
    uint8_t received_byte = REG_SPI0_RDR & 0xFF;

    // Load next byte to transmit
    if (pos < BUFFER_SIZE) {
      REG_SPI0_TDR = sendStorage[pos++];
    } else {
      // All bytes have been sent
      transferComplete = true;
      pos = 0;  // Reset position for the next transfer
    }
  }
}
