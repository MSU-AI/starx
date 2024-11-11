#include <Wire.h>
extern TwoWire Wire1; // Declare Wire1 for secondary I²C bus
#include <SPI.h>
#include <Adafruit_LSM6DSOX.h>

#define CS_PIN 10 // Chip select pin for SPI

#define BUFFER_SIZE 48 // 12 floats * 4 bytes per float

// Sensor instances and addresses
Adafruit_LSM6DSOX sensor1;
uint8_t sensor1_addr = 0x6A; // Sensor 1 I2C address
Adafruit_LSM6DSOX sensor2;
uint8_t sensor2_addr = 0x6B; // Sensor 2 I2C address

// Arrays to hold sensor data
float masterData[12]; // Master's sensor data
float slaveData[12];  // Slave's sensor data

uint8_t slaveStorage[BUFFER_SIZE]; // Buffer to hold incoming data from slave

// Buffer to hold combined data for I2C transmission
#define TOTAL_DATA_SIZE 96        // 24 floats * 4 bytes per float
uint8_t i2cData[TOTAL_DATA_SIZE]; // Holds both masterData and slaveData

// Variables for I2C chunked transmission
volatile uint8_t requestedChunk = 0;                                   // Chunk index requested by the master
#define CHUNK_SIZE 32                                                  // Number of bytes per I2C chunk
#define TOTAL_CHUNKS ((TOTAL_DATA_SIZE + CHUNK_SIZE - 1) / CHUNK_SIZE) // Total number of chunks

void setup()
{
  Serial.begin(115200); // Initialize Serial for debugging

  // Initialize SPI communication
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH); // Deselect the slave
  SPI.begin();
  SPI.beginTransaction(SPISettings(21000000, MSBFIRST, SPI_MODE0));

  // Initialize sensors on default I2C bus (Wire)
  Wire.begin(); // Initialize default I2C bus as master
  while (!sensor1.begin_I2C(sensor1_addr, &Wire))
  {
    Serial.println("Failed to initialize sensor1!");
    delay(10);
  }

  while (!sensor2.begin_I2C(sensor2_addr, &Wire))
  {
    Serial.println("Failed to initialize sensor2!");
    delay(10);
  }

  sensor1.setAccelDataRate(LSM6DS_RATE_416_HZ);
  sensor2.setAccelDataRate(LSM6DS_RATE_416_HZ);

  // Initialize I2C as a slave on the secondary I2C bus (Wire1)
  Wire1.begin(0x08);             // Join the I2C bus with address #8
  Wire1.onReceive(receiveEvent); // Register the receive event handler
  Wire1.onRequest(requestEvent); // Register the request event handler
}

void loop()
{
  // Read master's sensor data
  updateMasterSensorData();

  // Transfer data byte by byte (sending dummy data)
  digitalWrite(CS_PIN, LOW); // Select the slave
  for (size_t i = 0; i < BUFFER_SIZE; i++)
  {
    slaveStorage[i] = SPI.transfer(0x00); // Send dummy byte and receive data
  }
  digitalWrite(CS_PIN, HIGH); // Deselect the slave

  // Reconstruct floats received from slave
  memcpy(slaveData, slaveStorage, sizeof(slaveData));

  // Prepare data for I2C transmission

  Serial.println(slaveData[0]);
  prepareI2CData();
}

// Function to read master's sensor data
void updateMasterSensorData()
{

  // Read sensor1 data
  sensor1.readAcceleration(masterData[0], masterData[1], masterData[2]);
  sensor1.readGyroscope(masterData[3], masterData[4], masterData[5]);

  // Read sensor2 data
  sensor2.readAcceleration(masterData[6], masterData[7], masterData[8]);
  sensor2.readGyroscope(masterData[9], masterData[10], masterData[11]);
}

// Function to prepare data for I2C transmission
void prepareI2CData()
{
  // Combine masterData and slaveData into i2cData buffer
  memcpy(i2cData, masterData, sizeof(masterData));                    // Copy masterData
  memcpy(i2cData + sizeof(masterData), slaveData, sizeof(slaveData)); // Copy slaveData
}

// I2C receive event handler
void receiveEvent(int numBytes)
{
  if (numBytes >= 1)
  {
    requestedChunk = Wire1.read(); // Read the requested chunk index
    // Read additional bytes if needed (not in this case)
  }
}

// I2C request event handler
void requestEvent()
{
  // Send data in chunks of CHUNK_SIZE bytes based on requestedChunk
  uint16_t offset = requestedChunk * CHUNK_SIZE;
  uint8_t bytesToSend = CHUNK_SIZE;

  if (offset + CHUNK_SIZE > TOTAL_DATA_SIZE)
  {
    bytesToSend = TOTAL_DATA_SIZE - offset;
  }

  Wire1.write(i2cData + offset, bytesToSend);
}
