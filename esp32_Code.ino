#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <SPI.h>
#include <SD.h>

#include <atomic>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <unistd.h>

#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

// ESP32 DevKit V1. Both sensors share the I2C bus.
constexpr uint8_t sensorSdaPin = 21;
constexpr uint8_t sensorSclPin = 22;
constexpr uint8_t sdClockPin = 18;
constexpr uint8_t sdMisoPin = 19;
constexpr uint8_t sdMosiPin = 23;
constexpr uint8_t sdSelectPin = 5;
constexpr uint8_t stopButtonPin = 0;  // Onboard BOOT button

constexpr uint8_t outputPins[] = {13, 12, 14, 27};
constexpr size_t pinCount = sizeof(outputPins) / sizeof(outputPins[0]);

constexpr int64_t accelIntervalUs = 10000;  // Target 100 BMI160 reads/s
constexpr int64_t csvIntervalUs = 20000;    // Target 50 CSV rows/s
constexpr int64_t syncIntervalUs = 1000000;
constexpr unsigned csvQueueDepth = 256;
constexpr char sdMountPoint[] = "/sd";
constexpr float seaLevelPressurePa = 101325.0f;
constexpr float metersPerSecondSquaredPerG = 9.80665f;

constexpr uint32_t slowStepUs = 1000000;
constexpr float slowRateHz = 1.0f;
constexpr float fastRateHz = 100.0f;
constexpr float accelerationLevelsG[] = {
    1.10f, 1.445f, 1.79f, 2.135f, 2.48f,
    2.825f, 3.17f, 3.515f, 3.86f, 4.205f,
    4.55f, 4.895f, 5.24f, 5.585f, 5.93f,
    6.275f, 6.62f, 7.31f, 8.00f
};
constexpr size_t levelCount =
    sizeof(accelerationLevelsG) / sizeof(accelerationLevelsG[0]);
constexpr float countsPerG = 2048.0f;  // BMI160 at +/-16 g

struct Acceleration {
  float xMs2;
  float yMs2;
  float zMs2;
  float magnitudeG;
};

struct BarometerReading {
  float pressurePa;
  float temperatureC;
  float altitudeM;
};

struct FlightSample {
  uint32_t elapsedMs;
  BarometerReading barometer;
  Acceleration acceleration;
  uint32_t lightStepUs;
};

enum class LogState : uint32_t { Inactive, Recording, Stopping, Closed };

constexpr Acceleration noAcceleration = {NAN, NAN, NAN, NAN};
constexpr BarometerReading noBarometerReading = {NAN, NAN, NAN};

Adafruit_BMP280 bmp;
uint8_t bmpAddress = 0x76;
uint8_t bmiAddress = 0x69;
bool barometerReady = false;
bool accelerometerReady = false;
Acceleration latestAcceleration = noAcceleration;

size_t activePin = 0;
uint32_t stepIntervalUs = slowStepUs;
int64_t lastStepUs = 0;
int64_t nextAccelUs = 0;
int64_t nextCsvUs = 0;
int64_t logStartedUs = 0;

QueueHandle_t csvQueue = nullptr;
std::atomic<LogState> logState{LogState::Inactive};
bool queueOverflowReported = false;

bool readBmiRegisters(uint8_t reg, uint8_t *data, size_t count) {
  Wire.beginTransmission(bmiAddress);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0 ||
      Wire.requestFrom(bmiAddress, count) != count) {
    return false;
  }

  for (size_t i = 0; i < count; i++) {
    data[i] = static_cast<uint8_t>(Wire.read());
  }
  return true;
}

bool writeBmiRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(bmiAddress);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool startAccelerometer() {
  uint8_t chipId = 0;
  if (!readBmiRegisters(0x00, &chipId, 1) || chipId != 0xD1) {
    bmiAddress = 0x69;
    if (!readBmiRegisters(0x00, &chipId, 1) || chipId != 0xD1) {
      return false;
    }
  }

  if (!writeBmiRegister(0x7E, 0x11)) {  // Accelerometer normal mode
    return false;
  }
  delay(50);  // Let the accelerometer start before setting its range.
  return writeBmiRegister(0x40, 0x28) &&  // 100 Hz output data rate
         writeBmiRegister(0x41, 0x0C);    // +/-16 g range
}

uint32_t intervalFromAcceleration(float accelerationG) {
  size_t level = 0;
  // Use the highest acceleration threshold reached.
  while (level + 1 < levelCount &&
         accelerationG >= accelerationLevelsG[level + 1]) {
    ++level;
  }

  const float fraction = (accelerationLevelsG[level] - accelerationLevelsG[0]) /
                         (accelerationLevelsG[levelCount - 1] -
                          accelerationLevelsG[0]);
  const float rateHz = slowRateHz + fraction * (fastRateHz - slowRateHz);
  return static_cast<uint32_t>(lroundf(1000000.0f / rateHz));
}

Acceleration readAcceleration() {
  uint8_t data[6];
  if (!readBmiRegisters(0x12, data, sizeof(data))) {
    return noAcceleration;
  }

  float axesG[3];
  for (size_t axis = 0; axis < 3; ++axis) {
    const uint16_t bits = (uint16_t(data[2 * axis + 1]) << 8) |
                          data[2 * axis];
    axesG[axis] = static_cast<int16_t>(bits) / countsPerG;
  }

  const float magnitudeG = sqrtf(axesG[0] * axesG[0] +
                                 axesG[1] * axesG[1] +
                                 axesG[2] * axesG[2]);
  return {axesG[0] * metersPerSecondSquaredPerG,
          axesG[1] * metersPerSecondSquaredPerG,
          axesG[2] * metersPerSecondSquaredPerG,
          magnitudeG};
}

bool startBarometer() {
  if (!bmp.begin(bmpAddress)) {
    bmpAddress = 0x77;
    if (!bmp.begin(bmpAddress)) {
      return false;
    }
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X1,
                  Adafruit_BMP280::SAMPLING_X4,
                  Adafruit_BMP280::FILTER_OFF,
                  Adafruit_BMP280::STANDBY_MS_1);
  return true;
}

BarometerReading readBarometer() {
  if (!barometerReady) {
    return noBarometerReading;
  }

  Wire.beginTransmission(bmpAddress);
  if (Wire.endTransmission() != 0) {
    return noBarometerReading;
  }

  const float pressurePa = bmp.readPressure();
  const float temperatureC = bmp.readTemperature();
  if (!isfinite(pressurePa) || pressurePa < 30000.0f ||
      pressurePa > 110000.0f || !isfinite(temperatureC) ||
      temperatureC < -40.0f || temperatureC > 85.0f) {
    return noBarometerReading;
  }

  // Calculate altitude from the same pressure value saved to this row.
  const float altitudeM =
      44330.0f * (1.0f - powf(pressurePa / seaLevelPressurePa, 0.1903f));
  return {pressurePa, temperatureC, altitudeM};
}

bool sampleDue(int64_t nowUs, int64_t &nextUs, int64_t intervalUs) {
  if (nowUs < nextUs) {
    return false;
  }

  // Skip expired sample slots so each row corresponds to an actual read.
  nextUs += ((nowUs - nextUs) / intervalUs + 1) * intervalUs;
  return true;
}

void updateLights(int64_t nowUs) {
  if (nowUs - lastStepUs < stepIntervalUs) {
    return;
  }

  digitalWrite(outputPins[activePin], LOW);
  activePin = (activePin + 1) % pinCount;
  digitalWrite(outputPins[activePin], HIGH);
  lastStepUs = nowUs;
}

int createFlightFile(char *path, size_t pathSize) {
  for (unsigned number = 1; number <= 9999; ++number) {
    snprintf(path, pathSize, "%s/FLT%04u.CSV", sdMountPoint, number);
    // Exclusive creation protects previous recordings, including after reset.
    const int fd = open(path, O_WRONLY | O_CREAT | O_EXCL, 0666);
    if (fd >= 0 || errno != EEXIST) {
      return fd;
    }
  }
  return -1;
}

bool syncCsv(FILE *csv) {
  return fflush(csv) == 0 && fsync(fileno(csv)) == 0;
}

bool writeCsvRow(FILE *csv, const FlightSample &sample) {
  char barometerFields[64] = ",,";
  const BarometerReading &barometer = sample.barometer;
  if (isfinite(barometer.pressurePa)) {
    snprintf(barometerFields, sizeof(barometerFields), "%.2f,%.2f,%.2f",
             barometer.pressurePa, barometer.temperatureC, barometer.altitudeM);
  }

  char accelerationFields[64] = ",,,";
  const Acceleration &acceleration = sample.acceleration;
  if (isfinite(acceleration.magnitudeG)) {
    snprintf(accelerationFields, sizeof(accelerationFields),
             "%.3f,%.3f,%.3f,%.3f", acceleration.xMs2, acceleration.yMs2,
             acceleration.zMs2, acceleration.magnitudeG);
  }

  return fprintf(csv, "%lu,%s,%s,%.3f\n",
                 static_cast<unsigned long>(sample.elapsedMs),
                 barometerFields, accelerationFields,
                 sample.lightStepUs / 1000.0) > 0;
}

void csvWriterTask(void *argument) {
  FILE *csv = static_cast<FILE *>(argument);
  int64_t lastSyncUs = esp_timer_get_time();
  bool writeOk = true;

  while (writeOk) {
    // Once stopped, drain every queued row before closing the file.
    const bool stopping = logState.load() == LogState::Stopping;
    FlightSample sample;
    if (xQueueReceive(csvQueue, &sample, stopping ? 0 : pdMS_TO_TICKS(20)) ==
        pdTRUE) {
      writeOk = writeCsvRow(csv, sample);
    } else if (stopping) {
      break;
    }

    const int64_t nowUs = esp_timer_get_time();
    if (writeOk && nowUs - lastSyncUs >= syncIntervalUs) {
      writeOk = syncCsv(csv);
      lastSyncUs = nowUs;
    }
  }

  if (!writeOk) {
    logState.store(LogState::Inactive);
  }
  writeOk = writeOk && syncCsv(csv);
  if (fclose(csv) != 0) {
    writeOk = false;
  }

  logState.store(writeOk ? LogState::Closed : LogState::Inactive);
  Serial.println(writeOk ? "CSV closed." :
                           "SD write or sync failed. Some data may be missing.");
  vTaskDelete(nullptr);
}

void startRecording() {
  SPI.begin(sdClockPin, sdMisoPin, sdMosiPin, sdSelectPin);
  if (!SD.begin(sdSelectPin, SPI, 4000000, sdMountPoint)) {
    Serial.println("SD card mount failed. CSV recording did not start.");
    return;
  }

  char path[24];
  const int fd = createFlightFile(path, sizeof(path));
  if (fd < 0) {
    Serial.printf("Could not create a flight CSV (error %d).\n", errno);
    return;
  }

  FILE *csv = fdopen(fd, "w");
  if (csv == nullptr) {
    close(fd);
    Serial.println("Could not open the new flight CSV.");
    return;
  }

  constexpr char header[] =
      "elapsed_ms,pressure_Pa,temperature_C,estimated_altitude_m,"
      "ax_m_s2,ay_m_s2,az_m_s2,accel_g,light_step_ms\n";
  if (fputs(header, csv) == EOF || !syncCsv(csv)) {
    fclose(csv);
    Serial.println("Could not save the CSV header.");
    return;
  }

  csvQueue = xQueueCreate(csvQueueDepth, sizeof(FlightSample));
  if (csvQueue == nullptr) {
    fclose(csv);
    Serial.println("Could not allocate the CSV buffer.");
    return;
  }

  logState.store(LogState::Recording);
  // Run SD I/O on the other ESP32 core to keep it out of the sampling loop.
  const BaseType_t writerCore = xPortGetCoreID() == 0 ? 1 : 0;
  if (xTaskCreatePinnedToCore(csvWriterTask, "csvWriter", 8192, csv, 1,
                              nullptr, writerCore) != pdPASS) {
    logState.store(LogState::Inactive);
    vQueueDelete(csvQueue);
    csvQueue = nullptr;
    fclose(csv);
    Serial.println("Could not start the SD writer.");
    return;
  }

  Serial.printf("Recording to %s. Tap BOOT or send s to stop.\n",
                path + sizeof(sdMountPoint));
}

void setup() {
  Serial.begin(115200);
  pinMode(stopButtonPin, INPUT_PULLUP);

  for (const uint8_t pin : outputPins) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }

  if (Wire.begin(sensorSdaPin, sensorSclPin, 100000)) {
    Wire.setTimeOut(5);
    barometerReady = startBarometer();
    accelerometerReady = startAccelerometer();
  }

  if (barometerReady) {
    Serial.printf("BMP280 found at 0x%02X\n", static_cast<unsigned>(bmpAddress));
  } else {
    Serial.println("BMP280 unavailable. Pressure, temperature and altitude will be blank.");
  }

  if (accelerometerReady) {
    Serial.printf("BMI160 found at 0x%02X\n", static_cast<unsigned>(bmiAddress));
  } else {
    Serial.println("BMI160 unavailable. Acceleration will be blank and lights will run at 1 s per pin.");
  }

  if (barometerReady || accelerometerReady) {
    startRecording();
  } else {
    Serial.println("CSV recording did not start because neither sensor responded.");
  }

  digitalWrite(outputPins[activePin], HIGH);
  logStartedUs = esp_timer_get_time();
  lastStepUs = logStartedUs;
  nextAccelUs = logStartedUs;
  nextCsvUs = logStartedUs;
}

void loop() {
  LogState state = logState.load();
  if (state == LogState::Closed) {
    digitalWrite(outputPins[activePin], LOW);
    delay(20);
    return;
  }

  const int64_t nowUs = esp_timer_get_time();
  if (accelerometerReady && sampleDue(nowUs, nextAccelUs, accelIntervalUs)) {
    latestAcceleration = readAcceleration();
    stepIntervalUs = isfinite(latestAcceleration.magnitudeG)
                         ? intervalFromAcceleration(latestAcceleration.magnitudeG)
                         : slowStepUs;
  }
  updateLights(esp_timer_get_time());

  if (state == LogState::Recording) {
    if (digitalRead(stopButtonPin) == LOW || Serial.read() == 's') {
      if (logState.compare_exchange_strong(state, LogState::Stopping)) {
        Serial.println("Stopping recording. Saving queued samples...");
      }
    } else if (sampleDue(nowUs, nextCsvUs, csvIntervalUs)) {
      const FlightSample sample = {
          static_cast<uint32_t>((nowUs - logStartedUs) / 1000),
          readBarometer(), latestAcceleration, stepIntervalUs};
      if (xQueueSend(csvQueue, &sample, 0) != pdPASS && !queueOverflowReported) {
        queueOverflowReported = true;
        Serial.println("SD buffer full. Some CSV rows were skipped.");
      }
    }
  }
  delay(1);
}
