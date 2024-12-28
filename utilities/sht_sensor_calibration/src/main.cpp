/*-------DHT22 sensor-------*/
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN 7      // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22 // DHT 22 (AM2302)

DHT_Unified dht(DHTPIN, DHTTYPE);

uint32_t delayMS;

void setupDHT22();
void printDHT22Data(float *temp, float *hum);
/*----End DHT22 sensor-----*/

/*-------BME280 sensor-------*/
#include <BME280I2C.h>
#include <Wire.h>

#define SERIAL_BAUD 115200

void setupBME280();
void printBME280Data(Stream *client, float *temp, float *hum);

BME280I2C bme; // Default : forced mode, standby time = 1000 ms
               // Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,
/*----End BME280 sensor-----*/

/*-------SHT40 sensor-------*/
#include <Arduino.h>
#include <SensirionI2cSht4x.h>
#include <Wire.h>

#ifdef NO_ERROR
#undef NO_ERROR
#endif
#define NO_ERROR 0

SensirionI2cSht4x sht4x;

static char errorMessage[64];
static int16_t error;

void setupSHT40(int index);
void printSHT40Data(int index, float *temp, float *hum);
/*----End SHT40 sensor-----*/

/*-------AHT10 sensor-------*/
#include <Adafruit_AHTX0.h>
Adafruit_AHTX0 aht;

void setupAHT10();
void printAHT10Data(float *temp, float *hum);
/*----End AHT10 sensor-----*/

/*-------SHT30 sensor-------*/
#include <Arduino.h>
#include <SensirionI2cSht3x.h>
#include <Wire.h>

SensirionI2cSht3x sht3x;

void setupSHT30();
void printSHT30Data(float *temp, float *hum);
/*----End SHT30 sensor-----*/

/*-------LCD 20x04-------*/

#include <Wire.h>
#include <LiquidCrystal.h>

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

void setup_lcd();

/*----End LCD 20x04-----*/

/*-------I2C Switch-------*/

#define I2C_SWITCH_ADDRESS_0 0x70
#define I2C_SWITCH_ADDRESS_1 0x71
#define I2C_SWITCH_RSTN_PIN 6

void reset_i2c_switch();
void select_sht(uint8_t index);

/*----End I2C Switch-----*/

void setup()
{
  Serial.begin(115200);
  Wire.begin();

  for (int i = 0; i < 8; i++)
  {
    select_sht(i);
    setupSHT40(i);
    delay(1);
  }

  select_sht(8);
  setupSHT30();
  reset_i2c_switch(); // De-select

  setupDHT22();
  setupBME280();

  setupAHT10();

  setup_lcd();
}

void loop()
{
  unsigned long ms = millis();

  float temps[12];
  float hums[12];

  for (int i = 0; i < 8; i++)
  {
    select_sht(i);
    delay(1);
    printSHT40Data(i, &temps[i], &hums[i]);
  }

  select_sht(8);
  printSHT30Data(&temps[8], &hums[8]);
  reset_i2c_switch(); // De-select

  printDHT22Data(&temps[9], &hums[9]);
  printBME280Data(&Serial, &temps[10], &hums[10]);
  printAHT10Data(&temps[11], &hums[11]);
  Serial.println("------------------------------------");

  lcd.clear();
  for (int i = 0; i < 12; i++)
  {
    lcd.print(temps[i]);
    lcd.print(" ");
    if (i > 0 && (i + 1) % 3 == 0)
    {
      lcd.setCursor(0, (i + 1) / 3);
    }
  }

  delay(2000);

  lcd.clear();
  for (int i = 0; i < 12; i++)
  {
    lcd.print(hums[i]);
    lcd.print(" ");
    if (i > 0 && (i + 1) % 3 == 0)
    {
      lcd.setCursor(0, (i + 1) / 3);
    }
  }

  while (millis() - ms < 5000)
  {
  }
}

void setupDHT22()
{
  // Initialize device.
  dht.begin();
  Serial.println(F("DHTxx Unified Sensor Example"));
  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.println(F("Temperature Sensor"));
  Serial.print(F("Sensor Type: "));
  Serial.println(sensor.name);
  Serial.print(F("Driver Ver:  "));
  Serial.println(sensor.version);
  Serial.print(F("Unique ID:   "));
  Serial.println(sensor.sensor_id);
  Serial.print(F("Max Value:   "));
  Serial.print(sensor.max_value);
  Serial.println(F("°C"));
  Serial.print(F("Min Value:   "));
  Serial.print(sensor.min_value);
  Serial.println(F("°C"));
  Serial.print(F("Resolution:  "));
  Serial.print(sensor.resolution);
  Serial.println(F("°C"));
  Serial.println(F("------------------------------------"));
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println(F("Humidity Sensor"));
  Serial.print(F("Sensor Type: "));
  Serial.println(sensor.name);
  Serial.print(F("Driver Ver:  "));
  Serial.println(sensor.version);
  Serial.print(F("Unique ID:   "));
  Serial.println(sensor.sensor_id);
  Serial.print(F("Max Value:   "));
  Serial.print(sensor.max_value);
  Serial.println(F("%"));
  Serial.print(F("Min Value:   "));
  Serial.print(sensor.min_value);
  Serial.println(F("%"));
  Serial.print(F("Resolution:  "));
  Serial.print(sensor.resolution);
  Serial.println(F("%"));
  Serial.println(F("------------------------------------"));
}

void printDHT22Data(float *temp, float *hum)
{
  // Get temperature event and print its value.
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature))
  {
    Serial.println(F("Error reading temperature!"));
  }
  else
  {
    Serial.print(F("DHT22:\t\tTemperature: "));
    Serial.print(event.temperature);
    Serial.print(F(" °C"));
  }
  *temp = event.temperature;

  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity))
  {
    Serial.println(F("Error reading humidity!"));
  }
  else
  {
    Serial.print(F("\tHumidity: "));
    Serial.print(event.relative_humidity);
    Serial.println(F(" %RH"));
  }

  *hum = event.relative_humidity;
}

void setupBME280()
{
  while (!Serial)
  {
  } // Wait

  while (!bme.begin())
  {
    Serial.println("Could not find BME280 sensor!");
    delay(1000);
  }

  switch (bme.chipModel())
  {
  case BME280::ChipModel_BME280:
    Serial.println("Found BME280 sensor! Success.");
    break;
  case BME280::ChipModel_BMP280:
    Serial.println("Found BMP280 sensor! No Humidity available.");
    break;
  default:
    Serial.println("Found UNKNOWN sensor! Error!");
  }
}

void printBME280Data(
    Stream *client, float *temp, float *hum)
{
  float temperature(NAN), humidity(NAN), pressure(NAN);

  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_Pa);

  bme.read(pressure, temperature, humidity, tempUnit, presUnit);

  client->print("BME280:\t\tTemperature: ");
  client->print(temperature);
  client->print(" °C");
  client->print("\tHumidity: ");
  client->print(humidity);
  client->print(" %RH");
  client->print("\tPressure: ");
  client->print(pressure);
  client->println(" Pa");

  *temp = temperature;
  *hum = humidity;
}

void setupSHT40(int index)
{
  sht4x.begin(Wire, SHT40_I2C_ADDR_44);

  sht4x.softReset();
  delay(10);
  uint32_t serialNumber = 0;
  error = sht4x.serialNumber(serialNumber);
  if (error != NO_ERROR)
  {
    Serial.print("SHT40 [");
    Serial.print(index);
    Serial.println("]: Error trying to execute serialNumber()");
    return;
  }
  Serial.print("SHT40 [");
  Serial.print(index);
  Serial.print("]: serialNumber: ");
  Serial.print(serialNumber);
  Serial.println();
}

void printSHT40Data(int index, float *temp, float *hum)
{
  float aTemperature = 0.0;
  float aHumidity = 0.0;
  error = sht4x.measureHighPrecision(aTemperature, aHumidity);
  if (error != NO_ERROR)
  {
    Serial.print("SHT40 [");
    Serial.print(index);
    Serial.println("]: Error trying to execute measureLowestPrecision()");
    return;
  }
  Serial.print("SHT40 [");
  Serial.print(index);
  Serial.print("]:\tTemperature: ");
  Serial.print(aTemperature);
  Serial.print(" °C\tHumidity: ");
  Serial.print(aHumidity);
  Serial.print(" %RH");
  Serial.println();

  *temp = aTemperature;
  *hum = aHumidity;
}

void setupAHT10()
{
  if (!aht.begin(&Wire))
  {
    Serial.println("Could not find AHT? Check wiring");
    return;
  }
  Serial.println("AHT10 or AHT20 found");
}

void printAHT10Data(float *temp, float *hum)
{
  sensors_event_t humidity, temperature;
  aht.getEvent(&humidity, &temperature); // populate temp and humidity objects with fresh data
  Serial.print("AHT10:\t\tTemperature: ");
  Serial.print(temperature.temperature);
  Serial.print(" °C\tHumidity: ");
  Serial.print(humidity.relative_humidity);
  Serial.println(" %RH");

  *temp = temperature.temperature;
  *hum = humidity.relative_humidity;
}

void setupSHT30()
{
  sht3x.begin(Wire, SHT30_I2C_ADDR_44);

  sht3x.stopMeasurement();
  delay(1);
  sht3x.softReset();
  delay(100);
  uint16_t aStatusRegister = 0u;
  error = sht3x.readStatusRegister(aStatusRegister);
  if (error != NO_ERROR)
  {
    Serial.print("Error trying to execute readStatusRegister()");
    return;
  }
  Serial.print("aStatusRegister: ");
  Serial.print(aStatusRegister);
  Serial.println();
  error = sht3x.startPeriodicMeasurement(REPEATABILITY_MEDIUM,
                                         MPS_ONE_PER_SECOND);
  if (error != NO_ERROR)
  {
    Serial.print("Error trying to execute startPeriodicMeasurement()");
    return;
  }
}

void printSHT30Data(float *temp, float *hum)
{
  float aTemperature = 0.0;
  float aHumidity = 0.0;
  error = sht3x.blockingReadMeasurement(aTemperature, aHumidity);
  if (error != NO_ERROR)
  {
    Serial.print("Error trying to execute blockingReadMeasurement()");
    return;
  }
  Serial.print("SHT30:\t\tTemperature: ");
  Serial.print(aTemperature);
  Serial.print(" °C\tHumidity: ");
  Serial.print(aHumidity);
  Serial.print(" %RH");
  Serial.println();

  *temp = aTemperature;
  *hum = aHumidity;
}

void setup_lcd()
{
  lcd.begin(20, 4);
  lcd.clear();
}

void reset_i2c_switch(void)
{
  digitalWrite(I2C_SWITCH_RSTN_PIN, LOW);
  delay(1);
  digitalWrite(I2C_SWITCH_RSTN_PIN, HIGH);
}

void select_sht(uint8_t index)
{
  reset_i2c_switch();

  Wire.beginTransmission(I2C_SWITCH_ADDRESS_0);
  Wire.write(0);
  Wire.endTransmission();
  Wire.beginTransmission(I2C_SWITCH_ADDRESS_1);
  Wire.write(0);
  Wire.endTransmission();

  if (index <= 7)
  {
    Wire.beginTransmission(I2C_SWITCH_ADDRESS_0);
    Wire.write((1 << index));
    Wire.endTransmission();
  }
  else
  {
    Wire.beginTransmission(I2C_SWITCH_ADDRESS_1);
    Wire.write((1 << (index - 8)));
    Wire.endTransmission();
  }
}