/*
 *
 * Kod pro obnovu cidla SHT4x po pripadne kontaminaci, kdy cidlo nemeri spravne hodnoty
 * Kod byl testovan na deskach ESP32-S3 DevKit a ESP32-C3 LPKit, lze ale pouzit i s jinou deskou
 * V pripade pouziti jine desky je potreba spravne nastavit SDA, SCL piny I2C a pripadne pin pro zapnuti napajeni pro cidlo
 *
 * Merene hodnoty se vypisuji na seriovou konzoli, lze tak sledovat prubeh procedury
 *
 * Made by (c) laskakit.cz 2024
 *
 */

#include <Arduino.h>
#include <SensirionI2cSht4x.h>
#include <Wire.h>

SensirionI2cSht4x sht4x;

uint16_t error;
char errorMessage[256];
float temperature;
float humidity;

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

  sht4x.begin(Wire, 0x44);

  uint32_t serialNumber;

  for (int i = 1; i < 8; i++)
  {
    reset_i2c_switch();
    select_sht(i);

    error = sht4x.serialNumber(serialNumber);
    if (error)
    {
      Serial.print("Error trying to execute serialNumber(): ");
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
    }
    else
    {
      Serial.print("Serial Number: ");
      Serial.println(serialNumber);
      delay(1000);
      error = sht4x.measureHighPrecision(temperature, humidity);
    }

    unsigned long milliseconds = millis();
    unsigned long reference = (3UL * 60UL * 60UL * 1000UL) + milliseconds;

    while (humidity >= 5.0 && millis() < reference)
    {
      if (temperature >= 125.0)
      {
        delay(100);
      }
      error = sht4x.activateHighestHeaterPowerLong(temperature, humidity);

      if (error)
      {
        Serial.print("Error trying to execute measureHighPrecision(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
      }
      else
      {
        Serial.print("SHT4x [" + String(i) + "]: ");
        Serial.print("Temperature: ");
        Serial.print(temperature);
        Serial.print("°C ");
        Serial.print("Humidity: ");
        Serial.print(humidity);
        Serial.println("%RH");
      }
    }

    Serial.println("Sensor [" + String(i) + "] burn-in complete");
  }
}

void loop()
{
  delay(10000);
  Serial.println("Burn-in complete");
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