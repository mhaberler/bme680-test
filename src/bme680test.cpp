/***************************************************************************
  This is a library for the BME680 gas, humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BME680 Breakout
  ----> http://www.adafruit.com/products/3660

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <Adafruit_TMP117.h>
#include <Adafruit_LPS2X.h>

#include "I2CScanner.h"

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME680 bme; // I2C

#define LPS22_ADDR 0x5C
Adafruit_LPS22 lps;
Adafruit_TMP117 tmp117;

I2CScanner scanner;

bool have_tmp117, have_lps22, have_bme680;

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        delay(10);
    ;

    scanner.Init();
    scanner.Scan();
    Serial.println(F("sensor test"));

    have_tmp117 = tmp117.begin();
    have_bme680 = bme.begin();
    have_lps22 = lps.begin_I2C(LPS22_ADDR);

    Serial.printf("have_tmp117=%d  have_lps22=%d  have_bme680=%d\n", have_tmp117, have_lps22, have_bme680);
    // Try to initialize!
    if (have_lps22)

    {
        Serial.println("LPS22 Found!");
        Serial.print("Data rate set to: ");
        switch (lps.getDataRate())
        {
        case LPS25_RATE_ONE_SHOT:
            Serial.println("One Shot");
            break;
        case LPS25_RATE_1_HZ:
            Serial.println("1 Hz");
            break;
        case LPS25_RATE_7_HZ:
            Serial.println("7 Hz");
            break;
        case LPS25_RATE_12_5_HZ:
            Serial.println("12.5 Hz");
            break;
        case LPS25_RATE_25_HZ:
            Serial.println("25 Hz");
            break;
        }
    }
    else
    {

        Serial.println("Failed to find LPS22 chip");
    }
    
    if (have_bme680)
    {

        // Set up oversampling and filter initialization
        bme.setTemperatureOversampling(BME680_OS_8X);
        bme.setHumidityOversampling(BME680_OS_2X);
        bme.setPressureOversampling(BME680_OS_4X);
        bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
        bme.setGasHeater(320, 150); // 320*C for 150 ms;
    }
}

void loop()
{
    sensors_event_t temp; // create an empty event to be filled

    if (have_bme680)
    {

        if (!bme.performReading())
        {
            Serial.println("Failed to perform reading :(");
            return;
        }
        Serial.print("bme680 Temperature = ");
        Serial.print(bme.temperature);
        Serial.println(" *C");

        Serial.print("bme680 Pressure = ");
        Serial.print(bme.pressure / 100.0);
        Serial.println(" hPa");

        Serial.print("bme680 Humidity = ");
        Serial.print(bme.humidity);
        Serial.println(" %");

        Serial.print("bme680 Gas = ");
        Serial.print(bme.gas_resistance / 1000.0);
        Serial.println(" KOhms");

        Serial.print("bme680 Approx. Altitude = ");
        Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
        Serial.println(" m");

        Serial.println();
    }

    if (have_tmp117)
    {
        tmp117.getEvent(&temp); //fill the empty event object with the current measurements
        Serial.print("TMP117 Temperature  ");
        Serial.print(temp.temperature);
        Serial.println(" degrees C");
        Serial.println("");
    }
    if (have_lps22)
    {
        sensors_event_t pressure;
        lps.getEvent(&pressure, &temp); // get pressure
        Serial.print("LPS22 Temperature: ");
        Serial.print(temp.temperature);
        Serial.println(" degrees C");
        Serial.print("LPS22 Pressure: ");
        Serial.print(pressure.pressure);
        Serial.println(" hPa");
        Serial.println("");
    }
    delay(2000);
}
