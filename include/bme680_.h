#include <Adafruit_BME680.h>
#define SEALEVELPRESSURE_HPA (1013.25)

namespace bme680 {
Adafruit_BME680 bme;

void init() {
    while (!Serial)
        ;
    Serial.println(F("BME680 async test"));

    if (!bme.begin()) {
        Serial.println(F("Could not find a valid BME680 sensor, check wiring!"));
        while (1)
            ;
    }

    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150);  // 320*C for 150 ms
}
void printData() {
    // Tell BME680 to begin measurement.
    unsigned long endTime = bme.beginReading();
    if (endTime == 0) {
        Serial.println(F("Failed to begin reading :("));
        return;
    }
    Serial.print(F("Reading started at "));
    Serial.print(millis());
    Serial.print(F(" and will finish at "));
    Serial.println(endTime);

    Serial.println(F("You can do other work during BME680 measurement."));

    if (!bme.endReading()) {
        Serial.println(F("Failed to complete reading :("));
        return;
    }
    Serial.print(F("Reading completed at "));
    Serial.println(millis());

    Serial.print(F("Temperature = "));
    Serial.print(bme.temperature);
    Serial.println(F(" *C"));

    Serial.print(F("Pressure = "));
    Serial.print(bme.pressure / 100.0);
    Serial.println(F(" hPa"));

    Serial.print(F("Humidity = "));
    Serial.print(bme.humidity);
    Serial.println(F(" %"));

    Serial.print(F("Gas = "));
    Serial.print(bme.gas_resistance / 1000.0);
    Serial.println(F(" KOhms"));

    Serial.print(F("Approx. Altitude = "));
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(F(" m"));

    Serial.println();
    delay(2000);
}

String getData() {
     bme.beginReading();
     delay(50);
     bme.endReading();
    return "{\"temp680\":"+String(bme.temperature)+",\"hum680\":"+String(bme.humidity)+",\"pres680\":"+String(bme.pressure / 100.0)+",\"gas680\":"+String(bme.gas_resistance / 1000.0)+"}";

}

}  // namespace bme680