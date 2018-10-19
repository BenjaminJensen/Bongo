#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C

unsigned long delayTime;

void setup() {
    Serial.begin(9600);
    Serial.println(F("BME280 node"));

    Wire.begin();
    if (! bme.begin(&Wire)) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
    }


    delayTime = 5000;
    

    // indoor navigation
    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X2,  // temperature
                    Adafruit_BME280::SAMPLING_X16, // pressure
                    Adafruit_BME280::SAMPLING_X1,  // humidity
                    Adafruit_BME280::FILTER_X16);
    
    // suggested rate is 25Hz
    // 1 + (2 * T_ovs) + (2 * P_ovs + 0.5) + (2 * H_ovs + 0.5)
    // T_ovs = 2
    // P_ovs = 16
    // H_ovs = 1
    // = 40ms (25Hz)
    // with standby time that should really be 24.16913... Hz
    delayTime = 10000; // Overridden to 10.000 (10s)
}

void loop() {
    // Only needed in forced mode! In normal mode, you can remove the next line.
    bme.takeForcedMeasurement(); // has no effect in normal mode
    
    printValues();
    delay(delayTime);
}

void printValues() {
    Serial.print("T = ");
    Serial.print(bme.readTemperature());
    Serial.print(" C;");

    Serial.print("P = ");

    Serial.print(bme.readPressure() / 100.0F);
    Serial.print(" hPa;");

    Serial.print("A = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.print(" m;");

    Serial.print("H = ");
    Serial.print(bme.readHumidity());
    Serial.print(" %");

    Serial.println();
}
