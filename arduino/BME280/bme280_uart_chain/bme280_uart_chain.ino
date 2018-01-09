#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1013.25)

#define ID_PIN_0 2
#define ID_PIN_1 3
#define ID_PIN_2 4

Adafruit_BME280 bme; // I2C

unsigned long delayTime;
int id;

void setup() {
    // Setup ID pins
    pinMode(ID_PIN_0,INPUT_PULLUP);
    pinMode(ID_PIN_1,INPUT_PULLUP);
    pinMode(ID_PIN_2,INPUT_PULLUP);

    id = getID();
    Serial.begin(9600);
    //Serial.print("BME280 node:");
    //Serial.println(id);

    Wire.begin();
    if (! bme.begin(&Wire)) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
    }

    // indoor navigation
    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X2,  // temperature
                    Adafruit_BME280::SAMPLING_X16, // pressure
                    Adafruit_BME280::SAMPLING_X1,  // humidity
                    Adafruit_BME280::FILTER_X16);
    // Delaytime nneds to be high to avoid self-heating
    delayTime = 10000; // Overridden to 10.000 (10s)
}

void loop() {
    // make forced measurement
    bme.takeForcedMeasurement(); // 
    
	if(checkComs())
	{
		sendValues();
	}
    delay(delayTime);
}
int getID()
{
  int id;
  id = digitalRead(ID_PIN_0);
  id |= digitalRead(ID_PIN_1) << 1;
  id |= digitalRead(ID_PIN_2) << 2;
  id = ~id;
  id &= 0x07;
  return id;
}

bool checkComs()
{
	static bool idle = true;
	static char buffer[128];
	
	while(Serial.available())
	{
    int ret = readline(Serial.read(),buffer, 128);
		if(ret > 0)
		{
			Serial.println(buffer);
			idle=true;
		}
   else if (ret == 0)
   {
    idle = true;
   }
   else
   {
    idle = false;
   }
	}
	return idle;
}

int readline(int readch, char *buffer, int len)
{
		
	static int pos = 0;
	int rpos;

	if (readch > 0) {
		switch (readch) 
		{
			case '\n': // Ignore new-lines
      return 0;
				break;
			case '\r': // Return on CR
				rpos = pos;
				pos = 0;  // Reset position index ready for next time
				return rpos;
			default:
				if (pos < len-1) 
				{
					buffer[pos++] = readch;
					buffer[pos] = 0;
				}
				else
				{
					Serial.println("E:readline;buffer overrun");
				}
		}
	}
	// No end of line has been found, so return -1.
	return -1;
}

void sendValues() {
    Serial.print("i");
    Serial.print(id);
    
    Serial.print("t");
    Serial.print(bme.readTemperature());

    Serial.print("p");
    Serial.print(bme.readPressure() / 100.0F);

    //Serial.print("");
    //Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));

    Serial.print("h");
    Serial.print(bme.readHumidity());

    Serial.println();
}

