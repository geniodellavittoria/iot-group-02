#include <Adafruit_TSL2561_U.h>

#include <Adafruit_BME280.h>

#include <Adafruit_Sensor.h>


#define BME_SCK D4
#define BME_MISO D3
#define BME_MOSI D2
#define BME_CS D5
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

const int MOTION_PIN_D0 = 0;
const int MOTION_PIN_D2 = 2; // Pin connected to motion detector
int roomFrequency = 0;

int proximity_d0 = false;
int proximity_d2 = false;
int doorDetectionThreshold = 1000;

int startTime = 0;
int endTime = 0;
int currentTime = 0;

int leftMotionSensorTriggered = false;
int rightMotionSensorTriggered = false;

int luminosityInterval = 60000;
int luminosityStartTime = 0;
int luminosityCurrentTime = 0;

int temperatureInterval = 60000;
int temperatureStartTime = 0;
int temperatureCurrentTime = 0;


void setup() {
    pinMode(MOTION_PIN_D0, INPUT_PULLUP);
    pinMode(MOTION_PIN_D2, INPUT_PULLUP);

    Serial.println("Reading BME280 TEMP, HUMIDITY, PRESSURE DATA LIVE:");

    if (!bme.begin()) {
        Serial.println("Could not find a valid BME280 sensor, verify wiring!");
        while (1);
    }
}

void loop() {
    readTemperature();
    readLuminosity();
    readMotion();
    
}

void readTemperature() {
    temperatureCurrentTime = millis();
    if (temperatureStartTime + temperatureInterval <= temperatureCurrentTime) {
        temperatureStartTime = millis();
        Particle.publish("temperature", String(bme.readTemperature()));
        Particle.publish("humidity", String(bme.readHumidity()));
    }
}

void readLuminosity() {
    luminosityCurrentTime = millis();
    if (luminosityStartTime + luminosityInterval <= luminosityCurrentTime) {
        luminosityStartTime = millis();
        sensors_event_t event;
        tsl.getEvent(&event);
     
         if (event.light)
        {
            Particle.publish("luminosity", String(event.light));
        }
        else
        {
            Serial.print("Sensor overload");
        }
    }
}

void readMotion() {
    proximity_d0 = digitalRead(MOTION_PIN_D0);
    proximity_d2 = digitalRead(MOTION_PIN_D2);
    
  
    leftMotionSensorTriggered = proximity_d0 == LOW;
    rightMotionSensorTriggered = proximity_d2 == LOW;
    if (!leftMotionSensorTriggered && !rightMotionSensorTriggered) {
        delay(1000);
        return;
    }

    startTime = millis();
    currentTime = startTime;
    endTime = startTime + doorDetectionThreshold;
    while (endTime >= currentTime) {
        if ((proximity_d2 == LOW) && (proximity_d0 == LOW)) {
            roomFrequency++;
            Particle.publish("motion",String(roomFrequency));
            leftMotionSensorTriggered = false;
            rightMotionSensorTriggered = false;
            delay(2000);
            return;
        }
        currentTime = millis();
    }

}


