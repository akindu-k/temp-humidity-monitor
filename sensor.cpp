#include <DHT.h>
#include <DHT_U.h>





#include <Arduino.h>


// Define the sensor pin and type
#define DHTPIN 11      // Replace with the actual pin connected to your sensor
#define DHTTYPE DHT22 // Define the type of sensor: DHT11, DHT22, DHT21

DHT dht(DHTPIN, DHTTYPE);

unsigned long startTime = 0;

void setup() {
  Serial.begin(9600);
  dht.begin();
  Serial.println("Temp and Humidity Reading");
}

void loop() {
  startTime = millis(); // Start the timer
  //delay(60000)
  while (millis() - startTime < 10000) { // Run for 30 seconds
    delay(2000); // DHT11 needs a 2-second delay between readings

    // Reading temperature and humidity
    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();

    // Check if any reads failed and exit early (to try again).
    if (isnan(humidity) || isnan(temperature)) {
      Serial.println("Failed to read from DHT sensor!");
      return;
    }

    Serial.println("Sensor triggered!");
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" *C");
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.println(" %");
  }

  
  // Optional: Perform any cleanup or additional tasks after termination

}