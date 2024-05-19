#include "privateInfo.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <PubSubClient.h>
#include <ESP8266WiFi.h> // ESP8266 WiFi support.  https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266WiFi


Adafruit_BME280 bme280;
WiFiClient wifiClient;
PubSubClient mqttClient( wifiClient );
const float seaLevelPressureHpa     = 1013.25;
const unsigned int printInterval    = 10000; // How long to wait between stat printouts.
unsigned long lastPrintTime         = 0;     // The last time a MQTT publish was performed.
const unsigned int pollInterval     = 5000;  // How long to wait between polling telemetry.
unsigned long lastPollTime          = 0;     // The last time a telemetry poll was performed.
const unsigned int publishInterval  = 20000; // How long to wait between publishing telemetry.
unsigned long lastPublishTime       = 0;     // The last time a telemetry publish was performed.
unsigned long lastBrokerConnect     = 0;     // The last time a MQTT broker connection was attempted.
unsigned long brokerCoolDown        = 7000;  // The minimum time between MQTT broker connection attempts.
unsigned long wifiConnectionTimeout = 15000; // The amount of time to wait for a Wi-Fi connection.
char ipAddress[16];                          // A character array to hold the IP address.
char macAddress[18];                         // A character array to hold the MAC address, and append a dash and 3 numbers.
char rssi;                                   // A global to hold the Received Signal Strength Indicator.
unsigned int ledBlinkInterval  = 200;        // Time between blinks.
unsigned long printCount       = 0;          // A counter of how many times the stats have been published.
unsigned long lastLedBlinkTime = 0;          // The last time LED was blinked.
const unsigned int ONBOARD_LED = 2;          // The GPIO which the onboard LED is connected to.
const uint16_t port            = 1883;       // The broker port.
const String rssiTopic         = "BackDeck/SolarESP32/rssi";
const String tempCTopic        = "BackDeck/SolarESP32/bme280/tempC";
const String tempFTopic        = "BackDeck/SolarESP32/bme280/tempF";
const String humidityTopic     = "BackDeck/SolarESP32/bme280/humidity";
const String pressureTopic     = "BackDeck/SolarESP32/bme280/pressure";
const String altitudeTopic     = "BackDeck/SolarESP32/bme280/altitude";
float tempC                    = 21.12F;
float tempF                    = 21.12F;
float pressureHpa              = 21.12F;
float altitudeMeters           = 21.12F;
float humidity                 = 21.12F;


void printValues() 
{
  float tempC = bme280.readTemperature();
  float tempF = ( tempC * 1.8 ) + 32;
  float pressureHpa = bme280.readPressure() / 100.0F;
  float altitudeMeters = bme280.readAltitude( seaLevelPressureHpa );
  float humidity = bme280.readHumidity();

  Serial.printf( "Temperature: %.2f °C\n", tempC );

  Serial.printf( "Temperature: %.2f °F\n", tempF );

  Serial.printf( "Pressure: %.2f hPa\n", pressureHpa );

  Serial.printf( "Humidity: %.2f %%\n", humidity );

  Serial.printf( "Altitude: %.1f m\n", altitudeMeters );

  Serial.println();
}


/**
 * @brief readTelemetry() will read the telemetry and save values to global variables.
 */
void pollTelemetry()
{
   rssi           = WiFi.RSSI();
   tempC          = bme280.readTemperature();
   tempF          = ( tempC * 1.8F ) + 32;
   pressureHpa    = bme280.readPressure() / 100.0F;
   altitudeMeters = bme280.readAltitude( seaLevelPressureHpa );
   humidity       = bme280.readHumidity();
} // End of readTelemetry() function.


/**
 * @brief lookupWifiCode() will return the string for an integer code.
 */
String lookupWifiCode( int code )
{
   switch( code )
   {
      case 0:
         return "Idle";
      case 1:
         return "No SSID";
      case 2:
         return "Scan completed";
      case 3:
         return "Connected";
      case 4:
         return "Connection failed";
      case 5:
         return "Connection lost";
      case 6:
         return "Disconnected";
      default:
         return "Unknown Wi-Fi status code";
   }
} // End of lookupWifiCode() function.


/**
 * @brief lookupMQTTCode() will return the string for an integer state code.
 */
String lookupMQTTCode( int code )
{
   switch( code )
   {
      case -4:
         return "Connection timeout";
      case -3:
         return "Connection lost";
      case -2:
         return "Connect failed";
      case -1:
         return "Disconnected";
      case 0:
         return "Connected";
      case 1:
         return "Bad protocol";
      case 2:
         return "Bad client ID";
      case 3:
         return "Unavailable";
      case 4:
         return "Bad credentials";
      case 5:
         return "Unauthorized";
      default:
         return "Unknown MQTT state code";
   }
} // End of lookupMQTTCode() function.


/**
 * @brief printTelemetry() will print the telemetry to the serial port.
 */
void printTelemetry()
{
   Serial.println();
   printCount++;

   Serial.printf( "Publish count %ld\n", printCount );
   Serial.println( __FILE__ );
   Serial.println();

   Serial.println( "Wi-Fi info:" );
   Serial.printf( "  MAC address: %s\n", macAddress );
   int wifiStatusCode      = WiFi.status();
   String wifiStatusString = lookupWifiCode( wifiStatusCode );
   Serial.printf( "  Wi-Fi status text: %s\n", wifiStatusString.c_str() );
   Serial.printf( "  Wi-Fi status code: %d\n", wifiStatusCode );
   if( wifiStatusCode == 3 )
   {
      Serial.printf( "  IP address: %s\n", ipAddress );
      Serial.printf( "  RSSI: %hhd\n", rssi );
   }
   Serial.println();

   Serial.println( "MQTT info:" );
   Serial.printf( "  Broker: %s:%d\n", BROKER_IP, port );
   String mqttStateString = lookupMQTTCode( mqttClient.state() );
   Serial.printf( "  MQTT state: %s\n", mqttStateString.c_str() );
   if( mqttClient.connected() )
   {
      // Serial.print( "  MQTT broker domain: " );
      // Serial.println( mqttClient.getServerDomain() );
      // Serial.print( "  MQTT broker IP: " );
      // Serial.println( mqttClient.getServerIP() );
      // Serial.print( "  MQTT broker port: " );
      // Serial.println( mqttClient.getServerPort() );
   }

   tempC          = bme280.readTemperature();
   tempF          = ( tempC * 1.8F ) + 32;
   pressureHpa    = bme280.readPressure() / 100.0F;
   altitudeMeters = bme280.readAltitude( seaLevelPressureHpa );
   humidity       = bme280.readHumidity();
   Serial.printf( "Temperature: %.2f °C\n", tempC );
   Serial.printf( "Temperature: %.2f °F\n", tempF );
   Serial.printf( "Pressure: %.2f hPa\n", pressureHpa );
   Serial.printf( "Humidity: %.2f %%\n", humidity );
   Serial.printf( "Altitude: %.1f m\n", altitudeMeters );
   Serial.println();
} // End of the printTelemetry() function.


/**
 * @brief publishTelemetry() will publish telemetry to the MQTT broker.
 */
void publishTelemetry()
{
   // Load the latest values into Strings.
   auto tempCValue    = String( tempC );
   auto tempFValue    = String( tempF );
   auto pressureValue = String( pressureHpa );
   auto altitudeValue = String( altitudeMeters );
   auto humidityValue = String( humidity );
   auto rssiValue     = String( rssi );
   // Publish the values.
   mqttClient.publish( rssiTopic.c_str(), rssiValue.c_str() );
   mqttClient.publish( tempCTopic.c_str(), tempCValue.c_str() );
   mqttClient.publish( tempFTopic.c_str(), tempFValue.c_str() );
   mqttClient.publish( pressureTopic.c_str(), pressureValue.c_str() );
   mqttClient.publish( humidityTopic.c_str(), humidityValue.c_str() );
   mqttClient.publish( altitudeTopic.c_str(), altitudeValue.c_str() );
} // End of the publishTelemetry() function.


/**
 * @brief wifiBasicConnect() will connect to a SSID.
 */
void wifiBasicConnect()
{
   // Turn the LED off to show Wi-Fi is not connected.
   digitalWrite( ONBOARD_LED, LOW );

   Serial.printf( "Attempting to connect to Wi-Fi SSID '%s'", WIFI_SSID );
   WiFi.mode( WIFI_STA );
   WiFi.config( INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE );
   WiFi.setHostname( HOSTNAME );
   WiFi.begin( WIFI_SSID, WIFI_PASSWORD );

   unsigned long wifiConnectionStartTime = millis();

   // Loop until connected, or until wifiConnectionTimeout.
   while( WiFi.status() != WL_CONNECTED && ( millis() - wifiConnectionStartTime < wifiConnectionTimeout ) )
   {
      Serial.print( "." );
      delay( 1000 );
   }
   Serial.println( "" );

   if( WiFi.status() == WL_CONNECTED )
   {
      Serial.print( "wifiClient.status(): " );
      Serial.println( wifiClient.status() );
      Serial.print( "WL_CONNECTED: " );
      Serial.println( WL_CONNECTED );
      // Print that Wi-Fi has connected.
      Serial.println( "\nWi-Fi connection established!" );
      snprintf( ipAddress, 16, "%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3] );
      // Turn the LED on to show that Wi-Fi is connected.
      digitalWrite( ONBOARD_LED, HIGH );
      return;
   }
   else
      Serial.println( "Wi-Fi failed to connect in the timeout period.\n" );
} // End of the wifiBasicConnect() function.


/**
 * @brief mqttConnect() will connect to the MQTT broker.
 */
void mqttConnect()
{
   // Connect the first time.  Connect after cool down.
   if( lastBrokerConnect == 0 || ( millis() - lastBrokerConnect ) > brokerCoolDown )
   {
      Serial.println( "  mqttConnect()" );
      lastBrokerConnect = millis();
      digitalWrite( ONBOARD_LED, LOW );
      Serial.printf( "  Connecting to broker at %s:%d...\n", BROKER_IP, port );
      mqttClient.setServer( BROKER_IP, port );

      if( mqttClient.connect( macAddress ) )
         Serial.print( "  Connected to MQTT Broker.\n" );
      else
      {
         int mqttStateCode      = mqttClient.state();
         String mqttStateString = lookupMQTTCode( mqttStateCode );
         Serial.printf( "  MQTT state: %s\n", mqttStateString.c_str() );
         Serial.printf( "  MQTT state code: %d\n", mqttStateCode );
         return;
      }

      mqttClient.subscribe( "led1" );
      digitalWrite( ONBOARD_LED, HIGH );
   }
} // End of the mqttConnect() function.


/**
 * @brief infiniteLoop() is a way to put the MCU into a blocking state.
 */
void infiniteLoop()
{
// #pragma clang diagnostic push
// #pragma ide diagnostic ignored "EndlessLoop"
  while( true )
  {
    delay( 100 );
    // Serial.println( "Blink!" );
    toggleLED();
  }
// #pragma clang diagnostic pop
} // End of the infiniteLoop() function.


/**
 * @brief toggleLED() will change the state of the LED.
 * This function does not manage any timings.
 */
void toggleLED()
{
   if( digitalRead( ONBOARD_LED ) != 1 )
      digitalWrite( ONBOARD_LED, 1 );
   else
      digitalWrite( ONBOARD_LED, 0 );
} // End of the toggleLED() function.


void setup() 
{
  // Give time to connect to serial after a flash.
  delay( 1000 );
  Serial.begin( 115200 );
  // Pause one second if the serial port is not yet ready.
  if( !Serial )
    delay( 1000 );
  Serial.println();
  Serial.println( F( "BME280 test" ) );

  unsigned status = bme280.begin( 0x76 );
  if( !status )
  {
    Serial.println( "Could not find a valid BME280 sensor, check wiring, address, sensor ID!" );
    Serial.print( "SensorID was: 0x" ); 
    Serial.println( bme280.sensorID(), 16 );
    Serial.print( "        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n" );
    Serial.print( "   ID of 0x56-0x58 represents a BMP 280,\n" );
    Serial.print( "        ID of 0x60 represents a BME 280.\n" );
    Serial.print( "        ID of 0x61 represents a BME 680.\n" );
    Serial.println( "\n" );
    Serial.println( "Going into an infinite delay loop!" );
    while( 1 )
      delay( 10 );
  }
  Serial.println();
}


void loop() 
{
  if( lastPrintTime == 0 || ( ( millis() - lastPrintTime ) > printInterval ) )
  {
    printValues();
    lastPrintTime = millis();
  }
}
