#include "privateInfo.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <PubSubClient.h>
#include <WiFi.h> // Arduino Wi-Fi support.  This header is part of the standard library.  https://www.arduino.cc/en/Reference/WiFi


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
long rssi;															     // A global to hold the Received Signal Strength Indicator.
unsigned int ledBlinkInterval  = 200;        // Time between blinks.
unsigned long printCount       = 0;          // A counter of how many times the stats have been published.
unsigned long lastLedBlinkTime = 0;          // The last time LED was blinked.
const unsigned int ONBOARD_LED = 2;          // The GPIO which the onboard LED is connected to.
const uint16_t port            = 1883;       // The broker port.
const String rssiTopic         = "BackDeck/SolarESP32/rssi";
const String ipTopic           = "BackDeck/SolarESP32/ip";
const String macTopic          = "BackDeck/SolarESP32/mac";
const String tempCTopic        = "BackDeck/SolarESP32/bme280/tempC";
const String tempFTopic        = "BackDeck/SolarESP32/bme280/tempF";
const String humidityTopic     = "BackDeck/SolarESP32/bme280/humidity";
const String pressureTopic     = "BackDeck/SolarESP32/bme280/pressure";
const String altitudeTopic     = "BackDeck/SolarESP32/bme280/altitude";
const String HOSTNAME          = "GenericESP";       // The HOSTNAME.
float tempC                    = 21.12F;
float tempF                    = 21.12F;
float pressureHpa              = 21.12F;
float altitudeMeters           = 21.12F;
float humidity                 = 21.12F;


void printValues() 
{
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
   mqttClient.publish( ipTopic.c_str(), ipAddress );
   mqttClient.publish( macTopic.c_str(), macAddress );
} // End of the publishTelemetry() function.


/**
 * @brief checkForSSID() will scan for all visible SSIDs, see if any match 'ssidName',
 * and return a count of how many matches were found.
 *
 * @param ssidName the SSID name to search for.
 * @return int the count of SSIDs which match the passed parameter.
 */
int checkForSSID( const char *ssidName )
{
	int ssidCount = 0;
	byte networkCount = WiFi.scanNetworks();
	if( networkCount == 0 )
		Serial.println( "No WiFi SSIDs are in range!" );
	else
	{
		Serial.printf( "WiFi SSIDs in range: %d\n", networkCount );
		for( int i = 0; i < networkCount; ++i )
		{
			// Check to see if this SSID matches the parameter.
			if( strcmp( ssidName, WiFi.SSID( i ).c_str() ) == 0 )
				ssidCount++;
		}
	}
	return ssidCount;
} // End of checkForSSID() function.


/**
 * @brief wifiBasicConnect() will connect to a SSID.
 */
void wifiBasicConnect()
{
   // Turn the LED off to show Wi-Fi is not connected.
   digitalWrite( ONBOARD_LED, LOW );

  // Don't even try to connect if the SSID cannot be found.
  int ssidCount = checkForSSID( WIFI_SSID );
  if( ssidCount < 1 ) 
    return;
  else
   Serial.printf( "Found %d matching SSID.\n", ssidCount );

   Serial.printf( "Attempting to connect to Wi-Fi SSID '%s'", WIFI_SSID );
   WiFi.mode( WIFI_STA );
   WiFi.config( INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE );
  //  WiFi.setHostname( HOSTNAME );
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
      Serial.print( "WiFi.status(): " );
      Serial.println( WiFi.status() );
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
  Serial.println( F( "Beginning setup..." ) );

	// Get the MAC address and store it in macAddress.
	snprintf( macAddress, 18, "%s", WiFi.macAddress().c_str() );


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
  Serial.println( F( "Setup complete." ) );
  Serial.println();
}


void loop() 
{
	if( WiFiClass::status() != WL_CONNECTED )
		wifiBasicConnect();
	else if( !mqttClient.connected() )
		mqttConnect();
	else
		mqttClient.loop();

  if( lastPrintTime == 0 || ( ( millis() - lastPrintTime ) > printInterval ) )
  {
		pollTelemetry();
    printTelemetry();
		publishTelemetry();
    lastPrintTime = millis();
		Serial.printf( "Next print in %u seconds.\n\n", printInterval / 1000 );
  }

	// Turn on the LED if MQTT is connected, otherwise blink the LED.
	if( lastLedBlinkTime == 0 || ( ( millis() - lastLedBlinkTime ) > ledBlinkInterval ) )
	{
		// If Wi-Fi is connected, but MQTT is not, blink the LED.
		if( WiFiClass::status() == WL_CONNECTED )
		{
			if( mqttClient.state() != 0 )
				toggleLED();						  // Toggle the LED state to show that Wi-Fi is connected but MQTT is not.
			else
				digitalWrite( ONBOARD_LED, 1 ); // Turn the LED on to show both Wi-Fi and MQTT are connected.
		}
		else
			digitalWrite( ONBOARD_LED, 0 ); // Turn the LED off to show that Wi-Fi is not connected.
		lastLedBlinkTime = millis();
	}
}
