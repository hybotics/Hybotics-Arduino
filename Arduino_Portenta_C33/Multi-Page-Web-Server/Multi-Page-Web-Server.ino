/*
 * Created by ArduinoGetStarted.com
 *
 * This example code is in the public domain
 *
 * Tutorial page: https://arduinogetstarted.com/tutorials/arduino-web-server-multiple-pages
 */

#include "Adafruit_SHT4x.h"
Adafruit_SHT4x sht4 = Adafruit_SHT4x();

#include  "Multi-Page-Web-Server.h"
#include  "Secrets.h"

#define PAGE_HOME 0
#define PAGE_TEMPERATURE 1
#define PAGE_DOOR 2
#define PAGE_LED 3

#define PAGE_ERROR_404 -1
#define PAGE_ERROR_405 -2

char ssid[] = WIFI_SSID;  // change your network SSID (name)
char passwd[] = WIFI_PASSWD;   // change your network password (use for WPA, or use as key for WEP)

int wifi_status = WL_IDLE_STATUS;
ColorRGB color;
uint16_t request_count = 0;

WiFiServer server(80);

/*    
  We are connected now, so print out the connection status:
*/
void print_wifi_status(void) {
  //  Print your board's IP address:
  Serial.print("Connected! IP Address is ");
  Serial.println(WiFi.localIP());

  //  Print the received signal strength:
  Serial.print("Signal strength (RSSI):");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");
}

/*
  Halt everything - used for unrecoverable errors
*/
void halt (void) {
  Serial.println("Halting...");

  while (true) {
    delay(1);
  }
}

/*
  Convert the Celsius temperature to Fahrenheit

  Returns: (float) temperature in fahrenheit
*/
float to_fahrenheit (float celsius) {
  return celsius * 1.8 + 32;
}

Environment_Data check_data (Environment_Data curr_data) {
  Environment_Data result;
  Three_Axis triple = {0.0, 0.0, 0.0};

  //  If we have current data, copy it
  if (curr_data.valid) {
    //  Save existing data
    result = curr_data;
  } else {
    //  Initialize data structure
    result.valid = false;

    result.celsius = 0.0;
    result.fahrenheit = 0.0;
    result.humidity = 0.0;

    result.accelerometer = triple;
    result.gyroscope = triple;
    result.temperature = 0.0;
  
    result.magnetometer = triple;
  }

  return result;
}

Environment_Data get_temperature(Environment_Data curr_data, Adafruit_SHT4x *sht) {
  Environment_Data sensors;
  sensors_event_t rel_humidity, temperature;
  uint32_t timestamp = millis();
  float celsius, humidity;

  sensors = check_data(curr_data);

  sht->getEvent(&rel_humidity, &temperature);// populate temp and humidity objects with fresh data
  celsius = temperature.temperature;

  sensors.celsius = celsius;
  sensors.fahrenheit = to_fahrenheit(celsius);
  sensors.humidity = rel_humidity.relative_humidity;

  return sensors;
}

/*
  Initialize the SHT4x temperature and humidity sensor
*/
Adafruit_SHT4x init_sht4x (Adafruit_SHT4x *sht) {
  if (sht->begin()) {
    Serial.println();
    Serial.print("Found an SHT4x sensor with the serial number 0x");
    Serial.println(sht->readSerial(), HEX);

    // You can have 3 different precisions, higher precision takes longer
    sht->setPrecision(SHT4X_HIGH_PRECISION);

    switch (sht->getPrecision()) {
      case SHT4X_HIGH_PRECISION: 
        Serial.print("High precision");
        break;
      case SHT4X_MED_PRECISION: 
        Serial.print("Med precision");
        break;
      case SHT4X_LOW_PRECISION: 
        Serial.print("Low precision");
        break;
    }

    Serial.print(", ");

    //  You can have 6 different heater settings
    //    higher heat and longer times uses more power
    //    and reads will take longer too!
    sht->setHeater(SHT4X_NO_HEATER);

    switch (sht->getHeater()) {
      case SHT4X_NO_HEATER: 
        Serial.println("No heater");
        break;
      case SHT4X_HIGH_HEATER_1S: 
        Serial.println("High heat for 1 second");
        break;
      case SHT4X_HIGH_HEATER_100MS: 
        Serial.println("High heat for 0.1 second");
        break;
      case SHT4X_MED_HEATER_1S: 
        Serial.println("Medium heat for 1 second");
        break;
      case SHT4X_MED_HEATER_100MS: 
        Serial.println("Medium heat for 0.1 second");
        break;
      case SHT4X_LOW_HEATER_1S: 
        Serial.println("Low heat for 1 second");
        break;
      case SHT4X_LOW_HEATER_100MS: 
        Serial.println("Low heat for 0.1 second");
        break;
    }

    Serial.println();
  } else {
    Serial.println("Could not find any SHT4x sensors!");

    halt();
  }

  return *sht;
}

bool connect_to_wifi (char *ssid, char *passwd, uint8_t timeout=CONNECTION_TIMEOUT_MS) {
  bool connected = false;
  uint8_t connect_attempts = 0;

/*
  TODO: Add connection attempt counter and limit on while()
*/

  //  Attempt to connect to WiFi network:
  while (wifi_status != WL_CONNECTED) {
    connect_attempts += 1;

    Serial.print("Attempt #");
    Serial.print(connect_attempts);
    Serial.print(" to connect to the '");
    Serial.print(ssid);
    Serial.println("' network");

    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    wifi_status = WiFi.begin(ssid, passwd);

    //  Wait for connection:
    delay(CONNECTION_TIMEOUT_MS);

    //  Print connection status
    print_wifi_status();

    connected = true;
  }

  return connected;
}

void blink_rgb (ColorRGB color, uint8_t blink_rate_ms=DEFAULT_BLINK_RATE_MS, uint8_t nr_cycles=DEFAULT_NR_CYCLES) {
  uint8_t count;

  for (count=0; count<nr_cycles; count++) {
    digitalWrite(LEDR, color.redB);
    digitalWrite(LEDG, color.greenB);
    digitalWrite(LEDB, color.blueB);

    delay(blink_rate_ms);

    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDB, HIGH);

    delay(blink_rate_ms);
  }
}

void setup() {
  //RTCTime current_time;
  Environment_Data curr_data;
  String firmware_version, date_time;
  bool connected;

  curr_data.valid = false;

  //  Initialize serial and wait for the port to open:
  Serial.begin(115200);

  while(!Serial) {
    delay(10);
  }

  Serial.println();
  Serial.println("Arduino Portenta C33/H7 Multi-Page Web Server");
  Serial.println();

  connected = connect_to_wifi(ssid, passwd);

  if (connected) {
    //  Initialize the onboard RGB LED
    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);

    //  Turn the LEDs OFF. Pins are active LOW=ON
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDB, HIGH);

    //  Set color to BLUE
    color.redB = HIGH;
    color.greenB = HIGH;
    color.blueB = LOW;

    //  Initialize the SHT4x Temperature and Humidity sensors
    if (USING_SHT45_TEMP) {
       sht4 = init_sht4x(&sht4);
    }
  
    //  Start the web server
    Serial.println();
    Serial.println("Starting the web server");
    server.begin(WEB_SERVER_PORT);
    Serial.println();
  } else {
    Serial.println("Unable to connect to WiFi - halting");

    halt();
  }
}

void loop() {
  Environment_Data sensors;
  float celsius, fahrenheit, humidity;
  WiFiClient client;
  float lux;
  String html, HTTP_req = "", HTTP_header;

  //  Listen for incoming clients
  client = server.available();

  if (client) {
    request_count += 1;
    Serial.print("***** Request #");
    Serial.println(request_count);

    blink_rgb(color);

    //  Read the first line of HTTP request header
    while (client.connected()) {
      if (client.available()) {
        Serial.println("New HTTP Request");
        HTTP_req = client.readStringUntil('\n');  // read the first line of HTTP request
        Serial.print("<< ");
        Serial.println(HTTP_req);  // print HTTP request to Serial Monitor
        break;
      }
    }

    // read the remaining lines of HTTP request header
    while (client.connected()) {
      if (client.available()) {
        HTTP_header = client.readStringUntil('\n');  // read the header line of HTTP request

        if (HTTP_header.equals("\r"))  // the end of HTTP request
          break;

        Serial.print("<< ");
        Serial.println(HTTP_header);  //  Print HTTP request to Serial Monitor
      }
    }

    // ROUTING
    // This example supports the following:
    // - GET /
    // - GET /index
    // - GET /index.html
    // - GET /temperature
    // - GET /temperature.html
    // - GET /door
    // - GET /door.html
    // - GET /led
    // - GET /led.html

    int page_id = 0;

    if (HTTP_req.indexOf("GET") == 0) {  // check if request method is GET
      if (HTTP_req.indexOf("GET / ") > -1 || HTTP_req.indexOf("GET /index ") > -1 || HTTP_req.indexOf("GET /index.html ") > -1) {
        Serial.println("Home page");
        page_id = PAGE_HOME;
      } else if (HTTP_req.indexOf("GET /temperature ") > -1 || HTTP_req.indexOf("GET /temperature.html ") > -1) {
        Serial.println("Temperature page");
        page_id = PAGE_TEMPERATURE;
      } else if (HTTP_req.indexOf("GET /door ") > -1 || HTTP_req.indexOf("GET /door.html ") > -1) {
        Serial.println("Door page");
        page_id = PAGE_DOOR;
      } else if (HTTP_req.indexOf("GET /led ") > -1 || HTTP_req.indexOf("GET /led.html ") > -1) {
        Serial.println("LED page");
        page_id = PAGE_LED;
      } else {  // 404 Not Found
        Serial.println("404 Not Found");
        page_id = PAGE_ERROR_404;
      }
    } else {  // 405 Method Not Allowed
      Serial.println("405 Method Not Allowed");
      page_id = PAGE_ERROR_405;
    }

    // send the HTTP response
    // send the HTTP response header
    if (page_id == PAGE_ERROR_404)
      client.println("HTTP/1.1 404 Not Found");
    if (page_id == PAGE_ERROR_405)
      client.println("HTTP/1.1 405 Method Not Allowed");
    else
      client.println("HTTP/1.1 200 OK");

    client.println("Content-Type: text/html");
    client.println("Connection: close");  // the connection will be closed after completion of the response
    client.println();                     // the separator between HTTP header and body

    // send the simple HTTP response body
    client.println("<!DOCTYPE HTML>");
    client.println("<html>");
    client.println("<head>");
    client.println("<link rel=\"icon\" href=\"data:,\">");
    client.println("</head>");

    client.println("<p>");

    client.println("<p>");
    client.print("<CENTER><H1>From the home of <span style=\"color: blue;\">");
    client.println("Hybrid Robotics</span></H1></CENTER>");

    client.print("<H3>Request #");
    client.print(request_count);
    client.println("</H3>");

    switch (page_id) {
      case PAGE_HOME:
        client.print("<CENTER><H1><span style=\"color: blue;\">");
        client.println("Hybrid Robotics</span></H1></CENTER>");

        break;

      case PAGE_TEMPERATURE:
        client.println("This is temperature page");

        if (USING_SHT45_TEMP) {
          Serial.println("Getting temperature and humidity readings");

          sensors = get_temperature(sensors, &sht4);
          celsius = sensors.celsius;
          fahrenheit = to_fahrenheit(celsius);
          humidity = sensors.humidity;

          Serial.print("Temperature is ");
          Serial.print(fahrenheit);
          Serial.print("°F (");
          Serial.print(celsius);
          Serial.println("°C)");

          Serial.print("Humidity is ");
          Serial.print(humidity);
          Serial.println("% rH");

          Serial.println("Sending sensor readings");

          client.print("<H3>Temperature: <span style=\"color: green;\">");
          client.print(fahrenheit, 2);
          client.print("&deg;F </span>(");
          client.print("<span style=\"color: yellow;\">");
          client.print(celsius);
          client.println("&deg;C</span>)</H3>");

          client.print("<H3>Humidity: <span style=\"color: magenta;\">");
          client.print(humidity);
          client.println("% rH</span></H3><BR \>");
        }

        break;
      case PAGE_DOOR:
        client.println("This is door page");
        break;

      case PAGE_LED:
        client.println("This is LED page");
        blink_rgb(color);
        break;

      case PAGE_ERROR_404:
        client.println("Page Not Found");
        break;

      case PAGE_ERROR_405:
        client.println("Method Not Allowed");
        break;
    }

    client.println("</html>");
    client.flush();

    client.flush();

    //  Give the web browser time to receive the data
    //delay(10);

    Serial.println();

    //  Close the connection:
    client.stop();
  }
}
