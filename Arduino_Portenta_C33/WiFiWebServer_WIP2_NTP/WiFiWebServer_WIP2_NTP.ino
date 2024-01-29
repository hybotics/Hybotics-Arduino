/*
 */

#include "Secrets.h"
#include "WiFiWebServer.h"

#include <NTP.h>
WiFiUDP Udp; // A UDP instance to let us send and receive packets over UDP
NTP time_client(Udp);

/*//Include the NTP library
#include <NTPClient.h>
WiFiUDP Udp; // A UDP instance to let us send and receive packets over UDP
NTPClient time_client(Udp);
*/

// Include the RTC library
#include "RTC.h"

#include <Adafruit_LSM6DSOX.h>
Adafruit_LSM6DSOX sox;

#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
Adafruit_LIS3MDL lis3;

#include "Adafruit_SHT4x.h"
Adafruit_SHT4x sht4 = Adafruit_SHT4x();

#include "Adafruit_VEML7700.h"
Adafruit_VEML7700 veml = Adafruit_VEML7700();

char ssid[] = WIFI_SSID;  // change your network SSID (name)
char passwd[] = WIFI_PASSWD;   // change your network password (use for WPA, or use as key for WEP)

int wifi_status = WL_IDLE_STATUS;
uint16_t request_count = 0;

WiFiServer server(WEB_SERVER_PORT);
ColorRGB color;

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
  Get the current date and time
*/
String timestamp (bool hours_ty=false, bool long_month=false) {
  String long_months[12] = { "January", "February", "March", "April", "May", "June", "July",
    "August", "September", "October", "November", "December" };
  RTCTime current_time;
  String date_time, date_str, time_str;
  String year_str, month_str, day_str;
  String hours_str, min_sec_str;
  String am_pm = "am";
  uint8_t position, str_len,  month, hours;

  // Retrieve the date and time from the RTC and print them
  RTC.getTime(current_time); 
  date_time = String(current_time);

  str_len = date_time.length();
  position = date_time.indexOf("T");
  date_str = date_time.substring(0, position);
  time_str = date_time.substring(position + 1, str_len);

  str_len = date_str.length();
  year_str = date_str.substring(0, 4);
  month_str = date_str.substring(5, 7);
  month = month_str.toInt();
  day_str = date_str.substring(8, str_len);

  str_len = time_str.length();
  hours_str = time_str.substring(0, 2);
  hours = hours_str.toInt();
  min_sec_str = time_str.substring(3, str_len);

  if (!hours_ty) {
    if (hours > 12) {
      hours = hours - 12;
      am_pm = " PM";
    } else {
      am_pm = " AM";
    }

    time_str = String(hours) + ":" + min_sec_str + am_pm;
  }

  if (long_month) {
    month_str = long_months[month - 1];
    date_str = month_str + " " + day_str + ", " + year_str;
  } else {
    date_str = month_str + "/" + day_str + "/" + year_str;
  }

  date_time = date_str + " at " + time_str;
  return date_time;
}

/*
  Set the onboard RTC from an NTP (internet) time source

  Returns: (NTPClient) a time client
*/
RTCTime set_rtc (int8_t utc_offset_hrs, NTP *time_cl) {
  RTCTime current_time, time_to_set;
  uint32_t unix_time;

  //  Get the current date and time from an NTP server and convert
  //    it to UTC +2 by passing the time zone offset in hours.
  // You may change the time zone offset to your local one.
  unix_time = time_cl->epoch() + (utc_offset_hrs * 3600);
  time_to_set = RTCTime(unix_time);
  RTC.setTime(time_to_set);

  //  Set the UTC time offset for the current timezone
  //time_cl->setTimeOffset(utc_offset_hrs);
  //void ruleDST(const char* tzName, int8_t week, int8_t wday, int8_t month, int8_t hour, int tzOffset);
  /**
   * @brief set the rule for DST (daylight saving time)
   * start date of DST 
   * 
   * @param tzName name of the time zone
   * @param week Last, First, Second, Third, Fourth (0 - 4)
   * @param wday Sun, Mon, Tue, Wed, Thu, Fri, Sat (0 - 7)
   * @param month Jan, Feb, Mar, Apr, May, Jun, Jul, Aug, Sep, Oct, Nov, Dec (0 -11)
   * @param hour the local hour when rule chages
   * @param tzOffset sum of summertime and timezone offset
   void ruleDST(const char* tzName, int8_t week, int8_t wday, int8_t month, int8_t hour, int tzOffset);
   */
  time_cl->ruleDST("PDT", Third, Sun, Mar, 10, -8);
  time_cl->ruleSTD("PST", First, Sun, Nov, 3, -7); // last sunday in october 3:00, timezone +60min (+1 GMT)
  //time_cl->ruleDST("PDT", Second, Sun, Mar, 1, 1);

  /**
   * @brief set the rule for STD (standard day)
   * end date of DST
   * 
   * @param tzName name of the time zone
   * @param week Last, First, Second, Third, Fourth (0 - 4)
   * @param wday Sun, Mon, Tue, Wed, Thu, Fri, Sat (0 - 7)
   * @param month Jan, Feb, Mar, Apr, May, Jun, Jul, Aug, Sep, Oct, Nov, Dec (0 -11)
   * @param hour the local hour when rule chages
   * @param tzOffset timezone offset
    void ruleSTD(const char* tzName, int8_t week, int8_t wday, int8_t month, int8_t hour, int tzOffset);
   */
  //time_cl->ruleSTD("PST");
  return current_time;
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

Environment_Data get_lis3mdl (Environment_Data curr_data, Adafruit_LIS3MDL *lis3) {
  Environment_Data sensors;
  sensors_event_t event; 

  sensors = check_data(curr_data);

  lis3->getEvent(&event);
  sensors.magnetometer.x = event.magnetic.x;
  sensors.magnetometer.y = event.magnetic.y;
  sensors.magnetometer.z = event.magnetic.z;

  return sensors;
}

Environment_Data get_lsm6dsox (Environment_Data curr_data, Adafruit_LSM6DSOX *sx) {
  Environment_Data sensors;
  sensors_event_t accel, gyro, temperature, mag;

  sensors = check_data(curr_data);

  sx->getEvent(&accel, &gyro, &temperature);

  sensors.accelerometer.x = accel.acceleration.x;
  sensors.accelerometer.y = accel.acceleration.y;
  sensors.accelerometer.z = accel.acceleration.z;
  sensors.gyroscope.x = gyro.gyro.x;
  sensors.gyroscope.y = gyro.gyro.y;
  sensors.gyroscope.z = gyro.gyro.z;
  sensors.temperature = temperature.temperature;

  return sensors;
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

Adafruit_LIS3MDL init_lis3mdl (Adafruit_LIS3MDL *lis3) {
  // Try to initialize!
  if (lis3->begin_I2C()) {          // hardware I2C mode, can pass in address & alt Wire
    Serial.println("LIS3MDL Found!");

    lis3->setPerformanceMode(LIS3MDL_MEDIUMMODE);
    Serial.print("Performance mode set to: ");

    switch (lis3->getPerformanceMode()) {
      case LIS3MDL_LOWPOWERMODE:
        Serial.println("Low");
        break;
      case LIS3MDL_MEDIUMMODE:
        Serial.println("Medium");
        break;
      case LIS3MDL_HIGHMODE:
        Serial.println("High");
        break;
      case LIS3MDL_ULTRAHIGHMODE:
        Serial.println("Ultra-High");
        break;
    }

    lis3->setOperationMode(LIS3MDL_CONTINUOUSMODE);
    Serial.print("Operation mode set to: ");

    // Single shot mode will complete conversion and go into power down
    switch (lis3->getOperationMode()) {
      case LIS3MDL_CONTINUOUSMODE:
        Serial.println("Continuous");
          break;
      case LIS3MDL_SINGLEMODE:
        Serial.println("Single mode");
        break;
      case LIS3MDL_POWERDOWNMODE:
        Serial.println("Power-down");
        break;
    }

    lis3->setDataRate(LIS3MDL_DATARATE_155_HZ);
    // You can check the datarate by looking at the frequency of the DRDY pin
    Serial.print("Data rate set to: ");

    switch (lis3->getDataRate()) {
      case LIS3MDL_DATARATE_0_625_HZ:
        Serial.println("0.625 Hz");
        break;
      case LIS3MDL_DATARATE_1_25_HZ:
        Serial.println("1.25 Hz");
        break;
      case LIS3MDL_DATARATE_2_5_HZ:
        Serial.println("2.5 Hz");
        break;
      case LIS3MDL_DATARATE_5_HZ:
        Serial.println("5 Hz");
        break;
      case LIS3MDL_DATARATE_10_HZ:
        Serial.println("10 Hz");
        break;
      case LIS3MDL_DATARATE_20_HZ:
        Serial.println("20 Hz");
        break;
      case LIS3MDL_DATARATE_40_HZ:
        Serial.println("40 Hz");
        break;
      case LIS3MDL_DATARATE_80_HZ:
        Serial.println("80 Hz");
        break;
      case LIS3MDL_DATARATE_155_HZ:
        Serial.println("155 Hz");
        break;
      case LIS3MDL_DATARATE_300_HZ:
        Serial.println("300 Hz");
        break;
      case LIS3MDL_DATARATE_560_HZ:
        Serial.println("560 Hz");
        break;
      case LIS3MDL_DATARATE_1000_HZ:
        Serial.println("1000 Hz");
        break;
    }
    
    lis3->setRange(LIS3MDL_RANGE_4_GAUSS);
    Serial.print("Range set to: ");

    switch (lis3->getRange()) {
      case LIS3MDL_RANGE_4_GAUSS:
        Serial.println("+-4 gauss");
        break;
      case LIS3MDL_RANGE_8_GAUSS:
        Serial.println("+-8 gauss");
        break;
      case LIS3MDL_RANGE_12_GAUSS:
        Serial.println("+-12 gauss");
        break;
      case LIS3MDL_RANGE_16_GAUSS:
        Serial.println("+-16 gauss");
        break;
    }

    lis3->setIntThreshold(500);
    lis3->configInterrupt(false, false, true, // enable z axis
                            true, // polarity
                            false, // don't latch
                            true); // enabled!
  } else {
    Serial.println("Unable to find the magnetometer!");

    halt();
  }

  return *lis3;
}

Adafruit_LSM6DSOX init_lsm6dsox (Adafruit_LSM6DSOX *sx) {
  //  Initialie the IMU
  if (sx->begin_I2C()) {
    Serial.println("Found the LSM6DSOX IMU!");

    sx->setAccelRange(LSM6DS_ACCEL_RANGE_2_G);

    Serial.print("Accelerometer range set to: ");

    switch (sx->getAccelRange()) {
      case LSM6DS_ACCEL_RANGE_2_G:
        Serial.println("+-2G");
        break;
      case LSM6DS_ACCEL_RANGE_4_G:
        Serial.println("+-4G");
        break;
      case LSM6DS_ACCEL_RANGE_8_G:
        Serial.println("+-8G");
        break;
      case LSM6DS_ACCEL_RANGE_16_G:
        Serial.println("+-16G");
        break;
    }

    sx->setAccelDataRate(LSM6DS_RATE_12_5_HZ);

    Serial.print("Accelerometer data rate set to: ");

    switch (sx->getAccelDataRate()) {
      case LSM6DS_RATE_SHUTDOWN:
        Serial.println("0 Hz");
        break;
      case LSM6DS_RATE_12_5_HZ:
        Serial.println("12.5 Hz");
        break;
      case LSM6DS_RATE_26_HZ:
        Serial.println("26 Hz");
        break;
      case LSM6DS_RATE_52_HZ:
        Serial.println("52 Hz");
        break;
      case LSM6DS_RATE_104_HZ:
        Serial.println("104 Hz");
        break;
      case LSM6DS_RATE_208_HZ:
        Serial.println("208 Hz");
        break;
      case LSM6DS_RATE_416_HZ:
        Serial.println("416 Hz");
        break;
      case LSM6DS_RATE_833_HZ:
        Serial.println("833 Hz");
        break;
      case LSM6DS_RATE_1_66K_HZ:
        Serial.println("1.66 KHz");
        break;
      case LSM6DS_RATE_3_33K_HZ:
        Serial.println("3.33 KHz");
        break;
      case LSM6DS_RATE_6_66K_HZ:
        Serial.println("6.66 KHz");
        break;
    }

    sx->setGyroRange(LSM6DS_GYRO_RANGE_250_DPS );

    Serial.print("Gyroscope range set to: ");

    switch (sx->getGyroRange()) {
      case LSM6DS_GYRO_RANGE_125_DPS:
        Serial.println("125 degrees/s");
        break;
      case LSM6DS_GYRO_RANGE_250_DPS:
        Serial.println("250 degrees/s");
        break;
      case LSM6DS_GYRO_RANGE_500_DPS:
        Serial.println("500 degrees/s");
        break;
      case LSM6DS_GYRO_RANGE_1000_DPS:
        Serial.println("1000 degrees/s");
        break;
      case LSM6DS_GYRO_RANGE_2000_DPS:
        Serial.println("2000 degrees/s");
        break;
      case ISM330DHCX_GYRO_RANGE_4000_DPS:
        break; // unsupported range for the DSOX
    }

    sx->setGyroDataRate(LSM6DS_RATE_12_5_HZ);

    Serial.print("Gyro data rate set to: ");

    switch (sx->getGyroDataRate()) {
      case LSM6DS_RATE_SHUTDOWN:
        Serial.println("0 Hz");
        break;
      case LSM6DS_RATE_12_5_HZ:
        Serial.println("12.5 Hz");
        break;
      case LSM6DS_RATE_26_HZ:
        Serial.println("26 Hz");
        break;
      case LSM6DS_RATE_52_HZ:
        Serial.println("52 Hz");
        break;
      case LSM6DS_RATE_104_HZ:
        Serial.println("104 Hz");
        break;
      case LSM6DS_RATE_208_HZ:
        Serial.println("208 Hz");
        break;
      case LSM6DS_RATE_416_HZ:
        Serial.println("416 Hz");
        break;
      case LSM6DS_RATE_833_HZ:
        Serial.println("833 Hz");
        break;
      case LSM6DS_RATE_1_66K_HZ:
        Serial.println("1.66 KHz");
        break;
      case LSM6DS_RATE_3_33K_HZ:
        Serial.println("3.33 KHz");
        break;
      case LSM6DS_RATE_6_66K_HZ:
        Serial.println("6.66 KHz");
        break;
    }
  } else {
      Serial.println("Failed to find the LSM6DSOX IMU!");

      halt();
  }

  return *sx;
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

void setup(void) {
  RTCTime current_time;
  Environment_Data curr_data;
  String firmware_version, date_time;
  bool connected;

  //  Force initialization of the environment data structure
  curr_data.valid = false;

  //  Initialize serial and wait for the port to open:
  Serial.begin(115200);
  Serial.println("Initializing serial port");

  while(!Serial) {
    ;
  }

  Serial.println();
  Serial.println("Arduino Portenta C33/H7 Web Server");
  Serial.println();

  //  Check firmware version
  firmware_version = WiFi.firmwareVersion();

  if (firmware_version < WIFI_FIRMWARE_LATEST_VERSION)
    Serial.println("Please upgrade the firmware!");

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

    if (USING_LSM6DSOX_LIS3MDL_IMU) {  
      sox = init_lsm6dsox(&sox);
    }

    if (USING_LIS3MDL_MAG) {
      lis3 = init_lis3mdl(&lis3);
    }

    if (USING_VEML_LUX) {
      if (veml.begin()) {
        Serial.println("Found a VEML7700 Lux sensor");
        Serial.println("Settings used for reading:");
        Serial.print(F("Gain: "));

        switch (veml.getGain()) {
          case VEML7700_GAIN_1:
            Serial.println("1");
            break;
          case VEML7700_GAIN_2:
            Serial.println("2");
            break;
          case VEML7700_GAIN_1_4:
            Serial.println("1/4");
            break;
          case VEML7700_GAIN_1_8: Serial.println("1/8"); break;
        }

        Serial.print(F("Integration Time (ms): "));

        switch (veml.getIntegrationTime()) {
          case VEML7700_IT_25MS:
            Serial.println("25");
            break;
          case VEML7700_IT_50MS:
            Serial.println("50");
            break;
          case VEML7700_IT_100MS:
            Serial.println("100");
            break;
          case VEML7700_IT_200MS:
            Serial.println("200");
            break;
          case VEML7700_IT_400MS:
            Serial.println("400");
            break;
          case VEML7700_IT_800MS:
            Serial.println("800");
            break;
        }
      } else {
        Serial.println("Unable to find the VEML7700 sensor!");

        halt();
      }
    }

    //  Start the time client
    Serial.println("Starting the time client");
    time_client.begin();
    time_client.update();

    //  Start the Real Time Clock and set the current time
    Serial.println("Starting the RTC");
    RTC.begin();
    set_rtc(UTC_OFFSET_HRS, &time_client);
    RTC.getTime(current_time);

    Serial.println();
    Serial.print("Today is ");
    Serial.println(timestamp());
  
    //  Start the web server
    Serial.println();
    Serial.println("Starting the web server");
    server.begin(WEB_SERVER_PORT);
    Serial.println();

    date_time = timestamp(false, true);
  } else {
    Serial.print("Unable to connect to the '");
    Serial.print(ssid);
    Serial.print("' network!");

    halt();
  }
}

void loop(void) {
  Environment_Data sensors;
  float celsius, fahrenheit, humidity;
  WiFiClient client;
  float lux;

  client = server.available();

  if (client) {
    request_count += 1;
    Serial.print("***** Request #");
    Serial.println(request_count);

    blink_rgb(color);
    
    //  Read the HTTP request header line by line
    while (client.connected()) {
      if (client.available()) {
        String HTTP_header = client.readStringUntil('\n');  // read the header line of HTTP request

        if (HTTP_header.equals("\r"))  // the end of HTTP request
          break;

        Serial.print("<< ");
        Serial.println(HTTP_header);  // print HTTP request to Serial Monitor
      }
    }

    //  Send the HTTP response
    //  Send the HTTP response header
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println("Connection: close");  // the connection will be closed after completion of the response
    client.println();                     // the separator between HTTP header and body

    //  Send the HTTP response body
    client.println("<!DOCTYPE HTML>");
    client.println("<html>");
    client.println("<head>");
    client.println("<link rel=\"icon\" href=\"data:,\">");
    client.println("</head>");

    Serial.println("Ready to send header");

    client.println("<p>");
    client.print("<CENTER><H1>From the home of <span style=\"color: blue;\">");
    client.println("Hybrid Robotics</span></H1></CENTER>");

    client.print("<CENTER><H2>");
    client.print(timestamp());
    client.println("</H2></CENTER>");

    client.print("<H3>Request #");
    client.print(request_count);
    client.println("</H3>");

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

    if (USING_LSM6DSOX_LIS3MDL_IMU) {
      Serial.println("Getting IMU reading");
      sensors = get_lsm6dsox(sensors, &sox);

      Serial.print("IMU Temperature: <span style=\"color: green;\">");
      Serial.print(to_fahrenheit(sensors.temperature));
      Serial.print("°F (");
      Serial.print(sensors.temperature);
      Serial.println("°C</span>)");

      client.print("IMU Temperature: <span style=\"color: green;\">");
      client.print(to_fahrenheit(sensors.temperature));
      client.print("&deg;F </span>(<span style=\"color: yellow;\">");
      client.print(sensors.temperature);
      client.println("&deg;C</span>)</BR >");

      //  Display the results (acceleration is measured in m/s^2)
      Serial.print("Accelerometer: x = ");
      Serial.print(sensors.accelerometer.x);
      Serial.print(", y = ");
      Serial.print(sensors.accelerometer.y);
      Serial.print(", z = ");
      Serial.print(sensors.accelerometer.z);
      Serial.println(" m/s^2");

      client.print("Accelerometer: <span style=\"color: yellow;\">x = ");
      client.print(sensors.accelerometer.x);
      client.print(", y = ");
      client.print(sensors.accelerometer.y);
      client.print(", z = ");
      client.print(sensors.accelerometer.z);
      client.println(" m/s^2</span></BR >");

      // Display the results (rotation is measured in radians/s)
      Serial.print("Gyroscope: x = ");
      Serial.print(sensors.gyroscope.x);
      Serial.print(", y = ");
      Serial.print(sensors.gyroscope.y);
      Serial.print(", z = ");
      Serial.print(sensors.gyroscope.z);
      Serial.println(" radians/s");
      Serial.println();

      client.print("Gyroscope: <span style=\"color: yellow;\">x = ");
      client.print(sensors.gyroscope.x);
      client.print(", y = ");
      client.print(sensors.gyroscope.y);
      client.print(", z = ");
      client.print(sensors.gyroscope.z);
      client.println(" radians/s</span></BR >");
    }

    if (USING_LIS3MDL_MAG) {
      Serial.println("Getting magnetometer readings");
      sensors = get_lis3mdl(sensors, &lis3);

      //  Display the results (magnetic field is measured in uTesla)
      Serial.print("Magnetometer: x = ");
      Serial.print(sensors.magnetometer.x); 
      Serial.print(", y = ");
      Serial.print(sensors.magnetometer.y); 
      Serial.print(", z = ");
      Serial.print(sensors.magnetometer.z); 
      Serial.println(" uTesla");

      client.print("Magnetometer: <span style=\"color: yellow;\">x = ");
      client.print(sensors.magnetometer.x); 
      client.print(", y = ");
      client.print(sensors.magnetometer.y); 
      client.print(", z = ");
      client.print(sensors.magnetometer.z); 
      client.println(" uTesla<span></BR >");
    }

    if (USING_VEML_LUX) {
      lux = veml.readLux(VEML_LUX_AUTO);

      Serial.println("Light level is ");
      Serial.println(lux);
      Serial.println(" Lux");

      client.println("Light level is <span style=\"color: yellow;\">");
      client.println(lux);
      client.println(" Lux</span></BR >");
    }

    client.println("</p>");
    client.println("</html>");
    client.flush();

    //  Give the web browser time to receive the data
    delay(10);

    Serial.println("Closing connection");

    //  Close the connection
    Serial.println();
    client.stop();
  }
}
