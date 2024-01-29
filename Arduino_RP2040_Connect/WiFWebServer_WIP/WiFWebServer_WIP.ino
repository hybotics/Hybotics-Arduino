/*
 Udp NTP Client

 Get the time from a Network Time Protocol (NTP) time server
 Demonstrates use of UDP sendPacket and ReceivePacket
 For more on NTP time servers and the messages needed to communicate with them,
 see http://en.wikipedia.org/wiki/Network_Time_Protocol

 created 4 Sep 2010
 by Michael Margolis
 modified 9 Apr 2012
 by Tom Igoe

 This code is in the public domain.

*/

#include <SPI.h>
#include <WiFiNINA.h>

#include <WiFiUdp.h>
// A UDP instance to let us send and receive packets over UDP
WiFiUDP Udp; // A UDP instance to let us send and receive packets over UDP

int status = WL_IDLE_STATUS;
#include  "Secrets.h" 
#include  "WiFiUpdNtpClient.h"

#define UTC_OFFSET_HRS  -8

//  Please enter your sensitive data in the Secret tab/Secrets.h
char ssid[] = WIFI_SSID;        // your network SSID (name)
char pass[] = WIFI_PASSWD;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;            // your network key index number (needed only for WEP)

unsigned int localPort = 2390;      // local port to listen for UDP packets

IPAddress timeServer(162, 159, 200, 123); // pool.ntp.org NTP server

const int NTP_PACKET_SIZE = 48; // NTP timestamp is in the first 48 bytes of the message

byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets

WiFiServer server(80);

//  send an NTP request to the time server at the given address
unsigned long send_ntp_packet(IPAddress& address) {
  //Serial.println("1");
  //  Set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  //  Initialize values needed to form NTP request
  //    (see URL above for details on the packets)
  //Serial.println("2");
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  //Serial.println("3");

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  //Serial.println("4");
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  //Serial.println("5");
  Udp.endPacket();
  //Serial.println("6");
}

void print_wifi_status() {
  //  Print the SSID of the network you're attached to:
  Serial.print("Network: ");
  Serial.println(WiFi.SSID());

  //  Print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("Signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

unsigned long ntp_time(unsigned long high, unsigned long low) {
  return high << 16 | low;
}

CurrentTime get_time(byte *pbuff, int16_t utc_offset_hrs) {
  // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:

  const unsigned long seventy_years = 2208988800UL;
  CurrentTime curr_time;
  String stamp;
  unsigned long highWord, lowWord, local_time, unix_time, secs_since_1900;

  highWord = word(pbuff[40], pbuff[41]);
  lowWord = word(pbuff[42], pbuff[43]);

  //  Combine the four bytes (two words) into a long integer
  //  This is NTP time (seconds since Jan 1 1900):
  //secsSince1900 = highWord << 16 | lowWord;
  secs_since_1900 = ntp_time(highWord, lowWord);

  //  Subtract seventy years:
  unix_time = (secs_since_1900 - seventy_years);

  //  Adjust time for local time zone
  local_time = unix_time + (utc_offset_hrs * 3600);

  curr_time.utc_unix_time = unix_time;
  curr_time.local_unix_time = local_time;

  curr_time.hours = (local_time % 86400L) / 3600;
  curr_time.minutes = (local_time  % 3600) / 60;
  curr_time.seconds = local_time % 60;

  /*
    Create the time stamp
  */
  stamp = String(curr_time.hours) + ":";

  if (curr_time.minutes < 10) {
    stamp = stamp + "0";
  }

  stamp = stamp + String(curr_time.minutes);
  stamp = stamp + ":";

  if (curr_time.seconds < 10) {
    stamp = stamp + "0";
  }

  stamp = stamp + String(curr_time.seconds);

  curr_time.time_stamp = stamp;

  return curr_time;
}

void setup(void) {
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ;// Wait for serial port to connect. Needed for native USB port only
  }

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    //  Do not continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to network '");
    Serial.print(ssid);
    Serial.println("'");

    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    //  Wait 1/2 second for connection:
    delay(500);
  }

  Serial.println("Connected to WiFi");
  print_wifi_status();

  Serial.println("Starting connection to the NTP server...");
  Udp.begin(localPort);

  server.begin();
}

void loop(void) {
  CurrentTime current_time;

  send_ntp_packet(timeServer); // send an NTP packet to a time server
  //  Wait to see if a reply is available
  delay(1000);

  if (Udp.parsePacket()) {
    Serial.println();
    Serial.println("Packet received");
    //  We've received a packet, read the data from it
    Udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

    //  The timestamp starts at byte 40 of the received packet and is four bytes,
    //    or two words, long. First, extract the two words:
    //highWord = word(packetBuffer[40], packetBuffer[41]);
    //lowWord = word(packetBuffer[42], packetBuffer[43]);
    current_time = get_time(packetBuffer, UTC_OFFSET_HRS);

    //  Print local Unix time:
    Serial.print("Unix time = ");
    Serial.println(current_time.local_unix_time);

    Serial.print("Current time is '");
    Serial.print(current_time.time_stamp);
    Serial.println("'");
  }

  //  Listen for incoming clients
  WiFiClient client = server.available();

  if (client) {
    Serial.println("New client");
    // an HTTP request ends with a blank line
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.write(c);
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the HTTP request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          // send a standard HTTP response header
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/html");
          client.println("Connection: close");  // the connection will be closed after completion of the response
          client.println("Refresh: 5");  // refresh the page automatically every 5 sec
          client.println();
          client.println("<!DOCTYPE HTML>");
          client.println("<html>");
          // output the value of each analog input pin
          for (int analogChannel = 0; analogChannel < 6; analogChannel++) {
            int sensorReading = analogRead(analogChannel);
            client.print("analog input ");
            client.print(analogChannel);
            client.print(" is ");
            client.print(sensorReading);
            client.println("<br />");
          }
          client.println("</html>");
          break;
        }
        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
        } else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }

    // close the connection:
    client.stop();
    Serial.println("client disconnected");
  }

  delay(10);
}
