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


//Include the NTP library
#include <NTPClient.h>
WiFiUDP Udp; // A UDP instance to let us send and receive packets over UDP
NTPClient time_client(Udp);

#include "RTC.h"

int status = WL_IDLE_STATUS;
#include "Secrets.h" 

//  Please enter your sensitive data in the Secret tab/Secrets.h
char ssid[] = WIFI_SSID;        // your network SSID (name)
char pass[] = WIFI_PASSWD;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;            // your network key index number (needed only for WEP)

unsigned int localPort = 2390;      // local port to listen for UDP packets

IPAddress timeServer(162, 159, 200, 123); // pool.ntp.org NTP server

const int NTP_PACKET_SIZE = 48; // NTP timestamp is in the first 48 bytes of the message

byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets

// A UDP instance to let us send and receive packets over UDP
WiFiUDP Udp;

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

void printWifiStatus() {
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

unsigned long timestamp(unsigned long high, unsigned long low) {
  return high << 16 | low;
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
  printWifiStatus();

  Serial.println("Starting connection to the NTP server...");
  Udp.begin(localPort);
}

void loop(void) {
  // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
  const unsigned long seventyYears = 2208988800UL;

  unsigned long highWord, lowWord, epoch, secsSince1900;

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
    highWord = word(packetBuffer[40], packetBuffer[41]);
    lowWord = word(packetBuffer[42], packetBuffer[43]);

    //  Combine the four bytes (two words) into a long integer
    //  This is NTP time (seconds since Jan 1 1900):
    //secsSince1900 = highWord << 16 | lowWord;
    secsSince1900 = timestamp(highWord, lowWord);
    Serial.print("Seconds since Jan 1, 1900 = ");
    Serial.println(secsSince1900);

    //  Now convert NTP time into everyday time:
    Serial.print("Unix time = ");
    // subtract seventy years:
    epoch = secsSince1900 - seventyYears;
    //  Print Unix time:
    Serial.println(epoch);

    //  Print the hour, minute and second:
    Serial.print("UTC time is ");       // UTC is the time at Greenwich Meridian (GMT)
    Serial.print((epoch  % 86400L) / 3600); // print the hour (86400 equals secs per day)
    Serial.print(':');

    if (((epoch % 3600) / 60) < 10) {
      // In the first 10 minutes of each hour, we'll want a leading '0'
      Serial.print('0');
    }

    Serial.print((epoch  % 3600) / 60); // print the minute (3600 equals secs per minute)
    Serial.print(':');

    if ((epoch % 60) < 10) {
      // In the first 10 seconds of each minute, we'll want a leading '0'
      Serial.print('0');
    }

    Serial.println(epoch % 60); // print the second
  }

  //  Wait ten seconds before asking for the time again
  delay(10000);
}
