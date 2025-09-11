#include <Arduino.h>
#include <NativeEthernetUdp.h>
#include <pb_decode.h>
#include <pb_encode.h>
#include <proto/geometry_msgs__Vector3.pb.h>

using Vector3 = protolink__geometry_msgs__Vector3_geometry_msgs__Vector3;

// Set Teensy IP and MAC address
byte MAC[] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
const IPAddress IP(192, 168, 0, 100);

// Set listen port
const unsigned int PORT = 7000;  // port to listen on

// Set send ip-address and port
const IPAddress DIST_IP(192, 168, 0, 50);
const unsigned int DIST_PORT = 9000;

EthernetUDP Udp;
double x = 0;
double y = 0;
double z = 0;

void publish()
{
  Vector3 msg = protolink__geometry_msgs__Vector3_geometry_msgs__Vector3_init_zero;
  msg.x = ++x;
  msg.y = ++y;
  msg.z = ++z;
  msg.has_x = true;
  msg.has_y = true;
  msg.has_z = true;

  uint8_t packetBuffer[1024];
  pb_ostream_t stream = pb_ostream_from_buffer(packetBuffer, sizeof(packetBuffer));
  bool result =
    pb_encode(&stream, protolink__geometry_msgs__Vector3_geometry_msgs__Vector3_fields, &msg);

  if (result) {
    Udp.beginPacket(DIST_IP, DIST_PORT);
    Udp.write(packetBuffer, stream.bytes_written);
    Udp.endPacket();
    Serial.printf(
      "Publish data   --> x: %lf,  y: %lf,  z: %lf, written data: %d\n\n", msg.x, msg.y, msg.z,
      stream.bytes_written);
  } else {
    Serial.printf("Encode error\n\n");
  }
}

void subscribe()
{
  Vector3 msg = protolink__geometry_msgs__Vector3_geometry_msgs__Vector3_init_zero;
  uint8_t packetBuffer[1024];

  int num_bytes = Udp.read(packetBuffer, sizeof(packetBuffer));
  pb_istream_t stream = pb_istream_from_buffer(packetBuffer, num_bytes);
  bool result =
    pb_decode(&stream, protolink__geometry_msgs__Vector3_geometry_msgs__Vector3_fields, &msg);

  if (result) {
    x = msg.x;
    y = msg.y;
    z = msg.z;
    Serial.printf("Subscribe data --> x: %lf,  y: %lf,  z:%lf\n", msg.x, msg.y, msg.z);
  } else {
    Serial.printf("Decode error\n");
  }
}

void setup()
{
  Serial.begin(9600);

  Ethernet.begin(MAC, IP);
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    while (true) {
      delay(1);  // do nothing, no point running without Ethernet hardware
    }
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
  }

  // Start UDP
  Udp.begin(PORT);
}

void loop()
{
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    subscribe();
    publish();
  }

  delay(10);
}
