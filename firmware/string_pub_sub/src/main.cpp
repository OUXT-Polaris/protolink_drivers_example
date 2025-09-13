#include <Arduino.h>
#include <NativeEthernetUdp.h>
#include <pb_decode.h>
#include <pb_encode.h>
#include <proto/std_msgs__String.pb.h>

using ProtoString = protolink__std_msgs__String_std_msgs__String;

constexpr unsigned int STRING_BUF_SIZE = 128;
constexpr unsigned int UDP_BUF_SIZE = 1024;

// Set Teensy IP and MAC address
byte MAC[] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
const IPAddress IP(192, 168, 0, 100);

// Set listen port
const unsigned int PORT = 8000;  // port to listen on

// Set send ip-address and port
const IPAddress DIST_IP(192, 168, 0, 50);
const unsigned int DIST_PORT = 9000;

EthernetUDP Udp;
String recive_str;

// Callback function to encode a string
bool encode_string_callback(pb_ostream_t * stream, const pb_field_t * field, void * const * arg)
{
  const String & str = *static_cast<const String *>(*arg);

  // Write the string to the stream
  if (!pb_encode_tag_for_field(stream, field)) return false;

  return pb_encode_string(stream, reinterpret_cast<const pb_byte_t *>(str.c_str()), str.length());
}

// Callback function to decode a string
bool decode_string_callback(pb_istream_t * stream, const pb_field_t * field, void ** arg)
{
  size_t len = stream->bytes_left;

  // Check length to prevent buffer overflow
  // Assumes the destination buffer size is 128 bytes.
  if (len > STRING_BUF_SIZE - 1) {
    Serial.printf("Error: String is too long. (%d > %d)\n", len, (STRING_BUF_SIZE - 1));
    return false;
  }

  // null check
  if (reinterpret_cast<String *>(*arg) == nullptr) {
    Serial.println("arg is null!");
    return false;
  }

  // Read String data as char
  char buffer[STRING_BUF_SIZE];
  if (!pb_read(stream, (pb_byte_t *)buffer, len)) return false;
  buffer[len] = '\0';

  **reinterpret_cast<String **>(arg) = buffer;
  return true;
}

void publish()
{
  ProtoString msg = protolink__std_msgs__String_std_msgs__String_init_zero;
  String str_arg = recive_str + " world";
  msg.data.funcs.encode = encode_string_callback;
  msg.data.arg = &str_arg;

  uint8_t packetBuffer[UDP_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(packetBuffer, sizeof(packetBuffer));
  bool result = pb_encode(&stream, protolink__std_msgs__String_std_msgs__String_fields, &msg);

  if (result) {
    Udp.beginPacket(DIST_IP, DIST_PORT);
    Udp.write(packetBuffer, stream.bytes_written);
    Udp.endPacket();
    Serial.printf(
      "Publish data   --> data: %s, written data: %d\n\n", str_arg.c_str(), stream.bytes_written);
  } else {
    Serial.printf("Encode error\n");
  }
}

void subscribe()
{
  ProtoString msg = protolink__std_msgs__String_std_msgs__String_init_zero;
  String str_arg;
  msg.data.funcs.decode = decode_string_callback;
  msg.data.arg = &str_arg;

  uint8_t packetBuffer[UDP_BUF_SIZE];
  int num_bytes = Udp.read(packetBuffer, sizeof(packetBuffer));
  pb_istream_t stream = pb_istream_from_buffer(packetBuffer, num_bytes);
  bool result = pb_decode(&stream, protolink__std_msgs__String_std_msgs__String_fields, &msg);

  if (result) {
    recive_str = str_arg;
    Serial.printf("Subscribe data --> data: %s\n", str_arg.c_str());
  } else {
    Serial.printf("Decode error\n");
  }
}

void setup()
{
  Serial.begin(9600);

  Ethernet.begin(MAC, IP);
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println(
      "Ethernet shield was not found.  Sorry, can't run without "
      "hardware. :(");
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
