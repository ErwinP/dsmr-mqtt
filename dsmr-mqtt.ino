// -*- mode: c++ -*-

#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include "dsmr-be.h"  //https://github.com/mrWheel/arduino-dsmr-be
#include "credentials.h"

/**
 * The next three defines are used to define the constatnts used
 * in for inverting the ESP8266's hardware UART. The register
 * base address, the UART control register and the invert bit 
 * are defined.
 */
// See: https://github.com/esp8266/Arduino/issues/4896
#define ESP8266_REG(addr) *((volatile uint32_t *)(0x60000000+(addr)))
#define U0C0        ESP8266_REG(0x020) //CONF0
#define UCRXI       19 //Invert RX

#define LED_RED     0
#define LED_BLUE    2

#define P1_DATA_REQ 4
#define BAUD_RATE   115200
#define OFF         HIGH
#define ON          LOW

#ifndef STASSID
#define STASSID     "XXXX"
#define STAPSK      "XXXX"
#endif

#ifndef MQTT_USER
#define MQTT_USER   "YYYY"
#define MQTT_PASSWD "YYYY"ds
#define MQTT_SERVER "FQDN or IP address"
#define MQTT_PORT   8883
#define MQTT_FP     "F7 ... 31"
#endif
#define CLIENT_ID   "dsmr"
#define INTERVAL    5000
#define MQTT_BUFSIZE 1450

// ============================ Wifi ============================

const char* ssid = STASSID;
const char* password = STAPSK;

// Initialise the WiFi Client object
WiFiClient wifiClient;

// ============================ TLS ============================

const char fingerprint[] PROGMEM = MQTT_FP;

// ============================ MQTT ============================

const char* mqtt_server = MQTT_SERVER;
const int mqtt_port = MQTT_PORT;
const char* mqtt_user = MQTT_USER;
const char* mqtt_password = MQTT_PASSWD;
const char* client_id = CLIENT_ID;
const char* dsmr_topic = "dsmr";
const char* dsmr_debug = "dsmr/debug";

// Initialise the MQTT Client object
PubSubClient pubsubClient(wifiClient); 

// For (re)connecting, a non-blocking wait is used.
unsigned long previousMillis = 0;
unsigned long interval = INTERVAL; // 5 seconds

// ============================ DSMR ============================

/**
 * Define the data we're interested in, as well as the datastructure to
 * hold the parsed data. This list shows all supported fields, remove
 * any fields you are not using from the below list to make the parsing
 * code smaller.
 * Each template argument below results in a field of the same name.
 */

// First define all data, used for debugging.
using MyData = ParsedData<
  /* String */ identification,
  /* String */ p1_version,
  /* String */ timestamp,
  /* String */ equipment_id,
  /* FixedValue */ energy_delivered_tariff1,
  /* FixedValue */ energy_delivered_tariff2,
  /* FixedValue */ energy_returned_tariff1,
  /* FixedValue */ energy_returned_tariff2,
  /* String */ electricity_tariff,
  /* FixedValue */ power_delivered,
  /* FixedValue */ power_returned,
  /* FixedValue */ electricity_threshold,
  /* uint8_t */ electricity_switch_position,
  /* uint32_t */ electricity_failures,
  /* uint32_t */ electricity_long_failures,
  /* String */ electricity_failure_log,
  /* uint32_t */ electricity_sags_l1,
  /* uint32_t */ electricity_sags_l2,
  /* uint32_t */ electricity_sags_l3,
  /* uint32_t */ electricity_swells_l1,
  /* uint32_t */ electricity_swells_l2,
  /* uint32_t */ electricity_swells_l3,
  /* String */ message_short,
  /* String */ message_long,
  /* FixedValue */ voltage_l1,
  /* FixedValue */ voltage_l2,
  /* FixedValue */ voltage_l3,
  /* FixedValue */ current_l1,
  /* FixedValue */ current_l2,
  /* FixedValue */ current_l3,
  /* FixedValue */ power_delivered_l1,
  /* FixedValue */ power_delivered_l2,
  /* FixedValue */ power_delivered_l3,
  /* FixedValue */ power_returned_l1,
  /* FixedValue */ power_returned_l2,
  /* FixedValue */ power_returned_l3,
  /* uint16_t */ gas_device_type,
  /* String */ gas_equipment_id,
  /* uint8_t */ gas_valve_position,
  /* TimestampedFixedValue */ gas_delivered,
  /* uint16_t */ thermal_device_type,
  /* String */ thermal_equipment_id,
  /* uint8_t */ thermal_valve_position,
  /* TimestampedFixedValue */ thermal_delivered,
  /* uint16_t */ water_device_type,
  /* String */ water_equipment_id,
  /* uint8_t */ water_valve_position,
  /* TimestampedFixedValue */ water_delivered,
  /* uint16_t */ slave_device_type,
  /* String */ slave_equipment_id,
  /* uint8_t */ slave_valve_position,
  /* TimestampedFixedValue */ slave_delivered
>;

using DSMRData = ParsedData<
//  /* String */ identification,
//  /* String */ p1_version,
//  /* String */ timestamp,
//  /* String */ equipment_id,
  /* FixedValue */ energy_delivered_tariff1,
  /* FixedValue */ energy_delivered_tariff2,
//  /* FixedValue */ energy_returned_tariff1,
//  /* FixedValue */ energy_returned_tariff2,
  /* String */ electricity_tariff,
  /* FixedValue */ power_delivered,
//  /* FixedValue */ power_returned,
//  /* FixedValue */ electricity_threshold,
//  /* uint8_t */ electricity_switch_position,
  /* uint32_t */ electricity_failures,
  /* uint32_t */ electricity_long_failures,
//  /* String */ electricity_failure_log,
//  /* uint32_t */ electricity_sags_l1,
//  /* uint32_t */ electricity_sags_l2,
//  /* uint32_t */ electricity_sags_l3,
//  /* uint32_t */ electricity_swells_l1,
//  /* uint32_t */ electricity_swells_l2,
//  /* uint32_t */ electricity_swells_l3,
//  /* String */ message_short,
//  /* String */ message_long,
  /* FixedValue */ voltage_l1,
  /* FixedValue */ voltage_l2,
  /* FixedValue */ voltage_l3,
  /* uint16_t */ current_l1,
  /* uint16_t */ current_l2,
  /* uint16_t */ current_l3,
//  /* FixedValue */ power_delivered_l1,
//  /* FixedValue */ power_delivered_l2,
//  /* FixedValue */ power_delivered_l3
//  /* FixedValue */ power_returned_l1,
//  /* FixedValue */ power_returned_l2,
//  /* FixedValue */ power_returned_l3,
//  /* uint16_t */ gas_device_type,
//  /* String */ gas_equipment_id,
//  /* uint8_t */ gas_valve_position,
  /* TimestampedFixedValue */ gas_delivered
//  /* uint16_t */ thermal_device_type,
//  /* String */ thermal_equipment_id,
//  /* uint8_t */ thermal_valve_position,
//  /* TimestampedFixedValue */ thermal_delivered,
//  /* uint16_t */ water_device_type,
//  /* String */ water_equipment_id,
//  /* uint8_t */ water_valve_position,
//  /* TimestampedFixedValue */ water_delivered,
//  /* uint16_t */ slave_device_type,
//  /* String */ slave_equipment_id,
//  /* uint8_t */ slave_valve_position,
//  /* TimestampedFixedValue */ slave_delivered
>;

/**
 * This illustrates looping over all parsed fields using the
 * ParsedData::applyEach method.
 *
 * When passed an instance of this Printer object, applyEach will loop
 * over each field and call Printer::apply, passing a reference to each
 * field in turn. This passes the actual field object, not the field
 * value, so each call to Printer::apply will have a differently typed
 * parameter.
 *
 * For this reason, Printer::apply is a template, resulting in one
 * distinct apply method for each field used. This allows looking up
 * things like Item::name, which is different for every field type,
 * without having to resort to virtual method calls (which result in
 * extra storage usage). The tradeoff is here that there is more code
 * generated (but due to compiler inlining, it's pretty much the same as
 * if you just manually printed all field names and values (with no
 * cost at all if you don't use the Printer).
 */
struct Printer {
  template<typename Item>
  void apply(Item &i) {
    if (i.present()) {
      Serial.print(Item::name);
      Serial.print(F(": "));
      Serial.print(i.val());
      Serial.print(Item::unit());
      Serial.println();
    }
  }
};

P1Reader reader(&Serial, P1_DATA_REQ);
DSMRData data;

// ============================ Setup ============================

void setup() {
  
  // setup LEDs
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(P1_DATA_REQ, OUTPUT);
  digitalWrite(LED_RED, OFF);
  digitalWrite(LED_BLUE, ON);
  digitalWrite(P1_DATA_REQ, LOW);
  
  // setup Serial. Invert receive (RX)
  Serial.begin(BAUD_RATE, SERIAL_8N1);
  U0C0 |= BIT(UCRXI); // Inverse RX

  // Say hello to the world
  Serial.println("Well hello there");
  Serial.println();
  Serial.println("--> Following parameters are set: ");
  Serial.println("Set SSID:");
  Serial.println(STASSID);
  Serial.println("Set MQTT server:");
  Serial.println(MQTT_SERVER);
  Serial.println("Set MQTT port:");
  Serial.println(MQTT_PORT);
  Serial.println("Set MQTT user:");
  Serial.println(MQTT_USER);
  Serial.println("<-- End");
  Serial.println();

  // setup the 
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");

  }

  Serial.println("");
  Serial.println("WiFi connected!");  
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Netmask: ");
  Serial.println(WiFi.subnetMask());
  Serial.print("Gateway: ");
  Serial.println(WiFi.gatewayIP());
  

  // Setup TLS fingerprint
  // wifiClient.setFingerprint(fingerprint);
  // wifiClient.setInsecure();

  // Set MQTT broker, port to use and the internal send/receive buffer size
  Serial.println();
  Serial.println();
  Serial.println("Setting up MQTT...");
  pubsubClient.setServer(mqtt_server, mqtt_port);
  pubsubClient.setBufferSize(MQTT_BUFSIZE);
  Serial.println("   ... done");
  
  // setup the OTA
  Serial.println();
  Serial.println();
  Serial.println("Setting up OTA...");
  
  ArduinoOTA.onStart([]() {
    digitalWrite(LED_BLUE, ON);
    digitalWrite(LED_RED, OFF);
    reader.disable();
    pubsubClient.disconnect();
  });
  ArduinoOTA.onEnd([]() {
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
  });
  ArduinoOTA.onError([](ota_error_t error) {
    // Blink 3 times with red
    digitalWrite(LED_RED, ON);
    delay(500);
    digitalWrite(LED_RED, OFF);
    delay(250);
    digitalWrite(LED_RED, ON);
    delay(500);
    digitalWrite(LED_RED, OFF);
    delay(250);
    digitalWrite(LED_RED, ON);
    delay(500);
    digitalWrite(LED_RED, OFF);
  });
  ArduinoOTA.begin();

  Serial.println("   ... done");

  Serial.println("Entering the first test of the readout");
  Serial.println("   Opening connection to MQTT..");
  // start a read right away and report back.
  if (!pubsubClient.connected()) {
    reconnect();
  }
  Serial.println("   ... done");

  Serial.println("   Trying to do a read...");
  pubsubClient.publish(dsmr_debug, "Start of first read");
  reader.enable(true); // NOT with parameter once=false for continuous readings
  if (reader.available()) {
    pubsubClient.publish(dsmr_debug, "Reader seems to be available at the start.");
    Serial.println("   Reader seems to be available at the start.");
    MyData data;
    String err;
    if (reader.parse(&data, &err)) {
      // Parse succesful, print result
      data.applyEach(Printer());
      Serial.println("   Successful parse!");
      pubsubClient.publish(dsmr_debug, "Successful parse!");
    } else {
      // Parser error, print error
      Serial.println("   here was an error during parsing!");
      pubsubClient.publish(dsmr_debug, "There was an error during parsing!");
    }
  } else {
    Serial.println("   Reader not available from the start.");
    pubsubClient.publish(dsmr_debug, "Reader not available from the start.");
  }
  Serial.println("   ... done");
  

//  digitalWrite(LED_RED, ON);
  digitalWrite(LED_BLUE, OFF);

  Serial.println();
  Serial.println();
  Serial.println("Finished setup, entering the DSMRData loop..");
}

// ============================ Loop ============================


  
void loop() {

  DSMRData data;
  String err;
  unsigned long currentMillis;

  // Make sure the MQTT conncetion stays alive
  // keepalive();
    
  ArduinoOTA.handle();

  if (!pubsubClient.connected()) {

    Serial.println("Not connected to the MQTT broker?");
    
    digitalWrite(LED_RED, OFF);
    currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      reconnect();
      previousMillis = currentMillis;
    }
  } else {
    digitalWrite(LED_RED, ON);
    // Allow mqtt and dsmr reader to update
    pubsubClient.loop();
    reader.loop();
    if (reader.available()) {
      Serial.println("DSMR Reader available!");
      digitalWrite(LED_BLUE, ON);
      if (reader.parse(&data, &err)) {
        Serial.println("Going public!");
        publish(data);
      } else {
        pubsubClient.publish(dsmr_debug, "Reader not parsing?");
        delay(5000); // Wait 5 sec
      }      
      digitalWrite(LED_BLUE, OFF);
    } else {
      pubsubClient.publish(dsmr_debug, "Reader not available, sleeping for 5 min.");
      delay(300000); // Wait 5 min
    }
  }
}

void reconnect() {
  
  // Attempt to connect to the mqtt broker
    if (pubsubClient.connect(client_id, mqtt_user, mqtt_password)) {
      // Once connected, publish an announcement...
      Serial.println("Sending first connection to the MQTT broker...");
      pubsubClient.publish(dsmr_topic, "CONNECTED");
      Serial.println("   ... done");
    }
    
    if (!pubsubClient.connected()) {
      // If still not connected, than wait 1 sec and retry
      delay(1000) ;
    }
    
}

void keepalive() {

  if (pubsubClient.connected()) {
     pubsubClient.publish(dsmr_debug, "(keeping connection alive)");
  } else {

    while (!pubsubClient.connected()) {
      
      if (pubsubClient.connect(client_id, mqtt_user, mqtt_password)) {
        // Once connected, publish an announcement...
        Serial.println("Sending first connection to the MQTT broker...");
        pubsubClient.publish(dsmr_topic, "CONNECTED");
        Serial.println("   ... done");
      }
      
      if (!pubsubClient.connected()) { // If still not connected, than wait 1 sec and retry
        delay(1000);
      }
    }
  }    
}

void publish(DSMRData data) {

  char buf[128];
  
  /* FixedValue */
  pubsubClient.publish("dsmr/energy_delivered_tariff1", itoa(data.energy_delivered_tariff1.int_val(), buf, 10));
  /* FixedValue */
  pubsubClient.publish("dsmr/energy_delivered_tariff2", itoa(data.energy_delivered_tariff2.int_val(), buf, 10));
  /* String */
  pubsubClient.publish("dsmr/electricity_tariff", data.electricity_tariff.c_str());
  /* FixedValue */
  pubsubClient.publish("dsmr/power_delivered", itoa(data.power_delivered.int_val(), buf, 10));
  /* uint32_t */
  pubsubClient.publish("dsmr/electricity_failures", itoa(data.electricity_failures, buf, 10));
  /* uint32_t */
  pubsubClient.publish("dsmr/electricity_long_failures", itoa(data.electricity_long_failures, buf, 10));
  /* FixedValue */
  pubsubClient.publish("dsmr/voltage_l1", itoa(data.voltage_l1.int_val(), buf, 10));
  /* FixedValue */
  pubsubClient.publish("dsmr/voltage_l2", itoa(data.voltage_l2.int_val(), buf, 10));
  /* FixedValue */
  pubsubClient.publish("dsmr/voltage_l3", itoa(data.voltage_l3.int_val(), buf, 10));
  /* uint16_t */
  pubsubClient.publish("dsmr/current_l1", itoa(data.current_l1, buf, 10));
  /* uint16_t */
  pubsubClient.publish("dsmr/current_l2", itoa(data.current_l2, buf, 10));
  /* uint16_t */
  pubsubClient.publish("dsmr/current_l3", itoa(data.current_l3, buf, 10));
  /* TimestampedFixedValue */
  pubsubClient.publish("dsmr/gas_delivered", itoa(data.gas_delivered.int_val(), buf, 10));
  // pubsubClient.publish("dsmr/identification", data.identification.c_str());
  // pubsubClient.publish("dsmr/power_delivered", itoa(data.power_delivered.int_val().int_val(), buf, 10));
}
