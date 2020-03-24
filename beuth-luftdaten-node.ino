/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses ABP (Activation-by-personalisation), where a DevAddr and
 * Session keys are preconfigured (unlike OTAA, where a DevEUI and
 * application key is configured, while the DevAddr and session keys are
 * assigned/generated in the over-the-air-activation procedure).
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!
 *
 * To use this sketch, first register your application and device with
 * the things network, to set or generate a DevAddr, NwkSKey and
 * AppSKey. Each device should have their own unique values for these
 * fields.
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/
//Used MC: Heltec ESP 32 LoRa v1

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
//Libraries for the used sensors
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SDS011.h> 
// has been modified from original library for use with newer SDS011 and is supplied in the repo

// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
//MSB first
static const PROGMEM u1_t NWKSKEY[16] = {  };

// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
//MSB first
static const u1_t PROGMEM APPSKEY[16] = {  };

//MSB first
// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x ; // <-- Change this address for every node!

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static uint8_t mydata[20] =  { 0 }; //Payload Buffer
//PAYLOAD: 4B TEMP, 4B PRESS, 4B HUM, 2B PM25, 2B PM10, 4B SDS_ID
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 130;
//Airtime for 20B = 56.576ms | 30000ms/56.576ms = 530 frames per day
//Send frequency = 60s*60m*24h/449 = 163s between each packet

// Pin mapping of the LoRa Chip for Heltec LoRa 32
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 33, 32},
};

//Definitions for the BME 280 temperature, pressure & humidity Sensor
Adafruit_BME280 bme;
//SDA Pin 21
//SCL Pin 22
float temperature, pressure,  humidity;

//Definitions for the particulate matter sensor SDS011
float p10sum, p25sum, p10f, p25f;
int p10, p25;
int err;
SDS011 my_sds;
#ifdef ESP32
HardwareSerial port(2);
#endif
//RX PIN 16
//TX PIN 17
int count = 0; 
unsigned long SDS_ID = 0;
byte sensorid[2] = {0xe1, 0x71};
String SDS_ID_s = "";

union intAsBytes //for easier access int to float
{
    int i;
    byte b[sizeof(int)];
}intAsBytes;

//Code from the example, evaluation of LoRa Events
void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            Serial.println("==========================================================");
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}
//Sending and preparing the payload
void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        readSensors(); //Evaluate the sensors
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    //Convert SDSID to Luftdaten.info Specs
    SDS_ID = int(sensorid[0]);                
    SDS_ID += (int(sensorid[1]) << 8);
    SDS_ID_s = Float2String(SDS_ID);    
    int dezPoint = SDS_ID_s.indexOf('.');
    SDS_ID_s = SDS_ID_s.substring(0, dezPoint);                          
    SDS_ID = int(SDS_ID);
    mydata[16]  = byte(SDS_ID);
    mydata[17]  = byte(SDS_ID >>  8);
    mydata[18]  = byte(SDS_ID >> 16);
    mydata[19]  = byte(SDS_ID >> 24);
  
    SPI.begin(5, 19, 27); //SPI Pins for LoRa Module
    Serial.begin(115200);
    Serial.println("Starting");

    my_sds.begin(&port); //SDS011 sensor initilization

    //Check for connection of BME280 sensor
    bool status;
    status = bme.begin(0x76);  
    if (!status) {
      Serial.println("Could not find a valid BME280 sensor, check wiring!");
      while (1);
    }

    //Settings for the BME280 as a weather station
    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF);
 
    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    // NA-US channels 0-71 are configured automatically
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
    #elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);

    // Start job
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}

void readSensors(){ //Evaluate the sensors
  Serial.print(os_getTime());
  Serial.println(": Reading Sensors(30s)");
  
  my_sds.wakeup();//Wakeup SDS from sleep
  delay(30000);//Wait to put enough air trough the sensor single measurement
  
  //BME 280 Measurement
  bme.takeForcedMeasurement();
  temperature = bme.readTemperature();
  pressure = (bme.readPressure()/100);
  humidity = bme.readHumidity();
  
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" *C");
  
  Serial.print("Pressure: ");
  Serial.print(pressure);
  Serial.println(" hPa");

  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");

  intAsBytes.i = int(temperature*100);//times 100 and rounding for no decimals
  mydata[0] = intAsBytes.b[3];//for easier decoding in the TTN JS encoder(problem: JS has no types)
  mydata[1] = intAsBytes.b[2];
  mydata[2] = intAsBytes.b[1];
  mydata[3] = intAsBytes.b[0];

  intAsBytes.i = int(pressure);
  mydata[4] = intAsBytes.b[3];
  mydata[5] = intAsBytes.b[2];
  mydata[6] = intAsBytes.b[1];
  mydata[7] = intAsBytes.b[0];

  intAsBytes.i = int(humidity*100);
  mydata[8] = intAsBytes.b[3];
  mydata[9] = intAsBytes.b[2];
  mydata[10] = intAsBytes.b[1];
  mydata[11] = intAsBytes.b[0];

  //SDS011 Measurement
  /*
  //Single Measurement
  my_sds.read(&p25f, &p10f);
  Serial.println("P2.5: " + String(p25f) + "µg/m³");
  Serial.println("P10:  " + String(p10f) + "µg/m³");
  p25 = int(p25*10);
  p10 = int(p10*10);
  */
  
  //SDS011 average of 5 measurements
  while(count < 5){
    err = my_sds.read(&p25f, &p10f);
    if (!err) {
      Serial.println(count);
      Serial.println("P2.5: " + String(p25f));
      Serial.println("P10:  " + String(p10f));
      p25sum += p25f;
      p10sum += p10f;
      count++;
    }
    
    if(count == 5){
      p25sum = p25sum/5;
      p10sum = p10sum/5;
      Serial.println("P2.5_Mittelwert: " + String(p25sum) + "µg/m³");
      Serial.println("P10_Mittelwert:  " + String(p10sum) + "µg/m³"); 
      p25 = round(p25sum*10);
      p10 = round(p10sum*10);
      break;
    }
  }
  count = 0;
  
  intAsBytes.i = p25;
  mydata[12] = intAsBytes.b[1];
  mydata[13] = intAsBytes.b[0];

  intAsBytes.i = p10;
  mydata[14] = intAsBytes.b[1];
  mydata[15] = intAsBytes.b[0];
  my_sds.sleep();//Sending the SDS back to sleep for longetivity of the device
  
  Serial.print("Payload: ");//Printing the Payload for debugging
  for(int i = 1; i<21; i++){
    Serial.printf("%02X ", mydata[i-1]);
    if(i % 4 == 0){
      Serial.print(" ");
    }
  }
  Serial.println();
  
}


String Float2String(const float value) {
  // Convert a float to String with two decimals.
  char temp[15];
  String s;

  dtostrf(value,13, 1, temp);
  s = String(temp);
  s.trim();
  return s;
}
