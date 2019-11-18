// include default platform libraries
#include <Arduino.h>
#include <Wire.h>

// include IBM LMIC LoRaWAN stack
#include <lmic.h>
#include <hal/hal.h>

// include the display library
#include <U8x8lib.h>
 
// include the BME280 sensor libraries
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// include CayenneLPP library
#include <CayenneLPP.h>

// define the I2C pins for TTGO v2.1 - T3_v1.6
#define I2C_SDA 21              // GPIO 21
#define I2C_SCL 22              // GPIO 22

// define the GPIOs for the display
// NOTE: this will cause warning during the compilation, because 
//       we overwrite the Arduino settings for this platform here
#define OLED_SDA I2C_SDA        // GPIO 21
#define OLED_SCL I2C_SCL        // GPIO 22
#define OLED_RST U8X8_PIN_NONE  // none

// LoRaWAN NwkSKey, network session key
// generated: { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
static const PROGMEM u1_t NWKSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// LoRaWAN AppSKey, application session key
// generated: { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
static const u1_t PROGMEM APPSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x00FF00FF;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

// Pin mapping for LoRaWAN
const lmic_pinmap lmic_pins = 
{
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 19,
  .dio = {/*dio0*/ 26, /*dio1*/ 33, /*dio2*/ 32}
};

// function declarations
void do_send(osjob_t* j);
void buildDataContainer();
void readSensorData();
void displaySensorData();

// These callbacks are only used in over-the-air activation (OTAA), so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

// Job function, which is periodically called.
static osjob_t sendjob;

Adafruit_BME280 bme280;
// variables to hold the data from BME280
float temperature;
float pressure;
float humidity;

// create CayenneLPP data container - 51 bytes max. payload
CayenneLPP lpp(51);

// define the display type that we use
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* clock=*/ OLED_SCL, /* data=*/ OLED_SDA, /* rst */ OLED_RST);

void setup() 
{
  // set up serial port for debugging information --> 115200bps
  Serial.begin(115200);
  Serial.println("TTGO LoRa32 v2.1 - BME280 weather station started");

  // initialize I2C
  // Wire.begin(I2C_SDA, I2C_SCL);

  // initialize BME280 sensor
  // start communication with the BME280 on address 0x76
  int status = bme280.begin(0x76);
  if (!status) 
  {
      Serial.println("Could not find a valid BME280 sensor, check wiring!");
      while (1);
  }

  // set up the display
  u8x8.begin(); 
  u8x8.setPowerSave(0);
  
  // use a font that is suitable for the small display
  // more fonts can be found here: https://github.com/olikraus/u8g2/wiki/fntlistall
  u8x8.setFont(u8x8_font_8x13_1x2_f);

  // set up LoRaWAN
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
  LMIC_setDrTxpow(DR_SF10,27);

  // Start job
  do_send(&sendjob);
}

void loop() 
{
  os_runloop_once();
}

/*
 * Prepare sensordata for LoRaWAN transmission
 */
void buildDataContainer() 
{
  lpp.reset();
  lpp.addTemperature(1, temperature);     // data 2 bytes
  lpp.addBarometricPressure(2, pressure); // data 2 bytes
  lpp.addRelativeHumidity(3, humidity);   // data 1 byte
}

/*
 * Read sensor data
 */
void readSensorData() 
{
  temperature = bme280.readTemperature();
  pressure = bme280.readPressure() / 100.0F;
  humidity = bme280.readHumidity();
}

/*
 * Show sensor data on the display
 */
void displaySensorData() 
{
  u8x8.clearDisplay();
  u8x8.setCursor(0, 0);
  
  u8x8.print("Temp: ");
  u8x8.print(temperature);
  u8x8.println("°C");

  u8x8.print("Pre:  ");
  u8x8.print(pressure);
  u8x8.println("hPa");

  u8x8.print("Hum:  ");
  u8x8.print(humidity);
  u8x8.println("%");
}

// LoRaWAN event handling
void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch(ev) 
  {
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
      if (LMIC.txrxFlags & TXRX_ACK)
      {
        Serial.println(F("Received ack"));
      }

      if (LMIC.dataLen) 
      {
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

// send the prepared data packet via LoRaWAN
void do_send(osjob_t* j) 
{
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) 
  {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } 
  else 
  {
    // prepare data for sending
    readSensorData();
    displaySensorData();
    buildDataContainer();
    
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
    Serial.println(F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}