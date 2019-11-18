Hier die einzelnen Teile, die ihr im Laufe des Workshops in euer neu erstelltest
Projekt noch übernehmen müsst.

Einzubindende Bibliotheken:
===========================

```C
// include IBM LMIC LoRaWAN stack
#include <lmic.h>
#include <hal/hal.h>
```

Funktionsdeklarationen:
=======================

Definition der *"do_Send(...)"* Methode (wird hinzugefügt).

```C
// function declarations
void do_send(osjob_t* j);
```

Deklarationen:
==============

In diesem Teil werden die Keys zur Kommunikation festgelegt, wir verwenden hier ABP (Authentication By Personalization). Im Anschluß wird das Sendeintervall des Sendejobs definiert und die zur Kommunikation mit dem LoRa Transceiver auf dem Development Board notwendigen Pins definiert. Zum Schluß folgt die Definition für die OTAA Callbacks (werden bei ABP nicht verwendet, ein Weglassen würde aber zu Warnings während der Compilierung führen) und des Sendejobs, der periodisch ausgeführt werden soll.

```C
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

// These callbacks are only used in over-the-air activation (OTAA), so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

// Job function, which is periodically called.
static osjob_t sendjob;
```

Setup():
========

Im Setup-Teil wird der LMIC Stack erst initialisiert und dann auf einen definitierten Ausgangszustand gebracht. Anschließend werden die Keys in Abhängikeit eines vorhandenen Flashs in den Arbeitsspeicher des Mikrocontrollers kopiert und die Session im Stack mit den Keys initialisiert.

Ein weiterer wichtiger Teil ist die Konfiguration der zu verwendenden Frequenzen für LoRaWAN. In der EU wird die Konfiguration *"CFG_eu868"* verwendet, sodass die Kanäle und Übertragungsraten definiert werden.  
Mehr Informationen zu den verwendeten Frequenzen in den unterschiedlichen Teilen der Welt kann man auf den Seiten des [The Things Network](https://www.thethingsnetwork.org/docs/lorawan/frequency-plans.html) finden.

Danach wird noch der Spreading Factor für das RX2 Window eines Datenpaketes gesetzt (im The Things Network immer SF9) und der Spreading Factor für den Knoten gesetzt (hier SF10 - im Idealfall aber SF7).

Zum Abschluß wird der Job zum Senden der Daten zum ersten Mal in den Scheduler des LMIC Stacks eingereiht und ausgeführt.

```C
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
  LMIC_setDrTxpow(DR_SF10, 27);

  // Start job
  do_send(&sendjob);
```

Loop() (komplett ersetzen):
===========================

In der Loop wird nur noch die *"Run"* Methode des LMIC Stacks aufgerufen. Alles Weitere (Verwaltung der Threads, Einhaltung der Duty Cycle, Ansprechen der Hardware, etc.) wird vom Stack übernommen.

```C
void loop() 
{
  os_runloop_once();
}
```

Neue Methoden:
==============

Sobald der LMIC Stack ein Event aus dem LoRaWAN Netzwerk empfängt, wird die *"onEvent"* Methode aufgerufen. Hier kann auf Fehler reagiert werden (bspw. bei einer OTAA Session eine Neuanmeldung ausgeführt werden).

Wichtig ist hier der Fall *"TX_COMPLETE"*. Er wird aufgerufen, wenn die Daten erfolgreich gesendet werden konnten. In diesem Schritt werden außerdem die Daten aus dem RX Window des Gateways entgegengenommen (Daten, die von der Application an den Knoten gesendet wurden) und ___ganz wichtig___ der Sendejob wird wieder eingereiht.

```C
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
```

In der Sendemethode wird zuerst überprüft, ob evtl. noch ein Sendejob läuft. In diesem Fall wird der zu sendende Job verworfen. Sollte kein aktiver Job vorhanden sein, werden die Daten aufbereitet und dem LMIC Stack übergeben. Wenn die Duty Cycle eingehalten wurde, wird das Paket direkt gesendet, ansonsten wird es so lange verzögert, bis man wieder innerhalb der Grenzen senden kann.

```C
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
```