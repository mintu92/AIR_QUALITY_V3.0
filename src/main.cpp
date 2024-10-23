#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Arduino.h>
#include <credential.h>
#include <SensorReader.h>
#include <UARTSensorRead.h>
#include <AQICalculator.h>

SensorReader sensor(PB11, PB10);


void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16);}

//---------------------------------------------------------------------------- Global variable --------------------------------------------------------------------------------------

struct sensorData
{
  int16_t eCO2;
  int16_t eTVOC;
  int16_t ePM25;
  int16_t ePM10;
  int16_t eLUX;
  int16_t eMOT;
  float eTMP;
  float eHUM;
  float eO3; 
  float eMIC;

};
#define PACKET_SIZE sizeof(sensorData)
union LoRa_Packet
{
  sensorData sensor;
  byte LoRaPacketBytes[PACKET_SIZE];
};
LoRa_Packet levelinfo;

static osjob_t sendjob;
//..................................................... Pin mapping.................................................................
const lmic_pinmap lmic_pins = 
{
    .nss = PB12,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = PC14,
    .dio = {PA8,PB8,PB9},
};

const int led = PC13; 
unsigned long joinStartTime = 0; 
bool Joined_st = false;  
int ackRetries = 0; // Track the number of retransmissions
const int maxRetries = 3; // Set the maximum number of retransmission attempts

int temp = 0;
int      lux = 0,ns = 0,lv = 0,mx = 0,co2 = 0,o3 = 0,tvoc = 0,pm25 = 0,pm10 = 0,mot = 0, tm = 0 ; 
uint16_t    hum = 0,tmp = 0;  
uint8_t  buffer[25] = {0};
unsigned long lastResetTime = 0;
static uint8_t messageCounter = 0;

extern "C" char* sbrk(int i);

// ----------------------------------- check available memory  -----------------------------------------------

int freeMemory() 
{
    char top;
    return &top - reinterpret_cast<char*>(sbrk(0));
}

void checkMemoryUsage() 
{
    int freeMem = freeMemory();
    Serial.print("Free memory: ");
    Serial.println(freeMem);
}

// ---------------------------------------------------------------------------------------------------------------
// --------------------------------------- TICK time to Human readble time conversation --------------------------

void printTime(unsigned long ticks) 
{
    const unsigned long ticksPerSecond = 64516;  
    unsigned long totalSeconds = ticks / ticksPerSecond;
    unsigned int hours = totalSeconds / 3600;
    unsigned int minutes = (totalSeconds % 3600) / 60;
    unsigned int seconds = totalSeconds % 60;

    Serial.print("Uptime: ");
    Serial.print(hours);
    Serial.print(":");
    Serial.print(minutes);
    Serial.print(":");
    Serial.print(seconds);
    Serial.print(":");
}
// ---------------------------------------------------------------------------------------------------------------

void do_send(osjob_t* j) 
{
    Serial.print("LMIC Status: ");
    if (LMIC.opmode == OP_NONE) {
        Serial.println("No operation pending");
        return;
    }

    if (LMIC.opmode & OP_SCAN) {
        Serial.println("Scanning for a beacon");
    }
    if (LMIC.opmode & OP_TRACK) {
        Serial.println("Tracking a beacon");
    }
    if (LMIC.opmode & OP_JOINING) {
        Serial.println("Joining ongoing");
    }
    if (LMIC.opmode & OP_TXDATA) {
        Serial.println("Data transmission ongoing");
    }
    if (LMIC.opmode & OP_POLL) {
        Serial.println("Querying server for data");
    }
    if (LMIC.opmode & OP_REJOIN) {
        Serial.println("Rejoin ongoing");
    }
    if (LMIC.opmode & OP_SHUTDOWN) {
        Serial.println("Shutdown ongoing");
    }
    if (LMIC.opmode & OP_TXRXPEND){                                                             // Check if there is not a current TX/RX job running{
      Serial.println("OP_TXRXPEND, not sending");
    }
    else
    {
  if (sensor.readSensorData()) 
  {
    Serial.println("Sensor data:");
    sensor.printSensorData();

    levelinfo.sensor.eCO2     = sensor.getCO2();
    levelinfo.sensor.eO3      = sensor.getO3()*0.001;
    levelinfo.sensor.eTVOC    = sensor.getTVOC();
    levelinfo.sensor.ePM25    = sensor.getPM25();
    levelinfo.sensor.ePM10    = sensor.getPM10();
    levelinfo.sensor.eTMP     = sensor.getTemperature()*0.1;
    levelinfo.sensor.eHUM     = sensor.getHumidity();
    levelinfo.sensor.eLUX     = sensor.getLUX();
    levelinfo.sensor.eMIC     = sensor.getMicLevel();
    levelinfo.sensor.eMOT     = sensor.getMotion();

    levelinfo.LoRaPacketBytes[sizeof(levelinfo.LoRaPacketBytes) - 1] = messageCounter++;

    LMIC_setTxData2(1, levelinfo.LoRaPacketBytes, sizeof(levelinfo.LoRaPacketBytes), 1);
    Serial.println("Packet queued"); 
    Serial.print(F("Frame counter (uplink): "));
    Serial.println(LMIC.seqnoUp);
  } 
  else 
  {
    Serial.println("Failed to read sensor data");
  }
    buffer[sizeof(buffer)] = {0} ;                                        // Next TX is scheduled after TX_COMPLETE event.
    checkMemoryUsage();  
   //}
  }
}

void onEvent (ev_t ev) 
{
    unsigned long currentTime = os_getTime();
    printTime(currentTime);
    Serial.print(" ->>>  ");
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
            joinStartTime = millis();
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            LMIC_setLinkCheckMode(0);
            Joined_st = true;
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
            if (LMIC.txrxFlags & TXRX_ACK) {
                Serial.println(F("Received ACK"));
                ackRetries = 0; // Reset retry count on successful ACK
            } 
            else 
            {
                    Serial.println(F("Max retries reached, resetting LMIC..."));
                    ackRetries = 0; // Reset retry count
                    LMIC_reset();    // Reset LMIC to retry OTAA and start fresh
                    LMIC_startJoining(); // Re-initiate join process
                    NVIC_SystemReset(); // Reset the MCU if needed
            }
            if (LMIC.dataLen) 
            {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            txInterval = Joined_st ? TX_INTERVAL_LONG : TX_INTERVAL_INITIAL;
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(txInterval), do_send);            // Schedule next transmission
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            Serial.println(F("EV_RXCOMPLETE"));             // data received in ping slot
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.print(F("Unknown event: "));
            Serial.println(ev);
            break;
    }
}

void setup() 
{
  sensor.begin(9600);  
  Serial.begin(115200);

  

    Serial.println("Device name       : " + (String)devname);
    Serial.println("Devid             : " + (String)devid);
    Serial.println("Mode              : " + (String)devmode);
    Serial.println("Core              : " + (String)devcore);
    Serial.println("Firmware Version  : " + (String)devver);
    Serial.println("Manufacturer      : " + (String)mfg);
    Serial.println("Frequency         : " + (String)devfrq);
    Serial.println("Debug Mode        : " + (String)devdebug);
    Serial.println("Chip ID           : " + (String)chipid);

   for (int i =0; i<40; i++)
    {
    Serial.print(".");
    }
    Serial.println(" Device starting ........................................");
    delay(3000);
    pinMode(led, OUTPUT);
    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    SPI.setMOSI(PB15);       // Use SPI2 on the STM32F407ve board.
    SPI.setMISO(PB14);
    SPI.setSCLK(PB13);
    SPI.begin();

    os_init();
    LMIC_reset();           // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_startJoining();    // Start job (sending automatically starts OTAA too)
    LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);
    do_send(&sendjob);
    lastResetTime = millis();
    delay(2000);
}


void loop() 
{
     if (millis() - lastResetTime >= 8L * 60L * 60L * 1000L)
     {
        Serial.println("device cycle complete and now restarting...");
        NVIC_SystemReset();  // Restart the microcontroller
     }
     else if ((!Joined_st && millis() - joinStartTime >= JOIN_TIMEOUT) || (!LMIC.devaddr && (millis() - lastResetTime >= DEVICE_CYCLE_TIMEOUT))) 
     {
        Serial.println("Join timeout, restarting the device... or Device cycle complete, restarting...");
        LMIC_reset();
        LMIC_startJoining(); // Restart joining process
        checkMemoryUsage();
        NVIC_SystemReset(); // Full system reset
     }
     else
     { 
        os_runloop_once();
     }
}

 