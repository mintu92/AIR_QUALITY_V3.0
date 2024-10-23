#include <stdint.h>  // For standard integer types
typedef uint8_t u1_t;

const char* devname   = "Smart AQI-Sensor";                                         // put device name registerd in your chirpstack device (D = Display eneable).
const char* devid     = "a4e5958273d72baa";                                         // put device EUI registerd in your chirpstack device.
const char* devmode   = "LORAWAN OTAA";                                             // Select lorawan mode (ABP / OTAA).
const char* devcore   = "STMF103C8T6 + Cortex-M3";                                  // Device core used.
const char* devver    = "V 1.0";                                                    // Firmware v1.0.
const char* mfg       = "MachineSens IoT LLC";                                      // Manufactureing company name.
const char* devfrq    = "EU-868";                                                   // Lorawan regional frequency band.
const char* devdebug  = "USART ";                                                   // Available debug mode.
const char* chipid    = "0x410";                                                    // Available MCU id.

//---------------------------------------------------------------- NODE DETAILS --------------------------------------------------------------------------

/*static const u1_t APPEUI[8]=   {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa0};
static const u1_t DEVEUI[8]=   {0xef, 0xc0, 0x13, 0x98, 0xf4, 0xf8, 0x02, 0xb0}; 
static const u1_t APPKEY[16] = {0xf9, 0x6f, 0xe9, 0xba, 0x03, 0xb6, 0x45, 0xec, 0x76, 0x58, 0xa4, 0x72, 0x01, 0xe2, 0x7e, 0x16};
*/
static const u1_t APPEUI[8]=   {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa0};
static const u1_t DEVEUI[8]=   {0x58, 0xcb, 0x24, 0x29, 0xca, 0xa1, 0x01, 0x7c}; 
static const u1_t APPKEY[16] = {0xCA, 0x30, 0x2F, 0x19, 0x9E, 0xAD, 0xC1, 0x8D, 0x3C, 0x30, 0x42, 0xA3, 0x1C, 0xB8, 0x1A, 0x46};

unsigned txInterval = 0;
const unsigned long TX_INTERVAL_INITIAL = 30;     // 30 seconds for initial join phase
const unsigned long TX_INTERVAL_LONG = (60); // 3 minutes for normal operation after join
const unsigned long JOIN_TIMEOUT = (3 * 60 * 1000);  
const unsigned long DEVICE_CYCLE_TIMEOUT = 5 * 60L * 1000L;  // 5-minute cycle timeout