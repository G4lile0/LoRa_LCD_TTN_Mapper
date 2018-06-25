/*******************************************************************************

   
Arduino LoRaWAN TTN node with LCD for mapping and testing different channels and Spreading Factors
by G4lile0 
   
 25-06-2018  first public release
  
   
   

   
   
// Button Long / Short Press script by: 
// (C) 2011 By P. Bauermeister
// http://www.instructables.com/id/Arduino-Dual-Function-Button-Long-PressShort-Press/

   
  
Based on the script from Thomas Telkamp and Matthijs Kooijman modified by jfmateos

https://github.com/jfmateos/thethingnetwork_madrid_taller_gateway_nodo_single_channel/tree/master/Nodo

   
   Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman

   Permission is hereby granted, free of charge, to anyone
   obtaining a copy of this document and accompanying files,
   to do whatever they want with them without any restriction,
   including, but not limited to, copying, modification and redistribution.
   NO WARRANTY OF ANY KIND IS PROVIDED.

   This example sends a valid LoRaWAN packet with payload "Hello,
   world!", using frequency and encryption settings matching those of
   the The Things Network.

   This uses ABP (Activation-by-personalisation), where a DevAddr and
   Session keys are preconfigured (unlike OTAA, where a DevEUI and
   application key is configured, while the DevAddr and session keys are
   assigned/generated in the over-the-air-activation procedure).

   Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
   g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
   violated by this sketch when left running for longer)!

   To use this sketch, first register your application and device with
   the things network, to set or generate a DevAddr, NwkSKey and
   AppSKey. Each device should have their own unique values for these
   fields.

   Do not forget to define the radio type correctly in config.h.




 *******************************************************************************/





#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <LowPower.h>
#include  "adcvcc.h"
#include <Arduino.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif


//U8G2_PCD8544_84X48_1_4W_SW_SPI u8g2(U8G2_R0, /* clock=*/ 17, /* data=*/ 3, /* cs=*/ 5, /* dc=*/ 4, /* reset=*/ 6);  // Nokia 5110 Display
U8G2_PCD8544_84X48_1_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 5, /* dc=*/ 4, /* reset=*/ 6);            // Nokia 5110 Display

// Adapt these to your board and application timings:

#define BUTTON1_PIN              14  // Button 1
#define BUTTON2_PIN              15  // Button 2
#define ALARM_PIN                16  // buzzer alarm (beep on LOW)             
#define DEFAULT_LONGPRESS_LEN    10  // Min nr of loops for a long press
#define DELAY                    40  // Delay per loop in ms
#define BACKLIGHT_PIN            3  // LED display alarm (beep on LOW)             



//static const PROGMEM int airtime[6] ={1300,741,370,185,102,57};
//static const PROGMEM int air_time[6] ={57 ,102, 185, 370, 741 , 1300 };

static int air_time[6] ={1300,741,370,185,102,57};


//  alarm (off = off; true = on)
boolean  ALARM        =  false;
boolean  BACKLIGHT    =  false;
boolean  MANUAL       =  true;
unsigned long    auto_timer;
unsigned long    last_packet;


//temporal to check speed of each menu and set the right delay. 
unsigned long time;


//////////////////////////////////////////////////////////////////////////////

enum { EV_NONE=0, EV_SHORTPRESS, EV_LONGPRESS };

//////////////////////////////////////////////////////////////////////////////
// Class definition

class ButtonHandler {
  public:
    // Constructor
    ButtonHandler(int pin, int longpress_len=DEFAULT_LONGPRESS_LEN);

    // Initialization done after construction, to permit static instances
    void init();

    // Handler, to be called in the loop()
    int handle();

  protected:
    boolean was_pressed;     // previous state
    int pressed_counter;     // press running duration
    const int pin;           // pin to which button is connected
    const int longpress_len; // longpress duration
};

ButtonHandler::ButtonHandler(int p, int lp)
: pin(p), longpress_len(lp)
{
}

void ButtonHandler::init()
{
  pinMode(pin, INPUT);
  digitalWrite(pin, HIGH); // pull-up
  was_pressed = false;
  pressed_counter = 0;
}

int ButtonHandler::handle()
{
  int event;
  int now_pressed = !digitalRead(pin);

  if (!now_pressed && was_pressed) {
    // handle release event
    if (pressed_counter < longpress_len)
      event = EV_SHORTPRESS;
    else
      event = EV_LONGPRESS;
  }
  else
    event = EV_NONE;

  // update press running duration
  if (now_pressed)
    ++pressed_counter;
  else
    pressed_counter = 0;

  // remember state, and we're done
  was_pressed = now_pressed;
  return event;
}

//////////////////////////////////////////////////////////////////////////////

// Instanciate button objects
ButtonHandler button1(BUTTON1_PIN);
ButtonHandler button2(BUTTON2_PIN);



#define NORMALINTERVAL 900 // 15 minutes (normal) 
int interval = NORMALINTERVAL;

byte LMIC_transmitted = 0;
int LMIC_event_Timeout = 0;
int LMIC_event_airtime = 0;

int bateria = 0;

// LoRaWAN NwkSKey, network session key (formato MSB)
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const PROGMEM u1_t NWKSKEY[16] = { 0x, 0x, 0x, 0x, 0x, 0x, 0x, 0x, 0x, 0x, 0x, 0x, 0x, 0x, 0x, 0x };

// LoRaWAN AppSKey, application session key (formato MSB)
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const u1_t PROGMEM APPSKEY[16] = { 0x, 0x, 0x, 0x, 0x, 0x, 0x, 0x, 0x, 0x, 0x, 0x, 0x, 0x, 0x, 0x };

// LoRaWAN end-device address (DevAddr) (formato MSB)
static const u4_t DEVADDR = 0x ; // <-- Change this address for every node!

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) {  }
void os_getDevEui (u1_t* buf) {  }
void os_getDevKey (u1_t* buf) {  }

static uint8_t mydata[] = "Hello, world!";
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 9,
  .dio = {2, 7, 8},
};




// Define the single channel and data rate (SF) to use
  int channel = 0;
//g4  int dr = DR_SF7;
  int dr = DR_SF7;


// Disables all channels, except for the one defined above, and sets the
// data rate (SF). This only affects uplinks; for downlinks the default
// channels or the configuration from the OTAA Join Accept are used.
//
// Not LoRaWAN compliant; FOR TESTING ONLY!
//
void forceTxSingleChannelDr() {
  for (int i = 0; i < 9; i++) { // For EU; for US use i<71
    if (i != channel) {
      LMIC_disableChannel(i);
    }
  }
  // Set data rate (SF) and transmit power for uplink
  LMIC_setDrTxpow(dr, 14);
}

/* ======================================================================
  Function: ADC_vect
  Purpose : IRQ Handler for ADC
  Input   : -
  Output  : -
  Comments: used for measuring 8 samples low power mode, ADC is then in
          free running mode for 8 samples
  ====================================================================== */
ISR(ADC_vect)
{
  // Increment ADC counter
  _adc_irq_cnt++;
}

void onEvent (ev_t ev) {

 //  auto timer , 10 seconds + 1% air time
    last_packet= os_getTime() ;
    auto_timer= last_packet + 10*OSTICKS_PER_SEC+( (air_time[dr])*OSTICKS_PER_SEC*101/1000);    
    
                                                                
//  Serial.println( LMIC.bcnRxsyms);
//  Serial.println( LMIC.bcnRxtime);
//  Serial.println( LMIC.freq);
//  Serial.println( LMIC.globalDutyAvail);
//  Serial.println( LMIC.txend);
// Serial.println( LMIC.bcninfo.txtime);
//  Serial.println( (LMIC.globalDutyAvail-os_getTime())/32768);

//  Serial.println("s: ");
 
  
//  Serial.print(os_getTime());
//  Serial.print(": ");
//  Serial.print(auto_timer);

//  Serial.println(": ");
  
  switch (ev) {
    case EV_SCAN_TIMEOUT:
//      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
//      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
//      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
//      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
//      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
//      Serial.println(F("EV_JOINED"));
      break;
    case EV_RFU1:
//      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
//      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
//      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
//      Serial.println(F("EV_TXCOMPLETE (+RX w)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Rec ack"));

      if (LMIC.dataLen) {
   //     Serial.println(F("Rec "));
   //     Serial.println(LMIC.dataLen);
   //     Serial.println(F("bytes"));
          u8g2.setDrawColor(1);
          u8g2.drawBox(0, 0, 82, 11);
          u8g2.setCursor(2,10);
          u8g2.setDrawColor(0);
          u8g2.print (F("Rec "));
          u8g2.print (LMIC.dataLen);
          u8g2.print (F("bytes"));
          delay(500);
        
      }

      /*
        if (LMIC.dataLen) { // data received in rx slot after tx
        fprintf(stdout, "Received %d bytes of payload: 0x", LMIC.dataLen);
        for (int i = 0; i < LMIC.dataLen; i++) {
          fprintf(stdout, "%02X", LMIC.frame[LMIC.dataBeg + i]);
        }
        fprintf(stdout, "\n");
        }
      */
      // Schedule next transmission
      //os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      LMIC_transmitted = 1;
      break;
    case EV_LOST_TSYNC:
//      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
//      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
//      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
//      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
//      Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
//      Serial.println(F("Unknown event"));
      break;
  }
}

/*
 * 

void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    int batt = (int)(readVcc() / 100);  // readVCC returns  mVolt need just 100mVolt steps
    byte batvalue = (byte)batt; // no problem putting it into a int.
    unsigned char carga_de_pago[4];
    carga_de_pago[0] = 0x03;
    carga_de_pago[1] = 0x02;
    int16_t help;
    help = batvalue * 10;
    carga_de_pago[2] = help >> 8;
    carga_de_pago[3] = help;

    LMIC_setTxData2(1, carga_de_pago, sizeof(carga_de_pago), 0);
    Serial.println(F("P queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

*/


void do_send_abierta(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
//    Serial.println(F("OP_TXRXPEND"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    int batt = (int)(readVcc() / 100);  // readVCC returns  mVolt need just 100mVolt steps
    byte batvalue = (byte)batt; // no problem putting it into a int.
    unsigned char carga_de_pago[7];
    carga_de_pago[0] = 0x03;
    carga_de_pago[1] = 0x02;//Entrada analógica
    int16_t help;
    help = batvalue * 10;
    carga_de_pago[2] = help >> 8;
    carga_de_pago[3] = help;
    carga_de_pago[4] = 0x04;
    carga_de_pago[5] = 0x00;//Entrada digital
    carga_de_pago[6] = 0x01;

    LMIC_setTxData2(1, carga_de_pago, sizeof(carga_de_pago), 0);
//    Serial.println(F("P q"));
  }
}



void puertaAbierta() {
  os_setCallback (&sendjob, do_send_abierta);
}



void beep() {

   if (ALARM==1)  {
         
      digitalWrite(ALARM_PIN, LOW);
      delay(10);
      digitalWrite(ALARM_PIN, HIGH);    
                  }
           }


void long_beep() {

   if (ALARM==1)  {
      digitalWrite(ALARM_PIN, LOW);
      delay(40);
      digitalWrite(ALARM_PIN, HIGH);    
                  }
           }


void init_LoRa() {
  
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
  //LMIC_setDrTxpow(DR_SF7,14);

  forceTxSingleChannelDr();
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
  
  
  }


void setup() {

  // init buttons pins; I suppose it's best to do here
  button1.init();
  button2.init();

  //temporal to check speed of each menu and set the right delay. 
  //time = millis();


  //intit buzzez pin and backlight
  pinMode(ALARM_PIN , OUTPUT);
  pinMode(BACKLIGHT_PIN , OUTPUT);
  digitalWrite(ALARM_PIN, LOW);
  digitalWrite(BACKLIGHT_PIN, LOW);
  
  delay(5);
  digitalWrite(ALARM_PIN, HIGH);
  digitalWrite(BACKLIGHT_PIN, HIGH);

    
  Serial.begin(115200);
//  Serial.println(F("Boot"));

//  showBootStatus(mcusr);
//  pinMode(3, INPUT_PULLUP);

  // LMIC init
  os_init();
  init_LoRa();


  u8g2.begin();  
  bateria = readVcc() / 100 ;

}

void loop() {


     u8g2.firstPage();
  do {
//     u8g2.setDrawColor(0);

    u8g2.setFont(u8g2_font_amstrad_cpc_extended_8r);
    u8g2.drawBox(0, 0, 82, 11);
    u8g2.setCursor(2,10);
    u8g2.setDrawColor(0);
    u8g2.print ("TTN Mapper");
    u8g2.setCursor(1,20);
    u8g2.setDrawColor(1);
    u8g2.print ("Ch:");
    u8g2.setCursor(27,20);
    u8g2.print (channel );
    u8g2.setCursor(41,20);
    u8g2.print ("SF:");
    u8g2.setCursor(66,20);
    u8g2.print (12-dr);
    u8g2.setDrawColor(0);
    u8g2.drawBox(1, 19, 82, 10);
    u8g2.setCursor(1,28);
    u8g2.print ("AirT:");
   // u8g2.setCursor(41,30);
  
    u8g2.print (air_time[dr]);
  //  u8g2.setCursor(66,30);
    u8g2.print ("ms ");
    
    u8g2.setDrawColor(1);
    u8g2.setCursor(2,37);


    // if I use the standar dtostrf and a float variable I run out of flash memory, then I used this way to convert an int in a "float" string
    char temp[3];
    char* formato="%i";
    sprintf(temp, formato, bateria);
    u8g2.print (temp[0]);
    u8g2.print (",");
    u8g2.print (temp[1]);
    u8g2.print ("V");

    
//   u8g2.print (bateria);
   u8g2.setCursor(41,37);
//   u8g2.print ("man");

 // if(LMIC.txend > os_getTime()) {
    u8g2.print ((os_getTime()-LMIC.globalDutyAvail)/OSTICKS_PER_SEC);
 //   }
   u8g2.print ("s");

//  map ( (os_getTime(),last_packet,auto_timer,0,82) 
//  u8g2.drawBox(1, 38, map(os_getTime(),last_packet,auto_timer,0,82), 10);
//  u8g2.drawBox(1, 38, 82, 10);

// for auto mode, progress bar until next packet.

 if (MANUAL==true) {u8g2.drawBox(1, 38, 82, 10);} else {u8g2.drawBox(1, 38, map(os_getTime(),last_packet,auto_timer,0,82), 10);};
//  u8g2.drawBox(1, if (MANUAL==true) {82 } else {map(os_getTime(),last_packet,auto_timer,0,82)};, 82, 10);

   u8g2.setCursor(2,47);
   u8g2.setDrawColor(2);

  u8g2.setFontMode(1);

  if (MANUAL==true) {u8g2.print ("Man."); } else {u8g2.print ("Auto"); };
  if (ALARM==true) {u8g2.print ("  BEEP"); };

  u8g2.setFontMode(0);

  
//    u8g2.drawStr(0,24,"Channel: ");
  } while ( u8g2.nextPage() );

  
   
   // handle button
  int event1 = button1.handle();
  int event2 = button2.handle();




  // if button 1 have a long press enable or disable alarm
  if ((event1 == 1) && (event2 == 1))  {
    
    ALARM = !ALARM;   // Switch the ALARM
  //  Serial.println("B12L");
    beep();
    event1 = event2 = 0 ;
    }


  // if button 1 have a long press enable or disable alarm
  if ((event1 == 2) && (event2 == 2))  {
    BACKLIGHT = !BACKLIGHT;   // Switch the ALARM
  //  Serial.println("B12L");
    digitalWrite(BACKLIGHT_PIN, BACKLIGHT);
    event1 = event2 = 0 ;
    }



  // if button 1 have a long press enable or disable alarm
  if (event1 == 2)   {
    
    MANUAL = !MANUAL;   // Switch the ALARM
  //  Serial.println("B12L");
    beep();
    event1 = event2 = 0 ;
    
    }


// if there we are in auto mode and reach the next packet timing, simulate that button 1 is pressed 
  if ((MANUAL==false) && (auto_timer<os_getTime())){
    auto_timer= os_getTime() + 1000*OSTICKS_PER_SEC;  
    event1 = 1;
  }
  

// if button 2 short press change channel

  if (event2 == 1)   {
    channel++;
    if (channel==9)  channel = 0;
    os_init();
    init_LoRa();
     for (int i = 0; i < channel; i++) {
       beep();
       delay(40);
      }
     
    }
  




   // if button 2 long press change SF

  if (event2 == 2)   {
    dr--;
    if (dr==-1)  dr = 5 ;
    os_init();
    init_LoRa();
     for (int i = 0; i < dr; i++) {
       long_beep();
       delay(80);
      }
  }
  

//  if (event2 == 2)   Serial.println("Boton 2 L");

  if (event1 == 1)  {
    
  puertaAbierta();
  bateria = readVcc() / 100 ;
  beep();
  // Do something here
    // Wait for response of the queued message (check if message is send correctly)
    os_runloop_once();
    // Continue until message is transmitted correctly
    LMIC_event_Timeout = 60 * 100; // 60 * 100 times 10mSec = 60 seconds
    while (LMIC_transmitted != 1)    {
      os_runloop_once();
      // Add timeout counter when nothing happens:
      LMIC_event_airtime = LMIC_event_Timeout;
//        Serial.println(LMIC_event_airtime);
      delay(10);
      if (LMIC_event_Timeout-- == 0)      {
        // Timeout when there's no "EV_TXCOMPLETE" event after 60 seconds
//        Serial.println(F("not tx"));
        break;
      }
    }

//      Serial.println(6000-LMIC_event_airtime);

    LMIC_transmitted = 0;
  
  } 


  delay(DELAY);
  
}
