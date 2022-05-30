#define TYPE_PPM2ELRS


#define RADIO_ADDRESS                  0xEA
#define ADDR_MODULE                    0xEE  //  Crossfire transmitter
#define TYPE_CHANNELS                  0x16
// ELRS command
#define ELRS_ADDRESS                   0xEE
#define ELRS_BIND_COMMAND              0xFF
#define ELRS_WIFI_COMMAND              0xFE
#define ELRS_PKT_RATE_COMMAND          1
#define ELRS_TLM_RATIO_COMMAND         2

#define ELRS_POWER_COMMAND             3
#define TYPE_SETTINGS_WRITE            0x2D
#define ADDR_RADIO                     0xEA  //  Radio Transmitter

#define RC_CHANNEL_MIN 172
#define RC_CHANNEL_MID 991
#define RC_CHANNEL_MAX 1811



#define RC_CHANS 12
volatile uint16_t rcValue[RC_CHANS] = {1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502}; // interval [1000;2000]
//static uint8_t rcChannel[RC_CHANS] = {SERIAL_SUM_PPM};

//#define PPM_PIN_INTERRUPT          attachInterrupt(0, rxInt, RISING); //PIN 0
//PPM_PIN_INTERRUPT

int Aileron_OFFSET = 0;        // values read from the pot 
int Elevator_OFFSET  = 0; 
int Throttle_OFFSET =0;
int Rudder_OFFSET  = 0; 
// internal crsf variables
#define CRSF_TIME_NEEDED_PER_FRAME_US   1100 // 700 ms + 400 ms for potential ad-hoc request
//#define CRSF_TIME_BETWEEN_FRAMES_US     6666 // At fastest, frames are sent by the transmitter every 6.667 milliseconds, 150 Hz
#define CRSF_TIME_BETWEEN_FRAMES_US     4000 // 4 ms 250Hz @ BAUD 115200 
#define CRSF_DIGITAL_CHANNEL_MIN 172
#define CRSF_DIGITAL_CHANNEL_MAX 1811
#define CRSF_PAYLOAD_OFFSET offsetof(crsfFrameDef_t, type)
#define CRSF_MAX_CHANNEL 16
#define CRSF_FRAME_SIZE_MAX 64
#define SERIAL_BAUDRATE 115200
#define CRSF_MSP_RX_BUF_SIZE 128
#define CRSF_MSP_TX_BUF_SIZE 128
#define CRSF_PAYLOAD_SIZE_MAX   60
#define CRSF_PACKET_LENGTH 22
#define CRSF_PACKET_SIZE  26
#define CRSF_CMD_PACKET_SIZE  8
#define CRSF_FRAME_LENGTH 24;   // length of type + payload + crc




int rateButtonPressed=0;
int powerButtonPressed=0;
boolean rateChangeHasRun = false;
boolean powerChangeHasRun = false;

// from https://github.com/DeviationTX/deviation/pull/1009/ ELRS menu implement in deviation TX
static u8  currentPktRate =5; //  "250Hz", "150Hz", "50Hz"
  //                                1         3       5      

//915 transmitter 
//static u8  currentPktRate =1; //  "200Hz", "100Hz", "50Hz", "25Hz"
  //                                2         4       5        6


//500mw for e19 module
static u8  currentPower =2 ;//  "10mW", "25mW", "50mW", "100mW", "250mW" , "500mW"
  //                               0     1         2        3        4         5
  
  
static u8 currentTlmRatio =0 ;
static u8 currentBind = 0;
static u8 currentWiFi = 0;
static u8 getParamsCounter = 0;
static u8 currentFrequency = 6;


static u16 convertPktRateToPeriod(u8 rfFreq, u8 rate)
{
  if (rfFreq == 0) return CRSF_TIME_BETWEEN_FRAMES_US;
  switch (rate) {
    case 0:
      if (rfFreq == 6) return 2000;
      return 5000;
    case 1:
      if (rfFreq == 6) return 4000;
      return 10000;
    case 2:
      if (rfFreq == 6) return 6666;
      return 20000;
    case 3:
      if (rfFreq == 6) return 20000;
      return 40000;
    case 4: return 40000;
  }
  return CRSF_TIME_BETWEEN_FRAMES_US;
}

// crc implementation from CRSF protocol document rev7
static u8 crsf_crc8tab[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9};

u8 crsf_crc8(const u8 *ptr, u8 len) {
    u8 crc = 0;
    for (u8 i=0; i < len; i++) {
        crc = crsf_crc8tab[crc ^ *ptr++];
    }
    return crc;
}

void crsfPreparePacket(uint8_t packet[], int channels[]){

    static int output[CRSF_MAX_CHANNEL] = {0};
    const uint8_t crc = crsf_crc8(&packet[2], CRSF_PACKET_SIZE-3);
    /*
     * Map 1000-2000 with middle at 1500 chanel values to
     * 173-1811 with middle at 992 S.BUS protocol requires
     */
    for (uint8_t i = 0; i < CRSF_MAX_CHANNEL; i++) {
        output[i] = map(channels[i], RC_CHANNEL_MIN, RC_CHANNEL_MAX, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX);
    }    


    // packet[0] = UART_SYNC; //Header
    packet[0] = ADDR_MODULE; //Header
    packet[1] = 24;   // length of type (24) + payload + crc
    packet[2] = TYPE_CHANNELS;
    packet[3] = (uint8_t) (channels[0] & 0x07FF);
    packet[4] = (uint8_t) ((channels[0] & 0x07FF)>>8 | (channels[1] & 0x07FF)<<3);
    packet[5] = (uint8_t) ((channels[1] & 0x07FF)>>5 | (channels[2] & 0x07FF)<<6);
    packet[6] = (uint8_t) ((channels[2] & 0x07FF)>>2);
    packet[7] = (uint8_t) ((channels[2] & 0x07FF)>>10 | (channels[3] & 0x07FF)<<1);
    packet[8] = (uint8_t) ((channels[3] & 0x07FF)>>7 | (channels[4] & 0x07FF)<<4);
    packet[9] = (uint8_t) ((channels[4] & 0x07FF)>>4 | (channels[5] & 0x07FF)<<7);
    packet[10] = (uint8_t) ((channels[5] & 0x07FF)>>1);
    packet[11] = (uint8_t) ((channels[5] & 0x07FF)>>9 | (channels[6] & 0x07FF)<<2);
    packet[12] = (uint8_t) ((channels[6] & 0x07FF)>>6 | (channels[7] & 0x07FF)<<5);
    packet[13] = (uint8_t) ((channels[7] & 0x07FF)>>3);
    packet[14] = (uint8_t) ((channels[8] & 0x07FF));
    packet[15] = (uint8_t) ((channels[8] & 0x07FF)>>8 | (channels[9] & 0x07FF)<<3);
    packet[16] = (uint8_t) ((channels[9] & 0x07FF)>>5 | (channels[10] & 0x07FF)<<6);  
    packet[17] = (uint8_t) ((channels[10] & 0x07FF)>>2);
    packet[18] = (uint8_t) ((channels[10] & 0x07FF)>>10 | (channels[11] & 0x07FF)<<1);
    packet[19] = (uint8_t) ((channels[11] & 0x07FF)>>7 | (channels[12] & 0x07FF)<<4);
    packet[20] = (uint8_t) ((channels[12] & 0x07FF)>>4  | (channels[13] & 0x07FF)<<7);
    packet[21] = (uint8_t) ((channels[13] & 0x07FF)>>1);
    packet[22] = (uint8_t) ((channels[13] & 0x07FF)>>9  | (channels[14] & 0x07FF)<<2);
    packet[23] = (uint8_t) ((channels[14] & 0x07FF)>>6  | (channels[15] & 0x07FF)<<5);
    packet[24] = (uint8_t) ((channels[15] & 0x07FF)>>3);
    
    packet[25] = crsf_crc8(&packet[2], packet[1]-1); //CRC


}

uint8_t crsfPacket[CRSF_PACKET_SIZE];
int rcChannels[CRSF_MAX_CHANNEL];
uint32_t crsfTime = 0;



uint8_t crsfCmdPacket[CRSF_CMD_PACKET_SIZE];
void buildElrsPacket(uint8_t packetCmd[],u8 command, u8 value)
{
  packetCmd[0] = ADDR_MODULE;
  packetCmd[1] = 6; // length of Command (4) + payload + crc
  packetCmd[2] = TYPE_SETTINGS_WRITE;
  packetCmd[3] = ELRS_ADDRESS;
  packetCmd[4] = ADDR_RADIO;
  packetCmd[5] = command;
  packetCmd[6] = value;
  packetCmd[7] = crsf_crc8(&packetCmd[2], packetCmd[1]-1);

}


IRAM_ATTR void rxInt(void) {
    uint16_t now,diff;
    static uint16_t last = 0;
    static uint8_t chan = 0;
  #if defined(FAILSAFE)
    static uint8_t GoodPulses;
  #endif
  
    now = micros();
    sei();
    diff = now - last;
    last = now;
    if(diff>3000) chan = 0;
    else {
      if(900<diff && diff<2200 && chan<RC_CHANS ) {   //Only if the signal is between these values it is valid, otherwise the failsafe counter should move up
        rcValue[chan] = diff;
        #if defined(FAILSAFE)
          if(chan<4 && diff>FAILSAFE_DETECT_TRESHOLD) GoodPulses |= (1<<chan); // if signal is valid - mark channel as OK
          if(GoodPulses==0x0F) {                                               // If first four chanells have good pulses, clear FailSafe counter
            GoodPulses = 0;
            if(failsafeCnt > 20) failsafeCnt -= 20; else failsafeCnt = 0;
          }
        #endif
      }
    chan++;
  }
}



void setup() {
  // put your setup code here, to run once:

    pinMode(2,INPUT);
    attachInterrupt(2, rxInt, RISING);

    for (uint8_t i = 0; i < CRSF_MAX_CHANNEL; i++) {
        rcChannels[i] = RC_CHANNEL_MIN;
    }
    
    delay(1000); //Give enough time for uploda firmware
    Serial.begin(SERIAL_BAUDRATE);
}
#ifdef TYPE_PPM 
void loop() {
  Serial.print(rcValue[0]) ;  Serial.print('\t') ; 
  Serial.print(rcValue[1]) ;  Serial.print('\t') ; 
  Serial.print(rcValue[2]) ;  Serial.print('\t') ; 
  Serial.print(rcValue[3]) ;  Serial.print('\t') ; 
  Serial.print(rcValue[4]) ;  Serial.print('\t') ; 
  Serial.print(rcValue[5]) ;  Serial.print('\t') ; 
  Serial.print("\r\n") ; 
}
#endif

#ifdef TYPE_PPM2ELRS
void loop() {
    uint32_t currentMicros = micros();
    if (currentMicros > crsfTime) {
    rcChannels[0] = map(rcValue[0],1000,2000,RC_CHANNEL_MIN,RC_CHANNEL_MAX); 
    rcChannels[1] = map(rcValue[1],1000,2000,RC_CHANNEL_MIN,RC_CHANNEL_MAX); 
    rcChannels[2] = map(rcValue[2],1000,2000,RC_CHANNEL_MIN,RC_CHANNEL_MAX);
    rcChannels[3] = map(rcValue[3],1000,2000,RC_CHANNEL_MIN,RC_CHANNEL_MAX);    
    rcChannels[4] = map(rcValue[2],1000,2000,RC_CHANNEL_MIN,RC_CHANNEL_MAX);
    rcChannels[5] = map(rcValue[3],1000,2000,RC_CHANNEL_MIN,RC_CHANNEL_MAX);    
    
    } 
    buildElrsPacket(crsfCmdPacket,ELRS_PKT_RATE_COMMAND,currentPktRate);
    Serial.write(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
    delay(4);
    buildElrsPacket(crsfCmdPacket,ELRS_POWER_COMMAND,currentPower);
    Serial.write(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
    delay(4);
    if (currentMicros > crsfTime) {
        crsfPreparePacket(crsfPacket, rcChannels);
      //For gimal calibation only
         Serial.write(crsfPacket, CRSF_PACKET_SIZE);
       
        crsfTime = currentMicros + CRSF_TIME_BETWEEN_FRAMES_US;
    }
}
#endif
