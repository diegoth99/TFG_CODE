#include "LoRaWan_APP.h"
#include "Arduino.h"
#ifndef LoraWan_RGB
#define LoraWan_RGB 0
#endif
#define RF_FREQUENCY                                868000000 // Hz

#define TX_OUTPUT_POWER                             16        // dBm

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       8         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false


static RadioEvents_t RadioEvents;
int16_t rssi,rxSize;

//LORA COMMAND CSMA_CA
typedef struct Lora_CommandC_Struct{
    uint16_t flag;              //5
    uint16_t TimeNextPacket;
    uint32_t ExperimentTime;
    uint16_t TRY_ACK_MAX;
    uint16_t SENS_TIME; 
    uint16_t WAIT_TIME;
    uint16_t T_SIFS;
    uint16_t T_NAV_RTS;
    uint16_t T_NAV_CTS;
    uint32_t T_BEACON;
       
};
typedef union Lora_CommandC{
    Lora_CommandC_Struct Lora_ComC;
    uint8_t byteArray[24]; 
};
Lora_CommandC ComC;

void setup() {
    Serial.begin(115200);

    rssi=0;

    Radio.Init( &RadioEvents );
    Radio.SetChannel( RF_FREQUENCY );
    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 ); 
   
    ComC.Lora_ComC.flag = 6;
    
    //Rellenamos variables para enviar el Comando CSMA/CA
    ComC.Lora_ComC.ExperimentTime = 600000;  
    ComC.Lora_ComC.TimeNextPacket = 1; 
    
    ComC.Lora_ComC.TRY_ACK_MAX = 5; 
    ComC.Lora_ComC.SENS_TIME = 528;       //2*(Ttx + Tprop + Tproc) + SIFS = 2*(164,4 + 1,4 + 10) + 176 = 527,6 == 528        
    ComC.Lora_ComC.WAIT_TIME = 528;       //2*(Ttx + Tprop + Tproc) + SIFS = 2*(164,4 + 1,4 + 10) + 176 = 527,6 == 528  
    ComC.Lora_ComC.T_SIFS = 176;          //Ttx + Tprop + Tproc = 164,4 + 1,4 + 10 = 175,8 == 176
    ComC.Lora_ComC.T_NAV_RTS = 994;       //(SIFS+Tproc+Tprop)*3 + TtxCTS + TtxDP + TtxACK = (176+1,4+10)*3 + 133,6 + 164,4 + 133,6 = 993,8 = 994 ms
    ComC.Lora_ComC.T_NAV_CTS = 673;       //(SIFS+Tproc+Tprop)*2 + TtxDP + TtxACK = (176+1,4+10)*2 + 164,4 + 133,6 = 672,8 = 673 ms
    ComC.Lora_ComC.T_BEACON = 80000;                  
   
    //ENVIO DEL COMANDO
    Serial.println("CC1;Comand CSMA_CA sended;");
    turnOnRGB(COLOR_SEND,0);                  
    Radio.Send((uint8_t *)ComC.byteArray, sizeof(ComC.byteArray));
    turnOnRGB(0,0); 
   }
void loop(){
  }
