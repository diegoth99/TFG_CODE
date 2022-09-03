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

//LORA COMMAND ALOHA
typedef struct Lora_CommandA_Struct{
    uint16_t flag;              //5
    uint16_t TimeNextPacket;
    uint32_t ExperimentTime;
    uint16_t WAIT_TIME_ALOHA; 
    uint16_t TRY_ACK_MAX; 
    uint32_t T_BEACON;  
};
typedef union Lora_CommandA{
    Lora_CommandA_Struct Lora_ComA;
    uint8_t byteArray[16]; 
};
Lora_CommandA ComA;

void setup() {
    Serial.begin(115200);

    rssi=0;

    Radio.Init( &RadioEvents );
    Radio.SetChannel( RF_FREQUENCY );
    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 ); 
   
    ComA.Lora_ComA.flag = 5;
    
    //Rellenamos variables para enviar el Comando Aloha
    ComA.Lora_ComA.ExperimentTime = 300000;  
    ComA.Lora_ComA.TimeNextPacket = 5000; 
    
    
    ComA.Lora_ComA.WAIT_TIME_ALOHA = 352;          //2*(Ttx + Tprop + Tproc) = 2*(164,4 + 1,4 + 10) = 351,6 == 352 ms 
    ComA.Lora_ComA.TRY_ACK_MAX = 5; 
    ComA.Lora_ComA.T_BEACON = 80000;  
                         
    //ENVIO DEL COMANDO
    Serial.println("CC1;Comand ALOHA sended;");
    turnOnRGB(COLOR_SEND,0);                  
    Radio.Send((uint8_t *)ComA.byteArray, sizeof(ComA.byteArray));
    turnOnRGB(0,0); 
   }
void loop(){
  }
