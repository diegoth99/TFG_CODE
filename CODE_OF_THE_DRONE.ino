#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <Wire.h>

#ifndef LoraWan_RGB
#define LoraWan_RGB 0
#endif
#define RF_FREQUENCY                                868000000 // Hz
#define TX_OUTPUT_POWER                             0        // dBm                     //Ponerlo mas bajo (defecto:5) 
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
void OnTxDone( void );
void OnTxTimeout( void );
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );
void OnRxTimeout(void);

typedef enum
{
    RX_START,
    RX,
    TX_ALOHA,
    TX_CSMA_CA,
    LOWPOWER
}States_t;
States_t state;

typedef enum
{
    P_RTS,
    P_DATAPACKET
}Packets_t;
Packets_t Packet;

typedef enum
{
    ALOHA,
    CSMA_CA
}Protocol_t;
Protocol_t protocol;

//Boolean Parameters  
bool ACK;
bool beacon_sended;                  
bool CTS; 
bool ProtocolAloha;
bool min_one_beacon_sended;
bool waiting_data_packet;
bool new_wait_time;

//Time Parameters 
long tiempo_exp = 0;                //Time since the node is powered on
long time_beacon =0;                    //The beacon is transmitted again every time_beacon  
long t_send_beacon = 0;             //Time at which the previous beacon was sent
long time_lapse = 0;                //Difference between current time and the sending of the beacon
long Tswitch = 0;               //Time in which the protocol is changed
long t_envio_com = 0;           //Time when the command si received. 
long t_start_wait_time = 0;
long wait_time = 0;
long new_waiting_time = 0;

//Delay
long Time_NAV_RTS = 0;          //Network Allocation Vector for RTS
long Time_NAV_CTS = 0;          //Network Allocation Vector for CTS        
long DIFS = 0;                  //Distributed Coordination Function Inter Frame Space
long SIFS = 0;                  //Short Inter Frame Spacing
long t_to_next_packetC = 0;
long t_to_next_packetA = 0;


int16_t Rssi,rxSize;

//VARIABLES OF THE NODE
int SAT_ID;
int CSMA_CA_PACKET_TYPE; //1
int ALOHA_PACKET_TYPE;   //0

//LORA DATAPACKET
typedef struct Lora_Packet_Struct{
               uint8_t flag;              //0
               uint8_t satellite_id;      
               uint8_t packet_type;       //0 or 1
               uint8_t node_id;         
               uint32_t packet_id;          
               uint64_t timestamp;        
               uint32_t pos_x;            
               uint32_t pos_y;            
               uint16_t pos_z;          
               uint16_t temperature;       
               uint16_t soilmoisture;    
           };
typedef union Lora_Packet{
    Lora_Packet_Struct Lora_Packet_S;
    uint8_t byteArray[30]; 
};
Lora_Packet LoraPacket;

//LORA ACK
typedef struct Lora_ACK_Struct{
     uint8_t flag;              //1
     uint8_t satellite_id;      
     uint8_t packet_type;
     uint8_t node_id;
     uint32_t packet_id;
     uint64_t timestamp;
     uint16_t free_slots;
};
typedef union Lora_ACK{
    Lora_ACK_Struct Lora_ACK_S;
    uint8_t byteArray[18]; 
};
Lora_ACK LoraACK;

//LORA BEACON
typedef struct Lora_Beacon_Struct{
    uint16_t flag;              //2
    uint16_t satellite_id;
    uint32_t timestamp;    
};
typedef union Lora_Beacon{
    Lora_Beacon_Struct Lora_BS;
    uint8_t byteArray[8]; 
};
Lora_Beacon LoraB;

//LoraCTS
typedef struct Lora_CTS_Struct{
     uint8_t flag;              //3
     uint8_t satellite_id;      //1
     uint8_t packet_type;
     uint8_t node_id;
     uint32_t packet_id;
     uint64_t timestamp;
     uint16_t NAV_CTS;
};
typedef union Lora_CTS{
    Lora_CTS_Struct Lora_CTS_S;
    uint8_t byteArray[18]; 
};
Lora_CTS LoraCTS;

//LoraRTS
typedef struct Lora_RTS_Struct{
     uint8_t flag;              //4
     uint8_t satellite_id;      //1
     uint8_t packet_type;
     uint8_t node_id;
     uint32_t packet_id;
     uint64_t timestamp;
     uint16_t NAV_RTS;
};
typedef union Lora_RTS{
    Lora_RTS_Struct Lora_RTS_S;
    uint8_t byteArray[18]; 
};
Lora_RTS LoraRTS;

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

                                                                            
                                                                            //SETUP 
void setup() {
    Serial.begin(115200);

    //Inicialización de los parámetros a 0
    Rssi=0;
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.RxTimeout = OnRxTimeout;
    
    Radio.Init( &RadioEvents );
    Radio.SetChannel( RF_FREQUENCY );
    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );

    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   0, true, 0, 0, LORA_IQ_INVERSION_ON, true );
    
    //Initialization                       
    beacon_sended = false;
    ACK = false;
    CTS = false;
    ProtocolAloha = false;
    min_one_beacon_sended = false;
    waiting_data_packet = false;
    new_wait_time = false;
    
    //VARIABLES OF THE NODE
    SAT_ID = 100;
    ALOHA_PACKET_TYPE = 0;   //0:ALOHA //1:CSMA/CA
    CSMA_CA_PACKET_TYPE = 1;
    state = RX_START;                                      //As default we start with state RX
}

void loop(){
	switch(state)
	{
    case RX_START:
      Radio.Rx( 0 );  
      state = LOWPOWER;
      break;
    
    case RX:
      //Radio.Rx(t_wait_time);  
      if(new_wait_time == false){
        t_start_wait_time = millis();
        Radio.Rx(wait_time); 
        state = LOWPOWER;
      }
      else{
        Radio.Rx(new_waiting_time);
        state = LOWPOWER;
        }
      break;    
                            
		case TX_CSMA_CA:

        if(millis() <= t_envio_com + ComC.Lora_ComC.ExperimentTime){                                           

          time_lapse = millis() - t_send_beacon;                                      
          if (time_lapse >= time_beacon){               
            beacon_sended = false;                               
            ACK = false;                             
            CTS = false; 
            }

          if (beacon_sended == false){      
            
            //Rellenamos variables para enviar el Beacon
            LoraB.Lora_BS.flag = 2;
            LoraB.Lora_BS.satellite_id = SAT_ID;  
            LoraB.Lora_BS.timestamp = millis();
                                 
            //ENVIO DEL BEACON
            Serial.print("X06;Beacon sended;");
            Serial.printf("%d; ; ; ;",LoraB.Lora_BS.satellite_id);
            Serial.println(LoraB.Lora_BS.timestamp); 
            turnOnRGB(COLOR_SEND,0); 
            min_one_beacon_sended = true;
            beacon_sended = true;                                      //The beacon has been sent therefore beacon_sended = true
            
            t_send_beacon = millis();                                   
            Radio.Send((uint8_t *)LoraB.byteArray, sizeof(LoraB.byteArray)); 
            }
  
          //ENVIO DE CTS
            if (CTS == true){                                     
              delay(SIFS); 
              // We fill variables. Some of them with what we have received from the RTS. 
              LoraCTS.Lora_CTS_S.flag = 3;
              LoraCTS.Lora_CTS_S.satellite_id = SAT_ID;                              
              LoraCTS.Lora_CTS_S.packet_type = CSMA_CA_PACKET_TYPE;                 
              LoraCTS.Lora_CTS_S.packet_id +=1;                                     
              LoraCTS.Lora_CTS_S.node_id = LoraRTS.Lora_RTS_S.node_id;              //It is sent to the same node from which the RTS was received 
              LoraCTS.Lora_CTS_S.timestamp = millis();
              LoraCTS.Lora_CTS_S.NAV_CTS = Time_NAV_CTS;
                                       
              Serial.print("X07;CTS sended;");
              Serial.printf("%d;",LoraCTS.Lora_CTS_S.satellite_id);
              Serial.printf("%d;",LoraCTS.Lora_CTS_S.packet_type);
              Serial.printf("%d;",LoraCTS.Lora_CTS_S.packet_id);
              Serial.printf("%d;",LoraCTS.Lora_CTS_S.node_id);                     
              Serial.printf("%d; ; ; ; ; ; ; ; ; ;",LoraCTS.Lora_CTS_S.timestamp);
              Serial.println(LoraCTS.Lora_CTS_S.NAV_CTS);

              waiting_data_packet = true;
              turnOnRGB(COLOR_SEND,0); 
               
              //Una vez enviado el ACK pasamos a estado RX.
              Radio.Send((uint8_t *)LoraCTS.byteArray, sizeof(LoraCTS.byteArray));
              }
              
          //ENVIO DE ACK
          else if (ACK == true){                                     
                ACK = false;                                      
                delay(SIFS);
                // We fill variables. Some of them with what we have received from the DataPacket 
                LoraACK.Lora_ACK_S.flag = 1;
                LoraACK.Lora_ACK_S.satellite_id = SAT_ID;                              
                LoraACK.Lora_ACK_S.packet_type = CSMA_CA_PACKET_TYPE;                 
                LoraACK.Lora_ACK_S.packet_id +=1;                                     
                LoraACK.Lora_ACK_S.node_id = LoraPacket.Lora_Packet_S.node_id;        //It is sent to the same node from which the DataPacket was received  
                LoraACK.Lora_ACK_S.timestamp = millis();
                LoraACK.Lora_ACK_S.free_slots = 0;                                    
  
                //ENVIO DEL ACK
                Serial.print("X08;ACK sended;");
                Serial.printf("%d;",LoraACK.Lora_ACK_S.satellite_id);
                Serial.printf("%d;",LoraACK.Lora_ACK_S.packet_type);
                Serial.printf("%d;",LoraACK.Lora_ACK_S.packet_id);
                Serial.printf("%d;",LoraACK.Lora_ACK_S.node_id);                     
                Serial.printf("%d; ; ; ; ; ; ;",LoraACK.Lora_ACK_S.timestamp);
                Serial.println(LoraACK.Lora_ACK_S.free_slots);
                 
                turnOnRGB(COLOR_SEND,0); 
                
                Radio.Send((uint8_t *)LoraACK.byteArray, sizeof(LoraACK.byteArray));      
              }
        }
        
        else{
          Serial.println("CSMA_CA experiment has finished");
          beacon_sended = false;
          ACK = false;
          CTS = false;
          ProtocolAloha = false;
          min_one_beacon_sended = false;
          waiting_data_packet = false;
          new_wait_time = false;
          
          delay(10000);
          state = RX_START;
          }
        state=LOWPOWER;
        break;

    case TX_ALOHA:
        if(millis() <= t_envio_com + ComA.Lora_ComA.ExperimentTime){           
          time_lapse = millis() - t_send_beacon;    
          if (time_lapse >= time_beacon){               
            beacon_sended = false;                                      //beacon_sended is false since no one has been sent after the time lapse             
            ACK = false;                                                //Resetea variables
            }
  
          if (beacon_sended == false){      
            beacon_sended = true;                                      //The beacon has been sent so beacon_sended = true
            min_one_beacon_sended = true;
            // We fill variables to send the Beacon
            LoraB.Lora_BS.flag = 2;
            LoraB.Lora_BS.timestamp = millis();
            LoraB.Lora_BS.satellite_id = SAT_ID;                    
  
            //SEND BEACON
            Serial.print("X01;Beacon sended;");
            Serial.printf("%d; ; ; ;",LoraB.Lora_BS.satellite_id);
            Serial.println(LoraB.Lora_BS.timestamp); 
             
            turnOnRGB(COLOR_SEND,0); 
            //Current Send Beacon time. Then it is compared.
            t_send_beacon = millis();
            Radio.Send((uint8_t *)LoraB.byteArray, sizeof(LoraB.byteArray)); 
            }
  
          //ENVIO DE ACK
          if (ACK == true){                                      
              ACK = false;                                        
        
              //Rellenamos variables. Alguans de ellas con lo que hemos recibido del DataPacket. 
              LoraACK.Lora_ACK_S.flag = 1;
              LoraACK.Lora_ACK_S.satellite_id = SAT_ID;                             //Valor fijo e invariable
              LoraACK.Lora_ACK_S.packet_type = ALOHA_PACKET_TYPE;                   //0 - Aloha Packet Type
              LoraACK.Lora_ACK_S.packet_id +=1;                                     //packet_id va sumandose a cada uno que envia
              LoraACK.Lora_ACK_S.node_id = LoraPacket.Lora_Packet_S.node_id;        //Se envia al mismo nodo. En recepción del ground observan si este ACK es para ellos o no. 
              LoraACK.Lora_ACK_S.timestamp = millis();
              LoraACK.Lora_ACK_S.free_slots = 0;                                    

              //ENVIO DEL ACK
              Serial.print("X02;ACK sended;");
              Serial.printf("%d;",LoraACK.Lora_ACK_S.satellite_id);
              Serial.printf("%d;",LoraACK.Lora_ACK_S.packet_type);
              Serial.printf("%d;",LoraACK.Lora_ACK_S.packet_id);
              Serial.printf("%d;",LoraACK.Lora_ACK_S.node_id);                     
              Serial.printf("%d; ; ; ; ; ; ;",LoraACK.Lora_ACK_S.timestamp);
              Serial.println(LoraACK.Lora_ACK_S.free_slots);
               
              turnOnRGB(COLOR_SEND,0); 
              Radio.Send((uint8_t *)LoraACK.byteArray, sizeof(LoraACK.byteArray));      
            }
        }
        
        else{
          Serial.println("ALOHA experiment has finished");
          ProtocolAloha = false;
          min_one_beacon_sended = false;
          beacon_sended = false;
          delay(10000);
          state = RX_START;
          }    
        state=LOWPOWER;
        break; 
    
    case LOWPOWER:
        lowPowerHandler();
        break;
          default:
            break;
 
	}
  Radio.IrqProcess( );
}

void OnTxDone( void ){
	turnOnRGB(0,0);
	switch(protocol){
    case CSMA_CA:
    	if (CTS == true){
    	  state=RX;
    	  }
    	else{
    	  state = RX_START;
    	  }
    break;

     case ALOHA:
      state = RX_START;
     break;
  }
}

void OnTxTimeout( void ){
    Radio.Sleep( );
    if(ProtocolAloha == true){
      turnOnRGB(0,0);
      state = TX_ALOHA;
      }
    else{
      turnOnRGB(0,0);
      state = TX_CSMA_CA;
      }
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr ){
    Rssi=rssi;
    rxSize=size;
    turnOnRGB(COLOR_RECEIVED,0);
    Radio.Sleep( );

    
    //DATAPACKET
    if(payload[0] == 0){
              for(int z=0 ; z<rxSize ; z++){
                LoraPacket.byteArray[z] = payload[z];
                }
    Packet = P_DATAPACKET; 
    }

    //RTS
    if(payload[0] == 4){ 
            for(int z=0 ; z<rxSize ; z++){
                LoraRTS.byteArray[z] = payload[z];
                }
    Packet = P_RTS;
    }

    //PROTOCOL ALOHA
    if(payload[0] == 5){ 
            for(int z=0 ; z<rxSize ; z++){
                ComA.byteArray[z] = payload[z];
                }
    ProtocolAloha = true;
    t_envio_com = millis();
    t_to_next_packetA = ComA.Lora_ComA.TimeNextPacket;
    time_beacon = ComA.Lora_ComA.T_BEACON;
    Serial.println("CC1;Initializing Protocol ALOHA");
    Serial.printf("Experiment Time: %d\n", ComA.Lora_ComA.ExperimentTime);  
    Serial.printf("Time to send next packet at ground: %d\n", t_to_next_packetA); 
    Serial.printf("Time of the Beacon repetition: %d\n", time_beacon);                         
    protocol = ALOHA;
    }
    
    //PROTOCOL CSMA_CA
    if(payload[0] == 6){ 
            for(int z=0 ; z<rxSize ; z++){
                ComC.byteArray[z] = payload[z];
                }
    ProtocolAloha = false;    
    t_envio_com = millis();
    t_to_next_packetC = ComC.Lora_ComC.TimeNextPacket;
    SIFS = ComC.Lora_ComC.T_SIFS;
    time_beacon = ComC.Lora_ComC.T_BEACON;
    wait_time = ComC.Lora_ComC.WAIT_TIME;
    Serial.println("CC2;Initializing Protocol CSMA_CA");
    Serial.printf("Experiment Time: %d\n", ComC.Lora_ComC.ExperimentTime);  
    Serial.printf("SIFS: %d\n", SIFS);    
    Serial.printf("Time of the Beacon repetition: %d\n", time_beacon);                    
    protocol = CSMA_CA;
    }

    switch(protocol){
      
      case ALOHA:
        if(min_one_beacon_sended == true){
          if(millis() <= t_envio_com + ComA.Lora_ComA.ExperimentTime){
            switch(Packet){
            case P_DATAPACKET:
              if(LoraPacket.Lora_Packet_S.satellite_id == SAT_ID){
                Serial.print("X03;Data Packet Received;");
                Serial.printf("%d;",LoraPacket.Lora_Packet_S.satellite_id);
                Serial.printf("%d;",LoraPacket.Lora_Packet_S.packet_type);
                Serial.printf("%d;",LoraPacket.Lora_Packet_S.packet_id);  
                Serial.printf("%d;",LoraPacket.Lora_Packet_S.node_id);
                Serial.printf("%d;",millis());
                Serial.printf("%d;",LoraPacket.Lora_Packet_S.pos_x);
                Serial.printf("%d;",LoraPacket.Lora_Packet_S.pos_y);
                Serial.printf("%d;",LoraPacket.Lora_Packet_S.pos_z);
                Serial.printf("%d;",LoraPacket.Lora_Packet_S.temperature);
                Serial.println(LoraPacket.Lora_Packet_S.soilmoisture);
                
                ACK = true; 
                state = TX_ALOHA;
               }
               
              else{
                  Serial.println("X04;The received message is not for this satellite"); 
                  state = RX;
                }    
            break;
            } 
          }
          
          else{
            Serial.println("ALOHA experiment has finished");
            ProtocolAloha = false;
            min_one_beacon_sended = false;
            beacon_sended = false;
            delay(10000);
            state = RX_START;
            }
        }
        
        else{
          beacon_sended = false;
          state = TX_ALOHA;
          }
      break;

      case CSMA_CA:
        if(min_one_beacon_sended == true){
          if(millis() <= t_envio_com + ComC.Lora_ComC.ExperimentTime){
            switch(Packet){
            
              case P_RTS:
                if(LoraRTS.Lora_RTS_S.satellite_id == SAT_ID){
                  
                  Serial.print("X09;RTS received;");
                  Serial.printf("%d;",LoraRTS.Lora_RTS_S.satellite_id);
                  Serial.printf("%d;",LoraRTS.Lora_RTS_S.packet_type);
                  Serial.printf("%d;",LoraRTS.Lora_RTS_S.packet_id);
                  Serial.printf("%d;",LoraRTS.Lora_RTS_S.node_id);
                  Serial.printf("%d; ; ; ; ; ; ; ; ;",millis());
                  Serial.println(LoraRTS.Lora_RTS_S.NAV_RTS);
                  
                  if(waiting_data_packet == true){
                    Serial.print("X12;RTS received but we are expecting a DataPacket; ; ; ; ;");
                    Serial.println(millis());
                    new_waiting_time = wait_time - (millis() - t_start_wait_time);
                    
                    new_wait_time = true;
                    state = RX;
                    
                    if(new_waiting_time <= 0){
                      Serial.print("X14;RXTIMEOUT. DataPacket has not been received; ; ; ; ;");
                      Serial.println(millis());
                      waiting_data_packet = false;
                      CTS = false;
                      new_wait_time = false;
                      state = RX_START;
                      } 
                    }
                    
                  else{
                    CTS = true;
                    state = TX_CSMA_CA;
                    }
                 }
                 
                else{
                  Serial.println("X10;The received message is not for this satellite"); 
                  state = RX;
                  }        
            break; 
        
            //DataPacket
            case P_DATAPACKET:
              if(LoraPacket.Lora_Packet_S.satellite_id = SAT_ID){ 
                Serial.print("X11;Data Packet Received;");
                Serial.printf("%d;",LoraPacket.Lora_Packet_S.satellite_id);
                Serial.printf("%d;",LoraPacket.Lora_Packet_S.packet_type);
                Serial.printf("%d;",LoraPacket.Lora_Packet_S.packet_id);  
                Serial.printf("%d;",LoraPacket.Lora_Packet_S.node_id);
                Serial.printf("%d;",millis());
                Serial.printf("%d;",LoraPacket.Lora_Packet_S.pos_x);
                Serial.printf("%d;",LoraPacket.Lora_Packet_S.pos_y);
                Serial.printf("%d;",LoraPacket.Lora_Packet_S.pos_z);
                Serial.printf("%d;",LoraPacket.Lora_Packet_S.temperature);
                Serial.println(LoraPacket.Lora_Packet_S.soilmoisture);
                
                CTS = false;
                ACK = true;
                waiting_data_packet = false; 
                state = TX_CSMA_CA;
               }
           
              else{
              Serial.println("X10;The received message is not for this satellite"); 
              state = RX;
               }   
            break; 
          } 
          }

          else{
            Serial.println("CSMA_CA experiment has finished");
            ProtocolAloha = false;
            min_one_beacon_sended = false;
            beacon_sended = false;
            waiting_data_packet = false;
            new_wait_time = false;
            delay(10000);
            state = RX_START;
            }
        }

        else{
          beacon_sended = false;
          state = TX_CSMA_CA;
          }
      break;
    }
}

void OnRxTimeout(void){
  Radio.Sleep( );
  if(millis() <= t_envio_com + ComC.Lora_ComC.ExperimentTime){
    Serial.print("X13;RXTIMEOUT. DataPacket has not been received; ; ; ; ;");
    Serial.println(millis());
    CTS = false;
    waiting_data_packet = false;
    new_wait_time = false; 
    state = RX_START;
  }
  else{
    Serial.println("CSMA_CA experiment has finished");
    ProtocolAloha = false;
    min_one_beacon_sended = false;
    beacon_sended = false;
    waiting_data_packet = false;
    new_wait_time = false;
    delay(10000);
    state = RX_START;
    }
}
