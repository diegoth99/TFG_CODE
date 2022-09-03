#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <Wire.h>
#include "HDC1080.h"

#ifndef LoraWan_RGB
#define LoraWan_RGB 0
#endif
#define RF_FREQUENCY                                868000000 // Hz
#define TX_OUTPUT_POWER                             0         // dBm 
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
    RX,
    RX_ALOHA,
    RX_CSMA_CA,
    TX_ALOHA,
    TX_CSMA_CA,
    LOWPOWER
}States_t;
States_t state;


typedef enum
{
    P_BEACON,
    P_RTS,
    P_CTS,
    P_ACK,
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
bool min_one_beacon_received; 
bool sensing_channel;
bool wrong_packet;
bool RTS; 
bool send_datapacket; 
bool expecting_CTS; 
bool ProtocolAloha;
bool new_wait_time;

//Time Parameters 
long timer = 0;                 //Timer for comparisons with the waittime
long tiempo_exp=0;              //Time since the node is powered on
long t_wait_time;               //Time of wait in ALOHA
long SENS_TIMEOUT_VALUE = 0;    //Time for sense the channel 
long tiempo_envio = 0;          //Sending Time  
long tiempo_actual = 0;         //Actual Time                          
long Dif_t = 0;                 //Temporary difference between the timestamp of the node and the timestamp of the satellite. 
long t_envio_com = 0;           //Time when the command si received. 
long new_waiting_time = 0;
long t_start_wait_time = 0;

//Delay
long Time_NAV_RTS = 0;          //Network Allocation Vector for RTS
long Time_NAV_CTS = 0;          //Network Allocation Vector for CTS
long TP = 0;                    //Propagation Time equals to SIFS, used for random delays
long RANDOM = 0;                //Random time used for sense the channel 
long T_b=0;                     //BackOff Time
long DIFS = 0;                  //Distributed Coordination Function Inter Frame Space
long SIFS = 0;                  //Short Inter Frame Spacing
long t_wait_time_ALOHA = 0;
//BackOff y Delays
int try_rec_ACK = 0;            //Counter of times that an attempt has been made to receive the ACK
int try_rec_ACK_max = 0;        //Maximum number of attempts to receive the ACK
long R=0;                       //Random BackOff number
long t_to_next_packetA = 0;
long t_to_next_packetC = 0;

//Parameters for capturing data from the HDC1080 sensor
double SensorTemperature;
HDC1080 hdc1080;

int16_t Rssi,rxSize;

//Soil Moisture Sensor variables
const int AirValue = 3250;   
const int WaterValue = 1900;  
int soilMoistureValue = 0;
int soilmoisturepercent = 0;

//VARIABLES OF THE NODE
int ID_NODE;        
int CSMA_CA_PACKET_TYPE;   
int ALOHA_PACKET_TYPE;
int POSITION_X;
int POSITION_Y;
int POSITION_Z;  

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


void setup() {
    Serial.begin(115200);

    //HDC1080 Setup
    pinMode(Vext, OUTPUT);
    digitalWrite(Vext, LOW);
    hdc1080.begin(0x40);

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
    min_one_beacon_received = false; 
    sensing_channel = false;
    wrong_packet = false;
    RTS = false; 
    send_datapacket = false; 
    expecting_CTS = false; 
    ProtocolAloha = false;
    new_wait_time = false;
    
    //VARIABLES OF THE NODE
    ID_NODE = 10;        
    ALOHA_PACKET_TYPE = 0;   //0:ALOHA //1:CSMA/CA
    CSMA_CA_PACKET_TYPE = 1;
    POSITION_X = 700;
    POSITION_Y = 500;
    POSITION_Z = 300; 

    state = RX;                  //As default we start with state RX
}

void loop(){

	switch(state)
	{
    case RX:
      Radio.Rx(0);
      state=LOWPOWER; 
      break;
      
    case RX_CSMA_CA:
    
      if(millis() <= t_envio_com + ComC.Lora_ComC.ExperimentTime){      
          
          if(sensing_channel == true){
            Radio.Rx(SENS_TIMEOUT_VALUE);
            }
            
          else{
            if(new_wait_time == false){
                t_start_wait_time = millis();
                Radio.Rx(t_wait_time); 
              }
            else{
              Radio.Rx(new_waiting_time);
              }
            }
           state = LOWPOWER;
        }
          
      else{
         Serial.println("CSMA_CA experiment has finished");
         ProtocolAloha = false;
         min_one_beacon_received = false;
         delay(10000);
         state = RX; 
        }
      break;  
      

		case TX_CSMA_CA: 
   
        if(RTS == true){
          R = random(0,(pow(2,try_rec_ACK)-1));                 
          T_b = R * t_wait_time;
          DIFS = SIFS + T_b;
          Serial.print("X33;DIFS; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;");
          Serial.println(DIFS);
          delay(DIFS);
          
          LoraRTS.Lora_RTS_S.flag = 4;
          LoraRTS.Lora_RTS_S.satellite_id = LoraB.Lora_BS.satellite_id;
          LoraRTS.Lora_RTS_S.packet_type = CSMA_CA_PACKET_TYPE;
          LoraRTS.Lora_RTS_S.packet_id += 1;
          LoraRTS.Lora_RTS_S.node_id = ID_NODE;
          
          LoraRTS.Lora_RTS_S.timestamp = millis() - Dif_t;
          LoraRTS.Lora_RTS_S.NAV_RTS = Time_NAV_RTS;          
          
          //ENVIO DEL RTS
          Serial.printf("X28;RTS sended;");
          Serial.printf("%d;",LoraRTS.Lora_RTS_S.satellite_id);
          Serial.printf("%d;",LoraRTS.Lora_RTS_S.packet_type);
          Serial.printf("%d;",LoraRTS.Lora_RTS_S.packet_id);
          Serial.printf("%d;",LoraRTS.Lora_RTS_S.node_id);
          Serial.printf("%d; ; ; ; ; ; ; ; ;",LoraRTS.Lora_RTS_S.timestamp);
          Serial.println(LoraRTS.Lora_RTS_S.NAV_RTS);
          RTS = false; 
          expecting_CTS = true; 
          sensing_channel = false;
          
          turnOnRGB(COLOR_SEND,0); //change rgb color          
          tiempo_envio = millis();                                               //Graba el tiempo actual para luego compararlo         
          Radio.Send((uint8_t *)LoraRTS.byteArray, sizeof(LoraRTS.byteArray));
          }

        if(send_datapacket == true){                  
          delay(SIFS);
          LoraPacket.Lora_Packet_S.flag = 0;
          LoraPacket.Lora_Packet_S.satellite_id = LoraB.Lora_BS.satellite_id;          //Copia el satellite_id que le ha llegado del Beacon.
          LoraPacket.Lora_Packet_S.timestamp = millis() - Dif_t;  
          LoraPacket.Lora_Packet_S.packet_type = CSMA_CA_PACKET_TYPE;                                     
          LoraPacket.Lora_Packet_S.packet_id += 1;                                 
          LoraPacket.Lora_Packet_S.node_id = ID_NODE;
          LoraPacket.Lora_Packet_S.pos_x = POSITION_X;
          LoraPacket.Lora_Packet_S.pos_y = POSITION_Y;
          LoraPacket.Lora_Packet_S.pos_z = POSITION_Z;

          //Lectura del sensor HDC1080 - Temperatura
          SensorTemperature = hdc1080.readTemperature();
          LoraPacket.Lora_Packet_S.temperature = SensorTemperature; 
          
          //Lectura del sensor SoilMoisture
          soilMoistureValue = analogRead(ADC);  
          soilmoisturepercent = map(soilMoistureValue, AirValue, WaterValue, 0, 100);
          if(soilmoisturepercent >= 100){
            LoraPacket.Lora_Packet_S.soilmoisture=100;
            }
          else if(soilmoisturepercent <=0){
            LoraPacket.Lora_Packet_S.soilmoisture=0;
            }
          else if(soilmoisturepercent >0 && soilmoisturepercent < 100){    
            LoraPacket.Lora_Packet_S.soilmoisture = soilmoisturepercent;
            }

          //ENVIO DEL DATAPACKET
          Serial.printf("X29;Data Packet sended;");
          Serial.printf("%d;",LoraPacket.Lora_Packet_S.satellite_id);
          Serial.printf("%d;",LoraPacket.Lora_Packet_S.packet_type);
          Serial.printf("%d;",LoraPacket.Lora_Packet_S.packet_id);
          Serial.printf("%d;",LoraPacket.Lora_Packet_S.node_id);
          Serial.printf("%d;",LoraPacket.Lora_Packet_S.timestamp);
          Serial.printf("%d;",LoraPacket.Lora_Packet_S.pos_x);
          Serial.printf("%d;",LoraPacket.Lora_Packet_S.pos_y);
          Serial.printf("%d;",LoraPacket.Lora_Packet_S.pos_z);
          Serial.printf("%d;",LoraPacket.Lora_Packet_S.temperature);
          Serial.println(LoraPacket.Lora_Packet_S.soilmoisture);
          send_datapacket = false; 
          expecting_CTS = false; 
          sensing_channel = false; 
          new_wait_time = false;
                                      
          turnOnRGB(COLOR_SEND,0);
          tiempo_envio = millis(); //Graba el tiempo actual
          Radio.Send((uint8_t *)LoraPacket.byteArray, sizeof(LoraPacket.byteArray));
          }
        
        state=LOWPOWER;
        break;
      
    case RX_ALOHA:
    
      if( millis() <= t_envio_com + ComA.Lora_ComA.ExperimentTime){      
          Radio.Rx(t_wait_time_ALOHA);
          state=LOWPOWER;
        }
          
      else{
         Serial.println("ALOHA experiment has finished");
         ProtocolAloha = false;
         min_one_beacon_received = false;
         delay(10000);
         state=RX; 
        } 
      break;

    case TX_ALOHA:
    
        LoraPacket.Lora_Packet_S.flag = 0;        
        LoraPacket.Lora_Packet_S.satellite_id = LoraB.Lora_BS.satellite_id;               //Elemento constante. Copia el satellite_id que le ha llegado del Beacon.
        LoraPacket.Lora_Packet_S.timestamp = millis() - Dif_t;                            //Recoge el tiempo en millis del experimeto.  
        LoraPacket.Lora_Packet_S.packet_type = ALOHA_PACKET_TYPE;                         //Tipo de paquete enviado: ALOHA(0);CSMA/CA(1) 
        LoraPacket.Lora_Packet_S.packet_id += 1;                                          //Cada paquete de datos enviado irá variando por unidad sumando 1 a cada ciclo del bucle                         
        LoraPacket.Lora_Packet_S.node_id = ID_NODE;                                       //Elemento constante
        LoraPacket.Lora_Packet_S.pos_x = POSITION_X;                                      //Posiciones del nodo.     
        LoraPacket.Lora_Packet_S.pos_y = POSITION_Y;                                      //Posiciones del nodo.   
        LoraPacket.Lora_Packet_S.pos_z = POSITION_Z;                                      //Posiciones del nodo.   

        //Lectura del sensor HDC1080 - Temperatura
        SensorTemperature = hdc1080.readTemperature();
        LoraPacket.Lora_Packet_S.temperature = SensorTemperature;
      
        //Lectura del sensor SoilMoisture
        soilMoistureValue = analogRead(ADC);  //put Sensor insert into soil
        soilmoisturepercent = map(soilMoistureValue, AirValue, WaterValue, 0, 100);
        if(soilmoisturepercent >= 100){
          LoraPacket.Lora_Packet_S.soilmoisture=100;}
        else if(soilmoisturepercent <=0){
          LoraPacket.Lora_Packet_S.soilmoisture=0;}
        else if(soilmoisturepercent >0 && soilmoisturepercent < 100){    
          LoraPacket.Lora_Packet_S.soilmoisture = soilmoisturepercent;}

        //ENVIO DEL DATAPACKET
        Serial.print("X10;Data Packet sended;");
        Serial.printf("%d;",LoraPacket.Lora_Packet_S.satellite_id);
        Serial.printf("%d;",LoraPacket.Lora_Packet_S.packet_type);
        Serial.printf("%d;",LoraPacket.Lora_Packet_S.packet_id);
        Serial.printf("%d;",LoraPacket.Lora_Packet_S.node_id);
        Serial.printf("%d;",LoraPacket.Lora_Packet_S.timestamp);
        Serial.printf("%d;",LoraPacket.Lora_Packet_S.pos_x);
        Serial.printf("%d;",LoraPacket.Lora_Packet_S.pos_y);
        Serial.printf("%d;",LoraPacket.Lora_Packet_S.pos_z);
        Serial.printf("%d;",LoraPacket.Lora_Packet_S.temperature);
        Serial.println(LoraPacket.Lora_Packet_S.soilmoisture);
        
        turnOnRGB(COLOR_SEND,0); //change rgb color
        
        tiempo_envio = millis();                                                          //Graba el tiempo actual para luego compararlo
        Radio.Send((uint8_t *)LoraPacket.byteArray, sizeof(LoraPacket.byteArray));
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
  
	if(ProtocolAloha == true){
	  turnOnRGB(0,0);
    state = RX_ALOHA;
	  }
  else{
    turnOnRGB(0,0);
    state = RX_CSMA_CA;
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
    Radio.Sleep( );
    turnOnRGB(COLOR_RECEIVED,0);
    
    //DATAPACKET
    if(payload[0] == 0){
              for(int z=0 ; z<rxSize ; z++){
                LoraPacket.byteArray[z] = payload[z];
                }
    Packet = P_DATAPACKET; 
    }

    //ACK
    if(payload[0] == 1){
           for(int z=0 ; z<rxSize ; z++){
              LoraACK.byteArray[z] = payload[z];
              }
    Packet = P_ACK; 
    }

    //BEACON
    if(payload[0] == 2){       
          for(int z=0; z < rxSize; z++){
            LoraB.byteArray[z]=payload[z];
            }
    min_one_beacon_received = true;
    Packet = P_BEACON;
    }

    //CTS        
    if(payload[0] == 3){
            for(int z=0 ; z<rxSize ; z++){
                LoraCTS.byteArray[z] = payload[z];
                }
    Packet = P_CTS;
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
    t_wait_time_ALOHA = ComA.Lora_ComA.WAIT_TIME_ALOHA;
    try_rec_ACK_max = ComA.Lora_ComA.TRY_ACK_MAX;
    
    Serial.println("CC1;Initializing Protocol ALOHA");
    Serial.printf("Experiment Time: %d\n", ComA.Lora_ComA.ExperimentTime);  
    Serial.printf("Time to send next packet: %d\n", t_to_next_packetA); 
    Serial.printf("Wait Time ALOHA: %d\n", t_wait_time_ALOHA);    
    Serial.printf("MAX try to receive ACK: %d\n", try_rec_ACK_max);                  
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
    Time_NAV_RTS = ComC.Lora_ComC.T_NAV_RTS;
    Time_NAV_CTS = ComC.Lora_ComC.T_NAV_CTS;
    SIFS = ComC.Lora_ComC.T_SIFS;
    try_rec_ACK_max = ComC.Lora_ComC.TRY_ACK_MAX;
    t_wait_time = ComC.Lora_ComC.WAIT_TIME;
    SENS_TIMEOUT_VALUE = ComC.Lora_ComC.SENS_TIME;
    
    Serial.println("CC2;Initializing Protocol CSMA_CA");
    Serial.printf("Experiment Time: %d\n", ComC.Lora_ComC.ExperimentTime);  
    Serial.printf("Time to send next packet: %d\n", t_to_next_packetC); 
    Serial.printf("NAV RTS: %d\n", Time_NAV_RTS);
    Serial.printf("NAV CTS: %d\n", Time_NAV_CTS);
    Serial.printf("SIFS: %d\n", SIFS);
    Serial.printf("MAX try to receive ACK: %d\n", try_rec_ACK_max);
    Serial.printf("Wait Time: %d\n", t_wait_time);
    Serial.printf("Sensing Time: %d\n", SENS_TIMEOUT_VALUE);
    Serial.printf("Time of the Beacon repetition: %d\n", ComC.Lora_ComC.T_BEACON);                       
    protocol = CSMA_CA;
    }

    if(min_one_beacon_received == true){
      switch(protocol){
        
        case ALOHA:
          if(millis() <= t_envio_com + ComA.Lora_ComA.ExperimentTime){
              
              switch(Packet){
                
                //El paquete es un Beacon
                case P_BEACON:
                tiempo_exp = millis();
                Dif_t = tiempo_exp - (LoraB.Lora_BS.timestamp + 88);
                Serial.print("X34;Dif_t;;;;;;;;;;;;;;;;"); 
                Serial.println(Dif_t);
                Serial.print("X01;Beacon received;");                       
                Serial.printf("%d; ; ; ;",LoraB.Lora_BS.satellite_id);
                Serial.println(millis() - Dif_t);  
                state = TX_ALOHA;
                break; 
              
              //Mensaje recibido es un ACK 
               case P_ACK:
                 if (min_one_beacon_received == false){
                    Serial.print("X02;Received packet is an ACK but the node has not yet received a beacon; ; ;");               
                    Serial.printf("%d;",LoraACK.Lora_ACK_S.packet_id);                 
                    Serial.printf("%d;",LoraACK.Lora_ACK_S.node_id);
                    Serial.println(millis()-Dif_t);
                    state = RX_ALOHA;          
                    }
                    
                 else { 
                   if(LoraACK.Lora_ACK_S.node_id == ID_NODE){
                        Serial.print("X04;ACK packet has been received within the waiting time;");   
                        Serial.printf("%d;",LoraACK.Lora_ACK_S.satellite_id);
                        Serial.printf("%d;",LoraACK.Lora_ACK_S.packet_type);
                        Serial.printf("%d;",LoraACK.Lora_ACK_S.packet_id);
                        Serial.printf("%d;",LoraACK.Lora_ACK_S.node_id);                     
                        Serial.printf("%d; ; ; ; ; ; ;",millis() - Dif_t);
                        Serial.println(LoraACK.Lora_ACK_S.free_slots);
                        try_rec_ACK = 0;
                        T_b = 0;
                        delay(t_wait_time_ALOHA);
                        delay(t_to_next_packetA);
                        state = TX_ALOHA;              
                    }
                    
                    else{
                        Serial.print("X03;Received ACK message does not correspond to the node; ; ;");               
                        Serial.printf("%d;",LoraACK.Lora_ACK_S.packet_id);                 
                        Serial.printf("%d;",LoraACK.Lora_ACK_S.node_id);
                        Serial.println(millis()-Dif_t);  
                        wrong_packet = true;
                        }
                    }         
                break;
                       
              //Mensaje recibido es un DATAPACKET de otro nodo
               case P_DATAPACKET:   
                    if(min_one_beacon_received == false){
                      Serial.print("X05;Received packet is a DataPacket but the node has not yet received a beacon; ; ;");               
                      Serial.printf("%d;",LoraPacket.Lora_Packet_S.packet_id);                 
                      Serial.println(LoraPacket.Lora_Packet_S.node_id);
                      state = RX_ALOHA;  
                      }
          
                    else{
                      Serial.print("X06;Packet discarded because it is a Data Packet; ; ;");               
                      Serial.printf("%d;",LoraPacket.Lora_Packet_S.packet_id);                 
                      Serial.printf("%d;",LoraPacket.Lora_Packet_S.node_id);  
                      Serial.println(millis()-Dif_t);
                      wrong_packet = true;
                      }
               break;
              }
              
              if(wrong_packet == true){
                  
                  wrong_packet = false;
                  tiempo_actual= millis();
                  timer = tiempo_actual - tiempo_envio;
                  
                  if(timer <= t_wait_time_ALOHA){
                    //Proceso de espera
                    while(timer <= t_wait_time_ALOHA){
                      tiempo_actual= millis();
                      timer = tiempo_actual - tiempo_envio;   
                      }            
                    Serial.print("X07;Wait Time exceeded. ACK packet has not been received; ; ; ; ;"); 
                    Serial.println(millis()-Dif_t);
                    
                    try_rec_ACK++;      
                    if (try_rec_ACK>=try_rec_ACK_max){
                      Serial.println("X09;ACK receive attempts exceeded");
                      try_rec_ACK = 0;                                      //Reinicio contador para el siguiente bucle.    
                      T_b = 0;
                      state = TX_ALOHA;      
                      } 
              
                    else{
                      //Procedemos a realizar el bucle del BackOff
                      R = random(0,(pow(2,try_rec_ACK)-1));
                      T_b = R * t_wait_time_ALOHA;
                      Serial.print("X08;New backoff time; ; ; ; ; ; ; ; ; ; ; ; ;");
                      Serial.println(T_b);
                      delay(T_b);
                      state = TX_ALOHA;
                      }  
                  }
                    
                  else{
                    Serial.print("X07;Wait Time exceeded. ACK packet has not been received; ; ; ; ;"); 
                    Serial.println(millis()-Dif_t);      
                    
                    if (try_rec_ACK>=try_rec_ACK_max){
                      Serial.println("X09;ACK receive attempts exceeded");            
                      try_rec_ACK = 0;                                      //Reinicio contador para el siguiente bucle.  
                      T_b = 0;
                      state = TX_ALOHA;      
                      } 
              
                    else{
                      //Procedemos a realizar el bucle del BackOff
                      R = random(0,(pow(2,try_rec_ACK)-1));                  //Igual encontramos con el problema random. Igual requiere poner una randomSeed()
                      T_b = R * t_wait_time_ALOHA;
                     Serial.print("X08;New backoff time; ; ; ; ; ; ; ; ; ; ; ; ;");
                      Serial.println(T_b);
                      delay(T_b);
                      state = TX_ALOHA;
                    }
                 }
              }
            }
            
          else{
            Serial.println("ALOHA experiment has finished");
            ProtocolAloha = false;
            min_one_beacon_received = false;
            delay(10000);
            
            state = RX;
            }
          break;
          
        case CSMA_CA:
            
          if( millis() <= t_envio_com + ComC.Lora_ComC.ExperimentTime){
            switch(Packet){   
            
            //El paquete es un Beacon
            case P_BEACON:
              tiempo_exp = millis();
              Dif_t = tiempo_exp - (LoraB.Lora_BS.timestamp + 88);
              Serial.print("X34;Dif_t;;;;;;;;;;;;;;;;;"); 
              Serial.println(Dif_t);
              Serial.print("X27;Beacon received;");                       
              Serial.printf("%d; ; ; ;",LoraB.Lora_BS.satellite_id);
              Serial.println(millis() - Dif_t);  
                 
              sensing_channel = true;
              RTS = false;
              send_datapacket = false;
              try_rec_ACK = 0;
              state = RX_CSMA_CA;
            break;
    
            //Mensaje recibido es un RTS 
            case P_RTS:       
                if(sensing_channel == true){
                  Serial.print("X13;RTS received while sensing; ; ;");
                  Serial.printf("%d;",LoraRTS.Lora_RTS_S.packet_id);  
                  Serial.printf("%d;",LoraRTS.Lora_RTS_S.node_id);
                  Serial.println(millis()-Dif_t);
                  
                  Serial.println("X35;Delay NAV RTS");             
                  delay (Time_NAV_RTS);
                  state = RX_CSMA_CA; 
                  }
                    
                else{
                  Serial.print("X12;RTS from other node received; ; ;");
                  Serial.printf("%d;",LoraRTS.Lora_RTS_S.packet_id);  
                  Serial.printf("%d;",LoraRTS.Lora_RTS_S.node_id); 
                  Serial.println(millis()-Dif_t);
                  
                  new_waiting_time = t_wait_time - (millis() - t_start_wait_time);
                  new_wait_time = true;
                  
                  if(new_waiting_time <= 0){
                      if(expecting_CTS == true){
                        Serial.println("XX1;WaitTime exceeded. CTS has not been received");
                      }
                      else{
                        Serial.println("XX2;WaitTime exceeded. ACK has not been received");
                        }
                      new_wait_time = false;
                      wrong_packet = true;
                   }
                  else{
                      state = RX_CSMA_CA;
                    }
                  }
              break;
            
            //Mensaje recibido es un CTS 
            case P_CTS:
                if(sensing_channel == true){
                  Serial.print("X14;CTS received while sensing; ; ;");
                  Serial.printf("%d;",LoraCTS.Lora_CTS_S.packet_id);  
                  Serial.printf("%d;",LoraCTS.Lora_CTS_S.node_id);
                  Serial.println(millis()-Dif_t); 
                  Serial.println("X36;Delay NAV CTS");
                  delay (Time_NAV_CTS);
                  state = RX_CSMA_CA; 
                  }
                    
                else{
                  if(expecting_CTS == true){
    
                    if(LoraCTS.Lora_CTS_S.node_id == ID_NODE){
                      Serial.print("X17;CTS received;");
                      Serial.printf("%d;",LoraCTS.Lora_CTS_S.satellite_id);
                      Serial.printf("%d;",LoraCTS.Lora_CTS_S.packet_type);
                      Serial.printf("%d;",LoraCTS.Lora_CTS_S.packet_id);
                      Serial.printf("%d;",LoraCTS.Lora_CTS_S.node_id);                     
                      Serial.printf("%d; ; ; ; ; ; ; ; ; ;",millis() - Dif_t);
                      Serial.println(LoraCTS.Lora_CTS_S.NAV_CTS);
                      send_datapacket = true;
                      state = TX_CSMA_CA;
                      }
                    else{
                      Serial.print("X16;Wrong CTS received; ; ;");
                      Serial.printf("%d;",LoraCTS.Lora_CTS_S.packet_id);
                      Serial.printf("%d;",LoraCTS.Lora_CTS_S.node_id);
                      Serial.println(millis()-Dif_t);
                      wrong_packet = true;
                      }
                    }
                    
                  else {
                    Serial.print("X15;We have received a CTS when we expected an ACK; ; ;");
                    Serial.printf("%d;",LoraCTS.Lora_CTS_S.packet_id);
                    Serial.printf("%d;",LoraCTS.Lora_CTS_S.node_id);
                    Serial.println(millis()-Dif_t);
                    wrong_packet = true;
                    }
                    
                  }
            break; 
               
            //Mensaje recibido es un ACK 
            case P_ACK:
                if(sensing_channel == true){
                  Serial.print("X38;ACK received while sensing; ; ;");
                  Serial.printf("%d;",LoraACK.Lora_ACK_S.packet_id);
                  Serial.printf("%d;",LoraACK.Lora_ACK_S.node_id);
                  Serial.println(millis()-Dif_t);
                  TP = SIFS; 
                  RANDOM = random(TP,2*TP);                   
                  Serial.print("X37;NonPersistence Random Wait; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;");
                  Serial.println(RANDOM);
                  delay(RANDOM);
                   
                  state = RX_CSMA_CA; 
                  }
                
                else{
                  if(expecting_CTS == true){
                    Serial.print("X18;We were waiting for a CTS and an ACK has arrived; ; ;");
                    Serial.printf("%d;",LoraACK.Lora_ACK_S.packet_id);
                    Serial.printf("%d;",LoraACK.Lora_ACK_S.node_id);
                    Serial.println(millis()-Dif_t);
                    wrong_packet = true;
                    }
      
                  else{
                    if(LoraACK.Lora_ACK_S.node_id == ID_NODE){
                      Serial.print("X20;ACK packet has been received within the waiting time;");
                      Serial.printf("%d;",LoraACK.Lora_ACK_S.satellite_id);
                      Serial.printf("%d;",LoraACK.Lora_ACK_S.packet_type);
                      Serial.printf("%d;",LoraACK.Lora_ACK_S.packet_id);
                      Serial.printf("%d;",LoraACK.Lora_ACK_S.node_id);                     
                      Serial.printf("%d; ; ; ; ; ; ;",millis() - Dif_t);
                      Serial.println(LoraACK.Lora_ACK_S.free_slots);
                      
                      sensing_channel = true;
                      wrong_packet = false;

                      delay(SIFS);
                      delay(t_to_next_packetC);
                      try_rec_ACK = 0;
                      T_b = 0;
                      state = RX_CSMA_CA; 
                      }
                    else{
                      Serial.print("X19;Received ACK message does not correspond to the node; ; ;");
                      Serial.printf("%d;",LoraACK.Lora_ACK_S.packet_id);
                      Serial.printf("%d;",LoraACK.Lora_ACK_S.node_id);
                      Serial.println(millis()-Dif_t);
                      wrong_packet = true;
                      }
                    }
                  }
            break;        
                 
            //Mensaje recibido es un DataPacket de otro nodo
            case P_DATAPACKET:
                if(sensing_channel == true){
                  Serial.print("X21;DataPacket received while sensing; ; ;");
                  Serial.printf("%d;",LoraPacket.Lora_Packet_S.packet_id);  
                  Serial.printf("%d;",LoraPacket.Lora_Packet_S.node_id);
                  Serial.println(millis()-Dif_t);          
                  TP = SIFS; 
                  RANDOM = random(TP,2*TP);                 
                  delay(RANDOM);
                  Serial.print("X37;NonPersistence Random Wait; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;");
                  Serial.println(RANDOM);
                   
                  state = RX_CSMA_CA; 
                  }
                  
                else{
                  Serial.printf("X22;DataPacket from other node received; ; ;");
                  Serial.printf("%d;",LoraPacket.Lora_Packet_S.packet_id);  
                  Serial.printf("%d;",LoraPacket.Lora_Packet_S.node_id);
                  Serial.println(millis()-Dif_t);
                  wrong_packet = true;
                  }
            break; 
          }
        
             //Mensaje erroneo
             if(wrong_packet == true){
            
              wrong_packet = false;
              tiempo_actual= millis();
              timer = tiempo_actual - tiempo_envio;
              
              if(timer <= t_wait_time){
                while(timer <= t_wait_time){
                  tiempo_actual= millis();
                  timer = tiempo_actual - tiempo_envio;   //Cuando el valor sea igual al valor en milisegundos 
                  }
                
                if(expecting_CTS == true){
                  Serial.print("X23;Wait Time exceeded. CTS packet has not been received.; ; ; ; ;");
                  Serial.println(millis()-Dif_t); 
                  expecting_CTS = false;
                  
                  try_rec_ACK++;      
                  if (try_rec_ACK>=try_rec_ACK_max){
                    Serial.println("X25;ACK receive attempts exceeded");
                     
                    sensing_channel = true;
                    try_rec_ACK = 0;     
                    T_b = 0;
                    state = RX_CSMA_CA;      
                    } 
            
                  else{
                    //Procedemos a realizar el bucle del BackOff
                    R = random(0,(pow(2,try_rec_ACK)-1));                  //Igual encontramos con el problema random. Igual requiere poner una randomSeed()
                    T_b = R * t_wait_time;
                    Serial.print("X26;New BackOff time; ; ; ; ; ; ; ; ; ; ; ; ;");
                    Serial.println(T_b);
                    delay(T_b);
                     
                    sensing_channel = true;
                    state = RX_CSMA_CA;
                    }
                }
      
                else{
                  Serial.print("X24;Wait Time exceeded. ACK packet has not been received; ; ; ; ;");
                  Serial.println(millis()-Dif_t); 
                  try_rec_ACK++;      
                  if (try_rec_ACK>=try_rec_ACK_max){
                    Serial.println("X25;ACK receive attempts exceeded");
                     
                    sensing_channel = true;
                    try_rec_ACK = 0;     
                    T_b = 0;             
                    state = RX_CSMA_CA;      
                    } 
            
                  else{
                    //Procedemos a realizar el bucle del BackOff
                    R = random(0,(pow(2,try_rec_ACK)-1));                  
                    T_b = R * t_wait_time;
                    Serial.print("X26;New BackOff time; ; ; ; ; ; ; ; ; ; ; ; ;");
                    Serial.println(T_b);
                    delay(T_b);
                     
                    sensing_channel = true;
                    state = RX_CSMA_CA;
                    } 
                  }  
              }
      
              else{
                if(expecting_CTS == true){
                  Serial.print("X23;Wait Time exceeded. CTS packet has not been received.; ; ; ; ;");
                  Serial.println(millis()-Dif_t); 
                  expecting_CTS = false;
                  
                  try_rec_ACK++;      
                  if (try_rec_ACK>=try_rec_ACK_max){
                    Serial.println("X25;ACK receive attempts exceeded");
                     
                    sensing_channel = true;
                    try_rec_ACK = 0;     
                    T_b = 0;
                    
                    state = RX_CSMA_CA;      
                    } 
            
                  else{
                    //Procedemos a realizar el bucle del BackOff
                    R = random(0,(pow(2,try_rec_ACK)-1));                 
                    T_b = R * t_wait_time;
                    Serial.print("X26;New BackOff time; ; ; ; ; ; ; ; ; ; ; ; ;");
                    Serial.println(T_b);
                    delay(T_b);
                     
                    sensing_channel = true;
                    state = RX_CSMA_CA;
                    }
                  }
      
                else{
                  Serial.print("X24;Wait Time exceeded. ACK packet has not been received; ; ; ; ;");
                  Serial.println(millis()-Dif_t);
                  try_rec_ACK++;      
                  if (try_rec_ACK>=try_rec_ACK_max){
                    Serial.println("X25;ACK receive attempts exceeded");
                     
                    sensing_channel = true;
                    try_rec_ACK = 0;     
                    T_b = 0;
                    state = RX_CSMA_CA;      
                    } 
            
                  else{
                    //Procedemos a realizar el bucle del BackOff
                    R = random(0,(pow(2,try_rec_ACK)-1));                 
                    T_b = R * t_wait_time;
                    Serial.print("X26;New BackOff time; ; ; ; ; ; ; ; ; ; ; ; ;");
                    Serial.println(T_b);
                    delay(T_b);
                     
                    sensing_channel = true;
                    state = RX_CSMA_CA;
                    } 
                  } 
             }
          }   
     }
          
          else{
            Serial.println("CSMA_CA experiment has finished");
            ProtocolAloha = false;
            min_one_beacon_received = false;
            delay(10000);
            state = RX;
            }   
          break;    
        }
    }

    else{
      state = RX;
      }
}

void OnRxTimeout(void){
  Radio.Sleep( );
  switch(protocol){
    case ALOHA:
    if( millis() <= t_envio_com + ComA.Lora_ComA.ExperimentTime){
      
      Serial.print("X11;RX Timeout. ACK packet has not been received; ; ; ; ;");
      Serial.println(millis()-Dif_t); 
      try_rec_ACK++; 
           
      if (try_rec_ACK>=try_rec_ACK_max){
        Serial.println("X09;ACK receive attempts exceeded");
        try_rec_ACK = 0;    //Reinicio contador para el siguiente bucle.  
        T_b = 0;
        state = TX_ALOHA;      
       } 
    
      else{
        //Procedemos a realizar el bucle del BackOff
        R = random(0,(pow(2,try_rec_ACK)-1));
        T_b = R * t_wait_time;
        Serial.print("X26;New BackOff time; ; ; ; ; ; ; ; ; ; ; ; ;");
        Serial.println(T_b);
        delay(T_b);
          
        state = TX_ALOHA;
        }
      }
   
    else{
      Serial.println("ALOHA experiment has finished");
      ProtocolAloha = false;
      min_one_beacon_received = false;
      try_rec_ACK = 0; 
      T_b = 0;
      delay(10000);
      state = RX;
      }
    break;
    
    case CSMA_CA:
    if( millis() <= t_envio_com + ComC.Lora_ComC.ExperimentTime){
      if(sensing_channel == true){
        sensing_channel = false;
        Serial.print("X32;The channel is free; ; ; ; ;");
        Serial.println(millis()-Dif_t);
        RTS = true;
        state = TX_CSMA_CA;
      }
    
      else{
        if(expecting_CTS == true){
          Serial.print("X30;RX Timeout. CTS packet has not been received; ; ; ; ;");
          Serial.println(millis()-Dif_t); 
          expecting_CTS = false;
          new_wait_time = false;
          
          try_rec_ACK++;      
          if (try_rec_ACK>=try_rec_ACK_max){
            Serial.println("X25;ACK receive attempts exceeded");
             
            sensing_channel = true;
            try_rec_ACK = 0;     
            T_b = 0;
            state = RX_CSMA_CA;
            } 
    
          else{
            //Procedemos a realizar el bucle del BackOff
            R = random(0,(pow(2,try_rec_ACK)-1));                   
            T_b = R * t_wait_time;
            Serial.print("X26;New BackOff time; ; ; ; ; ; ; ; ; ; ; ; ;");
            Serial.println(T_b);
            delay(T_b);
             
            sensing_channel = true;
            state = RX_CSMA_CA;
            }
        }
  
        else{
        Serial.print("X31;RX Timeout. ACK packet has not been received; ; ; ; ;");
        Serial.println(millis()-Dif_t);
        new_wait_time = false;
        try_rec_ACK++;      
        
        if (try_rec_ACK>=try_rec_ACK_max){
          Serial.println("X25;ACK receive attempts exceeded");
           
          sensing_channel = true;
          try_rec_ACK = 0;     
          T_b = 0;
          state = RX_CSMA_CA;      
          } 
  
        else{
          //Procedemos a realizar el bucle del BackOff
          R = random(0,(pow(2,try_rec_ACK)-1));                  
          T_b = R * t_wait_time;
          Serial.print("X26;New BackOff time; ; ; ; ; ; ; ; ; ; ; ; ;");
          Serial.println(T_b);
          delay(T_b);
           
          sensing_channel = true;
          state = RX_CSMA_CA;
          } 
        }
      }
     }

    else{
      Serial.println("CSMA_CA experiment has finished");
      ProtocolAloha = false;
      min_one_beacon_received = false;
      delay(10000);
      state = RX;
      }
    break;
   }
}
