#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <mcp_can.h>
#include "vesc_can_bus_arduino.h"
#include <VescUart.h>

#include <Servo.h>

// put function declarations here:
int myFunction(int, int);

// #define USE_VESC

typedef enum{
  WAITING_FOR_MAGIC_NUMBER,
  WAITING_FOR_FRAME_ID,
  WAITING_FOR_END_NUMBER
}state_machine;

typedef enum{
  ASKING_FEEDBACK,
  WAITING_FOR_FEEDBACK,
  SENDING_TORQUE
}Ostate_machine;

/*uint16_t lookup_table[256] = {
		0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
		0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
		0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
		0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
		0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
		0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
		0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
		0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
		0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
		0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
		0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
		0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
		0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
		0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
		0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
		0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
		0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
		0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
		0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
		0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
		0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
		0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
		0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
		0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
		0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
		0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
		0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
		0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
		0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
		0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
		0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
		0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};


uint16_t compute_crc16(uint16_t * data) {
	# La table extraite de votre STM32
	

	uint16_t crc = 0x0000  ;
	uint8_t len=0;
	for byte in data:
		# L'index est calculé par : (byte XOR (crc haut))
		# Dans l'asm : eor.w r3, r3, r0, lsr #8
		index = byte ^ (crc >> 8)
		
		# On récupère la valeur de la table et on fait XOR 
		# avec le reste du CRC décalé à gauche
		# Dans l'asm : eor.w r0, r5, r3 (où r3 est r0 << 8)
		crc = (lookup_table[index] ^ (crc << 8)) & 0xFFFF
		len+=1
	return crc
  }*/

  
static const unsigned short crc16tab[256]= {
	0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
	0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
	0x1231,0x0210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,
	0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
	0x2462,0x3443,0x0420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,
	0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,
	0x3653,0x2672,0x1611,0x0630,0x76d7,0x66f6,0x5695,0x46b4,
	0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
	0x48c4,0x58e5,0x6886,0x78a7,0x0840,0x1861,0x2802,0x3823,
	0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
	0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0x0a50,0x3a33,0x2a12,
	0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
	0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0x0c60,0x1c41,
	0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
	0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0x0e70,
	0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
	0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,
	0x1080,0x00a1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
	0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,
	0x02b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
	0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,
	0x34e2,0x24c3,0x14a0,0x0481,0x7466,0x6447,0x5424,0x4405,
	0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,
	0x26d3,0x36f2,0x0691,0x16b0,0x6657,0x7676,0x4615,0x5634,
	0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,
	0x5844,0x4865,0x7806,0x6827,0x18c0,0x08e1,0x3882,0x28a3,
	0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,
	0x4a75,0x5a54,0x6a37,0x7a16,0x0af1,0x1ad0,0x2ab3,0x3a92,
	0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,
	0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0x0cc1,
	0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,
	0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0x0ed1,0x1ef0
};
  
unsigned short crc16_ccitt(const void *buf, int len)
{
	register int counter;
	register unsigned short crc = 0;
	for( counter = 0; counter < len; counter++)
		crc = (crc<<8) ^ crc16tab[((crc>>8) ^ *(char *)buf++)&0x00FF];
	return crc;
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint8_t Vpforce_buffer[256];
uint8_t Odrive_buffer[256];
uint8_t Vpforce_index=0;
uint8_t Odrive_index=0;
state_machine Vpforce_state;
Ostate_machine Odrive_state;

uint8_t writing_torque=0;
uint8_t writing_pos=0;
uint8_t new_torque=0;

float torque_x=0.0f;
float torque_y=0.0f;
float pos_x=0.0f;
float pos_y=0.0f;

char send_03[15]={0x00,0x03,0x01,0x30,0x03,0xa0,0x0e,0x07,0xf8,0xff,0x2b,0x34,0xcd,0xa7,0x00};
char Odrive_feedback[]="f 1\r\n";

HardwareSerial Vpforce(USART6);
HardwareSerial Odrive(USART1);
HardwareSerial debug(USART2);

VescUart UART;

int Servo_Y = PA0;
int Servo_X = PA1;
Servo servox;
Servo servoy;

static float input_rc_ratio = 0.5f;

static float force_multiplicator = 10.0f;

int X_pin = PA4;
int Y_pin = PA5;

static float X_input=0.0f;
static float Y_input=0.0f;

static float x_min=330.0f;
static float x_mid=527.0f;
static float x_max=800.0f;

static float y_min=337.0f;
static float y_mid=520.0f;
static float y_max=650.0f;

static float xx_table[7]={250.0f,350.0f,450.0f,520.0f,620.0f,710.0f,800.0f};
static float xy_table[7]={0.0f,100.0f,250.0f,510.0f,875.0f,975.0f,1024.0f};
static float yx_table[7]={330.0f,390.0f,450.0f,510.0f,620.0f,710.0f,800.0f};
static float yy_table[7]={0.0f,100.0f,250.0f,510.0f,850.0f,925.0f,1024.0f};


float map_output(float value, float * input, float * output)
{
  float outputvalue;
  uint8_t i=0;
  if(value<=input[0])
  {
    return output[0];
  }
  if(value>=input[6])
  {
    return output[6];
  }
  for (i=0;i<=6;i++)
  {
    if(value>=input[i] && value<input[i+1])
    {
      outputvalue=mapfloat(value,input[i],input[i+1],output[i],output[i+1]);
      break;
    }
  }
  return outputvalue;
}

// MCP_CAN CAN0(PA4);  // Set CS to pin PA4
// #define CAN0_INT PA2 // Set INT to pin PA2
void call()
{

}

static void __attribute__ ((optimize("-O0"))) task1(void *pvParameters) {
  uint8_t ch;
  Vpforce_state=WAITING_FOR_MAGIC_NUMBER;
  Vpforce_index=0;
  float old_pos_x=0.0f;
  float old_pos_y=0.0f;
  static uint8_t pos_y_h;
  static uint8_t pos_y_l;
  static uint8_t pos_x_h;
  static uint8_t pos_x_l;
  static uint8_t crc_h;
  static uint8_t crc_l;
  static uint8_t tor_y_h;
  static uint8_t tor_y_l;
  static uint8_t tor_x_h;
  static uint8_t tor_x_l;
  static int16_t tor_y;
  static char debug_buffer[32];
  static uint32_t tick = HAL_GetTick();
  static uint32_t oldtick = HAL_GetTick();
  static uint32_t diff_tick=0;
  static float current_pos=0.0f;
  static float old_torque_y=0.0f;
  uint8_t cho;
    float pos=0.0f;
  float speed=0.0f;
  char tor_buf[32];
  static float servo_out=90.0f;
  Odrive_index=0;
  for (;;) {
      if (Vpforce.available()>0)
      {
        while(Vpforce.available()>0)
        {
          ch=Vpforce.read();
          // debug.write(ch);
          switch(Vpforce_state)
          {
            case WAITING_FOR_MAGIC_NUMBER:
            {
              if(ch==0x00)
              {
                Vpforce_state=WAITING_FOR_FRAME_ID;
              }
              else
              {}
              break;
            }
            case WAITING_FOR_FRAME_ID:
            {
              if(ch==0x00)
              {
                Vpforce_state=WAITING_FOR_FRAME_ID;
              }
              else
              {
                Vpforce_state=WAITING_FOR_END_NUMBER;
                Vpforce_buffer[Vpforce_index]=ch;
                Vpforce_index++;
              }
              break;
            }
            case WAITING_FOR_END_NUMBER:
            {
              if(ch==0x00)
              {
                Vpforce_state=WAITING_FOR_MAGIC_NUMBER;
                Vpforce_index=0;
                switch(Vpforce_buffer[0])
                {
                  case 0x04:
                  {
                    if(writing_pos==0)//odrive is not currently writing his feedback
                    {
                      old_pos_x=pos_x;
                      old_pos_y=pos_y;
                    }
                    if(Vpforce_buffer[1]==0x31)
                    {

                    }
                    else if(Vpforce_buffer[1]==0x11)
                    {
                      #ifdef USE_VESC
                       if ( UART.getVescValues() ) {
                        current_pos=UART.data.pidPos;   
                        pos_y=4000.0f*(current_pos)/360.0f;
                       }
                      #else
                      for(uint8_t i=0;i<5;i++)
                      {
                        Odrive.write(Odrive_feedback[i]);
                      }
                      while(Odrive.available()>0)
                      {

                        cho=Odrive.read();
                        Odrive_buffer[Odrive_index]=cho;
                        // if(ch==0x0A)
                        // {
                        //   Odrive_index=0;
                        //   Odrive_state = SENDING_TORQUE;
                        // }
                        Odrive_index++;
                        // debug.write(ch);
                        if(cho=='\n')
                        {
                          int n = sscanf((char*)Odrive_buffer, "%f %f\r\n", &pos, &speed);
                          pos_y=pos*2000.0f;
                          Odrive_index=0;
                        }
                      }
                        // pos_y=4000.0f*(current_pos)/360.0f;
                      #endif
                      // tick = HAL_GetTick();
                      // debug.println(tick);
                      // tick =xTaskGetTickCount();
                      // debug.println(tick);
                      char header[2] = {0x21,0x70};
                      char dy[3]={0x00,0x00,0x00};
                      char dy_p[3]={0x01,0x01,0x05};
                      char footer[2] = {0x2a,0x30};

                      uint16_t crc=0;
                      // char data1[10]={0x01,0x30,0x03,0xa0,0x0e,0x07,0xf8,0xff,0x2b,0x34};//cda7 //0003217003c3010101052a30dac900
                      // crc=crc16_ccitt(&data1,10);
                      char data[10]={0x01,0x30,0x00,0xa0,0x0e,0x00,0xf8,0xff,0x2b,0x34};//cda7 //0003217003c3010101052a30dac900
                      crc=crc16_ccitt(&data,10);
                      // pos_y=3744.0f;
                      // pos_y=2331.67017f;
                      int16_t posy_int=(int16_t)pos_y;
                      char trame[15]={0x00,0x03,0x01,0x30,0x03,0xa0,0x0e,0x07,0xf8,0xff,0x2b,0x34,0xcd,0xa7,0x00};
                                    /*
                      y_2s=(y.to_bytes(2,byteorder="little", signed=True)).hex()
                      # print(y_2s)
                      trame = (header+'00'+y_2s+dy+footer)*/
                      pos_y_h=(uint8_t)(posy_int >> 8);
                      pos_y_l=(uint8_t)(posy_int);
                      data[3]=pos_y_l;
                      data[4]=pos_y_h;
                      crc=crc16_ccitt(&data,10);
                      crc_h=(uint8_t)(crc >> 8 );
                      crc_l=(uint8_t)(crc);
                      if(pos_y_h==0x00)
                      {
                        trame[4]=0x02;
                        trame[5]=pos_y_l;
                        trame[6]=pos_y_h;
                      }
                      else
                      {
                        trame[4]=0x03;
                        trame[5]=pos_y_l;
                        trame[6]=pos_y_h;
                      }
                      trame[12]=crc_l;
                      trame[13]=crc_h;
                      for(uint8_t i=0;i<=15;i++)
                      {
                        Vpforce.write(trame[i]);
                      }
                      // torque_y=0.1f;
                    }
                  }
                  break;
                  case 0x02:
                  {
                    
                    // for(uint8_t i=0;i<=6;i++)
                    // {
                    //   debug.write(Vpforce_buffer[i]);
                    // }
                    if(Vpforce_buffer[1]==0x20)
                    {

                    }
                    else if(Vpforce_buffer[1]==0x30)
                    {

                    }
                    // uint8_t nb = sprintf(debug_buffer,"0x02, %02x %02x %02x %02x \r\n",Vpforce_buffer[1],Vpforce_buffer[2],Vpforce_buffer[3],Vpforce_buffer[4]);

                    // for(uint8_t i=0;i<=nb;i++)
                    // {
                    //   debug.write(debug_buffer[i]);
                    // }

                    writing_torque=1;
                    torque_x=0.0f;
                    torque_y=0.0f;
                    tick = HAL_GetTick();
                    diff_tick=tick-oldtick;
                    // if(diff_tick>1)
                    {
                      debug.println("0x02");
                      debug.println(torque_y);
                      if(old_torque_y!=torque_y)
                      {
                        // servo.write(servo_out);
                        // UART.setDuty(torque_y);
                        #ifdef USE_VESC
                          UART.setCurrent(torque_y*3.0f);
                        #else
                          uint8_t nb = sprintf(tor_buf,"c 1 %f\r\n",torque_y);
                          for(uint8_t i=0;i<=nb;i++)
                          {
                            Odrive.write(tor_buf[i]);
                          }
                        #endif
                        old_torque_y=torque_y;
                      }
                      oldtick=tick;
                    }
                    // tick = HAL_GetTick();
                    // debug.println("0x02");
                    // debug.println(tick);
                    // debug.write(printf("%f\r",torque_y));
                    new_torque=1;

                  }
                  break;
                  case 0x03:
                  {
                    writing_torque=1;
                    // for(uint8_t i=0;i<=6;i++)
                    // {
                    //   debug.write(Vpforce_buffer[i]);
                    // }
                    // if(Vpforce_buffer[1]==0x20)
                    // {

                    // }
                    // else if(Vpforce_buffer[1]==0x30)
                    // {

                    // }
                    // uint8_t nb = sprintf(debug_buffer,"0x03, %02x %02x %02x %02x \r",Vpforce_buffer[1],Vpforce_buffer[2],Vpforce_buffer[3],Vpforce_buffer[4]);

                    // for(uint8_t i=0;i<=nb;i++)
                    // {
                    //   debug.write(debug_buffer[i]);
                    // }

                    if(Vpforce_buffer[1]==0xFF)
                    {

                    }
                    else if(Vpforce_buffer[1]==0x10 && Vpforce_buffer[3]==0x04 && Vpforce_buffer[4]==0x01)
                    {
                      tor_y_l=Vpforce_buffer[2];
                      tor_y = (int16_t)tor_y_l;
                      torque_y=(float)tor_y/1000.0f;
                      
                      tick = HAL_GetTick();
                      diff_tick=tick-oldtick;
                      // if(diff_tick>1)
                      {
                        debug.println("0x03");
                        debug.println(torque_y);
                        if(old_torque_y!=torque_y)
                        {
                          // UART.setDuty(torque_y);
                          servo_out=mapfloat(torque_y, -0.2f, 0.2f, 0.0f, 180.0f);
                          // servo.write(servo_out);
                          #ifdef USE_VESC
                            UART.setCurrent(torque_y*3.0f);
                          #else
                            uint8_t nb = sprintf(tor_buf,"c 1 %f\r\n",torque_y);
                            for(uint8_t i=0;i<=nb;i++)
                            {
                              Odrive.write(tor_buf[i]);
                            }
                          #endif
                          old_torque_y=torque_y;
                        }
                        oldtick=tick;
                      }
                      // debug.println("0x03");
                      // debug.write(printf("%f\r",torque_y));
                      new_torque=1;
                    }

                  }
                  break;
                  case 0x07:
                  {
                    writing_torque=1;
                    // for(uint8_t i=0;i<=6;i++)
                    // {
                    //   debug.write(Vpforce_buffer[i]);
                    // }
                    // if(Vpforce_buffer[1]==0x20)
                    // {

                    // }
                    // else if(Vpforce_buffer[1]==0x30)
                    // {

                    // }
                    // uint8_t nb = sprintf(debug_buffer,"0x07, %02x %02x %02x %02x \r",Vpforce_buffer[1],Vpforce_buffer[2],Vpforce_buffer[3],Vpforce_buffer[4]);

                    // for(uint8_t i=0;i<=nb;i++)
                    // {
                    //   debug.write(debug_buffer[i]);
                    // }

                    if(Vpforce_buffer[1]==0xFF)
                    {

                    }
                    else if(Vpforce_buffer[1]==0x10 && Vpforce_buffer[4]==0x01)
                    {
                      tor_y_h=Vpforce_buffer[3];
                      tor_y_l=Vpforce_buffer[2];
                      tor_y = (int16_t)tor_y_h<<8 | tor_y_l;
                      torque_y=(float)tor_y/1000.0f;
                      if(torque_y>20.0f || torque_y<-20.0f)
                      {
                        debug.write(printf("%f\r",torque_y));
                      }
                      tick = HAL_GetTick();
                      diff_tick=tick-oldtick;
                      // if(diff_tick>1)
                      {
                        debug.println("0x07");
                        debug.println(torque_y);
                        if(old_torque_y!=torque_y)
                        {
                          servo_out=mapfloat(torque_y, -0.2f, 0.2f, 0.0f, 180.0f);
                          // servo.write(servo_out);
                          // UART.setDuty(torque_y);
                          #ifdef USE_VESC
                            UART.setCurrent(torque_y*3.0f);
                          #else
                            uint8_t nb = sprintf(tor_buf,"c 1 %f\r\n",torque_y);
                            for(uint8_t i=0;i<=nb;i++)
                            {
                              Odrive.write(tor_buf[i]);
                            }
                          #endif
                          old_torque_y=torque_y;
                        }
                        oldtick=tick;
                      }
                      // debug.println("0x07");
                      
                      // debug.write(printf("%f\r",torque_y));
                    
                      new_torque=1;
                    }
                    else
                    {}
                  }
                  break;
                }
                writing_torque=0;
                memset(&Vpforce_buffer,0x00,256);
              }
              else
              {
                Vpforce_buffer[Vpforce_index]=ch;
                Vpforce_index++;
              }
              break;
            }
          }
        }
      }
      // vTaskDelay(1);
      // digitalWrite(LED1, HIGH);
      // vTaskDelay(1000);
      // digitalWrite(LED1, LOW);
  }
}
 
static void __attribute__ ((optimize("-O0"))) task_vp_servo(void *pvParameters) {
  uint8_t ch;
  Vpforce_state=WAITING_FOR_MAGIC_NUMBER;
  Vpforce_index=0;
  float old_pos_x=0.0f;
  float old_pos_y=0.0f;
  static uint8_t pos_y_h;
  static uint8_t pos_y_l;
  static uint8_t pos_x_h;
  static uint8_t pos_x_l;
  static uint8_t crc_h;
  static uint8_t crc_l;
  static uint8_t tor_y_h;
  static uint8_t tor_y_l;
  static uint8_t tor_x_h;
  static uint8_t tor_x_l;
  static int16_t tor_y;
  static int16_t tor_x;
  static char debug_buffer[32];
  static uint32_t tick = HAL_GetTick();
  static uint32_t oldtick = HAL_GetTick();
  static uint32_t diff_tick=0;
  static float current_pos=0.0f;
  static float old_torque_y=0.0f;
  static float old_torque_x=0.0f;
  uint8_t cho;
    float pos=0.0f;
  float speed=0.0f;
  char tor_buf[32];
  static float servo_out=90.0f;
  Odrive_index=0;

  static float servo_x=90.0f;
  static float servo_y=90.0f;
  static float servo_out_x=90.0f;
  static float servo_out_y=90.0f;

  for (;;) {
      if (Vpforce.available()>0)
      {
        while(Vpforce.available()>0)
        {
          ch=Vpforce.read();
          // debug.write(ch);
          switch(Vpforce_state)
          {
            case WAITING_FOR_MAGIC_NUMBER:
            {
              if(ch==0x00)
              {
                Vpforce_state=WAITING_FOR_FRAME_ID;
              }
              else
              {}
              break;
            }
            case WAITING_FOR_FRAME_ID:
            {
              if(ch==0x00)
              {
                Vpforce_state=WAITING_FOR_FRAME_ID;
              }
              else
              {
                Vpforce_state=WAITING_FOR_END_NUMBER;
                Vpforce_buffer[Vpforce_index]=ch;
                Vpforce_index++;
              }
              break;
            }
            case WAITING_FOR_END_NUMBER:
            {
              if(ch==0x00)
              {
                Vpforce_state=WAITING_FOR_MAGIC_NUMBER;
                Vpforce_index=0;
                switch(Vpforce_buffer[0])
                {
                  case 0x04:
                  {
                    if(writing_pos==0)//odrive is not currently writing his feedback
                    {
                      old_pos_x=pos_x;
                      old_pos_y=pos_y;
                    }
                    if(Vpforce_buffer[1]==0x31)
                    {
                      X_input=analogRead(X_pin);
                      // servo_x=mapfloat((float)X_input,x_min, x_max, 50.0f, 140.0f);
                      float mapped_X =map_output(X_input,xx_table,xy_table);
                      pos_x=mapfloat(mapped_X,0.0f, 1024.0f, 300.0f, 1200.0f);
                      servo_x=mapfloat(mapped_X,0.0f, 1024.0f, 65.0f, 125.0f);
                      // servox.write(servo_x);

                      uint16_t crc=0;
                      // char data1[10]={0x01,0x30,0x03,0xa0,0x0e,0x07,0xf8,0xff,0x2b,0x34};//cda7 //0003217003c3010101052a30dac900
                      // crc=crc16_ccitt(&data1,10);

                      //            char data[10]={0x01,0x30,0x00,0xa0,0x0e,0x00,0xf8,0xff,0x2b,0x34};//cda7 //0003217003c3010101052a30dac900
                      // char trame[15]={0x00,0x03,0x01,0x30,0x03,0xa0,0x0e,0x07,0xf8,0xff,0x2b,0x34,0xcd,0xa7,0x00};

                                 char data[10]={0x21,0x30,0x00,0x98,0x02,0x00,0xfe,0xff,0x2b,0x30};//cda7 //0003217003c3010101052a30dac900
                      char trame[15]={0x00,0x03,0x21,0x30,0x03,0x98,0x02,0x07,0xfe,0xff,0x2b,0x30,0x1e,0x38,0x00};
                      crc=crc16_ccitt(&data,10);
                      // pos_y=3744.0f;
                      // pos_y=2331.67017f;
                      int16_t posx_int=(int16_t)pos_x;
                                    /*
                      y_2s=(y.to_bytes(2,byteorder="little", signed=True)).hex()
                      # print(y_2s)
                      trame = (header+'00'+y_2s+dy+footer)*/
                      pos_x_h=(uint8_t)(posx_int >> 8);
                      pos_x_l=(uint8_t)(posx_int);
                      data[3]=pos_x_l;
                      data[4]=pos_x_h;
                      crc=crc16_ccitt(&data,10);
                      crc_h=(uint8_t)(crc >> 8 );
                      crc_l=(uint8_t)(crc);
                      if(pos_x_h==0x00)
                      {
                        trame[4]=0x02;
                        trame[5]=pos_x_l;
                        trame[6]=pos_x_h;
                      }
                      else
                      {
                        trame[4]=0x03;
                        trame[5]=pos_x_l;
                        trame[6]=pos_x_h;
                      }
                      trame[12]=crc_l;
                      trame[13]=crc_h;
                      for(uint8_t i=0;i<=15;i++)
                      {
                        Vpforce.write(trame[i]);
                      }

                    }
                    else if(Vpforce_buffer[1]==0x11)
                    {
                       // Read analog here
                      Y_input=analogRead(Y_pin);
                      // pos_y=(float)Y_input;
                      // servo_y=mapfloat((float)Y_input,y_min, y_max, 120.0f, 60.0f);
                      float mapped_Y =map_output(Y_input,yx_table,yy_table);
                      pos_y=mapfloat(mapped_Y,0.0f, 1024.0f, 300.0f, 1200.0f);
                      servo_y=mapfloat(mapped_Y,0.0f, 1024.0f, 120.0f, 60.0f);

                      uint16_t crc=0;
                      // char data1[10]={0x01,0x30,0x03,0xa0,0x0e,0x07,0xf8,0xff,0x2b,0x34};//cda7 //0003217003c3010101052a30dac900
                      // crc=crc16_ccitt(&data1,10);
                      char data[10]={0x01,0x30,0x00,0xa0,0x0e,0x00,0xf8,0xff,0x2b,0x34};//cda7 //0003217003c3010101052a30dac900
                      char trame[15]={0x00,0x03,0x01,0x30,0x03,0xa0,0x0e,0x07,0xf8,0xff,0x2b,0x34,0xcd,0xa7,0x00};
                      crc=crc16_ccitt(&data,10);
                      // pos_y=3744.0f;
                      // pos_y=2331.67017f;
                      int16_t posy_int=(int16_t)pos_y;
                      // char trame[15]={0x00,0x03,0x01,0x30,0x03,0xa0,0x0e,0x07,0xf8,0xff,0x2b,0x34,0xcd,0xa7,0x00};
                                    /*
                      y_2s=(y.to_bytes(2,byteorder="little", signed=True)).hex()
                      # print(y_2s)
                      trame = (header+'00'+y_2s+dy+footer)*/
                      pos_y_h=(uint8_t)(posy_int >> 8);
                      pos_y_l=(uint8_t)(posy_int);
                      data[3]=pos_y_l;
                      data[4]=pos_y_h;
                      crc=crc16_ccitt(&data,10);
                      crc_h=(uint8_t)(crc >> 8 );
                      crc_l=(uint8_t)(crc);
                      if(pos_y_h==0x00)
                      {
                        trame[4]=0x02;
                        trame[5]=pos_y_l;
                        trame[6]=pos_y_h;
                      }
                      else
                      {
                        trame[4]=0x03;
                        trame[5]=pos_y_l;
                        trame[6]=pos_y_h;
                      }
                      trame[12]=crc_l;
                      trame[13]=crc_h;
                      for(uint8_t i=0;i<=15;i++)
                      {
                        Vpforce.write(trame[i]);
                      }
                      // torque_y=0.1f;
                    }
                  }
                  break;
                  case 0x02:
                  {
                    if(Vpforce_buffer[1]==0x20)
                    {

                    }
                    else if(Vpforce_buffer[1]==0x30 && Vpforce_buffer[3]==0x04 && Vpforce_buffer[4]==0x01)
                    {
                      writing_torque=1;
                      torque_x=0.0f;
                      if(old_torque_x!=torque_x)
                      {
                        servox.write(servo_x);
                        old_torque_x=torque_x;
                      }
                      new_torque=1;
                    }
                    else if(Vpforce_buffer[1]==0x10 && Vpforce_buffer[3]==0x04 && Vpforce_buffer[4]==0x01)
                    {
                      writing_torque=1;
                      torque_y=0.0f;
                      if(old_torque_y!=torque_y)
                      {
                        servoy.write(servo_y);
                        old_torque_y=torque_y;
                      }
                      if(old_torque_x!=torque_x)
                      new_torque=1;
                    }
                    // writing_torque=1;
                    // torque_x=0.0f;
                    // torque_y=0.0f;
                    // tick = HAL_GetTick();
                    // diff_tick=tick-oldtick;
                    // // if(diff_tick>1)
                    // {
                    //   debug.println("0x02");
                    //   debug.println(torque_y);
                    //   if(old_torque_y!=torque_y)
                    //   {
                    //     servoy.write(servo_y);
                    //     // servo.write(servo_out);
                    //     // UART.setDuty(torque_y);
                    //     old_torque_y=torque_y;
                    //   }
                    //   if(old_torque_x!=torque_x)
                    //   {
                    //     servox.write(servo_x);
                    //     // servo.write(servo_out);
                    //     // UART.setDuty(torque_y);
                    //     old_torque_x=torque_x;
                    //   }
                    //   oldtick=tick;
                    // }
                    // // tick = HAL_GetTick();
                    // // debug.println("0x02");
                    // // debug.println(tick);
                    // // debug.write(printf("%f\r",torque_y));
                    // new_torque=1;

                  }
                  break;
                  case 0x03:
                  {
                    writing_torque=1;

                    if(Vpforce_buffer[1]==0xFF)
                    {

                    }
                    else if(Vpforce_buffer[1]==0x10 && Vpforce_buffer[3]==0x04 && Vpforce_buffer[4]==0x01)
                    {
                      tor_y_l=Vpforce_buffer[2];
                      tor_y = (int16_t)tor_y_l;
                      torque_y=(float)tor_y/1000.0f;
                      
                      tick = HAL_GetTick();
                      diff_tick=tick-oldtick;
                      // if(diff_tick>1)
                      {
                        debug.println("0x03");
                        debug.println(torque_y);
                        if(old_torque_y!=torque_y)
                        {
                          servo_out_y=servo_y-torque_y*force_multiplicator;
                          if(servo_out_y<=50.0f)
                          {
                            servo_out_y=50.0f;
                          }
                          else if(servo_out_y>=130.0f)
                          {
                            servo_out_y=130.0f;
                          }
                          servoy.write(servo_out_y);
                          old_torque_y=torque_y;
                        }
                        oldtick=tick;
                      }
                      // debug.println("0x03");
                      // debug.write(printf("%f\r",torque_y));
                      new_torque=1;
                    }
                    else if(Vpforce_buffer[1]==0x30 && Vpforce_buffer[3]==0x04 && Vpforce_buffer[4]==0x01)
                    {
                      tor_x_l=Vpforce_buffer[2];
                      tor_x = (int16_t)tor_x_l;
                      torque_x=(float)tor_x/1000.0f;
                      
                      tick = HAL_GetTick();
                      diff_tick=tick-oldtick;
                      // if(diff_tick>1)
                      {
                        debug.println("0x03");
                        debug.println(torque_y);
                        if(old_torque_x!=torque_x)
                        {
                          servo_out_x=servo_x+torque_x*force_multiplicator;
                          if(servo_out_x<=50.0f)
                          {
                            servo_out_x=50.0f;
                          }
                          else if(servo_out_x>=130.0f)
                          {
                            servo_out_x=130.0f;
                          }
                          servox.write(servo_out_x);
                          old_torque_x=torque_x;
                        }
                        oldtick=tick;
                      }
                      // debug.println("0x03");
                      // debug.write(printf("%f\r",torque_y));
                      new_torque=1;
                    }

                  }
                  break;
                  case 0x07:
                  {
                    writing_torque=1;
                    if(Vpforce_buffer[1]==0xFF)
                    {

                    }
                    else if(Vpforce_buffer[1]==0x10 && Vpforce_buffer[4]==0x01)
                    {
                      tor_y_h=Vpforce_buffer[3];
                      tor_y_l=Vpforce_buffer[2];
                      tor_y = (int16_t)tor_y_h<<8 | tor_y_l;
                      torque_y=(float)tor_y/1000.0f;
                      if(torque_y>20.0f || torque_y<-20.0f)
                      {
                        debug.write(printf("%f\r",torque_y));
                      }
                      tick = HAL_GetTick();
                      diff_tick=tick-oldtick;
                      // if(diff_tick>1)
                      {
                        debug.println("0x07");
                        debug.println(torque_y);
                        if(old_torque_y!=torque_y)
                        {
                          servo_out_y=servo_y-torque_y*force_multiplicator;
                          if(servo_out_y<=45.0f)
                          {
                            servo_out_y=45.0f;
                          }
                          else if(servo_out_y>=135.0f)
                          {
                            servo_out_y=135.0f;
                          }
                          servoy.write(servo_out_y);
                          old_torque_y=torque_y;
                        }
                        oldtick=tick;
                      }
                      // debug.println("0x07");
                      
                      // debug.write(printf("%f\r",torque_y));
                    
                      new_torque=1;
                    }
                    else if(Vpforce_buffer[1]==0x30 && Vpforce_buffer[4]==0x01)
                    {
                      tor_x_h=Vpforce_buffer[3];
                      tor_x_l=Vpforce_buffer[2];
                      tor_x = (int16_t)tor_x_h<<8 | tor_x_l;
                      torque_x=(float)tor_x/1000.0f;
                      if(torque_x>20.0f || torque_x<-20.0f)
                      {
                        debug.write(printf("%f\r",torque_x));
                      }
                      tick = HAL_GetTick();
                      diff_tick=tick-oldtick;
                      // if(diff_tick>1)
                      {
                        debug.println("0x07");
                        debug.println(torque_x);
                        if(old_torque_x!=torque_x)
                        {
                          servo_out_x=servo_x+torque_x*force_multiplicator;
                          if(servo_out_x<=45.0f)
                          {
                            servo_out_x=45.0f;
                          }
                          else if(servo_out_x>=135.0f)
                          {
                            servo_out_x=135.0f;
                          }
                          servox.write(servo_out_x);
                          old_torque_x=torque_x;
                        }
                        oldtick=tick;
                      }
                      // debug.println("0x07");
                      
                      // debug.write(printf("%f\r",torque_y));
                    
                      new_torque=1;
                    }
                    else
                    {}
                  }
                  break;
                }
                writing_torque=0;
                memset(&Vpforce_buffer,0x00,256);
              }
              else
              {
                Vpforce_buffer[Vpforce_index]=ch;
                Vpforce_index++;
              }
              break;
            }
          }
        }
      }
      // vTaskDelay(1);
      // digitalWrite(LED1, HIGH);
      // vTaskDelay(1000);
      // digitalWrite(LED1, LOW);
  }
}

static void __attribute__ ((optimize("-O0"))) task2(void *pvParameters) {
  uint8_t ch;
  Odrive_state = ASKING_FEEDBACK;
  Odrive_index=0;
  uint16_t loop_timeout=0;
  float pos=0.0f;
  float speed=0.0f;
  char tor_buf[32];

  for (;;) {
    switch(Odrive_state)
    {
      case ASKING_FEEDBACK:
      {
        for(uint8_t i=0;i<5;i++)
        {
          Odrive.write(Odrive_feedback[i]);
        }
        Odrive_state = WAITING_FOR_FEEDBACK;
      }
      break;
      case WAITING_FOR_FEEDBACK:
      {
        if (Odrive.available()>0)
        {
          while(Odrive.available()>0)
          {

            ch=Odrive.read();
            Odrive_buffer[Odrive_index]=ch;
            // if(ch==0x0A)
            // {
            //   Odrive_index=0;
            //   Odrive_state = SENDING_TORQUE;
            // }
            Odrive_index++;
            // debug.write(ch);
            if(ch=='\n')
            {
              int n = sscanf((char*)Odrive_buffer, "%f %f\r\n", &pos, &speed);
              pos_y=pos*2000.0f;
              Odrive_index=0;
              Odrive_state = SENDING_TORQUE;
            }
          }
        }
        else
        {
          if(loop_timeout>5000)
          {
            Odrive_state = ASKING_FEEDBACK;
          }
          else
          {
            loop_timeout++;
          }
        }
      }
      break;
      case SENDING_TORQUE:
      {
        uint8_t nb = sprintf(tor_buf,"c 1 %f\r\n",torque_y);
        for(uint8_t i=0;i<=nb;i++)
        {
          Odrive.write(tor_buf[i]);
        }
        Odrive_state = ASKING_FEEDBACK;
      }
      break;
    }
    if(new_torque==1)
    {
      //send odrive torque command
      new_torque=0;
    }
      vTaskDelay(1);
      // digitalWrite(LED2, HIGH);
      // vTaskDelay(200);
      // digitalWrite(LED2, LOW);
  }
}

static void __attribute__ ((optimize("-O0"))) task3(void *pvParameters) {
  uint8_t ch;
  Odrive_state = ASKING_FEEDBACK;
  Odrive_index=0;
  uint16_t loop_timeout=0;
  float pos=0.0f;
  float speed=0.0f;
  char tor_buf[32];
  static float old_pos=0.0f;
  static float current_pos=0.0f;
  static float old_diff_pos=0.0f;
  static float diff_pos=0.0f;
  static float pos_final=0.0f;

  for (;;) {
    switch(Odrive_state)
    {
      case ASKING_FEEDBACK:
      {
        if ( UART.getVescValues() ) {
          current_pos=UART.data.pidPos;
          // debug.println(UART.data.pidPos);
          diff_pos=current_pos-old_pos;
          // if(current_pos>180.0f)
          // {
          //   pos_final=2000.0f*(current_pos-360.0f)/360.0f;
          // }
          // else if(diff_pos<-180.0f)
          // {
          //   pos_final=2000.0f*(current_pos)/360.0f;
          // }
          // else
          {
            pos_final=4000.0f*(current_pos)/360.0f;
          }
          // pos_final+=2000.0f;
          // debug.println(pos_final);
          pos_y=pos_final;
          // Odrive_state = SENDING_TORQUE;
          UART.setCurrent(torque_y*3.0f);
        }
        else
        {
          // debug.println("timeout");
          // UART.setCurrent(0.0f);
        }
      }
      break;
    //   case WAITING_FOR_FEEDBACK:
    //   {
    //     if (Odrive.available()>0)
    //     {
    //       while(Odrive.available()>0)
    //       {

    //         ch=Odrive.read();
    //         Odrive_buffer[Odrive_index]=ch;
    //         // if(ch==0x0A)
    //         // {
    //         //   Odrive_index=0;
    //         //   Odrive_state = SENDING_TORQUE;
    //         // }
    //         Odrive_index++;
    //         // debug.write(ch);
    //         if(ch=='\n')
    //         {
    //           int n = sscanf((char*)Odrive_buffer, "%f %f\r\n", &pos, &speed);
    //           pos_y=pos*2000.0f;
    //           Odrive_index=0;
    //           Odrive_state = SENDING_TORQUE;
    //         }
    //       }
    //     }
    //     else
    //     {
    //       if(loop_timeout>5000)
    //       {
    //         Odrive_state = ASKING_FEEDBACK;
    //       }
    //       else
    //       {
    //         loop_timeout++;
    //       }
    //     }
    //   }
    //   break;
      case SENDING_TORQUE:
      {
        // debug.println(torque_y);
        UART.setCurrent(torque_y*3.0f);
        // vTaskDelay(1);
        Odrive_state = ASKING_FEEDBACK;
      }
      break;
    }
    if(new_torque==1)
    {
      //send odrive torque command
      new_torque=0;
    }
      vTaskDelay(1);
      // digitalWrite(LED2, HIGH);
      // vTaskDelay(200);
      // digitalWrite(LED2, LOW);
  }
}

void setup() {
  // put your setup code here, to run once:
  Vpforce.begin(3000000);
  // uart_attach_rx_callback(Vpforce,call);
  #ifdef USE_VESC
  Odrive.begin(460800);
  #else
  Odrive.begin(460800);
  #endif
  debug.begin(921600);

  UART.setSerialPort(&Odrive);

  servoy.attach(Servo_Y);
  servoy.write(90);
  servox.attach(Servo_X);
  servox.write(90);

  int result = myFunction(2, 3);
  xTaskCreate(task_vp_servo,"Task1",
              1000,NULL,tskIDLE_PRIORITY + 2,NULL);
  // xTaskCreate(task2,"Task2",
  //             1000,NULL,tskIDLE_PRIORITY + 2,NULL);
  vTaskStartScheduler();
  // Vpforce.onReceive(call);
}

void loop() {
  // uint8_t ch;
  // ch=Vpforce.read();
  // {
  //   debug.write(ch);
  // }
  // Vpforce.write("123456789\r\n");
  // Odrive.write("123456789\r\n");
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}

//need to implement rx callback for reading vpforce