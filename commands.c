/**********************************************************************************************************************************************
The commands.c file is for commands that will be displayed through the serial terminal. 
In order to add a command you must create a function as seen below.
Then function must be added to the "const CMD_SPEC cmd_tbl[]={{"help"," [command]",helpCmd}" table at the end of the file.
**********************************************************************************************************************************************/
#include <msp430.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <terminal.h>
#include <commandLib.h>
#include <stdlib.h>
#include <ARCbus.h>
#include <SDlib.h>
#include <i2c.h>
#include <Radio_functions.h>
#include <UCA2_uart.h>  
#include "COMM.h"
#include "AX25_EncodeDecode.h"
#include "COMM_Events.h"
#include "temp.h"

extern CTL_EVENT_SET_t COMM_evt; // define because this lives in COMM.c

//*********************************************************************************** RADIO COMMANDS *****************************************************
//NOTE do not define global vars in a local function ie "radio_select"
int writeReg(char **argv,unsigned short argc){
  char regaddr, regdata;  // expecting [radio] [address] [data]
  int radio_check;

  if(argc>3){ // input checking and set radio address 
    printf("Error : Too many arguments\r\n");
    return -1;
  }

  radio_check = set_radio_path(argv[1]);  // set radio_select  

   if (radio_check==-1) {
      printf("Error: Unknown radio \"%s\"\r\n",argv[1]);
      return -2;
    }
  else{
    //printf("Radio = %i\r\n",radio_select);
    regaddr=strtoul(argv[2],NULL,0);
    regdata = strtoul(argv[3],NULL,0);
    Radio_Write_Registers(regaddr, regdata, radio_select);
    //Radio_Write_Registers(regaddr, regdata, 1);
    printf("Wrote register 0x%02x = 0x%02x [regaddr, regdata]\r\n", regaddr, regdata);
    return 0;
  }

  printf("Error : %s requires 3 arguments but %u given\r\n",argv[0],argc);
  return -2;
}

int readReg(char **argv,unsigned short argc){
  char result, radio, regaddr;  // expecting [radio]  [address]
  int radio_check;

  if(argc>2){
    printf("Error : Too many arguments\r\n");
    return -1;
  }

  radio_check = set_radio_path(argv[1]);  // set radio_select  

   if (radio_check==-1) {
      printf("Error: Unknown radio \"%s\"\r\n",argv[1]);
      return -2;
    }
  else {
    printf("Radio = %i\r\n",radio_select);
    regaddr=strtoul(argv[2],NULL,0);
    result= Radio_Read_Registers(regaddr, radio_select);
    printf("Register 0x%02x = 0x%02x [regaddr, regdata]\r\n", regaddr, result);
  }
  return 0;
}

int status_Cmd(char **argv,unsigned short argc){
char status1, status2, radio, state1, state2;
// state info
 const char* statetbl[32]={"SLEEP","IDLE","XOFF","VCOON_MC","REGON_MC","MANCAL","VCOON","REGON","STARTCAL","BWBOOST","FS_LOCK","IFADCON","ENDCAL","RX","RX_END","RX_RST","TXRX_SWITCH","RXFIFO_OVERFLOW","FSTXON","TX","TX_END","RXTX_SWITCH","TXFIFO_UNDERFLOW"};
// read 0x00 --> 0x2E
 status1=Radio_Read_Status(TI_CCxxx0_MARCSTATE,CC2500_1); // get status of CC1101
 status2=Radio_Read_Status(TI_CCxxx0_MARCSTATE,CC2500_2); // get status of CC2500
 state1=status1&(~(BIT7|BIT6|BIT5)); //get state of CC2500_1
 state2=status2&(~(BIT7|BIT6|BIT5)); //get state of CC2500_2
  if(0x00==state1){
   printf("Radio CC1011 is in the SLEEP state or may be unconnected");
  }
  else if(0x00==state2){
   printf("Radio CC2500 is in the SLEEP state or may be unconnected");
  }
  else{
  // store stat stuff
    printf("The status of the CC2500_1 is %s.\r\n",statetbl[status1]);
    printf("The state of the CC2500_1 is %i.\r\n",state1);
    printf("The status of the CC2500_2 is %s.\r\n",statetbl[status2]);
    printf("The state of the CC2500_2 is %i.\r\n",state2);
  }
return 0;
} 

// stream [radio] [zeros|ones] [optional seed]
//TODO   (update for second radio)
int streamCmd(char **argv,unsigned short argc){
short radio_check;
// input checking
  // input checking for radio select
    radio_check = set_radio_path(argv[1]);  // set radio_select  
   if (radio_check==-1) {
      printf("Error: Unknown radio \"%s\"\r\n",argv[1]);
      return -2;
    }
  // input checking for streamed data
  if(!strcmp(argv[2],"value")){
    data_mode=TX_DATA_PATTERN;
    data_seed=atoi(argv[3]); // arg to stream (0xXX)
  }
  else if(!strcmp(argv[2],"random")){
    data_mode=TX_DATA_RANDOM;
    if(argc==3){
      data_seed=atoi(argv[3]);
      if(data_seed==0){
        data_seed=1;
      }
    }
    else{
      data_seed=1;
    }
  }
 
  // input case statment to pick from enum table in COMM.h
  ctl_events_set_clear(&COMM_evt,COMM_EVT_CC2500_1_TX_START,0); 
  
  printf("Push any key to stop\r\n");
  getchar(); // waits for any char 
  
  Radio_Write_Registers(TI_CCxxx0_PKTCTRL0, 0x00, radio_select);         // Fixed byte mode
  state = TX_END;

  return 0;
}


//Better power function using enum(set up in radio radiofunctions.h)
int powerCmd(char **argv,unsigned short argc)
{
int radio_level;
enum power_level power;
set_radio_path(argv[1]); //select the radio
power=strtoul(argv[2],NULL,0);
 if (power ==  power1){
  Radio_Write_Registers(TI_CCxxx0_PATABLE, 0x00, radio_select);
  printf("The power selected is -55 dBm\r\n" );
  printf("The radio selected is radio CC2500_%i radio \r\n", radio_select);
  }
  else if(power ==  power2){
  Radio_Write_Registers(TI_CCxxx0_PATABLE, 0x50, radio_select);
  printf("The power selected is -30 dBm\r\n" );
  printf("The radio selected is radio CC2500_%i radio \r\n", radio_select);
  }
    else if(power ==  power3){
  Radio_Write_Registers(TI_CCxxx0_PATABLE, 0x44, radio_select);
  printf("The power selected is -28 dBm\r\n" );
  printf("The radio selected is radio CC2500_%i radio \r\n", radio_select);
  }
    else if(power ==  power4){
  Radio_Write_Registers(TI_CCxxx0_PATABLE, 0xC0, radio_select);
  printf("The power selected is -26 dBm\r\n" );
  printf("The radio selected is radio CC2500_%i radio \r\n", radio_select);
  }
    else if(power ==  power5){
  Radio_Write_Registers(TI_CCxxx0_PATABLE, 0x84, radio_select);
  printf("The power selected is -24 dBm\r\n" );
  printf("The radio selected is radio CC2500_%i radio \r\n", radio_select);
  }
    else if(power ==  power6){
  Radio_Write_Registers(TI_CCxxx0_PATABLE, 0x81, radio_select);
  printf("The power selected is -22 dBm\r\n" );
  printf("The radio selected is radio CC2500_%i radio \r\n", radio_select);
  }
    else if(power ==  power7){
  Radio_Write_Registers(TI_CCxxx0_PATABLE, 0x46, radio_select);
  printf("The power selected is -20 dBm\r\n" );
  printf("The radio selected is radio CC2500_%i radio \r\n", radio_select);
  }
    else if(power ==  power8){
  Radio_Write_Registers(TI_CCxxx0_PATABLE, 0x93, radio_select);
  printf("The power selected is -18 dBm\r\n" );
  printf("The radio selected is radio CC2500_%i radio \r\n", radio_select);
  }
    else if(power ==  power9){
  Radio_Write_Registers(TI_CCxxx0_PATABLE, 0x55, radio_select);
  printf("The power selected is -16 dBm\r\n" );
  printf("The radio selected is radio CC2500_%i radio \r\n", radio_select);
  }
    else if(power ==  power10){
  Radio_Write_Registers(TI_CCxxx0_PATABLE, 0x8D, radio_select);
  printf("The power selected is -14 dBm\r\n" );
  printf("The radio selected is radio CC2500_%i radio \r\n", radio_select);
  }
    else if(power ==  power11){
  Radio_Write_Registers(TI_CCxxx0_PATABLE, 0xC6, radio_select);
  printf("The power selected is -12 dBm\r\n" );
  printf("The radio selected is radio CC2500_%i radio \r\n", radio_select);
  }
    else if(power ==  power12){
  Radio_Write_Registers(TI_CCxxx0_PATABLE, 0x97, radio_select);
  printf("The power selected is -10 dBm\r\n" );
  printf("The radio selected is radio CC2500_%i radio \r\n", radio_select);
  }
    else if(power ==  power13){
  Radio_Write_Registers(TI_CCxxx0_PATABLE, 0x6E, radio_select);
  printf("The power selected is -8 dBm\r\n" );
  printf("The radio selected is radio CC2500_%i radio \r\n", radio_select);
  }
    else if(power ==  power14){
  Radio_Write_Registers(TI_CCxxx0_PATABLE, 0x7F, radio_select);
  printf("The power selected is -6 dBm\r\n" );
  printf("The radio selected is radio CC2500_%i radio \r\n", radio_select);
  }
  else if(power ==  power15){
  Radio_Write_Registers(TI_CCxxx0_PATABLE, 0xA9, radio_select);
  printf("The power selected is -4 dBm\r\n" );
  printf("The radio selected is radio CC2500_%i radio \r\n", radio_select);
  }
    else if(power ==  power16){
  Radio_Write_Registers(TI_CCxxx0_PATABLE, 0xBB, radio_select);
  printf("The power selected is -2 dBm\r\n" );
  printf("The radio selected is radio CC2500_%i radio \r\n", radio_select);
  }
    else if(power ==  power17){
  Radio_Write_Registers(TI_CCxxx0_PATABLE, 0xFE, radio_select);
  printf("The power selected is -0 dBm\r\n" );
  printf("The radio selected is radio CC2500_%i radio \r\n", radio_select);
  }
    else if(power ==  power18){
  Radio_Write_Registers(TI_CCxxx0_PATABLE, 0xFF, radio_select);
  printf("The power selected is +1 dBm\r\n" );
  printf("The radio selected is radio CC2500_%i radio \r\n", radio_select);
  }
  else{
  printf("error not a valid power");
  }
}

int transmitTestCmd(char **argv,unsigned short argc){
  int i=0;
  data_mode=TX_DATA_BUFFER;
  for(i=0;i<19;i++){
    Tx1Buffer[i]=Packet_NoBit[i];
  }
//while(UCA2_CheckKey()==EOF){
  P7OUT ^= BIT1;  // flip a led every loop 
  ctl_events_set_clear(&COMM_evt,COMM_EVT_CC2500_1_TX_START,0);
  //BUS_delay_usec(50);  // delay in ms

//}
}

int transmit_test2(char **argv,unsigned short argc){

  int i=0;
  for(i=0;i<19;i++){
    Tx1Buffer[i]=Packet_NoBit[i];
  }
 while(UCA2_CheckKey()==EOF){
  RF_Send_Packet(Tx1Buffer, 19, CC2500_1);
  BUS_delay_usec(50);  // delay in ms 
 }
} 

/*
int SRES(char **argv,unsigned short argc)
TI_CCxxx0_SRES
*/

//table of commands with help
const CMD_SPEC cmd_tbl[]={{"help"," [command]",helpCmd},
                   {"status","",status_Cmd},
                   {"stream","[zeros|ones|[value [val]]]\r\n""Stream data from radio.\n\r",streamCmd},
                   {"writereg","Writes data to radio register\r\n [radio] [adress] [data].\n\r",writeReg},
                   {"readreg","reads data from a radio register\r\n [radio] [adrss].\n\r",readReg},
                   {"transmitTest","Testing tranmission of data\r\n [data][event].\n\r", transmitTestCmd},
                   {"power","Changes the transmit power of the radio [radio][power].\n\rex. CC2500_1 -24\n\r",powerCmd},
                   {"transmit_test2","Testing tranmission of data\r\n [data][event].\n\r", transmit_test2},
                   ARC_COMMANDS,CTL_COMMANDS,// ERROR_COMMANDS
                   //end of list
                   {NULL,NULL,NULL}};

