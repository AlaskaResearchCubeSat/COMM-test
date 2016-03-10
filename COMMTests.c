#include <msp430.h>
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctl.h>
#include <terminal.h>
#include <ARCbus.h>
#include <SDlib.h>
#include <crc.h>
#include <commandLib.h>
#include <Error.h>
#include <UCA1_uart.h>
#include "COMMTests.h"
#include "COMM_errors.h"
#include "Radio_functions.h"

#define ARRAY_SIZE(a) (sizeof(a)/sizeof(a[0]))

COMM_STAT status;
short beacon_on=0, beacon_flag=0;

//reset a MSP430 on command
int resetCmd(char **argv,unsigned short argc){
  //force user to pass no arguments to prevent unwanted resets
  if(argc!=0){
    printf("Error : %s takes no arguments\r\n",argv[0]);
    return -1;
  }
  //print reset message
  puts("Initiating reset\r\n");
  //write to WDTCTL without password causes PUC
  WDTCTL=0;
  //Never reached due to reset
  puts("Error : Reset Failed!\r");
  return 0;
}

//make clocks available for measuring by outputting them to pins
int clkCmd(char *argv[],unsigned short argc){
    unsigned short bgnd=0,stop=0;
    P7OUT|=BIT6;
    if(argc!=0){
        //check for background clock test
        if(!strcmp("bgnd",argv[1])){
            //notify user
            printf("Starting Background clock test\r\n");
            bgnd=1;
        //check for end of background clock test
        }else if(!strcmp("stop",argv[1])){
            //notify user
            printf("Stopping Background clock Test\r\n");
            stop=1;
        }
    }
    if(!stop){
        //output clocks on P5 pins
        P5SEL=BIT4|BIT5|BIT6;
        P5DIR=BIT4|BIT5|BIT6;
    }
    //if not stopping or background wait for input
    //from user so that measurements can be taken
    if(!stop && !bgnd){
        //flush keys
        //TODO: is this needed?
        //debugRxFlush();
        //notify user
        printf("Preforming Clock Test\r\nPress any key to terminate.");
        //wait for key press
        //UCA1_Getc();                    //this way sends CPU to low power mode causing incorrect MCLK readings
        while(UCA1_CheckKey()==EOF);      //this way blokcs lower priority tasks from running. not a big deal cause there is only one useful task
        //clock test done notify user
        printf("\r\nClock Test terminated.\r\n");
    }
    //if not background then clear select bits
    if(!bgnd){
        P5SEL&=~(BIT4|BIT5|BIT6);
        P5DIR&=~(BIT4|BIT5|BIT6);
    }
    //if in background mode clocks still being outputted now
    //otherwise clock pins are now inputs
    P7OUT=BIT7;
}

//drive waveforms on P1 0-7 and P3 0-2 and 4-5
int busCmd(char *argv[],unsigned short argc){
    //mask array, contains values to write to port
    const unsigned char m[]={(1<<0),(1<<1),(1<<2),(1<<3),(1<<4),(1<<5),(1<<6),(1<<7),(1<<2),(1<<1),(1<<0),(1<<5),(1<<4)};
    //port array contains address of port register to write to
    volatile unsigned char *(port[])={&P1OUT,&P1OUT,&P1OUT,&P1OUT,&P1OUT,&P1OUT,&P1OUT,&P1OUT,&P3OUT,&P3OUT,&P3OUT,&P3OUT,&P3OUT};
    //last port pointer used for cleanup
    volatile unsigned char *pLast=&P1OUT;
    int i=0;
    //check to see that both arrays are equally sized
    //this should be optimized out by most compilers
    P7OUT|=BIT5;
    if(ARRAY_SIZE(m)!=ARRAY_SIZE(port)){
        printf("Internal Error\r\n");
        return 1;
    }
    //inform the user that the test is starting
    printf("Preforming Bus Test\r\nPress any key to terminate.\r\n");
    //setup P1 for output
    P1OUT=0;
    P1REN=0;
    P1SEL=0;
    P1DIR=0xFF;
    //setup P3
    P3OUT=0;
    P3REN=0;
    P3SEL&=~(BIT0|BIT1|BIT2|BIT4|BIT5);
    P3DIR|= (BIT0|BIT1|BIT2|BIT4|BIT5);
    //wait for key press
    while(UCA1_CheckKey()==EOF){
        *port[i]=m[i];
        //if last port different from current port
        //clear pin values
        if(pLast!=port[i]){
            *pLast=0;
        }
        //remember last port
        pLast=port[i];
        i++;
        //wrap arround
        if(i>=ARRAY_SIZE(port)){
            i=0;
        }
    }
    //notify user of completion
    printf("Bus Test complete.\r\n");
    //set ports to input
    P1DIR=0;
    P3DIR=0;
    P7OUT=BIT7;
    return 0;
}

///////////////////// RADIO CMDS /////////////////////////////////////

//Turn on COMM TODO 
int onCmd(char *argv[],unsigned short argc){
  //output lower four bits COMM address (Ox13) to P7 LED's
  P7OUT=BIT1|BIT0;

  //Perhaps should set a register here that says we are commanded on.
  
  printf("COMM On.  Check LEDs: 0bxxxx0011\r\n");
}


//Turn off COMM TODO
int offCmd(char *argv[],unsigned short argc){
   //output lower four bits COMM address (Ox13) to P7 LED's
  P7OUT=0;

  //Perhaps should set a register here that says we are commanded off.
  
  printf("COMM Off.  Check LEDs: 0bxxxx0000\r\n");
}

//Retreive status COMM TODO
int statusCmd(char *argv[],unsigned short argc){

  int i;
   //flash lower four bits COMM address (Ox13) to P7 LED's 10 times
   P7OUT=BIT1|BIT0;
   for (i=0;i<10;i++){
	ctl_timeout_wait(ctl_get_current_time()+102);
	P7OUT=~(BIT1|BIT0);
	ctl_timeout_wait(ctl_get_current_time()+102);
	P7OUT=(BIT1|BIT0);
   }
  //Need to send back status through terminal.
  
  P7OUT=BIT1|BIT0; //finish present CDH address
  printf("COMM On.  Check LEDs: flashing 0bxxxx0011 - 0bxxxx1100\r\n");
}

int writeReg(char **argv,unsigned short argc){
  char radio, regaddr, regdata;
  if(argc>3){
    printf("Error : Too many arguments\r\n");
    return -1;
  }
  if(argc==3){
    if(!strcmp(argv[1],"CC1101")){
      radio = CC1101;
    } else if(!strcmp(argv[1],"CC2500")){
      radio = CC2500;
    } else {
      printf("Error: Unknown radio \"%s\"\r\n",argv[1]);
      return -2;
    }
    regaddr=strtoul(argv[2],NULL,0);
    regdata = strtoul(argv[3],NULL,0);
    Radio_Write_Registers(regaddr, regdata, radio);
    printf("Wrote register 0x%02x = 0x%02x [regaddr, regdata]\r\n", regaddr, regdata);
    return 0;
  }
  printf("Error : %s requires 3 arguments but %u given\r\n",argv[0],argc);
  return -2;
}

int readReg(char **argv,unsigned short argc){
  char result, radio, regaddr;
  if(argc>2){
    printf("Error : Too many arguments\r\n");
    return -1;
  }
  if(argc==2){
    if(!strcmp(argv[1],"CC1101")){
      radio = CC1101;
    } else if(!strcmp(argv[1],"CC2500")){
      radio = CC2500;
    } else {
      printf("Error: Unknown radio \"%s\"\r\n",argv[1]);
      return -2;
    }
    printf("Radio = %i\r\n",radio);
    regaddr=strtoul(argv[2],NULL,0);
    result= Radio_Read_Registers(regaddr, radio);
    printf("Register 0x%02x = 0x%02x [regaddr, regdata]\r\n", regaddr, result);
    return 0;
  }
  printf("Error : %s requires 2 arguments but %u given\r\n",argv[0],argc);
  return -2;
}

//checks MSP pins connected to the radio (NOTE only set up for 1 radio (CC1101))
int gdoin_cmd(char **argv,unsigned short argc){
unsigned char input;
//reset chip
//Radio_Write_Registers(30,)
  // tell user test is running 
  printf("Preforming GDO test.\r\n");
  //put radio in known state 5
  Radio_Write_Registers(TI_CCxxx0_IOCFG0,0x6F,CC1101); //(MSP pin 2.0 high GDO0)
  Radio_Write_Registers(TI_CCxxx0_IOCFG2,0x2F,CC1101); //(MSP pin 2.1 low GDO2)
  //check val
  input=P2IN;
  if (((input&0x03)%2)==BIT0){                    //mask out P2 pins 7-->3 and check bit 1
      Radio_Write_Registers(TI_CCxxx0_IOCFG0,0x2F,CC1101); //(MSP pin 2.0 low GDO0)
      Radio_Write_Registers(TI_CCxxx0_IOCFG2,0x6F,CC1101); //(MSP pin 2.1 High GDO2)
      printf("Port 2 pin 0 connected\r\nTest successful");
      input=P2IN;
      if((input&0x03)==BIT1){                //mask out P2 pins 7-->3 and check bit 2
        Radio_Write_Registers(TI_CCxxx0_IOCFG0,0x2F,CC1101); //(MSP pin 2.1 low GDO2)
        printf("Port 2 pin 1 connected\r\n");
      }
      else{
        printf("Test failed check radio GDO0 or MSP port 2 pin 1 \r\nPIN2 expected 0x%02X receved 0x%02X\r\n",BIT1,input);
      }
  }
  else{
    printf("Test failed check radio GDO2 or MSP port 2 pin 0 \r\nPIN2 expected 0x%02X receved 0x%02X\r\n",BIT0,input);
  }
  return 0;
}

//set up Radio GD01 and GD02 for SPI test
int spitest(char **argv,unsigned short argc){
unsigned char reg;
Radio_Write_Registers(0,0x6F,CC1101);
reg=Radio_Read_Registers(0,CC1101);
//check reg has been written 
  if (reg==0x6F){
    printf("SPI test sucsses!\r\n");
  }
  else {
    printf("SPI test failed check the radio pins (1,2,20) and MSP pins 5.(1,2,3)\r\n");
    printf("PIN2 reset to  0x%02X\r\n",P2IN);
   }
   //reset radio reg to 0
   Radio_Write_Registers(0,0x0,CC1101);
  return 0;
} 

int beaconCmd(char **argv,unsigned short argc){
  if(argc>1){
    printf("Error : Too many arguments\r\n");
    return -1;
  }
  if(argc==1){
    if(!strcmp(argv[1],"on")){
      beacon_on=1;
    }else if(!strcmp(argv[1],"off")){
      beacon_on=0;
    }else{
      printf("Error : Unknown argument \"%s\"\r\n",argv[1]);
      return -2;
    }
  }
  printf("Beacon : %s\r\n",beacon_on?"on":"off");
  return 0;
}

//table of commands with help
const CMD_SPEC cmd_tbl[]={{"help"," [command]",helpCmd},
                    {"reset","\r\n\t""Reset the MSP430",resetCmd},
                    {"spitest","Tests the SPI lines", spitest},
                    {"clk","[bgnd|stop]\r\n\r""Output the clock signals on port 5 pins 4-6",clkCmd},
                    {"bus","\r\n\t""Output pattern on BUS pins",busCmd},
                    {"on","[bgnd|stop]\r\n\t""Command COMM on",onCmd},
                    {"Off","port [port ...]\r\n\t""Command COMM off",offCmd},
                    {"StatusCOMM","\r\n\t""Get CDH status",statusCmd},
                    {"writeradioreg","[radio regaddr data]", writeReg},
                    {"readradioreg","[radio regaddr]", readReg},
                    {"gdoin","",gdoin_cmd},
                    {"beacon","[on|off]\r\n\t""Turn on/off status requests and beacon\r\n",beaconCmd},
                   //end of list

                   {NULL,NULL,NULL}};




