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
#include "COMM.h"
#include "COMM_errors.h"
#include "Radio_functions.h"

#define ARRAY_SIZE(a) (sizeof(a)/sizeof(a[0]))


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

// parsing the "witch radio cmd" passes back 1 for CC1101/ 0 for CC2500
char radio_cmd(char *arg){
char radio;
    if(!strcmp(arg,"CC1101")){
      radio = CC1101;
    } else if(!strcmp(arg,"CC2500")){
    radio=CC2500;
    }else{
      //bad things
      printf("erorr unknown radio %s\r\n",arg);
      return -1;
      }
return radio;
}

//who am i... passes back radio name as a string
char * whoami_cmd(int r){
  char * radio;
  if(r==CC1101){
    radio="CC1101";
    return radio;
  }
  else if(r==CC2500){
    radio="CC2500";
    return radio;
    }
 return "err";
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

int streamCmd(char **argv,unsigned short argc){
  if(!strcmp(argv[1],"value")){
    data_mode=TX_DATA_PATTERN;
    data_seed=atoi(argv[2]);
  }else if(!strcmp(argv[1],"random")){
    data_mode=TX_DATA_RANDOM;
    if(argc==2){
      data_seed=atoi(argv[2]);
      if(data_seed==0){
        data_seed=1;
      }
    }else{
      data_seed=1;
    }
  }
  
  ctl_events_set_clear(&COMM_evt,CC1101_EV_TX_START,0);
  
  printf("Push any key to stop\r\n");
  getchar(); // waits for any char 
  
  Radio_Write_Registers(TI_CCxxx0_PKTCTRL0, 0x00, CC1101);         // Fixed byte mode
  state = TX_END;

  return 0;
}

int power_Cmd(char **argv,unsigned short argc){
  const int power_dbm[8]=             { -30, -20, -15, -10,   0,   5,   7,  10};
  const unsigned char power_PTABLE[8]={0x12,0x0E,0x1D,0x34,0x60,0x84,0xC8,0xC0};
  unsigned long input;
  int idx,i;
  int pwr;
  char *end;
  unsigned char read;

  if(argc>0){
    input=strtol(argv[1],&end,0);
    if(*end=='\0' || !strcmp(end,"dBm")){
      pwr=input;
    }else{
      printf("Error : unknown suffix \"%s\" for power \"%s\"\r\n",end,argv[1]);
      return -1;
    }
    for(i=0,idx=0;i<8;i++){
      //find the power that is closest to desired
      if(abs(power_dbm[i]-pwr)<abs(power_dbm[idx]-pwr)){
        idx=i;
      }
    }
    printf("Setting radio to %idBm\r\n",power_dbm[idx]);
    Radio_Write_Registers(TI_CCxxx0_PATABLE,power_PTABLE[idx],CC1101);
  }
  read=Radio_Read_Registers(TI_CCxxx0_PATABLE,CC1101);
  for(i=0,idx=-1;i<8;i++){
    if(power_PTABLE[i]==read){
      idx=i;
      break;
    }
  }
  if(idx==-1){
    printf("PTABLE = 0x%02X\r\n",read);
  }else{
    printf("PTABLE = %idBm = 0x%02X\r\n",power_dbm[idx],read);
  }
  return 0;
}

//enable or disable the COMM radio amplifier 
int amp_Cmd(char **argv,unsigned short argc){
//NOTES MSP switch connection "RF1_SW" on P6.0/A0 
if(argc>0){
  if(!strcmp(argv[1],"yes")){
    // turn on amp 
    P6OUT|=RF_SW1;  }
  else if(!strcmp(argv[1],"no")){
    // turn off amp
    P6OUT&=~RF_SW1;
  }
  else {
     printf("Enter a valid input 'on' or 'off'\r\n");
     return 0;
   }
}
printf("The CC1011 raido amp is %s\r\n",(P6OUT&RF_SW1)?"on":"off");
return 0;
}

// read radio status 
int status_Cmd(char **argv,unsigned short argc){
char status, radio,state,fifo;
const char *state_str=NULL;
// read 0x00 --> 0x2E
  if(argc>=1){
    radio=radio_cmd(argv[1]);
    if(radio==-1){
      return -1;
    }
  }
   status=Radio_Strobe(TI_CCxxx0_SNOP, radio);
   state=status&((BIT6|BIT5|BIT4))>>4;
   fifo=status&0xF;
  // read status from byte
  switch(state){
        case 0:
          state_str="IDLE";
          break;
        case 1:
          state_str="RX";
          break;
        case 2:
          state_str="TX";
          break;
        case 3:
          state_str="FSTXON";
          break;
        case 4:
          state_str="CALIBRATE";
          break;
        case 5:
          state_str="SETTLING";
          break;
        case 6:
          state_str="REFIFO_OVERFLOW";
          break;
        case 7:
          state_str="TXFIFO_UNDERFLOW";
          break;
          }
  printf("The status of the %s is %s\r\n",whoami_cmd(radio),state_str);
  printf("The fifo has %i bytes\r\n",fifo);
  printf("The chip is %s\r\n",status&BIT7?"not ready":"ready");
return 0;
}

//turn off auto gain cmd
//TODO
int AGC_Cmd(char **argv,unsigned short argc){
  unsigned char AGC;
  AGC=Radio_Read_Registers(TI_CCxxx0_AGCTEST,CC1101);
  printf("The AGC test reg is %hhi\n",AGC);
  if(!strcmp(argv[1],"on")){

  }
  if(!strcmp(argv[1],"off")){

  }

return 0;
} 

// Strobes 
//TODO
int strobe_cmd(char **argv,unsigned short argc){

// create struct for look up table 
const SYM_ADDR strobeSym[]= {{"SRES",TI_CCxxx0_SRES},      // Reset chip.
                              {"SFSTXON",TI_CCxxx0_SFSTXON},// Enable/calibrate freq synthesizer
                              {"SXOFF",TI_CCxxx0_SXOFF},    // Turn off crystal oscillator.
                              {"SCAL",TI_CCxxx0_SCAL},      // Calibrate freq synthesizer & disable
                              {"SRX",TI_CCxxx0_SRX},        // Enable RX.
                              {"STX",TI_CCxxx0_STX},        // Enable TX.
                              {"SIDLE",TI_CCxxx0_SIDLE},    // Exit RX / TX
                              {"SAFC",TI_CCxxx0_SAFC},      // AFC adjustment of freq synthesizer
                              {"SWOR",TI_CCxxx0_SWOR},      // Start automatic RX polling sequence
                              {"SPWD",TI_CCxxx0_SPWD},      // Enter pwr down mode when CSn goes hi
                              {"SFRX",TI_CCxxx0_SFRX},      // Flush the RX FIFO buffer.
                              {"SFTX",TI_CCxxx0_SFTX},      // Flush the TX FIFO buffer.
                              {"SWORRST",TI_CCxxx0_SWORRST},// Reset real time clock.
                              {"SNOP",TI_CCxxx0_SNOP},      // No operation.
                              {NULL,0xFF}};                 

if(radio_cmd){

}
//  Radio_Write_Burst_Registers(TI_CCxxx0_PATABLE, paTable_CC1101, paTableLen, CC1101);


return 0;
}
//table of commands with help
const CMD_SPEC cmd_tbl[]={{"help"," [command]",helpCmd},
                    {"reset","\r\n\t""Reset the MSP430",resetCmd},
                    {"spitest","Tests the SPI lines", spitest},
                    {"clk","[bgnd|stop]\r\n\r""Output the clock signals on port 5 pins 4-6",clkCmd},
                    {"bus","\r\n\t""Output pattern on BUS pins",busCmd},
                    {"writeradioreg","[radio regaddr data]", writeReg},
                    {"readradioreg","[radio regaddr]", readReg},
                    {"gdoin","",gdoin_cmd},
                    {"beacon","[on|off]\r\n\t""Turn on/off status requests and beacon\r\n",beaconCmd},
                    {"stream","[zeros|ones|[value [val]]]\r\n""Stream data from radio",streamCmd},
                    {"power","[power]\r\n""get/set the radio output power",power_Cmd},
                    {"amp","Turns the COMM board CC1011 radio amplifier on [on] or off [off]",amp_Cmd},
                    {"status","",status_Cmd},
                    {"AGC","Auto gain command [off],[on]",AGC_Cmd},
                    {"Strobe","[radio],[strobe name-->SRES,SFSTXON,SXOFF,SCAL,SRX,STX,SIDLE,SWOR,SPWD,SFRX,SFTX,SWORRST,SNOP]",strobe_cmd},
                   //end of list

                   {NULL,NULL,NULL}};




