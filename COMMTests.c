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

int nullCmd(char **argv,unsigned short argc){
  printf("NULL \r\n");
  printf("Starting status %x \r\n",Radio_Read_Status(TI_CCxxx0_MARCSTATE,CC1101));

return 0;
}
 
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

///////////////////// RADIO CMDS /////////////////////////////////////
//done
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

//done
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
    //printf("Radio = %i\r\n",radio);
    regaddr=strtoul(argv[2],NULL,0);
    result= Radio_Read_Registers(regaddr, radio);
    //printf("Register 0x%02x = 0x%02x [regaddr, regdata]\r\n", regaddr, result);
    printf("Register 0x%02x = 0x%02x\r\n", regaddr, result);
    return 0;
  }
  printf("Error : %s requires 2 arguments but %u given\r\n",argv[0],argc);
  return -2;
}

//parsing the "witch radio cmd" passes back 1 for CC1101/ 0 for CC2500 (done)
int radio_cmd(char *arg){
int radio;
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

//who am i... passes back radio name as a string (done)
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
//TODO (update for second radio)
int gdoin_cmd(char **argv,unsigned short argc){
  unsigned char input;
  int radio;
  //reset chip
  // find the radio to run test on 
  if(argc>1){
    radio=radio_cmd(argv[1]);
    if(radio==-1){
      return -1;
    }
  }
  // tell user test is running 
  printf("Preforming GDO test\r\n");
  //put radio in known state 5
  Radio_Write_Registers(TI_CCxxx0_IOCFG0,0x6F,CC1101); //(MSP pin 2.0 low GDO0 CC1101)(MSP pin 2.2 Low GDO0 CC2500)
  Radio_Write_Registers(TI_CCxxx0_IOCFG2,0x2F,CC1101); //(MSP pin 2.1 High GDO2 CC1101)(MSP pin 2.3 High GDO2 CC2500)
  //check val
  input=P2IN;
  if (((input&0x03)%2)==BIT0){                    //mask out P2 pins 7-->3 and check bit 1
      Radio_Write_Registers(TI_CCxxx0_IOCFG0,0x2F,CC1101); //(MSP pin 2.0 low GDO0 CC1101)(MSP pin 2.2 Low GDO0 CC2500)
      Radio_Write_Registers(TI_CCxxx0_IOCFG2,0x6F,CC1101); //(MSP pin 2.1 High GDO2 CC1101)(MSP pin 2.3 High GDO2 CC2500)
      printf("Port 2 pin 0 connected\r\n");
      input=P2IN;
      if((input&0x03)==BIT1){                //mask out P2 pins 7-->3 and check bit 2
        Radio_Write_Registers(TI_CCxxx0_IOCFG0,0x2F,CC1101); //(MSP pin 2.1 High GDO2 CC1101)(MSP pin 2.3 High GDO2 CC2500)
        printf("Port 2 pin 1 connected\r\nTest successful\r\n");
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
//TODO (update for second radio)
int spitest_Cmd(char **argv,unsigned short argc){
unsigned char reg;
Radio_Write_Registers(TI_CCxxx0_FREQ0,0xAA,CC1101); //write 10's to freq
reg=Radio_Read_Registers(TI_CCxxx0_FREQ0,CC1101);
//check reg has been written 10.....
  if (reg==0xAA){
  }
  Radio_Write_Registers(TI_CCxxx0_FREQ0,0x55,CC1101); //write 01's to freq
reg=Radio_Read_Registers(TI_CCxxx0_FREQ0,CC1101);
//check reg has been written 01...
  if (reg==0x55){
    printf("SPI test success!\r\n");
  }
  else {
    printf("SPI test failed check the radio pins (1,2,20) and MSP pins 5.(1,2,3)\r\n");
    printf("PIN2 reset to  0x%02X\r\n",P2IN);
   }
   //reset radio reg to 0
   Radio_Write_Registers(0,0x0,CC1101);
  return 0;
} 

// beacons radio 
//TODO (update for second radio)
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

// streams data from radio
//TODO   (update for second radio)
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

//Select power output from the radio chip
//TODO (update for second radio)
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

//enable or disable the COMM radio amplifier [radio,on/off] 
//TODO  (update for second radio)
int amp_Cmd(char **argv,unsigned short argc){
    int radio;
    char state;

    if(argc>1){
        radio=radio_cmd(argv[1]);
        if(radio==-1){
          return -1;
        }
      
    //NOTES MSP switch connection "RF1_SW" on P6.0/A0 <-- CC1101 --> CC2500 P6.1/A1
        if(!strcmp(argv[2],"on")){
          // turn on amp 
          if(radio==0){
            state=P6OUT|RF_SW1;
          }
          if(radio==1){
            state=P6OUT|RF_SW2;
          }
        }
        else if(!strcmp(argv[2],"off")){
          // turn off amp
          if(radio==0){
           state=P6OUT&~RF_SW1;
          }
          if(radio==1){
           state=P6OUT&~RF_SW2;
          }
        }
          printf("The %s raido amp is %s\r\n",whoami_cmd(radio),(state)?"on":"off");
    }
      else {
         printf("Enter a valid input 'on' or 'off'\r\n");
         return 0;
      }
  return 0;
}

// read radio status 
//TODO  (test NC case)
int status_Cmd(char **argv,unsigned short argc){
char status1, status2, radio, state1, state2;
// state info
const char* statetbl[32]={"SLEEP","IDLE","XOFF","VCOON_MC","REGON_MC","MANCAL","VCOON","REGON","STARTCAL","BWBOOST","FS_LOCK","IFADCON","ENDCAL","RX","RX_END","RX_RST","TXRX_SWITCH","RXFIFO_OVERFLOW","FSTXON","TX","TX_END","RXTX_SWITCH","TXFIFO_UNDERFLOW"};
// read 0x00 --> 0x2E
 status1=Radio_Read_Status(TI_CCxxx0_MARCSTATE,CC1101); // get status of CC1101
 status2=Radio_Read_Status(TI_CCxxx0_MARCSTATE,CC2500); // get status of CC2500
 state1=status1&(~(BIT7|BIT6|BIT5)); //get state of CC1101
 state2=status2&(~(BIT7|BIT6|BIT5)); //get state of CC2500
  if(0x00==state1){
   printf("Radio CC1011 is in the SLEEP state or may be unconnected");
  }
  else if(0x00==state2){
   printf("Radio CC2500 is in the SLEEP state or may be unconnected");
  }
  else{
  // store stat stuff
    printf("The status of the CC1101 is %s.\r\n",statetbl[status1]);
    printf("The state of the CC1101 is %i.\r\n",state1);
    printf("The status of the CC1101 is %s.\r\n",statetbl[status2]);
    printf("The state of the CC2500 is %i.\r\n",state2);
  }
return 0;
}

//turn off auto gain cmd  
//TODO  UDATE TO WRITE REG
int AGC_Cmd(char **argv,unsigned short argc){
  unsigned char AGC;
  // UNFINISHED
  const SYM_ADDR AGCSym[]= {{"NORMAL",TI_CCxxx0_SRES},       // Normal operation. Always adjust gain when required.
                           {"FSYNC",TI_CCxxx0_SFSTXON},     // The gain setting is frozen when a sync word has been found.
                           {"FANALOGUE",TI_CCxxx0_SFSTXON}, // Manually freeze the analogue gain setting and continue to adjust the digital gain.
                           {"FALL",TI_CCxxx0_SFSTXON},      // Manually freezes both the analogue and the digital gain setting. Used for manually overriding the gain.
                           {NULL,0xFF}};                 


  if((argc>1)&&(!(strcmp((argv[1]),"CC1101"))||(!(strcmp(argv[1],"CC2500"))))){ // check inputs

    AGC=Radio_Read_Registers(TI_CCxxx0_AGCTEST,radio_cmd(argv[1]));
    printf("The AGC test reg is %hhi\n",AGC);
    if(!strcmp(argv[2],"on")){
      
    }
    if(!strcmp(argv[2],"off")){
      
    }
    return 0;
  }
   else
    printf("Enter the valid input [radio , on/off]");
 return 0;  
}
 
//Strobes [RADIO] [CMD]
int strobe_cmd(char **argv,unsigned short argc){
  int radio,status;
  unsigned char strobe;
  // create struct for look up table 
  const SYM_ADDR strobeSym[]= {{"SRES",TI_CCxxx0_SRES},       // Reset chip.
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
  // input checking
  if((sizeof(argc)>1)&&(!(strcmp(argv[1],"CC1101"))||!(strcmp(argv[1],"CC2500")))){ 
    radio=radio_cmd(argv[1]);                     // assign radio 
    strobe=I2C_addr_lookup(argv[2],strobeSym);    //grab strobe address from table ,unsigned char I2C_addr_lookup(const char *str,const SYM_ADDR *syms);
    Radio_Strobe(strobe,radio);                   //strobe radio 
    status=status_Cmd(argv,argc);                 //check radio status
    printf("%s Strobe command sent.\r\n",argv[2]);
    if(strobe==0xFF){
      printf("Invalid strobe name.\r\nstrobe [radio] [strobe name]\r\n");
      return 0;
    }
  }
  else  
    printf("Enter valid input arguments [radio strobe].\r\n");
return 0;
}



// transmition and bandwidth check !
//TODO  (update for second radio)
int freq_cmd(char**argv,unsigned short argc){
  long fcar=437565000,fh,fl,fo,delta; // use long or freq will not fit
  int change,dsign;
  char radio;
  char * end;
  unsigned char val;

  radio =radio_cmd(argv[1]);
  fh=strtol(argv[2],&end,10);       // convert string user input to long int val for high freq input
  fl=strtol(argv[3],&end,10);
  if(radio!=255){                  // check radio input, unknown radio is 255
    printf("\r\n##########\r\ninput paramaters are fH %li and fL %li\r\n",fh,fl);
    fo=(fh+fl)/2;  // calc input center freq
    printf("The input center frequency (fo) is %li\r\n",fo);
    printf("High low frequency delta is %li\r\n",llabs(fh-fl)); 
    delta=fcar-fo; // calc diff in center freq's 
    printf("Delta of the center frequencys is %li\r\n",llabs(delta));

 //write reg 0x0f 0x63 where 0xf is the center freq reg and can change by steps of 400 Hz Adjusted in loop
    if(llabs(delta)>397){  //(fxosc/2^16)=396.7285
      change=delta/(RF_OSC_F/65536);    //calc change
      fcar=fcar*(RF_OSC_F/65536);  //(fxosc/2^16)*FERQ[23:0] --> carrier frequency should ==f01+-400 

      if(delta>=0){
        dsign=1;  // sign of delta is positive 
        val=0x63+change; // how many "ticks" to change the default ref 0x63 by.
      }
      else{
        dsign=0;  //sign of delta is negative 
        val=0x63-change; 
      }

      printf("\r\n##########\r\nAdjusted center frequency projection from %li to --> %li\r\n",fo,(dsign)?fo-(change*400):fo+(change*400)); // (dsign)?... selects the correct opp depending on the sign of delta
      printf("Changing reg 0x0f from 0x%02x --> 0x%02X\r\n",Radio_Read_Registers(0x0F,radio),val);
      Radio_Write_Registers(TI_CCxxx0_FREQ0,val,radio); // write reg
      printf("reg 0x0F writen as 0x%02X. \r\nRead back as 0x%02X.\r\n",val,Radio_Read_Registers(0x0F,radio));
    }
    else{
      printf("Frequency Test passed \r\n");
    }
  }
  else 
    printf("bad input\r\n");
return 0;
}

//LED strobe on plugin (done)
LED_cmd(char** argv, unsigned short argc){
  P7DIR=0xFF;
  P7OUT=0x00;
  while(async_CheckKey()==EOF){
    P7OUT=P7OUT+1;
    ctl_timeout_wait(ctl_get_current_time()+50);
  }
  P7OUT = BUS_ADDR_COMM; // re-set LED to COMM addr
return 0;
}
                  

//table of commands with help
const CMD_SPEC cmd_tbl[]={{"help"," [command]",helpCmd},
                    {"null","null does nothing",nullCmd},
                    {"reset","\r\n\t""Reset the MSP430\n\r",resetCmd},
                    {"spitest","Tests the SPI lines\n\r", spitest_Cmd},
                    {"clk","[bgnd|stop]\r\n\r""Output the clock signals on port 5 pins 4-6\n\r",clkCmd},
                    {"writeReg","[radio regaddr data]\n\r", writeReg},
                    {"readreg","[radio regaddr]\n\r", readReg},
                    {"gdoin","[radio]<-- defult set to CC1101\r\nPulses gdoin ports on the radio\r\n",gdoin_cmd},
                    {"beacon","[on|off]\r\n\t""Turn on/off status requests and beacon\r\n",beaconCmd},
                    {"stream","[zeros|ones|[value [val]]]\r\n""Stream data from radio\n\r",streamCmd},
                    {"power","[power]\r\n""get/set the radio output power\n\r",power_Cmd},
                    {"amp","Turns the COMM board CC1011 radio amplifier on [on] or off [off]\n\r",amp_Cmd},
                    {"status","",status_Cmd},
                    {"AGC","Auto gain command [off],[on]",AGC_Cmd},
                    {"strobe","[radio],[strobe name-->SRES,SFSTXON,SXOFF,SCAL,SRX,STX,SIDLE,SWOR,SPWD,SFRX,SFTX,SWORRST,SNOP]\n\r",strobe_cmd},
                    {"freq","[radio],[freq1],[freq2] \n\rCorrects brodcast freq of the radio to defined value\n\r",freq_cmd},
                    {"LED","pulses LED's as binary counter",LED_cmd},


                   //end of list

                   {NULL,NULL,NULL}};




