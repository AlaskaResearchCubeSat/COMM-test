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
#include "COMM.h"

/*********************************************************** Using the Timer_A1 ***************************************************************
* DONT USE TIMER0_Ax_VECTOR !!! this interrupt is use in library code and will cause a collision 
* Use TIMERx_Ay_VECTOR x=2,3 & y=0,1
* TIMER0_Ax_VECTOR used in ARClib ? 
* TIMER1_Ax_VECTOR used in ????
**********************************************************************************************************************************************/
int example_timer_IR(char **argv,unsigned short argc){
  int timer_check;
  WDTCTL = WDTPW+WDTHOLD;                                   // Stop WDT
  P7DIR |= 0xFF;                                            // Setting port 7 to drive LED's (0xFF ==1111 1111)
  P7OUT = 0x00;                                             // Set all LED's on port 7 to start all off
//************************************ Set up clock [0] 
  TA2CTL |= TASSEL__ACLK | MC_2;                            // Setting Timer_A to ACLK(TASSEL_1) to continuous mode(MC_2)

//*********************************** Set timer interrupt enable [1] 
  TA2CCTL0 |= CCIE;                                          // Capture/compare interrupt enable #0
  TA2CCTL1 |= CCIE;                                          // Capture/compare interrupt enable #1

//*********************************** Set the timer count IR value [2] 
  TA2CCR0 = 10000;                                           // Timer0_A3 Capture/Compare @ 10000 counts
  TA2CCR1 = 1000;                                            // TA0IV_1 Capture/Compare @ 1000 counts

   while (1)                                                // poll in while loop until a key press
   {
      if ((timer_check=getchar()) != EOF)
     {
      break;                                                 // break out of loop if a key is pressed
     }
    ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS,0, 1<<15, CTL_TIMEOUT_DELAY, 1000); // wait in loop 
   }
  reset(0,ERR_SRC_CMD,CMD_ERR_RESET,0);                     // reset clock registers 
  return 0;
}

// ********************************* Timer_A0 interrupt code 
void Timer_A2_A0(void)__interrupt[TIMER2_A0_VECTOR]{     // Timer A0 interrupt service routine TA0IV_TA0IFG. 
  P7OUT^=BIT0; // toggle LEDs when IR is called
}

void Timer_A2_A1(void)__interrupt[TIMER2_A1_VECTOR]{     // Timer A0 interrupt service routine for capture comp 1 and 2
        P7OUT^=BIT1; // light LEDs
}
//*********************************************************************************** RADIO COMMANDS *****************************************************
/*
int writeReg(char **argv,unsigned short argc){
  char radio, regaddr, regdata;
  if(argc>3){ // input checking and set radio address 
    printf("Error : Too many arguments\r\n");
    return -1;
  }
  if(argc==3){
    if(!strcmp(argv[1],"CC2500_1")){
      radio_select = 0;
    } else if(!strcmp(argv[1],"CC2500_2")){
      radio_select = 1;
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
*/

int status_Cmd(char **argv,unsigned short argc){
char status1, status2, radio, state1, state2;
// state info
 const char* statetbl[32]={"SLEEP","IDLE","XOFF","VCOON_MC","REGON_MC","MANCAL","VCOON","REGON","STARTCAL","BWBOOST","FS_LOCK","IFADCON","ENDCAL","RX","RX_END","RX_RST","TXRX_SWITCH","RXFIFO_OVERFLOW","FSTXON","TX","TX_END","RXTX_SWITCH","TXFIFO_UNDERFLOW"};
// read 0x00 --> 0x2E
 status1=Radio_Read_Status(TI_CCxxx0_MARCSTATE,CC2500_1); // get status of CC1101
 status2=Radio_Read_Status(TI_CCxxx0_MARCSTATE,CC2500_2); // get status of CC2500
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

/*
// streams data from radio argv[1]=ADR ,argv[2]= mode, argv[3]= val
//TODO   (update for second radio)
int streamCmd(char **argv,unsigned short argc){
// input checking 
  if(!strcmp(argv[1],"value")){
    data_mode=TX_DATA_PATTERN;
    data_seed=atoi(argv[2]); // arg to stream (0xXX)
  }
  else if(!strcmp(argv[1],"random")){
    data_mode=TX_DATA_RANDOM;
    if(argc==2){
      data_seed=atoi(argv[2]);
      if(data_seed==0){
        data_seed=1;
      }
    }
    else{
      data_seed=1;
    }
  }
  
  ctl_events_set_clear(&COMM_sys_events,CC1101_EV_TX_START,0); // This has been hardcoded to 0 change "0" to address 
  
  printf("Push any key to stop\r\n");
  getchar(); // waits for any char 
  
  Radio_Write_Registers(TI_CCxxx0_PKTCTRL0, 0x00, 0);         // Fixed byte mode
  state = TX_END;

  return 0;
}
*/
//table of commands with help
const CMD_SPEC cmd_tbl[]={{"help"," [command]",helpCmd},
                   {"timer_IR","[time]...\r\n\tExample command to show how the timer can be used as an interupt",example_timer_IR},
                  // {"status","",status_Cmd},
                   //{"stream","[zeros|ones|[value [val]]]\r\n""Stream data from radio\n\r",streamCmd},
                   ARC_COMMANDS,CTL_COMMANDS,ERROR_COMMANDS,
                   //end of list
                   {NULL,NULL,NULL}};
