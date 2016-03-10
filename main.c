#include <msp430.h>
#include <stdio.h>
#include <string.h>
#include <ctl.h>
#include <UCA1_uart.h>
#include <terminal.h>
#include "timer.h"
#include "Radio_functions.h"

//task structure for idle task
CTL_TASK_t idle_task;

CTL_TASK_t terminal_task;

//stack for task
unsigned stack1[1+256+1];

//make printf send over UCA1
int __putchar(int ch){
  return UCA1_TxChar(ch);
}

int __getchar(void){
  return UCA1_Getc();
}


void initCLK(void){
  //set XT1 load caps, do this first so XT1 starts up sooner
  BCSCTL3=XCAP_0;
  //stop watchdog
  WDTCTL = WDTPW|WDTHOLD;
  //setup clocks

  //set DCO to 16MHz from calibrated values
  DCOCTL=0;
  BCSCTL1=CALBC1_16MHZ;
  DCOCTL=CALDCO_16MHZ;
}

void start_term(void *p){
  //start terminal task when done
  terminal(p);
}

void main(void){
  P7OUT=0xFF;
  P7DIR=0xFF;

  //initialize clocks
  initCLK();
  //setup timerA
  init_timerA();
  //initialize UART
  UCA1_init_UART();
 // init SPI and pins for radio 
  radio_init();
 //initialize tasking
  ctl_task_init(&idle_task, 255, "idle");  

  //start timerA
  start_timerA();

  //initialize stack
  memset(stack1,0xcd,sizeof(stack1));  // write known values into the stack
  stack1[0]=stack1[sizeof(stack1)/sizeof(stack1[0])-1]=0xfeed; // put marker values at the words before/after the stack

  P7OUT=BIT7;

  //create tasks
  ctl_task_run(&terminal_task,2,start_term,"COMM Test Program Ready","terminal",sizeof(stack1)/sizeof(stack1[0])-2,stack1+1,0);
  
  // drop to lowest priority to start created tasks running.
  ctl_task_set_priority(&idle_task,0);

  for(;;){
    LPM0;
  }
}



//==============[task library error function]==============

//something went seriously wrong
//perhaps should try to recover/log error
void ctl_handle_error(CTL_ERROR_CODE_t e){
  switch(e){
    case CTL_ERROR_NO_TASKS_TO_RUN: 
      __no_operation();
      //puts("Error: No Tasks to Run\r");
    break;
    case CTL_UNSUPPORTED_CALL_FROM_ISR: 
      __no_operation();
      //puts("Error: Wait called from ISR\r");
    break;
    case CTL_UNSPECIFIED_ERROR:
      __no_operation();
      //puts("Error: Unspesified Error\r");
    break;
    default:
      __no_operation();
      //printf("Error: Unknown error code %i\r\n",e);
  }
  //something went wrong, reset
  WDTCTL=0;
}
