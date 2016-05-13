#include <msp430.h>
#include <stdio.h>
#include <string.h>
#include <ctl.h>
#include <UCA1_uart.h>
#include <terminal.h>
#include "timer.h"
#include "Radio_functions.h"
#include "COMM.h"

//task structure for idle task
CTL_TASK_t idle_task;

CTL_TASK_t terminal_task;
CTL_TASK_t comm_task;

//stack for task
unsigned stack1[1+256+1];
unsigned comm_stack[1+256+1];

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
  ctl_task_run(&comm_task, 50, COMM_events, NULL, "COMM_events", sizeof(comm_stack)/sizeof(comm_stack[0])-2,comm_stack+1,0);
  ctl_task_run(&terminal_task,2,start_term,"COMM Test Program Ready","terminal",sizeof(stack1)/sizeof(stack1[0])-2,stack1+1,0);
  
  // drop to lowest priority to start created tasks running.
  ctl_task_set_priority(&idle_task,0);

  for(;;){
    LPM0;
  }
}




void Port2_ISR (void) __ctl_interrupt[PORT2_VECTOR]
{
  unsigned char flags=P2IFG;
  P2IFG&=~flags;
   if (flags & CC1101_GDO0) // GDO0 is set up to assert when RX FIFO is greater than FIFO_THR.  This is an RX function only
    {
        P2IFG &= ~CC1101_GDO0;
        ctl_events_set_clear(&COMM_evt,CC1101_EV_RX_READ,0);
    } 

    if (flags & CC1101_GDO2) //GDO2 is set up to assert when TX FIFO is above FIFO_THR threshold.  
                             //Interrups on falling edge, i.e. when TX FIFO falls below FIFO_THR
    {
        switch(state)
        {
            case IDLE:
                 P2IFG &= ~CC1101_GDO2;
                 break;

            case TX_START:  //Called on falling edge of GDO2, Tx FIFO < threshold, Radio in TX mode, Packet in progress
                 state = TX_RUNNING;
                 P2IFG &= ~CC1101_GDO2;
                 ctl_events_set_clear(&COMM_evt,CC1101_EV_TX_THR,0);
                 break;
            
            case TX_RUNNING: //Called on falling edge of GDO2, Tx FIFO < threshold, Radio in TX mode, Packet in progress
                 P2IFG &= ~CC1101_GDO2;
                 ctl_events_set_clear(&COMM_evt,CC1101_EV_TX_THR,0);
                 break;

            case TX_END:  //Called on falling edge of GDO2, Tx FIFO < threshold, Radio in TX mode, Last part of packet to transmit
                 state = IDLE;
                 P2IFG &= ~CC1101_GDO2;
                 ctl_events_set_clear(&COMM_evt,CC1101_EV_TX_END,0);
                 break;

            default:
              P2IFG &= ~CC1101_GDO2;
              break;          
  
        }
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
