#include <msp430.h>
#include <ctl.h>
#include <stdio.h>
#include <ARCbus.h>
#include <string.h>
#include <SDlib.h>
#include "COMM.h"
#include "Radio_functions.h"
#include "i2c.h"

COMM_STAT status;
CTL_EVENT_SET_t COMM_sys_events;

short beacon_on=0, beacon_flag=0,data_mode=TX_DATA_BUFFER;
unsigned char data_seed, IMG_Blk, Tx1Buffer[600], RxBuffer[600], RxTemp[30];
unsigned int Tx1Buffer_Len, TxBufferPos=0, TxBytesRemaining, RxBuffer_Len=0, RxBufferPos=0, RxBytesRemaining, state;

//****************************************************** Subsystem Events ******************************************************************************
// 
//******************************************************************************************************************************************************
void sub_events(void *p) __toplevel{
  unsigned int e;
  int i, j, resp;
  unsigned char buf[BUS_I2C_HDR_LEN+sizeof(COMM_STAT)+BUS_I2C_CRC_LEN],*ptr;
  //source and type for SPI data
  char src,type;

  for(;;){
    e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&SUB_events,SUB_EV_ALL,CTL_TIMEOUT_NONE,0);

//******************* COMMAND TO POWER OFF??? NOTHING HAPPENING HERE **************
    if(e&SUB_EV_PWR_OFF){
      puts("System Powering Down\r\n");                                             //print message
      beacon_on = 0;
    }

//******************* COMMAND TO POWER ON??? NOTHING HAPPENING HERE **************
    if(e&SUB_EV_PWR_ON){
      puts("System Powering Up\r\n");                                               //print message
      beacon_on = 1;
    }

//******************* SEND COMM STATUS TO CDH ***************************
//NOTE when I2C commands are fixed this can be updated for use 
  /*  if(e&SUB_EV_SEND_STAT){
      puts("Sending status\r\n");                                                     //send status
      ptr=BUS_cmd_init(buf,CMD_COMM_STAT);                                            //setup packet
      for(i=0;i<sizeof(COMM_STAT);i++){                                               //fill in telemetry data
        ptr[i]=((unsigned char*)(&status))[i];
      }
      resp=BUS_cmd_tx(BUS_ADDR_CDH,buf,sizeof(COMM_STAT),0,BUS_I2C_SEND_FOREGROUND);  //send command
      ctl_events_set_clear(&COMM_sys_events,COMM_sys_events_STATUS_REQ,0);                          //call status update
      if(resp!=RET_SUCCESS){
        printf("Failed to send status %s\r\n",BUS_error_str(resp));
      }
    }
*/
// ******************* RECEIVING DATA OVER SPI *************************
    if(e&SUB_EV_SPI_DAT){
      puts("SPI data recived:\r");
      status.CC1101 = Radio_Read_Status(TI_CCxxx0_MARCSTATE, CC2500_1);
      printf("status.CC1101 = 0X%02X \r\n",status.CC1101);
      //First byte contains data type
      //Second byte contains sender address
      //Both bytes are removed before the data is passed on to COMM
      type=arcBus_stat.spi_stat.rx[0];
      src=arcBus_stat.spi_stat.rx[1];
      printf("SPI: type = 0x%02x, src = 0x%02x\r\n",type, src);
      //PrintBuffer(arcBus_stat.spi_stat.rx, arcBus_stat.spi_stat.len);  //TEST

         BUS_free_buffer_from_event();
       
    }

    if(e&SUB_EV_SPI_ERR_CRC){
      puts("SPI bad CRC\r");
      type=arcBus_stat.spi_stat.rx[0];
      src=arcBus_stat.spi_stat.rx[1];
      printf("SPI: type = 0x%02x, src = 0x%02x\r\n",type, src);
      printf("Trying to send beacon ON = %d FLAG = %d\r\n",beacon_on, beacon_flag);
    }
  }
}

//**************************************************************** radio commands ******************************************************************
//
//**************************************************************************************************************************************************
//gen data for transmit , random or pattern  
unsigned char tx_data_gen(unsigned char *dest,unsigned short size,int mode,unsigned char seed){
  int i;
  for(i=0;i<=size;i++){
    switch(mode){
      case TX_DATA_PATTERN:  //101... packet 
              dest[i]=seed;  // uses seed as patten              
      break;
      case TX_DATA_RANDOM:  //rand packet 
        seed=(seed>>1)^(-(seed&1)&0xB8);
        dest[i]=seed;
      break;
    }
  }
  return seed;
}

// for ease of terminal testing. default address is CC2500_2 = 1
int radio_select; // this is a global var
int set_radio_path(char *radio){
  //CC2500_1 = 0
  //CC2500_2 = 1 
  if (strcmp(radio,"CC2500_1")==0){
    radio_select=CC2500_1;
    return 0;
  }
  else{
    radio_select=CC2500_2;
    return 1;
  }

}



//handle COMM specific commands don't wait here.
int SUB_parseCmd(unsigned char src, unsigned char cmd, unsigned char *dat, unsigned short len){
  int i;
  
  switch(cmd){
    case CMD_BEACON_ON_OFF:
     beacon_flag = 1;
     return RET_SUCCESS;
  }
  return ERR_UNKNOWN_CMD;
}

void COMM_events(void *p) __toplevel{
  unsigned int e, count;
  int i, resp; 

    Reset_Radio(radio_select);                 // Reset Radios/initialize status

  __delay_cycles(800);                         // Wait for radio to be ready before writing registers.cc1101.pdf Table 13 indicates a power-on start-up time of 150 us for the crystal to be stable
                                               // After reset chip is in IDLE state
  Write_RF_Settings(radio_select);             // Write radios Settings

//  Radio_Write_Burst_Registers(TI_CCxxx0_PATABLE, paTable_CC1101, paTableLen, CC1101);
  Radio_Interrupt_Setup();
  //TODO Need to set up two radio setups. Might have to have them done separtely  
  Radio_Strobe(TI_CCxxx0_SRX, radio_select);          //Initialize CCxxxx in Rx mode

  // Need to wait first for RF on command from CDH.  Nothing happens until we get that command!
  // After RF on command we can send Beacon data.
  // Would like to create some test here where we read radio status and make sure it is in the correct state.

  ctl_events_init(&COMM_sys_events,0);                 //Initialize Event

  //endless loop
  for(;;){
    //wait for events
    e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&COMM_sys_events,COMM_EVT_ALL,CTL_TIMEOUT_NONE,0);

    //update status
    if(e&COMM_EVT_STATUS_REQ){

       status.CC1101 = Radio_Read_Status(TI_CCxxx0_MARCSTATE,radio_select);

       //check radio status
      if (Radio_Read_Status(TI_CCxxx0_MARCSTATE,radio_select) == 0x11)       // Check for RX FIFO overflow, if yes then flush dat buffer
      {
        Radio_Strobe(TI_CCxxx0_SFRX,radio_select); // do I need this?
        __delay_cycles(16000);              //what is the delay for?
        printf("Overflow Error, RX FIFO flushed, radio state now: ");
        printf("%x \r\n",Radio_Read_Status(TI_CCxxx0_MARCSTATE,radio_select));
      }

      if (Radio_Read_Status(TI_CCxxx0_MARCSTATE,radio_select) == 0x16)       // Check for TX FIFO underflow, if yes then flush dat buffer
      {
        Radio_Strobe(TI_CCxxx0_SFTX,radio_select);
        __delay_cycles(16000);              //what is the delay for?
        printf("Underflow Error, TX FIFO flushed, radio state now: ");
        printf("%x \r\n",Radio_Read_Status(TI_CCxxx0_MARCSTATE,radio_select));
      }
      //make sure radio is in receive mode
      Radio_Strobe(TI_CCxxx0_SRX,radio_select);

    }

    if(e & CC1101_EV_RX_READ){                  //READ RX FIFO
      // Triggered by GDO0 interrupt     
      // Entering here indicates that the RX FIFO is more than half filed.
      // Need to read RXThrBytes into RXBuffer then move RxBufferPos by RxThrBytes
      // Then wait until interrupt received again.
        Radio_Read_Burst_Registers(TI_CCxxx0_RXFIFO, RxTemp, RxThrBytes, radio_select);
        status.CC1101 = Radio_Read_Status(TI_CCxxx0_MARCSTATE,radio_select);
        P7OUT^=BIT0;  // flash LED on rx 
        //printf("Radio State: 0x%02x \n\r", status.CC1101);
    }

    if(e&CC1101_EV_TX_START){                 //INITIALIZE TX START
      //puts("TX Start\r\n");
      state = TX_START;
      TxBufferPos = 0;
      P1IFG &= ~CC2500_1_GDO2|CC2500_2_GDO2;
      P1IE |= CC2500_1_GDO2|CC2500_2_GDO2;             // Enable Port 2 GDO2

// Switch on the length of the initial packet.  
// If packet is > 256 then radio set up as INFINITE.
// If packet is > 64 and < 256 then radio is set up as Fixed Mode
// If packet is < 64 then radio is set up as Fixed Mode
/*
      if (Radio_Read_Status(TI_CCxxx0_MARCSTATE,CC1101) == 0x11)       // Check for RX FIFO overflow, if yes then flush dat buffer
      {
        Radio_Strobe(TI_CCxxx0_SFRX,CC1101); // do I need this?
        __delay_cycles(16000);              //what is the delay for?
        printf("Overflow Error, RX FIFO flushed, radio state now: ");
        printf("%x \r\n",Radio_Read_Status(TI_CCxxx0_MARCSTATE,CC1101));
      }

      if (Radio_Read_Status(TI_CCxxx0_MARCSTATE,CC1101) == 0x16)       // Check for TX FIFO underflow, if yes then flush dat buffer
      {
        Radio_Strobe(TI_CCxxx0_SFTX,CC1101);
        Radio_Strobe(TI_CCxxx0_SRX,CC1101); // do I need this?
        __delay_cycles(16000);              //what is the delay for?
        printf("Underflow Error, TX FIFO flushed, radio state now: ");
        printf("%x \r\n",Radio_Read_Status(TI_CCxxx0_MARCSTATE,CC1101));
      }
*/
      if(data_mode==TX_DATA_BUFFER){
        TxBytesRemaining = Tx1Buffer_Len;

        if(TxBytesRemaining > 64)
        {
          if(TxBytesRemaining > 256)
          {
            Radio_Write_Registers(TI_CCxxx0_PKTLEN, (Tx1Buffer_Len % 256), radio_select); // Pre-program the packet length
            Radio_Write_Registers(TI_CCxxx0_PKTCTRL0, 0x02, radio_select);                // Infinite byte mode
          }
          else
          {
            Radio_Write_Registers(TI_CCxxx0_PKTLEN, Tx1Buffer_Len, radio_select);  // Pre-program packet length
            Radio_Write_Registers(TI_CCxxx0_PKTCTRL0, 0x00, radio_select);         // Fixed byte mode
          }
          Radio_Write_Burst_Registers(TI_CCxxx0_TXFIFO, Tx1Buffer+TxBufferPos, 64, radio_select); // Write first 64 TX data bytes to TX FIFO
          TxBytesRemaining = TxBytesRemaining - 64;
          TxBufferPos = TxBufferPos + 64;
          state = TX_RUNNING;
        }
        else
        {
          Radio_Write_Registers(TI_CCxxx0_PKTLEN, Tx1Buffer_Len, radio_select);  // Pre-program packet length
          Radio_Write_Registers(TI_CCxxx0_PKTCTRL0, 0x00, radio_select);         // Fixed byte mode
          Radio_Write_Burst_Registers(TI_CCxxx0_TXFIFO, Tx1Buffer+TxBufferPos, Tx1Buffer_Len, radio_select); // Write TX data
          TxBytesRemaining = 0;
          TxBufferPos = TxBufferPos+Tx1Buffer_Len;
          state = TX_END;
        }
      }else{
          Radio_Write_Registers(TI_CCxxx0_PKTLEN, 1, radio_select); // Pre-program the packet length
          Radio_Write_Registers(TI_CCxxx0_PKTCTRL0, 0x02, radio_select);                // Infinite byte mode
          //fill buffer with data
          data_seed=tx_data_gen(Tx1Buffer,64,data_mode,data_seed);
          Radio_Write_Burst_Registers(TI_CCxxx0_TXFIFO, Tx1Buffer, 64, radio_select); // Write first 64 TX data bytes to TX FIFO
          state = TX_RUNNING;
      }

      Radio_Strobe(TI_CCxxx0_STX, radio_select);                                                  // Set radio state to Tx
   }

    if(e & CC1101_EV_TX_THR)
    {
      //printf("TX THR TxBytesRemaining = %d\r\n", TxBytesRemaining);        
      // Entering here indicates that the TX FIFO has emptied to below TX threshold.
      // Need to write TXThrBytes (30 bytes) from TXBuffer then move TxBufferPos by TxThrBytes
      // Then wait until interrupt received again or go to TX_END.
      if(data_mode==TX_DATA_BUFFER){
          if(TxBytesRemaining > TxThrBytes)
          {
             Radio_Write_Burst_Registers(TI_CCxxx0_TXFIFO, Tx1Buffer+TxBufferPos, TxThrBytes, radio_select);
             TxBufferPos += TxThrBytes;
             TxBytesRemaining = TxBytesRemaining - TxThrBytes;
             state = TX_RUNNING;
          }
          else
          {
             Radio_Write_Registers(TI_CCxxx0_PKTCTRL0, 0x00, radio_select); // Enter fixed length mode to transmit the last of the bytes
             Radio_Write_Burst_Registers(TI_CCxxx0_TXFIFO, Tx1Buffer+TxBufferPos, TxBytesRemaining, radio_select);
             TxBufferPos += TxBytesRemaining;
             TxBytesRemaining = 0;
             state = TX_END;
          }
        }
        else 
        {
         data_seed=tx_data_gen(Tx1Buffer,TxThrBytes,data_mode,data_seed);
         Radio_Write_Burst_Registers(TI_CCxxx0_TXFIFO, Tx1Buffer, TxThrBytes, radio_select);
         state = TX_RUNNING;
        }
       
        //printf("TX THR TxBytesRemaining = %d\r\n", TxBytesRemaining); 
    }
          
    if(e & CC1101_EV_TX_END)
    {
      // Entering here indicates that the TX FIFO has emptied to the last byte sent
      // No more bytes to send.
      // Need to change interrupts.        
      ctl_timeout_wait(ctl_get_current_time()+26);  //25 ms delay to flush 30 possible remaining bytes.  Before we turn off power amplifier
      P1IE &= ~CC2500_1_GDO2|CC2500_2_GDO2;                         // Disable Port 2 GDO2 interrupt
      P1IFG &= ~CC2500_1_GDO2|CC2500_2_GDO2;                        // Clear flag

      while (Radio_Read_Status(TI_CCxxx0_MARCSTATE,radio_select) == 0x13){
         __delay_cycles(500);
      }

      Radio_Write_Registers(TI_CCxxx0_PKTLEN,0xFF,radio_select);        //Reset PKTLEN
      Radio_Write_Registers(TI_CCxxx0_PKTCTRL0, 0x02, radio_select);    //Reset infinite packet length mode set
      printf("TX End\r\n");
      printf("TxBufferPos = %d\r\n", TxBufferPos);

      // Check for TX FIFO underflow, if yes then flush dat buffer
      if (Radio_Read_Status(TI_CCxxx0_MARCSTATE,radio_select) == 0x16)
      {
        Radio_Strobe(TI_CCxxx0_SFTX,radio_select);
        Radio_Strobe(TI_CCxxx0_SRX,radio_select);
        __delay_cycles(16000); //what is the delay for?
        printf("Underflow Error, TX FIFO flushed, radio state now: ");
        printf("%x \r\n",Radio_Read_Status(TI_CCxxx0_MARCSTATE,radio_select)); 
      }

      // Toggle LED to indicate packet sent
      P7OUT ^= BIT7;
    }
/*
    if(e&COMM_sys_events_IMG_DAT)
    {
      puts("Received IMG Data");
      //Need to store IMG data in SD Card so it is ready for transmission.
      //Question as to whether or not to store data as already processed AX.25
      //or raw data ready to be processed.
      //Data will transmit on Radio 2 on command.
    }

    if(e&COMM_sys_events_LEDL_DAT)
    {
      puts("Received LEDL Data");
      //Need to store LEDL data in SD Card so it is ready for transmission.
      //Question as to whether or not to store data as already processed AX.25
      //or raw data ready to be processed.
      //Data will transmit on Radio 2 on command.
    }

*/

   } //end for loop
}


void COMM_Radio_SPI_PinSetup(void){
//PORT MAP THE UCA3 TO PORT 3.5,6,7 
//Code was copid from the SD lib code. SD lib code is port mapped for any communication periferial. 
//I changed the MCC to Radio and applied the Radio SPI lines to the #defines in the Radio_functions.h file
//We could try to set this up like SD card so that radios can be accessed on any of the SPI perifials and will still work. 
  //unlock registers
  PMAPKEYID=PMAPKEY;
  //allow reconfiguration
  PMAPCTL|=PMAPRECFG;
  //setup SIMO
  RADIO_PMAP_SIMO=RADIO_PM_SIMO;
  //setup SOMI
  RADIO_PMAP_SOMI=RADIO_PM_SOMI;
  //setup SIMO
  RADIO_PMAP_UCLK=RADIO_PM_UCLK;
  //lock the Port map module
  PMAPKEYID=0;

}


void COMM_Setup(void){
//SPI setup for MSP430f6779A is done on Port 3
//Set up peripherals for COMM MSP
//Radio SPI on P3: P3.7=UCB1SIMO, P3.6=USB1SOMI, P3.5=UCB1CLK
//NOTE Redefined all SPI Setup on the UCA3 SPI port, COMM for ARC2 uses UCB1

  UCB1CTL1 = UCSWRST;                             // Put UCB1 into reset
  UCB1CTL0 = UCCKPH|UCMSB|UCMST|UCMODE_0|UCSYNC;  // Data Read/Write on Rising Edge
                                                  // MSB first, Master mode, 3-pin SPI
                                                  // Synchronous mode
  UCB1CTL1 |= UCSSEL_2;                           // SMCLK
  UCB1BR0 = 16;                                   // Set frequency divider so SPI runs at 16/16 = 1 MHz
  UCB1BR1 = 0;

  //Radio CS P5.1=CC2500_CS_1 (ENABLE1), P5.2=CC2500_CS_2 (ENABLE2), 
  //Initial state for CS is High, CS pulled low to initiate SPI
  P5OUT |= CS_2500_1;                     // Ensure CS for CC2500_1 is disabled
  P5OUT |= CS_2500_2;                     // Ensure CS for CC2500_2 is disabled

  
  P5DIR |= CS_2500_1;                     //Set output for CC2500_1 CS
  P5DIR |= CS_2500_2;                     //Set output for CC2500_2 CS
 
  P3DIR |= RADIO_PIN_SIMO|RADIO_PIN_SCK;

  //Set pins for SPI usage
  P3SEL0 |= RADIO_PINS_SPI;

  //Bring UCB1 out of reset state
  UCB1CTL1 &= ~UCSWRST;

  //UC1IE = UCB1TXIE|UCB1RXIE;                      //Enable transmit and receive interrupt


//SD Card SPI on P3/P5: P3.6=UCA1SIMO, P3.7=UCA2SOMI, P5.0=UCA1CLK
//THIS IS HANDELED IN JESSE'S LIBRARY

//Tx-Rx LED's P7.0-3 (inputs) CURRENTLY USED IN MAIN TO SHOW ADDR ON DEV BOARD.
}

void Radio_Interrupt_Setup(void){ // Enable RX interrupts only!  TX interrupt enabled in TX Start
  // Use GDO0 and GDO2 as interrupts to control TX/RX of radio
  P1DIR = 0;			        // Port 1 configured as inputs (i.e. GDO0 and GDO2 are inputs)
  P1IES = 0;
  P1IES |= CC2500_1_GDO2|CC2500_2_GDO2; // GDO0 interrupts on rising edge = 0, GDO2 interrupts on falling edge = 1
  P1IFG = 0;                            // Clear all flags
  P1IE |= CC2500_1_GDO0|CC2500_2_GDO0; // Enable GDO0 interrupt only (RX interrupt)
}



void PrintBuffer(char *dat, unsigned int len){
   int i;

   for(i=0;i<len;i++){
      printf("0X%02X ",dat[i]); //print LSB first
      if((i)%16==15){
        printf("\r\n");
      }
    }
    printf("\r\n");
    printf("\r\n");
}

void PrintBufferBitInv(char *dat, unsigned int len){
   int i;

   for(i=0;i<len;i++){
      printf("0X%02X ",__bit_reverse_char(dat[i])); //print MSB first so it is understandable
      if((i)%16==15){
        printf("\r\n");
      }
    }
    printf("\r\n");
    printf("\r\n");
}

