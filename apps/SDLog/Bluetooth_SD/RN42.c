#include "msp430.h"
#include "RN42.h"
#include "../5XX_hal/hal_DMA.h"
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include "../msp430_clock/msp430_clock.h"

// copied as directly as possible from TinyOS RovingNetworksP.nc
// http://code.google.com/p/tinyos-main/source/browse/trunk/tos/platforms/shimmer/chips/bluetooth/RovingNetworksP.nc

uint16_t secondTry, starting;
void (*runSetCommands_cb)(void);

uint8_t messageInProgress, transmissionOverflow, messageLength, txie_reg;
uint8_t messageBuffer[128];
uint8_t receiveBuffer[8];
char expectedCommandResponse[8], newName[17], newAutoMaster[13], newPIN[17], newSvcClass[5], newDevClass[5], newSvcName[17], newRawBaudrate[5],
      newBaudrate[5], newInquiryTime[5], newPagingTime[5];
char commandbuf[32];

uint8_t bt_setcommands_step,command_received, bt_setcommands_start;
uint8_t bt_runmastercommands_step, bt_runmastercommands_start;
uint8_t bt_getmac_step, bt_getmac_start;

uint8_t radioMode, charsSent, charsReceived;

uint8_t discoverable, authenticate, encrypt, setNameRequest, setPINRequest, resetDefaultsRequest, setSvcClassRequest,
   setDevClassRequest, setSvcNameRequest, setRawBaudrate, setBaudrate, disableRemoteConfig, newMode, getMacAddress,
   setCustomInquiryTime, setCustomPagingTime;

//master mode stuff
uint8_t btConnected, deviceConn;
char targetBt[16];

uint8_t (*dataAvailableFuncPtr)(uint8_t data) = 0;


void BT_startDone_cb(void (*cb)(void)){   runSetCommands_cb = cb;}

void initRN1() {
   // powerup state is reset == low (true); mike conrad of roving networks sez:
   // wait about 1s to 2s after reset toggle
   P4OUT |= BIT4;
}
void initRN2() {
   //UART_RTS interrupt: RTS raises when BT has trans overflow
   P1IES &= ~BIT3;      //can assume initially low as module is in reset so watch for low to high transition
   P1IFG &= ~BIT3;      //clear flag
   P1IE |= BIT3;        //enable interrupt

   //PIO2_CONNECT interrupt
   P1IES &= ~BIT0;      //can assume initially low as module is in reset so watch for low to high transition
   P1IFG &= ~BIT0;      //clear flag
   P1IE |= BIT0;        //enable interrupt

   P2OUT &= ~BIT2;      // toggling cts wakes it up
   P2OUT |= BIT2;
}
void initRN3() {
   P2OUT &= ~BIT2;      // active low
}

void setupUART() {
   P5SEL |= BIT6+BIT7;

   UCA1CTL1 |= UCSWRST;                      // **Put state machine in reset**
   UCA1CTL1 |= UCSSEL_2;                     // SMCLK
   UCA1BR0 = 13;                             // 24MHz 115200
   UCA1BR1 = 0;                              // 24MHz 115200
   UCA1MCTL = UCBRS_0 + UCBRF_0 + UCOS16;    // Modln UCBRSx=0, UCBRFx=0, over sampling
   UCA1CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
   UCA1IE |= UCTXIE;                         // Enable USCI_A1 TX and RX interrupt
}

void disableUART() {
   UCA1CTL1 |= UCSWRST;                      //Put state machine in reset
   UCA1IE &= ~UCTXIE;                        //Disable USCI_A1 TX interrupt only
}

void disableRN() {

   P4OUT &= ~BIT4;      //hold in reset
   disableUART();
   P1IE &= ~BIT3;       //disable RTS interrupt
   P1IE &= ~BIT0;       //disable Connection interrupt
}


//write data to be transmitted to the Bluetooth module
//returns 0 if fails, else 1
//will only fail if a previous BT_write is still in progress
uint8_t writeCommand(char *cmd, char *response) {
   if(messageInProgress)
      return 0;         //fail

   strcpy(expectedCommandResponse, response);
   DMA2SZ = strlen(expectedCommandResponse);
   DMA2_enable();
   charsReceived = 0;
   if(BT_write((uint8_t *)cmd, strlen(cmd)))
      return 1;         //success
   else
      return 0;         //fail
}


// Connect and Disconnect commands are exceptional commands in that
// they automatically return to data mode once they are issued
uint8_t writeCommandNoRsp(char * cmd) {
   charsReceived = 0;
   if(BT_write((uint8_t *)cmd, strlen(cmd)))
      return 1;         //success
   else
      return 0;         //fail
}


// this one is awkward. we need to send one command at a time and wait until the
// response is received until sending the next command
// There are a couple of possible approaches
// - use a stepped switch statement and call this function from the receive ISR
//   once the response is received. But results in overly long ISR, from sending
//   the next command from within the ISR and then "hoping" that the ISR exits
//   before the response comes back (not a problem, but bad design)
//   (see TestProject5 for this approach)
// - block on a variable between each step and reset variable in ISR
//   poor design, but this function will only need to be run rarely
//   (see TestProject6 for this approach)
// - put MSP430 in low power mode while waiting for response from RN42
//   best option powerwise, but need to ensure no other interrupt will start
//   processor executing while waiting for response. Also need to take out of low
//   power mode to send commands and return to LPM afterwards

void runSetCommands() {
   if(!bt_setcommands_start){
      BT_rst_MessageProgress();
      bt_setcommands_start=1;
      command_received = 1;
   }
   if(command_received){
      command_received = 0;
      if(bt_setcommands_step == 0){
         bt_setcommands_step++;
         writeCommand("$$$", "CMD\r\n");
         return;
      }

      // reset factory defaults
      if(bt_setcommands_step == 1){
         bt_setcommands_step++;
         if(resetDefaultsRequest) {
            writeCommand("SF,1\r", "AOK\r\n");
            return;
         }
      }

      // default is slave (== 0), otherwise set mode
      if(bt_setcommands_step == 2){
         bt_setcommands_step++;
         if(newMode) {
            sprintf(commandbuf, "SM,%d\r", radioMode);
            writeCommand(commandbuf, "AOK\r\n");
            return;
         }
      }


      if(bt_setcommands_step == 3){
         bt_setcommands_step++;
         if(getMacAddress) {
            sprintf(commandbuf, "GB\r");
            DMA2SZ = 14;
            DMA2_enable();
            writeCommandNoRsp(commandbuf);
            return; //wait until response is received
         }
      }

      if(bt_setcommands_step == 4){
         bt_setcommands_step++;
         if(radioMode == AUTO_MASTER_MODE ) {
            sprintf(commandbuf, "SR,%s\r", newAutoMaster);
            writeCommand(commandbuf, "AOK\r\n");
            return;
         }
      }

      if(bt_setcommands_step == 5){
         bt_setcommands_step++;
         //device is discoverable with a non-zero inquiry scan window
         //default "time" is 0x0200 (units unspecified)
         if(!discoverable) {
            writeCommand("SI,0000\r", "AOK\r\n");
            return;
         }
      }

      if(bt_setcommands_step == 6){
         bt_setcommands_step++;
         // device default is off
         if(authenticate) {
            writeCommand("SA,1\r", "AOK\r\n");
            return;
         }
      }

      if(bt_setcommands_step == 7){
         bt_setcommands_step++;
         // device default is off
         if(encrypt) {
            writeCommand("SE,1\r", "AOK\r\n");
            return;
         }
      }

      if(bt_setcommands_step == 8){
         bt_setcommands_step++;
         // default is none
         if(setNameRequest) {
            sprintf(commandbuf, "SN,%s\r", newName);
            writeCommand(commandbuf, "AOK\r\n");
            return;
         }
      }

      if(bt_setcommands_step == 9){
         bt_setcommands_step++;
         // default is none
         if(setPINRequest) {
            sprintf(commandbuf, "SP,%s\r", newPIN);
            writeCommand(commandbuf, "AOK\r\n");
            return;
         }
      }

      if(bt_setcommands_step == 10){
         bt_setcommands_step++;
         if(setSvcClassRequest) {
            sprintf(commandbuf, "SC,%s\r", newSvcClass);
            writeCommand(commandbuf, "AOK\r\n");
            return;
         }
      }

      if(bt_setcommands_step == 11){
         bt_setcommands_step++;
         if(setDevClassRequest) {
            sprintf(commandbuf, "SD,%s\r", newDevClass);
            writeCommand(commandbuf, "AOK\r\n");
            return;
         }
      }

      if(bt_setcommands_step == 12){
         bt_setcommands_step++;
         if(setSvcNameRequest) {
            sprintf(commandbuf, "SS,%s\r", newSvcName);
            writeCommand(commandbuf, "AOK\r\n");
            return;
         }
      }

      if(bt_setcommands_step == 13){
         bt_setcommands_step++;
         if(setRawBaudrate) {
            // set the baudrate to suit the MSP430 running at 8Mhz
            sprintf(commandbuf, "SZ,%s\r", newRawBaudrate);
            writeCommand(commandbuf, "AOK\r\n");
            return;
         }
      }

      if(bt_setcommands_step == 14){
         bt_setcommands_step++;
         if(disableRemoteConfig) {
            // disable remote configuration to enhance throughput
            writeCommand("ST,0\r", "AOK\r\n");
         } else {
            // disable remote configuration to enhance throughput
            writeCommand("ST,255\r", "AOK\r\n");
         }
         return;
      }

      if(bt_setcommands_step == 15){
         bt_setcommands_step++;
         if(setCustomInquiryTime) {
            sprintf(commandbuf, "SI,%s\r", newInquiryTime);
            writeCommand(commandbuf, "AOK\r\n");
         } else {
            // to save power only leave inquiry on for approx 40msec (every 1.28 secs)
            writeCommand("SI,0040\r", "AOK\r\n");
         }
         return;
      }

      if(bt_setcommands_step == 16){
         bt_setcommands_step++;
         if(setCustomPagingTime) {
            sprintf(commandbuf, "SJ,%s\r", newPagingTime);
            writeCommand(commandbuf, "AOK\r\n");
         } else {
            // to save power only leave paging on for approx 80msec (every 1.28 secs)
            writeCommand("SJ,0080\r", "AOK\r\n");
         }
         return;
      }

      if(bt_setcommands_step == 17){
         bt_setcommands_step++;
         if(setBaudrate) {
            // set the baudrate to suit the MSP430
            sprintf(commandbuf, "SU,%s\r", newBaudrate);
            writeCommand(commandbuf, "AOK\r\n");
            return;
         }
      }

      if(bt_setcommands_step == 18){
         bt_setcommands_step++;
         // exit command mode
         writeCommand("---\r", "END\r\n");
         return;
      }

      //arriving here =  all done perfectly
      if(bt_setcommands_step == 19){
         bt_setcommands_step=0;
         bt_setcommands_start = 0;
         runSetCommands_cb();
      }
   }
}

uint8_t runMasterCommands() {
   if(!bt_runmastercommands_start){
      BT_rst_MessageProgress();
      command_received = 1;
      bt_runmastercommands_start = 1;
      DMA2_disable();
   }
   if(command_received){
      command_received = 0;
      if(bt_runmastercommands_step == 0){
         bt_runmastercommands_step++;
         if(!secondTry){
            writeCommand("$$$", "CMD\r\n");
            return 0; //wait until response is received
         }
      }
      // Connect
      if(bt_runmastercommands_step == 1){
         bt_runmastercommands_step++;
         if(deviceConn && (!btConnected)){            //Connect
            secondTry = 1;
            sprintf(commandbuf, "C,%s\r", targetBt);
            writeCommand(commandbuf, "TRYING\r\n");
         } else if((!deviceConn) && (btConnected)) {  //Disconnect
            sprintf(commandbuf, "K,\r");
            writeCommand(commandbuf, "KILL\r\n");
         } else {                                     //exit command mode
            // not needed for connect and disconnect commands */
            writeCommand("---\r", "END\r\n");
            deviceConn = 0;
         }
         return 0;
      }

      if(bt_runmastercommands_step == 2){
         bt_runmastercommands_step = 0;
         bt_runmastercommands_start =0;
      }
   }
   return 0;
}


void BT_setGoodCommand(){
   command_received=1;
   *expectedCommandResponse = '\0';
   if(bt_setcommands_start)
      runSetCommands();
   if(bt_runmastercommands_start)
      runMasterCommands();
}
void sendNextChar() {
   uint16_t i=0;
   if(charsSent < messageLength) {
      while ((UCA1IFG & UCTXIFG)&& ++i) ; //ensure no tx interrupt is pending
      UCA1TXBUF = *(messageBuffer + charsSent++);
   } else {
      messageInProgress = 0;              //false
   }
}


void BT_init() {
   messageInProgress = transmissionOverflow = 0;

   //Turn on power (SW_BT P4.3 on SR30 and newer)
   P4OUT |= BIT3;

   txie_reg = 0;
   secondTry = 0;
   command_received = 0;
   bt_getmac_step = 0;
   bt_getmac_start = 0;
   bt_setcommands_start = 0;
   bt_setcommands_step = 0;
   bt_runmastercommands_step = 0;
   bt_runmastercommands_start = 0;
   getMacAddress = 0;
   newMode = 0;
   radioMode = SLAVE_MODE;
   discoverable = 1;
   authenticate = 0;
   encrypt = 0;
   resetDefaultsRequest = 0;
   setNameRequest = 0;
   setPINRequest = 0;
   setSvcClassRequest = 0;
   setSvcNameRequest = 0;
   setDevClassRequest = 0;
   setRawBaudrate = 0;
   disableRemoteConfig = 0;
   setCustomInquiryTime = 0;
   setCustomPagingTime = 0;
   setBaudrate = 0;
   memset(receiveBuffer,0,8);

   // connect/disconnect commands
   deviceConn = btConnected = 0;

   *expectedCommandResponse = '\0';   // NULL pointer
   charsReceived = 0;
}

void start3(){
   if(starting){
      initRN3();
      setupUART();
      msp430_register_timer_cb(runSetCommands, 15, 0);
      starting = 0;
   }
}
void start2(){
   if(starting){
      initRN2();
      msp430_register_timer_cb(start3, 5, 0);
   }
}
void BT_start(){
   starting = 1;
   initRN1();
   msp430_register_timer_cb(start2, 2000, 0);
}

void BT_disable() {
   disableRN();
   //Turn off power (SW_BT P4.3 on SR30 and newer)
   P4OUT &= ~BIT3;
   starting = 0;
}


//write data to be transmitted to the Bluetooth module
//returns 0 if fails, else 1
//will only fail if a previous BT_write is still in progress
uint8_t BT_write(uint8_t *buf, uint8_t len) {
   if(messageInProgress){
      return 0; //fail
   }

   messageInProgress = 1;
   charsSent = 0;
   memcpy(messageBuffer, buf, len);
   messageLength = len;

   if(!transmissionOverflow) {
      sendNextChar();
   }
   else{
      messageInProgress = 0;
   }

   return 1; //success
}

uint8_t BT_connect(uint8_t *addr) {
   deviceConn = 1;
   strcpy(targetBt, (const char *)addr);
   return runMasterCommands();
}


uint8_t BT_disconnect() {
   //Delay: If any bytes are seen before or after $$$ in a 1
   //second window, command mode will not be entered and these
   //bytes will be passed on to other side

   deviceConn = 0;

   return runMasterCommands();
}


void BT_setGetMacAddress(uint8_t val) {
   getMacAddress = val;
}

void BT_setRadioMode(uint8_t mode) {
   newMode = 1;
   radioMode = mode;
}

void BT_setAutoMaster(char * master) {
   snprintf(newAutoMaster, 13, "%s", master);
}

void BT_setDiscoverable(uint8_t disc) {
   discoverable = disc;
}


void BT_setEncryption(uint8_t enc) {
   encrypt = enc;
}


void BT_setAuthentication(uint8_t auth) {
   authenticate = auth;
}


void BT_setName(char * name) {
   setNameRequest = 1;
   snprintf(newName, 17, "%s", name);
}


void BT_setPIN(char * PIN) {
   setPINRequest = 1;
   snprintf(newPIN, 17, "%s", PIN);
}


void BT_setServiceClass(char * class) {
   setSvcClassRequest = 1;
   snprintf(newSvcClass, 5, "%s", class);
}


void BT_setServiceName(char * name) {
   setSvcNameRequest = 1;
   snprintf(newSvcName, 5, "%s", name);
}


void BT_setDeviceClass(char * class){
   setDevClassRequest = 1;
   snprintf(newDevClass, 5, "%s", class);
}


void BT_disableRemoteConfig(uint8_t disableConfig) {
   disableRemoteConfig = disableConfig;
}


//this one makes sense only to roving networks
//the supplied "rate_factor" is the baudrate * 0.004096
//this factor must be an integer value...
void BT_setRawBaudrate(char * rate_factor) {
   setRawBaudrate = 1;
   snprintf(newRawBaudrate, 5, "%s", rate_factor);
}


//to set the baudrate of the BT to MSP serial interface
//as per RovingNetworks command spec EG "SU,96" or "SU,230"
//SU,<rate> - Baudrate, {1200, 2400, 4800, 9600, 19.2,
//38.4, 57.6, 115K, 230K, 460K, 921K }
void BT_setBaudrate(char * new_baud) {
   setBaudrate = 1;
   snprintf(newBaudrate, 5, "%s", new_baud);
}


//Sets the Paging Scan Window - amount of time device
//spends enabling page scan (connectability).
//Minimum = (hex word) "0012", corresponding to about 1% duty cycle.
//Maximum = (hex word) "1000"
void BT_setPagingTime(char * hexval_time) {
   setCustomPagingTime = 1;
   snprintf(newPagingTime, 5, "%s", hexval_time);
}


//Sets the Inquiry Scan Window - amount of time device
//spends enabling inquiry scan (discoverability).
//Minimum = (hex word) "0012", corresponding to about 1% duty cycle.
//Maximum = (hex word) "1000"
void BT_setInquiryTime(char * hexval_time) {
   setCustomInquiryTime = 1;
   snprintf(newInquiryTime, 5, "%s", hexval_time);
}

void BT_resetDefaults() {
   resetDefaultsRequest = 1;
}


void BT_receiveFunction(uint8_t (*receiveFuncPtr)(uint8_t data)) {
   dataAvailableFuncPtr = receiveFuncPtr;
}


void BT_connectionInterrupt(uint8_t value) {
   btConnected = value;
}


void BT_rtsInterrupt(uint8_t value) {
   transmissionOverflow = value;
   if(transmissionOverflow){//in disabling sending
      txie_reg = UCA1IE & UCTXIE;
      UCA1IE &= ~UCTXIE;
   }else{// in resuming sending
      if(txie_reg)
         UCA1IE |= UCTXIE;
   }
}

void BT_rst_MessageProgress() {
   messageInProgress = 0;
   *expectedCommandResponse = '\0';

   command_received = 0;
   bt_setcommands_step = 0;
   bt_setcommands_start = 0;
   bt_runmastercommands_step = 0;
   bt_runmastercommands_start = 0;
}


uint8_t * BT_getExpResp(void){
   return (uint8_t*)expectedCommandResponse;
}

uint8_t BT_getGetMacAddress(){
   return getMacAddress;
}

#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void) {
   switch(__even_in_range(UCA1IV,4)) {
   case 0:break;                                // Vector 0 - no interrupt
   case 2:                              		   // Vector 2 - RXIFG
      break;
   case 4:                                      // Vector 4 - TXIFG
      if(!transmissionOverflow) {
         sendNextChar();
      }
      break;
   default: break;
   }
}
