#ifndef RN42_H
#define RN42_H

#include <stdint.h>

enum {
	SLAVE_MODE,
	MASTER_MODE,
	TRIGGER_MASTER_MODE,
	AUTO_MASTER_MODE
};

// initialize the vars and power on
extern void BT_init();

// set up the BT/UART pins
extern void BT_start();

// disable the BT/UART pins. if the start is in progress, stop it
extern void BT_disable();

// pass the expected response to the main function for user customized use (e.g.DMA).
extern uint8_t * BT_getExpResp(void);

// when received command is the same as the expected response, run this
extern void BT_setGoodCommand();

// set a callback function cb that runs when Bt is successfully started
extern void BT_startDone_cb(void (*cb)(void));

//write data to be transmitted to the Bluetooth module
//returns 0 if fails, else 1
//will only fail if a previous BT_write is still in progress
extern uint8_t BT_write(uint8_t *buf, uint8_t len);

//connect to a specific device that was previously discovered
extern uint8_t BT_connect(uint8_t *addr);
//after this command is called there will be no link to the connected device
extern uint8_t BT_disconnect();


//enum SLAVE_MODE, MASTER_MODE, TRIGGER_MASTER_MODE, AUTO_MASTER_MODE
extern void BT_setGetMacAddress(uint8_t val);
extern uint8_t BT_getGetMacAddress();
extern void BT_setRadioMode(uint8_t mode);
extern void BT_setAutoMaster(char * master);

extern void BT_setDiscoverable(uint8_t disc);
extern void BT_setEncryption(uint8_t enc);
extern void BT_setAuthentication(uint8_t auth);
extern void BT_setName(char *name); // max 16 chars
extern void BT_setPIN(char *name); // max 16 chars
extern void BT_setServiceClass(char *serviceClass); // max 4 chars (hex word)
extern void BT_setServiceName(char *name); // max 16 chars
extern void BT_setDeviceClass(char *deviceClass); // max 4 chars (hex word)
extern void BT_disableRemoteConfig(uint8_t disableConfig);
extern void BT_rst_MessageProgress();// reset messageInProgress to 0 as in the initialisation @weibo

//rate_factor is baudrate * 0.004096, e.g. to set 115200, pass in "472"
extern void BT_setRawBaudrate(char *rateFactor); // max 4 chars, must be integer

//provide one of the following as a string argument:
//1200, 2400, 4800, 9600, 19.2, 38.4, 57.6, 115K, 230K, 460K, 921K
extern void BT_setBaudrate(char *newBaud);

//save power by minimising time Inquiry/Page scanning
//module reset necessary for changes to take effect
extern void BT_setPagingTime(char *hexvalTime); // max 4 chars (hex word)
extern void BT_setInquiryTime(char *hexvalTime); // max 4 chars (hex word)

extern void BT_resetDefaults();


//pass in a pointer to the function that will get called when
//data arrives
//the pass in function returns a 1 if program execution should resume
//(i.e. clear LPM3 bits)
//otherwise return 0
extern void BT_receiveFunction(uint8_t (*receiveFuncPtr)(uint8_t data));


//this function needs to be called from within the BT_PIO ISR
//in order to inform the RN42 driver about the state change
//value needs to be 1 if interrupt was low to high else 0
extern void BT_connectionInterrupt(uint8_t value);

//this function needs to be called from within the BT_RTS ISR
//in order to inform the RN42 driver about the state change
//value needs to be 1 if interrupt was low to high else 0
extern void BT_rtsInterrupt(uint8_t value);
#endif //RN42_H
