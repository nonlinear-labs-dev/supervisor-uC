/******************************************************************************/
/** @version	V4.1 2018-04-13 KSTR
    @date		2018-04-13
    @author		Daniel Tzschentke /KSTR
	@brief    	mapping of io names and pins
	Changes	: corrected wrong mapping for POD0 and POD1 (yet board silkscreen is swapped)
*******************************************************************************/
#ifndef PIN_MAPPING_H_
#define PIN_MAPPING_H_

#include <avr/io.h>



// System
#define SYS_ADC_19V						PORTA, 7

// #define SYS_OPT_TaskOverflow			PORTx, y
#define SYS_OPT_SleepingLed				PORTD, 7

#define SYS_OPT_19V_FET					PORTB, 1
#define SYS_OPT_Relays_MUTE				PORTB, 0	// Mute Coil of Bi-Stable Relay
#define SYS_IPT_PwrBtn					PORTB, 2	// INT2
#define SYS_OPT_Relays_UNMUTE			PORTB, 3	// UnMute Coil of Bi-Stable Relay


//#define SYS_OPT_LpcOk					PORTB, 5	// ISP MOSI
//#define SYS_OPT_EpcOk					PORTB, 6	// ISP MISO


//#define SYS_OPT_HeartbeatLed			PORTx, y

//#define SYS_OPT_UART_RX					PORTD, 0
//#define SYS_OPT_UART_TX					PORTD, 1

#define SYS_OPT_Pod0			PORTC, 6			// used for V7 Hardware Detection
#define SYS_OPT_Pod1			PORTC, 7			// used for V7 Hardware Detection


// Embedded PC + Windows + Reaktor
#define EPC_OPT_Pwr						PORTA, 5	// Power Button
#define EPC_IPT_nPowerLed				PORTA, 6	// Power LED, inverted signalling!
//#define EPC_IPT_HdLed					PORTx, y



// LPC
//#define LPC_OPT_Rst						PORTx, y
//#define LPC_OPT_Isp						PORTx, y
//#define LPC_IPT_M4Crashflag				PORTx, y
//#define LPC_IPT_UsbHeartbeat				PORTx, y
//#define LPC_IPT_M4Heartbeat				PORTx, y



// Beagle Bone + Linux
#define BBB_OPT_ShutDown				PORTD, 3
#define BBB_IPT_SystemRunning			PORTD, 4

//#define BBB_OPT_Rst						PORTD, 2  
//#define BBB_IPT_LinuxHeartbeat			PORTD, 4
//#define BBB_IPT_PlaygroundHeartbeat		PORTD, 5
//#define BBB_IPT_DriverHeartbeat			PORTx, y



#endif /* PIN_MAPPING_H_ */