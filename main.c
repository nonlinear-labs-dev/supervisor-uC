/******************************************************************************/
/** @version	V4.3 2018-08-09 KSTR
    @date		2018-05-22
    @author		Daniel Tzschentke / KSTR
	@brief    	systick 1ms
    Changes :
	- Added Support for Bistable Relay w/ mute and un-mute coils.
	- Blinking Power Led during Start-Up (slow rate) and Shut-Down (fast).
	- After booting a 20 second hold time is active where power button is NOT scanned,
	  this is needed because the ePC seems to need 10+ seconds until it accepts a power-down
	- Optional: Pressing Power Button for 4 Seconds during Shut-Down forces Power-Down
	  (also, the Power LED blinks at a ultra-fast rate).
	- Option for 12MHz Xtal clock or 2MHz internal RC clock
	- Option for keeping uC alive in standby to allow for power LED
	  blinking in 2sec intervals to indicate standby (applied power).
	- Option for Power Fail sensing (V7 only): instant muting and shutdown (full reset) is
	  forced when supply input falls below ~15V. This even works when DC plug is
	  pulled provided a 1000uF/35V e-cap is installed at the power input. 
	  At Power-Up (into standby) or Reset system waits until 19V input is above
	  ~16V.
	- Added PCB revision detection
	- Added RESET for LPC after Power Up (requires rework on PCB)
	- Added Signaling of Hardware and Firmware Versions during PON :
		* if HW detect finds V7 hardware, LED blinks twice, otherwise once
		* after a delay, the firmware internal ID is displayed via according
		  (number of blinks) - 1
	- Added a mandatory 600ms button hold time for PON/POFF to debounce the switch
	  and avoid accidental triggering. Any action is triggered after the *release*
	  of the button to prevent cycling.
	- Added toggling of the shutdown request signal to BBB so BBB will eventually
	  see it no matter if it already accepts it at the time the user presses button.
	  Signal seems to be edge-sensitive, so keeping the pin high isn't enough.
	  
	
*******************************************************************************/

/*****
Firmware versions and Internal ID Numbers
The ID Numbers will be displayed as LED blinks during Power-Up, after HW- Version
	4.0			= 1
	4.1			= 2
	4.2			= 3
	4.3			= 4
	etc
****/

#define FW_Version_ID	4		// Version 4.3


// COMPILE-TIME OPTIONS :

// ------------ define this to use 2MHz Clock Freq :
#define __CLOCK_2MHZ__
/*
	Also, you need to set fuse SUT_CKSEL to "INTRCOSC_2MHZ_6CK_64MS" to
	program internal RC oscillator for 2MHz. Clock should be manually calibrated for
	3.3V supply. 
	Set SUT_CKSEL it to "EXTHIFXTALRES_258CK_4MS" with external 12MHz quartz when
	this option is off.
*/


// ------------ define this to keep uC alive during standby mode :
#define __NO_SLEEP__
/* 
	This allows a blinking LED in standby mode.
	When used with 2MHz Clock Option standby current consumption of
	uniboard_1b(V6) raises from 1.3mA in true sleep to only 2.6mA (0.05W @ 19V).
	With 12MHz Xtal clock current is 1.5mA (sleep) and 6.5mA (standby),
	which still is only 0.12W @ 19V.
*/


// --------- define this to keep outputs muted in standby with V6 hardware :
#define __MUTE_IN_STBY_V6__
/* 
	This keeps MUTE relays powered on during standby for the monostable relays
	in V6 hardware.	This increases standby current to 50mA (1W @ 19V).
	Has no effect on bistable relays in V7 as they are supplied from
	12VA which is off in standby.
*/


// --------- define this to check and catch severe ePC signal error
//#define __CHECK_EPC__
/* 
	When defined, this checks whether sensed ePC Power Led did change state
	after actuating ePC power button. If it didn't change, something is
	wrong and shutdown is initiated. Check is performed after the initial
	start-up wait time (20).
	Also, after start-up the signal is periodically checked and
	shutdown initiated when not present.
*/


// --------- define this to check and catch severe BBB signal error
//#define __CHECK_BBB__
/* 
	When defined, this checks whether BBB signals its 'alive' signal.
	If not, shutdown is initiated.
	Check is performed after the initial start-up wait time (20).
	Also, after start-up the signal is periodically checked and
	shutdown initiated when not present.
	Might still be unstable at the moment as the actual time until
	BBB signals "live" is not clearly determined
	==> DO NOT USE!
*/


// --------- define this to enable power fail procedures
#define __CHECK_POWERFAIL__
/* 
	When defined, the 19V input power line is constantly polled and once
	it goes below ~15V, mute relays are activated and after a short period
	a power down (full reset) is forced. At power-up, the 19V line is polled
	until it rises above ~16V, then booting continues.
	It's a good idea to use the 2.7V Brown-Out detector fuse as well.
	(BODLEVEL and BODEN)
	!! Option has NO effect unless a V7 hardware is detected !!
*/


// --------- define this to enable emergency off with pwr-btn
//#define __NOTHALT__
/* 
	When defined, a forced off is possible during shutdown
	when power button is pressed more than 4 seconds
*/


// --------- define this to enable restart after power-fail
//#define __AUTO_REBOOT__
/* 
	!!! DO NOT USE -- STILL BUGGY !!!
	When defined, system does an automatic reboot when a
	power fail occurred during normal operation.
	Does not reboot when power fail during start-up or
	shutdown.
*/







#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#include "bit_manipulation.h"
#include "pin_manipulation.h"
#include "coos.h"
#include "pin_mapping.h"


#define STATE_SLEEPING			0
#define STATE_BOOTING			1
#define STATE_MONITORING		2
#define STATE_SHUTTING_DOWN		3
#define STATE_PREPARE_SLEEPING	4

#define RELOAD_DEBOUNCE_TIMER			120	  // 120 * 5ms = 600ms
static uint8_t	btn_step = 0;
	
	
volatile uint8_t state = STATE_PREPARE_SLEEPING;
volatile uint8_t pwr_btn_debounce_timer = RELOAD_DEBOUNCE_TIMER;
volatile uint8_t flag_pwr_btn_pressed = 0;
volatile uint8_t flag_waiting_for_first_systick = 1;
volatile uint16_t timer_force_shutdown = 0;
volatile uint16_t ticker = 0; // 1ms interval free running up counter modulo 256
volatile uint8_t sm_reset = 0;
volatile uint8_t auto_reboot = 0;
volatile uint8_t relays = 1;
volatile uint8_t V7_hardware = 0;


void disable_irq_power_btn(void);
void enable_irq_power_btn(void);

uint8_t isPowerBtnPressed(void)
{
	return (PinGet(SYS_IPT_PwrBtn) == 0);
}

uint8_t ePC_off_state;
uint8_t ePC_on_state;
uint8_t Is_ePC_Off(void)
{
   #ifdef __CHECK_EPC__
	if (ePC_on_state != ePC_off_state)
		return (PinGet(EPC_IPT_nPowerLed) == ePC_off_state);
	else
		return 1;	// ePC didn't signal state change, so assume off
   #else
	return (PinGet(EPC_IPT_nPowerLed));
   #endif
}

uint8_t BBB_off_state;
uint8_t BBB_on_state;
uint8_t Is_BBB_Ready(void)
{
	return PinGet(BBB_IPT_SystemRunning);
}

uint8_t Is_BBB_Off(void)
{
   #ifdef __CHECK_BBB__	
	if (BBB_on_state != BBB_off_state)
		return (!Is_BBB_Ready());
	else
		return 1;	// BBB didn't signal state change, so assume off
   #else
	return Is_BBB_Ready() == 0;
   #endif

}



uint8_t Is_PowerGood(void)
{
	if (V7_hardware)
		return !BitGet(ACSR, ACO);	  // Analog Comparator Output
	else
		return 1;
}

// Helper functions
void PowerLED(uint8_t on)	
{
	if (on != 0)	PinSet(SYS_OPT_SleepingLed);
	else 			PinClr(SYS_OPT_SleepingLed);
}
	
void DebugHalt(uint8_t led)
{
	PowerLED(led);
	do {} while(1);
}

void Wait(uint16_t ms)
{
	ticker = 0;	
	while (ticker <= ms)
		;
}


void Hardware_ID(void)
{
	V7_hardware = 0;
	PinOpt(SYS_OPT_Pod0);
	PinClr(SYS_OPT_Pod0);
	PinIpt(SYS_OPT_Pod1);
	PinPue(SYS_OPT_Pod1);
	Wait(2);

	// In V7 Pod1 and Pod0 are connected via 10k,
	// so the internal weak pullup cannot pull Pod1 high
	V7_hardware = (PinGet(SYS_OPT_Pod1) == 0);

	PinIpt(SYS_OPT_Pod0);
	PinPue(SYS_OPT_Pod0);
}


void Reset_LPC(void)
{	// LPC-Reset is connected to Pod1 ("P0" on PCB Silkscreen) via diode
	PinOpt(SYS_OPT_Pod1);
	PinClr(SYS_OPT_Pod1);	  // Pull RESET down via diode
	Wait(2);
	PinSet(SYS_OPT_Pod1);
	PinIpt(SYS_OPT_Pod1);
	PinPue(SYS_OPT_Pod1);

}

void Display_HW_and_FW_Version(void)
{
	PowerLED(0);
	Wait(100);
	if (V7_hardware)
	{	// blink two times
		PowerLED(1); Wait(50); PowerLED(0); Wait(100);
		PowerLED(1); Wait(50); PowerLED(0); Wait(100);
	} else {
		PowerLED(1); Wait(50); PowerLED(0); Wait(100);
	}
	Wait(500);
	for (uint8_t i=0; i<FW_Version_ID; i++)
	{
		PowerLED(1); Wait(200); PowerLED(0); Wait(200);
	}
	Wait(700);
}

/******************************************************************************/
/** states
*******************************************************************************/
void state_sleeping(uint8_t reset)
{
	static uint8_t step = 0;
	if (reset)
	{
		step=0;
		return;
	}
	
	PowerLED(0);
	
   #ifdef __NO_SLEEP__
	PowerLED(step == 0);
	step++;
	if (step == 40)		// 40*50ms Blink rate = 2sec
		step = 0;
	if ((auto_reboot == 0x5A) || flag_pwr_btn_pressed)
	{	// auto_reboot overrides power button
		auto_reboot = 0;
		flag_pwr_btn_pressed = 0;
		PowerLED(1);
		step = 0;
		state = STATE_BOOTING;
	}

   #else

	BitClr(MCUCSR, ISC2);				// interrupt on falling edge
	set_sleep_mode(SLEEP_MODE_STANDBY);
	enable_irq_power_btn();
	enable_irq_power_btn();
	sei();
	sleep_enable();
	sleep_cpu();
	// ... now sleeping until Power Button activity ...
	sleep_disable();
	disable_irq_power_btn();
	PowerLED(1);
	state = STATE_BOOTING;
	step = 0;
   #endif
}



/************************************************************************
	turn on 19V
	wait till power good
	start booting
	wait untill everything is booted
************************************************************************/
void state_booting(uint8_t reset)
{
	static uint16_t boot_cnt = 0;
	if (reset)
	{
		boot_cnt = 0;
		return;
	}
	
	PowerLED(++boot_cnt & 0x04);

	
	switch(boot_cnt)
	{
		case 1:		// prepare Relay state FET-Switches so that it is
		{			// effective as soon as power ramps up, wait 200ms
			PinSet(SYS_OPT_Relays_MUTE);
			PinClr(SYS_OPT_Relays_UNMUTE);
			// set illegal ePC/BBB signaling states to detect errors
			ePC_off_state = 0;
			ePC_on_state = 0;
			BBB_off_state = 0;
			BBB_on_state = 0;
			break;
		}
		case 5:		// switch on system power, wait 250ms
		{
			
			BBB_off_state = Is_BBB_Ready();
			PinSet(SYS_OPT_19V_FET);
			break;
		}
		case 10:   // actuate ePC power switch, wait 500ms
		{
			ePC_off_state = PinGet(EPC_IPT_nPowerLed);
			PinSet(EPC_OPT_Pwr);
			break;
		}
		case 20:   // Reset LPC, release ePC power switch, wait 15s
		{
			Reset_LPC();
			ePC_on_state = PinGet(EPC_IPT_nPowerLed);
			PinClr(EPC_OPT_Pwr);
			break;
		}		
		case 300:	// de-energize Mute Coil, wait 50ms, because Relay release
					// is slower than spec due to protection diode!
		{			// ePC, Power and analog circuits now had ~15s to settle and bias up
			if (relays) PinClr(SYS_OPT_Relays_MUTE);
			break;
		}
		case 301:	// energize UnMute Coil, wait 50ms
		{			// (Relay spec is 1..2ms, w/o bounce-time)
			if (relays) PinSet(SYS_OPT_Relays_UNMUTE);
			break;
		}
		case 302:	// de-energize UnMute Coil, to save power
		{
			PinClr(SYS_OPT_Relays_UNMUTE);
			break;
		}
		case 400:	// Monitoring Power Button Starts after 20 Seconds!!
		{
			flag_pwr_btn_pressed = 0;
			state = STATE_MONITORING;

		   #ifdef __CHECK_EPC__
			if (ePC_on_state == ePC_off_state)
				state = STATE_SHUTTING_DOWN;	// ePC didn't seem to power up, so start shutdown
		   #endif
		   #ifdef __CHECK_BBB__
			BBB_on_state = Is_BBB_Ready();
			if (BBB_on_state == BBB_off_state)
				state = STATE_SHUTTING_DOWN;	// BBB didn't seem to power up, so start shutdown
		   #endif
			boot_cnt = 0;
			PowerLED(1);			
			break;	
		}
	}		
}


void state_monitoring(void)
{	
	if (flag_pwr_btn_pressed == 1)
	{
		state = STATE_SHUTTING_DOWN;
		flag_pwr_btn_pressed = 0;
	}
	else
	{
		state = STATE_MONITORING;
	}
   #ifdef __CHECK_EPC__
	if (Is_ePC_Off())
		state = STATE_SHUTTING_DOWN;
   #endif
   #ifdef __CHECK_BBB__
	if (Is_BBB_Off())
		state = STATE_SHUTTING_DOWN;
   #endif
}


// mute audio and signal shutdown request to BBB and ePC
void state_shutting_down(uint8_t reset)
{
	static uint16_t shutting_cnt = 1;
	if (reset)
	{
		shutting_cnt = 1;
		return;		
	}
	
	switch(shutting_cnt)
	{
		case 1:
		{
			PinSet(SYS_OPT_Relays_MUTE);
			shutting_cnt++;
			break;
		}
		case 3:
		{
		   #ifdef __CHECK_EPC__
			if (!Is_ePC_Off())	 // ePC running ?
				PinSet(EPC_OPT_Pwr);
			else
				ePC_on_state = ePC_off_state; // flag forced
		   #else
			PinSet(EPC_OPT_Pwr);
		   #endif

		   #ifdef __CHECK_BBB__
			if (!Is_BBB_Off())	 // BBB running ?
				PinSet(BBB_OPT_ShutDown);
			else
				BBB_on_state = BBB_off_state; // flag forced
		   #else
			PinSet(BBB_OPT_ShutDown);
		   #endif
			
			shutting_cnt++;
			break;
		}
		case 5:
		{
			PinClr(EPC_OPT_Pwr);
			PinClr(BBB_OPT_ShutDown);	
			shutting_cnt++;			
			break;
		}
		case 10:
		{	
			state = STATE_PREPARE_SLEEPING;
			shutting_cnt = 1;
			break;
		}
		default:
			shutting_cnt++;
	}
}


void state_prepare_sleeping(uint8_t reset)
{
	static uint16_t prep_cnt = 0;
	static uint16_t blink_cnt = 0;
	static uint8_t	forced = 0;
	if (reset)
	{
		prep_cnt = 0;
		blink_cnt = 0;
		forced = 0;
		return;		
	}
	
	
	// Wait until ePC and BBB both are shut down
	if (forced || (Is_BBB_Off() && Is_ePC_Off()) )
	{	// Ok, both are down
		prep_cnt++;
		blink_cnt = 0;
		
		switch(prep_cnt)
		{
			case 1:
			{
				PinClr(SYS_OPT_19V_FET);
				forced = 1;	// disable further Is_ePC_Off() sensing after power removal
				break;
			}
			case 5:
			{
			   #ifdef __MUTE_IN_STBY_V6__
				PinSet(SYS_OPT_Relays_MUTE);
			   #else
				if (relays) PinClr(SYS_OPT_Relays_MUTE);
			   #endif
				break;
			}
			case 7:
			{
				prep_cnt = 0;
				forced = 0;
				state = STATE_SLEEPING;
			}
		}
	}
	else // still waiting....
	{
		blink_cnt++;
		PowerLED(blink_cnt & 0x02);
		
// keep toggling the shutdown signal so BBB will eventually see it
		if (blink_cnt & 0x01)
			PinSet(BBB_OPT_ShutDown);
		else
			PinClr(BBB_OPT_ShutDown);

		
		if (!isPowerBtnPressed())	// power button released?
		{	// then keep cntr at low values
			blink_cnt &= 0x03;
		}
	   #ifdef __NOTHALT__
		else
		{	// else time-out after 4 Seconds and force shut-off
			PowerLED(blink_cnt & 0x01);		// ultra-fast blinking
			if (blink_cnt > 80)
				forced = 1;
		}
	   #endif
	}
}


void task_statemachine(void)
{
	if (sm_reset)
	{
		state_sleeping(1);
		state_booting(1);
		state_shutting_down(1);
		state_prepare_sleeping(1);
		sm_reset = 0;
		return;
	}
	switch (state)
	{
		case STATE_SLEEPING:
			state_sleeping(0);
			break;
		case STATE_BOOTING:
			state_booting(0);
			break;
		case STATE_MONITORING:
			state_monitoring();
			break;
		case STATE_SHUTTING_DOWN:
			state_shutting_down(0);
			break;
		case STATE_PREPARE_SLEEPING:
			state_prepare_sleeping(0);
	}
}



/******************************************************************************/
/** INITs
*******************************************************************************/
/*	TIMER
	t_timer = (prescale/f_cpu) * compare
<=> compare = ((t_timer * F_CPU) / prescale) - 1

	*prescale = 0, 1, 2, 8, 64, 256, 1024 */
void timer_init(void)
{
	// set Timer Mode to CTC (Clear Timer on Compare Match)
	BitSet( TCCR0, WGM01 );		
	BitClr( TCCR0, WGM00 );
		
   #ifdef __CLOCK_2MHZ__
	// 2MHz
	// prescaler ( 010 -> 8 ) and start the timer
	BitClr( TCCR0, CS00 );
	BitSet( TCCR0, CS01 );
	BitClr( TCCR0, CS02 );
	OCR0 = 249;				// Set the value that you want to count to minus one
   #else
	// 12MHz
	// prescaler ( 011 -> 64 ) and start the timer
	BitSet( TCCR0, CS00 );
	BitSet( TCCR0, CS01 );
	BitClr( TCCR0, CS02 );
	OCR0 = 187;				// Set the value that you want to count to minus one
   #endif
	BitSet( TIMSK, OCIE0 );		// Set the ISR COMPA vector
}


void task_check_pwr_btn(void)
{
   #ifdef __NO_SLEEP__

	switch (btn_step)
	{
		case 0:		// wait until pressed button
			if (isPowerBtnPressed())	// button pressed
			{
				pwr_btn_debounce_timer = RELOAD_DEBOUNCE_TIMER;
				btn_step = 1;
			}
			break;

		case 1:		// wait until hold time elapsed with pressed button
			if (isPowerBtnPressed())	// button still pressed
			{
				pwr_btn_debounce_timer--;
				if (pwr_btn_debounce_timer == 0)	// hold time elapsed
				{
					if ( (state == STATE_SLEEPING) || (state == STATE_MONITORING) )
						flag_pwr_btn_pressed = 1;		// signal a button event
					btn_step = 2;
				}
			} else btn_step = 0;	// button released too early, so reset chain
			break;

		case 2:		// wait until button released
			if (!isPowerBtnPressed())	// button released
			{
				btn_step = 0;
			}
			break;
		default:
			btn_step = 0;
			break;		
	}
	
   #else

	if( pwr_btn_debounce_timer > 0 )
	{
		pwr_btn_debounce_timer--;
	}
	else
	{
		if (isPowerBtnPressed()) // check if pwr btn is pressed
		{
			if( (state == STATE_SLEEPING) || (state == STATE_MONITORING) )
			{
				flag_pwr_btn_pressed = 1;
			}
			pwr_btn_debounce_timer = RELOAD_DEBOUNCE_TIMER; 
		}
	}
   #endif
}




void tasks_init(void)
{
	//coos_add_task(sys_heardbeat_led_process, 0, 1);
	coos_add_task(task_statemachine, 10, 50);
	coos_add_task(task_check_pwr_btn, 5, 5);
}

void ComparatorInit(void)
{
	
	PinIpt(SYS_ADC_19V);		// set as input to access Pullup control
	PinPud(SYS_ADC_19V);		// disable Pullup
	// Analog Comparator Setup :
	BitClr(ADCSRA, ADEN);		// ADC OFF
	BitSet(SFIOR, ACME);		// Comp -IN uses ADC Muxer
	BitSet(ADMUX, MUX0);		// Select ...
	BitSet(ADMUX, MUX1);		// ... channel 7 (PortA, 7) ...
	BitSet(ADMUX, MUX2);		// ... as Comp Input.
	BitSet(ACSR, ACBG);			// Select 2.65V Bandgap for Comp +IN
	BitClr(ACSR, ACIS0);		// Trigger IRQ ...
	BitClr(ACSR, ACIS1);		// ... on rising *and* falling edge
	BitSet(ACSR, ACIE);			// Enable Comparator Interrupts	
}


/******************************************************************************/
/** MAIN
*******************************************************************************/
int main(void)
{
	timer_init();
	ComparatorInit();
	coos_init();
	tasks_init();
	auto_reboot = 0;
Reset:
	cli();
	btn_step = 0;
	pwr_btn_debounce_timer = RELOAD_DEBOUNCE_TIMER;
	flag_pwr_btn_pressed = 0;
	flag_waiting_for_first_systick = 1;
	timer_force_shutdown = 0;

	state = STATE_SLEEPING;

	relays = 1;

	PinOpt(SYS_OPT_SleepingLed);
	PinClr(SYS_OPT_SleepingLed);

	PinOpt(SYS_OPT_19V_FET);
	PinClr(SYS_OPT_19V_FET);

	PinIpt(SYS_IPT_PwrBtn);
	PinPue(SYS_IPT_PwrBtn);
	
	PinOpt(SYS_OPT_Relays_UNMUTE);
	PinClr(SYS_OPT_Relays_UNMUTE);
	PinOpt(SYS_OPT_Relays_MUTE);
   #ifdef __MUTE_IN_STBY_V6__
	PinSet(SYS_OPT_Relays_MUTE);
   #else
    PinClr(SYS_OPT_Relays_MUTE);
   #endif
	
	
	PinOpt(EPC_OPT_Pwr);
	PinClr(EPC_OPT_Pwr);

	PinIpt(EPC_IPT_nPowerLed);
	
	PinOpt(BBB_OPT_ShutDown);
	PinClr(BBB_OPT_ShutDown);
	
	PinIpt(BBB_IPT_SystemRunning);
	PinPue(BBB_IPT_SystemRunning);
	
	sm_reset = 1;
	task_statemachine();		// reset state machine internal vars
	
	sei();

	while(flag_waiting_for_first_systick == 1)
		;

	Hardware_ID();
//DebugHalt(V7_hardware);

   #ifdef __CHECK_POWERFAIL__
	while(!Is_PowerGood())			// wait for Power Good
		PowerLED(ticker & 0x20);	// 32ms on/off times
   #endif

	while(isPowerBtnPressed())		// wait for released pwr button
		PowerLED(ticker & 0x20);	// 32ms on/off times

    Display_HW_and_FW_Version();
	
   #ifdef __AUTO_REBOOT__
	if (auto_reboot == 0x5A)
		Wait(3000);		// wait 3 seconds with 19V off
   #endif

	coos_clear_pending();
	while(1)
	{
		coos_dispatch();
	   #ifdef __CHECK_POWERFAIL__
		if (timer_force_shutdown==0 && !Is_PowerGood())
		{	// 19V supply dropped below 15V once
			PinClr(SYS_OPT_Relays_UNMUTE);
			PinSet(SYS_OPT_Relays_MUTE);
			relays = 0;
		   #ifdef __AUTO_REBOOT__
			if (state == STATE_MONITORING)
				auto_reboot = 0x5A;
		   #endif
			timer_force_shutdown = 5; // give relays 4..5ms to act before killing their supply
		}
		if (timer_force_shutdown == 1)	// delay time out?
			goto	Reset;	// turn of power & do warm start
	   #endif
	}

}



/******************************************************************************/
/** IRQ related
*******************************************************************************/
ISR(INT2_vect)										// irq only for wake-up
{
	BitClr(GICR, INT2);								// disable_power_btn_irq();
	pwr_btn_debounce_timer = RELOAD_DEBOUNCE_TIMER;
}


ISR (TIMER0_COMP_vect)								// timer 0 overflow interrupt
{
	flag_waiting_for_first_systick = 0;
	coos_update();
	if (timer_force_shutdown)
		timer_force_shutdown--;
	ticker++;
}

ISR (ANA_COMP_vect)			// analog comparator
{
	if (BitGet(ACSR, ACO))	// Comp Out set now means falling edge
		PinPud(SYS_ADC_19V);	// release Pullup to force a higher threshold
	else
		PinPue(SYS_ADC_19V);	// engage  Pullup to force lower threshold
}




void disable_irq_power_btn(void)
{
	BitClr(GICR, INT2);
}


void enable_irq_power_btn(void)
{
	BitSet(GICR, INT2);
}


// END OF FILE