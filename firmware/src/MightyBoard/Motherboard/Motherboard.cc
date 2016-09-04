/*
 * Copyright 2010 by Adam Mayer	 <adam@makerbot.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */

#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/atomic.h>
#include "Motherboard.hh"
#include "Configuration.hh"
#include "Steppers.hh"
#include "Command.hh"
#include "Interface.hh"
#include "Commands.hh"
#include "Eeprom.hh"
#include "EepromMap.hh"
#include "SoftI2cManager.hh"
#include "Piezo.hh"
#ifdef HAS_RGB_LED
#include "RGB_LED.hh"
#endif
#include "Errors.hh"
#include <avr/eeprom.h>
#include <util/delay.h>
#include "Menu_locales.hh"
#include "TemperatureTable.hh"
#include "SDCard.hh"
#include "TWI.hh"

//Warnings to remind us that certain things should be switched off for release

#ifdef ERASE_EEPROM_ON_EVERY_BOOT
	#warning "Release: ERASE_EEPROM_ON_EVERY_BOOT enabled in Configuration.hh"
#endif

#ifdef DEBUG_VALUE
	#warning "Release: DEBUG_VALUE enabled in Configuration.hh"
#endif

#if HONOR_DEBUG_PACKETS
	#warning "Release: HONOR_DEBUG_PACKETS enabled in Configuration.hh"
#endif

#ifdef DEBUG_ONSCREEN
	#warning "Release: DEBUG_ONSCREEN enabled in Configuration.hh"
#endif

#ifndef JKN_ADVANCE
	#warning "Release: JKN_ADVANCE disabled in Configuration.hh"
#endif

#ifdef DEBUG_SLOW_MOTION
	#warning "Release: DEBUG_SLOW_MOTION enabled in Configuration.hh"
#endif

#ifdef DEBUG_NO_HEAT_NO_WAIT
	#warning "Release: DEBUG_NO_HEAT_NO_WAIT enabled in Configuration.hh"
#endif

#ifdef DEBUG_SRAM_MONITOR
	#warning "Release: DEBUG_SRAM_MONITOR enabled in Configuration.hh"
#endif

//Frequency of Timer 5
//100 = (1.0 / ( 16MHz / 64 / 25 = 10KHz)) * 1000000
//#define MICROS_INTERVAL 100

/// ticks of 100 Microsecond units since board initialization
static volatile micros_t centa_micros = 0;
static volatile uint8_t clock_wrap    = 0;

// Seconds since board initialization
static volatile micros_t seconds = 0;
static int16_t mcount            = 0;

uint8_t board_status;
#ifdef HAS_RGB_LED
static bool heating_lights_active;
#endif

#if defined(COOLING_FAN_PWM)
#define FAN_PWM_BITS 6
static uint8_t fan_pwm_bottom_count;
bool           fan_pwm_enable = false;
#endif

/// Instantiate static motherboard instance
Motherboard Motherboard::motherboard;

/// Create motherboard object
Motherboard::Motherboard() :
#ifdef MODEL_REPLICATOR2
	therm_sensor(THERMOCOUPLE_DO,THERMOCOUPLE_SCK,THERMOCOUPLE_DI, THERMOCOUPLE_CS),
#endif
#if defined(HAS_I2C_LCD) || defined(HAS_VIKI_INTERFACE)
      lcd(),
#else
      lcd(LCD_STROBE, LCD_DATA, LCD_CLK),
#endif
	messageScreen(),
	mainMenu(),
	finishedPrintMenu(),
#ifdef HAS_VIKI_INTERFACE
      //The VIKI is both an LCD and a buttonArray, so pass it twice.      
      interfaceBoard(lcd, lcd, &mainMenu, &monitorModeScreen,
                     &messageScreen, &finishedPrintMenu),
#else
      interfaceBoard(buttonArray, lcd, &mainMenu, &monitorModeScreen,
                     &messageScreen, &finishedPrintMenu),
#endif
	platform_thermistor(PLATFORM_PIN, TemperatureTable::table_thermistor),
	platform_heater(platform_thermistor,platform_element,SAMPLE_INTERVAL_MICROS_THERMISTOR,
			// NOTE: MBI had the calibration_offset as 0 which then causes
			//       the calibration_offset for Tool 0 to be used instead
            		eeprom_offsets::T0_DATA_BASE + toolhead_eeprom_offsets::HBP_PID_BASE, false, 2),
	using_platform(eeprom::getEeprom8(eeprom_offsets::HBP_PRESENT, 1)),
// MOD Trax BEGIN
#ifdef HAS_ENCLOSURE
	enclosure_thermistor(ENCLOSURE_PIN, TemperatureTable::table_thermistor), // X-TODO
	enclosure_heater(enclosure_thermistor,enclosure_element,SAMPLE_INTERVAL_MICROS_THERMISTOR,
            		eeprom_offsets::T1_DATA_BASE + toolhead_eeprom_offsets::HBP_PID_BASE, false, 3),
	using_enclosure(eeprom::getEeprom8(eeprom_offsets::CHE_PRESENT, 1)),
#endif
// MOD Trax END
#ifdef MODEL_REPLICATOR2
	Extruder_One(0, EXA_PWR, EXA_FAN, ThermocoupleReader::CHANNEL_ONE, eeprom_offsets::T0_DATA_BASE),
	Extruder_Two(1, EXB_PWR, EXB_FAN, ThermocoupleReader::CHANNEL_TWO, eeprom_offsets::T1_DATA_BASE)
#else
	Extruder_One(0, EX1_PWR, EX1_FAN, THERMOCOUPLE_CS1,eeprom_offsets::T0_DATA_BASE),
	Extruder_Two(1, EX2_PWR, EX2_FAN, THERMOCOUPLE_CS2,eeprom_offsets::T1_DATA_BASE)
#endif
#ifdef PSTOP_SUPPORT
	, pstop_enabled(0)
#endif
// MOD Trax BEGIN
#ifdef PSTOP_MONITOR
	, pstop_enc_calibr(eeprom::getEepromFixed16(eeprom_offsets::PSTOP_CALIBRATION,1)) //7.3;
	, pstop_tolerance((100.0 - eeprom::getEeprom8(eeprom_offsets::PSTOP_TOLERANCE, PSTOP_DEFAULT_TOLERANCE))/100.0)
	, pstop_waiting(eeprom::getEeprom16(eeprom_offsets::PSTOP_WAITING,PSTOP_DEFAULT_WAITING))
	
	, last_seconds(0)
	
	, acc_counter(0)
	, list_pos(0)
	, run_seconds(0)
	
	, was_building(false)
	
	, pstop_test_R(0.0)
	, pstop_test_L(0.0)
#endif
// MOD Trax END
{
}

void Motherboard::setupAccelStepperTimer() {
        STEPPER_TCCRnA = 0x00;
        STEPPER_TCCRnB = 0x0A; //CTC1 + / 8 = 2Mhz.
        STEPPER_TCCRnC = 0x00;
        STEPPER_OCRnA  = 0x2000; //1KHz
        STEPPER_TIMSKn = 0x02; // turn on OCR3A match interrupt  [OCR5A for Rep 2]
}

#define ENABLE_TIMER_INTERRUPTS		TIMSK2		|= (1<<OCIE2A); \
                			STEPPER_TIMSKn	|= (1<<STEPPER_OCIEnA)

#define DISABLE_TIMER_INTERRUPTS	TIMSK2		&= ~(1<<OCIE2A); \
                			STEPPER_TIMSKn	&= ~(1<<STEPPER_OCIEnA)

// Initialize Timers
//
// Priority: 2, 1, 0, 3, 4, 5
//
// Replicator 1
//	0 = Buzzer
//	1 = Extruder 2 (PWM)
//	2 = Extruder/Advance timer
//	3 = Stepper
//	4 = Extruder 1 (PWM)
//	5 = Microsecond timer, "M" flasher, check SD card switch,
//             check P-Stop switch
//
//	Timer 0 = 8 bit with PWM
//	Timers 1,3,4,5 = 16 bit with PWM
//	Timer 2 = 8 bit with PWM
//
// Replicator 2
//	0 =
//	1 = Stepper
//	2 = Extruder/Advance timer
//	3 = Extruders (PWM)
//	4 = Buzzer
//	5 = Microsecond timer, "M" flasher, check SD card switch
//
//	Timer 0 = 8 bit with PWM
//	Timers 1,3,4,5 = 16 bit with PWM
//	Timer 2 = 8 bit with PWM

void Motherboard::initClocks(){

	// Reset and configure timer 0, the piezo buzzer timer
	// No interrupt, frequency controlled by Piezo

	// this call is handled in Piezo::reset() -- no need to make it here as well
	// Piezo::shutdown_timer();

#ifdef JKN_ADVANCE
	// Reset and configure timer 2
	// Timer 2 is 8 bit
	//
	//   - Extruder/Advance timer

	TCCR2A = 0x02;	// CTC
	TCCR2B = 0x04;	// prescaler at 1/64
	OCR2A  = 25;	// Generate interrupts 16MHz / 64 / 25 = 10KHz
	TIMSK2 = 0x02;  // turn on OCR2A match interrupt
#endif

	// Choice of timer is done in Configuration.hh via STEPPER_ macros
	//
	// Rep 1:
	//   Reset and configure timer 3, the stepper interrupt timer.
	//   ISR(TIMER3_COMPA_vect)
	//
	// Rep 2:
	//   Reset and configure timer 1, the stepper interrupt timer.
	//   ISR(TIMER1_COMPA_vect)

        setupAccelStepperTimer();

#ifdef MODEL_REPLICATOR2
	// reset and configure timer 3, the Extruders timer
	// Mode: Fast PWM with TOP=0xFF (8bit) (WGM3:0 = 0101), cycle freq= 976 Hz
	// Prescaler: 1/64 (250 KHz)
	TCCR3A = 0b00000001;
	TCCR3B = 0b00001011; /// set to PWM mode
	OCR3A  = 0;
	OCR3C  = 0;
	TIMSK3 = 0b00000000; // no interrupts needed
#else
	// reset and configure timer 1, the Extruder Two PWM timer
	// Mode: Fast PWM with TOP=0xFF (8bit) (WGM3:0 = 0101), cycle freq= 976 Hz
	// Prescaler: 1/64 (250 KHz)
	// No interrupt, PWM controlled by ExtruderBoard

	TCCR1A = 0b00000001;
	TCCR1B = 0b00001011;
	OCR1A  = 0x00;
	OCR1B  = 0x00;
	TIMSK1 = 0x00;	//No interrupts

	// reset and configure timer 4, the Extruder One PWM timer
	// Mode: Fast PWM with TOP=0xFF (8bit) (WGM3:0 = 0101), cycle freq= 976 Hz
	// Prescaler: 1/64 (250 KHz)
	// No interrupt, PWM controlled by ExtruderBoard

	TCCR4A = 0b00000001;
	TCCR4B = 0b00001011;
	TCCR4C = 0x00;
	OCR4A  = 0x00;
	OCR4B  = 0x00;
	TIMSK4 = 0x00;	//No interrupts
#endif

#if defined(PSTOP_SUPPORT)
	pstop_enabled = eeprom::getEeprom8(eeprom_offsets::PSTOP_ENABLE, 0);
#if defined(PSTOP_VECT) && !defined(PSTOP_MONITOR) // MOD Trax
// #if defined(PSTOP_VECT)
	// We set a LOW pin change interrupt on the X min endstop
	if ( pstop_enabled == 1 ) {
		PSTOP_MSK |= ( 1 << PSTOP_PCINT );
		PCICR     |= ( 1 << PSTOP_PCIE );
	}
#endif
#endif

	// Reset and configure timer 5:
	// Timer 5 is 16 bit
	//
	//   - Microsecond timer, SD card check timer, P-Stop check timer, LED flashing timer

	TCCR5A = 0x00; // WGM51:WGM50 00
	TCCR5B = 0x0B; // WGM53:WGM52 10 (CTC) CS52:CS50 011 (/64) => CTC, 250 KHz
	TCCR5C = 0x00;
	OCR5A  =   25; // 250KHz / 25 => 10 KHz
	TIMSK5 = 0x02; // | ( 1 << OCIE5A  -> turn on OCR5A match interrupt
}

/// Reset the motherboard to its initial state.
/// This only resets the board, and does not send a reset
/// to any attached toolheads.
void Motherboard::init() {

	SoftI2cManager::getI2cManager().init();

	// Check if the interface board is attached
	hasInterfaceBoard = interface::isConnected();

	initClocks();

	// Configure the debug pins.
	DEBUG_PIN.setDirection(true);
	DEBUG_PIN1.setDirection(true);
	DEBUG_PIN2.setDirection(true);
	DEBUG_PIN3.setDirection(true);	
	DEBUG_PIN4.setDirection(true);
	DEBUG_PIN5.setDirection(true);
	DEBUG_PIN6.setDirection(true);
#ifdef MODEL_REPLICATOR
	DEBUG_PIN7.setDirection(true);
#endif 
}

void Motherboard::reset(bool hard_reset) {

#if HONOR_DEBUG_PACKETS
	indicateError(0); // turn on blinker
#endif

	// Initialize the host and slave UARTs
	UART::getHostUART().enable(true);
	UART::getHostUART().in.reset();

	if (hasInterfaceBoard) {

		// Make sure our interface board is initialized
		interfaceBoard.init();

		INTERFACE_DDR |= INTERFACE_LED;
		INTERFACE_LED_PORT |= INTERFACE_LED;

		splashScreen.hold_on = false;
		interfaceBoard.pushScreen(&splashScreen);

		if ( hard_reset )
			_delay_ms(3000);

		// Finally, set up the interface
		interface::init(&interfaceBoard, &lcd);

		interface_update_timeout.start(interfaceBoard.getUpdateRate());
	}

	// interface LEDs default to full ON
	interfaceBlink(0,0);

	// only call the piezo buzzer on full reboot start up
	// do not clear heater fail messages, though the user should not be able to soft reboot from heater fail
	if ( hard_reset ) {
		// Force reinitialization of TWI on hard reset.
		TWI_init(true);
#ifdef HAS_RGB_LED
		RGB_LED::init();
#endif
		Piezo::playTune(TUNE_SAILFISH_STARTUP);

		heatShutdown = 0;
		heatFailMode = HEATER_FAIL_NONE;
	}

	board_status = STATUS_NONE | STATUS_PREHEATING;
#ifdef HAS_RGB_LED
	heating_lights_active = false;
#endif

#ifdef MODEL_REPLICATOR2 
	therm_sensor.init();
	therm_sensor_timeout.start(THERMOCOUPLE_UPDATE_RATE);
#else
	cutoff.init();
	extruder_manage_timeout.start(SAMPLE_INTERVAL_MICROS_THERMOCOUPLE);
#endif

	// initialize the extruders
	Extruder_One.reset();
	Extruder_Two.reset();
    
	HBP_HEAT.setDirection(true);
	platform_thermistor.init();
	platform_heater.reset();
	platform_timeout.start(SAMPLE_INTERVAL_MICROS_THERMISTOR);

// MOD Trax BEGIN
#ifdef HAS_ENCLOSURE
	CHE_HEAT.setDirection(true);
	enclosure_thermistor.init();
	enclosure_heater.reset();
	enclosure_timeout.start(SAMPLE_INTERVAL_MICROS_THERMISTOR);
#endif
// MOD Trax END

	// Note it's less code to turn them all off at once
	//  then to conditionally turn of or disable
	heatersOff(true);

	// disable extruder two if sigle tool machine
	if ( eeprom::isSingleTool() )
	     Extruder_Two.disable(true);

	// disable platform heater if no HBP
	if ( !eeprom::hasHBP() )
	    platform_heater.disable(true);

// MOD Trax BEGIN
#ifdef HAS_ENCLOSURE
	if ( !eeprom::hasCHE() )
	    enclosure_heater.disable(true);
#endif
// MOD Trax END

	// user_input_timeout.start(USER_INPUT_TIMEOUT);
#ifdef HAS_RGB_LED
	RGB_LED::setDefaultColor();
#endif
	buttonWait = false;

	// turn off the active cooling fan
	EXTRA_FET.setDirection(true);
	setExtra(false);
}

/// Get the number of microseconds that have passed since
/// the board was booted.
micros_t Motherboard::getCurrentCentaMicros(uint8_t *wrap) {
	micros_t micros_snapshot;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
	     micros_snapshot = centa_micros;
	     *wrap = clock_wrap;
	}
	return micros_snapshot;
}

micros_t Motherboard::getCurrentSeconds() {
	micros_t seconds_snapshot;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		seconds_snapshot = seconds;
	}
	return seconds_snapshot;
}

/// Run the motherboard interrupt
void Motherboard::doStepperInterrupt() {
	//We never ignore interrupts on pause, because when paused, we might
	//extrude filament to change it or fix jams

	DISABLE_TIMER_INTERRUPTS;
	sei();

	steppers::doStepperInterrupt();

	cli();
	ENABLE_TIMER_INTERRUPTS;

#ifdef ANTI_CLUNK_PROTECTION
	//Because it's possible another stepper interrupt became due whilst
	//we were processing the last interrupt, and had stepper interrupts
	//disabled, we compare the counter to the requested interrupt time
	//to see if it overflowed.  If it did, then we reset the counter, and
	//schedule another interrupt for very shortly into the future.
	if ( STEPPER_TCNTn >= STEPPER_OCRnA ) {
		STEPPER_OCRnA = 0x01;	//We set the next interrupt to 1 interval, because this will cause the
					//interrupt to  fire again on the next chance it has after exiting this
					//interrupt, i.e. it gets queued.

		STEPPER_TCNTn = 0;	//Reset the timer counter

		//debug_onscreen1 ++;
	}
#endif
}

void Motherboard::HeatingAlerts() {
	int16_t setTemp = 0;
	int16_t deltaTemp = 0;
	int16_t top_temp = 0;
#ifdef HAS_RGB_LED
	int16_t div_temp = 0;
#endif

	/// show heating progress
	// TODO: top temp should use preheat temps stored in eeprom instead of a hard coded value
	Heater& heater0 = getExtruderBoard(0).getExtruderHeater();
	Heater& heater1 = getExtruderBoard(1).getExtruderHeater();

	if ( heater0.isHeating() || heater1.isHeating() ||
	     getPlatformHeater().isHeating() ) { // X-TODO ?
		
		if ( getPlatformHeater().isHeating() ) {
			deltaTemp = getPlatformHeater().getDelta()*2;
			setTemp = (int16_t)(getPlatformHeater().get_set_temperature())*2;
			top_temp = 260;
		}
		else {
			/// clear extruder paused states if needed
			if ( heater0.isPaused() ) heater0.Pause(false);
			if ( heater1.isPaused() ) heater1.Pause(false);
		}
		if ( heater0.isHeating() && !heater0.isPaused() )
		{
			deltaTemp += heater0.getDelta();
			setTemp += (int16_t)(heater0.get_set_temperature());
			top_temp += 260;
		}
		if ( heater1.isHeating() && !heater1.isPaused() ) {
			
			deltaTemp += heater1.getDelta();
			setTemp += (int16_t)(heater1.get_set_temperature());
			top_temp += 120;
		}

#ifdef HAS_RGB_LED
		if ( setTemp < deltaTemp )
			div_temp = (top_temp - setTemp);
		else
			div_temp = setTemp;
             
		if ( (div_temp != 0) && eeprom::heatLights() ) {
			if( !heating_lights_active ) {
#ifdef MODEL_REPLICATOR
				RGB_LED::clear();
#endif
				heating_lights_active = true;
			}
			RGB_LED::setColor((255*abs((setTemp - deltaTemp)))/div_temp, 0, (255*deltaTemp)/div_temp, false);
		}
#endif
	}
#ifdef HAS_RGB_LED
	else {
		if ( heating_lights_active ) {
			RGB_LED::setDefaultColor();
			heating_lights_active = false;
		}
	}
#endif
}

bool connectionsErrorTriggered = false;
void Motherboard::heaterFail(HeaterFailMode mode, uint8_t slave_id) {

	// record heat fail mode
	heatFailMode = mode;

	if ( heatFailMode == HEATER_FAIL_NOT_PLUGGED_IN ) {

		// MBI's code has a design flaw whereby it manages all heaters, whether they exist or
		// not and even if they are disabled!  Thus, the Heater::manage_temperature() routine
		// will attempt termperature reads on disabled heaters.  That, in turn, leads to failures
		// on non-existent or otherwise disabled heaters.  And when they fail, this routine is called.
		// Thus this routine ends up having to decide whether a failure is a false alarm or not.
		// The logic for doing that is non-trivial.  But, more importantly, such logic should not
		// have to exist: the manage routine shouldn't be trying to manage non-existent or disabled
		// heaters!

		// It would seem that some of the code in this routine is the non-trivial logic trying to
		// figure out whether or not a heater error should be ignored.  Thus, some of the code
		// here is a work around to the deeper problem.  Worse yet, there's an "if" test below
		// which is simply incorrect: it allows a Rep 2 (single heater) to keep on running when
		// its single heater fails with a "not plugged in" error:
		//
		//    !platform_heater.has_failed() == !false == true  // no heated platform on Rep 2
		//    eeprom::isSingleTool() == true                   // Rep 2 is single tool
		//    !(Extruder_One...has_failed() && Extruder_Two...has_failed()) == !(true && false) == !(false) == true
		//      ^^^ By the above, BOTH heaters have to fail on a Rep 2, but a Rep 2 has only one heater
		// 
		// Net result of the above logic is to always ignore a "not plugged in" error on a Rep 2 *unless*
		// the management routines just happen to also trigger an error on the non-existent HBP or
		// non-existent 2nd extruder.
#if 0
		// BEGIN MBI's original comment
		// if single tool, one heater is not plugged in on purpose
		// do not trigger a heatFail message unless both heaters are unplugged
		if ( !platform_heater.has_failed() && eeprom::isSingleTool() &&
			(!(Extruder_One.getExtruderHeater().has_failed() && Extruder_Two.getExtruderHeater().has_failed())) )
			return;
		// only fire the heater not connected error once.  The user should be able to dismiss this one
		else
#endif
		if ( connectionsErrorTriggered )
			return;
		else
			connectionsErrorTriggered = true;
	}

	// flag heat shutdown response
	heatShutdown = slave_id + 1;  // slave_ids are 0 = tool 0, 1 = tool 1, 2 = platform
}

// Motherboard class waits for a button press from the user
// used for firmware initiated error reporting
void Motherboard::startButtonWait(){
	// blink the interface LEDs
	interfaceBlink(25,15);

	interfaceBoard.waitForButton(0xFF);
	buttonWait = true;
}

// set an error message on the interface and wait for user button press
void Motherboard::errorResponse(const prog_uchar *msg, bool reset, bool incomplete) {
	errorResponse(msg, 0, reset, incomplete);
}

void Motherboard::errorResponse(const prog_uchar *msg1, const prog_uchar *msg2,
				bool reset, bool incomplete) {
	interfaceBoard.errorMessage(msg1, msg2, incomplete);
	startButtonWait();
	reset_request = reset;
}

bool triggered = false;
bool extruder_update = false;

// main motherboard loop
void Motherboard::runMotherboardSlice() {
	bool interface_updated = false;

	// check for user button press
	// update interface screen as necessary
	if ( hasInterfaceBoard ) {
		interfaceBoard.doInterrupt();
		// stagger motherboard updates so that they do not all occur on the same loop
		if ( interface_update_timeout.hasElapsed() ) {
			interfaceBoard.doUpdate();
			interface_update_timeout.start(interfaceBoard.getUpdateRate());
			interface_updated = true;
		}
	}

	if ( isUsingPlatform() && platform_timeout.hasElapsed() ) {
		// manage heating loops for the HBP
		platform_heater.manage_temperature();
		platform_timeout.start(SAMPLE_INTERVAL_MICROS_THERMISTOR);
	}
	
// MOD Trax BEGIN
#ifdef HAS_ENCLOSURE
	if ( isUsingEnclosure() && enclosure_timeout.hasElapsed() ) {
		// manage heating loops for the CHE
		enclosure_heater.manage_temperature();
		enclosure_timeout.start(SAMPLE_INTERVAL_MICROS_THERMISTOR);
	}
#endif
// MOD Trax END


	// if waiting on button press
	if ( buttonWait ) {
		// if user presses enter
		if ( interfaceBoard.buttonPushed() ) {

			// set interface LEDs to solid
			interfaceBlink(0,0);

#ifdef HAS_RGB_LED
			// restore default LED behavior
			RGB_LED::setDefaultColor();
#endif

			//clear error messaging
			buttonWait = false;
			interfaceBoard.popScreen();

			if ( reset_request )
				host::stopBuildNow();
			triggered = false;
		}
	}

	// if no user input for USER_INPUT_TIMEOUT, shutdown heaters and warn user
	// don't do this if a heat failure has occured
	// ( in this case heaters are already shutdown and separate error messaging used)
	if ( user_input_timeout.hasElapsed() &&
	     !heatShutdown &&
	     (host::getHostState() != host::HOST_STATE_BUILDING_FROM_SD) &&
	     (host::getHostState() != host::HOST_STATE_BUILDING) ) {

		BOARD_STATUS_SET(STATUS_HEAT_INACTIVE_SHUTDOWN);
		BOARD_STATUS_CLEAR(STATUS_PREHEATING);

		// alert user if heaters are not already set to 0
		if ( (Extruder_One.getExtruderHeater().get_set_temperature() > 0) ||
		     (Extruder_Two.getExtruderHeater().get_set_temperature() > 0) ||
// MOD Trax BEGIN
#ifdef HAS_ENCLOSURE
					(enclosure_heater.get_set_temperature() > 0) ||
#endif
// MOD Trax END
		     (platform_heater.get_set_temperature() > 0) ) {
			interfaceBoard.errorMessage(HEATER_INACTIVITY_MSG, false);
			startButtonWait();
#ifdef HAS_RGB_LED
			// turn LEDs blue
			RGB_LED::setColor(0,0,255, true);
#endif
		}
		// set tempertures to 0
		heatersOff(true);

		// clear timeout
		user_input_timeout.clear();
	}

	// respond to heatshutdown.  response only needs to be called once
	if ( heatShutdown && !triggered && !Piezo::isPlaying() ) {
		triggered = true;

		// rgb led response
		interfaceBlink(10,10);

		const prog_uchar *msg, *msg2 = 0;
		if ( heatShutdown < 3 ) {
			if ( eeprom::isSingleTool() ) msg = HEATER_TOOL_MSG;
			else if ( heatShutdown == 1 ) msg = HEATER_TOOL0_MSG;
			else msg = HEATER_TOOL1_MSG;
		}
// MOD Trax BEGIN
#ifdef HAS_ENCLOSURE
		else if ( heatShutdown == 4 ) msg = HEATER_ENCLOSURE_MSG;
#endif
// MOD Trax END
		else msg = HEATER_PLATFORM_MSG;

		/// error message
		switch (heatFailMode) {
		case HEATER_FAIL_SOFTWARE_CUTOFF:
		        msg2 = HEATER_FAIL_SOFTWARE_CUTOFF_MSG;
			break;
		case HEATER_FAIL_NOT_HEATING:
		        msg2 = HEATER_FAIL_NOT_HEATING_MSG;
			break;
		case HEATER_FAIL_DROPPING_TEMP:
		        msg2 = HEATER_FAIL_DROPPING_TEMP_MSG;
			break;
		case HEATER_FAIL_NOT_PLUGGED_IN:
			errorResponse(msg, HEATER_FAIL_NOT_PLUGGED_IN_MSG);
			// handled by Heater.cc
#if 0
			if ( Extruder_One.getExtruderHeater().has_failed() )
				Extruder_One.getExtruderHeater().set_target_temperature(0);
			if ( Extruder_Two.getExtruderHeater().has_failed() )
				Extruder_Two.getExtruderHeater().set_target_temperature(0);
// MOD Trax BEGIN
#ifdef HAS_ENCLOSURE
			if ( enclosure_heater.has_failed() )
				enclosure_heater.set_target_temperature(0);
#endif
// MOD Trax END
			if ( platform_heater.has_failed() )
				platform_heater.set_target_temperature(0);
#endif
			heatShutdown = 0;
			return;
		case HEATER_FAIL_BAD_READS:
			errorResponse(msg, HEATER_FAIL_READ_MSG);
			heatShutdown = 0;
			return;
		default:
			break;
		}
		interfaceBoard.errorMessage(msg, msg2);
		// All heaters off
		heatersOff(true);

		//error sound
		Piezo::playTune(TUNE_ERROR);

#ifdef HAS_RGB_LED
		// blink LEDS red
		RGB_LED::errorSequence();
#endif

		// disable command processing and steppers
		host::heatShutdown();
		command::heatShutdown();
		steppers::abort();
		steppers::enableAxes(0xff, false);
	}

	// Temperature monitoring thread
#ifdef MODEL_REPLICATOR2
	if ( therm_sensor_timeout.hasElapsed() && !interface_updated ) {
		if ( therm_sensor.update() ) {
			therm_sensor_timeout.start(THERMOCOUPLE_UPDATE_RATE);
			switch (therm_sensor.getLastUpdated()) {
			case ThermocoupleReader::CHANNEL_ONE:
				Extruder_One.runExtruderSlice();
				HeatingAlerts();
				break;
			case ThermocoupleReader::CHANNEL_TWO:
				Extruder_Two.runExtruderSlice();
				break;
			default:
				break;
			}
		}
	}
#else
	// stagger mid accounts for the case when we've just run the interface update
	if ( extruder_manage_timeout.hasElapsed() && !interface_updated ) {
		Extruder_One.runExtruderSlice();
		HeatingAlerts();
		extruder_manage_timeout.start(SAMPLE_INTERVAL_MICROS_THERMOCOUPLE);
		extruder_update = true;
	}
	else if (extruder_update) {
		Extruder_Two.runExtruderSlice();
		extruder_update = false;
	}
#endif

// MOD Trax BEGIN
#ifdef PSTOP_MONITOR
		micros_t seconds = getCurrentSeconds();
		if(seconds - last_seconds >= ACC_INTER) { // check filament progress every 3 seconds
			last_seconds = seconds;


/*#ifdef UART_DEBUG
	UART::getHostUART().printInt(board_status,1);
	UART::getHostUART().printString(" ");
	UART::getHostUART().printFloat(Motherboard::getBoard().getEnclosureHeater().get_current_temperature(),1);
	UART::getHostUART().printString(" ");
	UART::getHostUART().printFloat(Motherboard::getBoard().getEnclosureHeater().get_set_temperature(),1);
	UART::getHostUART().printString("\r\n");
#endif*/
		
			bool is_building = //(host::getHostState() == host::HOST_STATE_BUILDING 
												 //		|| host::getHostState() == host::HOST_STATE_BUILDING_FROM_SD) &&
													 host::getBuildState() == host::BUILD_RUNNING;
		
			if(!is_building)
				was_building = false;
			else if(!was_building){
				was_building = true;
			
				for(int i=0; i < LIST_LENGTH; i++) {
					list_enc_R[i] = list_step_R[i] = list_enc_L[i] = list_step_L[i] = 0;
				}
				
				acc_counter = 0;
				list_pos = 0;
				
				run_seconds = seconds;
			}
			else {
				int16_t cur_enc_R = 0;
				TWI_write_byte(PSTOP_MONITOR, 0x01); //READ_R
				_delay_us(10);
				TWI_read_byte(PSTOP_MONITOR, (uint8_t*)&cur_enc_R, 2);
				
				volatile extern int16_t steps_R;
				int16_t cur_step_R = steps_R;
				steps_R = 0;
				
				int16_t cur_enc_L = 0;
				TWI_write_byte(PSTOP_MONITOR, 0x02); //READ_L
				_delay_us(10);
				TWI_read_byte(PSTOP_MONITOR, (uint8_t*)&cur_enc_L, 2);
				
				volatile extern int16_t steps_L;
				int16_t cur_step_L = steps_L;
				steps_L = 0;
				
				list_enc_R[list_pos] += cur_enc_R;
				list_step_R[list_pos] += cur_step_R;
				
				list_enc_L[list_pos] += cur_enc_L;
				list_step_L[list_pos] += cur_step_L;
				
				if(++acc_counter >= LIST_ACC) {
					acc_counter = 0;
					
					if(++list_pos >= LIST_LENGTH)
						list_pos = 0;
					
					list_enc_R[list_pos] = 0;
					list_step_R[list_pos] = 0;
					
					list_enc_L[list_pos] = 0;
					list_step_L[list_pos] = 0;
				}
				
				float enc_R = 0;
				float step_R = 0;
				float enc_L = 0;
				float step_L = 0;
				
				for(int i=0; i < LIST_LENGTH; i++) {
					enc_R += list_enc_R[i];
					step_R += list_step_R[i];
					enc_L += list_enc_L[i];
					step_L += list_step_L[i];
				}
				enc_R *= pstop_enc_calibr;
				enc_L *= pstop_enc_calibr;
				
				// delta in percent 1 all is good, fail < 0.80
				pstop_test_R = step_R ? (enc_R / step_R) : 0.0;
				pstop_test_L = step_L ? (enc_L / step_L) : 0.0;
				
				if(step_R > 0 || step_L > 0) { // wait for the print to to start
	#if defined(PSTOP_SUPPORT)
					micros_t run_duration = (seconds > run_seconds) ? seconds - run_seconds : 0;
					if(run_duration > 0x7FFF) // dont overflow if build takes more than 9 hours
						run_duration = 0x7FFF;
					if(run_duration > pstop_waiting)
					{
						if ( (Motherboard::getBoard().pstop_enabled == 1) 
								&& ( (cur_step_R > 0 && pstop_test_R < pstop_tolerance) || (cur_step_L > 0 && pstop_test_L < pstop_tolerance) ) )
							command::pstop_triggered = true;
					}
	#endif
				}
			}
		}
#endif
// MOD Trax END

}

// reset user timeout to start from zero
void Motherboard::resetUserInputTimeout(){
	user_input_timeout.start(USER_INPUT_TIMEOUT);
}

/// Timer three comparator match interrupt
ISR(STEPPER_TIMERn_COMPA_vect) {
	Motherboard::getBoard().doStepperInterrupt();
}

#if defined(PSTOP_SUPPORT) && defined(PSTOP_VECT) && !defined(PSTOP_MONITOR) // MOD Trax
//#if defined(PSTOP_SUPPORT) && defined(PSTOP_VECT)

ISR(PSTOP_VECT) {
	if ( (Motherboard::getBoard().pstop_enabled == 1) && (PSTOP_PORT.getValue() == 0) ) command::pstop_triggered = true;
}

#endif

/// Number of times to blink the debug LED on each cycle
volatile uint8_t blink_count = 0;

/// number of cycles to hold on and off in each interface LED blink
uint8_t interface_on_time = 0;
uint8_t interface_off_time = 0;

/// The current state of the debug LED
enum {
	BLINK_NONE = 0,
	BLINK_ON,
	BLINK_OFF
};

/// state trackers for blinking LEDS
static uint8_t interface_blink_state = BLINK_NONE;

#if HONOR_DEBUG_PACKETS

/// Timer2 overflow cycles that the LED remains on while blinking
#define OVFS_ON 18
/// Timer2 overflow cycles that the LED remains off while blinking
#define OVFS_OFF 18
/// Timer2 overflow cycles between flash cycles
#define OVFS_PAUSE 80

/// Number of overflows remaining on the current blink cycle
int blink_ovfs_remaining = 0;

/// Number of blinks performed in the current cycle
int blinked_so_far = 0;

int blink_state = BLINK_NONE;

/// Write an error code to the debug pin.
void Motherboard::indicateError(int error_code) {
	if (error_code == 0) {
		blink_state = BLINK_NONE;
		DEBUG_PIN.setValue(false);
	}
	else if (blink_count != error_code) {
		blink_state = BLINK_OFF;
	}
	blink_count = error_code;
}
#endif

// set on / off period for blinking interface LEDs
// if both times are zero, LEDs are full on, if just on-time is zero, LEDs are full OFF
void Motherboard::interfaceBlink(uint8_t on_time, uint8_t off_time) {
	if ( off_time == 0 ) {
		interface_blink_state = BLINK_NONE;
		INTERFACE_LED_PORT |= INTERFACE_LED;
	}
	else if ( on_time == 0 ) {
		interface_blink_state = BLINK_NONE;
		INTERFACE_LED_PORT &= ~(INTERFACE_LED);
	}
	else {
		interface_on_time = on_time;
		interface_off_time = off_time;
		interface_blink_state = BLINK_ON;
	}
}

#ifdef JKN_ADVANCE
/// Timer 2 extruder advance
ISR(TIMER2_COMPA_vect) {
	steppers::doExtruderInterrupt();
}
#endif

/// Number of overflows remaining on the current overflow blink cycle
uint8_t interface_ovfs_remaining = 0;
uint8_t blink_overflow_counter = 0;

volatile micros_t m2;

#if defined(FF_CREATOR_X) && defined(__AVR_ATmega2560__)  ///add by FF_OU, impletement a softPWM to lower the HBP output
volatile uint8_t pwmcnt = 0;
#define PWM_H   210
#define PWM_COUST   255
#endif

/// Timer 5 overflow interrupt
ISR(TIMER5_COMPA_vect) {
#if defined(COOLING_FAN_PWM)
     static uint8_t fan_pwm_counter = 255;
#endif

     // Motherboard::getBoard().UpdateMicros();
  if ( ++centa_micros == 0 ) ++clock_wrap;
  if ( ++mcount >= 10000 ) {
	  seconds += 1;
	  mcount = 0;
  }

#if defined(FF_CREATOR_X) && defined(__AVR_ATmega2560__)  ///add by FF_OU, impletement a softPWM to lower the HBP output
     //softpwm
    if (pwmcnt < PWM_H) {
        HBP_HEAT.setValue(true);
    } else {
        HBP_HEAT.setValue(false);
    }
    if (pwmcnt >= PWM_COUST)
        pwmcnt = 0;
    else
        pwmcnt++;
#endif 

#if defined(COOLING_FAN_PWM)
    if ( fan_pwm_enable ) {
	 // fan_pwm_counter is uint8_t
	 //   we expect it to wrap such that 255 + 1 ==> 0
	 if ( ++fan_pwm_counter == 0 )
	      fan_pwm_counter = 256 - (1 << FAN_PWM_BITS);
	 EX_FAN.setValue(fan_pwm_counter <= fan_pwm_bottom_count);
    }
#endif

	if (blink_overflow_counter++ <= 0xA4)
	     return;

	blink_overflow_counter = 0;

#ifndef BROKEN_SD
	/// Check SD Card Detect
	if ( SD_DETECT_PIN.getValue() != 0x00 ) sdcard::mustReinit = true;
#endif

#if defined(PSTOP_SUPPORT)
#if !defined(PSTOP_VECT) && !defined(PSTOP_MONITOR) // MOD Trax
	if ( (Motherboard::getBoard().pstop_enabled == 1) && (PSTOP_PORT.getValue() == 0) ) command::pstop_triggered = true;
#endif
#if defined(PSTOP_ZMIN_LEVEL) && defined(Z_MIN_STOP_PORT) && defined(AUTO_LEVEL)
        if ( (Motherboard::getBoard().pstop_enabled == 1) && (Z_MIN_STOP_PORT.getValue() == 0) ) command::possibleZLevelPStop();
#endif
#if defined(PSTOP_2_SUPPORT)
	Y_MIN_STOP_PORT.setValue(extrusion_seen[0]); extrusion_seen[0] = false;
        #if EXTRUDERS > 1
	Z_MAX_STOP_PORT.setValue(extrusion_seen[1]); extrusion_seen[1] = false;
	#endif
#endif
#endif

#if HONOR_DEBUG_PACKETS
	/// Debug LEDS on Motherboard
	if (blink_ovfs_remaining > 0) {
		blink_ovfs_remaining--;
	} else {
		if (blink_state == BLINK_ON) {
			blinked_so_far++;
			blink_state = BLINK_OFF;
			blink_ovfs_remaining = OVFS_OFF;
			DEBUG_PIN.setValue(false);
		} else if (blink_state == BLINK_OFF) {
			if (blinked_so_far >= blink_count) {
				blink_state = BLINK_PAUSE;
				blink_ovfs_remaining = OVFS_PAUSE;
			} else {
				blink_state = BLINK_ON;
				blink_ovfs_remaining = OVFS_ON;
				DEBUG_PIN.setValue(true);
			}
		} else if (blink_state == BLINK_PAUSE) {
			blinked_so_far = 0;
			blink_state = BLINK_ON;
			blink_ovfs_remaining = OVFS_ON;
			DEBUG_PIN.setValue(true);
		}
	}
#endif

	/// Interface Board LEDs
	if ( interface_blink_state != BLINK_NONE ) {
		if ( interface_ovfs_remaining != 0 )
			interface_ovfs_remaining--;
		else if ( interface_blink_state == BLINK_ON ) {
			interface_blink_state = BLINK_OFF;
			interface_ovfs_remaining = interface_on_time;
			INTERFACE_LED_PORT |= INTERFACE_LED;
		}
		else if ( interface_blink_state == BLINK_OFF ) {
			interface_blink_state = BLINK_ON;
			interface_ovfs_remaining = interface_off_time;
			INTERFACE_LED_PORT &= ~(INTERFACE_LED);
		}
	}
}

void Motherboard::setUsingPlatform(bool is_using) {
  using_platform = is_using;
}

// MOD Trax BEGIN
#ifdef HAS_ENCLOSURE
void Motherboard::setUsingEnclosure(bool is_using) {
  using_enclosure = is_using;
}
#endif
// MOD Trax END

#if defined(COOLING_FAN_PWM)

void Motherboard::setExtra(bool on) {
     uint16_t fan_pwm; // Will be multiplying 8 bits by 100(decimal)

     // Disable any fan PWM handling in Timer 5
     fan_pwm_enable = false;

     if ( !on ) {
	  EXTRA_FET.setValue(false);
	  return;
     }

     // See what the PWM setting is -- may have been changed
     ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
	  fan_pwm = (uint16_t)eeprom::getEeprom8(eeprom_offsets::COOLING_FAN_DUTY_CYCLE,
						 COOLING_FAN_DUTY_CYCLE_DEFAULT);
     }

     // Don't bother with PWM handling if the PWM is >= 100
     // Just turn the fan on full tilt
     if ( fan_pwm >= 100 ) {
	  EXTRA_FET.setValue(true);
	  return;
     }

     // Fan is to be turned on AND we are doing PWM
     // We start the bottom count at 255 - 64 and then wrap
     fan_pwm_enable = true;
     fan_pwm_bottom_count = (255 - (1 << FAN_PWM_BITS)) +
	  (int)(0.5 +  ((uint16_t)(1 << FAN_PWM_BITS) * fan_pwm) / 100.0);
}

#else

void Motherboard::setExtra(bool on) {
     EXTRA_FET.setValue(on);
}

#endif

#if defined(FF_CREATOR_X) && defined(__AVR_ATmega2560__)
void softpwmHBP(bool on){
    if (on)
    {
       HBP_HEAT.setDirection(true);
    }else{
        HBP_HEAT.setDirection(false);
    }
}
#endif 

void BuildPlatformHeatingElement::setHeatingElement(uint8_t value) {
	// This is a bit of a hack to get the temperatures right until we fix our
	// PWM'd PID implementation.  We reduce the MV to one bit, essentially.
	// It works relatively well.
  	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
#if defined(FF_CREATOR_X) && defined(__AVR_ATmega2560__)
        softpwmHBP(value != 0);
#else
        HBP_HEAT.setValue(value != 0);
#endif
	}

}

// MOD Trax BEGIN
#ifdef HAS_ENCLOSURE
#define HEATER_HOLD_OFF 15

void BuildEnclosureHeatingElement::setHeatingElement(uint8_t value) {

	//UART::getHostUART().printFloat(Motherboard::getBoard().getEnclosureHeater().get_current_temperature(),1);
	//UART::getHostUART().printString(" ");
		
	//UART::getHostUART().printFloat(value,1);
	//UART::getHostUART().printString(" ");
		
	static uint8_t oldValue = 0;
		
	static uint16_t lowpassAverage = 0;
	lowpassAverage = (9*lowpassAverage + value)/10;
	if((oldValue == 0 && lowpassAverage < 8) || value == 0) // must be more than 12% also 0 takes always precedence
		lowpassAverage = 0;
	
	value = lowpassAverage;

	//UART::getHostUART().printFloat(value,1);

	static micros_t lastOff = 0;
	uint8_t newValue = value;
	if(oldValue != 0 && value == 0){
		lastOff = Motherboard::getBoard().getCurrentSeconds();
	}
	else if(value != 0){
		if(Motherboard::getBoard().getCurrentSeconds() - lastOff < HEATER_HOLD_OFF) {
			//UART::getHostUART().printString("hold off");
			value = 0;
		}
	}
	oldValue = newValue; // bypas holdoff
	
	//UART::getHostUART().printString("\r\n");

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
  	CHE_HEAT.setValue(value != 0);
	}
}
#endif
// MOD Trax END

void Motherboard::heatersOff(bool platform)
{
	motherboard.getExtruderBoard(0).getExtruderHeater().Pause(false);
	motherboard.getExtruderBoard(0).getExtruderHeater().set_target_temperature(0);
	motherboard.getExtruderBoard(1).getExtruderHeater().Pause(false);
	motherboard.getExtruderBoard(1).getExtruderHeater().set_target_temperature(0);
	if ( platform ) motherboard.getPlatformHeater().set_target_temperature(0);
// MOD Trax BEGIN
#ifdef HAS_ENCLOSURE
	if ( platform ) motherboard.getEnclosureHeater().set_target_temperature(0);
#endif
// MOD Trax END
	BOARD_STATUS_CLEAR(Motherboard::STATUS_PREHEATING);
}


void Motherboard::interfaceBlinkOn()
{
	motherboard.interfaceBlink(25, 15);
}

void Motherboard::interfaceBlinkOff()
{
	motherboard.interfaceBlink(0, 0);
}

void Motherboard::pauseHeaters(bool pause)
{
	motherboard.getExtruderBoard(0).getExtruderHeater().Pause(pause);
	motherboard.getExtruderBoard(1).getExtruderHeater().Pause(pause);
}

#ifdef DEBUG_VALUE

/// Get the current error code.
uint8_t Motherboard::getCurrentError() {
	return blink_count;
}

//Sets the debug leds to value
//This is in C, as we don't want C++ causing issues
//Note this is quite slow.

void setDebugValue(uint8_t value) {
	static bool initialized = false;

	if ( ! initialized ) {
		DEBUG_PIN1.setDirection(true);
		DEBUG_PIN2.setDirection(true);
		DEBUG_PIN3.setDirection(true);
		DEBUG_PIN4.setDirection(true);
		DEBUG_PIN5.setDirection(true);
		DEBUG_PIN6.setDirection(true);
		DEBUG_PIN7.setDirection(true);
		DEBUG_PIN8.setDirection(true);

		initialized = true;
	}

        DEBUG_PIN1.setValue(value & 0x80);
        DEBUG_PIN2.setValue(value & 0x40);
        DEBUG_PIN3.setValue(value & 0x20);
        DEBUG_PIN4.setValue(value & 0x10);
        DEBUG_PIN5.setValue(value & 0x08);
        DEBUG_PIN6.setValue(value & 0x04);
        DEBUG_PIN7.setValue(value & 0x02);
        DEBUG_PIN8.setValue(value & 0x01);
}

#endif
