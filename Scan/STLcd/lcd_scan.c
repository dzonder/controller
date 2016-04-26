/* Copyright (C) 2015-2016 by Jacob Alexander
 *
 * This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this file.  If not, see <http://www.gnu.org/licenses/>.
 */

// ----- Includes -----

// Compiler Includes
#include <Lib/ScanLib.h>

// Project Includes
#include <cli.h>
#include <kll_defs.h>
#include <led.h>
#include <print.h>

// Interconnect module if compiled in
#if defined(ConnectEnabled_define)
#include <connect_scan.h>
#endif

// Local Includes
#include "lcd_scan.h"
#include "font.h"



// ----- Defines -----

#define LCD_TOTAL_VISIBLE_PAGES 4
#define LCD_TOTAL_PAGES 9
#define LCD_PAGE_LEN 128
#define LCD_MAX_MARKERS 4
#define LCD_BUF_SIZE LCD_TOTAL_VISIBLE_PAGES * LCD_PAGE_LEN
#define LCD_TEXT_SIZE 8



// ----- Macros -----

// Number of entries in the SPI0 TxFIFO
#define SPI0_TxFIFO_CNT ( ( SPI0_SR & SPI_SR_TXCTR ) >> 12 )



// ----- Structs -----

typedef struct LCD_stack {
	uint8_t layersCount;  // How many layers should be displayed
	uint16_t layers[4];
} LCD_stack;

typedef struct LCD_state {
	bool markerOn[LCD_MAX_MARKERS];
	LCD_stack stack;
	unsigned char text1[LCD_TEXT_SIZE];
	unsigned char text2[LCD_TEXT_SIZE];
} LCD_state;

// ----- Function Declarations -----

void LCD_redrawDisplay( LCD_state *newState );

// CLI Functions
void cliFunc_lcdCmd          ( char* args );
void cliFunc_lcdColor        ( char* args );
void cliFunc_lcdDisp         ( char* args );
void cliFunc_lcdInit         ( char* args );
void cliFunc_lcdText         ( char* args );
void cliFunc_lcdMarkerToggle ( char* args );



// ----- Variables -----

// Current state of the LCD display, use to properly redraw the display.
LCD_state currentState;

// Full Toggle State
uint8_t cliFullToggleState = 0;

// Normal/Reverse Toggle State
uint8_t cliNormalReverseToggleState = 0;

// Scan Module command dictionary
CLIDict_Entry( lcdCmd,          "Send byte via SPI, second argument enables a0. Defaults to control." );
CLIDict_Entry( lcdColor,        "Set backlight color. 3 16-bit numbers: R G B. i.e. 0xFFF 0x1444 0x32" );
CLIDict_Entry( lcdDisp,         "Write byte(s) to given page starting at given address. i.e. 0x1 0x5 0xFF 0x00" );
CLIDict_Entry( lcdInit,         "Re-initialize the LCD display." );
CLIDict_Entry( lcdText,         "Set the text on the LCD, two rows are supported, each 8 characters." );
CLIDict_Entry( lcdMarkerToggle, "Toggle the marker (by number to status)." );

CLIDict_Def( lcdCLIDict, "ST LCD Module Commands" ) = {
	CLIDict_Item( lcdCmd ),
	CLIDict_Item( lcdColor ),
	CLIDict_Item( lcdDisp ),
	CLIDict_Item( lcdInit ),
	CLIDict_Item( lcdText ),
	CLIDict_Item( lcdMarkerToggle ),
	{ 0, 0, 0 } // Null entry for dictionary end
};



// ----- Interrupt Functions -----



// ----- Functions -----

inline void SPI_setup()
{
	// Enable SPI internal clock
	SIM_SCGC6 |= SIM_SCGC6_SPI0;

	// Setup MOSI (SOUT) and SCLK (SCK)
	PORTC_PCR6 = PORT_PCR_DSE | PORT_PCR_MUX(2);
	PORTC_PCR5 = PORT_PCR_DSE | PORT_PCR_MUX(2);

	// Setup SS (PCS)
	PORTC_PCR4 = PORT_PCR_DSE | PORT_PCR_MUX(2);

	// Master Mode, CS0
	SPI0_MCR = SPI_MCR_MSTR | SPI_MCR_PCSIS(1);

	// DSPI Clock and Transfer Attributes
	// Frame Size: 8 bits
	// MSB First
	// CLK Low by default
	SPI0_CTAR0 = SPI_CTAR_FMSZ(7)
		| SPI_CTAR_ASC(7)
		| SPI_CTAR_DT(7)
		| SPI_CTAR_CSSCK(7)
		| SPI_CTAR_PBR(0) | SPI_CTAR_BR(7);
}

// Write buffer to SPI FIFO
void SPI_write( uint8_t *buffer, uint8_t len )
{

	for ( uint8_t byte = 0; byte < len; byte++ )
	{
		// Wait for SPI TxFIFO to have 4 or fewer entries
		while ( !( SPI0_SR & SPI_SR_TFFF ) )
			delayMicroseconds(10);

		// Write byte to TxFIFO
		// CS0, CTAR0
		SPI0_PUSHR = ( buffer[ byte ] & 0xff ) | SPI_PUSHR_PCS(1);

		// Indicate transfer has completed
		while ( !( SPI0_SR & SPI_SR_TCF ) );
		SPI0_SR |= SPI_SR_TCF;
	}
}

// Write to a control register
void LCD_writeControlReg( uint8_t byte )
{
	// Wait for TxFIFO to be empt
	while ( SPI0_TxFIFO_CNT != 0 );

	// Set A0 low to enter control register mode
	GPIOC_PCOR |= (1<<7);

	// Write byte to SPI FIFO
	SPI_write( &byte, 1 );

	// Wait for TxFIFO to be empty
	while ( SPI0_TxFIFO_CNT != 0 );

	// Make sure data has transferred
	delayMicroseconds(10); // XXX Adjust if SPI speed changes

	// Set A0 high to go back to display register mode
	GPIOC_PSOR |= (1<<7);
}

// Write to display register
// Pages 0-7 normal display
// Page  8   icon buffer
void LCD_writeDisplayReg( uint8_t page, uint8_t *buffer, uint8_t len )
{
	// Set the register page
	LCD_writeControlReg( 0xB0 | ( 0x0F & page ) );

	// Set display start line
	LCD_writeControlReg( 0x40 );

	// Reset Column Address
	LCD_writeControlReg( 0x10 );
	LCD_writeControlReg( 0x00 );

	// Write buffer to SPI
	SPI_write( buffer, len );
}

inline void LCD_clearPage( uint8_t page )
{
	// Set the register page
	LCD_writeControlReg( 0xB0 | ( 0x0F & page ) );

	// Set display start line
	LCD_writeControlReg( 0x40 );

	// Reset Column Address
	LCD_writeControlReg( 0x10 );
	LCD_writeControlReg( 0x00 );

	for ( uint8_t page_reg = 0; page_reg < LCD_PAGE_LEN; page_reg++ )
	{
		uint8_t byte = 0;

		// Write buffer to SPI
		SPI_write( &byte, 1 );
	}

	// Wait for TxFIFO to be empty
	while ( SPI0_TxFIFO_CNT != 0 );
}

// Clear Display
void LCD_clear()
{
	// Setup each page
	for ( uint8_t page = 0; page < LCD_TOTAL_PAGES; page++ )
	{
		LCD_clearPage( page );
	}

	// Reset Page, Start Line, and Column Address
	// Page
	LCD_writeControlReg( 0xB0 );

	// Start Line
	LCD_writeControlReg( 0x40 );

	// Reset Column Address
	LCD_writeControlReg( 0x10 );
	LCD_writeControlReg( 0x00 );
}

// Intialize display
void LCD_initialize()
{
	// ADC Select (Normal)
	LCD_writeControlReg( 0xA0 );

	// LCD Off
	LCD_writeControlReg( 0xAE );

	// COM Scan Output Direction
	LCD_writeControlReg( 0xC0 );

	// LCD Bias (1/6 bias)
	LCD_writeControlReg( 0xA2 );

	// Power Supply Operating Mode (Internal Only)
	LCD_writeControlReg( 0x2F );

	// Internal Rb/Ra Ratio
	LCD_writeControlReg( 0x26 );

	// Reset
	LCD_writeControlReg( 0xE2 );

	// Electric volume mode set, and value
	LCD_writeControlReg( 0x81 );
	LCD_writeControlReg( 0x00 );

	// LCD On
	LCD_writeControlReg( 0xAF );

	// Clear Display RAM
	LCD_clear();
}

// Setup
inline void LCD_setup()
{
	// Register Scan CLI dictionary
	CLI_registerDictionary( lcdCLIDict, lcdCLIDictName );

	// Initialize SPI
	SPI_setup();

	// Setup Register Control Signal (A0)
	// Start in display register mode (1)
	GPIOC_PDDR |= (1<<7);
	PORTC_PCR7 = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
	GPIOC_PSOR |= (1<<7);

	// Setup LCD Reset pin (RST)
	// 0 - Reset, 1 - Normal Operation
	// Start in normal mode (1)
	GPIOC_PDDR |= (1<<8);
	PORTC_PCR8 = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
	GPIOC_PSOR |= (1<<8);

	// Run LCD intialization sequence
	LCD_initialize();

	// Redraw display
	memcpy( &currentState.text1, "        ", LCD_TEXT_SIZE );
	memcpy( &currentState.text2, "        ", LCD_TEXT_SIZE );
	LCD_redrawDisplay( &currentState );

	// Setup Backlight
	SIM_SCGC6 |= SIM_SCGC6_FTM0;
	FTM0_CNT = 0; // Reset counter

	// PWM Period
	// 16-bit maximum
	FTM0_MOD = 0xFFFF;

	// Set FTM to PWM output - Edge Aligned, Low-true pulses
	FTM0_C0SC = 0x24; // MSnB:MSnA = 10, ELSnB:ELSnA = 01
	FTM0_C1SC = 0x24;
	FTM0_C2SC = 0x24;

	// Base FTM clock selection (72 MHz system clock)
	// @ 0xFFFF period, 72 MHz / (0xFFFF * 2) = Actual period
	// Higher pre-scalar will use the most power (also look the best)
	// Pre-scalar calculations
	// 0 -      72 MHz -> 549 Hz
	// 1 -      36 MHz -> 275 Hz
	// 2 -      18 MHz -> 137 Hz
	// 3 -       9 MHz ->  69 Hz (Slightly visible flicker)
	// 4 -   4 500 kHz ->  34 Hz (Visible flickering)
	// 5 -   2 250 kHz ->  17 Hz
	// 6 -   1 125 kHz ->   9 Hz
	// 7 - 562 500  Hz ->   4 Hz
	// Using a higher pre-scalar without flicker is possible but FTM0_MOD will need to be reduced
	// Which will reduce the brightness range

	// System clock, /w prescalar setting
	FTM0_SC = FTM_SC_CLKS(1) | FTM_SC_PS( STLcdBacklightPrescalar_define );

	// Red
	FTM0_C0V = STLcdBacklightRed_define;
	PORTC_PCR1 = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(4);

	// Green
	FTM0_C1V = STLcdBacklightGreen_define;
	PORTC_PCR2 = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(4);

	// Blue
	FTM0_C2V = STLcdBacklightBlue_define;
	PORTC_PCR3 = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(4);
}


// LCD State processing loop
inline uint8_t LCD_scan()
{
	return 0;
}


// Signal from parent Scan Module that available current has changed
// current - mA
void LCD_currentChange( unsigned int current )
{
	// TODO - Power savings?
}

void LCD_drawText( uint8_t *spot, const unsigned char *text, const uint8_t len )
{
	for ( uint8_t i = 0; i < len; i++ )
	{
		memcpy(
			spot + sizeof( font[0] ) * i,
			&font[ text[i] - ' ' ],
			sizeof( font[0] )
		);
	}
}

// Draw buffer
uint8_t buffer[ LCD_BUF_SIZE ];
void LCD_redrawDisplay( LCD_state *newState )
{
	currentState = *newState;

	// Number data for LCD
	const uint8_t numbers[10][128] = {
		{ STLcdNumber0_define },
		{ STLcdNumber1_define },
		{ STLcdNumber2_define },
		{ STLcdNumber3_define },
		{ STLcdNumber4_define },
		{ STLcdNumber5_define },
		{ STLcdNumber6_define },
		{ STLcdNumber7_define },
		{ STLcdNumber8_define },
		{ STLcdNumber9_define },
	};

	// Color data for numbers
	const uint16_t colors[10][3] = {
		{ STLcdNumber0Color_define },
		{ STLcdNumber1Color_define },
		{ STLcdNumber2Color_define },
		{ STLcdNumber3Color_define },
		{ STLcdNumber4Color_define },
		{ STLcdNumber5Color_define },
		{ STLcdNumber6Color_define },
		{ STLcdNumber7Color_define },
		{ STLcdNumber8Color_define },
		{ STLcdNumber9Color_define },
	};

	const uint8_t markers[2][7] = {
		{ 0xFE, 0x82, 0x82, 0x82, 0x82, 0x82, 0xFE },
		{ 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE },
	};

	bool any_marker_on = false;
	for ( uint8_t i = 0; i < LCD_MAX_MARKERS; i++ )
		any_marker_on |= currentState.markerOn[ i ] != 0;

	memset( buffer, 0, LCD_BUF_SIZE );

	if ( !any_marker_on && currentState.stack.layersCount == 0 )
	{
		// Set default backlight
		FTM0_C0V = STLcdBacklightRed_define;
		FTM0_C1V = STLcdBacklightGreen_define;
		FTM0_C2V = STLcdBacklightBlue_define;
	}
	else if ( any_marker_on && currentState.stack.layersCount == 0 )
	{
		// Set background for markers only
		FTM0_C0V = STLcdMarkerBacklightRed_define;
		FTM0_C1V = STLcdMarkerBacklightGreen_define;
		FTM0_C2V = STLcdMarkerBacklightBlue_define;
	}
	else
	{
		// Set the color according to the "top-of-stack" layer
		uint16_t layerIndex = currentState.stack.layers[0];
		FTM0_C0V = colors[ layerIndex ][0];
		FTM0_C1V = colors[ layerIndex ][1];
		FTM0_C2V = colors[ layerIndex ][2];
	}

	// Dump 1st line of text
	{
		uint8_t pos = 128 - LCD_TEXT_SIZE * sizeof( font[0] );
		uint8_t *spot1 = &buffer[ 3 * 128 + pos ];
		LCD_drawText( spot1, currentState.text1, LCD_TEXT_SIZE );
		uint8_t *spot2 = &buffer[ 2 * 128 + pos ];
		LCD_drawText( spot2, currentState.text2, LCD_TEXT_SIZE );
	}

	// First dump marker information
	if ( any_marker_on )
	{
		uint8_t page = 0;
		for ( uint8_t marker = 0; marker < LCD_MAX_MARKERS; marker++ )
		{
			uint8_t pos = (LCD_MAX_MARKERS - marker - 1) * 8;
			uint8_t *spot = &buffer[ page * 128 + 96 + pos ];
			memcpy(
				spot,
				&markers[ currentState.markerOn[ marker ] ],
				sizeof( markers[0] )
			);
		}
	}

	// Now dump layer information
	for ( uint8_t page = 0; page < LCD_TOTAL_VISIBLE_PAGES; page++ )
	{
		uint8_t *spot = &buffer[ page * 128 ];

		// Write data
		for ( uint16_t layer = 0; layer < currentState.stack.layersCount; layer++ )
		{
			uint16_t layerIndex = currentState.stack.layers[ layer ];

			// Default to 0, if over 9
			if ( layerIndex > 9 )
				layerIndex = 0;

			memcpy(
				spot + 32 * layer,
				&numbers[ layerIndex ][ page * 32 ],
				32 );
		}
	}

	// Write buffer to the display
	for ( uint8_t page = 0; page < LCD_TOTAL_VISIBLE_PAGES; page++ )
		LCD_writeDisplayReg(
			page, 
			&buffer[page * LCD_PAGE_LEN],
			LCD_PAGE_LEN
		);
}


// ----- Capabilities -----

// A simple state markers, e.g. can be used as CapsLock indicator.
typedef struct LCD_marker_args {
	uint8_t num;  // [0, 3]
	uint8_t status;  // Values - 0: off, 1: on, 2: toggle
} LCD_marker_args;
void LCD_markerToggle_capability( uint8_t state, uint8_t stateType, uint8_t *args )
{
	if ( stateType == 0xFF && state == 0xFF )
	{
		print("LCD_markerToggle_capability(num,status)");
	}

	// Read arguments
	LCD_marker_args marker_args = *(LCD_marker_args*)args;

	LCD_state newState = currentState;
	bool *markerOn = &newState.markerOn[marker_args.num];

	// Handle the toggle
	if ( marker_args.status == 2 )
	{
		if ( stateType != 0 || state != 0x03 )
		{
			return;
		}
		marker_args.status = !*markerOn;
	}

	// Set status of the marker
	*markerOn = !!marker_args.status;

	if ( memcmp( &newState, &currentState, sizeof( currentState ) ) == 0 )
		return;

	// Only deal with the interconnect if it has been compiled in
#if defined(ConnectEnabled_define)
	if ( Connect_master )
	{
		// generatedKeymap.h
		extern const Capability CapabilitiesList[];

		// Broadcast layerStackExact remote capability (0xFF is the broadcast id)
		Connect_send_RemoteCapability(
			0xFF,
			LCD_markerToggle_capability_index,
			state,
			stateType,
			CapabilitiesList[ LCD_markerToggle_capability_index ].argCount,
			(uint8_t*)&marker_args
		);
	}
#endif

	LCD_redrawDisplay(&newState);
}

// Takes 1 8 bit length and 4 16 bit arguments, each corresponding to a layer index
// Ordered from top to bottom
// The first argument indicates how many numbers to display (max 4), set to 0 to load default image
void LCD_layerStackExact_capability( uint8_t state, uint8_t stateType, uint8_t *args )
{
	// Display capability name
	if ( stateType == 0xFF && state == 0xFF )
	{
		print("LCD_layerStackExact_capability(num,layer1,layer2,layer3,layer4)");
		return;
	}

	LCD_stack *newStack = (LCD_stack*)args;

	LCD_state newState = currentState;
	newState.stack = *newStack;

	LCD_redrawDisplay(&newState);
}

// Determines the current layer stack, and sets the LCD output accordingly
void LCD_layerStack_capability( uint8_t state, uint8_t stateType, uint8_t *args )
{
	// Display capability name
	if ( stateType == 0xFF && state == 0xFF )
	{
		print("LCD_layerStack_capability()");
		return;
	}

	LCD_stack newStack = currentState.stack;

	// Parse the layer stack, top to bottom
	extern uint16_t macroLayerIndexStack[];
	extern uint16_t macroLayerIndexStackSize;

	memset( &newStack, 0, sizeof( newStack ) );

	newStack.layersCount = macroLayerIndexStackSize;
	for ( uint16_t layer = 1; layer <= newStack.layersCount; layer++ )
	{
		newStack.layers[ layer - 1 ] =
			macroLayerIndexStack[ macroLayerIndexStackSize - layer ];
	}

	if ( memcmp( &newStack, &currentState.stack, sizeof( currentState.stack ) ) == 0 )
		return;

	// Only deal with the interconnect if it has been compiled in
#if defined(ConnectEnabled_define)
	if ( Connect_master )
	{
		// generatedKeymap.h
		extern const Capability CapabilitiesList[];

		// Broadcast layerStackExact remote capability (0xFF is the broadcast id)
		Connect_send_RemoteCapability(
			0xFF,
			LCD_layerStackExact_capability_index,
			state,
			stateType,
			CapabilitiesList[ LCD_layerStackExact_capability_index ].argCount,
			(uint8_t*)&newStack
		);
	}
#endif

	LCD_layerStackExact_capability( state, stateType, (uint8_t*)&newStack );
}



// ----- CLI Command Functions -----

void cliFunc_lcdInit( char* args )
{
	LCD_initialize();
}

void cliFunc_lcdCmd( char* args )
{
	char* curArgs;
	char* arg1Ptr;
	char* arg2Ptr = args;

	print( NL ); // No \r\n by default after the command is entered

	curArgs = arg2Ptr; // Use the previous 2nd arg pointer to separate the next arg from the list
	CLI_argumentIsolation( curArgs, &arg1Ptr, &arg2Ptr );

	// No args
	if ( *arg1Ptr == '\0' )
		return;

	// SPI Command
	uint8_t cmd = (uint8_t)numToInt( arg1Ptr );

	curArgs = arg2Ptr; // Use the previous 2nd arg pointer to separate the next arg from the list
	CLI_argumentIsolation( curArgs, &arg1Ptr, &arg2Ptr );

	// Single Arg
	if ( *arg1Ptr == '\0' )
		goto cmd;

	// TODO Deal with a0
cmd:
	info_msg("Sending - ");
	printHex( cmd );
	print( NL );
	LCD_writeControlReg( cmd );
}

void cliFunc_lcdColor( char* args )
{
	char* curArgs;
	char* arg1Ptr;
	char* arg2Ptr = args;

	// Colors
	uint16_t rgb[3]; // Red, Green, Blue

	// Parse integers from 3 arguments
	for ( uint8_t color = 0; color < 3; color++ )
	{
		curArgs = arg2Ptr;
		CLI_argumentIsolation( curArgs, &arg1Ptr, &arg2Ptr );

		// Give up if not enough args given
		if ( *arg1Ptr == '\0' )
			return;

		// Convert argument to integer
		rgb[ color ] = numToInt( arg1Ptr );
	}

	// Set PWM channels
	FTM0_C0V = rgb[0];
	FTM0_C1V = rgb[1];
	FTM0_C2V = rgb[2];
}

void cliFunc_lcdDisp( char* args )
{
	char* curArgs;
	char* arg1Ptr;
	char* arg2Ptr = args;

	// First process page and starting address
	curArgs = arg2Ptr;
	CLI_argumentIsolation( curArgs, &arg1Ptr, &arg2Ptr );

	// Stop processing args if no more are found
	if ( *arg1Ptr == '\0' )
		return;
	uint8_t page = numToInt( arg1Ptr );

	curArgs = arg2Ptr;
	CLI_argumentIsolation( curArgs, &arg1Ptr, &arg2Ptr );

	// Stop processing args if no more are found
	if ( *arg1Ptr == '\0' )
		return;
	uint8_t address = numToInt( arg1Ptr );

	// Set the register page
	LCD_writeControlReg( 0xB0 | ( 0x0F & page ) );

	// Set starting address
	LCD_writeControlReg( 0x10 | ( ( 0xF0 & address ) >> 4 ) );
	LCD_writeControlReg( 0x00 | ( 0x0F & address ));

	// Process all args
	for ( ;; )
	{
		curArgs = arg2Ptr;
		CLI_argumentIsolation( curArgs, &arg1Ptr, &arg2Ptr );

		// Stop processing args if no more are found
		if ( *arg1Ptr == '\0' )
			break;

		uint8_t value = numToInt( arg1Ptr );

		// Write buffer to SPI
		SPI_write( &value, 1 );
	}
}

void cliFunc_lcdText( char* args )
{
	LCD_state newState = currentState;

	memcpy( newState.text1, args, LCD_TEXT_SIZE );
	memcpy( newState.text2, args + LCD_TEXT_SIZE, LCD_TEXT_SIZE );

	LCD_redrawDisplay(&newState);
}

void cliFunc_lcdMarkerToggle( char* args )
{
	LCD_marker_args markerArgs;
	char* curArgs;
	char* arg1Ptr;
	char* arg2Ptr = args;

	// Process num and status arguments
	curArgs = arg2Ptr;
	CLI_argumentIsolation( curArgs, &arg1Ptr, &arg2Ptr );

	// Stop processing args if no more are found
	if ( *arg1Ptr == '\0' )
		return;
	markerArgs.num = numToInt( arg1Ptr );

	curArgs = arg2Ptr;
	CLI_argumentIsolation( curArgs, &arg1Ptr, &arg2Ptr );

	// Stop processing args if no more are found
	if ( *arg1Ptr == '\0' )
		return;
	markerArgs.status = numToInt( arg1Ptr );

	LCD_markerToggle_capability( 0x03, 0, (uint8_t*)&markerArgs );
}
