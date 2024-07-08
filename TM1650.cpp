#include "TM1650.h"

/** Constructor, uses default values for the parameters
 * so could be called with no parameters.
 * aNumDigits - number of display digits (default = 4)
 */
TM1650::TM1650(TwoWire *wr, unsigned int aNumDigits) {
	wire = wr;
	iNumDigits =  (aNumDigits > TM1650_NUM_DIGITS) ? TM1650_NUM_DIGITS : aNumDigits;
}

/** Initialization
 * initializes the driver. Turns display on, but clears all digits.
 */
void TM1650::init() {
	iPosition = NULL;
	for (unsigned int i=0; i<iNumDigits; i++) {
		iBuffer[i] = 0;
		iCtrl[i] = 0;
	}
	wire->beginTransmission(TM1650_DISPLAY_BASE);
	iActive = (wire->endTransmission() == 0);
	clear();
	displayOn();
}

/** Set brightness of all digits equally
 * aValue - brightness value with 1 being the lowest, and 7 being the brightest
 */
void TM1650::setBrightness(unsigned int aValue) {
	if (!iActive) return;

	iBrightness = (aValue > TM1650_MAX_BRIGHT) ? TM1650_MAX_BRIGHT : aValue;

	for (unsigned int i=0; i<iNumDigits; i++) {
		wire->beginTransmission(TM1650_DCTRL_BASE+(int)i);
		iCtrl[i] = (iCtrl[i] & TM1650_MSK_BRIGHT) | ( iBrightness << TM1650_BRIGHT_SHIFT );
		wire->write((byte) iCtrl[i]);
		wire->endTransmission();
	}
}

/** Set brightness of all digits equally
 * aValue - brightness value with 1 being the lowest, and 7 being the brightest
 */
void TM1650::setBrightnessGradually(unsigned int aValue) {
	if (!iActive || aValue == iBrightness) return;
	if (aValue > TM1650_MAX_BRIGHT) aValue = TM1650_MAX_BRIGHT;
	int step = (aValue < iBrightness) ? -1 : 1;
	unsigned int i = iBrightness;
	do {
		setBrightness(i);
		delay(50);
		i += step;
	} while (i!=aValue);
}

/** Turns display on or off according to aState
 */
void TM1650::displayState(bool aState) {
	if (aState) displayOn ();
	else displayOff();
}

/** Turns the display on
 */
void TM1650::displayOn() {
	if (!iActive) return;
	for (unsigned int i=0; i<iNumDigits; i++) {
		wire->beginTransmission(TM1650_DCTRL_BASE+(int)i);
		iCtrl[i] = (iCtrl[i] & TM1650_MSK_ONOFF) | TM1650_BIT_DOT;
		wire->write((byte) iCtrl[i]);
		wire->endTransmission();
	}
}

/** Turns the display off
 */
void TM1650::displayOff() {
	if (!iActive) return;
	for (unsigned int i=0; i<iNumDigits; i++) {
		wire->beginTransmission(TM1650_DCTRL_BASE+(int)i);
		iCtrl[i] = (iCtrl[i] & TM1650_MSK_ONOFF);
		wire->write((byte) iCtrl[i]);
		wire->endTransmission();
	}
}

/** Directly write to the CONTROL register of the digital position
 * aPos = position to set the control register for
 * aValue = value to write to the position
 *
 * Internal control buffer is updated as well
 */
void TM1650::controlPosition(unsigned int aPos, byte aValue) {
	if (!iActive) return;
	if (aPos < iNumDigits) {
		wire->beginTransmission(TM1650_DCTRL_BASE + (int)aPos);
		iCtrl[aPos] = aValue;
		wire->write(aValue);
		wire->endTransmission();
	}
}

/** Directly write to the digit register of the digital position
 * aPos = position to set the digit register for
 * aValue = value to write to the position
 *
 * Internal position buffer is updated as well
 */
void TM1650::setPosition(unsigned int aPos, byte aValue) {
	if (!iActive) return;
	if (aPos < iNumDigits) {
		wire->beginTransmission(TM1650_DISPLAY_BASE + (int)aPos);
		iBuffer[aPos] = aValue;
		wire->write(aValue);
		wire->endTransmission();
	}
}

/** Directly set/clear a 'dot' next to a specific position
 * aPos = position to set/clear the dot for
 * aState = display the dot if true, clear if false
 *
 * Internal buffer is updated as well
 */
void	TM1650::setDot(unsigned int aPos, bool aState) {
	iBuffer[aPos] = (iBuffer[aPos] & 0x7F) | (aState ? TM1650_DOT : 0);
	setPosition(aPos, iBuffer[aPos]);
}

/** Clear all digits. Keep the display on.
 */
void TM1650::clear() {
	if (!iActive) return;
	for (unsigned int i=0; i<iNumDigits; i++) {
		wire->beginTransmission(TM1650_DISPLAY_BASE+(int)i);
		iBuffer[i] = 0;
		wire->write((byte) 0);
		wire->endTransmission();
	}
}

/** Display string on the display
 * aString = character array to be displayed
 *
 * Internal buffer is updated as well
 * Only first N positions of the string are displayed if
 *  the string is longer than the number of digits
 */
void TM1650::displayString(char *aString) {
	if (!iActive) return;
	for (unsigned int i=0; i<iNumDigits; i++) {
		byte a = ((byte) aString[i]) & ~TM1650_DOT;
		byte dot = ((byte) aString[i]) & TM1650_DOT;
#ifndef TM1650_USE_PROGMEM
		iBuffer[i] = TM1650_CDigits[a];
#else
		iBuffer[i] = pgm_read_byte_near(TM1650_CDigits + a);
#endif
		if (a) {
			wire->beginTransmission(TM1650_DISPLAY_BASE+(int)i);
			wire->write(iBuffer[i] | dot);
			wire->endTransmission();
		}
		else break;
	}
}

/** Display string on the display in a running fashion
 * aString = character array to be displayed
 *
 * Starts with first N positions of the string.
 * Subsequent characters are displayed with 1 char shift each time displayRunningShift() is called
 *
 * returns: number of iterations remaining to display the whole string
 */
int TM1650::displayRunning(char *aString) {
	strncpy(iString, aString, TM1650_MAX_STRING+1);
	iPosition = iString;
	iString[TM1650_MAX_STRING] = '\0'; //just in case.
	displayString(iPosition);
	unsigned int l = strlen(iPosition);
	if (l <= iNumDigits) return 0;
	return (l - iNumDigits);
}

/** Display next segment (shifting to the left) of the string set by displayRunning()
 * Starts with first N positions of the string.
 * Subsequent characters are displayed with 1 char shift each time displayRunningShift is called
 *
 * returns: number of iterations remaining to display the whole string
 */
int TM1650::displayRunningShift() {
	if (strlen(iPosition) <= iNumDigits) return 0;
	displayString(++iPosition);
	return (strlen(iPosition) - iNumDigits);
}

/** Display character on the display
 * aPos = the position of the display character
 * aData = character to be displayed
 * aDot = display the dot if true, clear if false
 *
 * Display of the character at the specified position of the display.
 */
void TM1650::displayChar(byte aPos, byte aData, bool aDot) {
	if (!iActive) return;
	byte a = aData & ~TM1650_DOT;
#ifndef TM1650_USE_PROGMEM
	aData = TM1650_CDigits[a];
#else
	aData = pgm_read_byte_near(TM1650_CDigits + a);
#endif
	if (aDot) aData |= TM1650_DOT;
	iBuffer[aPos] = aData;
	wire->beginTransmission(TM1650_DISPLAY_BASE + aPos);
	wire->write(aData);
	wire->endTransmission();
}

/** Getting the buttons pressed
 *
 * returns: the code of the pressed buttons, for details refer to the documentation on the chip TM1650
 */
byte TM1650::getButtons(void) {
	byte keys = 0;
	wire->requestFrom(TM1650_DCTRL_BASE, 2);
	keys = wire->read();
	// wire->read();
	return keys;
}