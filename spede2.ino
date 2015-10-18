/*
	Speden Speli SpeedTester
	using ATmega328P

	Note: fuses must be set with avrdude to use external 16Mhz clock with prescaler 1/8 to obtain 2Mhz system clock
	TODO: document the avrdude commandline here!

	Pins
	====

	Final i/o pins

	D0	seg G (LOW = segment on, INPUT = segment off)
	D1	seg F
	D2	seg A
	D3	not connected
	D4	seg B
	D5	button 1 (LOW = button pressed)
	D6	button 2
	D7	button 3
	D8	button 4
	D9	led 4 (HIGH = led on, LOW = led off)
	D10	led 3
	D11	speaker (PWM)
	D12	led 2
	D13 led 1
	A0	seg E
	A1  seg D
	A2  seg C
	A3  digit 1 (HIGH = digit on, LOW = digit off)
	A4	digit 3
	A5	digit 2
*/


/*
	Display flickering issue
	========================

	Display flickering is caused by tone library, here is what happens:
	- tone library is implemented using a hardware interrupt occurring at desired tone frequency
	- for a MCU running at 2Mhz, the interrupt consumes quite a lot of MCU time
	- e.g. tone of frequency 415hz needs an interrupt running at 830hz, interrupt is called every 2000000/830 = 2400 cycles
	- varying frequencies cause variations to led display update frequency
	- this can be seen as flickering / shimmering
	- to fix this, we need to reimplement tone playback so that it does not use interrupts and toggles the tone pin using hardware timers
	- see this document for details: http://arduino.cc/en/Tutorial/SecretsOfArduinoPWM
		- especially the section "Fast PWM Mode with OCRA top"
*/

#include <EEPROM.h>

#define CLOCK_SCALER	8		// Atmega running at 2Mhz

const byte digitPins[] = { A3, A5, A4 };
const byte segmentPins[] = { 2, 4, A2, A1, A0, 1, 0 };
const byte buttonPins[] = { 5, 6, 7, 8 };
const byte ledPins[] = { 13, 12, 10, 9 };

// Segment bits for numbers 0-9
const byte segments[10] = {
  B0111111, // 0  ABCDEF-
  B0000110, // 1  -BC----
  B1011011, // 2  AB-DE-G
  B1001111, // 3  ABCD--G  
  B1100110, // 4  -BC--FG
  B1101101, // 5  A-CD-FG
  B1111101, // 6  A-BCDEF
  B0000111, // 7  ABC----
  B1111111, // 8  ABCDEFG
  B1101111, // 9  ABCD-FG
};

enum
{
	STATE_START_MENU,
	STATE_SPEED_GAME,
	STATE_MEMORY_GAME,
	STATE_GAME_OVER
};

int score = 0;
int8_t led = 0;
int8_t prevLed = 0;
int nextTimer = 0;
int level = 0;
int hiscore[2] = { 0, 0 };
int startMenuTimer = 0;
int prevButtonState[] = { HIGH, HIGH, HIGH, HIGH };
uint8_t state = STATE_START_MENU;
long lastButtonPress = 0;

// current game mode for showing hiscore
uint8_t currentGameMode = 0; // 0 = speed game, 1 = memory game

int memSeed = 0; // random seed for memory game

uint8_t attractModeTimer = 0;
uint8_t powerSave = 0;			// if set, display update frequency is throttled

//=====================================
// tone playback using hardware timers
//=====================================

// Frequencies of tones played when buttons are pressed (frequency in hz = 2000000/32/freq/2)
const uint8_t toneFreq[] = { 56, 50, 42, 38 }; // 558hz (~CS4), 626hz (~DS4), 744hz (~FS4), 822hz (~GS4)
const int tonePin = 11;

void mytone(uint8_t freq) {
  pinMode(tonePin, OUTPUT);

  // configure timer 2 to phase-correct PWM mode
  // in this mode the timer counts from zero to compare value in OCR2A and back to zero
  // when timer reaches OCR2A, timer output A is toggled (output A = pin 11)
  // therefore the frequency of the generated square wave is 2000000/32/OCR2A/2
  TCCR2A = _BV(COM2A0) | _BV(COM2B1) | _BV(WGM20);

  // prescaler: _BV(CS22)  = 64
  // prescaler: _BV(CS21)|_BV(CS20)  = 32
  // prescaler: _BV(CS21)  = 8
  TCCR2B = _BV(WGM22) | _BV(CS21) | _BV(CS20);		// clock prescaler 1/32

  OCR2A = freq;
  OCR2B = freq;
}

void toneoff() {
	pinMode(tonePin, INPUT);
}

///////

inline void pinModeF(uint8_t pin, uint8_t mode)
{
	uint8_t bit = digitalPinToBitMask(pin);
	uint8_t port = digitalPinToPort(pin);
	volatile uint8_t *reg, *out;

	// JWS: can I let the optimizer do this?
	reg = portModeRegister(port);
	out = portOutputRegister(port);

	if (mode == INPUT) { 
		*reg &= ~bit;
		*out &= ~bit;
	} else if (mode == INPUT_PULLUP) {
		*reg &= ~bit;
		*out |= bit;
	} else {
		*reg |= bit;
	}
}

inline void digitalWriteF(uint8_t pin, uint8_t val)
{
	uint8_t bit = digitalPinToBitMask(pin);
	uint8_t port = digitalPinToPort(pin);
	volatile uint8_t *out = portOutputRegister(port);
	if (val == LOW) {
		*out &= ~bit;
	} else {
		*out |= bit;
	}
}

inline int digitalReadF(uint8_t pin)
{
	uint8_t bit = digitalPinToBitMask(pin);
	uint8_t port = digitalPinToPort(pin);
	if (*portInputRegister(port) & bit) return HIGH;
	return LOW;
}

///////

class Random
{
public:
	Random() {}
 
	void init(int seed)
	{
		m_state = seed;
		m_prevValue = -1;
	}

	int randomTweaked(int max)
	{
		// make row of same values less probable
		int v = random(max);
		if(v == m_prevValue && random(10) < 6)
			v = random(4);
		m_prevValue = v;
		return v;
	}

	int random(int max)
	{
		m_state = m_state * 214013L + 2531011L;
		int s = (m_state >> 16) & 0x7fff;
		return s % max;
	}

private:
	unsigned long	m_state;
	int				m_prevValue;
};

// Read hiscore value from EEPROM
void readHiscore()
{
	hiscore[0] = (EEPROM.read(0) << 8) + EEPROM.read(1);
	hiscore[1] = (EEPROM.read(2) << 8) + EEPROM.read(3);

	// EEPROM initial value is FFFF
	if(hiscore[0] == 0xffff)
		hiscore[0] = 0;
	if(hiscore[1] == 0xffff)
		hiscore[1] = 0;
}

// Write hiscore value to EEPROM
void writeHiscore()
{
	EEPROM.write(0, hiscore[0] >> 8);
	EEPROM.write(1, hiscore[0] & 0xff);
	EEPROM.write(2, hiscore[1] >> 8);
	EEPROM.write(3, hiscore[1] & 0xff);
}

void setup()
{
	for(byte i = 0; i <= 6; i++)
		pinMode(segmentPins[i], INPUT);

	for(byte i = 0; i < 3; i++)
	{
		pinMode(digitPins[i], OUTPUT);
		digitalWriteF(digitPins[i], LOW);
	}

	for(byte i = 0; i < 4; i++)
	{
		pinMode(buttonPins[i], INPUT_PULLUP);
		pinMode(ledPins[i], OUTPUT);
		digitalWriteF(ledPins[i], LOW);
	}

	// Power consumption:
	// @16Mhz 14.5mA (MCU runnning, no sound, no leds)
	// @ 2Mhz  5.0mA (MCU runnning, no sound, no leds)
	// @ 1Mhz  4.5mA (MCU runnning, no sound, no leds)

	// power saving options
	ADCSRA = 0;  // disable ADC
  	//PRR = 0xFF;  // turn off various modules (kills timers and therefore sound)

	readHiscore();
	attractMode();
}

// Updates display with current score.
// Flashes 3 digits quickly on the display.
// Display is turned off if enable is false.
void updateDisplay(int score, boolean enable)
{
	int s = score;

	for(int pos = 2; pos >= 0; pos--)
	{
		int digit = s % 10;
		s /= 10;

		// turn on segments
		for(byte i = 0; i < 7; i++)
		{
			if(segments[digit] & (1<<i))
			{
				// segment on
				pinModeF(segmentPins[i], OUTPUT);
				digitalWriteF(segmentPins[i], LOW);	
			}
			else
			{
				// segment off
				pinModeF(segmentPins[i], INPUT);
				digitalWriteF(segmentPins[i], LOW);	
			}
		}

		// turn on the digit
		if(enable)
		  digitalWriteF(digitPins[pos], HIGH);

		//delayMicroseconds(50);
		//delayMicroseconds(5);

		__asm__ __volatile__ (
			".rept 100\n\t"
			"nop\n\t"
			".endr"
			:
			:
			:
		);

		// turn off the digit
		digitalWriteF(digitPins[pos], LOW);

		// lower duty cycle to save battery
		//delayMicroseconds(100);
		//delayMicroseconds(10);

		__asm__ __volatile__ (
			".rept 100\n\t"
			"nop\n\t"
			".endr"
			:
			:
			:
		);

		// kill leading zeros to save power
		//if(s == 0)
		//	return;
	}
}

// Updates the start menu. Switch between previous score and hiscore on the display.
// Start a new game when a button is pressed. Clear the hiscore is all buttons are held down for 2 seconds.
void startMenu()
{
	if(startMenuTimer == 0) {
		if(++attractModeTimer == 5 - powerSave*3) {
			attractMode();
			attractModeTimer = 0;
			powerSave = 1;	// enter power save mode
		}
	}

	// flick between previous score and hiscore display
	int s = score;
	startMenuTimer++;
	if(startMenuTimer >= 4000)
		s = hiscore[currentGameMode];
	if(startMenuTimer >= 8000)
		startMenuTimer = 0;

	updateDisplay(s, startMenuTimer < 3975 || (startMenuTimer > 4000 && startMenuTimer < 7975));

	// read button state
	int buttonState = 0;
	for(int i = 0; i < 4; i++)
		if(digitalReadF(buttonPins[i]) == LOW)
	buttonState |= 1<<i;

	// reset hiscore if all buttons are held down for 2 seconds
	static long resetHiscoreTimer = 0;
	if(buttonState == 15)
	{
		if(resetHiscoreTimer == 0)
			resetHiscoreTimer = millis();

		if(millis() - resetHiscoreTimer > 2000/CLOCK_SCALER)
		{
			updateDisplay(0, false);
			mytone(500);
			hiscore[0] = 0;
			hiscore[1] = 0;
			writeHiscore();
			delay(700/CLOCK_SCALER);
			toneoff();
			resetHiscoreTimer = 0;
		}
	}
	else
	{
		resetHiscoreTimer = 0;
	}

	// start new game if a single button is pressed for 100ms
	static int startNewGameTimer = 0;
	if(buttonState == 1 || buttonState == 2 || buttonState == 4 || buttonState == 8)
	{
		if(startNewGameTimer == 0)
			startNewGameTimer = millis();
		if(millis() - startNewGameTimer > 50/CLOCK_SCALER)
		{  
			// start new game
			updateDisplay(score, false);
			delay(2000/CLOCK_SCALER);

			int gameMode = (buttonState == 1 || buttonState == 2 ? STATE_SPEED_GAME : STATE_MEMORY_GAME);

			startNewGame(gameMode);
			startNewGameTimer = 0;
		}
	}
	else
	{
		startNewGameTimer = 0;
	}

	// power save "sleep"
	// throttles update frequency so that display leds are dimmer
	if(powerSave)
		delayMicroseconds(500);
}

void attractMode()
{
	const int8_t seq1[] = { 0, 1, 2, 3, 2, 1, 0, -1 };
	const int8_t* s = seq1;

	while(*s >= 0)
	{
		int8_t i = *s++;
		digitalWriteF(ledPins[i], HIGH);
		mytone(toneFreq[i]);
		delaydisp(500);
		digitalWriteF(ledPins[i], LOW);
		toneoff();
		delaydisp(50);
	}
}

//=====================================================
// Game 1: Speed Game
//=====================================================

Random memRand1;
Random memRand2;
int memState1;
int memState2;
int gameSpeed;

// Prepares game state for a new game.
void startNewGame(int gameMode)
{
	state = gameMode;
	currentGameMode = (gameMode == STATE_SPEED_GAME ? 0 : 1);
	score = 0;
	level = -1;
	led = -1;
	prevLed = -1;
	nextTimer = 0;
	gameSpeed = 450;	// initial game speed, higher values are slower
	powerSave = 0;

	for(int i = 0; i < 4; i++) 
		prevButtonState[i] = HIGH;

	// set random seed, so that every game has a different sequence
	randomSeed(millis());
	memSeed = millis();

	// new seed for memory game
	memRand1.init(memSeed);
	memRand2.init(memSeed);
	memState1 = 0;
	memState2 = 0;
}

void playSpeedGame()
{
	// update time
	nextTimer--;
	if(nextTimer < 10)
	{
		toneoff();
		led = -1;
	}
	if(nextTimer < 0)
	{
		led = memRand1.randomTweaked(4);
		memState1++;

		prevLed = led;

		if(level < 40)
			gameSpeed -= 3;
		else if(level < 80)
			gameSpeed -= 2;
		else if(level < 120)
			gameSpeed--;
		else
			gameSpeed -= level & 1;
		gameSpeed = max(gameSpeed, 100);
			
		nextTimer = gameSpeed;
		//nextTimer = max(400 - level*3, 50);

		level++;

		mytone(toneFreq[led]);
	}

	// update leds
	for(int i = 0; i < 4; i++)
		digitalWriteF(ledPins[i], led == i || (digitalReadF(buttonPins[i]) == LOW && nextTimer > 5) ? HIGH : LOW);

	// update display
	updateDisplay(score, true);

	// update input
	for(int i = 0; i < 4; i++)
	{
		int but = digitalReadF(buttonPins[i]);
		if(but == LOW && prevButtonState[i] == HIGH)
		{
			// ignore button press if time since last press is too short
			if(millis() - lastButtonPress > 50/CLOCK_SCALER) 
			{ 
				// correct button pressed?
				if( i == memRand2.randomTweaked(4) )
				{
					score++;
					memState2++;

					if(memState1 == memState2) {
						toneoff();
						led = -1;  // turn off led
					}
				}
				else
				{
					gameOver();
				}

				lastButtonPress = millis();
			}
		}
		prevButtonState[i] = but;
	}
}

// Game over. Play a game over sound and blink score.
void gameOver()
{
	mytone(62*2);	// 250hz

	// new hiscore?
	if(score > hiscore[currentGameMode])
	{
		hiscore[currentGameMode] = score;
		writeHiscore();
	}

	// turn on leds
	for(int i = 0; i < 4; i++)
		digitalWriteF(ledPins[i], HIGH);

	// flash display
	for(int i = 0; i < 70*5*2; i++)
	{
		if(i == 70*4)	// was 70*2
			mytone(78*2);
		boolean enable = 1 - (i/100) & 1;	// was i/60
		updateDisplay(score, enable);
	}

	toneoff();

	// turn off leds
	for(int i = 0; i < 4; i++)
		digitalWriteF(ledPins[i], LOW);
	updateDisplay(score, false);

	// enter menu
	state = STATE_START_MENU;
	startMenuTimer = 0;
	attractModeTimer = 0;
}

//=====================================================
// Game 2: Memory Game
//=====================================================

byte readButtonState()
{
	byte buttonState = 0;
	if(digitalReadF(buttonPins[0]) == LOW)
		buttonState += 1;
	if(digitalReadF(buttonPins[1]) == LOW)
		buttonState += 2;
	if(digitalReadF(buttonPins[2]) == LOW)
		buttonState += 4;
	if(digitalReadF(buttonPins[3]) == LOW)
		buttonState += 8;
	return buttonState;
}

byte whichButton(byte state)
{
	if(state == 1)
		return 0;
	else if(state == 2)
		return 1;
	else if(state == 4)
		return 2;
	else if(state == 8)
		return 3;
	else
		return -1;
}

void delaydisp(int ms)
{
	for(int i = 0; i < ms/6; i++)
	{
		updateDisplay(score, true);
	}
}

void playMemoryGame()
{
	level = max(level, 0);

	// show sequence
	randomSeed(memSeed);
	for(int i = 0; i < level; i++)
	{
		int led = random(0,4);

		// flash led & play tone
		mytone(toneFreq[led]);
		digitalWriteF(ledPins[led], HIGH);
		delaydisp(600*2);
		digitalWriteF(ledPins[led], LOW);
		toneoff();

		delaydisp(200*2);
	}

	// wait for user to repeat sequence
	randomSeed(memSeed);
	int len = 0;
	while(len < level)
	{
		updateDisplay(score, true);

		// read input
		int buttonState = readButtonState();   

		// game over if more than one button is down
		if(buttonState != 0 && buttonState != 1 && buttonState != 2 && buttonState != 4 && buttonState != 8)
		{
			gameOver();
			return;
		}

		if(buttonState > 0)
		{
			// button is pressed                   
			// lit up button and play tone
			byte b = whichButton(buttonState);
			mytone(toneFreq[b]);
			digitalWriteF(ledPins[b], HIGH);

			delaydisp(100);

			// wait until button is released and stop sound and turn off leds
			while(readButtonState() != 0)
				updateDisplay(score, true);
			toneoff();
			for(int i = 0; i < 4; i++)
				digitalWriteF(ledPins[i], LOW);

			// was the correct button pressed?
			int next = random(0,4);
			if(b == next)
			{
				// correct button was pressed
				score++;
				len++;
			}
			else
			{
				gameOver();
				return;
			}
		}
	}
    
	delaydisp(2000);
	level++;
}

void loop()
{	
	// test sound (pin 3)
	// mytone(56*2); delay(100); toneoff(); delay(100);	// 279hz
	// mytone(50*2); delay(100); toneoff(); delay(100);	// 312.5hz
	// mytone(42*2); delay(100); toneoff(); delay(100);	// 372hz
	// mytone(38*2); delay(100); toneoff(); delay(100);	// 411hz
	// tone(3, toneFreq[0], 100); delay(100); toneoff(); delay(100);
	// tone(3, toneFreq[1], 100); delay(100); toneoff(); delay(100);
	// tone(3, toneFreq[2], 100); delay(100); toneoff(); delay(100);
	// tone(3, toneFreq[3], 100); delay(100); toneoff(); delay(100);

	// test display
	//int s = millis() / 100;
	//updateDisplay(s, true);
	//delay(1);

	// test sound
	// for(int i = 0; i < 4; i++)
	// {
	// 	//tone(11, toneFreq[i]*8, 1000);
	// 	mytone(toneFreq[i]);
	// 	delay(1500/8);
	// }

	// test buttons & leds
	/*
	for(byte i = 0; i < 4; i++)
	{
		if(digitalRead(buttonPins[i]) == LOW)
		{
			digitalWriteF(ledPins[0], LOW);
			digitalWriteF(ledPins[1], LOW);
			digitalWriteF(ledPins[2], LOW);
			digitalWriteF(ledPins[3], LOW);
			digitalWriteF(ledPins[i], HIGH);
		}
	}
	*/

	if(state == STATE_START_MENU)
		startMenu();
	else if(state == STATE_SPEED_GAME)
		playSpeedGame();
	else if(state == STATE_MEMORY_GAME)
		playMemoryGame();
	else
		gameOver();
}
