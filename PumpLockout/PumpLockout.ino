
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <SevenSegment.h>
#include <EEPROM.h>

/* 
 *  TODO:
 *  
 *  Rewrite delay so it is hidden in the countdown?
 *  Switch to timing using Timer2 and external clock?
 *  Check power draw
 */

//*********************  VERSION NUMBER   ***********************************
const char*   kVersion = " 1.6L";

// Digital and analog I/O
#define IN_SWITCH_CLEAR 2
#define IN_SWITCH_PUMP_REQUEST 3
#define IN_PUMP_CURRENT A2
#define LCD_CLOCK 4
#define LCD_DATA  5
#define LCD_LOAD  6
#define PRI_RELAY_CLOSE 7
#define SEC_RELAY_CLOSE 8
#define IN_ENCODER_A A1
#define IN_ENCODER_B A0
#define IN_ENCODER_SWITCH  10
#define LED_LOCKOUT 11
#define LED_WAIT    12
#define LED_RUN     13

/* Pump threshold
 ( 1/2 HP / 80% efficiency ) / 120 V = 3.9 A rms
 TA12-100 transformer with 200 Ω readout gives 1 V = 5 A
 So 3.9 A = 0.78 V rms
 We have a rectifier so we see peak voltage, 0.78 * 1.414 = ~1.1 V when pump is running
 RC time constant of rectifier circuit is 0.141 s (8.5 cycles)
 Set threshold to 0.2 V, which occurs 0.22 s (13 cycles) after pump shuts off.

 Measured in circuit (I think my TA12-100 has 50 Ω resistor instead of 200),
 after the Schottky diode I get a DC value of .275 V for 1.775 A, with a forward drop
 of about 300 mV. So that's .575 V for 1.775 A, which gives 3.9 A ==> 1.26 V.
 Subtract back the forward drop to get about 0.9 V.
 I think a threshold of 0.2 V is still fine.
 */
static const int    kPumpOffCurrentThreshold = 0.2 * 1024.0 / 5.0;


typedef enum e_state
{
  st_ready = 0,   // Wait time has finished, or reset button has been pushed. Ready to pump
  st_pump,        // Pumping water
  st_decay,       // Waiting for pump to stop spinning
  st_wait,        // Waiting for minimum time between pumpings to finish
  st_lockout,     // Pump called for water too soon. Pumping is locked out until reset.
  st_program      // Program button has been pushed, setting max pump time or wait time
} state_t;

typedef enum e_prog
{
  pr_pump = 0,
  pr_wait,
  pr_exit
} prog_t;

state_t state;
prog_t  progState;

unsigned long timeVal;    // Current time (counting up or down), in seconds

// Seconds to wait for pump (or test motor) to spin down so it's not producing voltage
static const unsigned long  kDecayTime = 4;

// Time settings, in seconds
unsigned long maxPumpTime;
unsigned long minWaitTime;

// Time remaining in current mode, in seconds
long   timeRemainingInMode;

// Value of millis() when timeRemainingInMode was last decremented
unsigned long   msAtLastDecrement;

// Time display types
typedef enum e_printStyle
{
  printStyle_MMSS = 0,     // MM:SS
  printStyle_HHMM,         // HH:MM
  printStyle_secs,         // ---S
} printStyle_t;

// Next state when coming out of decay
static state_t nextState;

// Max allowable set times, in seconds
static const unsigned long minMaxPumpTime = 20;          // 20 seconds would mean we've installed really high power pump!
static const unsigned long maxMaxPumpTime = 10 * 60;     // 10 minutes is a really long time for the pump to run
static const unsigned long minMinWaitTime = 30 * 60;     // 30 minutes is a lot of water usage
static const unsigned long maxMinWaitTime = 9 * 60 * 60; // 9 hours is too long if there's more than two people up there

// Time step when programming, in seconds
static const long   kPumpStep = 1;
static const long   kWaitStep = 60;

// Maximum time to display in lockout mode, when we count up. 99 hours, 99 minutes
static const unsigned long  kMaxLockoutTime = 362340;

// Where the time settings are stored in EEPROM
#define kMaxPumpTimeAddress   0
#define kMinWaitTimeAddress   4

// Encoder rotation and button-press
typedef enum e_encoderState
{
  es_none = 0,
  es_click,
  es_increment,
  es_decrement
} encoderState_t;

encoderState_t  encoderState;
boolean           encoderA;       // Last (debounced) state of pin A (clock), HIGH or LOW. HIGH means input is high, which is switch-open
boolean           encoderSwitch;  // Last (debounced) state of the switch, true or false (true means closed, which is LOW)
unsigned long               programSwitchTimeout;         // Time (from millis()) when we go to program mode if button still down
static const unsigned long  kProgramSwitchDelay = 2000;   // Delay for going to program mode, in ms.

// Switch closures
volatile boolean  clearSwitchPushed;
volatile boolean  pumpRequested;

// LCD display
SevenSegment lcd(LCD_CLOCK, LCD_DATA, LCD_LOAD);

/*********************   Sleep stuff    ***************************/
volatile char f_wdt=1;  // watchdog overflow detection, which is just ignored.

// Converting ticks to milliseconds
const unsigned short millisPerTick = 138;   // Assumes Watchdog prescaler set to WDTCSR_PRE_125ms
volatile unsigned long millisCounter;
volatile unsigned long millisOffset;

// Counting individual sleeps, for more steady flashing
volatile unsigned long sleepTicks;
volatile unsigned short blinkTicks;

// Blinking LED times for program mode
const unsigned short redBlinkCycleTicks = 8;    // 8 ticks (= 1 second) blink cycle time
const unsigned short redBlinkOnTicks = 3;      // 3 ticks (=0.375 second) blink duty cycle

// Watchdog Timer Control Register values for prescaler (section 10.9.2 in ATmega328P data sheet)
// All other bits in WDTCSR should be zero when writing the prescaler bits
// (See the manual or setup() code to see what you have to do prior to writing the bits)
// Note that the timings in the constant names assume a 125 kHz oscillator, but according
// to the graph of oscillator frequency vs temperature and voltage, it's more like 119 kHz,
// so for instance  the "125ms" constant gives a period of 137 ms at 20°C and 5 V.
#define WDTCSR_PRE_16ms   0b000000
#define WDTCSR_PRE_32ms   0b000001
#define WDTCSR_PRE_63ms   0b000010
#define WDTCSR_PRE_125ms  0b000011
#define WDTCSR_PRE_250ms  0b000100
#define WDTCSR_PRE_p5     0b000101
#define WDTCSR_PRE_1s     0b000110
#define WDTCSR_PRE_2s     0b000111
#define WDTCSR_PRE_4s     0b100000
#define WDTCSR_PRE_8s     0b100001





/***************************************************
 *  Name:        setup
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: Setup for the serial comms and the
 *                Watch dog timeout. 
 *
 ***************************************************/
void setup() {

  // Set output pins
  pinMode(LCD_CLOCK, OUTPUT);
  pinMode(LCD_DATA, OUTPUT);
  pinMode(LCD_LOAD, OUTPUT);
  pinMode(PRI_RELAY_CLOSE, OUTPUT);
  pinMode(SEC_RELAY_CLOSE, OUTPUT);
  pinMode(LED_LOCKOUT, OUTPUT);
  pinMode(LED_WAIT, OUTPUT);
  pinMode(LED_RUN, OUTPUT);

  // Set input pins
  pinMode(IN_SWITCH_CLEAR, INPUT_PULLUP);
  pinMode(IN_SWITCH_PUMP_REQUEST, INPUT_PULLUP);
  pinMode(IN_ENCODER_A, INPUT_PULLUP);
  pinMode(IN_ENCODER_B, INPUT_PULLUP);
  pinMode(IN_ENCODER_SWITCH, INPUT_PULLUP);
  pinMode(IN_PUMP_CURRENT, INPUT);

  // Set up LCD wiring. AY0438 driver chip. Digit, d.p., digit, colon, dp, digit, dp, digit.
  // See README file from SevenSegment library for more info
  lcd.begin( "AY0438", "8.8|8.8" );

  // Segment/lamp test
  lcd.print( "8.8:8.8" );

  digitalWrite( LED_RUN, HIGH );
  digitalWrite( LED_WAIT, HIGH );
  digitalWrite( LED_LOCKOUT, HIGH );
  
  Serial.begin(115200);
  Serial.println("Initialising...");
  delay(100); //Allow for serial print to complete.

  // Set an interrupt on IN_SWITCH_CLEAR, looking for a rising edge signal and executing the "clearSwitch" Interrupt Service Routine
  attachInterrupt( digitalPinToInterrupt(IN_SWITCH_CLEAR), clearSwitch ,FALLING); 

  // Set an interrupt on IN_SWITCH_PUMP_REQUEST, looking for a rising edge signal and executing the "pumpRequest" Interrupt Service Routine
  attachInterrupt( digitalPinToInterrupt(IN_SWITCH_PUMP_REQUEST), pumpRequest ,FALLING); 

  clearSwitchPushed = false;
  nextState = st_lockout;   // This should be overwritten before being read

  // set state of programming encoder
  encoderA = digitalRead( IN_ENCODER_A );
  encoderSwitch = false;   // Start with switch not pressed
  programSwitchTimeout = 0;

  millisOffset = 0;
  millisCounter = 0;
  sleepTicks = 0;
  
  noInterrupts();
  //Asynchronous Operation of Timer/Counter2 (so it keeps running during sleep)
  //The CPU main clock frequency must be more than four times the oscillator frequency
  //a. Disable the Timer/Counter2 interrupts by clearing OCIE2x and TOIE2
  TIMSK2 = 0;
  
  //b. Select clock source by setting AS2 as appropriate
  //When AS2 is written to one, Timer/Counter2 is clocked from a crystal oscillator
  //connected to the timer oscillator 1 (TOSC1) pin
  ASSR |= (1 << AS2);
  
  //c. Write new values to TCNT2, OCR2x, and TCCR2x
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;

  //d. To switch to asynchronous operation: Wait for TCN2xUB, OCR2xUB, and TCR2xUB
  while (ASSR & 0x1F) {}

  //no need to change TCCR2A, normal counting mode
  //prescaler set to 128
  TCCR2B |= (1 << CS22) | (1 << CS20);

  //e. Clear the Timer/Counter2 interrupt flags
  TIFR2 = 0x07;

  /*** Setup the Watchdog Timer ***/
  
  /* Clear the reset flag. */
  MCUSR &= ~(1<<WDRF);
  
  /* In order to change WDE or the prescaler, we need to
   * set WDCE (This will allow updates for 4 clock cycles).
   */
  WDTCSR |= (1<<WDCE) | (1<<WDE);

  /* set new watchdog timeout prescaler value */
  //WDTCSR = 1<<WDP1 | 1<<WDP2; /* 1.0 seconds */
  WDTCSR = WDTCSR_PRE_125ms;       
  
  /* Enable the WD interrupt (note no reset). */
  WDTCSR |= _BV(WDIE);

  //f. Enable interrupts, if needed
  // TIMSK2 |= (1 << TOIE2);
  interrupts();

  // Get the saved values of timeouts from EEPROM
  EEPROM.get( kMaxPumpTimeAddress, maxPumpTime );
  EEPROM.get( kMinWaitTimeAddress, minWaitTime );
  
  Serial.print("Plop version ");
  Serial.println( kVersion );
  delay(100); //Allow for serial print to complete.


  // Leave segment/lamp test there for a while
  delay( 800 );
  lcd.print( "PLOP" );
  digitalWrite( LED_RUN, LOW );
  digitalWrite( LED_WAIT, LOW );
  digitalWrite( LED_LOCKOUT, LOW );


  // Leave splash screen up for a while
  delay( 1300 );

  // Display version number
  lcd.print( kVersion );
  delay( 1300 );
  setState( st_ready );

}


/***************************************************
 *  Name:        clearSwitch
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: Interrupt routine called when the CLEAR button is pressed
 * 
 *
 ***************************************************/
void clearSwitch( void )
{
  clearSwitchPushed = true;
}


/***************************************************
 *  Name:        pumpRequest
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: Interrupt routine called when the pump pressure switch closes to request pumping
 * 
 *
 ***************************************************/
void pumpRequest( void )
{
  pumpRequested = true;
}


/***************************************************
 *  Name:        ISR(WDT_vect)
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: Watchdog Interrupt Service. This
 *               is executed when watchdog timed out.
 *               
 * From https://donalmorrissey.blogspot.com/2010/04/sleeping-arduino-part-5-wake-up-via.html
 * Archived at https://web.archive.org/web/20210506142954/https://donalmorrissey.blogspot.com/2010/04/sleeping-arduino-part-5-wake-up-via.html
 * 
 * In addition, I count the number of ticks to track milliseconds. It's only good to ~1% over the typical summertime
 * temperature range in Maine, but that's probably good enough, and it lets me use POWER DOWN mode for minimum power draw.
 * We also collect the ms counter value when awakening, so that we actually have a globally 
 * monotonic value for myMillis within one watchdog tick.
 * 
 *
 ***************************************************/
ISR(WDT_vect)
{
  if(f_wdt == 0)
  {
    f_wdt=1;
    millisCounter += millisPerTick;
      
    millisOffset = millis();
  }
  else
  {
    //Serial.println("WDT Overrun!!!");
  }
}


/***************************************************
 *  Name:        enterSleep
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: Enters the arduino into sleep mode.
 *
 ***************************************************/
void enterSleep(void)
{
  /* 
   *  Now using SLEEP_MODE_PWR_DOWN for lowest power consumption. 
   */
  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   
  sleep_enable();
  
  sleep_mode();                               // Here's the actual call where we sleep

  /*
   * Wake back up.
   * First thing to do is disable sleep. 
   */
  sleep_disable();


  // Count the sleep tick for flashing the red light
  sleepTicks += 1;
  blinkTicks += 1;
  
  /* Re-enable the peripherals. */
  power_all_enable();
}

/***************************************************
 *  Name:        myMillis()
 *
 *  Returns:     corrected number of milliseconds
 *
 *  Parameters:  None.
 *
 *  Description: Because we sleep in power-save (or possibly power-down) mode, timer0,
 *                which is used by the system millis(), is disabled during sleep.
 *                This function provides a corrected version that should track
 *                true elapsed time. Works by counting ticks from the watchdog timer.
 *                Which is not particularly accurate, but should be better than 1%, and
 *                provides lower power draw than using POWER_SAVE mode.
 *
 ***************************************************/
unsigned long myMillis(void)
{
  unsigned long mm;

  mm = millis() - millisOffset;
  return millisCounter + mm;
}


void loop() {
  // If we're not in program mode, go to power-save for a second before doing whatever else we're doing.
  if( state != st_program )
  {
    /* Don't forget to clear the overrun flag. */
    f_wdt = 0;
    
    /* Re-enter sleep mode. */
    enterSleep();

    // Check to see if the switch to enter program mode is being held down.
    // After 3 seconds we return true
    if( checkProgramSwitch() )
    {
      setState( st_program );
    }

  }


  switch( state )
  {
    case st_ready:
      doReady();
      break;
      
    case st_pump:
      doPump();
      break;
      
    case st_decay:
      doDecay();
      break;
      
    case st_wait:
      doWait();
      break;
      
    case st_lockout:
      doLockout();
      break;
      
    case st_program:
      doProgram();
      break;

  }
  
}


/***************************************************
 *  Name:        setState
 *
 *  Returns:     nothing
 *
 *  Parameters:  
 *   e_state  newState    state to set to
 *
 *  Description: Sets the state to newState, and does whatever actions happen on that state change
 *
 ***************************************************/
 void setState( state_t newState )
 {

//  Serial.print( "state: " );
//  Serial.println( newState );
  
  switch( newState )
  {
    case st_ready:
      digitalWrite( PRI_RELAY_CLOSE, LOW );   // Make extra sure relays are both unpowered
      digitalWrite( SEC_RELAY_CLOSE, LOW );   // Make extra sure relays are both unpowered
      digitalWrite( LED_RUN, LOW );
      digitalWrite( LED_WAIT, LOW );
      digitalWrite( LED_LOCKOUT, LOW );

      // Check pump request line directly in case it was on when we booted up
      pumpRequested = ( digitalRead( IN_SWITCH_PUMP_REQUEST ) == LOW );
  
      lcd.clear();
      lcd.print( "rvn " );
//      Serial.println( "rvn " );

      break;
      
    case st_pump:
      digitalWrite( SEC_RELAY_CLOSE, HIGH ); // Disconnect sense terminals from primary relay
      delay( 10 );                           // Secondary relay is guaranteed to switch in 3 ms
      digitalWrite( PRI_RELAY_CLOSE, HIGH ); // Connect power to pump
      digitalWrite( LED_RUN, HIGH );
      digitalWrite( LED_WAIT, LOW );
      digitalWrite( LED_LOCKOUT, LOW );
      timeRemainingInMode = maxPumpTime;
      msAtLastDecrement = myMillis() - 1000;
      //printToLCD( timeRemainingInMode, printStyle_MMSS );

      // Wait for pump to start drawing current so the pump-done detector doesn't fire immediately
      delay( 500 );
      break;
      
    case st_decay:
      // Disconnect pump power, but leave sense terminals disconnected
      digitalWrite( PRI_RELAY_CLOSE, LOW );   // Disconnect power from pump
      digitalWrite( SEC_RELAY_CLOSE, HIGH );  // Keep sense terminals disconnected from primary relay

      if( st_wait == nextState )
      {
        digitalWrite( LED_RUN, LOW );
        digitalWrite( LED_WAIT, HIGH );
        digitalWrite( LED_LOCKOUT, LOW );
      }
      else if( st_lockout == nextState )
      {
        digitalWrite( LED_RUN, LOW );
        digitalWrite( LED_WAIT, LOW );
        digitalWrite( LED_LOCKOUT, HIGH );
      }
      timeRemainingInMode = kDecayTime;
      msAtLastDecrement = myMillis() - 1000;
      printToLCD( timeRemainingInMode, printStyle_secs );
      break;
      
    case st_wait:
      digitalWrite( SEC_RELAY_CLOSE, LOW ); // Connect sense terminals to primary relay
      digitalWrite( PRI_RELAY_CLOSE, LOW ); // Keep power to pump off
      digitalWrite( LED_RUN, LOW );
      digitalWrite( LED_WAIT, HIGH );
      digitalWrite( LED_LOCKOUT, LOW );
      timeRemainingInMode = minWaitTime;
      msAtLastDecrement = myMillis() - 1000;
      pumpRequested = false;
      clearSwitchPushed = false;
      //printToLCD( timeRemainingInMode, printStyle_HHMM );
      break;
      
    case st_lockout:
      digitalWrite( SEC_RELAY_CLOSE, LOW ); // Connect sense terminals to primary relay
      digitalWrite( PRI_RELAY_CLOSE, LOW ); // Keep power to pump off
      digitalWrite( LED_RUN, LOW );
      digitalWrite( LED_WAIT, LOW );
      digitalWrite( LED_LOCKOUT, HIGH );
      timeRemainingInMode = 0;          // We count up in lockout mode
      msAtLastDecrement = myMillis() - 1000;
      blinkTicks = 0;
      clearSwitchPushed = false;
      //printToLCD( timeRemainingInMode, printStyle_HHMM );
      break;
            
    case st_program:
      digitalWrite( PRI_RELAY_CLOSE, LOW );
      digitalWrite( SEC_RELAY_CLOSE, LOW );
      digitalWrite( LED_RUN, HIGH );
      digitalWrite( LED_WAIT, LOW );
      digitalWrite( LED_LOCKOUT, LOW );

      encoderA = HIGH;        // Start ready for a high->low transition
      encoderSwitch = true;   // Start with switch pressed, so we don't immediately leave
      
      progState = pr_pump;
      printToLCD( maxPumpTime, printStyle_MMSS );
      
      break;

    
  }

  state = newState;
  
 }



/***************************************************
 *  Name:         doReady
 *
 *  Returns:      nothing
 *
 *  Parameters:   nothing
 *
 *  Description:  Main loop function in ready state
 *
 ***************************************************/
void doReady( void )
{
  // Check for pump request
  if( pumpRequested )
  {
    setState( st_pump );
  }
}


/***************************************************
 *  Name:         doPump
 *
 *  Returns:      nothing
 *
 *  Parameters:   nothing
 *
 *  Description:  Main loop function in pump state.
 *                Update time remaining on LCD
 *                If time goes to zero, go to lockout state
 *                If pump request goes away, go to decay state
 *                Ignore clear requests
 *
 ***************************************************/
void doPump( void )
{
  unsigned long   msSinceLastDecrement;
  long            decrement;
  boolean         pumpDone;
  int             current;
  
  msSinceLastDecrement = myMillis() - msAtLastDecrement;

  if( msSinceLastDecrement >= 1000 )
  {
    // Decrement counter
    decrement = msSinceLastDecrement / 1000;
    timeRemainingInMode -= decrement;
    msAtLastDecrement += decrement * 1000;
    
    if( timeRemainingInMode <0 )
    {
      // Pump overran! Go to lockout via decay state
      nextState = st_lockout;
      setState( st_decay );
    }
    else
    {
      // Still time remaining.
      // Update display
      printToLCD( timeRemainingInMode, printStyle_MMSS );

      // Check for pump done
      // Which we do by looking for voltage on the current meter pin
      // It has a time constant of several cycles (see calculation at definition of kPumpOffCurrentThreshold),
      // so by the time we see it being low we know the current is zero (or very close).

      current = analogRead( IN_PUMP_CURRENT );
      if( current < kPumpOffCurrentThreshold )
      {
        // Go to decay state to wait for pump to spin down, then to wait state
        nextState = st_wait;
        setState( st_decay );
      }
    }
  }
}


/***************************************************
 *  Name:         doDecay
 *
 *  Returns:      nothing
 *
 *  Parameters:   nothing
 *
 *  Description:  Main loop function in decay state (waiting for pump to spin down so it's not creating voltage)
 *                Update time remaining on LCD
 *                If time goes to zero, go to wait state
 *                If pump request occurs, go to lockout state
 *                Ignore clear button
 *
 ***************************************************/
void doDecay( void )
{
  unsigned long   msSinceLastDecrement;
  long            decrement;
  boolean         newPumpRequest;

  msSinceLastDecrement = myMillis() - msAtLastDecrement;
  if( msSinceLastDecrement >= 1000 )
  {
    // Decrement counter
    decrement = msSinceLastDecrement / 1000;
    timeRemainingInMode -= decrement;
    msAtLastDecrement += decrement * 1000;

    if( timeRemainingInMode <0 )
    {
      // Pump should have spun down by now, we can go check for more requests
      // In reality the pump has 50 psi of pressure bringing it to a quick halt;
      // this is really only needed so I can test with, say, a table saw.
      setState( nextState );
    }
    else
    {
      // Still time remaining.
      // Update display
      printToLCD( timeRemainingInMode, printStyle_secs );

    }
  }
}



/***************************************************
 *  Name:         doWait
 *
 *  Returns:      nothing
 *
 *  Parameters:   nothing
 *
 *  Description:  Main loop function in wait state
 *                Update time remaining on LCD
 *                If time goes to zero, go to ready state
 *                If pump request occurs, go to lockout state
 *                If clear button is pressed, go to ready state
 *
 ***************************************************/
void doWait( void )
{
  unsigned long   msSinceLastDecrement;
  long            decrement;
  boolean         newPumpRequest;

  msSinceLastDecrement = myMillis() - msAtLastDecrement;
  if( msSinceLastDecrement >= 1000 )
  {
    // Decrement counter
    decrement = msSinceLastDecrement / 1000;
    timeRemainingInMode -= decrement;
    msAtLastDecrement += decrement * 1000;

    if( timeRemainingInMode <0 )
    {
      // The full wait time passed. Ready for more pumping.
      setState( st_ready );
    }
    else
    {
      // Still time remaining.
      // Update display
      printToLCD( timeRemainingInMode, printStyle_HHMM );

      // Check for pump request
      newPumpRequest = ( digitalRead( IN_SWITCH_PUMP_REQUEST ) == LOW );
      if( newPumpRequest )
      {
        setState( st_lockout );
      }
      else if( clearSwitchPushed )
      {
        setState( st_ready );
      }
    }
  }
}


/***************************************************
 *  Name:         doLockout
 *
 *  Returns:      nothing
 *
 *  Parameters:   nothing
 *
 *  Description:  Main loop function in lockout state
 *                Update time remaining on LCD (count up to maximum value)
 *                If clear button is pressed, go to ready state
 *
 ***************************************************/
void doLockout( void )
{
  unsigned long   msSinceLastIncrement;
  long            increment;
  boolean         newPumpRequest;
  
  // Do the LED blinking asynchronous from actual elapsed time, so we don't get
  // aliasing between the sleep period and the blink period
  if( blinkTicks >= redBlinkCycleTicks )
  {
    // Turn off LED and reset counter. It flashes on for part of every second.
    digitalWrite( LED_LOCKOUT, HIGH );
    blinkTicks = 0;
  }
  else if( blinkTicks >= redBlinkOnTicks )
  {
    // Turn on LED. 
    digitalWrite( LED_LOCKOUT, LOW );
  }

  msSinceLastIncrement = myMillis() - msAtLastDecrement;
  if( msSinceLastIncrement >= 1000 )
  {
    // Increment counter
    increment = msSinceLastIncrement / 1000;
    timeRemainingInMode += increment;
    msAtLastDecrement += increment * 1000;

    if( timeRemainingInMode > kMaxLockoutTime )
    {
      // We've been locked out for more than 99:99. Don't roll over the display.
      timeRemainingInMode = kMaxLockoutTime;
    }
    
    // Update display
    printToLCD( timeRemainingInMode, printStyle_HHMM );

    // Check for clear button pressed
    if( clearSwitchPushed )
    {
      setState( st_ready );
    }
  }

}


/***************************************************
 *  Name:         doProgram
 *
 *  Returns:      nothing
 *
 *  Parameters:   nothing
 *
 *  Description:  Main loop function in program state. We stay here until we get a click,
 *                so that we don't miss any rotations (or worse, get the wrong sign)
 *
 ***************************************************/
void doProgram( void )
{
  encoderState_t  encoder;
  long  value;
  long  change;
  long  sign;
  boolean stayHere;

  for( stayHere = true; stayHere; )
  {
    encoder = checkEncoder();
  
    if( es_click == encoder )
    {
      // Move on to next programming mode
      if( pr_pump == progState )
      {
        progState = pr_wait;
        EEPROM.put( kMaxPumpTimeAddress, maxPumpTime );
        digitalWrite( LED_RUN, LOW );
        digitalWrite( LED_WAIT, HIGH );
        printToLCD( minWaitTime, printStyle_HHMM );
      }
      else
      {
        EEPROM.put( kMinWaitTimeAddress, minWaitTime );
        digitalWrite( LED_RUN, LOW );
        digitalWrite( LED_WAIT, HIGH );
        progState = pr_exit;
        setState( st_ready );
        programSwitchTimeout = 0;
      }

      stayHere = false;
    }
    else if( es_none != encoder )
    {
      // Increment or decrement
      if( es_increment == encoder )
      {
        sign = +1;
      }
      else
      {
        sign = -1;
      }
  
      if( pr_pump == progState )
      {
        value = maxPumpTime;
        change = kPumpStep;
      }
      else
      {
        value = minWaitTime;
        change = kWaitStep;
      }
  
      value += sign * change;
  
      if( pr_pump == progState )
      {
        if( value > maxMaxPumpTime )
        {
          value = maxPumpTime;
        }
        else if( value < minMaxPumpTime )
        {
          value = minMaxPumpTime;
        }
        maxPumpTime = value;
        printToLCD( maxPumpTime, printStyle_MMSS );
      }
      else
      {
        if( value > maxMinWaitTime )
        {
          value = maxMinWaitTime;
        }
        else if( value < minMinWaitTime )
        {
          value = minMinWaitTime;
        }
        
        minWaitTime = value;
        printToLCD( minWaitTime, printStyle_HHMM );
        EEPROM.put( kMinWaitTimeAddress, minWaitTime );
      }      
    }    
  }
}


/***************************************************
 *  Name:         checkProgramSwitch
 *
 *  Returns:      true if program switch has been pressed long enough to go to program state
 *
 *  Parameters:   none
 *
 *  Description:  It's time to go to program mode if the button has been pressed for 2 seconds.
 *                Note that in normal modes we only awaken occasionally, so it might actually take longer
 *
 ***************************************************/
boolean checkProgramSwitch()
{
  unsigned char   nowState;
  boolean         doIt = false;
  
  nowState = digitalRead( IN_ENCODER_SWITCH );

  if( LOW ==  nowState )
  {
    // Button is down. If we're not already counting, start counter to enter program state.
    // If millis rolls over, this will fail, and user will have to curse and push the button again.
    if( 0 == programSwitchTimeout )
    {
      programSwitchTimeout = myMillis() + kProgramSwitchDelay;
    }
    else if( myMillis() > programSwitchTimeout )
    {
      // Passed the delay without the switch being up during any poll
      doIt = true;
    }
  }
  else
  {
    // Button not currently down. Reset timer
    programSwitchTimeout = 0;
  }

  return doIt;
}

/***************************************************
 *  Name:         checkEncoder
 *
 *  Returns:      debounced encoder state (nothing, left rotation, right rotation, or button push)
 *
 *  Parameters:   none
 *
 *  Description:  Debounces encoder and returns most recent state (since last time this was called)
 *                Button pushes override rotations, if both have happened.
 *                This shall be called only in program state.
 *                This function is completely separate from checkProgramSwitch(), which is only for entering program state.
 *
 ***************************************************/
encoderState_t checkEncoder( void )
{
  encoderState_t  eState;
  unsigned char   nowSwitch;
  unsigned char   nowA;
  unsigned char   nowB;

  eState = es_none;   // Default is nothing happened
  
  // First check the switch
  nowSwitch = digitalRead( IN_ENCODER_SWITCH );
  if( HIGH == nowSwitch )
  {
    // Switch is not pressed, so the encoder switch is open.
    encoderSwitch = false;
  }
  
  if( false == encoderSwitch && LOW == nowSwitch )
  {
    // Transition from not-pushed to pushed.
    // Cheesy inline debounce
    delay( 5 );

    if( LOW == digitalRead( IN_ENCODER_SWITCH )  )
    {
      eState = es_click;
      encoderSwitch = true;
    }
  }
  else
  {
    // No switch-press. Continue on to check rotations
    // On negative-going edges of A, B is high=clockwise, low=CCW, for Bourns PEC12R
    nowA = digitalRead( IN_ENCODER_A );
    nowB = digitalRead( IN_ENCODER_B );

    if( HIGH == encoderA && LOW == nowA )
    {
      delay( 1 );   // debounce

      nowA = digitalRead( IN_ENCODER_A );
      if( LOW == nowA )
      {
        // Yup, still low. Must be a real transition.
        // Got a rotation, one direction or the other.

        // No debounce needed for B, since it's in quadrature to A
        if( HIGH == nowB )
        {
          eState = es_increment;
        }
        else
        {
          eState = es_decrement;
        }
      }
    }
    
    encoderA = nowA;
  }

  return eState;
}


 /***************************************************
 *  Name:         printToLCD
 *
 *  Returns:      nothing
 *
 *  Parameters:  
 *        unsigned long   value       The value to print, in seconds
 *        printStyle_t    printStyle  See definition of type for styles
 *        True if display as HH:MM, else display as MM:SS
 *
 *  Description:  Set the LCD to display the given value.
 *
 ***************************************************/
void printToLCD( unsigned long value, printStyle_t printStyle )
 {

  static char buf[9];
  
  unsigned long left;
  unsigned long right;

  if( printStyle_HHMM == printStyle )
  {
    value /= 60;
  }

  if( state != st_program && state != st_lockout )
  {
    // Add one to the value so we don't sit on 00:00 for a second
    // Don't do it if we're programming, or if we're counting up (lockout mode)
    value += 1;
  }
  
  left = value / 60;
  right = value % 60;

  if( left > 99 )
  {
    left = 99;
  }

  if( printStyle_secs == printStyle )
  {
    sprintf(buf, "---%01ld\0", right);
  }
  else
  {
    sprintf(buf, "%02ld:%02ld\0", left, right);
  }

  lcd.print(buf);
  
//  Serial.println(buf);
//  delay(100);

}



/***************************************************
 *  Name:         foo
 *
 *  Returns:      bar
 *
 *  Parameters:  
 *        unsigned long   baz       quux
 *
 *  Description:  froomkin
 *
 ***************************************************/
