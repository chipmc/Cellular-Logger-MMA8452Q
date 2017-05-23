/*
 * Project Cellular-Logger-MMA8452Q
 * Description: Cellular Connected Data Logger
 * Author: Chip McClelland
 * Date:18 May 2017
 */

 // Set parameters
 // The SparkFun MMA8452 breakout board defaults to 1, set to 0 if SA0 jumper on the bottom of the board is set
 #define SA0 1
 #if SA0
 #define MMA8452_ADDRESS 0x1D  // SA0 is high, 0x1C if low
 #else
 #define MMA8452_ADDRESS 0x1C
 #endif

 //Time Period Definitions - used for debugging
 #define HOURLYPERIOD Time.hour(t)   // Normally hour(t) but can use minute(t) for debugging
 #define DAILYPERIOD Time.day(t) // Normally day(t) but can use minute(t) or hour(t) for debugging

 //These defines let me change the memory map and configuration without hunting through the whole program
 #define VERSIONNUMBER 7             // Increment this number each time the memory map is changed
 #define WORDSIZE 8                  // For the Word size
 #define PAGESIZE 4096               // Memory size in bytes / word size - 256kb FRAM
 // First Word - 8 bytes for setting global values
 #define DAILYOFFSET 2               // First word of daily counts
 #define HOURLYOFFSET 30             // First word of hourly counts (remember we start counts at 1)
 #define DAILYCOUNTNUMBER 28         // used in modulo calculations - sets the # of days stored
 #define HOURLYCOUNTNUMBER 4064      // used in modulo calculations - sets the # of hours stored - 256k (4096-14-2)
 #define VERSIONADDR 0x0             // Memory Locations By Name not Number
 #define SENSITIVITYADDR 0x1         // For the 1st Word locations
 #define DEBOUNCEADDR 0x2            // One byte for debounce (stored in cSec mult by 10 for mSec)
 #define MONTHLYREBOOTCOUNT 0x3      // This is where we store the reboots - indication of system health
 #define DAILYPOINTERADDR 0x4        // One byte for daily pointer
 #define HOURLYPOINTERADDR 0x5       // Two bytes for hourly pointer
 #define CONTROLREGISTER 0x7         // This is the control register acted on by both Simblee and Arduino
 //Second Word - 8 bytes for storing current counts
 #define CURRENTHOURLYCOUNTADDR 0x8  // Current Hourly Count
 #define CURRENTDAILYCOUNTADDR 0xA   // Current Daily Count
 #define CURRENTCOUNTSTIME 0xC       // Time of last count
 //These are the hourly and daily offsets that make up the respective words
 #define DAILYDATEOFFSET 1           //Offsets for the value in the daily words
 #define DAILYCOUNTOFFSET 2          // Count is a 16-bt value
 #define DAILYBATTOFFSET 4           // Where the battery charge is stored
 #define HOURLYCOUNTOFFSET 4         // Offsets for the values in the hourly words
 #define HOURLYBATTOFFSET 6          // Where the hourly battery charge is stored
 // LED Pin Value Variables
 #define REDLED 4          // led connected to digital pin 4
 #define YELLOWLED 6       // The yellow LED
 #define LEDPWR 7          // This pin turns on and off the LEDs
 // Finally, here are the variables I want to change often and pull them all together here
 #define SOFTWARERELEASENUMBER "0.1"
 #define PARKCLOSES 19
 #define PARKOPENS 7
 // Defines for Ubidots
 #define TOKEN "6EALvOEabrFiA6XViWY3hxcFMG0yDr"  // Put here your Ubidots TOKEN
 #define DATA_SOURCE_NAME "umstead-electron-1"

 // Included Libraries
 #include "Adafruit_FRAM_I2C.h"  // Library for FRAM functions

 // Prototypes
 Adafruit_FRAM_I2C fram = Adafruit_FRAM_I2C(); // Init the FRAM
 FuelGauge batteryMonitor;      // Prorotype for the fuel gauge (included in Particle core library)


 // Pin Constants
 const int int2Pin = D2;
 const int redLED = D7;
 const int yellowLED = B1;
 const int greenLED = B0;
 const int tmp36Pin = A2;

 // Program Constants
 const byte SCALE = 2;          // Sets full-scale range to +/-2, 4, or 8g. Used to calc real g values.
 int InputValue = 0;            // Raw sensitivity input
 byte Sensitivity = 0x01;       // Hex variable for sensitivity
 int i=0;     // For stepping through responses in myHandler

 // Accelerometer Variables
 const byte accelFullScaleRange = 2;  // Sets full-scale range to +/-2, 4, or 8g. Used to calc real g values.
 const byte dataRate = 3;             // output data rate - 0=800Hz, 1=400, 2=200, 3=100, 4=50, 5=12.5, 6=6.25, 7=1.56
 byte accelInputValue = 1;            // Raw sensitivity input (0-9);
 byte accelSensitivity;               // Hex variable for sensitivity
 byte accelThreshold = 100;           // accelThreshold value to decide when the detected sound is a knock or not
 unsigned int debounce;               // This is a minimum debounce value - additional debounce set using pot or remote terminal

 // FRAM and Unix time variables
 time_t t;
 byte lastHour = 0;  // For recording the startup values
 byte lastDate = 0;   // These values make sure we record events if time has lapsed
 unsigned int hourlyPersonCount = 0;  // hourly counter
 unsigned int dailyPersonCount = 0;   //  daily counter
 byte currentHourlyPeriod;    // This is where we will know if the period changed
 byte currentDailyPeriod;     // We will keep daily counts as well as period counts
 int countTemp = 0;          // Will use this to see if we should display a day or hours counts

 // Battery monitor
 float stateOfCharge = 0;            // stores battery charge level value

 //Menu and Program Variables
 unsigned long lastBump = 0;         // set the time of an event
 boolean ledState = LOW;                 // variable used to store the last LED status, to toggle the light
 int delaySleep = 1000;               // Wait until going back to sleep so we can enter commands
 int menuChoice=0;                   // Menu Selection
 boolean refreshMenu = true;         //  Tells whether to write the menu
 boolean inTest = false;             // Are we in a test or not
 boolean LEDSon = true;              // Are the LEDs on or off
 unsigned int LEDSonTime = 30000;    // By default, turn the LEDS on for 30 seconds - remember only awake time counts not real seconds
 int numberHourlyDataPoints;         // How many hourly counts are there
 int numberDailyDataPoints;          // How many daily counts are there
 const char* releaseNumber = SOFTWARERELEASENUMBER;  // Displays the release on the menu
 byte bootcount = 0;                 // Counts reboots
 int bootCountAddr = 0;              // Address for Boot Count Number


 void setup()
 {
     Serial.begin(9600);
     Wire.begin();                       //Create a Wire object

     Serial.println("");                 // Header information
     Serial.print(F("Cellular-Logger-MMA8452Q - release "));
     Serial.println(releaseNumber);

     Particle.subscribe("hook-response/hourly", myHandler, MY_DEVICES);      // Subscribe to the integration response event

     pinMode(int2Pin,INPUT);            // accelerometer interrupt pinMode
     pinMode(redLED, OUTPUT);           // declare the Red LED Pin as an output
     pinMode(yellowLED, OUTPUT);        // declare the Yellow LED Pin as as OUTPUT
     pinMode(greenLED,OUTPUT);          // declare the Yellow LED Pin as as OUTPUT

     if (fram.begin()) {                // you can stick the new i2c addr in here, e.g. begin(0x51);
         Serial.println(F("Found I2C FRAM"));
     } else {
         Serial.println(F("No I2C FRAM found ... check your connections"));
     }

    if (FRAMread8(VERSIONADDR) != VERSIONNUMBER) {  // Check to see if the memory map in the sketch matches the data on the chip
      Serial.print(F("FRAM Version Number: "));
      Serial.println(FRAMread8(VERSIONADDR));
      Serial.read();
      Serial.println(F("Memory/Sketch mismatch! Erase FRAM? (Y/N)"));
      while (!Serial.available());
      switch (Serial.read()) {    // Give option to erase and reset memory
        case 'Y':
          ResetFRAM();
          break;
        case 'y':
          ResetFRAM();
          break;
        default:
          Serial.println(F("Cannot proceed"));
          BlinkForever();
      }
    }

     // Import the accelSensitivity and Debounce values from memory
     Serial.print(F("Sensitivity set to: "));
     accelSensitivity = FRAMread8(10-SENSITIVITYADDR);
     Serial.println(accelSensitivity);
     Serial.print(F("Debounce set to: "));
     debounce = FRAMread8(DEBOUNCEADDR)*10;     // We mulitply by ten since debounce is stored in 100ths of a second
     if (debounce > delaySleep) delaySleep = debounce;       // delaySleep must be bigger than debounce afterall
     Serial.println(debounce);

     byte c = readRegister(MMA8452_ADDRESS,0x0D);  // Read WHO_AM_I register for accelerometer
     if (c == 0x2A) // WHO_AM_I should always be 0x2A
     {
         initMMA8452(SCALE, dataRate);  // init the accelerometer if communication is OK
         Serial.println(F("MMA8452Q is online..."));
     }
     else
     {
         Serial.print(F("Could not connect to MMA8452Q: 0x"));
         Serial.println(c, HEX);
     }
     initMMA8452(SCALE,dataRate);

     Time.zone(-4);                   // Set time zone to Eastern USA daylight saving time
     StartStopTest(1);                // Default action is for the test to be running
 }

 void loop() {
   if (refreshMenu) {
       refreshMenu = 0;
       Serial.println(F("Remote Trail Counter Program Menu"));
       Serial.println(F("0 - Display Menu"));
       Serial.println(F("1 - Display status"));
       Serial.println(F("2 - Set the time zone"));
       Serial.println(F("3 - Change the sensitivitiy"));
       Serial.println(F("4 - Change the debounce"));
       Serial.println(F("5 - Reset the counter"));
       Serial.println(F("6 - Reset the memory"));
       Serial.println(F("7 - Start / stop counting"));
       Serial.println(F("8 - Dump hourly counts"));
       Serial.println(F("9 - Last 14 day's counts"));
       NonBlockingDelay(100);
   }
   if (Serial.available() >> 0) {      // Only enter if there is serial data in the buffer
       switch (Serial.read()) {          // Read the buffer
           case '0':
               refreshMenu = 1;
               break;
           case '1':   // Display Current Status Information
               Serial.print(F("Current Time:"));
               t = Time.now();
               Serial.println(Time.timeStr(t)); // Prints time t - example: Wed May 21 01:08:47 2014  // Give and take the bus are in this function as it gets the current time
               stateOfCharge = batteryMonitor.getSoC();
               Serial.print(F("State of charge: "));
               Serial.print(stateOfCharge);
               Serial.println(F("%"));
               Serial.print(F("Sensitivity set to: "));
               Serial.println(10-FRAMread8(SENSITIVITYADDR));
               Serial.print(F("Debounce set to: "));
               Serial.println(FRAMread8(DEBOUNCEADDR)*10);        // We mulitply by 10 as debounce is stored in 100ths
               Serial.print(F("Hourly count: "));
               Serial.println(FRAMread16(CURRENTHOURLYCOUNTADDR));
               Serial.print(F("Daily count: "));
               Serial.println(FRAMread16(CURRENTDAILYCOUNTADDR));
               printSignalStrength();
               Serial.print("Temperature in case: ");
               Serial.print(getTemperature(0)); // Returns temp in F
               Serial.println(" degrees F");
               break;
           case '2':     // Set the time zone - to be implemented
               break;
           case '3':  // Change the sensitivity
               Serial.println(F("Enter 0 (least) to 10 (most)"));
               while (Serial.available() == 0) {  // Look for char in serial queue and process if found
                   continue;
               }
               accelInputValue = (Serial.parseInt());
               Serial.print(F("accelSensitivity set to: "));
               Serial.println(accelInputValue);
               accelSensitivity = 10-accelInputValue;
               FRAMwrite8(SENSITIVITYADDR, accelSensitivity);
               initMMA8452(accelFullScaleRange, dataRate);  // init the accelerometer if communication is OK
               Serial.print(F("Accelsensitivity = "));
               Serial.print(accelSensitivity);
               Serial.println(F(" MMA8452Q is online..."));
               break;
           case '4':  // Change the debounce value
               Serial.println(F("Enter debounce in mSec"));
               while (Serial.available() == 0) {  // Look for char in serial queue and process if found
                   continue;
               }
               debounce = Serial.parseInt();
               if (debounce > delaySleep) delaySleep = debounce;       // delaySleep must be bigger than debounce afterall
               Serial.print(F("Debounce set to: "));
               Serial.println(debounce);
               FRAMwrite8(DEBOUNCEADDR, debounce/10);     // Remember we store debounce in cSec
               break;
           case '5':  // Reset the current counters
               Serial.println(F("Counter Reset!"));
               FRAMwrite16(CURRENTDAILYCOUNTADDR, 0);   // Reset Daily Count in memory
               FRAMwrite16(CURRENTHOURLYCOUNTADDR, 0);  // Reset Hourly Count in memory
               hourlyPersonCount = 0;
               dailyPersonCount = 0;
               Serial.println(F("Resetting Counters and Simblee"));
               break;
           case '6': // Reset FRAM Memory
               ResetFRAM();
               break;
           case '7':  // Start or stop the test
               if (inTest == 0) {
                   StartStopTest(1);
               }
               else {
                   StartStopTest(0);
                   refreshMenu = 1;
               }
               break;
           case '8':   // Dump the hourly data to the monitor
               numberHourlyDataPoints = FRAMread16(HOURLYPOINTERADDR); // Put this here to reduce FRAM reads
               Serial.print("Retrieving ");
               Serial.print(HOURLYCOUNTNUMBER);
               Serial.println(" hourly counts");
               Serial.println(F("Hour Ending -   Count  - Battery %"));
               for (int i=0; i < HOURLYCOUNTNUMBER; i++) { // Will walk through the hourly count memory spots - remember pointer is already incremented
                   unsigned int address = (HOURLYOFFSET + (numberHourlyDataPoints + i) % HOURLYCOUNTNUMBER)*WORDSIZE;
                   countTemp = FRAMread16(address+HOURLYCOUNTOFFSET);
                   if (countTemp > 0) {
                       time_t unixTime = FRAMread32(address);
                       Serial.print(Time.timeStr(unixTime));
                       Serial.print(F(" - "));
                       Serial.print(countTemp);
                       Serial.print(F("  -  "));
                       Serial.print(FRAMread8(address+HOURLYBATTOFFSET));
                       Serial.println(F("%"));
                   }
               }
               Serial.println(F("Done"));
               break;
           case '9':  // Download all the daily counts
               numberDailyDataPoints = FRAMread8(DAILYPOINTERADDR);        // Put this here to reduce FRAM reads
               Serial.println(F("Date - Count - Battery %"));
               for (int i=0; i < DAILYCOUNTNUMBER; i++) {                  // Will walk through the 30 daily count memory spots - remember pointer is already incremented
                   int address = (DAILYOFFSET + (numberDailyDataPoints + i) % DAILYCOUNTNUMBER)*WORDSIZE;      // Here to improve readabiliy - with Wrapping
                   countTemp = FRAMread16(address+DAILYCOUNTOFFSET);       // This, again, reduces FRAM reads
                   if (countTemp > 0) {                                    // Since we will step through all 30 - don't print empty results
                       Serial.print(FRAMread8(address));
                       Serial.print(F("/"));
                       Serial.print(FRAMread8(address+DAILYDATEOFFSET));
                       Serial.print(F(" - "));
                       Serial.print(countTemp);
                       Serial.print(F("  -  "));
                       Serial.print(FRAMread8(address+DAILYBATTOFFSET));
                       Serial.println(F("%"));
                   }
               }
               Serial.println(F("Done"));
               break;
           default:
               Serial.println(F("Invalid choice - try again"));
       }
       Serial.read();  // Clear the serial buffer
   }
   if (inTest == 1) {
     CheckForBump();
   }
 }

 void CheckForBump() // This is where we check to see if an interrupt is set when not asleep or act on a tap that woke the Arduino
 {
     if (digitalRead(int2Pin)==1)    // If int2 goes High, either p/l has changed or there's been a single/double tap
     {
       byte source = readRegister(MMA8452_ADDRESS,0x0C);  // Read the interrupt source reg.
       readRegister(MMA8452_ADDRESS,0x22);  // Reads the PULSE_SRC register to reset it
       if ((source & 0x08)==0x08 && millis() >= lastBump + debounce)  // We are only interested in the TAP register and ignore debounced taps
       {
         Serial.println(F("It is a tap - counting"));
         lastBump = millis();    // Reset last bump timer
         t = Time.now();
         if (HOURLYPERIOD != currentHourlyPeriod) {
             LogHourlyEvent();
         }
         if (DAILYPERIOD != currentDailyPeriod) {
             LogDailyEvent();
         }
         hourlyPersonCount++;                    // Increment the PersonCount
         FRAMwrite16(CURRENTHOURLYCOUNTADDR, hourlyPersonCount);  // Load Hourly Count to memory
         dailyPersonCount++;                    // Increment the PersonCount
         FRAMwrite16(CURRENTDAILYCOUNTADDR, dailyPersonCount);   // Load Daily Count to memory
         FRAMwrite32(CURRENTCOUNTSTIME, t);   // Write to FRAM - this is so we know when the last counts were saved
         Serial.print(F("Hourly: "));
         Serial.print(hourlyPersonCount);
         Serial.print(F(" Daily: "));
         Serial.print(dailyPersonCount);
         Serial.print(F("  Time: "));
         Serial.println(Time.timeStr(t)); // Prints time t - example: Wed May 21 01:08:47 2014
         ledState = !ledState;              // toggle the status of the LEDPIN:
         digitalWrite(redLED, ledState);    // update the LED pin itself
      }
         else if (millis() < lastBump + debounce) {
             Serial.print(F("Tap was debounced - lastBump = "));
             Serial.print(lastBump);
             Serial.print(F(" debounce = "));
             Serial.print(debounce);
             Serial.print(F(" millis() = "));
             Serial.println(millis());
         }
         else if ((source & 0x08) != 0x08) Serial.println(F("Interrupt not a tap"));
     }
 }


 void StartStopTest(boolean startTest)  // Since the test can be started from the serial menu or the Simblee - created a function
 {
     if (startTest) {
         inTest = true;
         t = Time.now();                    // Gets the current time
         currentHourlyPeriod = HOURLYPERIOD;   // Sets the hour period for when the count starts (see #defines)
         currentDailyPeriod = DAILYPERIOD;     // And the day  (see #defines)
         // Deterimine when the last counts were taken check when starting test to determine if we reload values or start counts over
         time_t unixTime = FRAMread32(CURRENTCOUNTSTIME);
         lastHour = Time.hour(unixTime);
         lastDate = Time.day(unixTime);
         dailyPersonCount = FRAMread16(CURRENTDAILYCOUNTADDR);  // Load Daily Count from memory
         hourlyPersonCount = FRAMread16(CURRENTHOURLYCOUNTADDR);  // Load Hourly Count from memory
         if (currentDailyPeriod != lastDate) {
             LogHourlyEvent();
             LogDailyEvent();
         }
         else if (currentHourlyPeriod != lastHour) {
             LogHourlyEvent();
         }
         readRegister(MMA8452_ADDRESS,0x22);     // Reads the PULSE_SRC register to reset it
         Serial.println(F("Test Started"));
     }
     else {
         inTest = false;
         readRegister(MMA8452_ADDRESS,0x22);  // Reads the PULSE_SRC register to reset it
         t = Time.now();
         FRAMwrite16(CURRENTDAILYCOUNTADDR, dailyPersonCount);   // Load Daily Count to memory
         FRAMwrite16(CURRENTHOURLYCOUNTADDR, hourlyPersonCount);  // Load Hourly Count to memory
         FRAMwrite32(CURRENTCOUNTSTIME, t);   // Write to FRAM - this is so we know when the last counts were saved
         hourlyPersonCount = 0;        // Reset Person Count
         dailyPersonCount = 0;         // Reset Person Count
         Serial.println(F("Test Stopped"));
     }
 }

 void LogHourlyEvent() // Log Hourly Event()
 {
     time_t LogTime = FRAMread32(CURRENTCOUNTSTIME);     // This is the last event recorded - this sets the hourly period
     unsigned int pointer = (HOURLYOFFSET + FRAMread16(HOURLYPOINTERADDR))*WORDSIZE;  // get the pointer from memory and add the offset
     LogTime -= (60*Time.minute(LogTime) + Time.second(LogTime)); // So, we need to subtract the minutes and seconds needed to take to the top of the hour
     FRAMwrite32(pointer, LogTime);   // Write to FRAM - this is the end of the period
     FRAMwrite16(pointer+HOURLYCOUNTOFFSET,hourlyPersonCount);
     stateOfCharge = batteryMonitor.getSoC();
     FRAMwrite8(pointer+HOURLYBATTOFFSET,stateOfCharge);
     unsigned int newHourlyPointerAddr = (FRAMread16(HOURLYPOINTERADDR)+1) % HOURLYCOUNTNUMBER;  // This is where we "wrap" the count to stay in our memory space
     FRAMwrite16(HOURLYPOINTERADDR,newHourlyPointerAddr);
     // Take the temperature and report to Ubidots - may set up custom webhooks later
     float currentTemp = getTemperature(0);  // 0 argument for degrees F
     i=0;         // Reset the pointer for responses from Ubidots
     stateOfCharge = batteryMonitor.getSoC();
     String data = String::format("{\"hourly\":%i, \"battery\":%.1f, \"temp\":%.1f}",hourlyPersonCount, stateOfCharge, currentTemp);
     Particle.publish("hourly", data, PRIVATE);
     hourlyPersonCount = 0;               // Reset and increment the Person Count in the new period
     currentHourlyPeriod = HOURLYPERIOD;  // Change the time period
     Serial.println(F("Hourly Event Logged"));
 }

 void LogDailyEvent() // Log Daily Event()
 {
     time_t LogTime = FRAMread32(CURRENTCOUNTSTIME);// This is the last event recorded - this sets the daily period
     int pointer = (DAILYOFFSET + FRAMread8(DAILYPOINTERADDR))*WORDSIZE;  // get the pointer from memory and add the offset
     FRAMwrite8(pointer,Time.month(LogTime)); // The month of the last count
     FRAMwrite8(pointer+DAILYDATEOFFSET,Time.day(LogTime));  // Write to FRAM - this is the end of the period  - should be the day
     FRAMwrite16(pointer+DAILYCOUNTOFFSET,dailyPersonCount);
     stateOfCharge = batteryMonitor.getSoC();
     FRAMwrite8(pointer+DAILYBATTOFFSET,stateOfCharge);
     byte newDailyPointerAddr = (FRAMread8(DAILYPOINTERADDR)+1) % DAILYCOUNTNUMBER;  // This is where we "wrap" the count to stay in our memory space
     FRAMwrite8(DAILYPOINTERADDR,newDailyPointerAddr);
     i=0;         // Reset the pointer for responses from Ubidots
     stateOfCharge = batteryMonitor.getSoC();
     String data = String::format("{\"daily\":%i, \"hourly\":%i, \"battery\":%.1f}",dailyPersonCount, hourlyPersonCount, stateOfCharge);
     Particle.publish("daily", data, PRIVATE);
     dailyPersonCount = 0;    // Reset and increment the Person Count in the new period
     currentDailyPeriod = DAILYPERIOD;  // Change the time period
     Serial.println(F("Logged a Daily Event"));
 }

 // Initialize the MMA8452 registers
 // See the many application notes for more info on setting all of these registers:
 // http://www.nxp.com/products/sensors/accelerometers/3-axis-accelerometers/2g-4g-8g-low-g-12-bit-digital-accelerometer:MMA8452Q?tab=Documentation_Tab
 // Feel free to modify any values, these are settings that work well for me.
 void initMMA8452(byte fsr, byte dataRate)
 {
   MMA8452Standby();  // Must be in standby to change registers
   // Set up the full scale range to 2, 4, or 8g.
   if ((fsr==2)||(fsr==4)||(fsr==8))
     writeRegister(MMA8452_ADDRESS, 0x0E, fsr >> 2);
   else
     writeRegister(MMA8452_ADDRESS,0x0E, 0);
   // Setup the 3 data rate bits, from 0 to 7
   writeRegister(MMA8452_ADDRESS, 0x2A, readRegister(MMA8452_ADDRESS,0x2A) & ~(0x38));
   if (dataRate <= 7)
     writeRegister(MMA8452_ADDRESS,0x2A, readRegister(MMA8452_ADDRESS,0x2A) | (dataRate << 3));

   /* Set up single and double tap - 5 steps:
    1. Set up single and/or double tap detection on each axis individually.
    2. Set the threshold - minimum required acceleration to cause a tap.
    3. Set the time limit - the maximum time that a tap can be above the threshold
    4. Set the pulse latency - the minimum required time between one pulse and the next
    5. Set the second pulse window - maximum allowed time between end of latency and start of second pulse
    for more info check out this app note: http://www.nxp.com/assets/documents/data/en/application-notes/AN4072.pdf */
   writeRegister(MMA8452_ADDRESS, 0x21, 0x55);           // 1. single taps only on all axes
   writeRegister(MMA8452_ADDRESS, 0x23, accelSensitivity);    // 2. x thresh at 2g (0x20), multiply the value by 0.0625g/LSB to get the threshold
   writeRegister(MMA8452_ADDRESS, 0x24, accelSensitivity);    // 2. y thresh at 2g (0x20), multiply the value by 0.0625g/LSB to get the threshold
   writeRegister(MMA8452_ADDRESS, 0x25, accelSensitivity);    // 2. z thresh at .5g (0x08), multiply the value by 0.0625g/LSB to get the threshold
   writeRegister(MMA8452_ADDRESS, 0x26, 0xFF);           // 3. Max time limit as this is very dependent on data rate, see the app note
   writeRegister(MMA8452_ADDRESS, 0x27, 0x64);           // 4. 1000ms (at 100Hz odr, Normal, and LPF Disabled) between taps min, this also depends on the data rate
   writeRegister(MMA8452_ADDRESS, 0x28, 0xFF);           // 5. Mmax value between taps max
   writeRegister(MMA8452_ADDRESS, 0x2C, 0x02);           // Active high, push-pull interrupts
   writeRegister(MMA8452_ADDRESS, 0x2D, 0x08);           // Tap ints enabled
   writeRegister(MMA8452_ADDRESS, 0x2E, 0xB7);           // Taps on INT2 everyhting else to INT1
   MMA8452Active();  // Set to active to start reading
 }

 // Sets the MMA8452 to standby mode.
 // It must be in standby to change most register settings
 void MMA8452Standby()
 {
   byte c = readRegister(MMA8452_ADDRESS,0x2A);
   writeRegister(MMA8452_ADDRESS,0x2A, c & ~(0x01));
 }

 // Sets the MMA8452 to active mode.
 // Needs to be in this mode to output data
 void MMA8452Active()
 {
   byte c = readRegister(MMA8452_ADDRESS,0x2A);
   writeRegister(MMA8452_ADDRESS,0x2A, c | 0x01);
 }

 // Read a single byte from address and return it as a byte
 byte readRegister(int I2CAddress, byte address)
 {
   //Send a request
   //Start talking to the device at the specified address
   Wire.beginTransmission(I2CAddress);
   //Send a bit asking for requested register address
   Wire.write(address);
   //Complete Transmission
   Wire.endTransmission(false);
   //Read the register from the device
   //Request 1 Byte from the specified address
   Wire.requestFrom(I2CAddress, 1);
   //wait for response
   while(Wire.available() == 0);
   // Get the temp and read it into a variable
   byte data = Wire.read();
   return data;
 }

 // Writes a single byte (data) into address
 void writeRegister(int I2CAddress, unsigned char address, unsigned char data)
 {
   //Send a request
   //Start talking to the device at the specified address
   Wire.beginTransmission(I2CAddress);
   //Send a bit asking for requested register address
   Wire.write(address);
   Wire.write(data);
   //Complete Transmission
   Wire.endTransmission(false);
 }

// Begin section
uint8_t FRAMread8(unsigned int address)  // Read 8 bits from FRAM
{
    uint8_t result;
    //Serial.println("In FRAMread8");
    result = fram.read8(address);
    return result;
}

void FRAMwrite8(unsigned int address, uint8_t value)    // Write 8 bits to FRAM
{
    fram.write8(address,value);
}

int FRAMread16(unsigned int address)
{
    long two;
    long one;
    //Read the 2 bytes from  memory.
    two = fram.read8(address);
    one = fram.read8(address + 1);
    //Return the recomposed long by using bitshift.
    return ((two << 0) & 0xFF) + ((one << 8) & 0xFFFF);
}

void FRAMwrite16(unsigned int address, int value)   // Write 16 bits to FRAM
{
    //This function will write a 2 byte (16bit) long to the eeprom at
    //the specified address to address + 1.
    //Decomposition from a long to 2 bytes by using bitshift.
    //One = Most significant -> Four = Least significant byte
    byte two = (value & 0xFF);
    byte one = ((value >> 8) & 0xFF);
    //Write the 2 bytes into the eeprom memory.
    fram.write8(address, two);
    fram.write8(address + 1, one);
}

unsigned long FRAMread32(unsigned long address)
{
    long four;
    long three;
    long two;
    long one;
    //Read the 4 bytes from memory.
    four = fram.read8(address);
    three = fram.read8(address + 1);
    two = fram.read8(address + 2);
    one = fram.read8(address + 3);
    //Return the recomposed long by using bitshift.
    return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}

void FRAMwrite32(int address, unsigned long value)  // Write 32 bits to FRAM
{
    //This function will write a 4 byte (32bit) long to the eeprom at
    //the specified address to address + 3.
    //Decomposition from a long to 4 bytes by using bitshift.
    //One = Most significant -> Four = Least significant byte
    byte four = (value & 0xFF);
    byte three = ((value >> 8) & 0xFF);
    byte two = ((value >> 16) & 0xFF);
    byte one = ((value >> 24) & 0xFF);
    //Write the 4 bytes into the eeprom memory.
    fram.write8(address, four);
    fram.write8(address + 1, three);
    fram.write8(address + 2, two);
    fram.write8(address + 3, one);
}


void ResetFRAM()  // This will reset the FRAM - set the version and preserve delay and sensitivity
{
    // Note - have to hard code the size here due to this issue - http://www.microchip.com/forums/m501193.aspx
    Serial.println("Resetting Memory");
    for (unsigned long i=4; i < 32768; i++) {  // Start at 4 to not overwrite debounce and sensitivity
        FRAMwrite8(i,0x0);
        if (i==8192) Serial.println(F("25% done"));
        if (i==16384) Serial.println(F("50% done"));
        if (i==(24576)) Serial.println(F("75% done"));
        if (i==32767) Serial.println(F("Done"));
    }
    FRAMwrite8(VERSIONADDR,VERSIONNUMBER);  // Reset version to match #define value for sketch
}


void NonBlockingDelay(int millisDelay)  // Used for a non-blocking delay
{
    unsigned long commandTime = millis();
    while (millis() <= millisDelay + commandTime) { }
    return;
}

void BlinkForever() // When something goes badly wrong...
{
    Serial.println(F("Error - Reboot"));
    while(1) {
        digitalWrite(redLED,HIGH);
        delay(200);
        digitalWrite(redLED,LOW);
        delay(200);
    }
}

void myHandler(const char *event, const char *data)
{
  i++;
  Serial.print(i);
  Serial.print(event);
  Serial.print(", data: ");
  if (data)
    Serial.println(data);
  else
    Serial.println("NULL");
}

void printSignalStrength()
{
  CellularSignal sig = Cellular.RSSI();  // Prototype for Cellular Signal Montoring
  String s = String(sig.rssi) + String(",") + String(sig.qual);
  Serial.print("The signal strength is: ");
  Serial.println(sig);
}

float getTemperature(bool degC)
{
  //getting the voltage reading from the temperature sensor
  int reading = analogRead(tmp36Pin);

  // converting that reading to voltage, for 3.3v arduino use 3.3
  float voltage = reading * 3.3;
  voltage /= 4096.0;        // This is different than the Arduino where there are only 1024 steps

  // now print out the temperature
  float temperatureC = ((voltage - 0.5) * 100) - 5 ;  //converting from 10 mv per degree with 500 mV offset to degrees ((voltage - 500mV) times 100) - 5 degree calibration
  // now convert to Fahrenheit
  float temperatureF = (temperatureC * 9.0 / 5.0) + 32.0;

  if (degC)
  {
      return temperatureC;
  }
  else
  {
      return temperatureF;
  }
}
