/*
 * Project Cellular-Logger-MMA8452Q
 * Description: Cellular Connected Data Logger
 * Author: Chip McClelland
 * Date:18 May 2017
 */

 //Time Period Definitions - used for debugging
 #define HOURLYPERIOD hour(t)   // Normally hour(t) but can use minute(t) for debugging
 #define DAILYPERIOD day(t) // Normally day(t) but can use minute(t) or hour(t) for debugging

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
 // i2c Address for the accelerometer
 #define MMA8452_ADDRESS 0x1D  // Our Accelerometer I2C Address set with open jumper SA0

 // Included Libraries
 #include "Adafruit_FRAM_I2C.h"  // Library for FRAM functions

 // Prototypes
 Adafruit_FRAM_I2C fram = Adafruit_FRAM_I2C(); // Init the FRAM

 // Pin Constants
 const int int2Pin = D2;

 // Program Constants
 const byte SCALE = 2;          // Sets full-scale range to +/-2, 4, or 8g. Used to calc real g values.
 int InputValue = 0;            // Raw sensitivity input
 byte Sensitivity = 0x01;       // Hex variable for sensitivity

 // Variables for the control byte
 // Control Register  (8 - 7 Reserved, 6 - Simblee Reset, 5-Clear Counts, 4-Simblee Sleep, 3-Start / Stop Test, 2-Set Sensitivity, 1-Set Delay)
 byte signalDebounceChange = 0b00000001;      // These are the bit masks to set and clear the control register bits
 byte clearDebounceChange = 0b11111110;
 byte signalSentitivityChange = 0b00000010;
 byte clearSensitivityChange = 0b11111101;
 byte toggleStartStop = 0b00000100;
 byte toggleSimbleeSleep = 0b00001000;
 byte signalClearCounts = 0b00010000;
 byte clearClearCounts = 0b11101111;
 byte controlRegisterValue;                  // Holds the current control register value
 byte oldControlRegisterValue;               // Makes sure we can detect a change in the register value
 unsigned long lastCheckedControlRegister;   // When did we last check the control register
 int controlRegisterDelay = 1000;            // Ho often will we check the control register


 // Accelerometer Variables
 const byte accelFullScaleRange = 2;  // Sets full-scale range to +/-2, 4, or 8g. Used to calc real g values.
 const byte dataRate = 3;             // output data rate - 0=800Hz, 1=400, 2=200, 3=100, 4=50, 5=12.5, 6=6.25, 7=1.56
 byte accelInputValue = 1;            // Raw sensitivity input (0-9);
 byte accelSensitivity;               // Hex variable for sensitivity
 byte accelThreshold = 100;           // accelThreshold value to decide when the detected sound is a knock or not
 unsigned int debounce;               // This is a minimum debounce value - additional debounce set using pot or remote terminal


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
     Wire.begin();                //Create a Wire object
     pinMode(int2Pin,INPUT);
     if (fram.begin()) {  // you can stick the new i2c addr in here, e.g. begin(0x51);
         Serial.println(F("Found I2C FRAM"));
     } else {
         Serial.println(F("No I2C FRAM found ... check your connections"));
     }
     // Import the accelSensitivity and Debounce values from memory
     Serial.print(F("Sensitivity set to: "));
     accelSensitivity = FRAMread8(10-SENSITIVITYADDR);
     Serial.println(accelSensitivity);
     Serial.print(F("Debounce set to: "));
     debounce = FRAMread8(DEBOUNCEADDR)*10;     // We mulitply by ten since debounce is stored in 100ths of a second
     if (debounce > delaySleep) delaySleep = debounce;       // delaySleep must be bigger than debounce afterall
     Serial.println(debounce);

     FRAMwrite8(CONTROLREGISTER, toggleStartStop);       // Reset the control register and start the test

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
 }

 void loop() {
     if (digitalRead(int2Pin)) {
         Serial.print("Tap Pin is High");
         Serial.print(" INTR_REG: ");
         Serial.print(readRegister(MMA8452_ADDRESS,0x0C));  // Reads the INT_SRC register to reset it
         Serial.print("TAP_REG: ");
         Serial.println(readRegister(MMA8452_ADDRESS,0x22));  // Reads the PULSE_SRC register to reset it
         bool success;
         success = Particle.publish("motion-detected");
         if (!success) {
         Serial.println("Did not publish");// get here if event publish did not work
         }
     }
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
