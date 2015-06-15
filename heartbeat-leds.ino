/*
 * Heartbeat LEDs project by John Keefe
 * For details on the LED wiring for this project, and more,
 * visit http://johnkeefe.net/make-every-week-heartbeat-leds
 * 
 * ---
 *
 * Base Heart Rate Monitor Interface (HRMI) code originally by 
 * Dan Julio: http://danjuliodesigns.com/sparkfun/hrmi.html
 *
 * Updated by Nick Charlton: https://github.com/nickcharlton/hmri_arduino 
 *
 * Simple Arduino-based program to read values from the HRMI using the I2C interface
 *
 * Connections
 *    Arduino            HRMI
 *    -----------------------
 *      +5                +5 (Power for the HRMI)
 *      GND               GND
 *      Analog In 4       RX (I2C SDA) (recommend 4.7 kOhm pullup)
 *      Analog In 5       TX (I2C SCL) (recommend 4.7 kOhm pullup)
 *
 *
 * Note: By default the Arduino Wiring library is limited to a maximum
 *       I2C read of 32 bytes.  The Get Heartrate command is limited
 *       by this code to a maximum of 30 values (for a max I2C packet
 *       of 32 bytes).
 * 
 * 
 */

#include "Wire.h"
#include "hrmi_funcs.h"

/*
 * Configuration Information
 *
 * Change these constants to match your specific configuration.  These
 * values are the factory default (no OP1-OP7 jumpers installed).  Jumper
 * OP0 should be installed and jumper SJ1 removed.
 *
 * HRMI_HOST_BAUDRATE should be set to the baudrate the host will use
 *   to communicate with the Arduino over the serial interface.
 *
 * HRMI_I2C_ADDR should be set to the I2C address the HRMI is configured
 *   with.
 */
#define HRMI_HOST_BAUDRATE 9600
#define HRMI_I2C_ADDR      127


/*
 * Program constants
 */
#define MAX_IN_BUFFSIZE 16


/*
 * Global variables
 */
char serInStr[MAX_IN_BUFFSIZE];   // Serial input string array
int numEntries = 3;               // Number of HR values to request
int numRspBytes;                  // Number of Response bytes to read
byte i2cRspArray[34];	          // I2C response array, sized to read 32 HR values
byte hrmi_addr = HRMI_I2C_ADDR;   // I2C address to use

// Variables for the led colors
int blue;
int green;
int yellow;
int blue_peak = 70;
int green_peak = 80;
int yellow_peak = 90;
int blue_led_pin = 9;
int green_led_pin = 10;
int yellow_led_pin = 11;
int red_led_pin = 8;
int max_bright = 100;
int heart_rate;
int heart_sum;


/*
 * Arduino initialization code segment
 */
void setup()
{
  // Initialize the I2C communication
  hrmi_open();

  // Initialize the serial interface
  Serial.begin(HRMI_HOST_BAUDRATE);
  
  // Set up LEDs
  pinMode(blue_led_pin, OUTPUT);
  pinMode(green_led_pin, OUTPUT);
  pinMode(yellow_led_pin, OUTPUT);  
  pinMode(red_led_pin, OUTPUT);  
}


/*
 * Arduino main code loop
 */
void loop()
{
  
  // flash the red light
  digitalWrite(red_led_pin, HIGH);
  delay(50);
  digitalWrite(red_led_pin, LOW);
  
  // Request a set of heart rate values
  hrmiCmdArg(hrmi_addr, 'G', (byte) numEntries);
      
  // Get the response from the HRMI
  numRspBytes = numEntries + 2;
  if (hrmiGetData(hrmi_addr, numRspBytes, i2cRspArray) != -1) {
    // send the results back on the serial interface in ASCII form
    Serial.print(" => ");
    heart_sum = 0;
    for (int i=0; i<numRspBytes; i++) {
      Serial.print(i2cRspArray[i], DEC);
      Serial.print(" ");
      
      // for the final reading this loop, add three most recent together
     //  to the sum to average later
      if (i > 1) {
        heart_sum += i2cRspArray[i];
      }
      
    }
    
    
    // i2cRspArray[2] is the latest heartbeat reading
    // calculating hyperbola based on this formula:
    // y = (-0.8 * x^2) + 100
    // see all of the arcs drawn here: http://jkef.me/1Gxj4vN
    
    // take the average of the readings
    heart_rate = heart_sum / numEntries;
    
    // if the rate is below the blue peak, keep it on full
    if (heart_rate < blue_peak) {
      blue = max_bright;
    } else {
      blue = (-0.8 * sq(heart_rate - blue_peak) ) + max_bright;
      if (blue < 0) {
        blue = 0;
      }
    }
    
    green = (-0.8 * sq(heart_rate - green_peak) ) + max_bright;
    if (green < 0) {
      green = 0;
    }
    
    if (heart_rate > yellow_peak) {
      yellow = max_bright;
    } else {
      yellow = (-0.8 * sq(heart_rate - yellow_peak) ) + max_bright;
      if (yellow < 0) { 
        yellow = 0;
      }
    }
    
    // light the leds at the calculated brighnesses
    analogWrite(blue_led_pin, blue);  
    analogWrite(green_led_pin, green);  
    analogWrite(yellow_led_pin, yellow);  

    // print to serial monitor 
    Serial.print("avg:"); 
    Serial.print(heart_rate);  
    Serial.print(" blue:");
    Serial.print(blue);
    Serial.print(" green:");
    Serial.print(green);
    Serial.print(" yellow:");
    Serial.print(yellow);
    
    Serial.println();
   }

  
  // Based on the heart rate, delay for the
  // number of milliseconds between beats.
  // So 60000 milliseconds/min / 70 beats/min 
  // = 800 millesecond delay
  if (heart_rate == 0) {
    // make sure there's a reading
    delay(1000);
  } else {
    delay(60000/heart_rate); 
  }   
}

