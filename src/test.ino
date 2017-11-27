
/*********************************************************************
 This is an example for our nRF51822 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"
#include <Wire.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!
#include <Adafruit_NeoPixel.h>



// =====================================================neo pixel setup =====================================================
#include <Adafruit_NeoPixel.h>

#define PIN 8
#define N_LEDS 144

Adafruit_NeoPixel strip = Adafruit_NeoPixel(1, PIN, NEO_GRB + NEO_KHZ800);


// ==========================================================================================================



/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         1
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"
    #define BLUEFRUIT_HWSERIAL_NAME      Serial1
    
    #define BLUEFRUIT_UART_CTS_PIN          -1   // Not used with FLORA
    #define BLUEFRUIT_UART_RTS_PIN          -1   // Not used with FLORA

/*=========================================================================*/

// Create the bluefruit object, either software //Serial..uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
Adafruit_BluefruitLE_UART ble(Serial1, BLUEFRUIT_UART_MODE_PIN);

// i2c
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0();
const int ledPin =  7;

int FREQ_RATE = 13;
const int size_mem = 70;
float x[size_mem], y[size_mem], z[size_mem], acc_vm[size_mem]; // can be changed during testing /// change this shit
int glob_count = 0;
int state, multi_peak_timer, sg_timer, peak_time;
float peak = 0, temp_peak = 0;
float accel_vm;
float temp_value;
unsigned long time1, time2, res_time;



// for testing purpose
//int test_counter = 0;
//float test_arr[] = {0.1,0.1,1.1,0.1,7.1,10.1,1.0,0.2,0.3,0.4,1.0,0.6,0.7,0.8,0.9,0.91,0.92,0.93};
                   //0 , 0 , 1 , 2 , 2 , 2 , 2 , 3 , 3 , 3,  
                   //1 , 2 , 3 , 4 , 5 , 6 , 7 , 8 , 9 , 10, 11, 12, 13, 14, 15


/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper

void error(const __FlashStringHelper*err) {
  //Serial.println(err);
  while (1);
}

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/

void setupSensor()
{
  // 1.) Set the accelerometer range
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
 
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}

void check_accel()
{
   pinMode(ledPin, OUTPUT);
 
  //#ifndef ESP8266
  //  while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
  //#endif
    Serial.begin(9600);
    Serial.println(F("LSM raw read demo"));

  // Try to initialise and warn if we couldn't detect the chip
    if (!lsm.begin())
    {
      digitalWrite(ledPin, LOW);
     Serial.println(F("Oops ... unable to initialize the LSM9DS0. Check your wiring!"));
      while (1);
    }
    else
    {
      digitalWrite(ledPin, HIGH);  
    }
  Serial.println(F("Found LSM9DS0 9DOF"));
  Serial.println("");
  Serial.println("");
}

void check_bluetooth()
{
  //while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(9600);
  Serial.println(F("Adafruit Bluefruit Command Mode Example"));
  Serial.println(F("---------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in UART mode"));
  Serial.println(F("Then Enter characters to send to Bluefruit"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("******************************"));
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
    Serial.println(F("******************************"));
  }
}


void setup()
{
 strip.begin();
 check_accel();
 check_bluetooth();
 //time1=millis();
}
void loop(void)
{
 
  //digitalWrite(ledPin, HIGH); 
  chase(strip.Color(0, 0, 255)); // Blue
  sensors_event_t accel, mag, gyro, temp;
  
  //while(!//Serialavailable) 
  lsm.getEvent(&accel, &mag, &gyro, &temp);
  accel_vm = sqrt(accel.acceleration.x*accel.acceleration.x + accel.acceleration.y*accel.acceleration.y + accel.acceleration.z*accel.acceleration.z);
  x[glob_count] = accel.acceleration.x;
  y[glob_count] = accel.acceleration.y;
  z[glob_count] = accel.acceleration.z;
  acc_vm[glob_count] = accel_vm;
  ////Serialprintln("accel_vm: ");
  ////Serialprintln(accel_vm);
  glob_count++;
  kav_mia();

  time2=millis();
  if ((time2-time1)<80) {
    delay(80-(time2-time1));
  }  else {
    ////Serialprintln("The process takes more time");
  }
  time1=millis();
}
void kav_mia(){
  
  switch(state){
    
    case 0:
    //collect data
      if (glob_count == (1*FREQ_RATE)){
        state = 1;
      }
    break;

    case 1:
      Serial.println("state 1");
      if (accel_vm > 1.6){
        //change state to State 2
        peak = accel_vm;
        state = 2;
        multi_peak_timer = (2*FREQ_RATE) - 1;
      }else{
        for(int i=1 ; i< glob_count; i++){
          //shift 1 sample left
          acc_vm[i-1] = acc_vm[i];
          
        }
        glob_count = glob_count -1 ;
      }
    break;

    case 2:
      Serial.println("state 2");
      temp_peak = 0;
      if (multi_peak_timer > 1){
        
        //search for the highest peak 
        if (accel_vm > peak){
          
          //find a higher peak
          
          peak = accel_vm;
          multi_peak_timer = (2*FREQ_RATE)-1;
          int tem_index = glob_count - ((1*FREQ_RATE)+1);
          //shift 
         
          for (int j = tem_index; j< glob_count; j++){
            acc_vm[j-tem_index] = acc_vm[j];         
          }          
          glob_count = glob_count - tem_index;
        }else{
          multi_peak_timer--;
        }
      }else{
        // change to state 3
        state = 3;
        sg_timer = (2*FREQ_RATE);
      }
     
    break;
    
    case 3:
      Serial.println("state 3");
      if (sg_timer>1){
        //check whether there is peak or not
     
        if (accel_vm > 1.6 && accel_vm > temp_peak){
          ////Serialprintln("found peak in state 3");
          peak_time = glob_count;
          temp_peak = accel_vm;
        }else{
          ////Serialprintln("no peak");
        }
        sg_timer --;     
      }else{
        //do feature extraction/ send the data
        send_data();
        if (temp_peak == 0){
          // go back to state 1 and shift to left only the buffer
          state = 1;
          int temp_ind_1 = 4*FREQ_RATE;
          for (int k = temp_ind_1; k< glob_count ; k++){
            acc_vm [k-temp_ind_1] = acc_vm[k]; 
          }

          glob_count = glob_count - temp_ind_1; //reset the global counter

        }else{
          // go to state 2
          state = 2;
          multi_peak_timer = (2*FREQ_RATE)-1;
          int temp_ind_2 = peak_time - (FREQ_RATE+1);
          for(int l = temp_ind_2 ; l < glob_count ; l++ ){
            acc_vm[l-temp_ind_2] = acc_vm[l];
          }
          glob_count = glob_count - temp_ind_2;
        }
      }
            
    break;

    default:
        Serial.println(F("State is not found"));
    break;
    
   }
}

void send_data(){
  float mq;
  //float mq[10];

  //memcpy(mq, acc_vm, 5*FREQ_RATE*sizeof(float));
  //strip.setPixelColor(1, strip.Color(255,0,0));
 Serial.println(F("printing the data : #####################"));
 for (int a = 0; a < 5*FREQ_RATE ; a++){
    mq = acc_vm[a];   
    ble.print("AT+BLEUARTTX=");   
    ble.println(mq);
    Serial.println(mq);

  }
  Serial.println(F("finish : #####################"));
}

static void chase(uint32_t c) {
  for(uint16_t i=0; i<strip.numPixels()+4; i++) {
      strip.setPixelColor(i , c); // Draw new pixel
      //strip.setPixelColor(i-4, 0); // Erase pixel a few steps back
      strip.show();
      //delay(25);
  }
}



