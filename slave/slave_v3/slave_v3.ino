
/*
* Getting Started example sketch for nRF24L01+ radios
* This is a very basic example of how to send data from one node to another
* Updated: Dec 2014 by TMRh20
*/ 

#include <SPI.h>
#include "RF24.h"


#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>


/****************** User Config ***************************/
/***      Set this radio as radio number 0 or 1         ***/
bool radioNumber;

/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8 */
RF24 radio(7,8);
/**********************************************************/

enum command { NoCMD, Stop, Get };

byte addresses[][6] = {"1Node","2Node"}; 

#define BROAD_CH    0
#define PRIVATE_CH  10

byte nodeNumber = 3;

// Data structures
struct dataPackage
{
  unsigned int slaveID;
  unsigned int pkID;
  int someData1;
  int someData2;
  int someData3;
};
typedef struct dataPackage DataPackage;
DataPackage packageToSend = {0,0,0,0};
unsigned int packageCnt = 0;

// Used to control whether this node is sending or receiving
bool role = 0;


//WATCHDOG
volatile int f_wdt=1;
 
ISR(WDT_vect) {
  if(f_wdt == 0) {
    f_wdt=1;
  } else {
    //Serial.println("WDT Overrun!!!");
  }
}
 
void enterSleep(void) {
  radio.powerDown();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   /* EDIT: could also use SLEEP_MODE_PWR_DOWN for lowest power consumption. */
  sleep_enable();
 
  /* Now enter sleep mode. */
  sleep_mode();
 
  /* The program will continue from here after the WDT timeout*/
  sleep_disable(); /* First thing to do is disable sleep. */
 
  /* Re-enable the peripherals. */
  power_all_enable();
  radio.powerUp();
}


void setup() {
  Serial.begin(115200);
  Serial.println(F("RF24/examples/GettingStarted"));
  Serial.println(F("*** PRESS 'T' to begin transmitting to the other node"));


  //WATCHDOG
  /* Clear the reset flag. */
  MCUSR &= ~(1<<WDRF);
  /* In order to change WDE or the prescaler, we need to
   * set WDCE (This will allow updates for 4 clock cycles).
   */
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  /* set new watchdog timeout prescaler value */
  WDTCSR = 1<<WDP0 | 1<<WDP3; /* 8.0 seconds */
  /* Enable the WD interrupt (note no reset). */
  WDTCSR |= _BV(WDIE);
  
  
  radio.begin();

  // Set the PA Level low to prevent power supply related issues since this is a
  // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  radio.setPALevel(RF24_PA_LOW);

  // Switch to broadcast channel
  switchToBroadcastCH();
  
  // Open a writing and reading pipe
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1,addresses[1]);

  // Get node number from ROM
  updateNodeNumber();
  
  // Start the radio listening for data
  radio.startListening();
}

void loop() {

  // Start sequence
  bool succesfulRun = false;
  
  // Send request on broad cast channel
  byte req = nodeNumber | 0xF0;
  radio.stopListening();
  radio.write(&req, sizeof(byte));   
  radio.startListening(); 
  Serial.print(F("Now sending: "));  
  Serial.print(req);     
  Serial.print(F("... Waiting for ack... "));     

  // Wait for ack
  bool privateSession = false;
  if (waitForDataTO()) {                                    
      Serial.println(F("Did not receive ack: FAILED!"));
  }else{
      byte ack = 0;            
      radio.read( &ack, sizeof(byte) );
      delay(20);
      if (req == ack) {
        Serial.println(F("Connection Established: SUCCESS!"));
        privateSession = true;
      } else {
        Serial.println(F("Received wrong ack: FAILED!"));
      }  
  }
  radio.stopListening();

  if(privateSession) {
    // Wait for host the change channel and clear buffer
    //delay(50); 
    // Change channel
    switchToPrivateCH();
    // Clear read buffer
    byte someCrap;
    while(radio.available()){ radio.read( &someCrap, sizeof(someCrap) ); }
    radio.startListening(); 

    while(privateSession) {
      // Wait for CMD 
      if (waitForDataTO()) {                                    
        Serial.println(F("Did not receive CMD: FAILED!"));
        privateSession = false;
        Serial.println(F("--- RESETTING ---"));
      }else{
        // Read command
        command cmd = NoCMD;
        radio.read( &cmd, sizeof(cmd) );
        Serial.print(F("Received CMD: "));
        Serial.print(cmd);
        Serial.print(F(" ... "));
        delay(20);

        // Test cmd
        switch (cmd) {
          case Stop:
            privateSession = false;
            succesfulRun = true;
            Serial.println(F("Stop message! Closing connection: SUCCESS!"));
          break;
          case Get:
            // Do nothing
            // Maybe update data
            // Read sensors
          break;
          default: 
            privateSession = false;
            Serial.println(F(" Wrong CMD received: FAILED!"));
          break;
        }

        if(privateSession) {
          // Set up data to send
          packageToSend.slaveID = nodeNumber;
          packageToSend.pkID = packageCnt;
          packageToSend.someData1 = readLight();
          packageToSend.someData2 = readTemp();
          packageToSend.someData3 = readMoisture();
          //packageToSend.someData1 = 1337;
          //packageToSend.someData2 = 12321;
          // Send data
          Serial.print(F("Sending data package: "));
          Serial.print(packageCnt);
          Serial.print(F(" ... "));
          radio.stopListening();
          if(!radio.write( &packageToSend, sizeof(packageToSend))) {
            Serial.println(F("FAILED!"));
          } else {
            Serial.println(F("SUCCESS!"));
            packageCnt++;
            Serial.print(F("Light: "));
            Serial.print(packageToSend.someData1);
            Serial.print(F(", "));
            Serial.print(F("Temperature: "));
            Serial.println(packageToSend.someData2);
          }
          radio.startListening();
        }
      }
    }

    // Change channel
    radio.stopListening();
    switchToBroadcastCH();
    radio.startListening(); 
  } 

  if (succesfulRun) {
    // Take a break
    /*
    f_wdt = 0;
    enterSleep();
    */
  
    delay(5000);
  } else {
    // Try again
    // Maybe wait longer if another slave node is serviced...
    delay(1000);
  }

  //Serial.println(F("okay"));
}  
  
/****************** Ping Out Role ***************************/  
/*
if (role == 1)  {
    
    radio.stopListening();                                    // First, stop listening so we can talk.
    
    
    Serial.println(F("Now sending"));

    unsigned long start_time = micros();                             // Take the time, and send it.  This will block until complete
     if (!radio.write( &start_time, sizeof(unsigned long) )){
       Serial.println(F("failed"));
     }
        
    radio.startListening();                                    // Now, continue listening
    
    unsigned long started_waiting_at = micros();               // Set up a timeout period, get the current microseconds
    boolean timeout = false;                                   // Set up a variable to indicate if a response was received or not
    
    while ( ! radio.available() ){                             // While nothing is received
      if (micros() - started_waiting_at > 200000 ){            // If waited longer than 200ms, indicate timeout and exit while loop
          timeout = true;
          break;
      }      
    }
        
    if ( timeout ){                                             // Describe the results
        Serial.println(F("Failed, response timed out."));
    }else{
        unsigned long got_time;                                 // Grab the response, compare, and send to debugging spew
        radio.read( &got_time, sizeof(unsigned long) );
        unsigned long end_time = micros();
        
        // Spew it
        Serial.print(F("Sent "));
        Serial.print(start_time);
        Serial.print(F(", Got response "));
        Serial.print(got_time);
        Serial.print(F(", Round-trip delay "));
        Serial.print(end_time-start_time);
        Serial.println(F(" microseconds"));
    }

    // Try again 1s later
    delay(1000);
   
  }
 */


/****************** Pong Back Role ***************************/
/*
  if ( role == 0 )
  {
    unsigned long got_time;
    
    if( radio.available()){
                                                                    // Variable for the received timestamp
      while (radio.available()) {                                   // While there is data ready
        radio.read( &got_time, sizeof(unsigned long) );             // Get the payload
      }
     
      radio.stopListening();                                        // First, stop listening so we can talk   
      radio.write( &got_time, sizeof(unsigned long) );              // Send the final one back.      
      radio.startListening();                                       // Now, resume listening so we catch the next packets.     
      Serial.print(F("Sent response "));
      Serial.println(got_time);  
   }
 }
*/



/****************** Change Roles via Serial Commands ***************************/
/*
  if ( Serial.available() )
  {
    char c = toupper(Serial.read());
    if ( c == 'T' && role == 0 ){      
      Serial.println(F("*** CHANGING TO TRANSMIT ROLE -- PRESS 'R' TO SWITCH BACK"));
      role = 1;                  // Become the primary transmitter (ping out)
    
   }else
    if ( c == 'R' && role == 1 ){
      Serial.println(F("*** CHANGING TO RECEIVE ROLE -- PRESS 'T' TO SWITCH BACK"));      
       role = 0;                // Become the primary receiver (pong back)
       radio.startListening();
       
    }
  }


} // Loop
*/

int readLight() {
  return analogRead(A1);
}

int readTemp() {
  return analogRead(A0);
}

int readMoisture() {
  return analogRead(A2);
}

void switchToPrivateCH() {
  // Set channel
  radio.setChannel(PRIVATE_CH);
  // Enable acknowledge
  radio.setAutoAck(true);
  //radio.setAutoAck(false);
}

void switchToBroadcastCH() {
  // Set channel
  radio.setChannel(BROAD_CH);
  // Disable acknowledge
  radio.setAutoAck(false);
}

void updateNodeNumber () {
  // This value should be save in ROM
  //nodeNumber = 1;
}

bool waitForDataTO() {
  // Wait here until we get a response, or timeout (250ms)
  unsigned long started_waiting_at = millis();
  bool timeout = false;
  while ( ! radio.available() && ! timeout ) {
    if (millis() - started_waiting_at > 500 ) {
      timeout = true;
    } 
  }
  return timeout;
}

