/*
 Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.

 03/17/2013 : Charles-Henri Hallard (http://hallard.me)
              Modified to use with Arduipi board http://hallard.me/arduipi
						  Changed to use modified bcm2835 and RF24 library
TMRh20 2014 - Updated to work with optimized RF24 Arduino library

 */

/**
 * Example RF Radio Ping Pair
 *
 * This is an example of how to use the RF24 class on RPi, communicating to an Arduino running
 * the GettingStarted sketch.
 */

#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <unistd.h>
#include <RF24/RF24.h>
#include <fstream>
#include <cmath>


using namespace std;
typedef unsigned char byte;
//
// Hardware configuration
// Configure the appropriate pins for your connections

/****************** Raspberry Pi ***********************/

// Radio CE Pin, CSN Pin, SPI Speed
// See http://www.airspayce.com/mikem/bcm2835/group__constants.html#ga63c029bd6500167152db4e57736d0939 and the related enumerations for pin information.

// Setup for GPIO 22 CE and CE0 CSN with SPI Speed @ 4Mhz
//RF24 radio(RPI_V2_GPIO_P1_22, BCM2835_SPI_CS0, BCM2835_SPI_SPEED_4MHZ);

// NEW: Setup for RPi B+
//RF24 radio(RPI_BPLUS_GPIO_J8_15,RPI_BPLUS_GPIO_J8_24, BCM2835_SPI_SPEED_8MHZ);

// Setup for GPIO 15 CE and CE0 CSN with SPI Speed @ 8Mhz
//RF24 radio(RPI_V2_GPIO_P1_15, RPI_V2_GPIO_P1_24, BCM2835_SPI_SPEED_8MHZ);

// RPi generic:
RF24 radio(22,0);

/*** RPi Alternate ***/
//Note: Specify SPI BUS 0 or 1 instead of CS pin number.
// See http://tmrh20.github.io/RF24/RPi.html for more information on usage

//RPi Alternate, with MRAA
//RF24 radio(15,0);

//RPi Alternate, with SPIDEV - Note: Edit RF24/arch/BBB/spi.cpp and  set 'this->device = "/dev/spidev0.0";;' or as listed in /dev
//RF24 radio(22,0);


/****************** Linux (BBB,x86,etc) ***********************/

// See http://tmrh20.github.io/RF24/pages.html for more information on usage
// See http://iotdk.intel.com/docs/master/mraa/ for more information on MRAA
// See https://www.kernel.org/doc/Documentation/spi/spidev for more information on SPIDEV

// Setup for ARM(Linux) devices like BBB using spidev (default is "/dev/spidev1.0" )
//RF24 radio(115,0);

//BBB Alternate, with mraa
// CE pin = (Header P9, Pin 13) = 59 = 13 + 46 
//Note: Specify SPI BUS 0 or 1 instead of CS pin number. 
//RF24 radio(59,0);

/********** User Config *********/

enum command { NoCMD, Stop, Get };

#define BROAD_CH    0
#define PRIVATE_CH  10

// Data structures
struct dataPackage
{
  unsigned short int slaveID;
  unsigned short int pkID;
  short int someData1;
  short int someData2;
  short int someData3;
};
typedef struct dataPackage DataPackage;

/********************************/

// Radio pipe addresses for the 2 nodes to communicate.
const uint8_t pipes[][6] = {"1Node","2Node"};

// Interfaces
void switchToPrivateCH();
void switchToBroadcastCH();
bool waitForDataTO();
int getMean(int sampleMean[], int samples);
void service();
void log(unsigned long time, int slaveID, string text,int value);
void standard_deviation(float &mean, float &std, int data[], int n);


int main(int argc, char** argv){

//  bool role_ping_out = true, role_pong_back = false;
//  bool role = role_pong_back;

  cout << "RF24/examples/GettingStarted/\n";

  // Setup and configure rf radio
  radio.begin();

  // optionally, increase the delay between retries & # of retries
  radio.setRetries(15,15);
  
  // Set channel
  switchToBroadcastCH();
  
  // Dump the configuration of the rf unit for debugging
  radio.printDetails();


/********* Role chooser ***********/
/*
  printf("\n ************ Role Setup ***********\n");
  string input = "";
  char myChar = {0};
  cout << "Choose a role: Enter 0 for pong_back, 1 for ping_out (CTRL+C to exit) \n>";
  getline(cin,input);

  if(input.length() == 1) {
	myChar = input[0];
	if(myChar == '0'){
		cout << "Role: Pong Back, awaiting transmission " << endl << endl;
	}else{  cout << "Role: Ping Out, starting transmission " << endl << endl;
		role = role_ping_out;
	}
  }
  */
/***********************************/
	
	
	
	// Start pipes
    radio.openWritingPipe(pipes[1]);
    radio.openReadingPipe(1,pipes[0]);
	
	radio.startListening();
	
	// forever loop
	while (1)
	{
		// Wait for request
		if ( radio.available() )
		{
			
			// Fetch the last request
			char got_req;
			while(radio.available()){
				radio.read( &got_req, sizeof(char) );
			}
			
			printf("Got request: %i... Sending ack... Changing channel...\n",got_req);
			
			radio.stopListening();
			
			// Send ack 
			radio.write( &got_req, sizeof(char) );
			
			// Change channel
			switchToPrivateCH();
			
			// Wait for the other side to clear buffer
			delay(100);
			
			// Clear buffer
			byte someCrap;
			while(radio.available()){ radio.read( &someCrap, sizeof(someCrap) ); }
			
			service();
	
			// Change channel
			switchToBroadcastCH();
			radio.startListening();

		}

		
		delay(100); // release pressure from CPU

	} // forever loop

  return 0;
}

void service() {
	
	// Max service time for TO
	unsigned long maxServiceTime = millis() + 10*60*1000; // 10 min
	
	// Initialise arrays for service
	int serviceDataSize = 16;
	int serviceDataCnt = 0;
	int serviceLight[serviceDataSize] = {};
	int serviceTemp[serviceDataSize] = {};
	int serviceMoist[serviceDataSize] = {};
	
	// Initialise service debug
	unsigned long printPeriod = 1*1000; 	// 1 sec
	unsigned long printNext = millis() + printPeriod;
	
	// Take a mean of every data sampled beetween service
	int samples = 0;
	int sampleMeanBuffSize = 50;
	int sampleMeanLight[sampleMeanBuffSize] = {};
	int sampleMeanTemp[sampleMeanBuffSize] = {};
	int sampleMeanMoist[sampleMeanBuffSize] = {};
	
	// Take log of the first data when enter service
	bool logData = true;
	
	// Set up som time variables
	unsigned long serviceFrequency = 500;	
	unsigned long serviceNextTime = millis() + serviceFrequency;
	
	//int dataCnt = 30000;
	//for(int i = 1; i <= dataCnt; i++) {
	while (true) {
		// Send command
		command cmd = Get;
		radio.write( &cmd, sizeof(cmd) );
		radio.startListening();

		if ( waitForDataTO() ) {
			printf("Could not receive any data: FAILED!\n");
			radio.stopListening();
			break;
		} else {
			// Receive package
			DataPackage packageToSend;
			radio.read( &packageToSend, sizeof(packageToSend) );
			
			// Print raw data
			// printf("Data received::: Slave ID: %u, Light: %i, Temperature: %i, Moisture: %i\n",
			//	packageToSend.slaveID,packageToSend.someData1,packageToSend.someData2,packageToSend.someData3);
			
			// Put in mean array
			sampleMeanLight[samples] = packageToSend.someData1;
			sampleMeanTemp[samples] = packageToSend.someData2;
			sampleMeanMoist[samples] = packageToSend.someData3;
			
			// Increment samples
			samples = samples + 1;
			if (samples >= sampleMeanBuffSize) {
				printf("CRITICAL ERROR! samples was higher than buffer size\n");
			}
			
			// Do service here
			if (serviceNextTime < millis()) {
				// Save start time
				unsigned long serviceStartTime = millis();
				
				// Calculate mean of all sensor values
				int light = getMean(sampleMeanLight,samples);
				int temp =  getMean(sampleMeanTemp,samples);
				int moist = getMean(sampleMeanMoist,samples);
				// Load it to data array
				serviceLight[serviceDataCnt] = light;
				serviceTemp[serviceDataCnt] = temp;
				serviceMoist[serviceDataCnt] = moist;
				// Increment counter
				serviceDataCnt = serviceDataCnt + 1;
				if (serviceDataCnt >= serviceDataSize) {
					serviceDataCnt = 0;
				}
				
				// Log
				if (logData == true) {
					logData = false;
					log(serviceStartTime,packageToSend.slaveID,"light",light); 
					log(serviceStartTime,packageToSend.slaveID,"temp ",temp); 
					log(serviceStartTime,packageToSend.slaveID,"moist",moist); 
				}
				
				// Do other stuff

				
				
				// Check if anything needs to be printed
				if (millis() > printNext) {
					// Print info for this run!
					printf("Serviceing! Samples: %i\n",samples);
					printf("Mean: Light: %i, Temp: %i, Moist: %i\n"
						,light,temp,moist);
					
					// Print calculated stuff
					
					float mean;
					float std;
					standard_deviation(mean, std, serviceLight, serviceDataSize);
					printf("TEST! Mean: %.2f, STD: %.2f\n",mean,std);
						
						
					// Print next
					printNext = printNext + printPeriod;
				}
				
				// Next service in
				serviceNextTime = serviceNextTime + serviceFrequency;
				// Reset samples
				samples = 0;
			}
			
		}
		
		delay(20);
		radio.stopListening();
		
		if (maxServiceTime < millis()) {
			printf("WARNING: Max service time reached!\n"); 
			break;
		}
	}
	
	command cmd = Stop;
	radio.write( &cmd, sizeof(cmd) );
	radio.startListening();
}

void standard_deviation(float &mean, float &std, int data[], int n) {
    mean = 0.0;
	int sum_deviation=0.0;
    int i;
    for(i=0; i<n;++i)
    {
        mean+=data[i];
    }
    mean=mean/n;
    for(i=0; i<n;++i) {
		sum_deviation+=((float)data[i]-mean)*((float)data[i]-mean);
	}
    std = sqrt(sum_deviation/n);
}

void log(unsigned long time, int slaveID, string text, int value) {
	std::ofstream outfile;
	string logFile = "log_s_";
	std::string String = static_cast<ostringstream*>( &(ostringstream() << slaveID) )->str();
	logFile = logFile + String;
	char logFile_c[20];
	strcpy(logFile_c, logFile.c_str());
	FILE * pFileTXT;
	pFileTXT = fopen (logFile_c,"a");
	// Make text here
	fprintf(pFileTXT, "%lu, %i, %s, %i\n", time, slaveID, text.c_str(), value);
	// Close
	fclose (pFileTXT);
}

int getMean(int sampleMean[], int samples){
	int sum = 0;
	for (int i = 0; i < samples; i++) {
		sum = sum + sampleMean[i];
	}
	return sum/samples;
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