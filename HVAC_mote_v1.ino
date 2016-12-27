
///// LIBRARIES /////
#include <SPI.h>          // This library allows you to communicate with SPI devices, with the msp430 as the master device
#include <WiFi.h>         // Wifi connection library
#include <PubSubClient.h> // Mqtt protocol library http://pubsubclient.knolleary.net/api.html
#include "DHT.h"          // DHT sensor library
#include <aJSON.h>        // header file for creating and parsing JSON
#include <GFDS18B20.h>    // DS18B20 temperature sensor library
#include "MspFlash.h"     // Manipulate mspflash memory


///// Wifi & MQTT Connection & Topics /////
//char MQTT_srv[] = "mqtt.m2m.ce.teiep.gr";  // Mqtt Broker URL
//char MQTT_srv[] = "iot.eclipse.org";  // Mqtt Broker URL
char MQTT_srv_alt[] = "iot.eclipse.org"; // Mqtt alternate Broker URL
int port = 1883;     // Port number
char clientID[10];   // Client ID to connect to broker
char username[] = "m2mce";  // Username to connect to broker
char password[] = "mosq";   // Password to connect to broker

bool connected = false;        // true if connection is active (can be false due to MQTT problems
int  mqtt_failed_attempts=1;   // "0" indicates connected mqtt
#define max_mqtt_attempts_reboot 5      // After been connected, number of retries before reset/reboot



//TOPICS//
char Registration_request_topic[] = "m2mce/hvac/registration";
char Registration_request_topic_response[] = "m2mce/hvac/registration_response";
char HVAC_meas_topic[] = "m2mce/hvac/ce/meas";
char HVAC_cmd_topic[] = "m2mce/hvac/ce/act";
char HVAC_cmd_ack_topic[] = "m2mce/hvac/ce/act_ack";
char HVAC_reset_topic[] = "m2mce/hvac/reset";
char HVAC_alt_broker_topic[] = "m2mce/hvac/alarm";


WiFiClient wifiClient;   // the TCP client

void callback(char* topic, byte* payload, unsigned int length);   // callback function protype - otherwise next statement generates "out of scope" error
PubSubClient client(MQTT_srv, port, callback, wifiClient);        // PubSubClient (server, port, [callback], client, [stream])
PubSubClient client_alt(MQTT_srv_alt, port, callback, wifiClient);    // PubSubClient (alternate server, port, [callback], client, [stream])


long rssi = 0; // WIFI rssi(Received signal strength indication)
               // Attention!!! In order for rssi to work, it needs the commit from here: https://github.com/energia/Energia/commit/24b1824f7f3da7dec01dbedacff81b79876c880e
               // to the int32_t WiFiClass::RSSI() in the WiFi.cpp in the WiFi library!!! (copy/paste works perfect!!)


///// MSPFlash /////
//Use SEGMENT_B, SEGMENT_C or SEGMENT_D (each 64 bytes, 192 bytes in total)
#define flash SEGMENT_D
int pos = 0;


///// MAC Address and Node_ID/////
byte mac[10];                         // byte array for storing the mac address
char Node_ID[10];                     // global variable for storing the mac address
int Node_No = -1;                     // global variable for storing the Node Number instead of mac address NodeNo=-1 --> Unregistered Node


///// JSON message object placeholder/////
aJsonObject* sendJson;               // global variable for the root aJSON object to send to the server
aJsonObject* recvJson;               // global variable for the root aJSON object received from server
char* sendstring;                    // the string to hold the JSON object to send to server
char* recvstring;                    // the string to hold the JSON object to received from server


///// DS18B20 liquid temp sensor variables (according to GFDS18B20 library) /////
#define OWPIN  P6_5  // DS18B20 PINOUT 
#define MAXOW 10  //Max number of OW's used

byte ROMarray[MAXOW][8];
byte ROMtype[MAXOW];     // 28 for temp', 12 for switch etc.
byte ROMtemp[MAXOW];
byte result[MAXOW+5];

byte data[12];
byte i;
byte addr[8];
uint8_t ROMmax=0;
uint8_t ROMcount=0;
boolean foundOW =false;
byte HVAC_fluid_temp;          // global variable to store the temperature aquired from DS18B20
DS18B20 ds(OWPIN);             // currently on PIN P6_5


///// DHT11 sensor variables (according to DHT library) /////
// Uncomment whatever type you're using!
#define DHTTYPE DHT11          // DHT 11 
//#define DHTTYPE DHT22        // DHT 22  (AM2302)
//#define DHTTYPE DHT21        // DHT 21 (AM2301)

// Connect pin 1 (on the left) of the sensor to +3.3V or +5V
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

#define DHTPIN P3_7            // the data pin of the DHT connected to
DHT dht(DHTPIN, DHTTYPE);      // Initialize the DHT driver, currently on DHTPIN and DHTTYPE


///// RELAY control variables/////
#define HVAC_Unit_OnOff_PIN P3_6             // the pin controling the On/Off state of the fancoil
#define HVAC_Unit_fan_PIN P7_4               // the pin controling the Hi/Low state of the fancoil
boolean HVAC_Unit_OnOff = true;              // global variable to store the state of the pin
boolean HVAC_Unit_fan = true;                // global variable to store the state of the pin


///// GLOBAL VARIABLES /////
int msgCount=0;                  // to count the messages we send
#define LEDR RED_LED             // define red led as LEDR  
#define LEDG GREEN_LED           // define green led as LEDG
bool time4Meas = true;           // global boolean variable changing according to Minutes4Update
#define Minutes4Update 30000     // 120000 milliseconds = 2 minutes, 30000=30 sec to wait before sending the message to the MQTT broker
unsigned long elapsedtime;       // holds the last time a message send to the MQTT broker





///// INIT SETUP /////
void setup()
{


Serial.begin(9600); // Baud Rate for serial monitor
pinMode(LEDR, OUTPUT);        // configure Red Led as output
pinMode(LEDG, OUTPUT);        // configure Green Led as output
digitalWrite(LEDR, HIGH);     // TurnOn Red Led to indicate configuration process
digitalWrite(LEDG, HIGH);     // TurnOn Green Led to indicate configuration process
pinMode(HVAC_Unit_OnOff_PIN, OUTPUT);        // configure HVAC_Unit_OnOff_PIN as output
pinMode(HVAC_Unit_fan_PIN, OUTPUT);          // configure HVAC_Unit_fan_PIN as output
digitalWrite(HVAC_Unit_OnOff_PIN, HIGH);     // safe mode set to HIGH
digitalWrite(HVAC_Unit_fan_PIN, HIGH);       // safe mode set to HIGH

        Serial.println("Starting WiFi SmartConfig"); // Print in serial monitor
	WiFi.startSmartConfig(); //Enter SmartConfigMode
      
      while (WiFi.localIP() == INADDR_NONE) {
    // print dots while we wait for an ip addresss
    Serial.print(".");
    digitalWrite(LEDR, HIGH);            // configuration done turn on red led
    delay(300);
  }
  
	digitalWrite(LEDR, LOW);            // configuration done turn off led
	digitalWrite(LEDG, LOW);            // configuration done turn off led

     printWifiStatus();                       // we're connected now, so print in serial console the connection status
  
	 WiFi.macAddress(mac);            // read the mac address of the device ! Needed here to run every time or else somehow overwritten
	 sprintf(Node_ID, "%02X:%02X:%02X:%02X:%02X:%02X", mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]); // properly store in char array the mac as string
         sprintf(clientID, "%02X:%02X:%02X",mac[5], mac[4], mac[3]); //clientID as half mac address to connect to broker
           MQTTconnection();                                // call MQTTconnection function for connection to Mqtt Broker
          

	dht.begin();                           // initialize the DHT temperature sensor
	findOW();                             // find DS18B20 temperature sensor
	//displayOW();

	elapsedtime = millis();                // initialize the variable with current miliseconds passed

}//END of setup() function 



///// MAIN LOOP /////
void loop()
{
   // check from serial monitor if the user send 'e' and erase the flash memory 
  if ( Serial.available() )
  {
    switch ( Serial.read() )
    { 
      case 'e': doErase(); break;  
    }
  }
  
	if ( WiFi.status() == WL_CONNECTED ) {           // check if we have connection with the AP
              if ( client.connected() ){
                mqtt_failed_attempts=0;                     //Resets the wifi fail counts. "0" indicates connected
		connected=true;                            // Further tests are needed to verify IP and MQTT connectivity
              }   // when mqtt connection lost
              else{
              delay(3000);                                //slow down the loop before attempting recconect
	    mqtt_failed_attempts++;                     //increase the wifi fail count
	      MQTTconnection();                        // call MQTTconnection function for connection to Mqtt Broker
          
	    if (mqtt_failed_attempts > max_mqtt_attempts_reboot) {  //Decide to abort and reboot
                FAIL_SAFE();  // Turn off fan coil for safety
                  Alt_MQTT_Connection(); // Publish message to alternate_Broker that primary MQTT_Broker is down
                Serial.println("Reseting...");
	        WDTCTL = 0;                                        // zero out the WatchDog Timer so the device restarts
                } // END - if (mqtt_failed_attempts > max_mqtt_attempts_reboot)
              } // END - if ( client.connected() )
		
	}
	else{                                                  // when connection with AP lost
            FAIL_SAFE();  // Turn off fan coil for safety
	    Serial.println("Reseting...");
	        WDTCTL = 0;                                    // zero out the WatchDog Timer so the device restarts
              
        }    // END - if ( WiFi.status() == WL_CONNECTED)

  
	if (time4Meas && connected && Node_No!=-1){                  //Determine if it is time to measure/publish

	       HVAC_meas(); // Call HVAC_meas() function to publish measurments
	        time4Meas = false;                            // make the update variable false
	        elapsedtime = millis();                         // store the current time a message sent
	          rssi = WiFi.RSSI();                 // gets the received signal strength
	         //   Serial.print("signal strength (RSSI):"); // print the signal strength (RSSI): in serial monitor
	         //   Serial.print(rssi);                      // print the received signal strength in serial monitor
	         //   Serial.println(" dBm");
	     }// End of time4Meas IF
	if ((millis() - elapsedtime) > Minutes4Update) time4Meas = true; // check if MINS passed and change variable to true to send a message to the MQTT broker

        client.poll(); // Check if any message were received on the topic we subscribed to
	delay(1000); // Wait for 1sec

} // End of loop() function

///// callback function calls from client.poll() /////
void callback(char* topic, byte* payload, unsigned int length) {

 /*
  Serial.println("Received message for topic ");
  Serial.print(topic);
  Serial.print(" with length ");
  Serial.println(length);
  Serial.println("Message:");  
  Serial.write(payload, length);   // Writes binary data to the serial port
*/
   char* json;                    // copy the payload content into a char*
   
  json = (char*) malloc(length + 1);     // void *malloc(size_t size) allocates the requested memory and returns a pointer to it
  memcpy(json, payload, length);         // void * memcpy ( void * destination, const void * source, size_t num );    Copy block of memory
  json[length] = '\0';                   // '\0'  end character - ATTENTION!!! Should it be json[length+1] = '\0'  ???
   
  if ( strcmp(topic, Registration_request_topic_response) == 0 )     //check if we received a message in a specific topic
  {
     Serial.print("Registration_response message Received"); 
     recvJson = aJson.parse(json);              // create a JSON object from the received message  
     Registration_response();                // call Registration_response() function
  }                                             //End-if ( strcmp(topic, Registration_request_topic_response) == 0 )
  
  if ( strcmp(topic, HVAC_cmd_topic) == 0 )     // checks if a command type message has been received
  {
     Serial.print("Act message Received"); 
     recvJson = aJson.parse(json);               // create a JSON object from the received message  
     HVAC_cmd();                              //call HVAC_cmd() function 
        
  }                   //End-if ( strcmp(topic, HVAC_cmd_topic) == 0 )
  
  if ( strcmp(topic, HVAC_reset_topic) == 0 )  // checks if a command type message has been received
  {
    Serial.print("Reset message received");
    recvJson = aJson.parse(json);               // create a JSON object from the received message
    HVAC_reset();                               // call HVAC_reset() function
  }           //End-if ( strcmp(topic, HVAC_reset_topic) == 0 )
}                    // End of callback() function

///// Function about Read NodeNo from mspflash memory /////
void doRead()
{
  
  int i=0;
  //Serial.println(Node_No);
  //Serial.println(Node_No, HEX);
  //Serial.println(sizeof(int));
  //Serial.println(".");
  //Serial.println("Read:");
  Flash.read(flash+(pos * sizeof(int)), (unsigned char*)&Node_No, sizeof(int));
  //Serial.println(Node_No);  
  //Serial.println(Node_No, HEX);
  //Serial.println(".");
}

///// Function about Write NodeNo to mspflash memory /////
void doWrite()
{
 Serial.println("Write");
 Flash.write(flash + (pos * sizeof(int)), (unsigned char*)&Node_No, sizeof(int));
 Serial.println("Done.");
}

///// Function to Erase mspflash memory /////
void doErase()
{
 Serial.println("Erase"); 
 Flash.erase(flash); 
 Serial.println("Done."); 
}

///// Reset nodes function /////
void HVAC_reset() 
{
   if (recvJson != NULL) {                                    // if the message is valid JSON
    aJsonObject* oldNodeNo = aJson.getObjectItem(recvJson, "oldNodeNo"); // read the old NodeNo value
    
     if (oldNodeNo != NULL)   {              // if NodeID is in the message 
       if(oldNodeNo->valueint == Node_No){
         aJsonObject* NodeNo = aJson.getObjectItem(recvJson, "NodeNo");    // Read NodeNo value from server
                 
         if(NodeNo!=NULL && NodeNo->valueint==-1)
         {
            doErase(); // Erase mspflash memory
            WDTCTL = 0; // zero out the WatchDog Timer so the device restarts
         }  //End-if (NodeNo!=NULL && Node_No==-1)
      }                                    //End-if (oldNodeNo->valueint == Node_No)
     }                                     //End-if (oldNodeNo != NULL)
  }                                        //End-if (recvJson != NULL)
   free(recvJson);                                             // free the JSON string
  aJson.deleteItem(recvJson);                                 // delete the JSON Object

}// End of HVAC_reset() function


///// MQTTconnection() function /////
void MQTTconnection() // Function about connecting to Mqtt Broker
{
   
    if (!client.connected()) // Reconnect if the connection was lost
    {
      if(!client.connect(clientID, username, password)) //  boolean connect (clientID, username, password)
      {
        Serial.println("Connection to MQTT Broker failed ! Check ClientID , Name or Password");
      } 
      else 
      {
      Serial.println("Connection success to MQTT Broker");
       
         doRead(); // Read NodeNo from mspflash memory
         // NodeNo = -1  Unregistered Node
      if (Node_No == -1){
         Registration_request();          // call Registration_request function to create json message
	 Serial.println("Waiting for response to registration message");
        
      }
      
       if(client.subscribe(Registration_request_topic_response))  // Check if someone subscribe in Topic
      {
        //Serial.println("Subscription successfull"); 
      } // End of subscribe() function
       if(client.subscribe(HVAC_cmd_topic))  // Check if someone subscribe in Topic
      {
        //Serial.println("Subscription successfull"); 
      } // End of subscribe() function
      if(client.subscribe(HVAC_reset_topic))  // Check if someone subscribe in Topic
      {
        //Serial.println("Subscription successfull"); 
      } // End of subscribe() function
     } // End of client.connect() function
    } // End of client.connected() function
} // End of MQTTconnection() function


///// Alternate MQTT_connection function /////
void Alt_MQTT_Connection(){
          if (!client_alt.connected()) // Reconnect if the connection was lost
              {
                if(!client_alt.connect(clientID, username, password)) //  boolean connect (clientID, username, password)
                {
                  Serial.println("Connection to altenate MQTT Broker failed ! Check ClientID , Name or Password");
                } 
                else 
                {
                Serial.println("Connection success to alternate MQTT Broker");
                sendJson = aJson.createObject();                // Creating the root JSON Object
                if (sendJson == NULL) 
                  {                         // if object = NULL
                  Serial.print("JSON Object not created.");     // print in serial console JSON Object not created.
                  }                                               // END - if (sendJson == NULL)
                   aJson.addStringToObject(sendJson, "ALARM", "Connection with Primary MQTT Broker Lost"); // Add the mac address 
                   sendstring = aJson.print(sendJson);             // save the created JSON object as a string
                client_alt.publish(HVAC_alt_broker_topic, sendstring);           // int publish (topic, payload)
                Serial.print("Sent: ");                         // print in serial console the message Sent: 
                Serial.println(sendstring);                     // print in serial console the JSON string we sent
                free(sendstring);                               // release the variable for JSON string
                aJson.deleteItem(sendJson);                     // delete the JSON object
                }// END - if(!client.connect(clientID, username, password))
              }// END - if (!client.connected())
}// END of Alt_MQTT_Connection() function


///// Registration_request() function /////
void Registration_request(){
sendJson = aJson.createObject();                // Creating the root JSON Object
  if (sendJson == NULL) 
    {                         // if object = NULL
    Serial.print("JSON Object not created.");     // print in serial console JSON Object not created.
    }                                               // END - if (sendJson == NULL)
  aJson.addStringToObject(sendJson, "NodeID", Node_ID); // Add the mac address 
  aJson.addStringToObject(sendJson, "Ver", WiFi.firmwareVersion()); // Add the Version of CC3100 wifi BOOSTERPACK
  aJson.addStringToObject(sendJson, "Type", "hvac_comb"); // Add the Type of the device (hvac_meas,hvac_act,hvac_comb,hvac_fc,hvac_IR,other)
  aJson.addStringToObject(sendJson, "HW Ver", "MSP430F5529 REV4.0 + CC3100 REV3.0"); // Add the Harware Version we use
  
  ////publish Registration_request message /////
  sendstring = aJson.print(sendJson);             // save the created JSON object as a string
  client.publish(Registration_request_topic, sendstring);           // int publish (topic, payload)
        Serial.print("Sent: ");                         // print in serial console the message Sent: 
        Serial.println(sendstring);                     // print in serial console the JSON string we sent
        free(sendstring);                               // release the variable for JSON string
        aJson.deleteItem(sendJson);                     // delete the JSON object
}//End of Registration_request() function


///// Registration_response() function /////
void Registration_response(){
  if (recvJson != NULL) {                                    // if the message is valid JSON
    aJsonObject* NodeID = aJson.getObjectItem(recvJson, "NodeID"); // read the NodeID value
    
     if (NodeID != NULL)   {              // if NodeID is in the message 
     char* MAC=NodeID->valuestring;       // store mac value
      if ( strcmp(MAC, Node_ID) == 0 ) {  // compare mac from message with device mac
         aJsonObject* NodeNo = aJson.getObjectItem(recvJson, "NodeNo");    // Read NodeNo value from server
         Node_No=NodeNo->valueint;         // store NodeNumber value
            doErase(); // Erase mspflash memory
            doWrite(); // Write NodeNo to mspflash memory
            WDTCTL = 0; // zero out the WatchDog Timer so the device restarts
         aJsonObject* msgCnt = aJson.getObjectItem(recvJson, "msgCount");  // store to msgCnt the value received from server
      }                                    //End-if ( strcmp(MAC, Node_ID == 0 )
     }                                     //End-if (NodeID != NULL)
  }                                        //End-if (recvJson != NULL)
   free(recvJson);                                             // free the JSON string
  aJson.deleteItem(recvJson);                                 // delete the JSON Object
        
}//END of Registration_response() function


///// HVAC_meas() function /////
void HVAC_meas()
{
sendJson = aJson.createObject();                // Creating the root JSON Object to be filled with the measurements
  if (sendJson == NULL) 
    {                         // if object = NULL
    Serial.print("JSON Object not created.");     // print in serial console JSON Object not created.
    }                                         // END - if (sendJson == NULL)
  aJson.addNumberToObject(sendJson, "NodeNo", Node_No); // Add the NodeNo (Node Number)
 
  
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  int HVAC_hum = dht.readHumidity();                            // read and store the humidity from the sensor to variable - ATTENTION!! Should it be float according to the example in the lib?
  int HVAC_temp = dht.readTemperature();                        // read and store the temperature from the sensor to variable - ATTENTION!! Should it be float according to the example in the lib?

  if ((HVAC_temp == 0) & (HVAC_hum == 0)) {  /* t =0 AND h = 0 out of specs so error in reading (isnan(t) || isnan(h)) { // check if returns are valid, if they are NaN (not a number) then something went wrong! */    
    Serial.println("Failed to read from DHT");           // print in serial console Failed to read from DHT
    aJson.addNumberToObject(sendJson, "Humidity", -100); // add to the JSON object -100 to indicate error in reading
    aJson.addNumberToObject(sendJson, "Temperature", -100); // add to the JSON object -100 to indicate error in reading
  } else {                                               // else if returns are valid
    aJson.addNumberToObject(sendJson, "Humidity", HVAC_hum);    // add to the JSON object the value from the sensor
    aJson.addNumberToObject(sendJson, "Temperature", HVAC_temp); // add to the JSON object the value from the sensor
  }                                                      // END - if (isnan(t) || isnan(h))
  readOWTemperature(); // Reading temperature from DS18B20 sensor function and assign it to the global variable HVAC_fluid_temp
  aJson.addNumberToObject(sendJson, "CoolantTemp", HVAC_fluid_temp);// add the fluid temperature to the JSON object
  HVAC_Unit_OnOff = digitalRead(HVAC_Unit_OnOff_PIN); // check the state of the pin and assign the value to HVAC_Unit_OnOff variable
  aJson.addBooleanToObject(sendJson, "OnOff", HVAC_Unit_OnOff);// add the onoff status to the JSON object  
  HVAC_Unit_fan = digitalRead(HVAC_Unit_fan_PIN); // check the state of the pin and assign the value to HVAC_Unit_fan variable
  aJson.addBooleanToObject(sendJson, "HiLow", HVAC_Unit_fan);// add the hilow status to the JSON object
  aJson.addNumberToObject(sendJson, "RSSI", (int) rssi);// add the receiver signal strength to the JSON object
  aJson.addNumberToObject(sendJson, "msgCount", msgCount++); // count messages and add to the JSON object
  //publish HVAC_meas message
  sendstring = aJson.print(sendJson);             // save the created JSON object as a string
  client.publish(HVAC_meas_topic, sendstring);           // int publish (topic, payload)
        Serial.print("Sent: ");                         // print in serial console the message Sent: 
        Serial.println(sendstring);                     // print in serial console the JSON string we sent
        free(sendstring);                               // release the variable for JSON string
        aJson.deleteItem(sendJson);                     // delete the JSON object
}// End of HVAC_meas() function


///// HVAC_cmd() function /////
void HVAC_cmd(){
    if (recvJson != NULL) {                                    // if the message is valid JSON
    aJsonObject* NodeNo = aJson.getObjectItem(recvJson, "NodeNo"); // check the NodeNo
     if (NodeNo != NULL)   {              // if NodeNo is in the message 
      if (NodeNo->valueint == Node_No ){  // if NodeNo from message is the same with device NodeNo 
       aJsonObject* msgCnt = aJson.getObjectItem(recvJson, "msgCount"); // store to msgCnt the value received from server  
       aJsonObject* on = aJson.getObjectItem(recvJson, "OnOff");  // Read OnOff value from server
       aJsonObject* fan = aJson.getObjectItem(recvJson, "HiLow"); // Read HiLow value from server
       if (on != NULL) {                                       // if on value exists in message
        //Serial.println(on->valuebool);                      // print it to the serial console
        if (on->valuebool) {                                  // if on = true
          digitalWrite(HVAC_Unit_OnOff_PIN, HIGH);                      // set pin to HIGH
        } else {                                              // else if on = false
          digitalWrite(HVAC_Unit_OnOff_PIN, LOW);                       // set pin to LOW
        }                                                     // END - if (on->valuebool)
      }                                                       // END - if (on != NULL)
      if (fan != NULL) {                                      // if fan value exists in message
        //Serial.println(fan->valuebool);                     // print it to serial console
        if (fan->valuebool) {                                 // if fan = true
          digitalWrite(HVAC_Unit_fan_PIN, HIGH);                        // set pin to HIGH
        } else {                                              // else if fan = false
          digitalWrite(HVAC_Unit_fan_PIN, LOW);                         // set pin to LOW
        }                                                     // END - if (fan->valuebool)
      }      // END - if (fan != NULL)
      
       HVAC_response();// Call  HVAC_response() function
      
      }else{
        Serial.print("\n Act message for other device");
      }//End - if  MAC == Node_ID
     }// End - if NodeID != NULL
   }                                                           // END - if (recvJson != NULL)
  free(recvJson);                                             // free the JSON string
  aJson.deleteItem(recvJson);                                 // delete the JSON Object
  
}// End of HVAC_cmd() function


///// HVAC_response() function /////
void HVAC_response(){
  sendJson = aJson.createObject();                // Creating the root JSON Object
  if (sendJson == NULL) 
    {                         // if object = NULL
    Serial.print("JSON Object not created.");     // print in serial console JSON Object not created.
    }                                               // END - if (sendJson == NULL)
  aJson.addNumberToObject(sendJson, "NodeNo", Node_No); // Add the NodeNo to the JSON object 
    HVAC_Unit_OnOff = digitalRead(HVAC_Unit_OnOff_PIN); // check the state of the pin and assign the value to HVAC_Unit_OnOff variable
    aJson.addBooleanToObject(sendJson, "OnOff", HVAC_Unit_OnOff);//  add the OnOff to JSON object
       HVAC_Unit_fan = digitalRead(HVAC_Unit_fan_PIN); // check the state of the pin and assign the value to HVAC_Unit_fan variable
       aJson.addBooleanToObject(sendJson, "HiLow", HVAC_Unit_fan);// add the HiLow to JSON object
       aJson.addBooleanToObject(sendJson, "LEDR", digitalRead(LEDR));// add the LEDR to JSON object
       aJson.addBooleanToObject(sendJson, "LEDG", digitalRead(LEDG));// add the LEDG to JSON object
       aJson.addNumberToObject(sendJson, "msgCount", msgCount); // count messages and add to the JSON object
       //publish HVAC_response message
        sendstring = aJson.print(sendJson);             // save the created JSON object as a string
    client.publish(HVAC_cmd_ack_topic, sendstring);           // int publish (topic, payload)
        Serial.print("Sent: ");                         // print in serial console the message Sent: 
        Serial.println(sendstring);                     // print in serial console the JSON string we sent
        free(sendstring);                               // release the variable for JSON string
        aJson.deleteItem(sendJson);                     // delete the JSON object
}// End of HVAC_response function

///// Fail Safe function if connection lost then turn off fan coil /////
void FAIL_SAFE() {
  digitalWrite(LEDR, HIGH);            // configuration done turn on red led
  digitalWrite(HVAC_Unit_OnOff_PIN, HIGH);     // failsafe mode set to HIGH
  digitalWrite(HVAC_Unit_fan_PIN, HIGH);       // failsafe mode set to HIGH
  Serial.println("Enter Fail Safe Mode"); 
}// End of FAIL_SAFE function


void printWifiStatus()
{                   // function to print the SSID of the network we're attached to
  Serial.print("Network Name: ");          // print the Network Name: in serial console
  Serial.println(WiFi.SSID());             // print the SSID of the network you're attached to

  IPAddress ip = WiFi.localIP();           // gets the WiFi shield's IP address
  Serial.print("IP Address: ");            // print IP Address: in serial monitor
  Serial.println(ip);                      // prints the shield's IP address in serial monitor

  rssi = WiFi.RSSI();                 // gets the received signal strength
  Serial.print("signal strength (RSSI):"); // print the signal strength (RSSI): in serial monitor
  Serial.print(rssi);                      // print the received signal strength in serial monitor
  Serial.println(" dBm");                  // print the dBm in serial console
}                                          // END - void printWifiStatus()

 
void readOWTemperature(void){              //Send a global temperature convert command
  ds.reset();                              // reset the OneWire driver
  ds.write_byte(0xcc);                     // was ds.select(work); so request all OW's to do next command
  ds.write_byte(0x44);                     // start conversion, with parasite power on at the end
  delay(1000);                             // wait 1 sec needed for the conversion - ATTENTION!! Really??
  for (i=1; i<ROMmax+1;i++) {
    if (ROMtype[i]==0x28) {
      readOW(i);
      saveTemperature(i);
    }
  }
  for (i=1;i<ROMmax+1;i++){
    if (ROMtype[i]==0x28) {
      foundOW = true;
      //Serial.print(result[i]);
      HVAC_fluid_temp = result[i];
    }
  }
  ds.reset();
  //delay(500);                                 // wait 0,5 sec - ATTENTION!! Really??
}

void saveTemperature(uint8_t ROMno){
  int32_t newtemp32;
  float i; // uint8_t
  newtemp32=data[1]<<8;
  newtemp32=newtemp32+data[0]>>4;
  result[ROMno]=byte(newtemp32);
  i=(data[0] & 0x0F)* 625/1000;
  if (i>=5)  result[ROMno]++;
}

void readOW(uint8_t ROMno)
{
   uint8_t i;
   ds.reset();
   ds.select(ROMarray[ROMno]);
   ds.write_byte(0xBE);         // Read Scratchpad
   for ( i = 0; i < 9; i++) {   // need 9 bytes
      data[i] = ds.read_byte();
#if TEST
      if (data[i]<16) Serial.print("0");
      Serial.print(data[i], HEX);
      Serial.print(" ");
#endif   
  }
}

void findOW(void)
{
 byte addr[8]; 
 uint8_t i; 
 ROMmax=0;  ///////////////////////////////////////////////////////
 while (true){  //get all the OW addresses on the buss
   i= ds.search(addr);
   if ( i<10) {
      //Serial.print("ret=("); 
      //Serial.print(i);
      //Serial.print(") No more addresses.\n");
      ds.reset_search();
      delay(500);
      return;
   }
   //Serial.print("R=");
    for( i = 0; i < 8; i++) {
       if (i==0)  ROMtype[ROMmax+1]=addr[i];  // store the device type
         
      ROMarray[ROMmax+1][i]=addr[i];     

      //if (addr[i]<16) Serial.print("0"); 
      //Serial.print(addr[i], HEX);
      //Serial.print(" ");
    }
    ROMmax++;
    //Serial.print ("(OW");
    // Serial.print (ROMmax,HEX);
   //Serial.print (") Type="); 
   // Serial.println (ROMtype[ROMmax],HEX);
 } 
}


/*
void displayOW(void)
{
  uint8_t i;
  Serial.println ("From array");
  for (ROMcount=1; ROMcount<ROMmax+1; ROMcount++) {   
    ds.reset();
    for( i = 0; i < 8; i++) {
      if (ROMarray[ROMcount][i]<16) Serial.print("0");
      Serial.print( ROMarray[ROMcount][i], HEX);
      Serial.print(" ");
    }
    Serial.println("");
  }
}
*/
