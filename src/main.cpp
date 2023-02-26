#include <SPI.h>
#include <RHDatagram.h>
#include <RH_RF95.h>

#define RFM95_RST 4      //Radio Reset Pin
#define RFM95_INT 7      //Radio GPIO0/IRQ Pin
#define RFM95_CS 8       //Radio Chip Select Pin
#define RFM95_FREQ 915.0 //868 MHz or 915 MHz
#define LED 13           //Built-In LED

//Radio client/server addresses
#define REMOTE_ADDRESS 0 // This feather remote is the client
//#define ROVER_ADDRESS 1 // The feather on the Rover is the server
uint8_t roverAddress = 1;

//Radio Driver Instance
RH_RF95 rf95(RFM95_CS, RFM95_INT);

//Class to manage message delivery and receipt, using the driver (rf95) declared above
//RHReliableDatagram manager(rf95, SERVER_ADDRESS);
RHDatagram manager(rf95, REMOTE_ADDRESS);

// messageIn Struct
struct inStruct
{
  uint8_t resetConfirmed;
  uint8_t resetTime;
  uint8_t roverMode;
  uint8_t signalMSB;
  uint8_t signalLSB;
} messageIn;

// messageOut Struct
struct outStruct
{
  uint8_t resetFlag;
  uint8_t statusUpdate;
} messageOut;

// Buffer for passing messages via radio
uint8_t bufT[sizeof(messageIn)] = {0};
uint8_t bufO[sizeof(messageOut)] = {0};

// Interrupt variables
volatile unsigned long lastHeartbeat = 0; // Last time we received a message from Rover Feather

// Constants
const int signalCheckInterval = 50; // number of millisecs before temperature readings
const int recvInterval = 0;         // number of millisecs before checking for messages
const int sendInterval = 0;         // number of millisecs before sending messages
const int checkInterval = 3000;     // number of millisecs before checking for hearbeat
const int confInterval = 3500;      // number of millisecs to prevent false confirms
const int resetInterval = 3000;     // number of millisecs waiting for reset confirmation
const int heartbeatInterval = 5000; // number of millisecs waiting for Rover Heartbeat
const int flagInterval = 3000;      // number of millisecs waiting for flag serial message

// Variables
unsigned long previousSignalMillis = 0;
unsigned long previousSendMillis = 0;
unsigned long previousResetMillis = 0;
unsigned long previousRecvMillis = 0;
unsigned long previousConfMillis = 0;
unsigned long previousCheckMillis = 0;
unsigned long previousWakeMillis = 0;
unsigned long previousFlagMillis = 0;
unsigned long currentMillis = 0;
uint16_t signalAverage = 0;
int numberAttempts = 0;

bool messageOutFlag = false;
bool resetConfirmed = false;
bool waitingForConfirmation = false;
bool waitingForWake = false;
bool firstReset = true;
bool resetWaitFlag = false;
bool wakeupConfirmed = false;
bool wakeOutFlag = true;
bool wakeFlag = false;
bool motorsKilled = false;

//Blink LED Function
void blinkLED(int onDelayTime, int offDelayTime)
{
  digitalWrite(LED, HIGH);
  delay(onDelayTime);
  digitalWrite(LED, LOW);
  delay(offDelayTime);
}

void checkMessages()
{
  uint8_t bufR[sizeof(messageIn)]; //Gets filled up from client
  uint8_t len = sizeof(bufR);      //Length of bufR
  uint8_t from;                    //The address that sent the data

  if (currentMillis - previousRecvMillis >= recvInterval)
  {
    //Serial.println("In receive function");
    //delay(100);
    if (manager.available())
    {
      //Serial.println("Manager ===== available");
      //If the data was retrieved successfully, copy it
      if (manager.recvfrom(bufR, &len, &from))
      {
        if (from == roverAddress)
        {
          //Serial.println("Message ================ received");
          memcpy(&messageIn, bufR, sizeof(messageIn));
          previousRecvMillis = currentMillis;
          lastHeartbeat = currentMillis;
        }
      }
      else
      {
        //Serial.println("Message ================= FAILED");
      }
    }
    else
    {
      //Serial.println("Manager ===== NOT available");
    }
    delay(20);
  }

  if (resetWaitFlag && messageIn.resetConfirmed)
  {
    resetWaitFlag = false;
  }
  else
  {
    messageIn.resetConfirmed = false;
  }

  if (messageIn.resetConfirmed && !resetConfirmed)
  {
    resetConfirmed = true;
    motorsKilled = true;
  }
  else
  {
    resetConfirmed = false;
  }
}

void sendMessages()
{
  //Serial.println("In send function");
  if (messageOutFlag | wakeOutFlag)
  {
    if (messageOutFlag)
    {
      messageOut.resetFlag = 1;
    }
    else
    {
      messageOut.resetFlag = 0;
    }
    uint8_t len = sizeof(messageOut);
    messageOut.statusUpdate = 1;
    memcpy(bufO, &messageOut, len);
    Serial.println("Message ============= sending");
    for (int i = 0; i <= 4; i++)
    {
      if (manager.sendto(bufO, len, roverAddress))
      {
        //Serial.println("Message ================================= sent");
      }
    }
    resetWaitFlag = true;
    messageOut.resetFlag = 0;
    messageOutFlag = false;
    wakeOutFlag = false;
  }
}

void setMessageFlags()
{
  if (currentMillis - previousFlagMillis >= flagInterval)
  {
    long temp;
    if (Serial && Serial.available() > 0)
    {
      Serial.println("Enter 1 to change rover mode to 1");
      Serial.println("Enter 2 to change rover mode to 2");
      Serial.println("Enter 3 to kill motors");
      Serial.print("  Current Rover Mode: ");
      Serial.println(roverAddress, HEX);
      Serial.println();

      /* if a number is waiting, process it */
      if (isdigit(Serial.peek()))
      {
        temp = Serial.parseInt();
        if (temp == 1)
        {
          // messageOut.roverMode = 1;
        }
        else if (temp == 2) 
        {
          // messageOut.roverMode = 2;
        }
        else if (temp == 3) 
        {
          messageOutFlag = true;
          Serial.println("Attempting to kill motors");
        }
      }
      /* else throw it away */
      else
      {
        Serial.read();
      }

      Serial.println("");
    }
    previousFlagMillis = currentMillis;
  }
}

void setup()
{
  Serial.begin(9600);

  // Initialize Message Struct
  messageIn.resetTime = 0;
  messageIn.signalMSB = 0;
  messageIn.signalLSB = 0;
  messageIn.roverMode = 1;
  messageIn.resetConfirmed = false;
  messageOut.resetFlag = 0;
  messageOut.statusUpdate = 0;

  //Reseting Radio Before Use
  digitalWrite(RFM95_RST, LOW);
  pinMode(RFM95_RST, OUTPUT);
  delayMicroseconds(100); // Pull low for 100 microseconds to force reset
  pinMode(RFM95_RST, INPUT);
  delay(5); // Chip should be ready 5ms after low pulse

  //Blink Built-In LED If Driver Initialization Failed
  if (!rf95.init())
  {
    while (true)
    {
      Serial.println("Driver Initialization Failed");
      blinkLED(750, 250);
    }
  }

  //Blink Built-In LED If Manager Initialization Failed
  if (!manager.init())
  {
    while (true)
    {
      Serial.println("Manager Initialization Failed");
      blinkLED(900, 100);
    }
  }

  //Blink Built-In LED If Selected Frequency Failed to Set
  if (!rf95.setFrequency(RFM95_FREQ))
  {
    while (true)
    {
      Serial.println("Selected Frequency Failed to Set");
      blinkLED(250, 750);
    }
  }

  /*
   * Sets Transmission Power. Range is from +5dBm to +23dBM which
   * is about 3.2mW to 200mW respectivley. Refer to 
   * https://en.wikipedia.org/wiki/DBm to get dBm benchmarks.
   */
  rf95.setTxPower(23, false); //(power, useRFO)

  previousCheckMillis = millis();
  currentMillis = millis();
}

void loop()
{
  currentMillis = millis();
  //checkMessages();

  //noInterrupts();
  if (currentMillis - previousCheckMillis >= checkInterval)
  {
    if (messageIn.resetTime == 0)
    {
      Serial.println("  < 1 minute");
    }
    else if (messageIn.resetTime == 100)
    {
      Serial.println("  > 1 hour");
    }
    else if (messageIn.resetTime == 1)
    {
      Serial.print(messageIn.resetTime);
      Serial.println(" minute");
      
    }
    else
    {
      Serial.print(messageIn.resetTime);
      Serial.println(" minutes");
    }

    if ((currentMillis - lastHeartbeat > heartbeatInterval) | (lastHeartbeat == 0))
    {
      Serial.println("DISCONNECTED");
      numberAttempts++;

      if (numberAttempts >= 3) 
      {
        wakeOutFlag = true;
        numberAttempts = 0;
      }
      
    }
    else
    {
      Serial.println("CONNECTED");
    }

    if (motorsKilled)
    {
      motorsKilled = false;
      Serial.println("MOTOR KILL CONFIRMED");
    }

    // Remember the time and value of this update
    previousCheckMillis = millis();
  }

  if (messageOutFlag)
  {
    waitingForConfirmation = true;
    previousResetMillis = millis();
  }
  else if (wakeOutFlag)
  {
    waitingForWake = true;
    previousWakeMillis = millis();
  }

  if (waitingForConfirmation)
  {
    if (millis() - previousConfMillis <= confInterval)
    {
      resetConfirmed = false;
      messageIn.resetConfirmed = false;
    }
    if (resetConfirmed)
    {
      waitingForConfirmation = false;
      resetConfirmed = false;
      messageIn.resetConfirmed = false;
      previousConfMillis = millis();
    }
    else if (millis() - previousResetMillis >= resetInterval)
    {
      waitingForConfirmation = false;
    }
  }
  if (waitingForWake)
  {
    if (millis() - lastHeartbeat <= confInterval)
    {
      wakeupConfirmed = false;
    }
    if (wakeupConfirmed)
    {
      waitingForWake = false;
      wakeupConfirmed = false;
      previousWakeMillis = millis();
    }
    else if (millis() - previousWakeMillis >= resetInterval)
    {
      waitingForWake = false;
    }
  }
  
  //interrupts();
  setMessageFlags();
  checkMessages();
  sendMessages();
  delay(50);
}