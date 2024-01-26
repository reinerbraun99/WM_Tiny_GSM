/********************************************************************************
* Flowmeter V2a
* Sandwich
*
* By: Luthfi Pratama P. S. & Rainier Adrian
* PT. LCS
* 2023-09-19
*
* Change Log:
* 2023-09-19
* 1. Function added
* 2. Documentation added
* 3. Variable revised
* 4. Deep sleep mechanism revised
*
* 2023-09-22
* 1. Function added (reg_to_dec)
* 2. Change case 2 in main loop
* 3. Change variable for sandwich flowmeter
*
* 2023-10-04
* 1. Change pin: 
*    - SIM800_RX_PIN 21 --> 22
*    - SIM800_TX_PIN 22 --> 21
*    - SW_SIM800 5 --> 18
*    - SW_RS485 18 --> 19
*
* 2023-10-23
* 1. Change pin:
*    - SIM800_RX_PIN 22 --> 21
*    - SIM800_TX_PIN 21 --> 22
*
* 2023-10-27
* 1. Added telegram modbus for getting flow rate data
* 2. Minor change in main loop for getting data from modbus
* 3. Added errCount on setupmodule() for SIM800
*
* 2023-11-06
* 1. Added battery level function
*
* 2023-11-09
* 1. Added deepsleep if the battery is going LOW
* 2. Added accumulated volume for backend
*
* 2023-12-01
* 1. Change library for SIM800L (SIM800L to TINYGSM)
* 2. Add OTA Update for SIM800L using OTA Drive
*
* 2023-12-19
* 1. Adding process for handling and sending data
* 2. Adding setup mode
*
* 2024-01-04
* 1. Adding SMS handling process for config mode
*
* 2024-01-11
* 1. Minor update on SMS and HTTP response handling process
*
* 2024-01-22
* 1. Minor changes in saving data mechanism
 *******************************************************************************/
/*******************************************************************************
* Libraries
********************************************************************************/
// TinyGSM credentials
#define TINY_GSM_MODEM_SIM800
#define TINY_GSM_RX_BUFFER 1024

#include <otadrive_esp.h>
#include <Arduino.h>
#include <TinyGsmClient.h>
#include <SoftwareSerial.h>
#include <esp32-hal.h>
#include <Preferences.h>
#include <ModbusRtu.h>
#include <SPIFFS.h>

//PREFERENCES
Preferences preferences;
int lastState;
int trialState;
float lastVolume;
int trialPointer;

//SIM800L
#define SIM800_RX_PIN 21
#define SIM800_TX_PIN 22
#define SIM800_RST_PIN 2
#define SW_SIM800 18
SoftwareSerial SerialAT(SIM800_RX_PIN, SIM800_TX_PIN);

//RS-485
#define RS485_DI 17
#define RS485_RO 16
#define RS485_EN 15
#define SW_RS485 19

//Variable for Battery Level
#define ANALOG_PIN 4
bool BATT_LOW = false;
const float MAX_BATTERY_VOLTAGE = 4.2;
const float MAX_ANALOG_VAL = 4164.0;
float voltageLevel = 0;

//Variable for Modbus
uint16_t au16data[64]; //!< data array for modbus network sharing
uint8_t u8state; //!< machine state
uint8_t u8query; //!< pointer to message query
uint8_t u8response;

// Variable for Saving Data
String data_sent;
String data_sent_combined;
String savedData;
bool DATA_NOT_SENT = false;
char buffer[20]; // Buffer to store characters from HTTP response

// Variable for setup mode
bool setup_mode;
bool setup_mode_trigger;
String SMS_Message;
bool interval_sent;

//Modbus setup
Modbus master(0,Serial2,RS485_EN);

//Modbus query
modbus_t telegram[3];

// Random Data
long randNumber;

//Waiting time for Modbus
unsigned long u32wait;
long lastMillis = 0;
long lastMillisCase1 = 0;
long lastMillisCase2 = 0;

//Variable for SIM800L
const char apn[] = "Telkomsel";
const char user[] = "";
const char pass[] = "";
const char server[] = "103.156.114.74";
const int port = 2924;
const char path[] = "/api/iot?serialNumber=WM4ec8eb50&data=";
const char setup_url[] = "/api/iot/setup?serialNumber=WM4ec8eb50&interval=";

//Deepsleep time in micorseconds
#define S_TO_uS_FACTOR 1000000
long sleepTime;
String sleepTime_str;

// AT Command setup from TinyGSM
#ifdef DUMP_AT_COMMANDS
  #include <StreamDebugger.h>
  StreamDebugger debugger(SerialAT, Serial);
  TinyGsm modem(debugger);
#else
  TinyGsm modem(SerialAT);
#endif

// TinyGSM client declaration
TinyGsmClient client(modem);
TinyGsmClient gsm_otadrive_client(modem, 1);

// ========================================================================== //
//                                FUNCTIONS                                   //
// ========================================================================== //
/************************************************************************
 * Function : update_prgs()
 * Process  : update progress for firmware update
 * Input    : global variable
 ************************************************************************/
void update_prgs(size_t i, size_t total){
  Serial.printf("upgrade %d/%d   %d%%\n", i, total, ((i * 100) / total));
}

/************************************************************************
 * Function : HttpData()
 * Process  : print HTTP POST format data
 * Input    : path, data_sent_char, server
 ************************************************************************/
void HttpData(const char* path, const char* data_sent_char, const char* server){
  // Make a HTTP POST request:
  Serial.println("Performing HTTP POST...");
  client.print(String("POST ") + path + data_sent_char + " HTTP/1.1\r\n");
  client.print(String("Host: ") + server + "\r\n");
  client.println("Connection: keep-alive");
  //client.println("Content-Type: application/json");
  //client.println("Content-Length: ");
  //client.println(postData.length());
  client.println("ApiKey: 944244b7-312d-4bdc-ba55-84d173fac757");  // Include API key in headers
  client.println();  // End headers

  Serial.println("Sending : ");
  Serial.print(String("POST ") + path + data_sent_char + " HTTP/1.1\r\n");
  Serial.print(String("Host: ") + server + "\r\n");
  Serial.println("Connection: keep-alive");
  //Serial.println("Content-Type: application/json");
  //Serial.println("Content-Length: ");
  //client.println(postData.length());
  Serial.println("ApiKey: 944244b7-312d-4bdc-ba55-84d173fac757");  // Include API key in headers
  Serial.println();  // End headers
}

/************************************************************************
 * Function : saveData()
 * Process  : save data to preferences
 * Input    : DATA_NOT_SENT
 ************************************************************************/
void saveData(String data_sent, String data_sent_combined, bool DATA_NOT_SENT){
  trialState++;
  if (DATA_NOT_SENT == false){
    preferences.putString("savedData", data_sent);
    Serial.print("Data saved: ");
    Serial.println(data_sent);
  }
  else{
    preferences.putString("savedData", data_sent_combined);
    Serial.print("Data saved: ");
    Serial.println(data_sent_combined);
  }
  DATA_NOT_SENT = true;
  preferences.putBool("dataState", DATA_NOT_SENT);
}

/************************************************************************
 * Function : sendHttpPostRequest()
 * Process  : sending data via HTTP POST
 * Input    : accVolume, Debit, voltageLevel
 ************************************************************************/
void sendHttpPostRequest(float accVolume, float Debit, float voltageLevel) {
  preferences.begin("flowmeter", false);
  trialState = preferences.getInt("trialState", 0);
  Serial.print("Trial State: ");
  Serial.println(trialState);
  DATA_NOT_SENT = preferences.getBool("dataState", false);

  String data_sent = String(accVolume) + ";" + String(Debit) + ";" + String(voltageLevel);

  // Combine all data in data_sent
  String data_sent_combined = "";

  // Check if the last data did not sent
  if (DATA_NOT_SENT != false){
    savedData = preferences.getString("savedData", data_sent);
    Serial.print("Saved Data: ");
    Serial.println(savedData);
    // Combine data to data_sent_combined
    data_sent_combined = savedData + "#" + data_sent;
  }

  Serial.print(F("Connecting to "));
  Serial.print(server);
  if (!client.connect(server, port)) {
    Serial.println(" fail");
    delay(10000);
    saveData(data_sent, data_sent_combined, DATA_NOT_SENT);
    preferences.putInt("trialState", trialState);
    ESP.restart();
    return;
  }
  Serial.println(" OK");

  if (interval_sent == false){
    sleepTime_str = String(sleepTime);
    const char* interval_sent_char = sleepTime_str.c_str();
    HttpData(setup_url, interval_sent_char, server);
  }

  // Use this for real situation
  if (DATA_NOT_SENT == false){
    Serial.println(data_sent);
    const char* data_sent_char = data_sent.c_str();
    HttpData(path, data_sent_char, server);
  }
  // Use this for real situation
  else if (DATA_NOT_SENT == true){
    Serial.println(data_sent_combined);
    const char* data_sent_char = data_sent_combined.c_str();
    HttpData(path, data_sent_char, server);
  } 

  long timeout = millis();
  while (client.available() == 0) {
    if (millis() - timeout > 10000L) {
      Serial.println(F(">>> Client Timeout !"));
      client.stop();
      saveData(data_sent, data_sent_combined, DATA_NOT_SENT);
      break;
    }
  }

  int index = 0;   // Index for buffer
  
  // Read the server's response
  while (client.available()) {
    char c = client.read();
    Serial.print(c);
    
    // Add character to buffer
    buffer[index] = c;
    index++;
    
    // Null-terminate buffer
    buffer[index] = '\0';
    
    // Check if buffer contains "HTTP/1.1 200 OK"
    if (strstr(buffer, "HTTP/1.1 200 OK") != NULL && setup_mode_trigger == false) {
      trialState = 0;
      setup_mode = false;
      preferences.putInt("sleepTime", sleepTime);
      preferences.putBool("interval_sent", true);
      preferences.putBool("setup_mode", false);
      
      // Reset buffer and index for next use
      memset(buffer, 0, sizeof(buffer));
      index = 0;
    }
    // Check if HTTP OK
    else if (strstr(buffer, "HTTP/1.1 200 OK") != NULL && setup_mode_trigger == true){
      trialState = 0;
      preferences.putBool("dataState", false);

      // Reset buffer and index for next use
      memset(buffer, 0, sizeof(buffer));
      index = 0;
    }
    // Check if HTTP Error
    else if (strstr(buffer, "HTTP/1.1 40") != NULL) {
      Serial.println("Error occured");
      saveData(data_sent, data_sent_combined, DATA_NOT_SENT);

      // Reset buffer and index for next use
      memset(buffer, 0, sizeof(buffer));
      index = 0;
      break;
    }

    // Handle buffer overflow by resetting if buffer is full
    if (index >= sizeof(buffer) - 1) {
      memset(buffer, 0, sizeof(buffer));
      index = 0;
    }
  }

  // Disconnect from the server
  client.stop();

  Serial.println("HTTP POST request complete.");

  preferences.putInt("trialState", trialState);
}

/************************************************************************
 * Function : reg_to_float754()
 * Process  : changing register into float from IEEE754 format data
 * Input    : register with IEEE754 format data
 * Output   : float data
 ************************************************************************/
float reg_to_float(uint16_t input1, uint16_t input2){
  uint16_t reg1; long reg2; long combined_reg;
  float float_data;

  reg1 = input1;
  reg2 = input2;
  // Serial.print("Reg 1: ");
  // Serial.println(reg1);
  // Serial.print("Reg 2: ");
  // Serial.println(reg2);
  combined_reg = (reg1 << 16) + reg2;
  float_data = *(float *) &combined_reg;
  // Serial.print("Combined reg: ");
  // Serial.println(combined_reg);
  // Serial.print("Float data: ");
  // Serial.println(float_data);
  return float_data;
}

/************************************************************************
 * Function : reg_to_float()
 * Process  : changing register into decimal
 * Input    : register data
 * Output   : float data
 ************************************************************************/
float reg_to_dec(uint16_t input1, uint16_t input2){
  long reg1; uint16_t reg2; long combined_reg;

  reg1 = input1;
  reg2 = input2;
  combined_reg = (reg1 << 16) + reg2;
  return combined_reg;
}

/************************************************************************
 * Function : switchOnTrans()
 * Process  : switch ON RS-485, Flowmeter, and SIM800L
 * This function using global variables so there are no inputs or outputs
 ************************************************************************/
void switchOnTrans(){
  digitalWrite(SW_RS485, HIGH);
  digitalWrite(SW_SIM800, HIGH);
}

/************************************************************************
 * Function : switchOffTrans()
 * Process  : switch OFF RS-485, Flowmeter, and SIM800L
 * This function using global variables so there are no inputs or outputs
 ************************************************************************/
void switchOffTrans(){
  digitalWrite(SW_RS485, LOW);
  digitalWrite(SW_SIM800, LOW);
}

/************************************************************************
 * Function : batterylevel()
 * Process  : monitor battery voltage
 * This function using global variables so there are no inputs or outputs
 ************************************************************************/
void batterylevel(){
  int rawValue = analogRead(ANALOG_PIN);
  voltageLevel = (rawValue / MAX_ANALOG_VAL) * 2 * 1.1 * 3.3;
  float batteryFraction = voltageLevel / MAX_BATTERY_VOLTAGE;
  Serial.println((String)"Raw:" + rawValue + " Voltage:" + voltageLevel + "V Percent: " + (batteryFraction * 100) + "%");
  if (voltageLevel < 3.75){
    Serial.println("Battery is LOW, please change the battery immediately");
    BATT_LOW = true;
  }
}

/************************************************************************
 * Function : sim_unlock()
 * Process  : unlock sim800l
 * This function using global variables so there are no inputs or outputs
 ************************************************************************/
bool sim_unlock(char* pin)
{   
  bool unlocked = false;
  int status = modem.getSimStatus();
  Serial.println("Check sim pin status to unlock: %d \n");
  Serial.println(status);
  if (status == SIM_LOCKED)
  {
    Serial.println("Unlocking sim\n");
    bool unlocked = modem.simUnlock(pin);
    Serial.println( unlocked);
    return unlocked;
  }
  return unlocked;
}

/************************************************************************
 * Function : checkNetwork()
 * Process  : checking SIM800L network
 * This function using global variables so there are no inputs or outputs
 ************************************************************************/
void checkNetwork(){
  if (!modem.isNetworkConnected()) {
    Serial.println("Network disconnected");
    if (!modem.waitForNetwork(180000L, true)) {
      Serial.println(" fail");
      delay(10000);
      ESP.restart();
      return;
    }
    if (modem.isNetworkConnected()) {
      Serial.println("Network re-connected");
    }

#if TINY_GSM_USE_GPRS
    // and make sure GPRS/EPS is still connected
    if (!modem.isGprsConnected()) {
      Serial.println("GPRS disconnected!");
      Serial.print(F("Connecting to "));
      Serial.print(apn);
      if (!modem.gprsConnect(apn, user, pass)) {
        Serial.println(" fail");
        delay(10000);
        ESP.restart();
        return;
      }
      if (modem.isGprsConnected()) { Serial.println("GPRS reconnected"); }
    }
#endif
  }
}

/************************************************************************
 * Function : updateSerial()
 * Process  : checking serial and serialAT
 * This function using global variables so there are no inputs or outputs
 ************************************************************************/
void updateSerial(){
  while (Serial.available())
  {
    SerialAT.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while (SerialAT.available())
  {
    Serial.write(SerialAT.read());//Forward what Software Serial received to Serial Port
  }
}

/************************************************************************
 * Function : updateSMS()
 * Process  : checking serial and serialAT for SMS
 * This function using global variables so there are no inputs or outputs
 ************************************************************************/
void updateSMS(){
  while (Serial.available())
  {
    Serial.println("Serial available");
    SerialAT.write(Serial.read());//Forward what Serial received to Software Serial Port
  }

  static String currentLine = "";
  while (SerialAT.available()) {
    //Serial.println("SerialAT available");
    char c = SerialAT.read();
    Serial.print(c);

    if (c == '\n') {
      Serial.println("c == nnn");
      Serial.println(currentLine);
      currentLine.trim(); // Remove leading/trailing whitespace
      Serial.print("Current Line Trimmed: ");
      Serial.println(currentLine);
      SMS_Message = currentLine;
      char charArray[SMS_Message.length() + 1];
      SMS_Message.toCharArray(charArray, sizeof(charArray));

      char *token1 = strtok(charArray, "#");
      char *token2 = strtok(NULL, "#");

      if (token1 != NULL && token2 != NULL) {
        Serial.print("Token 1: ");
        Serial.println(token1);
        Serial.print("Token 2: ");
        Serial.println(token2);

        // Convert tokens to appropriate data types if needed
        if (strcmp(token1, "false") == 0) {
          Serial.println("Setup mode was received!");
          SMS_Message = currentLine;
          setup_mode_trigger = false;
          interval_sent = false;
          Serial.println("Sleep time was received!");
          int sleepTimeSMS = atoi(token2); // Convert char array to integer
          sleepTime = sleepTimeSMS;
        }
      }

      currentLine = ""; // Clear the line
      Serial.println("currentLine cleared");
    } else {
      //Serial.println("c != nnn");
      currentLine += c; // Append character to the line
    }
  }
}

/************************************************************************
 * Function : setup()
 * Process  : Setting up serials, pin modes, SIM800L, preferences, wifi
 * This function using global variables
 ************************************************************************/
void setup() {
  // Serial setting for MODBUS
  Serial2.begin(9600, SERIAL_8N1, RS485_RO, RS485_DI);
  // telegram 0: read registers
  telegram[0].u8id = 1; // slave address
  telegram[0].u8fct = 3; // function code (this one is registers read)
  telegram[0].u16RegAdd = 1; // start address in slave
  telegram[0].u16CoilsNo = 2; // number of elements (coils or registers) to read
  telegram[0].au16reg = au16data; // pointer to a memory array in the Arduino

  // telegram 1: read registers
  telegram[1].u8id = 1; // slave address
  telegram[1].u8fct = 3; // function code (this one is registers read)
  telegram[1].u16RegAdd = 8; // start address in slave
  telegram[1].u16CoilsNo = 4; // number of elements (coils or registers) to read
  telegram[1].au16reg = au16data+2; // pointer to a memory array in the Arduino

  // telegram 2: write a single register
  telegram[2].u8id = 1; // slave address
  telegram[2].u8fct = 3; // function code (this one is write a single register)
  telegram[2].u16RegAdd = 1437; // start address in slave
  telegram[2].u16CoilsNo = 2; // number of elements (coils or registers) to read
  telegram[2].au16reg = au16data+6; // pointer to a memory array in the Arduino

  // Initialize Serial Monitor for debugging
  Serial.begin(115200);
  
  // Initialize random number
  randomSeed(micros());

  // Setting up PINOUT
  pinMode(SW_RS485, OUTPUT);
  pinMode(SW_SIM800, OUTPUT);

  // Preferences setting
  preferences.begin("flowmeter", false);

  // Checking setup mode
  setup_mode = preferences.getBool("setup_mode", false);
  Serial.print("Setup mode: ");
  Serial.println(setup_mode);
  setup_mode_trigger = true;
  interval_sent = preferences.getBool("interval_sent", true);
  Serial.print("Interval sent: ");
  Serial.println(interval_sent);

  // Enable deepsleep wakeup time
  sleepTime = preferences.getInt("sleepTime", 60);
  Serial.print("Sleep time: ");
  Serial.print(sleepTime);
  Serial.println(" seconds");
  esp_sleep_enable_timer_wakeup(1000000ULL * sleepTime);
  lastState = preferences.getInt("lastState", 0); //get restart data
  Serial.printf("Last state before reset: %d \n", lastState);
  lastState++; //add restart data
  preferences.putInt("lastState", lastState); //save restart data

  // Check lastState value
  if (lastState > 3){
    switchOffTrans();
    preferences.putInt("lastState", 0);
    Serial.println("State has been reset");
    esp_deep_sleep_start();
  }

  // End preferences
  preferences.end();
  
  while(!Serial);

  // Begin SerialAT for SIM800L
  SerialAT.begin(9600);

  //Ouput Pin Declaration (SIM800L, RS-485, BECO)
  Serial.println("LOW...");
  digitalWrite(SW_SIM800, LOW); // turn the LED off by making the voltage LOW
  delay(5000);                  // wait for 5 second
  Serial.println("HIGH...");
  digitalWrite(SW_SIM800, HIGH);
  delay(5000);

  // Restart SIM800L
  pinMode(SIM800_RST_PIN, OUTPUT);
  delay(5000);
  digitalWrite(SIM800_RST_PIN, LOW);
  delay(5000);
  digitalWrite(SIM800_RST_PIN, HIGH);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  Serial.println(F("Initializing modem..."));
  modem.restart();

  // Get modem infor
  String modemInfo = modem.getModemInfo();
  Serial.print(F("Modem: "));
  Serial.println(modemInfo);

  // Unlock SIM card with a PIN
  modem.simUnlock("1234");

  Serial.print("Waiting for network...");
  if (!modem.waitForNetwork()) {
    Serial.println(" fail");
    delay(10000);
    ESP.restart();
    return;
  }
  Serial.println(" OK");

  Serial.print(F("Connecting to "));
  Serial.print(apn);
  if (!modem.gprsConnect(apn, user, pass)) {
    Serial.println(" fail");
    delay(10000);
    ESP.restart();
    return;
  }
  Serial.println(" OK");

  // Initialize SMS mode for SIM800L
  SerialAT.println("AT"); //Once the handshake test is successful, it will back to OK
  updateSerial();
  delay(200);
  SerialAT.println("AT+CMGF=1"); // Configuring TEXT mode
  updateSerial();
  delay(200);
  SerialAT.println("AT+CNMI=2,2,0,0,0"); // Decides how newly arrived SMS messages should be handled
  updateSerial();
  
  //RS-485 and BECO set to HIGH
  Serial.println("RS-485 is HIGH");
  digitalWrite(SW_RS485, HIGH);

  // Check battery level
  batterylevel();

  // Initate MODBUS Comms
  master.start();
  master.setTimeOut( 5000 ); // if there is no answer in 5000 ms, roll over
  u32wait = millis() + 1000;
  u8state = 0;
  u8query = 0;

  // OTA Drive begin
  SPIFFS.begin(true);
  OTADRIVE.setInfo("1583b8f7-1702-45dd-8151-68e5b07de8d6", "v@1.0.7");
  OTADRIVE.onUpdateFirmwareProgress(update_prgs);

  Serial.printf("Download a new firmware from SIM800, V=%s\n", OTADRIVE.Version.c_str());
}

/************************************************************************
 * Main loop
 ************************************************************************/
void loop() {
  switch( u8state ) {
  case 0:
    //Serial.println("Case 0");
    updateSMS();
    if (millis() > u32wait) {
      lastMillisCase1 = millis();
      u8state++; // wait state
    }
    break;
  case 1:
    //Serial.println("Case 1");
    updateSMS(); 
    master.query( telegram[u8query] ); // send query (only once)
    if (millis() - lastMillisCase1 >= 1000){
      lastMillisCase1 = millis();
      Serial.print("Query: ");
      Serial.println(u8query);
      u8state++;
      u8query++;
    }

    if (u8query > 2) u8query = 0;
    lastMillisCase2 = millis();
    break;
  case 2:
    //Serial.println("Case 2");
    updateSMS();
    if (millis() - lastMillisCase2 >= 1000) {
      lastMillisCase2 = millis();
      master.poll();
      // Check if communication is idle
      if (master.getState() == COM_IDLE) {
        // No response received, handle this case
        u8response++;
        Serial.println("No response from Modbus device");
        if (String(au16data[0]) == "0" && u8response <= 5){
          // Back to Case 1 and Query still 0
          Serial.println("au16data[0] no data, back to u8state 1 and u8query 0");
          u8state = 1;
          u8query = 0;
        }
        else if (String(au16data[2]) == "0" && u8response <= 10){
          // Back to Case 1 and Query still 1
          Serial.println("au16data[2] no data, back to u8state 1 and u8query 1");
          u8state = 1;
          u8query = 1;
        }
        else if(String(au16data[2]) == "0" && u8response == 15){
          Serial.println("No response from Modbus device for long time, going deep sleep...");
          checkNetwork();
          // Set this to 0 for real situation
          float accVolume = 0;
          float Debit = 0;

          updateSMS();
          sendHttpPostRequest(accVolume, Debit, voltageLevel);
          
          if (setup_mode == true){
            Serial.println("Back to Case 0");
            u8state = 0;
            u8query = 0;
            u8response = 0;
            u32wait = millis() + 1000;
          }
          else{
            Serial.println("Updating firmware...");
            OTADRIVE.updateFirmware(gsm_otadrive_client);
            Serial.println("Update finished");
            modem.gprsDisconnect();
            Serial.println("GPRS disconnected");
            switchOffTrans();
            Serial.println("All devices going low...");
            preferences.putInt("lastState", 0);
            preferences.end();
            Serial.print("Going deep sleep for: ");
            Serial.print(sleepTime);
            Serial.println(" seconds");
            esp_sleep_enable_timer_wakeup(1000000ULL * sleepTime);
            esp_deep_sleep_start();
          }
        }
        else{
          // Back to Case 1 and Query still 2
          u8state = 1;
          u8query = 2;
        }
        
        lastMillisCase1 = millis();
        //u32wait = millis() + 1000;
      }
      else {
        // Response received, proceed to the next state
        u8state++;
      }
    }
    break;
  case 3:
    updateSMS();
    Serial.println("Case 3");
    // Modbus data handler
    if (String(au16data[6])  != "0"){
      Serial.println("Millis: " + String(millis()));
      for (int i = 0; i < 8; i++) {
        Serial.println(String(i) + ": " + String(au16data[i]));
      }
      float Debit = reg_to_float(au16data[0], au16data[1]);
      float Vraw = reg_to_dec(au16data[3], au16data[2]);
      float Vfraction = reg_to_float(au16data[5], au16data[4]);
      float V = Vraw + Vfraction;
      int n = int(au16data[7]);
      int n2 = n-3;
      int npos = n2 * -1;
      float nfix = 1.0 / pow(10, npos);
      float Vfix = V * nfix;
      Serial.print("Volume: ");
      Serial.print(Vfix, 4);
      Serial.println("L");
      Serial.print("Debit: ");
      Serial.print(Debit, 4);
      Serial.println("m3/h");

      // Save data in preferences
      preferences.begin("flowmeter", false);
      lastVolume = preferences.getFloat("lastVolume", 0);
      Serial.print("Last volume : ");
      Serial.println(lastVolume);
      float accVolume = Vfix - lastVolume;
      Serial.print("Accumulated volume : ");
      Serial.println(accVolume);
      preferences.putFloat("lastVolume", Vfix);

      if (!OTADRIVE.timeTick(30))
      {
        delay(3000);
        return;
      }

      checkNetwork();
      // auto a = OTADRIVE.updateFirmwareInfo(gsm_otadrive_client);
      // Serial.printf("info: %d, %d, %s\n", a.available, a.size, a.version.c_str());
      // auto c = OTADRIVE.getConfigs(gsm_otadrive_client);
      // Serial.printf("config %s\n", c.c_str());
      // OTADRIVE.sendAlive(gsm_otadrive_client);
      sendHttpPostRequest(accVolume, Debit, voltageLevel);

      if (BATT_LOW == true){
        // Going deep sleep until the battery replaced
        Serial.println("Battery is LOW, going deep sleep until battery replaced");
        switchOffTrans();
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
        esp_deep_sleep_start();
      }
      else if (setup_mode == true){
        Serial.println("Back to Case 0");
        u8state = 0;
        u8query = 0;
        u8response = 0;
        u32wait = millis() + 1000;
        for (int i = 0; i < 8; i++){
          au16data[i] = 0;
        }
      }
      else{
        Serial.println("Updating firmware...");
        OTADRIVE.updateFirmware(gsm_otadrive_client);
        modem.gprsDisconnect();
        switchOffTrans();
        preferences.putInt("lastState", 0);
        preferences.end();
        esp_sleep_enable_timer_wakeup(1000000ULL * sleepTime);
        esp_deep_sleep_start();
      }
    }
    else{
      // Back to case 0 if there is no data
      Serial.println("No data");
      u8state = 0;
      u32wait = millis() + 1000;
    }
    break;
  }
}