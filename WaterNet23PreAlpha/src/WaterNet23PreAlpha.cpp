/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#line 1 "c:/Users/mligh/OneDrive/Particle/WaterNet23PreAlpha/src/WaterNet23PreAlpha.ino"
/*
 * Project WaterNet23PreAlpha
 * Description: Initial code for B404 with GPS and serial communications
 * Date: 3/18/2022
 */

#include "application.h"                    //Needed for I2C to GPS
#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_Ublox_GPS
#include <MicroNMEA.h>                      //http://librarymanager/All#MicroNMEA
#include "SdFat.h"
#include "sdios.h"

////////////////////
// PROGRAM MACROS //
////////////////////

void cmdLTEHandler(const char *event, const char *data);
void setup();
void loop();
void sendData(const char *dataOut, uint8_t dataSize, bool sendBLE, bool sendXBee, bool &sendLTE);
void testConnection(bool checkBLE, bool checkXBee, bool checkLTE);
static void BLEDataReceived(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context);
void wdogHandler();
void LEDHandler();
#line 17 "c:/Users/mligh/OneDrive/Particle/WaterNet23PreAlpha/src/WaterNet23PreAlpha.ino"
#define UART_TX_BUF_SIZE    30
#define SCAN_RESULT_COUNT   20

#define PHADDR              99               //default I2C ID number for EZO pH Circuit.
#define MCOND               100               //default I2C ID number for EZO Mini-Conductivity (0.1)
#define COND                101               //default I2C ID number for EZO pH Circuit. (1.0)
#define TEMPADDR            102
#define SENS_POLL_RT        2500
#define SENS_DATA_DLY       825

#define WATCHDOG_PD         15000           //Watchdog timer period in milliseconds
#define XBEE_WDOG_AVAIL     30000           //Watchdog interval between XBee messages for availablility check
#define BLE_WDOG_AVAIL      30000           //Watchdog interval between BLE messages for availability check

#define DEF_FILENAME        "WaterBot"
#define FILE_LABELS         "Time,Latitude,Longitude,Temperature,pH,Dissolved O2,Conductivity 0.1K,Conductivity 1K"
#define BLE_OFFLD_BUF       100
#define CUSTOM_DATA_LEN     8
#define MAX_FILENAME_LEN    30

#define chipSelect D8//A5

SYSTEM_MODE(MANUAL);

//GPS Buffers and Objects
char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));
SFE_UBLOX_GPS myGPS;

//SD File system object
SdFat sd((SPIClass*)&SPI1);

File myFile;
File logFile;
File logDir;

SerialLogHandler logHandler(LOG_LEVEL_INFO);                         //Log Configuration

///////////////////////
// BLE Configuration //
///////////////////////

const char* WaterNetService = "b4206910-dc4b-5743-c8b1-92d0e75182b0"; //Main BLE Service
const char* rxUuid          = "b4206912-dc4b-5743-c8b1-92d0e75182b0"; //GPS Latitude Service
const char* txUuid          = "b4206913-dc4b-5743-c8b1-92d0e75182b0"; //GPS Longitude Service
const char* offldUuid       = "b4206914-dc4b-5743-c8b1-92d0e75182b0"; //GPS Longitude Service

BleCharacteristic txCharacteristic("tx", BleCharacteristicProperty::NOTIFY, txUuid, WaterNetService);
BleCharacteristic rxCharacteristic("rx", BleCharacteristicProperty::WRITE_WO_RSP, rxUuid, WaterNetService, BLEDataReceived, NULL);
BleCharacteristic offloadCharacteristic("off", BleCharacteristicProperty::NOTIFY, offldUuid, WaterNetService);

BleScanResult scanResults[SCAN_RESULT_COUNT];

BleCharacteristic peerTxCharacteristic;
BleCharacteristic peerRxCharacteristic;
BleCharacteristic offldCopyCharacteristic;
BlePeerDevice peer;
BleAdvertisingData advData;                 //Advertising data
uint8_t BLECustomData[CUSTOM_DATA_LEN];

const unsigned long SCAN_PERIOD_MS = 2000;
unsigned long lastScan = 0;

//Function prototypes
void setupSPI();
void setupMotors();
void setupLTE();
void setupXBee();
void setupGPS();
bool getGPSLatLon();
void timeInterval();
void updateMotors();
void sendData(const char *event, uint8_t dataSize, bool sendBLE, bool sendXBee, bool &sendLTE);
void XBeeHandler();
void processCommand(const char *command, uint8_t mode, bool sendAck);
void sensorHandler();
void dataOffloader();

//Tmers
Timer watchdog(WATCHDOG_PD, wdogHandler);   //Create timer object for watchdog
Timer ledTimer(1000,LEDHandler);

//LED Control
LEDStatus status;

//////////////////////
// Global Variables //
//////////////////////

long latitude_mdeg;
long longitude_mdeg;
bool sendLatLonLTE;
uint8_t leftMotorSpeed, setLSpeed;
uint8_t rightMotorSpeed, setRSpeed;
bool updateMotorControl;
bool manualRC;
bool lowBattery;
bool LTEAvail, XBeeAvail, BLEAvail, GPSAvail;     //Flags for communicaton keep-alives/available
bool logSensors, logMessages, dataWait; //Flags for sensor timing/enables
bool offloadMode;
uint32_t senseTimer, dataTimer;
uint32_t XBeeRxTime, BLERxTime;
float sensePH, senseTemp, senseCond, senseMiniCond, senseDO;
char txBuf[UART_TX_BUF_SIZE];
size_t txLen = 0;
char filename[MAX_FILENAME_LEN];
char filenameMessages[MAX_FILENAME_LEN];

String xbeeBuf;

int i = 0;
///////////////////////////
// BOT NUMBER DEFINITION //
///////////////////////////
#define BOTNUM 1


/*class PeerBot{
    public:
        uint8_t botNumber;  //Bot ID of other bot
        bool LTEConnected;  //Status if other bot is connected to LTE
        bool XBeeConnected; //Status if other bot is receiving XBee data
        uint8_t BLEPeer;    //Identify which peer this bot is connected to

};*/

void processCommand(const char *command, uint8_t mode, bool sendAck){
    //Process if command is addressed to this bot "Bx" or all bots "AB"
    if((command[2] == 'A' && command[3] == 'B') || (command[2] == 'B' && command[3] == BOTNUM+48)){
        char dataStr[strlen(command)-7];
        char cmdStr[3];
        for(uint8_t i = 4; i < strlen(command);i++){
            if(i < 7) cmdStr[i-4] = command[i];
            else dataStr[i-7] = command[i];
        }
        if(!strcmp(cmdStr,"ack")){  //Acknowledgement for XBee and BLE
            if(mode == 1){  //Acknowledge from XBee

            }
            else if(mode == 2){ //Acknowledge from BLE
                
            }
            return;
        }
        if(!strcmp(cmdStr,"mtr")){  //Motor Speed Control
            char lSpd[3] = {dataStr[0],dataStr[1],dataStr[2]};
            char rSpd[3] = {dataStr[3],dataStr[4],dataStr[5]};
            setLSpeed = atoi(lSpd);
            setLSpeed = atoi(rSpd);
        }
        else if(!strcmp(cmdStr,"gps")){  //Received GPS data
            digitalWrite(D7, HIGH);
        }
        else if(!strcmp(cmdStr,"sns")){  //Received Sensor data 
            digitalWrite(D7, LOW);
        }
        else if(!strcmp(cmdStr,"req")){  //Data Request
            
        }
        else if(!strcmp(cmdStr,"pts")){
            Serial.println(dataStr);
            myFile.open("RawWrite.txt", O_RDWR | O_CREAT | O_AT_END);
            String modeStr[3] = {"LTE", "XBee", "Bluetooth"};
            myFile.printf("New string from %s: ", modeStr[mode]);
            myFile.println(dataStr);
            delay(5);
            myFile.close();
        }
        else if(!strcmp(cmdStr,"ccs")){  //Incoming communication status
            
        }
        else if(!strcmp(cmdStr,"aut")){  //Enter autonomous mode
            
        }
        else if(!strcmp(cmdStr,"dmp")){  //Enter SD Card "Dump Mode"
            offloadMode = true;
            status.setPattern(LED_PATTERN_BLINK);
            status.setColor(RGB_COLOR_BLUE);
            status.setSpeed(LED_SPEED_FAST);
        }
        else{   //Didn't recognize command (may be corrupted?) send back error signal
            if(mode == 1){

            }
            if(mode == 2){

            }
        }

        if(sendAck){    //Transmit out acknowledgement if needed

        }

    }
}

void cmdLTEHandler(const char *event, const char *data){
    processCommand(data, 0,false);
    if(logMessages){
        if(!logFile.isOpen()) logFile.open(filenameMessages, O_RDWR | O_CREAT | O_AT_END);
        logFile.printlnf("[INFO] Received BLE Message: %s",data);
        logFile.close();
    }
}

void setup(){
    status.setPriority(LED_PRIORITY_IMPORTANT);
    status.setActive(true);

    BLE.on();
    
    //Log.info("Hello from WaterNet23!");
    Serial.begin();
    Serial1.begin(9600, SERIAL_PARITY_EVEN);                        //Start serial for XBee module
    setupSPI();                                 //Setup SPI for BeagleBone
    setupXBee();                                //Setup XBee module
    setupMotors();                              //Setup Motor controller chips over I2C
    setupGPS();                                 //Setup GPS module
    setupLTE();                                 //Initialize LTE Flags

    manualRC = true;

    senseTimer = millis();
    dataTimer = millis();
    dataWait = false;
    logSensors = true;
    logMessages = true;
    offloadMode = false;

    BLE.addCharacteristic(txCharacteristic);    //Add BLE Characteristics for BLE serial
    BLE.addCharacteristic(rxCharacteristic);
    BLE.addCharacteristic(offloadCharacteristic);

    char deviceName[10];
    strcpy(deviceName,"WTN23_Bot");
    deviceName[9] = BOTNUM+48;
    BLECustomData[0] = BOTNUM;

    advData.appendServiceUUID(WaterNetService); // Add the app service
    advData.appendCustomData(BLECustomData,CUSTOM_DATA_LEN);
    //advData.appendLocalName(deviceName);           //Local advertising name

    BLE.advertise(&advData);                    //Start advertising the characteristics
    Wire.begin();
    Wire.setClock(CLOCK_SPEED_400KHZ);

    char timestamp[16];
    snprintf(timestamp,16,"%02d%02d%04d%02d%02d%02d",Time.month(),Time.day(),Time.year(),Time.hour(),Time.minute(),Time.second());
    strcpy(filename,DEF_FILENAME);
    strcat(filename,timestamp);
    strcpy(filenameMessages,filename);
    strcat(filename,".csv");
    strcat(filenameMessages,"_LOG.txt");

    Serial.println(filename);
    Serial.println(filenameMessages);

    watchdog.start();
    ledTimer.start();

    if (!sd.begin(chipSelect, SD_SCK_MHZ(4))) {
        Serial.println("Error: could not connect to SD card!");
        logSensors = false;
        logMessages = false;
    }
    if(logSensors){
        if(!myFile.isOpen()) myFile.open(filename, O_RDWR | O_CREAT | O_AT_END);
        myFile.println(FILE_LABELS);
        myFile.close();
    }
    if(logMessages){
        if(!logFile.isOpen()) logFile.open(filenameMessages, O_RDWR | O_CREAT | O_AT_END);
        logFile.printlnf("[INFO] WaterBot %d: Started Logging",BOTNUM);
        logFile.close();
    }
    // delete possible existing file
}

void loop(){
    if(getGPSLatLon()){
        //Log.info("Latitude (deg): %0.6f", ((float)latitude_mdeg/1000000.0));
        //Log.info("Longitude (deg): %0.6f", ((float)longitude_mdeg/1000000.0));
        GPSAvail = true;
        char latLonBuf[UART_TX_BUF_SIZE];
        sprintf(latLonBuf, "GPS Data: Lat:%0.6f Lon:%0.6f\n", ((float)latitude_mdeg/1000000.0),((float)longitude_mdeg/1000000.0));
        Serial.println(latLonBuf);
        //sendData(latLonBuf, UART_TX_BUF_SIZE, true, true, sendLatLonLTE);
    }
    else GPSAvail = false;
    //Serial.println("bing bong");
    //sensorHandler();
    //XBeeHandler();
    //updateMotors();
    if(offloadMode) dataOffloader();
    delay(100);
}

//This function gets called from the SparkFun Ublox Arduino Library
//As each NMEA character comes in you can specify what to do with it
//Useful for passing to other libraries like tinyGPS, MicroNMEA, or even
//a buffer, radio, etc.
void SFE_UBLOX_GPS::processNMEA(char incoming){
  //Take the incoming char from the Ublox I2C port and pass it on to the MicroNMEA lib
  //for sentence cracking
  nmea.process(incoming);
}

void setupMotors(){
    updateMotorControl = false;
}
//Initialization for LTE events and flags
void setupLTE(){
    Particle.subscribe("CCHub", cmdLTEHandler); //Subscribe to LTE data from Central Control Hub
    sendLatLonLTE = false;                      //Set LTE flags initially false
    LTEAvail = false;
}

void setupSPI(){
    SPI.begin(SPI_MODE_MASTER);
    SPI.setClockSpeed(1000000);
}
//Code to initially configure XBee module over serial
void setupXBee(){
    Serial1.printf("\n");    //First character to set Bypass mode
    delay(20);              //Wait some time before sending next character
    Serial1.printf("B");     //Second character to set Bypass mode
    delay(20);
    Serial1.printf("Hello from Bot %d\n", BOTNUM);   //Send Hello World message!
}

//I2C setup for NEO-M8U GPS
void setupGPS(){
    myGPS.begin(Wire);
    if (myGPS.isConnected() == false){
        //Log.warn("Ublox GPS not detected at default I2C address, freezing.");
        //while (1);
    }
    Wire.setClock(400000); //Increase I2C clock speed to 400kHz
}

bool getGPSLatLon(){
    myGPS.checkUblox(); //See if new data is available. Process bytes as they come in.

  if(nmea.isValid() == true){
    latitude_mdeg= nmea.getLatitude();
    longitude_mdeg = nmea.getLongitude();
    return true;
  }
  else{
    //Log.warn("Location not available: %d Sattelites",nmea.getNumSatellites());
  }
  return false;
}

void updateMotors(){
    if(updateMotorControl){
        updateMotorControl = false;        
    }
}

void sendData(const char *dataOut, uint8_t dataSize, bool sendBLE, bool sendXBee, bool &sendLTE){
    if(sendLTE){
        Particle.publish("Bot1dat", dataOut, PRIVATE);
        sendLTE = false;
    }
    if(sendBLE && BLE.connected()){
        uint8_t txBuf_tmp[UART_TX_BUF_SIZE];
        for(int i = 0; i < dataSize; i++) txBuf_tmp[i] = dataOut[i];
        if(dataSize < UART_TX_BUF_SIZE) txCharacteristic.setValue(txBuf_tmp, dataSize);
        else txCharacteristic.setValue(txBuf_tmp, UART_TX_BUF_SIZE);
    }
    if(sendXBee){
        Serial1.println(dataOut);
    }
}
void sensorHandler(){
    
    if(dataTimer < millis() && dataWait){
        Wire.requestFrom(PHADDR, 20, 1);
        byte code = Wire.read();               		                                      //the first byte is the response code, we read this separately.
        char tempSense[20];
        int c = 0;
        while(Wire.available()){   // slave may send less than requested
            tempSense[c++] = Wire.read();

        }
        sensePH = atof(tempSense);
        Serial.printlnf("pH: %f", sensePH);
        Wire.requestFrom(MCOND, 20, 1);
        code = Wire.read();               		                                      //the first byte is the response code, we read this separately.
        char mcondSense[20];
        c = 0;
        while(Wire.available()){   // slave may send less than requested
            mcondSense[c++] = Wire.read();

        }
        float senseMCond = atof(mcondSense);
        Serial.printlnf("MiniCond: %f",senseMCond);
        Wire.requestFrom(COND, 20, 1);
        code = Wire.read();               		                                      //the first byte is the response code, we read this separately.
        char condSense[20];
        c = 0;
        while(Wire.available()){   // slave may send less than requested
            condSense[c++] = Wire.read();

        }
        float senseCond = atof(condSense);
        Serial.printlnf("Conductivity: %f",senseCond);
        Wire.requestFrom(TEMPADDR, 20, 1);
        code = Wire.read();               		                                      //the first byte is the response code, we read this separately.
        char addrSense[20];
        c = 0;
        while(Wire.available()){   // slave may send less than requested
            addrSense[c++] = Wire.read();

        }
        float senseTemp = atof(addrSense);
        Serial.printlnf("Temperature: %f",senseTemp);
        dataWait = false;
        if(logSensors){
            char timestamp[16];
            snprintf(timestamp,16,"%02d%02d%04d%02d%02d%02d",Time.month(),Time.day(),Time.year(),Time.hour(),Time.minute(),Time.second());
            if(!myFile.isOpen()) myFile.open(filename, O_RDWR | O_CREAT | O_AT_END);
            myFile.printlnf("%s,%f,%f,%f,%f,%f,%f,%f",timestamp,((float)latitude_mdeg/1000000.0),((float)longitude_mdeg/1000000.0),senseTemp,sensePH,senseDO,senseMiniCond,senseCond);
            myFile.close();
        }
    }
    if(senseTimer < millis()){
        senseTimer = millis() + SENS_POLL_RT;
        dataTimer = millis() + SENS_DATA_DLY;
        Wire.beginTransmission(PHADDR);                                              //call the circuit by its ID number.
        Wire.write('r');                                                     //transmit the command that was sent through the serial port.
        Wire.endTransmission();                                                       //end the I2C data transmission.
        Wire.beginTransmission(MCOND);                                              //call the circuit by its ID number.
        Wire.write('r');                                                     //transmit the command that was sent through the serial port.
        Wire.endTransmission();                                                       //end the I2C data transmission.
        Wire.beginTransmission(COND);                                              //call the circuit by its ID number.
        Wire.write('r');                                                     //transmit the command that was sent through the serial port.
        Wire.endTransmission();                                                       //end the I2C data transmission.
        Wire.beginTransmission(TEMPADDR);                                              //call the circuit by its ID number.
        Wire.write('r');                                                     //transmit the command that was sent through the serial port.
        Wire.endTransmission();                                                       //end the I2C data transmission.
        dataWait = true;
    }
}
void XBeeHandler(){  
    while(Serial1.available()){
        String data = Serial1.readStringUntil('\n');
        char buffer[data.length()];
        for(int i = 0 ; i < data.length(); i++) buffer[i] = data.charAt(i);
        processCommand(buffer,1,true);
        Serial.println("New XBee Command:");
        Serial.println(data); 
        XBeeRxTime = millis();
        if(logMessages){
            if(!logFile.isOpen()) logFile.open(filenameMessages, O_RDWR | O_CREAT | O_AT_END);
            logFile.printlnf("[INFO] Received XBee Message: %s",data);
            logFile.close();
        }
    }
}

void testConnection(bool checkBLE, bool checkXBee, bool checkLTE){
    if(checkBLE){
        BLEAvail = false;
        if(BLE.connected()){
            uint8_t txBuf_tmp[7] = {'B',BOTNUM+48,'A','B','c','c','s'};
            txCharacteristic.setValue(txBuf_tmp, 7);
        }
    }
    if(checkXBee){
        Serial1.printf("B%dABccs\n", BOTNUM);
    }
    if(checkLTE){
        char lteBuf[7] = {'B',BOTNUM+48,'A','B','c','c','s'};
        Particle.publish("WNS", lteBuf, PRIVATE);
    }
}

static void BLEDataReceived(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context) {
    char btBuf[len];
    for (size_t ii = 0; ii < len; ii++) btBuf[ii] = data[ii];
    Serial.println("New BT Command:");
    Serial.println(btBuf);
    processCommand(btBuf,2,true);
    BLERxTime = millis();
    if(logMessages){
        if(!logFile.isOpen()) logFile.open(filenameMessages, O_RDWR | O_CREAT | O_AT_END);
        logFile.printlnf("[INFO] Received BLE Message: %s",btBuf);
        logFile.close();
    }
}

void wdogHandler(){
    if(Particle.connected()) LTEAvail = true;
    else if(LTEAvail){
        if(!logFile.isOpen()) logFile.open(filenameMessages, O_RDWR | O_CREAT | O_AT_END);
        logFile.printlnf("[WARN] XBee Messages have not been received in %ds, assuming XBee is unavailable",(XBEE_WDOG_AVAIL/1000));
        LTEAvail = false;
    }
    if(millis()-XBeeRxTime > XBEE_WDOG_AVAIL){
        if(XBeeAvail){
            if(!logFile.isOpen()) logFile.open(filenameMessages, O_RDWR | O_CREAT | O_AT_END);
            logFile.printlnf("[WARN] XBee Messages have not been received in %ds, assuming XBee is unavailable",(XBEE_WDOG_AVAIL/1000));
        }
        XBeeAvail = false;
    }
    else XBeeAvail = true;
    if(millis()-BLERxTime > BLE_WDOG_AVAIL){
        if(BLEAvail){
            if(!logFile.isOpen()) logFile.open(filenameMessages, O_RDWR | O_CREAT | O_AT_END);
            logFile.printlnf("[WARN] BLE Messages have not been received in %ds, assuming BLE is unavailable",(BLE_WDOG_AVAIL/1000));
        }
        BLEAvail = false;
    }
    else BLEAvail = true;
}

void dataOffloader(){
    if (!logDir.open("/")) {
        offloadMode = false;
        return;
    }
    while(!BLE.connected()){
        BLE.advertise(&advData);;
        delay(10);
    }
    char fileCode[8 + MAX_FILENAME_LEN];
    uint8_t codeBuf[8 + MAX_FILENAME_LEN];
    while (myFile.openNext(&logDir, O_RDONLY) && BLE.connected()) {
        char namebuf[MAX_FILENAME_LEN];
        myFile.getName(namebuf,MAX_FILENAME_LEN);
        Serial.printlnf("Checking if file %s is a .csv or .txt...", namebuf);
        if(!strstr(strlwr(namebuf + (strlen(namebuf) - 4)), ".csv")){
            myFile.close();
            continue;
        }
        else{
            memset(fileCode,0,8+MAX_FILENAME_LEN);
            strcpy(fileCode,"filename");
            strcat(fileCode,namebuf);
            Serial.printlnf("Sending command %s",fileCode);
            offloadCharacteristic.setValue(fileCode);
            Serial.printlnf("File %s is a .csv or .txt printing data", namebuf);
            delay(150);
            while(myFile.available()){
                char lineBuffer[BLE_OFFLD_BUF];
                memset(lineBuffer,0,BLE_OFFLD_BUF);
                myFile.readBytesUntil('\r',lineBuffer,BLE_OFFLD_BUF);
                //uint8_t lineBytes[strlen(lineBuffer)];
                //memcpy(lineBytes,lineBuffer,strlen(lineBuffer));
                offloadCharacteristic.setValue(lineBuffer);
                delayMicroseconds(500);
                //delay(1);
                //Serial.println(lineBuffer);
            }
            memset(fileCode,0,8+MAX_FILENAME_LEN);
            strcpy(fileCode,"filecomp");
            //memcpy(codeBuf,fileCode,8+MAX_FILENAME_LEN);
            offloadCharacteristic.setValue(fileCode);
            delay(150);
            
            myFile.close();
        }
    }
    logDir.close();
    memset(fileCode,0,8+MAX_FILENAME_LEN);
    strcpy(fileCode,"filedone");
    //memcpy(codeBuf,fileCode,8+MAX_FILENAME_LEN);
    offloadCharacteristic.setValue(fileCode);
    //BLE.disconnect();
    offloadMode = false;
}

void LEDHandler(){
    uint32_t SetColor;
    LEDPattern SetPattern;
    LEDSpeed SetSpeed;
    uint8_t statusMode;
    if(offloadMode){
        status.setPattern(LED_PATTERN_BLINK);
        status.setColor(RGB_COLOR_BLUE);
        status.setSpeed(LED_SPEED_FAST);
        return;                
    }
    if(lowBattery){
        SetPattern = LED_PATTERN_BLINK;
        SetSpeed = LED_SPEED_NORMAL;
    }
    else if(logSensors){
        SetPattern = LED_PATTERN_SOLID;
        SetSpeed = LED_SPEED_NORMAL;
    }
    else if(manualRC){
        SetPattern = LED_PATTERN_BLINK;
        SetSpeed = LED_SPEED_SLOW;
    }
    else{
        SetSpeed = LED_SPEED_NORMAL;
        SetPattern = LED_PATTERN_FADE;
    }

    statusMode = LTEAvail;
    statusMode |= XBeeAvail << 1;
    statusMode |= BLEAvail << 2;
    Serial.printlnf("Status: %d",statusMode);
    switch (statusMode){
    case 7:
        SetColor = RGB_COLOR_CYAN;
        break;
    case 6:
        SetColor = RGB_COLOR_YELLOW;
        break;
    case 5:
        SetColor = RGB_COLOR_WHITE;
        break;
    case 4:
        SetColor = RGB_COLOR_BLUE;
        break;
    case 3:
        SetColor = RGB_COLOR_GREEN;
        break;
    case 2:
        SetColor = RGB_COLOR_ORANGE;
        break;
    case 1:
        SetColor = RGB_COLOR_MAGENTA;
        break;
    case 0:
        SetColor = RGB_COLOR_RED;
        break;
    default:
        break;
    }
    status.setPattern(SetPattern);
    status.setColor(SetColor);
    status.setSpeed(SetSpeed);    
}
