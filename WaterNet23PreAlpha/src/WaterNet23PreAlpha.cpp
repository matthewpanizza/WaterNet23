/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#line 1 "/Users/matthewpanizza/Downloads/WaterNet23/WaterNet23PreAlpha/src/WaterNet23PreAlpha.ino"
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
void cmdLTEHandler(const char *event, const char *data);
void setup();
void loop();
void StatusHandler();
void testConnection(bool checkBLE, bool checkXBee, bool checkLTE);
static void BLEDataReceived(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context);
void wdogHandler();
void LEDHandler();
void BLEScan(int BotNumber);
#line 12 "/Users/matthewpanizza/Downloads/WaterNet23/WaterNet23PreAlpha/src/WaterNet23PreAlpha.ino"
#undef min
#undef max
#include <vector>

//////////////////////////////
// BOT CONFIGURATION MACROS //
//////////////////////////////

#define BOTNUM 1
#define STARTUP_WAIT_PAIR 0
#define ESC_PWM_L D4
#define ESC_PWM_R D5
#define chipSelect D8

////////////////////
// PROGRAM MACROS //
////////////////////

#define UART_TX_BUF_SIZE    30
#define SCAN_RESULT_COUNT   20
#define MAX_ERR_BUF_SIZE    15              //Buffer size for error-return string

#define PHADDR              99               //default I2C ID number for EZO pH Circuit.
#define MCOND               100               //default I2C ID number for EZO Mini-Conductivity (0.1)
#define COND                101               //default I2C ID number for EZO pH Circuit. (1.0)
#define TEMPADDR            102
#define SENS_POLL_RT        2500
#define SENS_DATA_DLY       825

#define WATCHDOG_PD         15000           //Watchdog timer period in milliseconds
#define STATUS_PD           15000            //Time between status updates published to CC Hub
#define XBEE_WDOG_AVAIL     30000           //Watchdog interval between XBee messages for availablility check
#define BLE_WDOG_AVAIL      30000           //Watchdog interval between BLE messages for availability check
#define LTE_MAX_STATUS      480             // (Divided by LTE STAT PD) Maximum number of status messages to send over LTE if other methods are unavailable
#define LTE_STAT_PD         4               //Divider for sending status via LTE to reduce data usage
#define XBEE_START_PUB      5000            //Time period between sending "Hello World" messages over XBee during setup

#define DEF_FILENAME        "WaterBot"
#define FILE_LABELS         "Time,Latitude,Longitude,Temperature,pH,Dissolved O2,Conductivity 0.1K,Conductivity 1K"
#define BLE_OFFLD_BUF       100
#define CUSTOM_DATA_LEN     8
#define MAX_FILENAME_LEN    30

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

Servo ESCL;     // create servo object to control the left motor ESC
Servo ESCR;     // create servo object to control the right motor ESC

///////////////////////
// BLE Configuration //
///////////////////////

class PairBot{
    public:
    uint8_t botNum;
    int rssi;
};

const char* WaterNetService = "b4206910-dc4b-5743-c8b1-92d0e75182b0"; //Main BLE Service
const char* rxUuid          = "b4206912-dc4b-5743-c8b1-92d0e75182b0"; //GPS Latitude Service
const char* txUuid          = "b4206913-dc4b-5743-c8b1-92d0e75182b0"; //GPS Longitude Service
const char* offldUuid       = "b4206914-dc4b-5743-c8b1-92d0e75182b0"; //GPS Longitude Service

const BleUuid serviceUuid("b4206910-dc4b-5743-c8b1-92d0e75182b0");
const BleUuid peerRxUuid("b4206912-dc4b-5743-c8b1-92d0e75182b0");
const BleUuid peerTxUuid("b4206913-dc4b-5743-c8b1-92d0e75182b0");

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
std::vector<PairBot> BLEPair;

//Function prototypes
void setupSPI();
void setupLTE();
void setupXBee();
void setupGPS();
bool getGPSLatLon();
void sendResponseData();
void timeInterval();
void updateMotors();
void sendData(const char *dataOut, uint8_t sendMode, bool sendBLE, bool sendXBee, bool sendLTE);
void XBeeHandler();
void processCommand(const char *command, uint8_t mode, bool sendAck);
void sensorHandler();
void dataOffloader();
void statusUpdate();

//Tmers
Timer watchdog(WATCHDOG_PD, wdogHandler);   //Create timer object for watchdog
Timer ledTimer(1000,LEDHandler);
Timer statusPD(STATUS_PD,StatusHandler);

//LED Control
LEDStatus status;

//////////////////////
// Global Variables //
//////////////////////

bool waitForConnection;
long latitude_mdeg, longitude_mdeg;
float latitude, longitude;
uint8_t leftMotorSpeed, setLSpeed;
uint8_t rightMotorSpeed, setRSpeed;
uint8_t battPercent;
bool updateMotorControl;
bool manualRC;
bool lowBattery;
bool statusReady;
uint8_t requestActive;
uint16_t LTEStatusCount;
uint8_t statusFlags;
bool LTEAvail, XBeeAvail, BLEAvail;     //Flags for communicaton keep-alives/available
bool logSensors, logMessages, dataWait; //Flags for sensor timing/enables
bool offloadMode;
uint32_t senseTimer, dataTimer;
uint32_t XBeeRxTime, BLERxTime;
float sensePH, senseTemp, senseCond, senseMiniCond, senseDO;
char txBuf[UART_TX_BUF_SIZE];
char errBuf[MAX_ERR_BUF_SIZE];
uint8_t errModeReply;
size_t txLen = 0;
char filename[MAX_FILENAME_LEN];
char filenameMessages[MAX_FILENAME_LEN];

String xbeeBuf;

int i = 0;


/*class PeerBot{
    public:
        uint8_t botNumber;  //Bot ID of other bot
        bool LTEConnected;  //Status if other bot is connected to LTE
        bool XBeeConnected; //Status if other bot is receiving XBee data
        uint8_t BLEPeer;    //Identify which peer this bot is connected to

};*/

//Mode 1 - BLE, Mode 2 - XBEE, Mode 4 - LTE
void processCommand(const char *command, uint8_t mode, bool sendAck){
    //Process if command is addressed to this bot "Bx" or all bots "AB"
    if((command[2] == 'A' && command[3] == 'B') || (command[2] == 'B' && command[3] == BOTNUM+48)){
        
        uint8_t checksum;
        char dataStr[strlen(command)-8];
        dataStr[strlen(command)-9] = '\0';
        char cmdStr[4];
        cmdStr[3] = '\0';
        char checkStr[3];
        checkStr[0] = command[strlen(command)-2];
        checkStr[1] = command[strlen(command)-1];
        checkStr[2] = '\0';
        checksum = (uint8_t)strtol(checkStr, NULL, 16);       // number base 16
        Serial.printlnf("Checksum: %02x, %03d",checksum,checksum);
        for(uint8_t i = 4; i < strlen(command)-2;i++){
            if(i < 7) cmdStr[i-4] = command[i];
            else dataStr[i-7] = command[i];
        }
        if(checksum != strlen(command)-2){
            Serial.printlnf("String Len: %d, Checksum: %d",strlen(command)-2,checksum);
            if(!logFile.isOpen()){
                logFile.open(filenameMessages, O_RDWR | O_CREAT | O_AT_END);
                logFile.printlnf("[WARN] Message Checksum Does Not Match!: %s",command);
                logFile.close();
            }
            else logFile.printlnf("[WARN] Message Checksum Does Not Match!: %s",command);
            Serial.println("Warning, checksum does not match");
            if((command[1] >= '0' && command[1] <= '9') || command[1] == 'C'){
                char rxBotNum[2];
                rxBotNum[0] = command[0];
                rxBotNum[1] = command[1];
                sprintf(errBuf,"B%d%2snak%3s",BOTNUM,rxBotNum,cmdStr);
            }
            else{
                sprintf(errBuf,"B%dABnak%3s",BOTNUM,cmdStr);
            }
            errModeReply = mode;
            return;
        }
        if(!strcmp(cmdStr,"ack")){  //Acknowledgement for XBee and BLE
            if(mode == 2){  //Acknowledge from XBee

            }
            else if(mode == 1){ //Acknowledge from BLE
                
            }
            return;
        }
        if(!strcmp(cmdStr,"mtr")){  //Motor Speed Control
            char lSpd[3] = {dataStr[0],dataStr[1],dataStr[2]};
            char rSpd[3] = {dataStr[3],dataStr[4],dataStr[5]};
            setLSpeed = atoi(lSpd);
            setRSpeed = atoi(rSpd);
            updateMotorControl = true;
            manualRC = true;
        }
        else if(!strcmp(cmdStr,"req")){  //Data Request
            requestActive = mode;
        }
        else if(!strcmp(cmdStr,"pts")){
            Serial.println(dataStr);
            if(!logFile.isOpen()){
                logFile.open(filenameMessages, O_RDWR | O_CREAT | O_AT_END);
                logFile.printlnf("[PUTS] Received String Command: %s",dataStr);
                logFile.close();
            }
            else logFile.printlnf("[PUTS] Received String Command: %s",dataStr);
        }
        else if(!strcmp(cmdStr,"hwa")){  //Incoming hello-world acknowledge
            waitForConnection = false;
        }
        else if(!strcmp(cmdStr,"aut")){  //Enter autonomous mode
            
        }
        else if(!strcmp(cmdStr,"dmp")){  //Enter SD Card "Dump Mode"
            offloadMode = true;
            status.setPattern(LED_PATTERN_BLINK);
            status.setColor(RGB_COLOR_BLUE);
            status.setSpeed(LED_SPEED_FAST);
        }
    }
}

void cmdLTEHandler(const char *event, const char *data){
    processCommand(data, 4,false);
    if(logMessages){
        if(!logFile.isOpen()) logFile.open(filenameMessages, O_RDWR | O_CREAT | O_AT_END);
        logFile.printlnf("[INFO] Received LTE Message: %s",data);
        logFile.close();
    }
}

void setup(){
    status.setPriority(LED_PRIORITY_IMPORTANT);
    status.setActive(true);

    uint32_t mtrArmTime = millis();
    ESCL.attach(ESC_PWM_L,1000,2000); //Initialize motor control
    ESCR.attach(ESC_PWM_R,1000,2000);
    ESCL.write(90);                   //Set ESC position to 90 for at least 2 seconds to "arm" the motors
    ESCR.write(90);

    BLE.on();
    
    //Log.info("Hello from WaterNet23!");
    Serial.begin();
    Serial1.begin(9600, SERIAL_PARITY_EVEN);                        //Start serial for XBee module
    setupSPI();                                 //Setup SPI for BeagleBone
    setupXBee();                                //Setup XBee module
    setupGPS();                                 //Setup GPS module
    setupLTE();                                 //Initialize LTE Flags

    manualRC = true;

    senseTimer = millis();
    dataTimer = millis();
    XBeeRxTime = 0;
    BLERxTime = 0;
    dataWait = false;
    logSensors = true;
    logMessages = true;
    offloadMode = false;
    requestActive = false;
    LTEStatusCount = LTE_MAX_STATUS;

    battPercent = 50;

    char deviceName[10];
    strcpy(deviceName,"WTN23_Bot");
    deviceName[9] = BOTNUM+48;
    BLECustomData[0] = BOTNUM;

    BLE.addCharacteristic(txCharacteristic);    //Add BLE Characteristics for BLE serial
    BLE.addCharacteristic(rxCharacteristic);
    BLE.addCharacteristic(offloadCharacteristic);

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
    statusPD.start();

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
        logFile.printlnf("[INFO] WaterBot %d: Started Logging!",BOTNUM);
        logFile.close();
    }
    if(STARTUP_WAIT_PAIR){
        waitForConnection = true;
        uint32_t publishMS = millis();
        char dataBuf[10];
        sprintf(dataBuf,"B%dCChwd",BOTNUM);
        while(waitForConnection){
            BLEScan(-2);
            XBeeHandler();
            if(millis() - publishMS >= XBEE_START_PUB){
                publishMS = millis();
                sendData(dataBuf,0,false,true,false);
            }
            delay(100);
        }
    }
}

void loop(){
    /*if(getGPSLatLon()){
        char latLonBuf[UART_TX_BUF_SIZE];
        latitude = ((float)latitude_mdeg/1000000.0);
        longitude = ((float)longitude_mdeg/1000000.0);
        //sprintf(latLonBuf, "GPS Data: Lat:%0.6f Lon:%0.6f\n", latitude, longitude);
        //Serial.println(latLonBuf);
        //sendData(latLonBuf, 0, true, true, false);
    }*/
    sensorHandler();
    XBeeHandler();
    statusUpdate();
    updateMotors();
    if(offloadMode) dataOffloader();
    if(errModeReply){
        sendData(errBuf,errModeReply,false,false,false);
        errModeReply = 0;
    }
    sendResponseData();
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
//Initialization for LTE events and flags
void setupLTE(){
    Particle.subscribe("CCHub", cmdLTEHandler); //Subscribe to LTE data from Central Control Hub
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

//Function to check if response data to a request needs to be sent out
void sendResponseData(){
    if(requestActive){
        char responseStr[50];
        memcpy(responseStr,0,50);
        sprintf(responseStr,"GL%0.6f,GO%0.6f,DO%0.4f,PH%0.4f,CA%0.4f,CB%0.4f,TP%0.4f",latitude,longitude,senseDO,sensePH,senseCond,senseMiniCond,senseTemp);
        sendData(responseStr,requestActive,false,false,false);
        requestActive = 0;
    }
}

void statusUpdate(){
    if(statusReady){
        char updateStr[28];
        sprintf(updateStr,"B%dABsup%d %d %.6f %.6f",BOTNUM,battPercent,statusFlags,latitude,longitude);
        Serial.println(updateStr);
        Serial.println(LTEStatusCount);
        if(!BLEAvail && !XBeeAvail && LTEStatusCount && (LTEStatusCount%LTE_STAT_PD == 0)){
            sendData(updateStr,0,false,false,true);
        }
        else{
            if(XBeeAvail || BLEAvail) LTEStatusCount = LTE_MAX_STATUS;
            sendData(updateStr,0,true,true,false);
        }
        if(LTEStatusCount) LTEStatusCount--;
        statusReady = false;
        delay(100);
        sendData("B1CCptsbigbot",0,true,false,false);
    }
}

void updateMotors(){
    if(updateMotorControl){
        ESCL.write(setLSpeed);
        ESCR.write(setRSpeed);
        updateMotorControl = false;        
    }
}

void sendData(const char *dataOut, uint8_t sendMode, bool sendBLE, bool sendXBee, bool sendLTE){
    char outStr[strlen(dataOut)+2];
    sprintf(outStr,"%s%02x",dataOut,strlen(dataOut));
    if(sendLTE || sendMode == 4){
        Particle.publish("Bot1dat", outStr, PRIVATE);
        sendLTE = false;
    }
    if((sendBLE || sendMode == 1) && BLE.connected()){
        uint8_t txBuf_tmp[strlen(outStr)];
        memcpy(txBuf_tmp,outStr,strlen(outStr));
        txCharacteristic.setValue(txBuf_tmp, strlen(outStr));
    }
    if(sendXBee || sendMode == 2){
        Serial1.println(outStr);
    }
}

void StatusHandler(){
    statusFlags = 0;
    statusFlags = LTEAvail;
    statusFlags |= XBeeAvail << 1;
    statusFlags |= BLEAvail << 2;
    statusFlags |= offloadMode << 3;
    statusFlags |= manualRC << 4;
    statusFlags |= lowBattery << 5;
    statusFlags |= logSensors << 6;
    statusReady = true;
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
        //Serial.printlnf("pH: %f", sensePH);
        Wire.requestFrom(MCOND, 20, 1);
        code = Wire.read();               		                                      //the first byte is the response code, we read this separately.
        char mcondSense[20];
        c = 0;
        while(Wire.available()){   // slave may send less than requested
            mcondSense[c++] = Wire.read();

        }
        float senseMCond = atof(mcondSense);
        //Serial.printlnf("MiniCond: %f",senseMCond);
        Wire.requestFrom(COND, 20, 1);
        code = Wire.read();               		                                      //the first byte is the response code, we read this separately.
        char condSense[20];
        c = 0;
        while(Wire.available()){   // slave may send less than requested
            condSense[c++] = Wire.read();

        }
        float senseCond = atof(condSense);
        //Serial.printlnf("Conductivity: %f",senseCond);
        Wire.requestFrom(TEMPADDR, 20, 1);
        code = Wire.read();               		                                      //the first byte is the response code, we read this separately.
        char addrSense[20];
        c = 0;
        while(Wire.available()){   // slave may send less than requested
            addrSense[c++] = Wire.read();

        }
        float senseTemp = atof(addrSense);
        //Serial.printlnf("Temperature: %f",senseTemp);
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
        processCommand(buffer,2,true);
        Serial.println("New XBee Command:");
        Serial.println(data); 
        if(buffer[0] == 'B' || buffer[0] == 'C') XBeeRxTime = millis();
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
    char btBuf[len+1];
    for (size_t ii = 0; ii < len; ii++) btBuf[ii] = data[ii];
    if(btBuf[len-1] != '\0') btBuf[len] = '\0';
    else btBuf[len-1] = '\0';
    Serial.println("New BT Command:");
    Serial.println(btBuf);
    processCommand(btBuf,1,true);
    if(btBuf[0] == 'A' || btBuf[0] == 'C') BLERxTime = millis();
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

    statusMode = 0;
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

void BLEScan(int BotNumber){
    size_t count = BLE.scan(scanResults, SCAN_RESULT_COUNT);
	if (count > 0) {
		for (uint8_t ii = 0; ii < count; ii++) {
			BleUuid foundServiceUuid;
			size_t svcCount = scanResults[ii].advertisingData.serviceUUID(&foundServiceUuid, 1);
            uint8_t BLECustomData[CUSTOM_DATA_LEN];
            scanResults->advertisingData.customData(BLECustomData,CUSTOM_DATA_LEN);
            if(BLECustomData[0] == BOTNUM) return;  //Don't connect to yourself...
            if (svcCount > 0 && foundServiceUuid == WaterNetService) {
                if(BotNumber == -2){
                    bool newBot = true;
                    PairBot *existingBot;
                    for(PairBot p: BLEPair){
                        if(BLECustomData[0] == p.botNum){
                            newBot = false;
                            existingBot = &p;
                        } 
                    }
                    if(newBot){
                        PairBot NewBot;
                        NewBot.rssi = scanResults->rssi;
                        NewBot.botNum = BLECustomData[0];
                        BLEPair.push_back(NewBot);
                    }
                    else{
                        existingBot->rssi = (scanResults->rssi + existingBot->rssi) >> 1;
                    }
                }
                if(BotNumber == -1 || BotNumber == BLECustomData[0]){   //Check if a particular bot number was specified
                    peer = BLE.connect(scanResults[ii].address);
				    if (peer.connected()) {
                        uint8_t bufName[BLE_MAX_ADV_DATA_LEN];
                        scanResults[ii].advertisingData.customData(bufName, BLE_MAX_ADV_DATA_LEN);
					    peer.getCharacteristicByUUID(peerTxCharacteristic, txUuid);
					    peer.getCharacteristicByUUID(peerRxCharacteristic, rxUuid);
						Serial.printlnf("Connected to Bot %d",bufName[0]);
                        bool newBot = true;
                    }
                    break;
                }
			}
		}
	}
}