/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#line 1 "c:/Users/mligh/OneDrive/Particle/WaterNet23/WaterNet23PreAlpha/src/WaterNet23PreAlpha.ino"
/*
 * Project WaterNet23PreAlpha
 * Description: Initial code for B404 with GPS and serial communications
 * Date: 3/18/2022
 */

#include "application.h"                    //Needed for I2C to GPS
#include "SparkFun_u-blox_GNSS_Arduino_Library.h"
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
void cmdLTEHandler(const char *event, const char *data);
void setup();
void loop();
uint8_t readPowerSys();
float deg2rad(float deg);
float readCompassHeading(float x_accel, float y_accel);
float calcDistance(float lat1, float lat2, float lon1, float lon2);
void printBLE(const char *dataOut);
void StatusHandler();
void testConnection(bool checkBLE, bool checkXBee, bool checkLTE);
static void BLEDataReceived(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context);
void motionHandler();
void wdogHandler();
void LEDHandler();
#line 11 "c:/Users/mligh/OneDrive/Particle/WaterNet23/WaterNet23PreAlpha/src/WaterNet23PreAlpha.ino"
#define X_AXIS_ACCELERATION 0
//#include <MicroNMEA.h>                      //http://librarymanager/All#MicroNMEA
#include "SdFat.h"
#include "sdios.h"
#undef min
#undef max
#include <vector>

//////////////////////////////
// BOT CONFIGURATION MACROS //
//////////////////////////////

#define BOTNUM              1           //VERY IMPORTANT - Change for each bot to configure which node in the network this bot is
#define STARTUP_WAIT_PAIR   0           //Set to 1 to wait for controller to connect and discover bots, turns on "advertising" on startup
#define BLE_DEBUG_ENABLED               //If enabled, will add a BLE characteristic for convenient printing of log messages to a BLE console

//Pin Configuration

#define ESC_PWM_L           D6          //Left motor ESC output pin
#define ESC_PWM_R           D5          //Right motor ESC output pin
#define SENSE_EN            D2          //Output pin to enable/disable voltage regulator for sensors
#define chipSelect          D8          //Chip select pin for Micro SD Card
#define BATT_ISENSE         A3          //Shunt monitor ADC input for battery supply current
#define SOL_ISENSE          A2          //Shunt monitor ADC input for solar array input current
#define BATT_VSENSE         A6          //Voltage divider ADC input for reading power rail (battery) voltage
#define PWR_EN              D22         
#define LEAK_DET            D23

/////////////////////////
// Compass Calibration //
/////////////////////////

#define N_BEARING   6.0
#define NW_BEARING  -18.0
#define W_BEARING   -40.0
#define SW_BEARING  -78.0
#define S_BEARING   125.0
#define SE_BEARING  86.0
#define E_BEARING   58.0
#define NE_BEARING  31.0

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

#define BAT_MIN             13.2            //Voltage to read 0% battery
#define BAT_MAX             16.4            //Voltage to read 100% battery
#define LOW_BATT_PCT        20            //Voltage to set low battery flag
#define VDIV_MULT           0.004835        //Calculate the ratio for ADC to voltage conversion 3.3V in on ADC = 4095 3.3V on 100kOhm + 20kOhm divider yields (3.3/20000)*120000 = 19.8V in MAX
#define BAT_ISENSE_MULT     33.0            //Calculate the maximum current the shunt can measure for the battery. Rs = 0.001, RL = 100k. Vo = Is * 0.1, max current is 33A
#define SLR_ISENSE_MULT     16.5            //Calculate the maximum current the shunt can measure for the solar array. Rs = 0.010, RL = 20k. Vo = Is * 0.2, max current is 16.5A

#define MTR_TIMEOUT         4000

#define REPL_NAK            false

SYSTEM_MODE(MANUAL);

//GPS Buffers and Objects
char nmeaBuffer[100];
//MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));
SFE_UBLOX_GNSS myGPS;

Adafruit_LIS3MDL lis3mdl;

//SD File system object
SdFat sd((SPIClass*)&SPI);

File myFile;
File logFile;
File logDir;

SerialLogHandler logHandler(LOG_LEVEL_INFO);                         //Log Configuration

Servo ESCL; 
Servo ESCR;

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

#ifdef BLE_DEBUG_ENABLED
    const char* bledbgUuid      = "b4206915-dc4b-5743-c8b1-92d0e75182b0"; //BLE Debug Console characteristic
    BleCharacteristic bledbgCharacteristic("dbg", BleCharacteristicProperty::NOTIFY, bledbgUuid, WaterNetService);
#endif

BleAdvertisingData advData;                 //Advertising data

uint8_t BLECustomData[CUSTOM_DATA_LEN];

//Function prototypes
void setupSPI();
void setupLTE();
void setupXBee();
void setupGPS();
bool getPositionData();
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
Timer motionTimer(2500, motionHandler);
Timer statusPD(STATUS_PD,StatusHandler);

//LED Control
LEDStatus status;

//////////////////////
// Global Variables //
//////////////////////

bool waitForConnection;
float latitude, longitude;
float compassHeading, travelHeading, targetDelta;
float targetLat, targetLon;
float travelDistance;
uint8_t leftMotorSpeed, setLSpeed;
uint8_t rightMotorSpeed, setRSpeed;
uint8_t battPercent;
float battVoltage, battCurrent, solarCurrent;
bool updateMotorControl;
uint8_t driveMode = 0;
bool lowBattery;
bool statusReady;
uint8_t requestActive;
uint16_t LTEStatusCount;
uint8_t statusFlags;
bool LTEAvail, XBeeAvail, BLEAvail;     //Flags for communicaton keep-alives/available
bool logSensors, logMessages, dataWait; //Flags for sensor timing/enables
bool offloadMode;
bool signalLED;
uint32_t senseTimer, dataTimer;
uint32_t XBeeRxTime, BLERxTime;
uint32_t lastMtrTime;
uint32_t lastStatusTime;
float sensePH, senseTemp, senseCond, senseMiniCond, senseDO;
char txBuf[UART_TX_BUF_SIZE];
char errBuf[MAX_ERR_BUF_SIZE];
uint8_t errModeReply;
size_t txLen = 0;
char filename[MAX_FILENAME_LEN];
char filenameMessages[MAX_FILENAME_LEN];

int i = 0;

//Mode 1 - BLE, Mode 2 - XBEE, Mode 4 - LTE
void processCommand(const char *command, uint8_t mode, bool sendAck){
    //Process if command is addressed to this bot "Bx" or all bots "AB"
    if((command[2] == 'A' && command[3] == 'B') || (command[2] == 'B' && command[3] == BOTNUM+48)){
        uint8_t checksum;
        char dataStr[strlen(command)-8];
        dataStr[strlen(command)-9] = '\0';
        char cmdStr[4];
        cmdStr[3] = '\0';
        char checkStr[3] = {command[strlen(command)-2], command[strlen(command)-1], '\0'};
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
            if(REPL_NAK){
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
            }
            return;
        }
        if(!strcmp(cmdStr,"ack")){  //Acknowledgement for XBee and BLE
            if(mode == 2){  //Acknowledge from XBee

            }
            else if(mode == 1){ //Acknowledge from BLE
                
            }
            return;
        }
        if(!strcmp(cmdStr,"ctl")){
            char tLat[8];
            char tLon[8];
            sscanf(dataStr,"%s %s %d %d %d",tLat,tLon,&driveMode,&logSensors,&signalLED);    //Target lat, target lon, drive mode, dataRecord, signal 
        }
        if(!strcmp(cmdStr,"mtr")){  //Motor Speed Control
            char lSpd[3] = {dataStr[0],dataStr[1],dataStr[2]};
            char rSpd[3] = {dataStr[3],dataStr[4],dataStr[5]};
            setLSpeed = atoi(lSpd);
            setRSpeed = atoi(rSpd);
            Serial.printlnf("Received Motor Command: LSpeed=%d,RSpeed=%d",setLSpeed,setRSpeed);
            if(setLSpeed > 90 && setLSpeed <=123) setLSpeed = 123;
            if(setRSpeed > 90 && setRSpeed <=123) setRSpeed = 123;
            if(setLSpeed < 90 && setLSpeed >=123) setLSpeed = 67;
            if(setRSpeed < 90 && setRSpeed >=123) setRSpeed = 67;
            ESCL.write(setLSpeed);
            ESCR.write(setRSpeed);
            updateMotorControl = true;
            lastMtrTime = millis();
            driveMode = 0;
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
        else if(!strcmp(cmdStr,"spc")){  //Incoming communication status
            lastStatusTime = millis();
        }
        else if(!strcmp(cmdStr,"hwa")){
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

    
    pinMode(SENSE_EN, OUTPUT);
    pinMode(PWR_EN, OUTPUT);
    pinMode(LEAK_DET, INPUT);
    digitalWrite(SENSE_EN,LOW);
    digitalWrite(PWR_EN,LOW);

    uint32_t mtrArmTime = millis();
    setLSpeed = 90;
    setRSpeed = 90;
    ESCL.attach(ESC_PWM_L,1000,2000);
    ESCR.attach(ESC_PWM_R,1000,2000);
    ESCL.write(setLSpeed);
    ESCR.write(setRSpeed);
    if(!STARTUP_WAIT_PAIR) delay(2000);         //Delay for 2 seconds to allow motor controllers to arm

    BLE.on();
    BLE.setTxPower(8);          //Max transmitting power
    
    //Log.info("Hello from WaterNet23!");
    Serial.begin();
    Serial1.begin(9600);                        //Start serial for XBee module
    setupSPI();                                 //Setup SPI for BeagleBone
    setupXBee();                                //Setup XBee module
    setupGPS();                                 //Setup GPS module
    setupLTE();                                 //Initialize LTE Flags

    senseTimer = millis();
    dataTimer = millis();
    lastStatusTime = millis();
    XBeeRxTime = 0;
    BLERxTime = 0;
    dataWait = false;
    logSensors = true;
    logMessages = true;
    offloadMode = false;
    requestActive = false;
    LTEStatusCount = LTE_MAX_STATUS;

    battPercent = 50;

    BLE.addCharacteristic(txCharacteristic);    //Add BLE Characteristics for BLE serial
    BLE.addCharacteristic(rxCharacteristic);
    BLE.addCharacteristic(offloadCharacteristic);
    #ifdef BLE_DEBUG_ENABLED
        BLE.addCharacteristic(bledbgCharacteristic);
    #endif

    BLECustomData[0] = BOTNUM;

    advData.appendServiceUUID(WaterNetService); // Add the app service
    advData.appendCustomData(BLECustomData,CUSTOM_DATA_LEN);

    BLE.advertise(&advData);                    //Start advertising the characteristics

    Wire.begin();
    Wire.setClock(CLOCK_SPEED_400KHZ);

    if (! lis3mdl.begin_I2C()) {          // hardware I2C mode, can pass in address & alt Wire
        Serial.println("Failed to find LIS3MDL chip");
    }
    else Serial.println("LIS3MDL Found!");
    lis3mdl.setPerformanceMode(LIS3MDL_HIGHMODE);
    lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
    lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
    lis3mdl.setRange(LIS3MDL_RANGE_8_GAUSS);
    lis3mdl.setIntThreshold(500);
    lis3mdl.configInterrupt(false, false, true, // enable z axis
                          true, // polarity
                          false, // don't latch
                          true); // enabled!

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
    motionTimer.start();
    ledTimer.start();
    statusPD.start();

    if (!sd.begin(chipSelect, SD_SCK_MHZ(8))) {
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
            XBeeHandler();
            if(millis() - publishMS >= XBEE_START_PUB){
                publishMS = millis();
                sendData(dataBuf,0,false,true,false);
            }
            delay(100);
        }
        Serial.println("Successfully paired with controller");
    }
    
}

void loop(){
    

    //Serial.println();
    if(getPositionData()){
        char latLonBuf[UART_TX_BUF_SIZE];
        //sprintf(latLonBuf, "GPS Data: Lat:%0.6f Lon:%0.6f\n", latitude, longitude);
        //Serial.println(latLonBuf);
        //sendData(latLonBuf, 0, true, true, false);
    }

    readPowerSys();
    //Serial.printlnf("Battery %: %d Voltage: %0.3fV, Battery Current: %0.4fA, Solar Current: %0.4fA",battPercent, battVoltage, battCurrent, solarCurrent);
    sensorHandler();
    XBeeHandler();
    statusUpdate();
    updateMotors();
    if(offloadMode) dataOffloader();
    if(errModeReply && REPL_NAK){
        sendData(errBuf,errModeReply,false,false,false);
        errModeReply = 0;
    }
    //sendData("B1CCptsbigbot",0,false,true,false);
    sendResponseData();
    delay(500);
}


//This function gets called from the SparkFun Ublox Arduino Library
//As each NMEA character comes in you can specify what to do with it
//Useful for passing to other libraries like tinyGPS, MicroNMEA, or even
//a buffer, radio, etc.
//void SFE_UBLOX_GPS::processNMEA(char incoming){
  //Take the incoming char from the Ublox I2C port and pass it on to the MicroNMEA lib
  //for sentence cracking
//  nmea.process(incoming);
//}
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
    //Serial1.printf("Hello from Bot %d\n", BOTNUM);   //Send Hello World message!
}

//I2C setup for NEO-M8U GPS
void setupGPS(){
    if(myGPS.begin() == false){
        delay(1000);
        Serial.println("Error, Could not initialize GPS");
    }
    myGPS.setI2COutput(COM_TYPE_UBX);
    myGPS.setPortInput(COM_PORT_I2C, COM_TYPE_UBX);
    Wire.setClock(400000); //Increase I2C clock speed to 400kHz
}

uint8_t readPowerSys(){
    battVoltage = (float) analogRead(BATT_VSENSE) * VDIV_MULT;
    int rawPCT = (int)(100 * (battVoltage - BAT_MIN)/(BAT_MAX - BAT_MIN));
    if(rawPCT < 0) rawPCT = 0;
    if(rawPCT > 100) rawPCT = 100;
    battPercent = (uint8_t) rawPCT;
    if(battPercent <= LOW_BATT_PCT) lowBattery = true;
    else lowBattery = false;
    battCurrent = (float) analogRead(BATT_ISENSE) * BAT_ISENSE_MULT / 4095;
    solarCurrent = (float) analogRead(SOL_ISENSE) * SLR_ISENSE_MULT / 4095;
    return battPercent;
}

float deg2rad(float deg) {
  return deg * (3.14159/180);
}

float readCompassHeading(float x_accel, float y_accel){
    float rawHeading = atan2(y_accel, x_accel) * 180.0 / M_PI;
    //Serial.printlnf("Raw Heading: %f", rawHeading);
    if(rawHeading >= N_BEARING && rawHeading < NE_BEARING){
        //Serial.println("Between N and NE");
        float diff = NE_BEARING - N_BEARING;
        return (45.0 * (rawHeading-N_BEARING)/diff);
    }
    else if(rawHeading >= NE_BEARING && rawHeading < E_BEARING){
        //Serial.println("Between E and NE");
        float diff = E_BEARING - NE_BEARING;
        return (45.0 * (rawHeading-NE_BEARING)/diff) + 45.0;
    }
    else if(rawHeading >= E_BEARING && rawHeading < SE_BEARING){
        //Serial.println("Between E and SE");
        float diff = SE_BEARING - E_BEARING;
        return (45.0 * (rawHeading-E_BEARING)/diff) + 90.0;
    }
    else if(rawHeading >= SE_BEARING && rawHeading < S_BEARING){
        //Serial.println("Between S and SE");
        float diff = S_BEARING - SE_BEARING;
        return (45.0 * (rawHeading-SE_BEARING)/diff) + 135.0;
    }
    else if(rawHeading >= NW_BEARING && rawHeading < N_BEARING){
        //Serial.println("Between N and NW");
        float diff = NW_BEARING - N_BEARING;
        return (-45.0 * (rawHeading-N_BEARING)/diff);
    }
    else if(rawHeading >= W_BEARING && rawHeading < NW_BEARING){
        //Serial.println("Between W and NW");
        float diff = W_BEARING - NW_BEARING;
        return (-45.0 * (rawHeading-NW_BEARING)/diff) - 45.0;
    }
    else if(rawHeading >=SW_BEARING && rawHeading < W_BEARING){
        //Serial.println("Between W and SW");
        float diff = SW_BEARING - W_BEARING;
        return (-45.0 * (rawHeading-W_BEARING)/diff) - 90.0;
    }
    
    else{   //Somewhere between south and southwest
        float maindiff = (180 + SW_BEARING) + (180 - S_BEARING);
        if(rawHeading > 0){
            //Serial.println("Between S and SW");
            float diff = 180.0 - S_BEARING;
            return -180.0 + (45.0 * (rawHeading - S_BEARING)/maindiff) ;
        }
        else{
            //Serial.println("Between S and SW, NR");
            float diff = 180.0 + SW_BEARING;
            return (45.0 * (diff/maindiff) * (rawHeading - SW_BEARING)/diff) - 135.0;
        }
    }
    return rawHeading;
}

float calcDistance(float lat1, float lat2, float lon1, float lon2){
    float dLat = deg2rad(lat1-lat2);
    float dLon = deg2rad(lon1-lon2);
    float a = sinf(dLat/2) * sinf(dLat/2) + cosf(deg2rad(lat2)) * cosf(deg2rad(lat1)) * sinf(dLon/2) * sinf(dLon/2); 
    float c = 2 * atan2(sqrt(a), sqrt(1.0-a)); 
    return 6371.0 * c; // Distance in km
}

bool getPositionData(){
    //myGPS.checkUblox(); //See if new data is available. Process bytes as they come in.

  //if(nmea.isValid() == true){
    //if(myGPS.isConnected()){
        targetLat = 35.769889;
        targetLon = -78.673824;
        latitude = 35.771801;
        longitude = -78.674378;
        //latitude = ((float)myGPS.getLatitude())/1000000.0;
        //longitude = ((float)myGPS.getLongitude())/1000000.0;
        lis3mdl.read();      // get X Y and Z data at once
        sensors_event_t event; 
        lis3mdl.getEvent(&event);
        compassHeading = readCompassHeading(event.magnetic.x,event.magnetic.y);
        if(targetLat >= -90 && targetLat <= 90 && targetLon >= -90 && targetLon <= 90){
            travelHeading = (atan2(targetLon-longitude, targetLat-latitude) * 180 / M_PI);
            travelDistance = calcDistance(targetLat,latitude,targetLon,longitude);
            if(travelHeading > 0){
                if(compassHeading > 0){
                    targetDelta = travelHeading - compassHeading;
                }
                else{
                    float diff = -(180.0 - travelHeading);
                    if(diff < compassHeading) targetDelta = travelHeading - compassHeading;
                    else targetDelta = 0 - (180.0 + compassHeading) - (180.0 - travelHeading);
                }
            }
            else{
                if(compassHeading > 0){
                    float diff = 180.0 + travelHeading;
                    if(diff > compassHeading) targetDelta = travelHeading - compassHeading;
                    else targetDelta = (180.0 - compassHeading) + (180.0 + travelHeading);
                }
                else{
                    targetDelta = travelHeading - compassHeading;
                }
            }
            char tempbuf[100];
            sprintf(tempbuf,"Raw : %f, Compass : %f, Travel hd: %f, T Delta: %f", atan2(event.magnetic.y, event.magnetic.x) * 180.0 / M_PI, compassHeading, travelHeading, targetDelta);
            printBLE(tempbuf);
            Serial.println(tempbuf);
        }
        
        return true;
    //}
    return false;
  //}
    
}

//Function to check if response data to a request needs to be sent out
void sendResponseData(){
    if(requestActive){
        char responseStr[65];
        memset(responseStr,0,65);
        //sprintf(responseStr,"B%dCCsns %0.6f %0.6f %0.4f %0.4f %0.4f %0.4f %0.4f",BOTNUM,latitude,longitude,senseDO,sensePH,senseCond,senseMiniCond,senseTemp);
        sprintf(responseStr,"B%dCCsns%0.6f %0.6f %d %d %d %d %d ",BOTNUM,latitude,longitude,(int)(senseDO*1000),(int)(sensePH*1000),(int)(senseCond*1000),(int)(senseMiniCond*1000),69000);
        sendData(responseStr,requestActive,false,false,false);
        requestActive = 0;
    }
}

void statusUpdate(){
    if(statusReady){
        char updateStr[28];
        sprintf(updateStr,"B%dABsup%d %d %.6f %.6f ",BOTNUM,battPercent,statusFlags,latitude,longitude);
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
        if(setLSpeed > 90 && setLSpeed <=123) setLSpeed = 123;
        if(setRSpeed > 90 && setRSpeed <=123) setRSpeed = 123;
        if(setLSpeed < 90 && setLSpeed >=123) setLSpeed = 67;
        if(setRSpeed < 90 && setRSpeed >=123) setRSpeed = 67;
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

void printBLE(const char *dataOut){
    #ifdef BLE_DEBUG_ENABLED
        uint8_t txBuf_tmp[strlen(dataOut)];
        memcpy(txBuf_tmp,dataOut,strlen(dataOut));
        bledbgCharacteristic.setValue(txBuf_tmp, strlen(dataOut));
    #endif
}

void StatusHandler(){
    statusFlags = 0;
    statusFlags = LTEAvail;
    statusFlags |= XBeeAvail << 1;
    statusFlags |= BLEAvail << 2;
    statusFlags |= offloadMode << 3;
    statusFlags |= driveMode << 4;
    statusFlags |= lowBattery << 6;
    statusFlags |= logSensors << 7;
    statusReady = true;
    Serial.println("Sending a status update!");
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
            myFile.printlnf("%s,%f,%f,%f,%f,%f,%f,%f",timestamp,latitude,longitude,senseTemp,sensePH,senseDO,senseMiniCond,senseCond);
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
        for(uint16_t i = 0 ; i < data.length(); i++) buffer[i] = data.charAt(i);
        if(data.length() > 1 && data.charAt(data.length()-1) == '\r') buffer[data.length()-1] = 0;
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

void motionHandler(){
    if(driveMode == 0 && millis() - lastMtrTime > MTR_TIMEOUT){
        setLSpeed = 90;
        setRSpeed = 90;
        updateMotorControl = true;
        ESCL.write(setLSpeed);
        ESCR.write(setRSpeed);
        Serial.printlnf("Warning, motor command has not been received in over %dms, cutting motors", MTR_TIMEOUT);
    }
}

void wdogHandler(){
    if(Particle.connected()) LTEAvail = true;
    else if(LTEAvail){
        if(!logFile.isOpen()) logFile.open(filenameMessages, O_RDWR | O_CREAT | O_AT_END);
        logFile.printlnf("[WARN] XBee Messages have not been received in %ds, assuming XBee is unavailable",(XBEE_WDOG_AVAIL/1000));
        LTEAvail = false;
    }
    if(millis()-XBeeRxTime > XBEE_WDOG_AVAIL || !XBeeRxTime){
        if(XBeeAvail){
            if(!logFile.isOpen()) logFile.open(filenameMessages, O_RDWR | O_CREAT | O_AT_END);
            logFile.printlnf("[WARN] XBee Messages have not been received in %ds, assuming XBee is unavailable",(XBEE_WDOG_AVAIL/1000));
        }
        XBeeAvail = false;
    }
    else XBeeAvail = true;
    if(millis()-BLERxTime > BLE_WDOG_AVAIL || !BLERxTime){
        if(BLEAvail && BLERxTime){
            if(!logFile.isOpen()) logFile.open(filenameMessages, O_RDWR | O_CREAT | O_AT_END);
            logFile.printlnf("[WARN] BLE Messages have not been received in %ds, assuming BLE is unavailable",(BLE_WDOG_AVAIL/1000));
        }
        BLEAvail = false;
    }
    else BLEAvail = true;
}

void dataOffloader(){
    Serial.println("Entering Data Offloader Mode");
    if (!logDir.open("/")) {
        offloadMode = false;
        Serial.println("Error, could not open root SD card directory");
        return;
    }
    Serial.println("Waiting for CCHub connection...");
    while(!BLE.connected()){
        BLE.advertise(&advData);;
        delay(10);
    }
    Serial.println("Starting transfer...");
    char fileCode[8 + MAX_FILENAME_LEN];
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
            noInterrupts();
            while(myFile.available()){
                char lineBuffer[BLE_OFFLD_BUF];
                memset(lineBuffer,0,BLE_OFFLD_BUF);
                //myFile.readBytes(lineBuffer,BLE_OFFLD_BUF);
                myFile.readBytesUntil('\r',lineBuffer,BLE_OFFLD_BUF);
                offloadCharacteristic.setValue(lineBuffer);
                //Serial.println(lineBuffer);
            }
            interrupts();
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
    if(signalLED){
        status.setPattern(LED_PATTERN_BLINK);
        status.setColor(RGB_COLOR_ORANGE);
        status.setSpeed(LED_SPEED_FAST);
        return;
    }
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
    else if(driveMode == 0){
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