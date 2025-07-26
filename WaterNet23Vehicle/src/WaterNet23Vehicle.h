//////////////////////////////
// BOT CONFIGURATION MACROS //
//////////////////////////////

#define BOTNUM              1           //VERY IMPORTANT - Change for each bot to configure which node in the network this bot is
#define STARTUP_WAIT_PAIR   0           //Set to 1 to wait for controller to connect and discover bots, turns on "advertising" on startup
//#define BLE_DEBUG_ENABLED               //If enabled, will add a BLE characteristic for convenient printing of log messages to a BLE console
#define LEAK_DET_BYPASS     0           //Set to 1 to disable shutdown upon leak detection
#define BATT_TRIG_LEAK      0           //Set to 1 to enable the battery leak cutting off system
//#define VERBOSE

//////////////////////////
// EEPROM Configuration //
//////////////////////////

#define EEPROM_KEY1_LOC         0x00                    //Location of the first key in EEPROM, used to check if the EEPROM is valid
#define EEPROM_KEY2_LOC         0x01                    //Location of the second key in EEPROM, used to check if the EEPROM is valid
#define EEPROM_KEY1             (uint8_t)0x23           //Key for the first byte of the EEPROM, used to check if the EEPROM is valid
#define EEPROM_KEY2             (uint8_t)0x129          //Key for the second byte of the EEPROM, used to check if the EEPROM is valid
#define EEPROM_COMP_CAL_LOC     0x02                    //Location of the compass calibration in EEPROM, used to store the calibration values for the compass


///////////////////////
// Pin Configuration //
///////////////////////

#define ESC_PWM_L           D6          //Left motor ESC output pin
#define ESC_PWM_R           D7          //Right motor ESC output pin
#define SENSE_EN            D2          //Output pin to enable/disable voltage regulator for sensors
#define chipSelect          D8          //Chip select pin for Micro SD Card
#define BATT_ISENSE         A3          //Shunt monitor ADC input for battery supply current
#define SOL_ISENSE          A2          //Shunt monitor ADC input for solar array input current
#define BAT_LEAK_DET        A4          //Digital input for reading battery leak sensor
#define PWR_BUT             A1          //Digital input for reading power button input
#ifdef A6
#define BATT_VSENSE         A6          //Voltage divider ADC input for reading power rail (battery) voltage
#endif
#ifdef D22
#define PWR_EN              D22         //Digital output for latching power mosfet on until shutoff
#endif
#ifdef D23
#define LEAK_DET            D23         //Digital input for on-PCB leak detection trace
#endif

/////////////////////////
// Compass Calibration //
/////////////////////////

#define COMP_OFFSET 0                       //Number of degrees to add to the raw compass reading to calibrate it to true north. This is a constant offset, not a full calibration
#define COMP_CAL_AVG_COUNT      5         //Number of samples to average for the compass calibration
#define COMPASS_CAL_TIMEOUT     3000      //Time in milliseconds to flash the LED while calibrating the compass

////////////////////
// PROGRAM MACROS //
////////////////////

#define SCAN_RESULT_COUNT   20

#define PHADDR              99              //default I2C ID number for EZO pH Circuit.
#define MCOND               100             //default I2C ID number for EZO Mini-Conductivity (0.1)
#define COND                101             //default I2C ID number for EZO Conductivity Circuit. (1.0)
#define TEMPADDR            102             //Default I2C address for temperature sensor
#define DOADDR              97              //Default I2C address for Dissolved Oxygen sensor
#define SENS_CONNECTED      false           //Flag to indicate that the sensors are connected, used to determine if the sensors are powered on or not
#define SENS_POLL_RT        2500            //Number of milliseconds between sensor reads
#define SENS_DATA_DLY       825             //Number of milliseconds between a request to a sensor and actually retrieving the reading

#define BUTTON_DEB_TIME     25              //Debounce time for button presses
#define BUTTON_IDLE_TIME    2000            //Time in milliseconds to wait before considering the button idle
#define WATCHDOG_PD         1000           //Watchdog timer period in milliseconds
#define STATUS_PD           10000            //Time between status updates published to CC Hub
#define STOP_RST_TIME       10000           //Time after receiving the last stop command to exit stop mode
#define XBEE_WDOG_AVAIL     5000           //Watchdog interval between XBee messages for availablility check
#define BLE_WDOG_AVAIL      5000           //Watchdog interval between BLE messages for availability check
#define LTE_MAX_STATUS      480             // (Divided by LTE STAT PD) Maximum number of status messages to send over LTE if other methods are unavailable
#define LTE_STAT_PD         4               //Divider for sending status via LTE to reduce data usage
#define XBEE_START_PUB      5000            //Time period between sending "Hello World" messages over XBee during setup
#define MANUAL_RAMP_PD      30             //Time period between motor ramp updates when in manual motor drive mode

#define DEF_FILENAME        "WaterBot"
#define FILE_LABELS         "Time,Latitude,Longitude,Temperature,pH,Dissolved O2,Conductivity 0.1K,Conductivity 1K"
#define BLE_OFFLD_BUF       100
#define CUSTOM_DATA_LEN     8
#define MAX_FILENAME_LEN    32

/////////////////////////
// Power System Macros //
/////////////////////////

#define BAT_MIN             13.2            //Voltage to read 0% battery
#define BAT_MAX             16.4            //Voltage to read 100% battery
#define LOW_BATT_PCT        20              //Voltage to set low battery flag
#define VDIV_MULT           0.004835        //Calculate the ratio for ADC to voltage conversion 3.3V in on ADC = 4095 3.3V on 100kOhm + 20kOhm divider yields (3.3/20000)*120000 = 19.8V in MAX
#define BAT_ISENSE_MULT     33.0            //Calculate the maximum current the shunt can measure for the battery. Rs = 0.001, RL = 100k. Vo = Is * 0.1, max current is 33A
#define SLR_ISENSE_MULT     16.5            //Calculate the maximum current the shunt can measure for the solar array. Rs = 0.010, RL = 20k. Vo = Is * 0.2, max current is 16.5A

////////////////////////
// Motor Drive Macros //
////////////////////////

#define MTR_IDLE_ARM        2000            //Number of milliseconds to hold motors stopped for arming
#define MTR_ST_FWD          100             //Minimum commanded speed for motors going forward
#define MTR_ST_REV          80              //Minimum commanded speed for motors in reverse
#define MTR_TIMEOUT         4000            //Timeout in milliseconds for turning off motors when being manually controlled
#define MTR_RAMP_SPD        3               //Rate to ramp motor speed to target speed (step size for going between a value somewhere between 0 and 180)
#define MTR_RAMP_TIME       50              //Time between ramp iterations
#define MTR_TRAVEL_SPD      0.45             //Percentage maximum travel speed for autonomous movement default
#define MTR_CUTOFF_RAD      1.5             //Radius to consider "arrived" at a target point
#define SENTRY_IDLE_RAD     4.0             //Radius to keep motors off in sentry mode after reaching the cutoff radius
#define GPS_POLL_TIME       990             //Rate to poll the GPS and calculate the distance
#define COMP_POLL_TIME      250             //Rate to poll the Compass and calculate the target heading

////////////////////////
// Drive Mode Macros ///
////////////////////////

#define DRIVE_MODE_MANUAL        0               // Manual drive mode
#define DRIVE_MODE_SENTRY        1               // Sentry mode
#define DRIVE_MODE_AUTONOMOUS    2               // Autonomous drive mode
