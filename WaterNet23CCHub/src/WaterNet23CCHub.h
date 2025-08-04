//////////////////////////////////////////////////////////
//                        MACROS                        //
//////////////////////////////////////////////////////////

// Pin Definitions

#define OLED_RESET              A0              //Pin used to reset the OLED screen on startup
#define JOYH_ADC                A2              //Horizontal Joystick
#define JOYV_ADC                A3              //Vertical Joystick ADC pin
#define JOY_BTN                 D23             //Joystick button press
#define L_DPAD                  A4              //Left DPAD button
#define R_DPAD                  A1              //Right DPAD button
#define U_DPAD                  A5              //Up DPAD button
#define D_DPAD                  D7              //Down DPAD button
#define E_DPAD                  D22             //"Enter" DPAD button
#define chipSelect              D8              //SD Card chip select pin
#define STOP_BTN                A7              //Stop button to disable all bot motors

//Program Parameters

#define DEF_FILENAME            "CCHUB"
#define BLE_OFFLD_BUF           100             //Maximum number of characters to fetch for each read from the SD card when offloading
#define SCAN_RESULT_COUNT       20              //Maximum devices to discover per scan
#define CUSTOM_DATA_LEN         8               //Number of bytes to include in the advertising packet for BLE
#define MAX_FILENAME_LEN        30              //Maximum number of characters in the filename for log files
#define MAX_ERR_BUF_SIZE        15              //Buffer size for error-return string
#define MAX_LTE_STATUSES        300             //Maximum number of statuses to send over LTE when XBee and BLE are unavailable

//Timing Control

#define XBEE_BLE_MAX_TIMEOUT    36
#define BLE_MAX_CONN_TIME       200             //20 second max time to successfully pair to bot
#define BLE_SCAN_PERIOD         1000            //Time betweeen BLE scans
#define LTE_BKP_Time            100             //Send LTE request after 10 seconds if not connected to any bot
#define LTE_CTL_PERIOD          29000           //Minimum time between sending LTE control packets periodically per bot
#define MTR_LTE_PERIOD          3000            //Minimum time between sending mtr command over LTE
#define MTR_UPDATE_TIME         750             //Frequency to send manual motor control packet in milliseconds
#define CONTROL_PUB_TIME        5000            //Number of milliseconds between sending control packets to bots
#define STOP_PUB_TIME           5000            //Time between sending stop messages when active
#define WB_MOD_UPDATE_TIME      60000           //Timeout for when status update packets will modify the class, prevents immediate overwrite when changing control variables

//Menu Parameters
#define i2c_Address             0x3c            //initialize with the I2C addr 0x3C Typically eBay OLED's
#define MAX_MENU_ITEMS          9               //Maximum number of settings per bot displayed on the menu (must go to where MenuItems.push_back() and count this)
#define DEBOUNCE_MS             150             //Number of milliseconds to disable button effects after pressing a button
#define OLED_MAX_X              128             //Number of pixels in the X direction
#define OLED_MAX_Y              64              //Number of pixels in the Y direction

//Control Parameters
#define JOY_DEADZONE        35                  //ADC reading offset in the middle to read no change in the the position
#define JOY_MID             2048                //ADC reading when the joystick is centered
#define JOY_MAX             4094                //ADC reading when the joystick is fully forward
#define JOY_MIN             1                   //ADC reading when the joystick is fully backward
#define LTE_MIN_DIFF        3                   //Minimum difference in motor speed to send an update over LTE
