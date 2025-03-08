#ifndef DAEMON_H_
#define DAEMON_H_

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "ic3definitions.h"
#include "sys/types.h"
// #include "comms/mesh/meshtypes.h"
// #include "storage/filequeue.h"
// #include "comms/xlr/xlrdriver.h"
/*-----------------------------------------------------------------------------
 *   Global Resources
 *-----------------------------------------------------------------------------*/

// different umote processes
#define UMPRC_XLR_ENABLED 1
#define UMPRC_XLR_SKIP_ACK 0
#define UMPRC_DUMMY_MS 75
#define UMPRC_XLR_IO_ENABLED 1
#define UMPRC_XLR_LINK_ENABLED 1
#define UMPRC_XLR_SESS_ENABLED 1
#define UMPRC_COTSRV_ENABLED 1
#define UMPRC_LINKEDSRV_ENABLED 1
#define UMPRC_FILELOG_ENABLED 1
#define UMPRC_POLAR_ENABLED 1

#define ATAK_ANDROID_ID_LEN 96
#define ATAK_ANDRIOD_TEAM_COLOR_MAX_LEN 32

/*
* Status of the system
*/
typedef struct ic3daemon {
    bool logsEnabled;    // Send log messages to the file
    bool perfLogEnabled; // Performance log activated
    int logsLevel;       // Logs level debug, error, info
    bool logsDataEnabled; // datalog file is enabled
    char uid_name[MAX_UID_NAME]; // Name of this device
    char callsign[CALLSIGN_SIZE]; // callsign or alias
    bool callsignLocked;
#if !defined(STM32_UMOTE2) && !defined(picoSTM32H753) && !defined(PICOSTM_COMBO) && !defined(PICOSTM_COMBO_REV_B)
    pid_t pid;         // Our process id
    pid_t sid;         // Session id
#endif
    int node_type; //Field or GW

    // Device Eth. IP
    char device_ip_ascii[16];

    //security
    bool encryptionEnabled;

    //storage
    char storageName[13];
    // COT_STORAGE_CONTROL cot_ctrl;
    bool storageEnabled;
    int retrySkipPacketsThreshold;
    int overflowSkipPacketsThreshold;
    int storageMinRetryDelay;
    int storageRetryQueueLimit;
    int storageRetryQueueSaveBatchSize;
    int storageOverflowQueueLimit;
    int storageOverflowQueueSaveBatchSize;

    // mesh
    int meshnet_algorithm;
    int fn_initial;
    int fn_interval;
    int avg_rssi_period_in_secs;
    int expiration_threshold;
    uint8_t rt_include_level;
    int parent_red_rssi_threshold;
    int parent_min_rssi_threshold;

    // Comms
    bool xlr_comms_enabled;
    bool radio_simulation;
    bool radioCompression;
    bool cloudCompression;

    // xlr
//     XLR_OP_PARAMS xlrOps;
    uint8_t gateway_addr[8];
    char gateway_addr_str[16];

    // second GW destination
    uint8_t gateway_addr2[8];
    char gateway_addr_str2[16];

    //linked server
    bool linkedserver_enabled;
    bool linkedserver_connected;
    int linkedserver_fd;
    char linkedserver_addr[32];
    int linkedserver_port;
    bool linkedserver_tcp;
    bool linkedserver_downstream;
    bool network_router;
    bool linkedserver_tls;
    char linkedserver_tls_port[8];
    char linkedserver_tls_ca_path[32];
    char linkedserver_tls_cert_path[32];
    char linkedserver_tls_key_path[32];
    char linkedserver_tls_key_pass[12];  
    bool linkedserver_filter_enabled;
    char linkedserver_filter_prefix[MAX_UID_NAME];
		
#if !defined(STM32_UMOTE2) && !defined(picoSTM32H753) && !defined(PICOSTM_COMBO) && !defined(PICOSTM_COMBO_REV_B)
    fd_set cot_clients;
#endif

    // internal sensors
    bool sensors_int_status;         //internal status sensors
    int sensors_int_status_rate;     //internal status update rate
    bool sensors_enable_broadcast;   // if true use broadcast for the status packet
    int sensors_int_simulation_rate; //sensor simulation rate in millisecs
    int sensors_int_simulation_size; //sensor simulation size of packet in bytes
    int sensors_int_simulation_limit; //sensor simulation limit of messages
    int sensors_int_simulation_enabled;
    int sensors_int_simulation_mode; // simulation mode: 0 TX, 1 RX
    bool sensors_int_gps_enabled;           // internal GPS
    char sensors_int_gps_device[16];        //gps device /dev/xxxx
    bool sensors_int_gps_override;          //is gps fixed?
    bool sensors_int_gps_override_atak;      //take gps from atak
    char sensors_int_gps_gLongitudeStr[16]; //fixed gps coords
    char sensors_int_gps_gLatitudeStr[16];  //fixed gps coords
    double sensors_int_gps_lat;
    double sensors_int_gps_lon;
    bool sensors_int_serial_enabled;
    char sensors_int_serial_device[16];        //device /dev/xxxx
    bool sensors_int_serial_nedp_enabled;
    bool sensors_int_serial_olympus_enabled;
    int sensors_int_serial_olympus_rate;
    int sensor_int_serial_olympus_filelimit;
    uint8_t sensors_int_serial_olympus_model; // 0 none, 1 epoch, 2 nortech
    int localserverEnabled;
    int localserverPort;
    int localserverPublic;
    int localserverConnectionCnt;
    bool localserver_tls_enabled;
    char localserver_tls_port[8];
    char localserver_tls_ca_path[32];
    char localserver_tls_cert_path[32];
    char localserver_tls_key_path[32];
    char localserver_tls_key_pass[12];
#if !defined(STM32_UMOTE2) && !defined(picoSTM32H753) && !defined(PICOSTM_COMBO) && !defined(PICOSTM_COMBO_REV_B)
    fd_set localserverClients;
    fd_set localserverReadfds;
#endif
    bool localserverUdpEnabled;
    int udpserverPort;
#if !defined(STM32_UMOTE2) && !defined(picoSTM32H753) && !defined(PICOSTM_COMBO) && !defined(PICOSTM_COMBO_REV_B)
    fd_set udpserverClients;
    fd_set udpserverReadfds;
#endif

    // ATAK unit
    bool atak_enabled;
    int atak_port;
    bool atak_broadcast_status;
    bool atak_broadcast_online;
    char atak_androidId[ATAK_ANDROID_ID_LEN];
    char atak_androidTeam[ATAK_ANDRIOD_TEAM_COLOR_MAX_LEN];

    // Analytics
    bool analytics_enabled;
    int analytics_algo;

    // support for file-based fragmentation and transport layer
    bool transport_layer;
    bool retire_fragment;

    // enable/disable additiona logging
    bool log_mesh_fbp_enabled;

    //enable packet re-broadcasting
    bool rebroadcast_enabled;

    // enable responding to ident requests
    bool enable_ident_response;

    // if true the status packets are set as volatile (default false)
    bool enable_volatile_status_packet;

    //binary router
    int rtr_local_port;
    uint8_t rtr_destination[8];

    // used in Linux to simulate delays in radio transmissions
    int sent_delay_millis;
    
    // commands
    bool command_set_send_configs;

    // cot wrapper
    bool cot_wrapper;

    // LEDs mode
    int led_mode;

    // PicoTag
    bool picotag_enabled;

    // I2C and SPI sensors
    bool fgauge_enabled;
      bool fgauge_int_enabled;
      uint8_t fgauge_soc_low;
      uint8_t fgauge_valrt_min;
      uint8_t fgauge_valrt_max;
      bool fgauge_soc_change_enabled;
    bool light_sensor_enabled;
      uint8_t light_sensor_int_mode;
      uint8_t light_sensor_int_ch;
      uint32_t light_sensor_var_thres;
      uint32_t light_sensor_upper_thres;
      uint32_t light_sensor_lower_thres;
    bool xlg_enabled;
      bool xlg_int_enabled;
      uint8_t xlg_wake_thres;
      uint8_t xlg_wake_dur;
      uint8_t xlg_int_interval; // 0 = only when activity/inactivity occurs

    // Weight Sensor (ADC)
    bool weight_sensor_enabled;
    
  #if defined(STM32_UMOTE2)
    bool press_enabled;
    bool temp_humid_enabled;
  #elif defined(picoSTM32H753) || defined(PICOSTM_COMBO) || defined(PICOSTM_COMBO_REV_B)
    bool pth_enabled;
    bool batt_charger_enabled;
      bool batt_charger_int_enabled;
      uint8_t batt_mode;
    
    // Low Power
    uint8_t low_power;

    // Pico Badge
    bool picobadge_enabled;
    uint8_t picobadge_volume;
    bool picobadge_tx;
    bool picobadge_tx_save;
    bool picobadge_rx_buttons;
    int picobadge_rx_max_saves;
    uint8_t picobadge_destination[8];
    int picobadge_skip;
    int decoder_gain;
    bool alarm_enable;
    bool k9_enabled;
    bool dsp_enabled;
    char dsp_type[5];
    int dsp_fs;
    int dsp_fc;
    float dsp_q;
    
    // fuel level sensor
    bool flevel_enabled;
    float flevel_resistor;
    float wdepth_resistor;
    
    // USB storage
    bool usb_storage;
  #endif
  
#if ENABLE_LOW_POWER_SLEEP
    int fnWakeTime;
    int statusWakeTime;
#endif

#if PICOSTM_REV_B
    bool out_5v_en;
#endif

} IC3Daemon;

extern IC3Daemon ic3;

// util functions
void threadStackUtilization(const char *label);

#endif
