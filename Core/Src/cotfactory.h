#ifndef _COTFACTORY_H
#define _COTFACTORY_H

#include <time.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "ic3definitions.h"

#if !defined(STM32_UMOTE2) && !defined(picoSTM32H753) && !defined(PICOSTM_COMBO) && !defined(PICOSTM_COMBO_REV_B)

#include <unistd.h>
#include <sys/types.h>
// #include "../sensors/linuxserial.h"
// #include "../sensors/nedp.h"

#else
//#include "uart.h"
#endif

// #include "../comms/mesh/meshnet.h"
// #include "../comms/mesh/routingtable.h"
#include "logger.h"
// #include "audioInterface.h"

#define COT_DEFAULT_TYPE "a-f-G-U-C-I"

#define COT_DATA_TIMESTAMP_SIZE 32
#define COT_DATA_POINT_SIZE 128

typedef struct cot_gps {
    bool locked;
    double latitude;
    double longitude;
    double hae;
    double ce;
    double le;
    uint16_t numSatellites;
} COT_GPS;

typedef struct cot_light {
    uint32_t ir;
    uint32_t green;
    uint32_t blue;
    uint32_t red;
} COT_LIGHT;

typedef struct cot_xlgyr {
    float xlx;
    float xly;
    float xlz;
    float gx;
    float gy;
    float gz;
} COT_XLGYR;

typedef struct cot_pth {
    float pressure;
    float temperature;
    float humidity;
} COT_PTH;

typedef struct cot_fgauge {
    float vcell;
    float soc;
    float crate;
} COT_FGAUGE;

typedef struct cot_flevel {
    float voltage;
    float current;
    float liquid;
    float air;
} COT_FLEVEL;

typedef struct cot_wdepth { // water depth (als-mpm-2f)
    float voltage;
    float current;
    float depth;
} COT_WDEPTH;

typedef struct cot_data {
    time_t startTime;
    time_t staleTime;
    char timeString[COT_DATA_TIMESTAMP_SIZE];
    char uid[24];
    COT_GPS gps;
    char stype[24];
    char xlr_ori[18];
    char xlr_dest[18];
    char xlr_gw[18];
    char xlr_net[8];
    char fw_version[8];
    char adc[32];
    uint32_t xlr_txretry;
    uint32_t xlr_tx;
    uint32_t xlr_rx;
    int8_t xlr_rssi_last;
    double xlr_tx_kbps;
    double xlr_tx_pps;
    double xlr_rx_pps;
    double xlr_rx_kbps;
//    uint32_t xlr_qlen;
    uint32_t xlr_txqlen;
    uint32_t xlr_rxqlen;
    uint64_t seq;
    int8_t mesh_level;
    int8_t power_level;
    // nedp nedp_data;
#if defined(STM32_UMOTE2) || defined(picoSTM32H753) || defined(PICOSTM_COMBO) || defined(PICOSTM_COMBO_REV_B)
    uint8_t medical;
    // hrm hrm_data;
    //Light
    COT_LIGHT light;
    //Accel/Gyro
    COT_XLGYR xlgyr;
    //PTH
    COT_PTH pth;
    //Fuel Gauge
    COT_FGAUGE fgauge;
    //Fuel Level Sensor
    COT_FLEVEL flevel;
    //Water-depth sensor (ALS-MPM-2F Water level Transmitter)
    COT_WDEPTH wdepth;
    //Power stats
    float bat_voltage;
    float charger_voltage;
    //Weight Sensor
    float adjusted_weight;
#endif
} COT_DATA;


void cot_create_dummy_message(char *simXmlBfr, int size, int sequence, char *uid);

void cot_add_header(char *buffer, COT_DATA *data, bool cot_wrapper);

void cot_add_point(char *buffer, bool cot_wrapper);

void cot_add_flowtag(char *buffer, COT_DATA *data);

void cot_add_remarks(char *buffer, COT_DATA *data, bool includeTags, bool cot_wrapper);

void cot_build(char *buffer, COT_DATA *data, bool cot_wrapper);

void cot_add_dummy_payload(char *buffer, int size);

void cot_build_dummy(char *buffer, COT_DATA *data, int size, char *test_prefix,
                        int sequence, int limit);

void cot_add_remarks_short(char *buffer, COT_DATA *data);

void cot_build_routingtable(char *buffer, COT_DATA *data);

void cot_add_remarks_nedp(char *buffer, COT_DATA *data);

void cot_build_atak_pong(char *buffer, int len);

void cot_build_command_response(char *buffer, int len, const char* uid, time_t timestamp, char *response);

void cot_timestamp_fmt(char* timeString, time_t startTime, const char * timeFormatString);

void cot_timestamp(char* timeString, time_t startTime);

void cot_timestamp_remarks(char* timeString, time_t startTime);

void cot_timestamp_now(char* timeString);

void cot_build_command_atakonline(char *buffer, unsigned int len, const char* uid, time_t timestamp, char *commandList);

unsigned int cot_build_atak_status_message(char *buffer, unsigned int len, const char* callsign, const char* uid, const char* TeamColor);

#if defined(STM32_UMOTE2) || defined(picoSTM32H753) || defined(PICOSTM_COMBO) || defined(PICOSTM_COMBO_REV_B)
void cot_add_heart_rate(char *buffer, COT_DATA *data, bool cot_wrapper);
#endif

#define RMK_NUM_OF_PARAMS       21
#define RMK_NUM_OF_PARAMS_PTAG  8
#define IP_ADDR_NUM_OF_PARAMS   1
#define PTH_NUM_OF_PARAMS       3
#define XLG_NUM_OF_PARAMS       6
#define FGG_NUM_OF_PARAMS       3
#define LIT_NUM_OF_PARAMS       4
#define FLV_NUM_OF_PARAMS       2
#define ATK_NUM_OF_PARAMS       2
#define WGT_NUM_OF_PARAMS       1

#define RMK_START_INDEX         0
#define IP_ADDR_START_INDEX     (RMK_START_INDEX + RMK_NUM_OF_PARAMS) // 21
#define PTH_START_INDEX         (IP_ADDR_START_INDEX + IP_ADDR_NUM_OF_PARAMS) // 22
#define XLG_START_INDEX         (PTH_START_INDEX + PTH_NUM_OF_PARAMS) // 25
#define FGG_START_INDEX         (XLG_START_INDEX + XLG_NUM_OF_PARAMS) // 31
#define LIT_START_INDEX         (FGG_START_INDEX + FGG_NUM_OF_PARAMS) // 34
#define FLV_START_INDEX         (LIT_START_INDEX + LIT_NUM_OF_PARAMS) // 38 
#define ATK_START_INDEX         (FLV_START_INDEX + FLV_NUM_OF_PARAMS) // 40
#define WGT_START_INDEX         (ATK_START_INDEX + ATK_NUM_OF_PARAMS) // 42

#define TOTAL_PARAMS_REMARKS    (RMK_NUM_OF_PARAMS + IP_ADDR_NUM_OF_PARAMS + PTH_NUM_OF_PARAMS + XLG_NUM_OF_PARAMS + FGG_NUM_OF_PARAMS + LIT_NUM_OF_PARAMS + FLV_NUM_OF_PARAMS + ATK_NUM_OF_PARAMS + WGT_NUM_OF_PARAMS)
#define MAX_PARAM_REMARKS_LEN   32
#define TOTAL_PARAMS_EVENT      2 // uid, timeString
#define MAX_PARAM_EVENT_LEN     32
#define TOTAL_PARAMS_POINT      3 // lat, lon, sat
#define MAX_PARAM_POINT_LEN     16
#define MAX_PARAM_ALERT_LEN     128

typedef struct {
    char uid[MAX_UID_NAME];
    char timeString[COT_DATA_TIMESTAMP_SIZE];
    char latitude[16];
    char longitude[16];
    char satellite[8];
} COT_REQUIRED_Typedef;

void cot_add_remarks_reform(char *buffer, 
        COT_REQUIRED_Typedef *data, 
        char remarks[TOTAL_PARAMS_REMARKS][MAX_PARAM_REMARKS_LEN], 
        uint8_t sensor, bool ptag);
void cot_add_header_reform(char *buffer, COT_REQUIRED_Typedef *data);
void cot_add_point_reform(char *buffer, COT_REQUIRED_Typedef *data);

// void cot_reform_audio_auk(char *buffer, PBMessage_TypeDef *msg);
// void cot_reform_alert(char *buffer, PBMessage_TypeDef *msg);

void cot_build_interrupt(char *buffer, COT_DATA *data, uint32_t interrupt, bool cot_wrapper) ;

#endif
