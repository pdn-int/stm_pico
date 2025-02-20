#ifndef IC3DEFINITIONS_H_
#define IC3DEFINITIONS_H_

// Labels
#define FIRMWARE_VERSION "381"

// Flags
#define LOG_ROTATION_ON 1
#define XBEE_PRO_SX 1

// Disabling threads for additional mem stack
#define DUMMY_THRD_ENABLED 1
#define POLAR_THRD_ENABLED 0

#define DUMMY_NO_COT_WRAPPER 1

// chee rxtx debug
#define BYTES_IN_FRAME_UNTIL_PAYLOAD 46
#define BYTES_IN_DUMMY_HEADER 29
#define BYTE_OFFSET (BYTES_IN_FRAME_UNTIL_PAYLOAD+BYTES_IN_DUMMY_HEADER)

#if defined(STM32_UMOTE2)
  #define ENABLE_LOW_POWER_SLEEP 0
  #define PICOEW 1
  #define WeightSensorEnabled 0
#elif defined(picoSTM32H753)
  #define ENABLE_LOW_POWER_SLEEP 0
  #define PICOEW 0
  #define WeightSensorEnabled 0
  
  // for PicoSTM rev B where difference between rev A and B, are LEDs
  #define PICOSTM_REV_VALUE   2
  #define PICOSTM_REV_B       (defined(picoSTM32H753) && (PICOSTM_REV_VALUE == 2))
#elif defined(PICOSTM_COMBO_REV_B) || defined(PICOSTM_COMBO)
  #define ENABLE_LOW_POWER_SLEEP 0 //1
  #define PICOEW 0
  #define WeightSensorEnabled 0 //1
#endif

// Allow binary data to pass to 8088
#define BINARY_ALLOWED            1

#if defined(picoSTM32H753) || defined(PICOSTM_COMBO) || defined(PICOSTM_COMBO_REV_B)
  // when push button and switch are shorted for default configs
  #define PB_SWITCH_SHORTED 1

  // sdram
  // picobadge requires SDRAM, since it needs a 400kB buffer
  #define SDRAM_ENABLED 1

  // fragmentation on emmc (0) or sdram (1)
  #if SDRAM_ENABLED
    #define FRAGMENTATION_USE_SDRAM 1
  #else
    #define FRAGMENTATION_USE_SDRAM 0
  #endif
  
  // Save recording or encoding data
  #define PICOBADGE_V3              0
  #define PICOBADGE_TX              1 && !PICOBADGE_V3 // PBv3 cannot transmit audio
  #define EXCLUSIVE_REC_BUTTON      (0 && PICOBADGE_TX) // 1 = extra rec button; 0 = rec button is up button
  #define AUDIO_AUK_COT             0
  
  // Allow USB storage when default configs are loaded
  #define DEFAULT_BOOT_USB_STORAGE  1
#else
  #define FRAGMENTATION_USE_SDRAM 0
#endif

// reduce parent req rt
#define REDUCE_PRNT_REQ_RT 1

// Resource Limits
#define MAX_SYSTEM_THREADS 64
#define LINKED_SERVER_NAME_BFR_SIZE 32
#define LOG_MAX_SIZE 200000
#define MAX_LINKED_SERVERS 8
#define TMPBFRSIZE 128
#define SYSTEM_MAX_MSG_BUFFER_SIZE 1024
#if XBEE_PRO_SX
	// note this can't be larger than radio.h:BIG_MAX_MESSAGE_LEN
	#define DUMMY_SENSOR_MAX_PACKET_SIZE FRAGMENT_MAX_SIZE
	#define SYSTEM_MAX_PACKET_SIZE 272 //256 + headers (for prosx)
	#define SYSTEM_MAX_PACKET_RX_SIZE 272 //256 + headers (for prosx)
	#define SYSTEM_MAX_FRAME_HEADER 20
#else
  // note this can't be larger than radio.h:BIG_MAX_MESSAGE_LEN
	#define DUMMY_SENSOR_MAX_PACKET_SIZE (1024)
	#define SYSTEM_MAX_PACKET_SIZE 1518 //1500 + headers
	#define SYSTEM_MAX_PACKET_RX_SIZE 1100 //1024 + headers
#endif
#define MAX_UID_NAME 16
#define MAX_SEQ_TIME_OFFSET_SECONDS 60
#define RADIO_ADDR_SIZE_IN_BYTES 8

#define NET_ADDRESS_SIZE 8

// Reources descriptors
#define SYSTYPE_FIELD 1
#define SYSTYPE_GATEWAY 0
#define XLR_UART_PORT "/dev/ttyUSB0"

#if !defined(STM32_UMOTE2) && !defined(picoSTM32H753) && !defined(PICOSTM_COMBO) && !defined(PICOSTM_COMBO_REV_B)
    #define CONFIG_PATH "/home/root/config.ini"
    #define LOG_PATH "/application/logs"
//    #define LOG_PERF_PATH "/home/root/performance.log"
//    #define STORAGE_FILEPATH "/application/"
//    #define MDIR_BASEPATH "/application"
    #define MDIR_PATH "/application"
#else
    #define CONFIG_PATH "M0:config.ini"
    #define LOG_PATH "M0:"
//    #define LOG_PERF_PATH "performance.log"
//    #define STORAGE_FILEPATH "M0:cotdata.txt"
//    #define MDIR_BASEPATH "M0:"
    #define MDIR_PATH "M0:\\mdir"
    #define ALIAS_DEFAULT_PATH "M0:alias_default"
#endif

#define LOG_FILENAME "ic3"
#define STORAGE_SIZE 4096

//Network ports
#define UDP_WO_COT_SERVER_UDP_PORT 8089
#define TCP_WO_COT_SERVER_PORT 8087
#define TCP_RW_COT_SERVER_PORT 8088

#if XBEE_PRO_SX
  #define TCP_BUFSIZE 8192 //4096
#else
  #define TCP_BUFSIZE 8192
#endif
#define TCP_SOCKET_BIN_SIZE 1440

// Operational
#define SUCCESS_CODE 0
#define ERROR_CODE 1

#define STATS_TXRX_FACTOR 8 //to convert kBps to kbps
// COT Protocol
#define REQUEST_COMMANDS_LABEL "_request"

// Callsign max length in characters
#define CALLSIGN_SIZE 13
// Message Types
#define MSG_TYPE_MESHNET 0
#define MSG_TYPE_GENERIC 1
#define MSG_TYPE_SYSTEMSTATUS 2
#define MSG_TYPE_POLAR 3
#define MSG_TYPE_NEDP 4
#define MSG_TYPE_DUMMY 5
#define MSG_TYPE_ATAK 6
#define MSG_TYPE_FILE 7
#define MSG_TYPE_RTR 8

// Mesh parameters
#define MESH_FIND_NEIGHBORS_INITIAL    10000  // Default interval to find a parent when we have none.
#define MESH_FIND_NEIGHBORS_INTERVAL  120000  // Default interval to find a NEW parent after one has been selected
#define DEFAULT_SAMPLING_PERIOD_IN_MILLIS (300*1000) //  5 minutes
#define MESH_EXPIRATION_THRESHOLD 5

// Storage default settings
#define RETRY_SKIP_PACKETS_THRESHOLD 8     // How many packets from TX queue are processed before trying to read from the retry queue
#define OVERFLOW_SKIP_PACKETS_THRESHOLD 0  // How many packets from TX queue are processed before trying to read from the overflow queue
#define STORAGE_MIN_RETRY_DELAY 100        // Minimum delay between sending packets from cot storage
#define STORAGE_RETRY_QUEUE_LIMIT (50 * 1000)       // Max number of packets to keep in the retry queue, if less 0 is unbounded.
#define STORAGE_OVERFLOW_QUEUE_LIMIT (100 * 1000)   // Max number of packets to keep in the overflow queue, if less than 0 is unbounded.
#define STORAGE_DEFAULT_SAVE_BATCH_SIZE 50          // How many every packets is the queue flushed to disk

// Low power bits
#define LOW_POWER_MCU_MASK    0b00000001
#define LOW_POWER_QSPI_MASK   0b00000010
#define LOW_POWER_RS485_MASK  0b00000100
#define LOW_POWER_ETH_MASK    0b00001000
#define LOW_POWER_SDRAM_MASK  0b00010000
#define LOW_POWER_EMMC_MASK   0b00100000

#endif
