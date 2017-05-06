/*
 *                        f8wConfig.h
 *
 *  Compiler command-line options used to define an F8W Z-Stack
 *  devices. To move an option from here to the project
 *  file, comment out or delete the option from this file and
 *  enter it into the "Define Symbols" box under the Preprocessor
 *  tab of the C/C++ Compiler Project Options. New user defined
 *  options may be added to this file, as necessary.
 *
 */

// Replacement for confiuration itmems with the CONST label.
#define CONFIG_ITEM

#if !defined ( ATTR_PACKED )
  #define ATTR_PACKED __attribute__((packed))
#endif

/* Enable ZigBee-Pro */
#if !defined(ZIGBEEPRO)
  #define ZIGBEEPRO    
#endif

/* Set to 0 for no security, otherwise non-0 */
#if !defined(SECURE)
  #define SECURE 1
#endif

#if !defined(DEFAULT_CHANLIST)
  /* Default channel is Channel 11 - 0x0B               */
  // Channels are defined in the following:
  //         0      : 868 MHz     0x00000001
  //         1 - 10 : 915 MHz     0x000007FE
  //        11 - 26 : 2.4 GHz     0x07FFF800
  //
  //#define MAX_CHANNELS_868MHZ     0x00000001
  //#define MAX_CHANNELS_915MHZ     0x000007FE
  //#define MAX_CHANNELS_24GHZ      0x07FFF800
  //#define DEFAULT_CHANLIST 0x04000000  // 26 - 0x1A
  //#define DEFAULT_CHANLIST 0x02000000  // 25 - 0x19
  //#define DEFAULT_CHANLIST 0x01000000  // 24 - 0x18
  //#define DEFAULT_CHANLIST 0x00800000  // 23 - 0x17
  #define DEFAULT_CHANLIST 0x00400000  // 22 - 0x16
  //#define DEFAULT_CHANLIST 0x00200000  // 21 - 0x15
  //#define DEFAULT_CHANLIST 0x00100000  // 20 - 0x14  // Demo Channel
  //#define DEFAULT_CHANLIST 0x00080000  // 19 - 0x13
  //#define DEFAULT_CHANLIST 0x00040000  // 18 - 0x12
  //#define DEFAULT_CHANLIST 0x00020000  // 17 - 0x11
  //#define DEFAULT_CHANLIST 0x00010000  // 16 - 0x10
  //#define DEFAULT_CHANLIST 0x00008000  // 15 - 0x0F
  //#define DEFAULT_CHANLIST 0x00004000  // 14 - 0x0E
  //#define DEFAULT_CHANLIST 0x00002000  // 13 - 0x0D
  //#define DEFAULT_CHANLIST 0x00001000  // 12 - 0x0C
  //#define DEFAULT_CHANLIST 0x00000800  // 11 - 0x0B
#endif

/* Define the default PAN ID.
 *
 * Setting this to a value other than 0xFFFF causes
 * ZDO_COORD to use this value as its PAN ID and
 * Routers and end devices to join PAN with this ID
 */
#if !defined(ZDAPP_CONFIG_PAN_ID)
//  #define ZDAPP_CONFIG_PAN_ID 0xFFFF
  #define ZDAPP_CONFIG_PAN_ID 0x0605
#endif

/* Minimum number of milliseconds to hold off the start of the device
 * in the network and the minimum delay between joining cycles.
 */
#if !defined(NWK_START_DELAY)
	#define NWK_START_DELAY 100
#endif

/* Mask for the random joining delay. This value is masked with
 * the return from osal_rand() to get a random delay time for
 * each joining cycle.  This random value is added to NWK_START_DELAY.
 * For example, a value of 0x007F will be a joining delay of 0 to 127
 * milliseconds.
 */
#if !defined(EXTENDED_JOINING_RANDOM_MASK)
	#define EXTENDED_JOINING_RANDOM_MASK 0x007F
#endif

/* Minimum number of milliseconds to delay between each beacon request
 * in a joining cycle.
 */
#if !defined(BEACON_REQUEST_DELAY)
	#define BEACON_REQUEST_DELAY 100
#endif

/* Mask for the random beacon request delay. This value is masked with the
 * return from osal_rand() to get a random delay time for each joining cycle.
 * This random value is added to BEACON_REQUEST_DELAY. For example, a value
 * of 0x00FF will be a beacon request delay of 0 to 255 milliseconds.
 */
#if !defined(BEACON_REQ_DELAY_MASK)
	#define BEACON_REQ_DELAY_MASK 0x00FF
#endif

/* Jitter mask for the link status report timer. This value is masked with the
 * return from osal_rand() to add a random delay to _NIB.nwkLinkStatusPeriod.
 * For example, a value of 0x007F allows a jitter between 0-127 milliseconds.
 */
#if !defined(LINK_STATUS_JITTER_MASK)
	#define LINK_STATUS_JITTER_MASK 0x007F
#endif

/* in seconds; set to 0 to turn off route expiry */
#if !defined(ROUTE_EXPIRY_TIME)
	#define ROUTE_EXPIRY_TIME 30
#endif

/* This number is used by polled devices, since the spec'd formula
 * doesn't work for sleeping end devices.  For non-polled devices,
 * a formula is used. Value is in 2 milliseconds periods
 */
#if !defined(APSC_ACK_WAIT_DURATION_POLLED)
	#define APSC_ACK_WAIT_DURATION_POLLED 3000
#endif

/*  Default indirect message holding timeout value:
 *  1-65535 (0 -> 65536) X CNT_RTG_TIMER X RTG_TIMER_INTERVAL
 */
#if !defined(NWK_INDIRECT_MSG_TIMEOUT)
	#define NWK_INDIRECT_MSG_TIMEOUT 7
#endif

/* The number of simultaneous route discoveries in network */
#if !defined(MAX_RREQ_ENTRIES)
	#define MAX_RREQ_ENTRIES 8
#endif

/* The maximum number of retries allowed after a transmission failure */
#if !defined(APSC_MAX_FRAME_RETRIES)
	#define APSC_MAX_FRAME_RETRIES 3
#endif

/* Max number of times retry looking for the next hop address of a message */
#if !defined(NWK_MAX_DATA_RETRIES)
#define NWK_MAX_DATA_RETRIES 2
#endif

/* Number of times retry to poll parent before indicating loss of synchronization
 * with parent. Note that larger value will cause longer delay for the child to
 * rejoin the network.
 */
#if !defined(MAX_POLL_FAILURE_RETRIES)
	#define MAX_POLL_FAILURE_RETRIES 2
#endif

/* The number of items in the broadcast table */
#if !defined(MAX_BCAST)
	#define MAX_BCAST 9
#endif

/* The maximum number of groups in the groups table */
#if !defined(APS_MAX_GROUPS)
	#define APS_MAX_GROUPS 16
#endif

/* no. of entries in the regular routing table plus additional
 * entries for route repair
 */
#if !defined(MAX_RTG_ENTRIES)
	#define MAX_RTG_ENTRIES 40
#endif

/* Maximum number of entries in the Binding table. */
#if !defined(NWK_MAX_BINDING_ENTRIES)
	#define NWK_MAX_BINDING_ENTRIES 4
#endif

/* Maximum number of cluster IDs for each binding table entry.
 * Note that any value other than the default value may cause a
 * compilation warning but Device Binding will function correctly.
 */
#if !defined(MAX_BINDING_CLUSTER_IDS)
	#define MAX_BINDING_CLUSTER_IDS 4
#endif

/* Default security key. */
#if !defined(DEFAULT_KEY)
	#define DEFAULT_KEY {0x01, 0x03, 0x05, 0x07, 0x09, 0x0B, 0x0D, 0x0F, 0x00, 0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0D}
#endif

/* Set the MAC MAX Frame Size (802.15.4 default is 102) */
#if !defined(MAC_MAX_FRAME_SIZE)
	#define MAC_MAX_FRAME_SIZE 116
#endif

/* Minimum transmissions attempted for Channel Interference detection,
 * Frequency Agility can be disabled by setting this parameter to zero.
 */
#if !defined(ZDNWKMGR_MIN_TRANSMISSIONS)
	#define ZDNWKMGR_MIN_TRANSMISSIONS 20
#endif

/****************************************
 * The following are for End Devices only
 ***************************************/

#if !defined(RFD_RCVC_ALWAYS_ON)
	#define RFD_RCVC_ALWAYS_ON FALSE
#endif

/* The number of milliseconds to wait between data request polls to the coordinator. */
#if !defined(POLL_RATE)
	#define POLL_RATE 1000
#endif

/* This is used after receiving a data indication to poll immediately
 * for queued messages...in milliseconds.
 */
#if !defined(QUEUED_POLL_RATE)
	#define QUEUED_POLL_RATE 100
#endif

/* This is used after receiving a data confirmation to poll immediately
 * for response messages...in milliseconds
 */
#if !defined(RESPONSE_POLL_RATE)
	#define RESPONSE_POLL_RATE 100
#endif

/* This is used as an alternate response poll rate only for rejoin request.
 * This rate is determined by the response time of the parent that the device
 * is trying to join.
 */
#if !defined(REJOIN_POLL_RATE)
	#define REJOIN_POLL_RATE 440
#endif
