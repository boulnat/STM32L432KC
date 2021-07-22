/*
 * canOPEN.h
 *
 *  Created on: Apr 17, 2021
 *      Author: boulnat
 */

#ifndef INC_CANOPEN_H_
#define INC_CANOPEN_H_

/**
 * @defgroup CO_CANopen CANopen stack
 * @{
 *
 * CANopenNode is free and open source CANopen Stack.
 *
 * CANopen is the internationally standardized (EN 50325-4) (CiA DS-301)
 * CAN-based higher-layer protocol for embedded control system. For more
 * information on CANopen see http://www.can-cia.org/
 *
 * CANopenNode homepage is https://github.com/CANopenNode/CANopenNode
 *
 * CANopenNode is distributed under the terms of the GNU General Public License
 * version 2 with the classpath exception.
 */


    #include "CO_driver.h"
    #include "CO_OD.h"
    #include "CO_SDO.h"
    #include "CO_Emergency.h"
    #include "CO_NMT_Heartbeat.h"
    #include "CO_SYNC.h"
    #include "CO_PDO.h"
    #include "CO_HBconsumer.h"
#if CO_NO_SDO_CLIENT == 1
    #include "CO_SDOmaster.h"
#endif
#if CO_NO_TRACE > 0
    #include "CO_trace.h"
#endif


/**
 * Default CANopen identifiers.
 *
 * Default CANopen identifiers for CANopen communication objects. Same as
 * 11-bit addresses of CAN messages. These are default identifiers and
 * can be changed in CANopen. Especially PDO identifiers are confgured
 * in PDO linking phase of the CANopen network configuration.
 */
typedef enum{
     CO_CAN_ID_NMT_SERVICE       = 0x000,   /**< 0x000, Network management */
     CO_CAN_ID_SYNC              = 0x080,   /**< 0x080, Synchronous message */
     CO_CAN_ID_EMERGENCY         = 0x080,   /**< 0x080, Emergency messages (+nodeID) */
     CO_CAN_ID_TIME_STAMP        = 0x100,   /**< 0x100, Time stamp message */
     CO_CAN_ID_TPDO_1            = 0x180,   /**< 0x180, Default TPDO1 (+nodeID) */
     CO_CAN_ID_RPDO_1            = 0x200,   /**< 0x200, Default RPDO1 (+nodeID) */
     CO_CAN_ID_TPDO_2            = 0x280,   /**< 0x280, Default TPDO2 (+nodeID) */
     CO_CAN_ID_RPDO_2            = 0x300,   /**< 0x300, Default RPDO2 (+nodeID) */
     CO_CAN_ID_TPDO_3            = 0x380,   /**< 0x380, Default TPDO3 (+nodeID) */
     CO_CAN_ID_RPDO_3            = 0x400,   /**< 0x400, Default RPDO3 (+nodeID) */
     CO_CAN_ID_TPDO_4            = 0x480,   /**< 0x480, Default TPDO4 (+nodeID) */
     CO_CAN_ID_RPDO_4            = 0x500,   /**< 0x500, Default RPDO5 (+nodeID) */
     CO_CAN_ID_TSDO              = 0x580,   /**< 0x580, SDO response from server (+nodeID) */
     CO_CAN_ID_RSDO              = 0x600,   /**< 0x600, SDO request from client (+nodeID) */
     CO_CAN_ID_HEARTBEAT         = 0x700    /**< 0x700, Heartbeat message */
}CO_Default_CAN_ID_t;


/**
 * CANopen stack object combines pointers to all CANopen objects.
 */
typedef struct{
    CO_CANmodule_t     *CANmodule[1];   /**< CAN module objects */
    CO_SDO_t           *SDO[CO_NO_SDO_SERVER]; /**< SDO object */
    CO_EM_t            *em;             /**< Emergency report object */
    CO_EMpr_t          *emPr;           /**< Emergency process object */
    CO_NMT_t           *NMT;            /**< NMT object */
    CO_SYNC_t          *SYNC;           /**< SYNC object */
    CO_RPDO_t          *RPDO[CO_NO_RPDO];/**< RPDO objects */
    CO_TPDO_t          *TPDO[CO_NO_TPDO];/**< TPDO objects */
    CO_HBconsumer_t    *HBcons;         /**<  Heartbeat consumer object*/
#if CO_NO_SDO_CLIENT == 1
    CO_SDOclient_t     *SDOclient;      /**< SDO client object */
#endif
#if CO_NO_TRACE > 0
    CO_trace_t         *trace[CO_NO_TRACE]; /**< Trace object for monitoring variables */
#endif
}CO_t;


/** CANopen object */
    extern CO_t *CO;


/**
 * Function CO_sendNMTcommand() is simple function, which sends CANopen message.
 * This part of code is an example of custom definition of simple CANopen
 * object. Follow the code in CANopen.c file. If macro CO_NO_NMT_MASTER is 1,
 * function CO_sendNMTcommand can be used to send NMT master message.
 *
 * @param CO CANopen object.
 * @param command NMT command.
 * @param nodeID Node ID.
 *
 * @return 0: Operation completed successfully.
 * @return other: same as CO_CANsend().
 */
#if CO_NO_NMT_MASTER == 1
    uint8_t CO_sendNMTcommand(CO_t *CO, uint8_t command, uint8_t nodeID);
#endif


/**
 * Initialize CANopen stack.
 *
 * Function must be called in the communication reset section.
 *
 * @param CANbaseAddress Address of the CAN module, passed to CO_CANmodule_init().
 * @param nodeId Node ID of the CANopen device (1 ... 127).
 * @param nodeId CAN bit rate.
 *
 * @return #CO_ReturnError_t: CO_ERROR_NO, CO_ERROR_ILLEGAL_ARGUMENT,
 * CO_ERROR_OUT_OF_MEMORY, CO_ERROR_ILLEGAL_BAUDRATE
 */
CO_ReturnError_t CO_init(
        int32_t                 CANbaseAddress,
        uint8_t                 NodeId,
        uint16_t                BitRate);


/**
 * Delete CANopen object and free memory. Must be called at program exit.
 *
 * @param CANbaseAddress Address of the CAN module, passed to CO_CANmodule_init().
 */
void CO_delete(int32_t CANbaseAddress);


/**
 * Process CANopen objects.
 *
 * Function must be called cyclically. It processes all "asynchronous" CANopen
 * objects.
 *
 * @param CO This object
 * @param timeDifference_ms Time difference from previous function call in [milliseconds].
 * @param timerNext_ms Return value - info to OS - maximum delay after function
 *        should be called next time in [milliseconds]. Value can be used for OS
 *        sleep time. Initial value must be set to something, 50ms typically.
 *        Output will be equal or lower to initial value. If there is new object
 *        to process, delay should be suspended and this function should be
 *        called immediately. Parameter is ignored if NULL.
 *
 * @return #CO_NMT_reset_cmd_t from CO_NMT_process().
 */
CO_NMT_reset_cmd_t CO_process(
        CO_t                   *CO,
        uint16_t                timeDifference_ms,
        uint16_t               *timerNext_ms);


/**
 * Process CANopen SYNC and RPDO objects.
 *
 * Function must be called cyclically from real time thread with constant
 * interval (1ms typically). It processes SYNC and receive PDO CANopen objects.
 *
 * @param CO This object.
 * @param timeDifference_us Time difference from previous function call in [microseconds].
 *
 * @return True, if CANopen SYNC message was just received or transmitted.
 */
bool_t CO_process_SYNC_RPDO(
        CO_t                   *CO,
        uint32_t                timeDifference_us);


/**
 * Process CANopen TPDO objects.
 *
 * Function must be called cyclically from real time thread with constant.
 * interval (1ms typically). It processes transmit PDO CANopen objects.
 *
 * @param CO This object.
 * @param syncWas True, if CANopen SYNC message was just received or transmitted.
 * @param timeDifference_us Time difference from previous function call in [microseconds].
 */
void CO_process_TPDO(
        CO_t                   *CO,
        bool_t                  syncWas,
        uint32_t                timeDifference_us);


#endif /* SRC_CANOPEN_H_ */
