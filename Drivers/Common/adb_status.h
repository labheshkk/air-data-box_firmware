/* adb_status.h - common status/error codes for all Air Data Box modules.
 * Convention: every function that can fail returns adb_status_t. ADB_OK is the
 * ONLY value that means "operation really succeeded". */
#ifndef ADB_STATUS_H
#define ADB_STATUS_H

typedef enum {
    ADB_OK = 0,
    ADB_ERR_PARAM,      /* NULL pointer / illegal argument (programming error)   */
    ADB_ERR_BUS,        /* transport (SPI/I2C) reported failure                  */
    ADB_ERR_TIMEOUT,    /* transport or device did not respond in time           */
    ADB_ERR_ID,         /* chip-ID / identity check failed                       */
    ADB_ERR_CRC,        /* calibration-data CRC mismatch                         */
    ADB_ERR_CALIB,      /* calibration data implausible (all 0x0000 / 0xFFFF)    */
    ADB_ERR_NOT_READY,  /* no valid measurement available yet                    */
    ADB_ERR_INVALID,    /* raw reading is a known invalid pattern                */
    ADB_ERR_RANGE,      /* converted value outside the sensor's specified range  */
    ADB_ERR_CONFIG,     /* device rejected / flagged our configuration           */
    ADB_ERR_STATE       /* device in unexpected state                            */
} adb_status_t;

#endif
