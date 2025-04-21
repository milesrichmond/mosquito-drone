#ifndef ADAFRUIT_BNO055_H
#define ADAFRUIT_BNO055_H

#include <stdint.h>

// --- ADDRESSES ---

// I2C Configuration



// General
#define BNO055_REG_DEVID        (0x00)
#define BNO055_DEVICE_ADDR      (0x29)
#define BNO055_REG_DATA_FORMAT  ()
#define BNO055_REG_DATA_START   ()
#define BNO055_REG_POWER_CTL    (0x3E)

// ID registers
#define BNO055_ID_CHIP      (0x00)
#define BNO055_ID_ACC       (0x01)
#define BNO055_ID_MAG       (0x02)
#define BNO055_ID_GYR       (0x03)
#define BNO055_ID_SW_REV    (0x04)
#define BNO055_ID_BL        (0x06)
#define BNO055_ID_PAGE      (0x07)

// Data registers
#define BNO055_REG_ACC      (0x08)
#define BNO055_REG_MAG      (0x0E)
#define BNO055_REG_GYR      (0x14)
#define BNO055_REG_EUL      (0x1B)
#define BNO055_REG_QUA      (0x20)
#define BNO055_REG_LIA      (0x28)
#define BNO055_REG_GRV      (0x2E)
#define BNO055_REG_TEMP     (0x34)

// Configuration
#define BNO055_REG_CALIB    (0x35)
#define BNO055_REG_UNIT_SEL (0x3B)
#define BNO055_REG_OPR_MODE (0x3D)
#define BNO055_REG_PWR_MODE (0x3E)

// Status
#define BNO055_REG_SYS_STATUS   (0x39)
#define BNO055_REG_SYS_ERR      (0x3A)


void bno055_init(void);

void bno055_read(uint8_t addr, int n, uint8_t* data);

void bno055_read_acc(uint16_t* data);
void bno055_read_mag(uint16_t* data);
void bno055_read_gyr(uint16_t* data);
void bno055_read_eul(uint16_t* data);
void bno055_read_qua(uint16_t* data);
void bno055_read_lia(uint16_t* data);
void bno055_read_grv(uint16_t* data);
void bno055_read_temp(uint8_t* data);

#endif
