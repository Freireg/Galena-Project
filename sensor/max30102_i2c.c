/**
 * Example: Read MAX30102 sensor using Milk-V Duo S I2C HAL
 * 
 * Wiring:
 *   I2C1_SDA -> MAX30102 SDA
 *   I2C1_SCL -> MAX30102 SCL
 *   3.3V     -> MAX30102 VIN
 *   GND      -> MAX30102 GND
 * 
 * Build:
 *   gcc max30102_read.c -lwiringx -o max30102_read
 */

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#define I2C_DEV         "/dev/i2c-1"
#define MAX30102_ADDR   0x57

// MAX30102 registers
#define REG_INT_STATUS1  0x00
#define REG_INT_ENABLE1  0x02
#define REG_FIFO_WR_PTR  0x04
#define REG_OVF_COUNTER  0x05
#define REG_FIFO_RD_PTR  0x06
#define REG_FIFO_DATA    0x07
#define REG_MODE_CONFIG  0x09
#define REG_SPO2_CONFIG  0x0A
#define REG_LED1_PA      0x0C
#define REG_LED2_PA      0x0D
#define REG_FIFO_CONFIG  0x08

// Mode bits
#define MODE_HR_ONLY     0x02
#define MODE_SPO2_EN     0x03

int fd_i2c = -1;

// ------------------ I2C helper functions ------------------

int i2c_write_byte(uint8_t reg, uint8_t data) {
    uint8_t buf[2] = { reg, data };
    if (write(fd_i2c, buf, 2) != 2) {
        perror("i2c_write_byte");
        return -1;
    }
    return 0;
}

int i2c_read_bytes(uint8_t reg, uint8_t *buf, uint8_t len) {
    if (write(fd_i2c, &reg, 1) != 1) {
        perror("i2c_read_bytes: reg write");
        return -1;
    }
    if (read(fd_i2c, buf, len) != len) {
        perror("i2c_read_bytes: data read");
        return -1;
    }
    return 0;
}

// ------------------ MAX30102 setup ------------------

void max30102_reset() {
    i2c_write_byte(REG_MODE_CONFIG, 0x40);  // Reset bit
    usleep(100000); // 100ms
}

void max30102_init() {
    // Clear interrupts
    i2c_write_byte(REG_INT_ENABLE1, 0x00);
    // FIFO: sample average = 4, FIFO rollover = enabled
    i2c_write_byte(REG_FIFO_CONFIG, 0x4F);
    // SPO2: 100Hz sample rate, 411us pulse width, 16-bit resolution
    i2c_write_byte(REG_SPO2_CONFIG, 0x27);
    // LED pulse amplitude (0–255)
    i2c_write_byte(REG_LED1_PA, 0x24); // RED
    i2c_write_byte(REG_LED2_PA, 0x24); // IR
    // Mode: SpO2 enabled (Red + IR)
    i2c_write_byte(REG_MODE_CONFIG, MODE_SPO2_EN);
}

// ------------------ Read FIFO ------------------

int max30102_read_fifo(uint32_t *red, uint32_t *ir) {
    uint8_t data[6];
    if (i2c_read_bytes(REG_FIFO_DATA, data, 6) < 0)
        return -1;

    *red = ((uint32_t)(data[0] & 0x03) << 16) | ((uint32_t)data[1] << 8) | data[2];
    *ir  = ((uint32_t)(data[3] & 0x03) << 16) | ((uint32_t)data[4] << 8) | data[5];
    return 0;
}

// ------------------ Main ------------------

int main(void) {
    printf("Initializing I2C...\n");

    fd_i2c = open(I2C_DEV, O_RDWR);
    if (fd_i2c < 0) {
        perror("Failed to open I2C device");
        return -1;
    }

    if (ioctl(fd_i2c, I2C_SLAVE, MAX30102_ADDR) < 0) {
        perror("Failed to set I2C address");
        close(fd_i2c);
        return -1;
    }

    printf("Initializing MAX30102...\n");
    max30102_reset();
    max30102_init();

    printf("Reading data...\n");
    while (1) {
        uint32_t red, ir;
        if (max30102_read_fifo(&red, &ir) == 0)
            printf("RED: %6lu | IR: %6lu\n", (unsigned long)red, (unsigned long)ir);
        else
            printf("Read error.\n");
        usleep(100000); // 100ms
    }

    close(fd_i2c);
    return 0;
}
