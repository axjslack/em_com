
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <time.h>

#include <linux/i2c-dev.h>


#include "libem_com.h"

#define NO_TIMEOUT (0)


static int i2c_select_slave(int fd, uint16_t address)
{
    if (ioctl(fd, I2C_SLAVE, address) < 0) {
        fprintf(stderr, "i2c: Failed to select slave address.\n");
        return -1;
    }

    return 0;
}



int i2c_write(char *i2cdev, uint16_t slave_address, const uint8_t *buffer, uint32_t count)
{
    int ret, fd;
    uint32_t nbBytesSent;

    //fd = fds[current_mikrobus_index];
    //if ((fds[mikrobus_index] = open(i2c_path, O_RDWR)) < 0)

    fd= open(i2cdev, O_RDWR);	

    if (fd < 0) {
        fprintf(stderr, "i2c: Cannot write to unitialized bus.\n");
        return -1;
    }

    if (buffer == NULL) {
        fprintf(stderr, "i2c: Cannot write using invalid buffer.\n");
        return -1;
    }

    if (count == 0)
        return 0;

    if ((ret = i2c_select_slave(fd, slave_address)) < 0)
        return ret;

    nbBytesSent = 0;
    while (nbBytesSent < count) {
        ret = write(fd, &buffer[nbBytesSent], count - nbBytesSent);
        if (ret < 0) {
            fprintf(stderr, "i2c: Failed to write.\n");
            return -1;
        }

        nbBytesSent += ret;
    }

    close(fd);

    return nbBytesSent;
}

int i2c_read(char *i2cdev, uint16_t slave_address, uint8_t *buffer, uint32_t count)
{
    int ret, fd;
    uint32_t nbBytesReceived = 0;
    uint32_t timeout = NO_TIMEOUT;

    fd=open(i2cdev, O_RDWR);	
    if (fd < 0) {
        fprintf(stderr, "i2c: Cannot read using unitialized bus.\n");
        return -1;
    }

    if (buffer == NULL) {
        fprintf(stderr, "i2c: Cannot read from invalid buffer.\n");
        return -1;
    }

    if (count == 0)
        return 0;

    if ((ret = i2c_select_slave(fd, slave_address)) < 0)
        return ret;

    while (nbBytesReceived < count) {
        if (timeout != NO_TIMEOUT) {
            fd_set set;
            struct timeval tmp_timeout;
            tmp_timeout.tv_sec = timeout / 1000;
            tmp_timeout.tv_usec = (timeout % 1000) * 1000;
            FD_ZERO(&set);
            FD_SET(fd, &set);

            ret = select(fd + 1, &set, NULL, NULL, &tmp_timeout);
            if (ret == -1) {
                fprintf(stderr, "i2c: Failed to wait for data.\n");
                return -1;
            } else if (ret == 0) {
                fprintf(stderr, "i2c: Read timeout.\n");
                return nbBytesReceived;
            }
        }

        ret = read(fd, &buffer[nbBytesReceived], count - nbBytesReceived);
        if (ret < 0) {
            fprintf(stderr, "i2c: Failed to read.\n");
            return -1;
        }

        nbBytesReceived += ret;
    }

    close(fd);	

    return nbBytesReceived;
}

int i2c_write_byte(char *i2cdev, uint16_t slave_address, uint8_t data)
{
    return i2c_write(i2cdev, slave_address, &data, 1);
}

int i2c_read_byte(char *i2cdev, uint16_t slave_address, uint8_t *data)
{
    return i2c_read(i2cdev, slave_address, data, 1);
}

int i2c_write_register(char *i2cdev, uint16_t address, uint8_t reg_address, uint8_t value)
{
    uint8_t buffer[2] = { reg_address, value };
    return i2c_write(i2cdev, address, buffer, sizeof(buffer));
}

int i2c_read_register(char *i2cdev, uint16_t address, uint8_t reg_address, uint8_t *data)
{
    if (i2c_write_byte(i2cdev, address, reg_address) < 0)
        return -1;

    return i2c_read_byte(i2cdev, address, data);
}


