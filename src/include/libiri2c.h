#ifndef _LIBI2C_H_
#define _LIBI2C_H_
#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <string.h>
#include <stdint.h>
#define VERSION_NAME "libiri2c "
#define VERSION_NUMBER "0.1.3"

#if defined(_WIN32)
#define HAVE_STRUCT_TIMESPEC
#include <pthread.h>
#define IRI2C_DEBUG(format, ...) iri2c_debug_print("libiri2c debug [%s:%d/%s] " format "\n", \
														 __FILE__,__LINE__, __FUNCTION__, __VA_ARGS__)
#define IRI2C_ERROR(format, ...) iri2c_error_print("libiri2c error [%s:%d/%s] " format "\n", \
														__FILE__,__LINE__, __FUNCTION__, __VA_ARGS__)
#elif defined(linux) || defined(unix)
#include <libgen.h>
#define IRI2C_DEBUG(format, ...) iri2c_debug_print("libiri2c debug [%s:%d/%s] " format "\n", \
														basename(__FILE__), __LINE__, __FUNCTION__, ##__VA_ARGS__)
#define IRI2C_ERROR(format, ...) iri2c_error_print("libiri2c error [%s:%d/%s] " format "\n", \
														basename(__FILE__), __LINE__, __FUNCTION__, ##__VA_ARGS__)
#endif

extern void (*iri2c_debug_print)(const char* fmt, ...);
extern void (*iri2c_error_print)(const char* fmt, ...);

#define VI2C_BUSY_STS_BIT				0x01
#define VI2C_RST_STS_BIT				0x02

#define VI2C_BUSY_STS_IDLE				0x00

#define VI2C_RST_STS_PASS				0x00

#define I2C_SLAVE_ID				0x3C

#define I2C_VD_BUFFER_STATUS		0x0200
#define I2C_VD_CHECK_ACCESS			0x8000

/**
* @brief Log level definition in libiri2c library
*/
enum Iri2cLogLevels_e
{
	/// print debug and error infomation
	IRI2C_LOG_DEBUG = 0,
	/// only print error infomation
	IRI2C_LOG_ERROR = 1,
	/// don't print debug and error infomation
	IRI2C_LOG_NO_PRINT = 2,
};

    	/**
	* @brief Error type in libiri2c library
	*/
	typedef enum {
		/** Function excute success */
		IRI2C_SUCCESS = 0,
		/** Function excute error */
		IRI2C_ERROR = -1,
        /** Invalid parameter */
		IRI2C_ERROR_PARAM = -2,
	}Iri2cError_t;

typedef struct{
    int fd;  //file description
    char* dev_node;
}I2cHandle_t;

typedef struct{
    uint16_t slave_id;
    uint16_t reg_addr;
    int32_t len;
    uint8_t* data;
    uint16_t polling_time;		///< polling timeout unit:ms
}I2cParam_t;



const char* iri2c_version(void);
void iri2c_log_register(enum Iri2cLogLevels_e log_level);
int i2c_device_open(I2cHandle_t* i2c_handle);
int i2c_data_read(I2cHandle_t* i2c_handle,I2cParam_t* cmd_param);
int i2c_data_write(I2cHandle_t* i2c_handle,I2cParam_t* cmd_param);
int i2c_device_close(I2cHandle_t* i2c_handle);

#ifdef __cplusplus
}
#endif
#endif

