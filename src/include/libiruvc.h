#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
* @file
*
* @brief Libiruvc library header file
*
*/

typedef struct _IruvcHandle_t IruvcHandle_t;

#define FORMAT_YUY2 "YUY2"
#define FORMAT_MJPEG "MJPEG"

#define FPS_CNT_MAX 32

#if defined(_WIN32)
#define DLLEXPORT __declspec(dllexport)
#elif defined(linux) || defined(unix)
#define DLLEXPORT
#endif


/**
* @brief Device's configuration
*/
typedef struct {
	unsigned int    pid;	///< pid
	unsigned int    vid;	///< vid
	char*			name;	///< device name
}DevCfg_t;


/**
* @brief Camera's supported stream information
*/
typedef struct
{
	char*			format;				///< format
	unsigned int	width;				///< resolution width
	unsigned int	height;				///< resolution height
	unsigned int	frame_size;			///< frame byte size
	unsigned int	fps[FPS_CNT_MAX];	///< frame per second
}CameraStreamInfo_t;


/**
* @brief Camera's one stream parameter
*/
typedef struct {
	DevCfg_t		dev_cfg;				///< device's configuration 
	char*			format;					///< format
	unsigned int	width;					///< resolution width
	unsigned int    height;					///< resolution height
	unsigned int	frame_size;				///< frame byte size
	unsigned int    fps;					///< frame per second
	unsigned int	timeout_ms_delay;		///< frame get's timeout timer(ms)
}CameraParam_t;


/**
* @brief Uvc control command parameters
*/
typedef struct {
	unsigned short wI2CSlaveID;			///< I2C slave ID
	unsigned short wI2CRegAddr;			///< I2C register's address
	unsigned short wLen;				///< wLen
	unsigned char* pbyData;				///< data pointer
	unsigned short polling_time;		///< polling timeout unit:ms
}I2cUsbParam_t;


/**
* @brief User's callback function and parameters
*/
typedef struct {
	IruvcHandle_t* iruvc_handle;	///< libiruvc's handle
	void* usr_func;					///< user's function
	void* usr_param;				///< parameters of user's function
}UserCallback_t;


/**
* @brief Control camera side's preview
*/
typedef enum {
	CLOSE_CAM_SIDE_PREVIEW = 0,	///< close camera side's preview
	KEEP_CAM_SIDE_PREVIEW = 1	///< keep camera side's preview
}cam_side_preview_ctl;


/**
 * @brief Return error's type
 */
typedef enum {
	/** Success */
	IRUVC_SUCCESS = 0,
	/** Error io */
	IRUVC_ERROR_IO = -1,
	/** Parameters error */
	IRUVC_ERROR_PARAM = -2,
	/** Error access */
	IRUVC_ERROR_ACCESS = -3,
	/** Find device failed */
	IRUVC_FIND_DEVICE_FAIL = -4,
	/** Entity not found */
	IRUVC_ERROR_NOT_FOUND = -5,
	/** Resource busy */
	IRUVC_ERROR_BUSY = -6,
	/** Operation timed out */
	IRUVC_ERROR_TIMEOUT = -7,
	/** Overflow */
	IRUVC_ERROR_OVERFLOW = -8,
	/** Pipe error */
	IRUVC_ERROR_PIPE = -9,
	/** System call interrupted */
	IRUVC_ERROR_INTERRUPTED = -10,
	/** Insufficient memory */
	IRUVC_ERROR_NO_MEM = -11,
	/** Operation not supported */
	IRUVC_ERROR_NOT_SUPPORTED = -12,
	/** Libiruvc's service context initialize failed */
	IRUVC_UVC_INIT_FAIL = -20,
	/** Get device list failed */
	IRUVC_GET_DEVICE_LIST_FAIL = -21,
	/** Get device info failed */
	IRUVC_GET_DEVICE_INFO_FAIL = -22,
	/** Open device fail */
	IRUVC_DEVICE_OPEN_FAIL = -23,
	/** Get device's descriptor failed */
	IRUVC_GET_DEVICE_DESCRIPTOR_FAIL = -24,
	/** Device is already opened */
	IRUVC_DEVICE_OPENED = -25,
	/** Device doesn't provide a matching stream */
	IRUVC_GET_FORMAT_FAIL = -26,
	/** User's callback fucntion is empty */
	IRUVC_USER_CALLBACK_EMPTY = -27,
	/** Device start streaming failed */
	IRUVC_START_STREAMING_FAIL = -28,
	/** Over time when getting frame */
	IRUVC_GET_FRAME_OVER_TIME = -29,
	/** Control transter failed */
	IRUVC_CONTROL_TRANSFER_FAIL = -30,
	/** Checking vdcmd send failed */
	IRUVC_CHECK_DONE_TIMEOUT = -31,
	/** Device is not UVC-compliant */
	IRUVC_ERROR_INVALID_DEVICE = -50,
	/** Mode not supported */
	IRUVC_ERROR_INVALID_MODE = -51,
	/** Resource has a callback (can't use polling and async) */
	IRUVC_ERROR_CALLBACK_EXISTS = -52,
	/** Undefined error */
	IRUVC_ERROR_OTHER = -99
}IruvcError_t;

/**
 * @brief Log message levels
 */
typedef enum {
	/// all debug&error infomation
	IRUVC_LOG_DEBUG = 0,
	/// all error infomation
	IRUVC_LOG_ERROR = 1,
	/// no print infomation(default)
	IRUVC_LOG_NO_PRINT = 2
}IruvcLogLevel_t;

/**
 * @brief Get libiruvc library's version
 *
 * @return version's string
 */
DLLEXPORT const char* iruvc_version(void);

/**
 * @brief Get libiruvc library's version number
 *
 * @return version number's string
 */
DLLEXPORT const char* iruvc_version_number(void);

/**
 * @brief Register log print via log_level
 *
 * @param[in] log_level debug information level
 *
 */
DLLEXPORT void iruvc_log_register(IruvcLogLevel_t log_level);


/**
 * @brief Create and initialize iruvc's handle
 *
 * @param NULL
 *
 * @return IruvcHandle_t struct pointer
 */
DLLEXPORT IruvcHandle_t* iruvc_create_handle(void);


/**
 * @brief Delete and release iruvc's handle
 *
 * @param[in] handle iruvc's handle
 *
 * @return NULL
 */
DLLEXPORT void iruvc_delete_handle(IruvcHandle_t* handle);


/**
 * @brief Get the same index from iruvc's handle
 *
 * @param[in] handle iruvc's handle
 *
 * @return same index
 */
DLLEXPORT int iruvc_get_same_idx(IruvcHandle_t* handle);


/**
 * @brief Initialize libiruvc's service context
 *
 * @param[in] handle iruvc's handle
 *
 * @return see IruvcError_t
 */
DLLEXPORT IruvcError_t iruvc_camera_init(IruvcHandle_t* iruvc_handle);


/**
 * @brief Return a devices' list with pid,vid and name
 *
 * @param[in] handle iruvc's handle
 * @param[out] devs_cfg devices' configuration
 *
 * @return see IruvcError_t
 */
DLLEXPORT IruvcError_t iruvc_camera_list(IruvcHandle_t* iruvc_handle, DevCfg_t devs_cfg[]);


/**
 * @brief Using a devices' configuration to find it, and list its supported stream infomation
 *
 * @param[in] handle iruvc's handle
 * @param[in] dev_cfg devices' configuration
 * @param[out] camera_stream_info camera's supported stream infomation
 *
 * @return see IruvcError_t
 */
DLLEXPORT IruvcError_t iruvc_camera_info_get(IruvcHandle_t* iruvc_handle, DevCfg_t* dev_cfg,\
											CameraStreamInfo_t camera_stream_info[]);


/**
 * @brief Open a camera device with its configuration
 *
 * @param[in] handle iruvc's handle
 * @param[in] dev_cfg devices' configuration
 *
 * @return see IruvcError_t
 */
DLLEXPORT IruvcError_t iruvc_camera_open(IruvcHandle_t* iruvc_handle, DevCfg_t dev_cfg);


/**
 * @brief Open a camera device with its configuration, 
 * and distinguish the same device via same_dev_index
 *
 * @param[in] handle iruvc's handle
 * @param[in] dev_cfg devices' configuration
 * @param[in] same_dev_index same device's index
 *
 * @return see IruvcError_t
 */
DLLEXPORT IruvcError_t iruvc_camera_open_same(IruvcHandle_t* iruvc_handle, DevCfg_t dev_cfg, int same_dev_index);


/**
 * @brief Start stream with specific one stream parameter of the camera,
 * you can use user's callback function or polling frame(usr_callback=NULL)
 *
 * @param[in] handle iruvc's handle
 * @param[in] camera_param camera's one selected stream parameter
 * @param[in] usr_callback user's callback function
 *
 * @return see IruvcError_t
 */
DLLEXPORT IruvcError_t iruvc_camera_stream_start(IruvcHandle_t* iruvc_handle, CameraParam_t camera_param, UserCallback_t* usr_callback);


/**
 * @brief Close camera's stream
 *
 * @param[in] handle iruvc's handle
 * @param[in] cam_preview control camera side's preview
 */
DLLEXPORT IruvcError_t iruvc_camera_stream_close(IruvcHandle_t* iruvc_handle, cam_side_preview_ctl cam_preview);


/**
 * @brief Disconnect the camera's device connection
 *
 * @param[in] handle iruvc's handle
 */
DLLEXPORT void iruvc_camera_close(IruvcHandle_t* iruvc_handle);


/**
 * @brief Release libiruvc's service context
 *
 * @param[in] handle iruvc's handle
 */
DLLEXPORT void iruvc_camera_release(IruvcHandle_t* iruvc_handle);


/**
 * @brief Polling frame transfered from uvc camera.
 * User can't recieve data when use user callback function
 *
 * @param[in] handle iruvc's handle
 * @param[out] raw_data raw frame data that transfered from camera
 *
 * @return see IruvcError_t
 */
DLLEXPORT IruvcError_t iruvc_frame_get(IruvcHandle_t* iruvc_handle, void *raw_data);


/**
 * @brief Polling frame transfered from uvc camera.
 * User can't recieve data when use user callback function
 *
 * @param[in] iruvc_handle iruvc's handle
 * @param[in] i2c_usb_cmd_param parameters of i2c_usb transfer
 *
 * @return see IruvcError_t
 */
DLLEXPORT IruvcError_t i2c_usb_data_read(IruvcHandle_t* iruvc_handle, I2cUsbParam_t* i2c_usb_cmd_param);


/**
 * @brief Polling frame transfered from uvc camera.
 * User can't recieve data when use user callback function
 *
 * @param[in] iruvc_handle iruvc's handle
 * @param[in] i2c_usb_cmd_param parameters of i2c_usb transfer
 *
 * @return see IruvcError_t
 */
DLLEXPORT IruvcError_t i2c_usb_data_write(IruvcHandle_t* iruvc_handle, I2cUsbParam_t* i2c_usb_cmd_param);


#ifdef __cplusplus
}
#endif
