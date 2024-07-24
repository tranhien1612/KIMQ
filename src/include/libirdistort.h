#pragma once

#ifdef __cplusplus
extern "C" {
#endif


#if defined(_WIN32)
#define DLLEXPORT __declspec(dllexport)
#elif defined(linux) || defined(unix)
#define DLLEXPORT
#endif
/**
* @brief Log level definition in irdistort library
*/
typedef enum {
    IRDISTORT_LOG_DEBUG = 0,		///< print debug and error infomation
    IRDISTORT_LOG_ERROR = 1,		///< only print error infomation
    IRDISTORT_LOG_NO_PRINT = 2,	///< don't print debug and error infomation
}irdistort_log_level_t; 



/**
*@brief 像素格式枚举
*/
typedef enum {
    IR_BYTE2_CH1,  /**< 适用于单通道，每个通道2字节的类型，如Y10，Y16等 */
    IR_BYTE1_CH3,  /**< 适用于三通道，每个通道1字节的类型，如RGB，BGR等 */
}IrUndistort_Pixel_Format;

/**
 * @brief 获取版本信息
 *
 * @return version
 */
DLLEXPORT const char* ir_undistort_version();

/**
 * @brief 设置并行计算的线程数
 *
 * @param threads 线程数
 */
DLLEXPORT void ir_undistort_set_threads(int threads);

/**
 * @brief 初始化
 *
 * @param rows 图像行数
 * @param cols 图像列数
 * @param[in] params 畸变参数
 * @param pix_fmt 像素格式
 */
DLLEXPORT void ir_undistort_init(int rows, int cols, const double* params, IrUndistort_Pixel_Format pix_fmt);

/**
 * @brief 运行算法
 *
 * @param[in] in 输入图像（首地址）
 * @param[out] out 输出图像（首地址），提前分配好内存大小
 */
DLLEXPORT void ir_undistort_run(const void* in, void* out);

/**
 * @brief 析构
 *
 */
DLLEXPORT void ir_undistort_destroy();


/**
 * @brief 用于绑定英菲的设备
 * @param[in] 设备句柄
 *
 * @return NULL
 */
DLLEXPORT void* ir_distort_bind_infisense_device(void* handle);

#ifdef __cplusplus
}
#endif
