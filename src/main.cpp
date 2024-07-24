#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <time.h>

#include <netdb.h>
#include <malloc.h>
#include <iostream>

#include <curl/curl.h>
#include "include/libiruvc.h"
#include "include/libirtemp.h"
#include "include/libirparse.h"
#include "include/libircmd.h"
#include "include/libiri2c.h"

#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>

#include <gst/gst.h>
#include <glib.h>
#include <glib-unix.h>

#define NO_ERROR  0
#define ERROR     1
#define INTERRUPT 2

#define TCP_PORT 37260
#define MAX_CONNECTION 2
#define UDP_ADDR "192.168.1.4"
#define UDP1_PORT 5600
#define UDP2_PORT 5601
#define UDP3_PORT 5602

#define CAMERA1_ID 0
#define CAMERA2_ID 1

#define loginfo(fmt, args ...)  do {printf("[%s-%d]: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)

typedef struct _GstCameraCtx GstCameraCtx;
struct _GstCameraCtx
{
  GMainLoop *mloop;
  GstElement *pipeline;
  
  GstElement *source;
  GstElement *framefilter;
  GstElement *tee;
  
  GstElement *stream_enc;
  GstElement *h264parse;
  GstElement *rtph264pay;
  GstElement *udpsink;
  
  GstElement *queue_record;
  GstElement *encoder;
  GstElement *parse;
  GstElement *muxer;
  GstElement *filesink;
  
  GstElement *framefilter_img;
  GstElement *multifilesink;

  GstElement *videoconvert;
};

GstCameraCtx camera1 = {};
GstCameraCtx camera2 = {};
GstCameraCtx camera3 = {};

/*----------------------------------------- IR Camera ----------------------------------------------*/
#define IMAGE_AND_TEMP_OUTPUT	//normal mode:get 1 image frame and temp frame at the same time 
#define PID_TYPE1	0x5830 
#define PID_TYPE2	0x5840 
#define VID_TYPE	0x0BDA 
#define TEMP_CONVERT(x) ((double)x / 64 - 273.15)

typedef struct {
    uint16_t width;
    uint16_t height;
    uint32_t byte_size;
}FrameInfo_t;

typedef struct {
    IruvcHandle_t* iruvc_handle;
    IrcmdHandle_t* ircmd_handle;
    UserCallback_t callback;
    uint8_t* raw_frame;
    uint8_t* image_frame;
    uint32_t image_byte_size;
    uint8_t* temp_frame;
    uint32_t temp_byte_size;
    FrameInfo_t image_info;
    FrameInfo_t temp_info;
    CameraParam_t camera_param;
    //timeval timer;
    uint8_t* image_tmp_frame1;
    uint8_t* image_tmp_frame2;
    uint8_t is_streaming;
}StreamFrameInfo_t;

static gboolean recording = FALSE;
cv::VideoWriter video;
cv::Mat imgRaw;
StreamFrameInfo_t stream_frame_info = { 0 };

//create the raw frame/image frame/temperature frame's buffer
int create_data_demo(StreamFrameInfo_t* stream_frame_info){
    if (stream_frame_info != NULL){
        if (stream_frame_info->raw_frame == NULL && stream_frame_info->image_frame == NULL && \
            stream_frame_info->temp_frame == NULL){
            stream_frame_info->raw_frame = (uint8_t*)malloc(stream_frame_info->camera_param.frame_size);
            stream_frame_info->image_frame = (uint8_t*)malloc(stream_frame_info->image_byte_size);
            stream_frame_info->temp_frame = (uint8_t*)malloc(stream_frame_info->temp_byte_size);
        }
    }
    return 0;
}

//recycle the raw frame/image frame/temperature frame's buffer
int destroy_data_demo(StreamFrameInfo_t* stream_frame_info){
    if (stream_frame_info != NULL){
        if (stream_frame_info->raw_frame != NULL){
            free(stream_frame_info->raw_frame);
            stream_frame_info->raw_frame = NULL;
        }

        if (stream_frame_info->image_frame != NULL){
            free(stream_frame_info->image_frame);
            stream_frame_info->image_frame = NULL;
        }

        if (stream_frame_info->temp_frame != NULL){
            free(stream_frame_info->temp_frame);
            stream_frame_info->temp_frame = NULL;
        }
    }
    return 0;
}

//init the display parameters
void display_init(StreamFrameInfo_t* stream_frame_info){
    int pixel_size = stream_frame_info->image_info.width * stream_frame_info->image_info.height * 3;
    if (stream_frame_info->image_tmp_frame1 == NULL){
        stream_frame_info->image_tmp_frame1 = (uint8_t*)malloc(pixel_size);
    }
    if (stream_frame_info->image_tmp_frame2 == NULL){
        stream_frame_info->image_tmp_frame2 = (uint8_t*)malloc(pixel_size);
    }
}

//recyle the display parameters
void display_release(StreamFrameInfo_t* stream_frame_info){
    if (stream_frame_info->image_tmp_frame1 != NULL){
        free(stream_frame_info->image_tmp_frame1);
        stream_frame_info->image_tmp_frame1 = NULL;
    }

    if (stream_frame_info->image_tmp_frame2 != NULL){
        free(stream_frame_info->image_tmp_frame2);
        stream_frame_info->image_tmp_frame2 = NULL;
    }
}

void load_stream_frame_info(StreamFrameInfo_t* stream_frame_info){
#if defined(IMAGE_AND_TEMP_OUTPUT)
    stream_frame_info->image_info.width = stream_frame_info->camera_param.width;
    stream_frame_info->image_info.height = stream_frame_info->camera_param.height / 2;
    stream_frame_info->image_byte_size = stream_frame_info->image_info.width * stream_frame_info->image_info.height * 2;

    stream_frame_info->temp_info.width = stream_frame_info->camera_param.width;
    stream_frame_info->temp_info.height = stream_frame_info->camera_param.height / 2;
    stream_frame_info->temp_byte_size = stream_frame_info->image_info.width * stream_frame_info->image_info.height * 2; //no temp frame input
#elif defined(IMAGE_OUTPUT) || defined(TEMP_OUTPUT)
    stream_frame_info->image_info.width = stream_frame_info->camera_param.width;
    stream_frame_info->image_info.height = stream_frame_info->camera_param.height;
    stream_frame_info->image_byte_size = stream_frame_info->image_info.width * stream_frame_info->image_info.height * 2;
    stream_frame_info->temp_byte_size = 0;
#endif
    create_data_demo(stream_frame_info);
}

void print_version_and_log_level(void){
    puts(irtemp_version());
    puts(irparse_version());
    puts(ircmd_version());
    puts(iruvc_version());
    puts(iri2c_version());
    
    iruvc_log_register(IRUVC_LOG_ERROR);
    irtemp_log_register(IRTEMP_LOG_ERROR);
    ircmd_log_register(IRCMD_LOG_ERROR);
    irparse_log_register(IRPARSE_LOG_ERROR);
}

Dot_t point = { 50, 50 };
Line_t line = { 128, 191, 128, 0 }; //start(x, y), end(x, y)
Area_t rect = { 50, 50, 20, 20 }; //x, y, width, height
    
//detect the point's temperature
void point_temp_demo(uint16_t* temp_data, TempDataRes_t temp_res){
    uint16_t temp = 0;
    if (get_point_temp(temp_data, temp_res, point, &temp) == IRTEMP_SUCCESS){
        printf("point --> temp=%.2f\n", TEMP_CONVERT(temp));
    }
}
    
//detect the rectangle's temperature
void rect_temp_demo(uint16_t* temp_data, TempDataRes_t temp_res){
    TempInfo_t temp_info = { 0 };

    if (get_rect_temp(temp_data, temp_res, rect, &temp_info) == IRTEMP_SUCCESS){
        printf("rect  --> max(%d, %d): temp=%.02f, min(%d, %d): temp=%.02f, avr=%.02f \n",
            temp_info.max_cord.x, temp_info.max_cord.y, TEMP_CONVERT(temp_info.max_temp),
            temp_info.min_cord.x, temp_info.min_cord.y, TEMP_CONVERT(temp_info.min_temp),
            TEMP_CONVERT(temp_info.avr_temp));
        //Point Box: fe 55 00 0a 22 01 09 [{16 03} {aa 01} {02 36} {02 26} 00] 8e, width, height, center_X, center_Y
        //char data[15] = {0xfe, 0x55, 0x00, 0x0a, 0x22, 0x01, 0x09, 0x16, 0x03, 0xaa, 0x01, 0x02, 0x36, 0x00, 0x83};
        //send(socktcp , data , sizeof(data), 0);
    }
}
    
void line_temp_demo(uint16_t* temp_data, TempDataRes_t temp_res){
    TempInfo_t temp_info = { 0 };

    if (get_line_temp(temp_data, temp_res, line, &temp_info) == IRTEMP_SUCCESS){
        printf("current line temp: max=%f, min=%f, avr=%f\n", \
            TEMP_CONVERT(temp_info.max_temp), \
            TEMP_CONVERT(temp_info.min_temp), \
            TEMP_CONVERT(temp_info.avr_temp));

    }
}

//get specific device via pid&vid from all devices
int get_dev_index_with_pid_vid(DevCfg_t devs_cfg[]){
    int cur_dev_index = 0;
    for (int i = 0; i < 64; i++){
        if ((devs_cfg[i].vid == VID_TYPE) && ((devs_cfg[i].pid == PID_TYPE1) || (devs_cfg[i].pid == PID_TYPE2))){
            cur_dev_index = i;
            printf("pid: 0x%.04x, vid: 0x%.04x, name=%s\n", devs_cfg[i].pid, devs_cfg[i].vid ,devs_cfg[i].name);
            return  cur_dev_index;
        }
    }
    printf("PID or VID is wrong!\n");
    return -1;
}

//display the frame by opencv 
int cnt = 0;
void display_one_frame(StreamFrameInfo_t* stream_frame_info, const char* title){
    if (stream_frame_info == NULL){
        return;
    }

    int pix_num = stream_frame_info->image_info.width * stream_frame_info->image_info.height;
    int width = stream_frame_info->image_info.width;
    int height = stream_frame_info->image_info.height;
    //yuv422_to_rgb(stream_frame_info->temp_frame, pix_num, stream_frame_info->image_tmp_frame2);
    stream_frame_info->image_info.byte_size = pix_num * 3;
    yuv422_to_rgb(stream_frame_info->image_frame, pix_num, stream_frame_info->image_tmp_frame2);
    
    imgRaw = cv::Mat(height, width, CV_8UC3, stream_frame_info->image_tmp_frame2);        
}

//set the camera_param from camera_stream_info and stream_index
CameraParam_t camera_para_set(DevCfg_t dev_cfg, int stream_index, CameraStreamInfo_t camera_stream_info[]){
    CameraParam_t camera_param = { 0 };
    camera_param.dev_cfg = dev_cfg;
    camera_param.format = camera_stream_info[stream_index].format;
    camera_param.width = camera_stream_info[stream_index].width;
    camera_param.height = camera_stream_info[stream_index].height;
    camera_param.frame_size = camera_param.width * camera_param.height * 2;
    camera_param.fps = camera_stream_info[stream_index].fps[0];
    camera_param.timeout_ms_delay = 1000;

    return camera_param;
}

//open camera device by camera_param
int ir_camera_open(IruvcHandle_t* handle, CameraParam_t* camera_param, int same_idx, int resolution_idx){
    DevCfg_t devs_cfg[64] = { 0 };
    CameraStreamInfo_t camera_stream_info[32] = { 0 };
    int rst = iruvc_camera_init(handle);
    if (rst < 0){
        printf("uvc_camera_init:%d\n", rst);
        return rst;
    }
    memset(devs_cfg, 0, sizeof(DevCfg_t) * 64); //clear the device list before get list
    rst = iruvc_camera_list(handle, devs_cfg);
    if (rst < 0){
        printf("uvc_camera_list:%d\n", rst);
        return rst;
    }

    int dev_index = 0;

    dev_index = get_dev_index_with_pid_vid(devs_cfg);

    if (dev_index < 0){
        printf("can not get this device!\n");
        return dev_index;
    }
    printf("cur_index = %d\n", dev_index);

    rst = iruvc_camera_info_get(handle, &(devs_cfg[dev_index]), camera_stream_info);
    if (rst < 0){
        printf("uvc_camera_info_get:%d\n", rst);
        return rst;
    }

    rst = iruvc_camera_open_same(handle, devs_cfg[dev_index], same_idx);
    if (rst < 0){
        printf("uvc_camera_open:%d\n", rst);
        return rst;
    }

    int i = 0;
    while (camera_stream_info[i].width != 0 && camera_stream_info[i].height != 0){
        printf("i: %d, size: %dx%d, fps: %d\n", i, camera_stream_info[i].width, camera_stream_info[i].height, camera_stream_info[i].fps[0]);
        i++;
    }
    *camera_param = camera_para_set(devs_cfg[dev_index], resolution_idx, camera_stream_info);
    return 0;
}

//stream start by stream_frame_info
int ir_camera_stream_on(StreamFrameInfo_t* stream_frame_info){
    int rst;

    stream_frame_info->callback.iruvc_handle = stream_frame_info->iruvc_handle;
    stream_frame_info->callback.usr_func = NULL;
    stream_frame_info->callback.usr_param = NULL;

    rst = iruvc_camera_stream_start(stream_frame_info->iruvc_handle, stream_frame_info->camera_param, \
        & stream_frame_info->callback);
    if (rst < 0){
        printf("uvc_camera_stream_start:%d\n", rst);
        return rst;
    }
    stream_frame_info->is_streaming = 1;
    return rst;
}

//stream stop
int ir_camera_stream_off(StreamFrameInfo_t* stream_frame_info){
    int rst = 0;

    rst = iruvc_camera_stream_close(stream_frame_info->iruvc_handle, CLOSE_CAM_SIDE_PREVIEW);
    if (rst < 0){
        return rst;
    }

    destroy_data_demo(stream_frame_info);
    stream_frame_info->is_streaming = 0;
    iruvc_camera_close(stream_frame_info->iruvc_handle);
    return rst;
}

void* stream_operation(StreamFrameInfo_t* stream_frame_info){
    TempDataRes_t temp_res;
    char title[80];
    int same_idx = 0;
    same_idx = iruvc_get_same_idx(stream_frame_info->iruvc_handle);
    sprintf(title, "Test%d", same_idx);
    if (stream_frame_info == NULL){
        return NULL;
    }

    unsigned int i = 0; //int
    int r = 0;
    int overtime_cnt = 0;
    int overtime_threshold = 2;
    while (stream_frame_info->is_streaming){// && (i <= 10000 * stream_frame_info->camera_param.fps))//display stream_time seconds
        r = iruvc_frame_get(stream_frame_info->iruvc_handle, stream_frame_info->raw_frame);
        if (r < 0){
            overtime_cnt++;
        }else{
            overtime_cnt = 0;
        }
        if (r < 0 && overtime_cnt >= overtime_threshold){
            ir_camera_stream_off(stream_frame_info);
            printf("uvc_frame_get failed\n ");
            return NULL;
        }
        if (stream_frame_info->raw_frame != NULL){
            raw_data_cut((uint8_t*)stream_frame_info->raw_frame, stream_frame_info->image_byte_size, \
                stream_frame_info->temp_byte_size, (uint8_t*)stream_frame_info->image_frame, \
                (uint8_t*)stream_frame_info->temp_frame);           
#if defined(TEMP_OUTPUT) 
            basic_y16_preview(stream_frame_info->ircmd_handle, BASIC_Y16_MODE_TEMPERATURE); //BASIC_Y16_MODE_YUV
#endif
            display_one_frame(stream_frame_info, title);
            if (i % 100 == 0){
                temp_res = {stream_frame_info->temp_info.width, stream_frame_info->temp_info.height};
                point_temp_demo((uint16_t*)stream_frame_info->temp_frame, temp_res);
                rect_temp_demo((uint16_t*)stream_frame_info->temp_frame, temp_res);
            }
        }
        i++;
    }

    ir_camera_stream_off(stream_frame_info);
    printf("stream thread exit!!\n");
    return NULL;
}

void *irCameraFun(void *vargp) { 
    int resolution_idx = 1;
    int same_idx = 0;

    setpriority(PRIO_PROCESS, 0, -20);
    print_version_and_log_level();
    
    IruvcHandle_t* iruvc_handle = iruvc_create_handle();
    int rst = ir_camera_open(iruvc_handle, &stream_frame_info.camera_param, same_idx, resolution_idx);
    if (rst < 0){
        puts("ir camera open failed!\n");
        getchar();
        return 0;
    }
    load_stream_frame_info(&stream_frame_info);
    stream_frame_info.iruvc_handle = iruvc_handle;
    stream_frame_info.ircmd_handle = ircmd_create_handle(iruvc_handle, VDCMD_I2C_USB_VDCMD);

    //uint16_t value = 0;
    //0:white_hot, 1:reserved, 2:sepia, 3: ironbow, 4: rainbow, 5: night, 6: aurora, 7: red_hot, 8: jungle, 9: medical, 10: black_hot, 11: glory_hot
    //basic_pseudo_color(stream_frame_info.ircmd_handle, SET_PARAMS_STA, &value);

    display_init(&stream_frame_info);
    rst = ir_camera_stream_on(&stream_frame_info);
    if (rst < 0){
        puts("ir camera stream on failed!\n");
        getchar();
        return 0;
    }
    stream_operation(&stream_frame_info);
    display_release(&stream_frame_info);
    puts("EXIT");
    getchar();
    return 0;
}

static gboolean push_frame(GstCameraCtx *data) {
    if(recording == TRUE){
        video.write(imgRaw);
    }
    cv::Mat frame(640, 512, CV_8UC3);
    if(!imgRaw.empty()) frame = imgRaw;

    GstBuffer *buffer;
    GstFlowReturn ret;
    guint size = frame.cols * frame.rows * frame.channels();
    buffer = gst_buffer_new_allocate(NULL, size, NULL);
    GstMapInfo map;
    gst_buffer_map(buffer, &map, GST_MAP_WRITE);
    memcpy(map.data, frame.data, size);
    gst_buffer_unmap(buffer, &map);
    GST_BUFFER_PTS(buffer) = GST_CLOCK_TIME_NONE;
    GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(1, GST_SECOND, 30 * 1);
    g_signal_emit_by_name(data->source, "push-buffer", buffer, &ret);
    gst_buffer_unref(buffer);
    if (ret != GST_FLOW_OK) {
        g_printerr("Error pushing buffer to appsrc: %s\n", gst_flow_get_name(ret));
        return FALSE;
    }
    return TRUE;
}
/*--------------------------------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------------------------------*/
int folder_exists(const char *folder_path) {
    struct stat st;
    if (stat(folder_path, &st) == 0) {
        if (S_ISDIR(st.st_mode)) {
            return 1; // Folder is created
        } else {
            return 0; // Folder is not created
        }
    }
    return 0;
}

int create_folder(const char *folder_path) {
    if (folder_exists(folder_path)) {
      return 1;
    }
    
    int status = mkdir(folder_path, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (status == -1) {
        perror("mkdir");
        return 0;
    }
    return 1;
}

void capture(){
    time_t current_time;
    struct tm *local_time;
    char timestamp[20];
    
    setenv("TZ", "Asia/Ho_Chi_Minh", 1); tzset();
    time(&current_time);
    local_time = localtime(&current_time);
    strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", local_time);
    
    const char *srcFile = "img1.jpg";
    char desFile[100];
    sprintf(desFile, "../image/rgb_%s.jpg", timestamp);

    if (rename(srcFile, desFile) == 0) {
        printf("capture successfully: %s\n", desFile);
    } else {
        fprintf(stderr, "capture failed: %s\n", desFile);
    }

    sprintf(desFile, "../image/ir_%s.jpg", timestamp);
    cv::imwrite(desFile, imgRaw);
}

void env_init(){
    const char *folder1_path = "../video";
    const char *folder2_path = "../image";

    if (!create_folder(folder1_path)) {
        fprintf(stderr, "Failed to create folder %s\n", folder1_path);
    }
    
    if (!create_folder(folder2_path)) {
        fprintf(stderr, "Failed to create folder %s\n", folder2_path);
    }
    
}
/*--------------------------------------------------------------------------------------------------*/


static GstPad *teepad;

static GstPadProbeReturn unlink_cb(GstPad *pad, GstPadProbeInfo *info, gpointer user_data) {
	//g_print("Unlinking...");
	GstPad *sinkpad = gst_element_get_static_pad (camera1.queue_record, "sink");
	gst_pad_unlink (teepad, sinkpad);
	gst_object_unref (sinkpad);

	gst_element_send_event(camera1.encoder, gst_event_new_eos()); 

	sleep(1);
	gst_bin_remove(GST_BIN (camera1.pipeline), camera1.queue_record);
	gst_bin_remove(GST_BIN (camera1.pipeline), camera1.encoder);
    gst_bin_remove(GST_BIN (camera1.pipeline), camera1.parse);
	gst_bin_remove(GST_BIN (camera1.pipeline), camera1.muxer);
	gst_bin_remove(GST_BIN (camera1.pipeline), camera1.filesink);

	gst_element_set_state(camera1.queue_record,  GST_STATE_NULL);
	gst_element_set_state(camera1.encoder,       GST_STATE_NULL);
    gst_element_set_state(camera1.parse,         GST_STATE_NULL);
	gst_element_set_state(camera1.muxer,         GST_STATE_NULL);
	gst_element_set_state(camera1.filesink,      GST_STATE_NULL);

	gst_object_unref(camera1.queue_record);
	gst_object_unref(camera1.encoder);
    gst_object_unref(camera1.parse);
	gst_object_unref(camera1.muxer);
	gst_object_unref(camera1.filesink);

	gst_element_release_request_pad (camera1.tee, teepad);
	gst_object_unref (teepad);

	//g_print("Unlinked\n");

	return GST_PAD_PROBE_REMOVE;
}

void stopRecording() {
	g_print("---------------------- stopRecording ----------------------\n");
	gst_pad_add_probe(teepad, GST_PAD_PROBE_TYPE_IDLE, unlink_cb, NULL, (GDestroyNotify) g_free);

    if(recording == TRUE){
        video.release();
    }

	recording = FALSE;
}

void startRecording() {
    g_print("---------------------- startRecording ----------------------\n");
    GstPadTemplate *templ = gst_element_class_get_pad_template(GST_ELEMENT_GET_CLASS(camera1.tee), "src_%u");
    teepad = gst_element_request_pad(camera1.tee, templ, NULL, NULL);

 	camera1.queue_record  = gst_element_factory_make("queue", "queue_record");
	camera1.encoder       = gst_element_factory_make("qtic2venc",    NULL);
    camera1.parse         = gst_element_factory_make("h264parse",    NULL);
	camera1.muxer         = gst_element_factory_make("mp4mux", NULL);
	camera1.filesink      = gst_element_factory_make("filesink", NULL);
 
    time_t current_time;
    struct tm *local_time;
    char timestamp[20];
    
    setenv("TZ", "Asia/Ho_Chi_Minh", 1); tzset();
    time(&current_time);
    local_time = localtime(&current_time);
    strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", local_time);
    
    char file_name[100];
    sprintf(file_name, "../video/rgb_%s.mp4", timestamp);
	g_object_set(camera1.filesink, "location", file_name, NULL);
 
 	gst_bin_add_many(GST_BIN(camera1.pipeline), camera1.queue_record, camera1.encoder, camera1.muxer, camera1.parse,  camera1.filesink, NULL);
	gst_element_link_many(camera1.queue_record, camera1.encoder, camera1.parse, camera1.muxer, camera1.filesink, NULL);

	gst_element_sync_state_with_parent(camera1.queue_record);
	gst_element_sync_state_with_parent(camera1.encoder);
    gst_element_sync_state_with_parent(camera1.parse);
	gst_element_sync_state_with_parent(camera1.muxer);
	gst_element_sync_state_with_parent(camera1.filesink);

	GstPad *sinkpad = gst_element_get_static_pad(camera1.queue_record, "sink");
	gst_pad_link(teepad, sinkpad);
	gst_object_unref(sinkpad);

    sprintf(file_name, "../video/ir_%s.mp4", timestamp);
    video = cv::VideoWriter(file_name, cv::VideoWriter::fourcc('H', '2', '6', '4'), 30, cv::Size(640, 512));

	recording = TRUE;
}
/*--------------------------------------------------------------------------------------------------*/

/*--------------------------------------- TCP Server -----------------------------------------------*/
void str2hex(const unsigned char *input, int len, char *output) {
    static const char hex_digits[] = "0123456789ABCDEF";
    for (int i = 0; i < len; ++i) {
        output[i * 2] = hex_digits[(input[i] >> 4) & 0xF];
        output[i * 2 + 1] = hex_digits[input[i] & 0xF];
    }
    output[len * 2] = '\0';
}

void *connection_handler(void *socket_desc)
{
    int sock = *(int*)socket_desc;
    char buffer[2024];  bzero(buffer, 2024);
    //char hex_buffer[1024 * 2 + 1];
    int msgLen = 0;
    
    //char* message = "Greetings! I am your connection handler\n";
    //write(sock , message , strlen(message));
    
    //char message[10] = {0x01, 0x02, 0x03, 0x04, 0x05};
    //write(sock, message, 5);
    
    while( (msgLen = recv(sock , buffer , 2024 , 0)) > 0 ){
      buffer[msgLen] = '\0';
      //printf("Received message: %s\n", buffer);
      //printf("Received message: 0x%.02x, 0x%.02x\n", buffer[3], buffer[4]);
      
      //2header, 1ctrl, 2len, 2seq, 1id, data, 2crc16
      //photo: 55 66 01 01 00 00 00 0c 00 34 ce
      //video: 55 66 01 01 00 00 00 0c 02 d1 12
      
      if(buffer[0] == 0x55 && buffer[1] == 0x66){
          if(buffer[7] == 0x0c && buffer[8] == 0x00){
              capture();
              char message[10] = {0x55, 0x66, 0x02, 0x01, 0x02, 0x00, 0x0b, 0x00, 0x49, 0x62};
              write(sock, message, 10);
          }else if(buffer[7] == 0x0c && buffer[8] == 0x02){
              if(recording == false) startRecording();
              else stopRecording();
          }else if(buffer[7] == 0xff){

          }
      
      }
      
      /*
      if(buffer[0] == 0x01){
          capture();
      }else if(buffer[0] == 0x02 && recording == false){
          startRecording();
      }else if(buffer[0] == 0x02 && recording == true){
          stopRecording();
      }else if(buffer[0] == 0x03){
          switchCamera();
      }
      */
      
    }
    
    if(msgLen == 0){
      printf("Client disconnected\n");
      fflush(stdout);
    }else if(msgLen == -1){
      printf("recv failed\n");
    }
    close(sock);
    return 0;
}

void *tcpServerFun(void *vargp) 
{ 
    env_init();
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0){
        printf("TCP ERROR opening socket\n");
        exit(1);
    }
    
    struct sockaddr_in serv_addr, cli_addr;
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(TCP_PORT);
    
    if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0){
       printf("TCP ERROR on binding\n");
       exit(1);
    }
    
    if (listen(sockfd, MAX_CONNECTION) < 0) {
        perror("Listen failed");
        exit(EXIT_FAILURE);
    }
    
    printf("Server listening on port %d\n",TCP_PORT);
    
    pthread_t client_id;
    int newsockfd;
    socklen_t clilen = sizeof(cli_addr);
    while( (newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen)) ){
       printf("Connection accepted\n");
       if( pthread_create( &client_id , NULL , connection_handler, (void*) &newsockfd) < 0){
            perror("could not create thread");
            close(newsockfd);
            continue; 
       }
       pthread_detach(client_id);
    }
    close(sockfd);
    return NULL; 
} 
/*-------------------------------------------------------------------------------------------------*/

/*--------------------------------------- Camera Init ---------------------------------------------*/
void camera1_Init(){
    /* Create gstreamer elements */
    camera1.pipeline    = gst_pipeline_new("pipeline");
    camera1.source      = gst_element_factory_make("qtiqmmfsrc",   "qmmfsrc0");
    camera1.framefilter = gst_element_factory_make("capsfilter",   "capsfilter");
    camera1.tee         = gst_element_factory_make ("tee","tee");
    //stream 
    camera1.stream_enc  = gst_element_factory_make("qtic2venc",    "stream_qtic2venc");
    camera1.h264parse   = gst_element_factory_make("h264parse",    "stream_h264parse");
    camera1.rtph264pay  = gst_element_factory_make("rtph264pay",   "rtph264pay");
    camera1.udpsink     = gst_element_factory_make("udpsink",      "uUpSink");
    //capture
    camera1.framefilter_img = gst_element_factory_make("capsfilter",   "capsfilter1");
    camera1.multifilesink   = gst_element_factory_make("multifilesink","multifilesink");
    
    if (!camera1.pipeline || !camera1.source || !camera1.framefilter || !camera1.tee || !camera1.stream_enc || !camera1.h264parse || !camera1.rtph264pay || !camera1.udpsink 
        || !camera1.framefilter_img || !camera1.multifilesink ) {
        g_printerr("Create element failed.\n");
        return;
    }

    g_object_set (G_OBJECT(camera1.source), "name", "camera0", NULL);
    g_object_set (G_OBJECT(camera1.source), "camera", CAMERA1_ID, NULL);
    g_object_set (G_OBJECT(camera1.framefilter), "caps", gst_caps_from_string("video/x-raw(memory:GBM),format=NV12,framerate=60/1,width=1920,height=1080"), NULL);
    g_object_set (G_OBJECT(camera1.rtph264pay), "pt", 96, NULL);
    g_object_set (G_OBJECT(camera1.udpsink), "host", UDP_ADDR, "port", UDP1_PORT, NULL);
    
    g_object_set (G_OBJECT(camera1.framefilter_img), "caps", gst_caps_from_string("image/jpeg,width=3840,height=2160"), NULL);
    g_object_set (G_OBJECT(camera1.multifilesink), "location", "img1.jpg", NULL);
    g_object_set (G_OBJECT(camera1.multifilesink), "sync", true, NULL);
    g_object_set (G_OBJECT(camera1.multifilesink), "async", false, NULL);
    
    gst_bin_add_many(GST_BIN(camera1.pipeline), camera1.source, camera1.framefilter, camera1.tee,
      camera1.stream_enc, camera1.h264parse, camera1.rtph264pay, camera1.udpsink, 
      camera1.framefilter_img, camera1.multifilesink, NULL);
    
    gst_element_link_many (camera1.source, camera1.framefilter, camera1.tee, NULL);
    gst_element_link_many (camera1.stream_enc, camera1.h264parse, camera1.rtph264pay, camera1.udpsink, NULL);//stream
    gst_element_link_many (camera1.source, camera1.framefilter_img, camera1.multifilesink, NULL); //capture
      
    GstPad * tee_stream_pad = gst_element_get_request_pad (camera1.tee, "src_%u");
    //g_print ("Obtained request pad %s for camera source branch.\n", gst_pad_get_name (tee_stream_pad));
    GstPad * queue_stream_pad = gst_element_get_static_pad (camera1.stream_enc, "sink");
    
    if (gst_pad_link (tee_stream_pad, queue_stream_pad) != GST_PAD_LINK_OK){
      	g_printerr ("Tee could not be linked for camera1.\n");
      	gst_object_unref (camera1.pipeline);
      	return;
    }
    gst_object_unref (queue_stream_pad);
    
    gst_element_set_state (camera1.pipeline, GST_STATE_PLAYING);
}

void camera2_Init(){
    /* Create gstreamer elements */
    camera2.pipeline    = gst_pipeline_new("pipeline");
    camera2.source      = gst_element_factory_make("qtiqmmfsrc",   "qmmfsrc0");
    camera2.framefilter = gst_element_factory_make("capsfilter",   "capsfilter");
    camera2.tee         = gst_element_factory_make ("tee","tee");
    //stream 
    camera2.stream_enc  = gst_element_factory_make("qtic2venc",    "stream_qtic2venc");
    camera2.h264parse   = gst_element_factory_make("h264parse",    "stream_h264parse");
    camera2.rtph264pay  = gst_element_factory_make("rtph264pay",   "rtph264pay");
    camera2.udpsink     = gst_element_factory_make("udpsink",      "uUpSink");
    //capture
    camera2.framefilter_img = gst_element_factory_make("capsfilter",   "capsfilter1");
    camera2.multifilesink   = gst_element_factory_make("multifilesink","multifilesink");
    
    if (!camera2.pipeline || !camera2.source || !camera2.framefilter || !camera2.tee || !camera2.stream_enc || !camera2.h264parse || !camera2.rtph264pay || !camera2.udpsink 
        || !camera2.framefilter_img || !camera2.multifilesink ) {
        g_printerr("Create element failed.\n");
        return;
    }

    g_object_set (G_OBJECT(camera2.source), "name", "camera0", NULL);
    g_object_set (G_OBJECT(camera2.source), "camera", CAMERA2_ID, NULL);
    g_object_set (G_OBJECT(camera2.framefilter), "caps", gst_caps_from_string("video/x-raw(memory:GBM),format=NV12,framerate=60/1,width=1920,height=1080"), NULL);
    g_object_set (G_OBJECT(camera2.rtph264pay), "pt", 96, NULL);
    g_object_set (G_OBJECT(camera2.udpsink), "host", UDP_ADDR, "port", UDP2_PORT, NULL);
    
    g_object_set (G_OBJECT(camera2.framefilter_img), "caps", gst_caps_from_string("image/jpeg,width=3840,height=2160"), NULL);
    g_object_set (G_OBJECT(camera2.multifilesink), "location", "img2.jpg", NULL);
    g_object_set (G_OBJECT(camera2.multifilesink), "sync", true, NULL);
    g_object_set (G_OBJECT(camera2.multifilesink), "async", false, NULL);
    
    gst_bin_add_many(GST_BIN(camera2.pipeline), camera2.source, camera2.framefilter, camera2.tee,
      camera2.stream_enc, camera2.h264parse, camera2.rtph264pay, camera2.udpsink, 
      camera2.framefilter_img, camera2.multifilesink, NULL);
    
    gst_element_link_many (camera2.source, camera2.framefilter, camera2.tee, NULL);
    gst_element_link_many (camera2.stream_enc, camera2.h264parse, camera2.rtph264pay, camera2.udpsink, NULL);//stream
    gst_element_link_many (camera2.source, camera2.framefilter_img, camera2.multifilesink, NULL); //capture
      
    GstPad * tee_stream_pad = gst_element_get_request_pad (camera2.tee, "src_%u");
    //g_print ("Obtained request pad %s for camera source branch.\n", gst_pad_get_name (tee_stream_pad));
    GstPad * queue_stream_pad = gst_element_get_static_pad (camera2.stream_enc, "sink");
    
    if (gst_pad_link (tee_stream_pad, queue_stream_pad) != GST_PAD_LINK_OK){
      	g_printerr ("Tee could not be linked for camera2.\n");
      	gst_object_unref (camera2.pipeline);
      	return;
    }
    gst_object_unref (queue_stream_pad);
    
    gst_element_set_state (camera2.pipeline, GST_STATE_PLAYING);
}

void camera3_Init(){
    /* Create gstreamer elements */
    camera3.pipeline    = gst_pipeline_new("pipeline3");
    camera3.source      = gst_element_factory_make("appsrc", "appsrc");
    camera3.framefilter = gst_element_factory_make("capsfilter", "capsfilter3");
    // camera3.tee         = gst_element_factory_make ("tee","tee");
    camera3.videoconvert= gst_element_factory_make("videoconvert", "videoconvert");
    camera3.stream_enc  = gst_element_factory_make("x264enc", "x264enc");
    camera3.h264parse   = gst_element_factory_make("h264parse",    "h264parse3");
    camera3.rtph264pay  = gst_element_factory_make("rtph264pay",   "rtph264pay3");
    camera3.udpsink     = gst_element_factory_make("udpsink",      "uUpSink3");
    
    if (!camera3.pipeline || !camera3.source || !camera3.framefilter /*|| !camera3.tee*/ || !camera3.videoconvert || !camera3.stream_enc 
        || !camera3.h264parse || !camera3.rtph264pay || !camera3.udpsink ) {
        g_printerr("Create element failed.\n");
        return;
    }

    g_object_set (G_OBJECT(camera3.framefilter), "caps", gst_caps_from_string("video/x-raw,format=RGB,framerate=30/1,width=640,height=512"), NULL);
    g_object_set(G_OBJECT(camera3.stream_enc),
                "speed-preset", 1,  // Ultrafast encoding speed
                "tune", 0x00000004,  // Tune for latency
                "byte-stream", TRUE, // Output byte stream format
                NULL);

    g_object_set (G_OBJECT(camera3.rtph264pay), "pt", 96, NULL);
    g_object_set (G_OBJECT(camera3.udpsink), "host", UDP_ADDR, "port", UDP3_PORT, NULL);
    
    gst_bin_add_many(GST_BIN(camera3.pipeline), camera3.source, camera3.framefilter, /*camera3.tee,*/ camera3.videoconvert,
      camera3.stream_enc, camera3.h264parse, camera3.rtph264pay, camera3.udpsink, NULL);
    gst_element_link_many (camera3.source, camera3.framefilter, camera3.videoconvert, camera3.stream_enc, camera3.h264parse, camera3.rtph264pay, camera3.udpsink, NULL);//stream
    // gst_element_link_many (camera3.source, camera3.framefilter, camera3.tee, NULL);
    // gst_element_link_many (camera3.stream_enc, camera3.h264parse, camera3.rtph264pay, camera3.udpsink, NULL);//stream  
    // GstPad * tee_stream_pad = gst_element_get_request_pad (camera3.tee, "src_%u");
    // GstPad * queue_stream_pad = gst_element_get_static_pad (camera3.stream_enc, "sink");
    
    // if (gst_pad_link (tee_stream_pad, queue_stream_pad) != GST_PAD_LINK_OK){
    //   	g_printerr ("Tee could not be linked for camera1.\n");
    //   	gst_object_unref (camera1.pipeline);
    //   	return;
    // }
    // gst_object_unref (queue_stream_pad);

    gst_element_set_state (camera3.pipeline, GST_STATE_PLAYING);
    g_timeout_add(1000 / 30, (GSourceFunc)push_frame, &camera3);
}
/*-------------------------------------------------------------------------------------------------*/

int main(int argc, char *argv[])
{
    gst_init(&argc, &argv);
    
    camera1_Init();
    // camera2_Init();
    camera3_Init();
    
    pthread_t thread1, thread2;
    pthread_create(&thread1, NULL, tcpServerFun, NULL);
    pthread_create(&thread2, NULL, irCameraFun, NULL);

    GMainLoop *loop;
    loop = g_main_loop_new(NULL, FALSE);
    g_main_loop_run(loop);
    
    gst_element_set_state (camera1.pipeline, GST_STATE_NULL);
    // gst_element_set_state (camera2.pipeline, GST_STATE_NULL);
    gst_element_set_state (camera3.pipeline, GST_STATE_NULL);
    gst_object_unref (GST_OBJECT (camera1.pipeline));
    // gst_object_unref (GST_OBJECT (camera2.pipeline));
    gst_object_unref (GST_OBJECT (camera3.pipeline));

    gst_deinit ();
    return 0;
}
