#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <time.h>
#include <stdlib.h>

#include <gst/gst.h>
#include <glib.h>
#include <glib-unix.h>

#define NO_ERROR  0
#define ERROR     1
#define INTERRUPT 2

#define MAX_CONNECTION 2
#define ADDR "192.168.1.80"
#define PORT 8554
#define CAMERA_ID 0    
    
static GstElement *pipeline, *source, *framefilter, *tee;
static GstElement *stream_enc, *h264parse, *rtph264pay, *udpsink; //stream
static GstElement *queue_record, *encoder, *parse, *muxer, *filesink; //record
static GstElement *framefilter_img, *multifilesink; //capture
static GstPad *teepad;


/*---------------------------------------------------------------------------------------*/

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

void copyfile(){
    time_t current_time;
    struct tm *local_time;
    char timestamp[20];
    
    setenv("TZ", "Asia/Ho_Chi_Minh", 1); tzset();
    time(&current_time);
    local_time = localtime(&current_time);
    strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", local_time);
    
    const char *source_file_path = "img.jpg";
    char destination_file_path[100];
    sprintf(destination_file_path, "image/%s.jpg", timestamp);

    if (rename(source_file_path, destination_file_path) == 0) {
        printf("JPEG file moved successfully from %s to %s\n", source_file_path, destination_file_path);
    } else {
        fprintf(stderr, "Failed to move JPEG file from %s to %s\n", source_file_path, destination_file_path);
    }
}

void env_init(){
    const char *folder1_path = "video";
    const char *folder2_path = "image";

    if (!create_folder(folder1_path)) {
        fprintf(stderr, "Failed to create folder %s\n", folder1_path);
    }
    
    if (!create_folder(folder2_path)) {
        fprintf(stderr, "Failed to create folder %s\n", folder2_path);
    }
    
}
/*--------------------------------------------------------------------------*/

static gboolean recording = FALSE;

static GstPadProbeReturn unlink_cb(GstPad *pad, GstPadProbeInfo *info, gpointer user_data) {
	g_print("Unlinking...");
	GstPad *sinkpad = gst_element_get_static_pad (queue_record, "sink");
	gst_pad_unlink (teepad, sinkpad);
	gst_object_unref (sinkpad);

	gst_element_send_event(encoder, gst_event_new_eos()); 

	sleep(1);
	gst_bin_remove(GST_BIN (pipeline), queue_record);
	gst_bin_remove(GST_BIN (pipeline), encoder);
  gst_bin_remove(GST_BIN (pipeline), parse);
	gst_bin_remove(GST_BIN (pipeline), muxer);
	gst_bin_remove(GST_BIN (pipeline), filesink);

	gst_element_set_state(queue_record,  GST_STATE_NULL);
	gst_element_set_state(encoder,       GST_STATE_NULL);
  gst_element_set_state(parse,         GST_STATE_NULL);
	gst_element_set_state(muxer,         GST_STATE_NULL);
	gst_element_set_state(filesink,      GST_STATE_NULL);

	gst_object_unref(queue_record);
	gst_object_unref(encoder);
  gst_object_unref(parse);
	gst_object_unref(muxer);
	gst_object_unref(filesink);

	gst_element_release_request_pad (tee, teepad);
	gst_object_unref (teepad);

	g_print("Unlinked\n");

	return GST_PAD_PROBE_REMOVE;
}

void stopRecording() {
	g_print("stopRecording\n");
	gst_pad_add_probe(teepad, GST_PAD_PROBE_TYPE_IDLE, unlink_cb, NULL, (GDestroyNotify) g_free);
	recording = FALSE;
}


void startRecording() {
  g_print("startRecording\n");
  GstPadTemplate *templ = gst_element_class_get_pad_template(GST_ELEMENT_GET_CLASS(tee), "src_%u");
  teepad = gst_element_request_pad(tee, templ, NULL, NULL);

 	queue_record  = gst_element_factory_make("queue", "queue_record");
	encoder       = gst_element_factory_make("qtic2venc",    NULL);
  parse         = gst_element_factory_make("h264parse",    NULL);
	muxer         = gst_element_factory_make("mp4mux", NULL);
	filesink      = gst_element_factory_make("filesink", NULL);
 
 
  time_t current_time;
  struct tm *local_time;
  char timestamp[20];
  
  setenv("TZ", "Asia/Ho_Chi_Minh", 1); tzset();
  time(&current_time);
  local_time = localtime(&current_time);
  strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", local_time);
  
  char file_name[100];
  sprintf(file_name, "video/%s.mp4", timestamp);
    
	g_print("Recording to file %s", file_name);
	g_object_set(filesink, "location", file_name, NULL);
 
 	gst_bin_add_many(GST_BIN(pipeline), gst_object_ref(queue_record), gst_object_ref(encoder), gst_object_ref(muxer), gst_object_ref(parse), gst_object_ref(filesink), NULL);
	gst_element_link_many(queue_record, encoder, parse, muxer, filesink, NULL);

	gst_element_sync_state_with_parent(queue_record);
	gst_element_sync_state_with_parent(encoder);
  gst_element_sync_state_with_parent(parse);
	gst_element_sync_state_with_parent(muxer);
	gst_element_sync_state_with_parent(filesink);

	GstPad *sinkpad = gst_element_get_static_pad(queue_record, "sink");
	gst_pad_link(teepad, sinkpad);
	gst_object_unref(sinkpad);

	recording = TRUE;
}
/*--------------------------------------------------------------------------------------------------*/


static guint signal_interrupt_id;
gboolean interrupt_handler(gpointer data)
{
    GstElement *pipeline = (GstElement *)data;
    gst_element_post_message (GST_ELEMENT(pipeline),
        gst_message_new_application(GST_OBJECT(pipeline),
        gst_structure_new("UserInterrupt", "message", G_TYPE_STRING, "Interrupted", NULL)));
    
    signal_interrupt_id = 0;
    return G_SOURCE_REMOVE;
}

static int run(GstElement *pipeline)
{
    GstBus *bus;
    GstMessage *msg;
    int res = NO_ERROR;

    bus = gst_element_get_bus(GST_ELEMENT(pipeline));
    signal_interrupt_id = g_unix_signal_add(SIGINT, (GSourceFunc)interrupt_handler, pipeline);
    
    while (TRUE) {
        msg = gst_bus_poll(bus, GST_MESSAGE_ANY, -1);
        
        if (msg == NULL) {
            break;
        }

        int type = GST_MESSAGE_TYPE(msg);

        if (type == GST_MESSAGE_EOS) {
            break;
        } else if (type == GST_MESSAGE_APPLICATION) {
            const GstStructure *st = gst_message_get_structure(msg);
            g_print("GST_MESSAGE_APPLICATION\n");
            if (gst_structure_has_name(st, "UserInterrupt")) {
                res = INTERRUPT;
                break;
            }
        } else if (type == GST_MESSAGE_ERROR) {
            gchar  *debug;
            GError *error;

            gst_message_parse_error(msg, &error, &debug);
            g_free(debug);
            g_printerr("Error: %s\n", error->message);
            g_error_free(error);
            res = ERROR;
            break;
        }
        if (msg) {
            gst_message_unref (msg);
        }
    }
   
    if (msg) {
        gst_message_unref (msg);
    }
    gst_object_unref (bus);

    if (signal_interrupt_id > 0) {
        g_source_remove (signal_interrupt_id);
    }

    return res;
}



/*----------------------------------- TCP Server -------------------------------------------*/
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
      printf("Received message: %s\n", buffer);
      //copyfile();
      
     	if (recording) stopRecording();
    	else startRecording();
      
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
    serv_addr.sin_port = htons(8888);
    
    if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0){
       printf("TCP ERROR on binding\n");
       exit(1);
    }
    
    if (listen(sockfd, MAX_CONNECTION) < 0) {
        perror("Listen failed");
        exit(EXIT_FAILURE);
    }
    
    printf("Server listening on port 5765\n");
    
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
/*-----------------------------------------------------------------------------------------------*/

    
int main(int argc, char *argv[])
{
    int res;

    gst_init(&argc, &argv);
    
    pthread_t thread_id; 
    pthread_create(&thread_id, NULL, tcpServerFun, NULL);

    /* Create gstreamer elements */
    pipeline    = gst_pipeline_new("pipeline");
    source      = gst_element_factory_make("qtiqmmfsrc",   "qmmfsrc0");
    framefilter = gst_element_factory_make("capsfilter",   "capsfilter");
    tee         = gst_element_factory_make ("tee","tee");
    //stream 
    stream_enc  = gst_element_factory_make("qtic2venc",    "stream_qtic2venc");
    h264parse   = gst_element_factory_make("h264parse",    "stream_h264parse");
    rtph264pay  = gst_element_factory_make("rtph264pay",   "rtph264pay");
    udpsink     = gst_element_factory_make("udpsink",      "uUpSink");
    //capture
    framefilter_img = gst_element_factory_make("capsfilter",   "capsfilter1");
    multifilesink   = gst_element_factory_make("multifilesink","multifilesink");
    
    if (!pipeline || !source || !framefilter || !tee || !stream_enc || !h264parse || !rtph264pay || !udpsink 
        || !framefilter_img || !multifilesink ) {
        g_printerr("Create element failed.\n");
        return -1;
    }

    g_object_set (G_OBJECT(source), "name", "camera", NULL);
    g_object_set (G_OBJECT(source), "camera", CAMERA_ID, NULL);
    g_object_set (G_OBJECT(framefilter), "caps", gst_caps_from_string("video/x-raw(memory:GBM),format=NV12,framerate=60/1,width=1920,height=1080"), NULL);
    g_object_set (G_OBJECT(rtph264pay), "pt", 96, NULL);
    g_object_set (G_OBJECT(udpsink), "host", ADDR, "port", PORT, NULL);
    
    g_object_set (G_OBJECT(framefilter_img), "caps", gst_caps_from_string("image/jpeg,width=3840,height=2160"), NULL);
    g_object_set (multifilesink, "location", "img.jpg", NULL);
    g_object_set (multifilesink, "sync", true, NULL);
    g_object_set (multifilesink, "async", false, NULL);
    
    gst_bin_add_many(GST_BIN(pipeline), source, framefilter, tee,
      stream_enc, h264parse, rtph264pay, udpsink, 
      framefilter_img, multifilesink, NULL);
    
    gst_element_link_many (source, framefilter, tee, NULL);
    gst_element_link_many (stream_enc, h264parse, rtph264pay, udpsink, NULL);//stream
    gst_element_link_many (source, framefilter_img, multifilesink, NULL); //capture

      
    GstPad * tee_stream_pad = gst_element_get_request_pad (tee, "src_%u");
    g_print ("Obtained request pad %s for camera source branch.\n", gst_pad_get_name (tee_stream_pad));
    GstPad * queue_stream_pad = gst_element_get_static_pad (framefilter, "sink");
    
    if (gst_pad_link (tee_stream_pad, queue_stream_pad) != GST_PAD_LINK_OK){
      	g_printerr ("Tee could not be linked 0.\n");
      	gst_object_unref (pipeline);
      	return -1;
    }
    gst_object_unref (queue_stream_pad);
    
    gst_element_set_state (pipeline, GST_STATE_PLAYING);

    res = run(pipeline);

    // check keyboard interrupt
    if (res == INTERRUPT) {
        gst_element_send_event(pipeline, gst_event_new_eos());

        // Wait EOS
        while (TRUE) {
            res = run(pipeline);
            if (res == NO_ERROR) {
                g_print("EOS - stop\n");
                break;
            } else if (res == INTERRUPT) {
                g_print("Interrupt when waiting for EOS - stop\n");
                break;
            } else if (res == ERROR) {
                g_print("Error when waiting for EOS\n");
                break;
            }
        }
    }

    g_print("Stop\n");

    gst_element_set_state (pipeline, GST_STATE_NULL);
    gst_object_unref (GST_OBJECT (pipeline));

    gst_deinit ();

    return 0;
}
