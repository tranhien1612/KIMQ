/*
* Copyright (c) 2020, 2022 Qualcomm Innovation Center, Inc.  All Rights Reserved.
*
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

/*
 * tcp_server.c:
 *
 * A sample app based on gstreamer
 * The purpose is helping users to learn how to implement the
 * tcp streaming server of ISP camera with gstreamer on the
 * Qualcomm platform through this sample app.
 * 
 * Client should use VLC player to play the url:
 *    tcp://<ip>:<port>/
 */
#include <stdio.h>
#include <stdbool.h>
#include <pthread.h>
#include <glib.h>
#include <gst/gst.h>
#include <glib-unix.h>

#define NUMBER_OF_CAMERA 1
#define ADDR "192.168.1.80"
#define PORT 8554

typedef struct _GstCameraSwitchCtx GstCameraSwitchCtx;
struct _GstCameraSwitchCtx
{
  // Pointer to the pipeline
  GstElement *pipeline;
  // Pointer to the mainloop
  GMainLoop *mloop;

  GstElement *qtiqmmfsrc_0;
  GstElement *qtiqmmfsrc_1;
  GstElement *qtiqmmfsrc_2;
  GstElement *capsfilter;

  GstElement *h264parse;
  GstElement *qtic2venc;
  GstElement *rtph264pay;
  GstElement *sink;

  guint cameraID;
  GMutex lock;
  gboolean exit;
  gboolean use_display;
  guint camera0;
  guint camera1;
  guint camera2;
};

void switch_camera (GstCameraSwitchCtx *cameraswitchctx) {

  GstElement *qmmf = NULL;
  GstElement *qmmf_second = NULL;
  guint id = 0;

  if(NUMBER_OF_CAMERA >= 2){
  g_print ("\n---------------- Switch_camera...\n");

  if (cameraswitchctx->cameraID == 1) {
    qmmf = gst_element_factory_make ("qtiqmmfsrc", "qtiqmmfsrc_0");
    g_object_set (G_OBJECT (qmmf), "name", "qmmf_0", NULL);
    g_object_set (G_OBJECT (qmmf), "camera", cameraswitchctx->camera0, NULL);
    cameraswitchctx->qtiqmmfsrc_0 = qmmf;
    id = 0;

    qmmf_second = cameraswitchctx->qtiqmmfsrc_1;
  } else if(cameraswitchctx->cameraID == 0){
    qmmf = gst_element_factory_make ("qtiqmmfsrc", "qtiqmmfsrc_1");
    g_object_set (G_OBJECT (qmmf), "name", "qmmf_1", NULL);
    g_object_set (G_OBJECT (qmmf), "camera", cameraswitchctx->camera1, NULL);
    cameraswitchctx->qtiqmmfsrc_1 = qmmf;
    id = 1;
    qmmf_second = cameraswitchctx->qtiqmmfsrc_0;
  } 

  gst_bin_add (GST_BIN (cameraswitchctx->pipeline), qmmf);
  gst_element_sync_state_with_parent (qmmf);
  gst_element_unlink (qmmf_second, cameraswitchctx->capsfilter);

  // Link the next camera stream
  g_print ("Linking next camera stream...\n");
  if (!gst_element_link (qmmf, cameraswitchctx->capsfilter)) {
    g_printerr ("Error: Link cannot be done!\n");
    return;
  }
  
  g_print ("Linked next camera stream successfully \n");

  // Set NULL state to the unlinked elemets
  gst_element_set_state (qmmf_second, GST_STATE_NULL);
  gst_bin_remove (GST_BIN (cameraswitchctx->pipeline), qmmf_second);
  cameraswitchctx->cameraID = id;
  }
}

static gboolean bus_callback(GstBus *bus, GstMessage *msg, gpointer data)
{
    GMainLoop *loop = (GMainLoop *)data;

    int type = GST_MESSAGE_TYPE(msg);
    
    if (type == GST_MESSAGE_ERROR) {
        gchar  *debug;
        GError *error;
        gst_message_parse_error(msg, &error, &debug);
        g_free(debug);
        g_printerr("Error: %s\n", error->message);
        g_error_free(error);
        g_main_loop_quit(loop);
    }

    return TRUE;
}
/*
static gboolean handle_interrupt_signal (gpointer userdata)
{
  GstCameraSwitchCtx *cameraswitchctx = (GstCameraSwitchCtx *) userdata;
  GstState state, pending;

  g_print ("\n\nReceived an interrupt signal, send EOS ...\n");

  if (!gst_element_get_state (
      cameraswitchctx->pipeline, &state, &pending, GST_CLOCK_TIME_NONE)) {
    gst_printerr ("ERROR: get current state!\n");
    gst_element_send_event (cameraswitchctx->pipeline, gst_event_new_eos ());
    return TRUE;
  }

  if (state == GST_STATE_PLAYING) {
    gst_element_send_event (cameraswitchctx->pipeline, gst_event_new_eos ());
  } else {
    g_main_loop_quit (cameraswitchctx->mloop);
  }

  g_mutex_lock (&cameraswitchctx->lock);
  cameraswitchctx->exit = true;
  g_mutex_unlock (&cameraswitchctx->lock);

  return TRUE;
}
*/
static void * thread_fn (gpointer user_data)
{
  GstCameraSwitchCtx *cameraswitchctx = (GstCameraSwitchCtx *) user_data;

 while (true) {
    sleep (5);
    g_mutex_lock (&cameraswitchctx->lock);
    if (cameraswitchctx->exit) {
      g_mutex_unlock (&cameraswitchctx->lock);
      return NULL;
    }
    g_mutex_unlock (&cameraswitchctx->lock);

    switch_camera (cameraswitchctx);
  }

  return NULL;
}

//Follow 1: CamSrc -> qtic2venc -> h264parse -> rtph264pay -> udpsink
int main(int argc, char *argv[])
{
    GMainLoop *loop;
    GstElement *pipeline, *qtiqmmfsrc_0, *capsfilter, *qtic2venc, *h264parse, *rtph264pay, *sink;
    GstBus *bus;
    guint bus_id;
    //guint intrpt_watch_id = 0;
    
    
    gst_init(&argc, &argv);
    
    GOptionContext *ctx = NULL;
    GstCameraSwitchCtx cameraswitchctx = {};
    cameraswitchctx.exit = false;
    //cameraswitchctx.use_display = false;
    cameraswitchctx.camera0 = 0;
    cameraswitchctx.camera1 = 1;
    g_mutex_init (&cameraswitchctx.lock);
  
    GOptionEntry entries[] = {/*
      { "display", 'd', 0, G_OPTION_ARG_NONE,
        &cameraswitchctx.use_display,
        "Enable display",
        "Parameter for enable display output"
      },*/
      { "camera0", 'm', 0, G_OPTION_ARG_INT,
        &cameraswitchctx.camera0,
        "ID of camera0",
        NULL,
      },
      { "camera1", 's', 0, G_OPTION_ARG_INT,
        &cameraswitchctx.camera1,
        "ID of camera1",
        NULL,
      },
      { "camera2", 'm', 0, G_OPTION_ARG_INT,
        &cameraswitchctx.camera2,
        "ID of camera2",
        NULL,
      },
      { NULL }
    };
  
    if ((ctx = g_option_context_new ("DESCRIPTION")) != NULL) {
      gboolean success = FALSE;
      GError *error = NULL;
  
      g_option_context_add_main_entries (ctx, entries, NULL);
      g_option_context_add_group (ctx, gst_init_get_option_group ());
  
      success = g_option_context_parse (ctx, &argc, &argv, &error);
      g_option_context_free (ctx);
  
      if (!success && (error != NULL)) {
        g_printerr ("ERROR: Failed to parse command line options: %s!\n",
             GST_STR_NULL (error->message));
        g_clear_error (&error);
        return -EFAULT;
      } else if (!success && (NULL == error)) {
        g_printerr ("ERROR: Initializing: Unknown error!\n");
        return -EFAULT;
      }
    } else {
      g_printerr ("ERROR: Failed to create options context!\n");
      return -EFAULT;
    }
    

    pipeline        = gst_pipeline_new("video-streaming");
    qtiqmmfsrc_0    = gst_element_factory_make("qtiqmmfsrc",   "qtiqmmfsrc_0");
    capsfilter      = gst_element_factory_make("capsfilter",   "frame-filter");
    qtic2venc       = gst_element_factory_make("qtic2venc",    "QtiC2venc");
    h264parse       = gst_element_factory_make("h264parse",    "h264-parse");
    rtph264pay      = gst_element_factory_make("rtph264pay",   "rtph264pay");
    sink            = gst_element_factory_make("udpsink",      "uUpSink");
    
    if (!pipeline || !qtiqmmfsrc_0 || !capsfilter || !qtic2venc || !h264parse || !rtph264pay || !sink) {
        g_printerr("Create element failed.\n");
        return -1;
    }
     
    g_object_set (G_OBJECT(capsfilter), "caps", gst_caps_from_string("video/x-raw,framerate=60/1,width=1920,height=1080"), NULL);
    g_object_set (G_OBJECT(rtph264pay), "pt", 96, NULL);
    g_object_set (G_OBJECT(sink), "host", ADDR, "port", PORT, NULL);
    g_object_set (G_OBJECT(h264parse), "config-interval", 1, NULL); 
    
    
    g_object_set (G_OBJECT (qtiqmmfsrc_0), "name", "qmmf_0", NULL);
    g_object_set (G_OBJECT (qtiqmmfsrc_0), "camera", cameraswitchctx.camera0, NULL);
    g_object_set (G_OBJECT (capsfilter), "name", "capsfilter", NULL);
   
    cameraswitchctx.pipeline = pipeline;
    cameraswitchctx.qtiqmmfsrc_0 = qtiqmmfsrc_0;
    cameraswitchctx.capsfilter = capsfilter;
    cameraswitchctx.cameraID = 0;
    cameraswitchctx.h264parse = h264parse;
    cameraswitchctx.qtic2venc = qtic2venc;
    cameraswitchctx.rtph264pay = rtph264pay;
    cameraswitchctx.sink = sink;


    gst_bin_add_many (GST_BIN (cameraswitchctx.pipeline), qtiqmmfsrc_0, capsfilter, qtic2venc, h264parse, rtph264pay, sink, NULL);
    gst_element_link_many (qtiqmmfsrc_0, capsfilter, qtic2venc, h264parse, rtph264pay, sink, NULL);
    
        
    loop = g_main_loop_new(NULL, FALSE);
    cameraswitchctx.mloop = loop;
    
    bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
    bus_id = gst_bus_add_watch(bus, bus_callback, loop);
    gst_object_unref(bus);
    
    //intrpt_watch_id = g_unix_signal_add (SIGINT, handle_interrupt_signal, &cameraswitchctx);
    
    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    
    pthread_t thread;
    pthread_create (&thread, NULL, &thread_fn, &cameraswitchctx);
  

    g_print("Start UDPSINK: addr: %s, port: %d\n", ADDR, PORT);
    g_main_loop_run(loop);
    g_print("Stop\n");
    gst_element_set_state(pipeline, GST_STATE_NULL);

    gst_object_unref(GST_OBJECT (pipeline));
    g_source_remove(bus_id);
    g_main_loop_unref(loop);

    gst_deinit();

    return 0;
}
