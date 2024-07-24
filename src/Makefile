CXX=g++ #CC=gcc
CXXFLAGS = -g -Wall
INCLAGS = -I./include #-I/home/origin/include

LDFLAGS = -L./aarch64 #-I/home/origin/aarch64
LDLIBS = -lirparse -lirprocess -lirtemp -liruvc -lircmd -liri2c -lcurl

GST_LIBS = $(shell pkg-config --cflags --libs gstreamer-1.0) #`pkg-config --cflags --libs gstreamer-1.0`
GSTAPP_LIBS = $(shell pkg-config --cflags --libs gstreamer-app-1.0) #`pkg-config --cflags --libs gstreamer-app-1.0`
OPENCV_LIBS = $(shell pkg-config --cflags --libs opencv4) #`pkg-config --cflags --libs opencv4`

main:
	$(CXX) $(CXXFLAGS) $(INCLAGS) -o main main.cpp $(LDFLAGS) $(LDLIBS) $(GST_LIBS) $(GSTAPP_LIBS) $(OPENCV_LIBS)

run:
	LD_LIBRARY_PATH="$PWD/aarch64:${LD_LIBRARY_PATH}"

clean:
	rm -f main
