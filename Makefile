# Copyright (c) 2020, 2022 Qualcomm Innovation Center, Inc.  All Rights Reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

FLAGS=`pkg-config --cflags --libs gstreamer-1.0`
FLAGS1=`pkg-config --cflags --libs gstreamer-app-1.0`

SOURCES = main.c

TARGETS = $(foreach n,$(SOURCES),$(basename $(n)))

all: ${TARGETS}

.PHONY: all clean

${TARGETS}: %:%.c
	$(CC) -Wall $< -o $@ $(FLAGS) $(FLAGS1)

clean:
	rm -f ${TARGETS}
