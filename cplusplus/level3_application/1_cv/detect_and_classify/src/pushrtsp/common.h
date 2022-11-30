#pragma once
#include <iostream>
#include <stdlib.h>
#include <sys/time.h>
#include <memory>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <opencv2/opencv.hpp>

extern"C" {
	#include <libavutil/mathematics.h>
	#include <libavutil/time.h>
	#include "libavcodec/avcodec.h"
	#include "libavformat/avformat.h"
	#include "libswscale/swscale.h"
	#include "libavutil/imgutils.h"
	#include "libavutil/opt.h"
};