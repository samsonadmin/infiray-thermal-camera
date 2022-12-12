

#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <linux/v4l2-controls.h>
#include <termio.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <stdbool.h>
#include <math.h>
#include <semaphore.h>
#include <pthread.h>

#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/core/core_c.h>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <fstream>
#include "thermometry.h"
#include "SimplePictureProcessing.h"
#include "Usb.h"
#include "pot.h"

using namespace std;
using namespace cv;

/**
 * OUTPUTMODE 4:配合libthermometry.so可以输出全局温度数据。
 *              配合simple.so使用专业级图像算法，可得到优秀画质的图像，但是需要主频1.8ghz。
 *              也可配合代码里的线性图像算法，画质稍低于高性能算法，但是对主频几乎没要求。
 *              输出数据格式：按照yuyv的形式，实际输出每个像素16bit中14bit为灰度数据，最后四行为参数。
 * OUTPUTMODE 4:Combine with libthermometry.so to output all around temperature data.
 *              Obtain high quality image with professional algorithms from simple.so, requires basic frequency at least 1.8ghz.
 *              Linear Algorithms produce lower quality image than professional algorithms but there are almost no requirements in basic frequency.
 *              Output data format: in yuyu form, the actual transferred is 16bit NUC data with 14 bits to store the wide dynamic grayscale value of one pixel.
 *              The last four lines of yuyu data are parameters.
 *   
 * 
 * OUTPUTMODE 5:配合libthermometry.so，可以直接输出中心点，最高温，最低温和额外指定的三个点的信息，不可输出全帧温度数据。              
 *              输出数据格式：输出yuyv格式的图像，最后四行为参数。
 *  OUTPUTMODE 5:With libthermometry.so to directly output temperature of the center, highest, lowest and extra three points. Can't output full frame temperature data.
 *               Output data format: graphs in yuyv format, the last four lines are parameters. 
 */
#define OUTPUTMODE 4
//#define OUTPUTMODE 5

#define TRUE 1
#define FALSE 0
#define MAX_BUFFER 2048

#define FILE_VIDEO1 "video"
#define FILE_DEV "/dev"
#define IRRGBA 0
#define IRRGB  1
//注意配置和设备相应的分辨率 Caution: set resolution that correspond to specification of the device
int IMAGEWIDTH = 384;
int IMAGEHEIGHT = 292;

pthread_t renderThreadId,handlerThreadId,bulkThreadPotId,bulkThreadFirmwareId;
static int fd=0;                          //设备描述符 Descriptor of device 
struct v4l2_streamparm stream_para;     //结构体v4l2_streamparm来描述视频流的属性 Struct v4l2_streamparm to describe the property of video stream
struct v4l2_capability cap;             //取得设备的capability，看看设备具有什么功能，比如是否具有视频输入,或者音频输入输出等 
                                        //Obtain capability of device, check if it has video input or audio input/output abilities.
struct v4l2_fmtdesc fmtdesc;            //枚举设备所支持的image format:  VIDIOC_ENUM_FMT    Enumerate image format the deivce supported: VIDIOC_ENUM_FMT
struct v4l2_format fmt,fmtack;          //子结构体struct v4l2_pix_format设置摄像头采集视频的宽高和类型：V4L2_PIX_FMT_YYUV V4L2_PIX_FMT_YUYV  
                                        //Substruct struct v4l2_pix_format for setting Height, Width and Type of captured video 
struct v4l2_requestbuffers req;         //向驱动申请帧缓冲的请求，里面包含申请的个数 Send request of frame buffer from driver, including amount of requests
struct v4l2_buffer buf;                 //代表驱动中的一帧 One frame in the driver 

struct buffer//从相机获得的数据缓存 Capture data buffer from Camera
{
    void * start;
    unsigned int length;
    long long int timestamp;
} *buffers;

struct irBuffer//使用专业级图像算法所需要的缓存   The buffer required by professional algorithm
{
    size_t** midVar;
    unsigned char* destBuffer;
} *irBuffers;

/**
 *temperatureData:最终输出的温度数据，采用10+全帧温度数据格式；例如10+384（宽）×288（高），前10个格式如下 
 *     The final output temperature data, in the form of "10 + Full Frame Temperature Data"; such as 10+384(width)×288(height), the top 10 as below
 *temperatureData[0]=centerTmp;
 *temperatureData[1]=(float)maxx1;
 *temperatureData[2]=(float)maxy1;
 *temperatureData[3]=maxTmp;
 *temperatureData[4]=(float)minx1;
 *temperatureData[5]=(float)miny1;
 *temperatureData[6]=minTmp;
 *temperatureData[7]=point1Tmp;
 *temperatureData[8]=point2Tmp;
 *temperatureData[9]=point3Tmp;
 *根据8004或者8005模式来查表，8005模式下仅输出以上注释的10个参数，8004模式下数据以上参数+全局温度数据
 *Search on the table with 8004 or 8005 mode, 8005 mode only outputs the 10 parameters above, 8004 mode include above parameters with overall temperature data
 *参见：thermometrySearch函数 Refer to function thermometrySearch
 */
float* temperatureData=NULL;

float* temperatureDataFixed=NULL;
float maxTmp=0;
float minTmp=0;

/**
 *设置三个单独点 Set three points
 *温度会出现在temperatureData[7]，temperatureData[8]，temperatureData[9] shows temperature
 *0<=viewX1<IMAGEWIDTH
 *0<=viewY1<IMAGEHEIGHT-4
 */
void setPoint(int viewX1,int viewY1,int indexOfPoint);
enum   v4l2_buf_type type;//帧类型  Type of buffer
struct v4l2_control ctrl;

/**
 *temperatureTable:温度映射表
 */
float temperatureTable[16384];

int init_v4l2(string videoX);           //初始化 Initialization
int v4l2_grab(void);                    //采集 Capture
int v4l2_control(int);                  //控制 Control
int traversalVideo(void);               //遍历video，如果是多个UVC设备可以在此处增加判断，是不是红外UVC  
                                        //Traveral video, may add determine statements if there are multiple UVC devices, weither infrared UVC

int delayy;
void sendCorrection(float correction);   //设置修正，一般取（-3.0)-3.0,用于整体温度向上或者向下修正  Set correction, usally -3.0/3.0 to correct temperature lower or higher

void sendReflection(float reflection);   //设置反射温度，一般情况下为环境温度  Set reflection temperature, usually ambient temperature
/*反射温度的确定：
当周围没有热源时，目标反射的是环境温度，所以一般情况下反射温度等于环境温度。
当周围有热源时，目标会反射热源的温度。如何确定反射温度：
1）取一张铝箔（红外反射镜），弄皱后再展平（朗伯面），将铝箔放在纸板上，亮面朝上。
2）调节热像仪发射率为1。
3）将铝箔放在目标表面前面并与之平行，用热像仪测量反射镜表面温度，此温度即反射温度。*/
/*How to find out reflection temperature:
When there is no heat source nearby, the object reflects ambient temperature, so usually refection equals ambient temperature.
When there is heat source, object reflects temperature of heat source. How to know the reflection temperature:
1)Take an aluminum foil as Mirror to reflect infrared ray, wrinkle the foil and then flat it. Put the foil on cardboard, the bright surface upwards  
2)Adjust the emissivity of device to 1;
3)Parallel foil with object, measure surface temperature of foil with thermal imager. The measured temperature is reflection temperature.
*/


void sendAmb(float amb);                        //设置环境温度   Set ambient temperature

void sendHumidity(float humidity);              //设置湿度（0-1.0），一般0.45   Set humidity (0-1.0), usually 0.45 

void sendEmissivity(float emiss);               //发射率（0-1.0），具体发射率见附表数据   Emissivity(0-1.0), refer to emissivity table

void sendDistance(unsigned short distance);              //距离（单位米）  Distance (Unit: Meter)   
void savePara();                                //保存参数到机芯，断电不丢失   Save parameter to the chipset
int v4l2_release();                             //释放v4l2  Release v4l2
int palette=0;
int displayMode=0;
int lowTmpPaletteIndex=0;
int computeMethod=2;
const unsigned char* paletteIronRainbow = getPalette(0);//256*3 铁虹   Iron Rainbow
const unsigned char* palette3 = getPalette(1);//256*3 彩虹1    Rainbow 1
const unsigned char* paletteRainbow = getPalette(2);//224*3 彩虹2    Rainbow 2  实际可用220
const unsigned char* paletteHighRainbow = getPalette(3);//448*3 高动态彩虹   HDR rainbow   实际可用444
const unsigned char* paletteHighContrast = getPalette(4);//448*3 高对比彩虹    High Contrast rainbow
const unsigned char* lavaRainbow = getPalette(5);//256*3 lava Rainbow
const unsigned char* paletteThIronRainbow = getPalette(6);//256*3 th Iron Rainbow 
const static unsigned char colormap_Iron888[256*3] =
{
    0x0,0x14,0x18,0x0,0x10,0x20,0x0,0xc,0x20,0x0,0x8,0x28,0x0,0x8,0x30,0x0,0x8,
    0x38,0x0,0x8,0x38,0x0,0x8,0x38,0x0,0x4,0x40,0x0,0x4,0x48,0x0,0x4,0x50,
    0x0,0x0,0x50,0x0,0x0,0x50,0x8,0x0,0x58,0x8,0x0,0x58,0x8,0x0,0x60,0x10,
    0x0,0x60,0x10,0x0,0x60,0x18,0x0,0x68,0x18,0x0,0x68,0x18,0x0,0x70,0x20,0x0,
    0x78,0x20,0x0,0x78,0x28,0x0,0x78,0x28,0x0,0x80,0x28,0x0,0x80,0x30,0x0,0x80,
    0x30,0x0,0x88,0x38,0x0,0x88,0x38,0x0,0x88,0x38,0x0,0x88,0x40,0x0,0x90,0x40,
    0x0,0x90,0x40,0x0,0x90,0x48,0x0,0x90,0x48,0x0,0x90,0x48,0x0,0x98,0x50,0x0,
    0x98,0x50,0x0,0x98,0x50,0x0,0x98,0x58,0x0,0x98,0x58,0x0,0x98,0x58,0x0,0x98,
    0x60,0x0,0x98,0x60,0x0,0xa0,0x68,0x0,0xa0,0x68,0x0,0xa0,0x68,0x0,0xa0,0x68,
    0x0,0xa0,0x70,0x0,0xa0,0x70,0x0,0xa0,0x70,0x0,0xa8,0x70,0x0,0xa8,0x78,0x0,
    0xa8,0x78,0x0,0xa8,0x78,0x0,0xa8,0x80,0x0,0xa0,0x80,0x0,0xa8,0x80,0x0,0xa8,
    0x88,0x0,0xa8,0x88,0x0,0xa0,0x88,0x0,0xa0,0x88,0x0,0xa0,0x88,0x0,0xa8,0x90,
    0x0,0xa0,0x90,0x0,0xa0,0x90,0x4,0xa0,0x98,0x4,0xa0,0x98,0x4,0xa0,0xa0,0x4,
    0xa0,0xa0,0x4,0xa0,0xa0,0x4,0xa0,0xa0,0x4,0xa0,0xa0,0x8,0xa0,0xa0,0x8,0x98,
    0xa8,0xc,0x98,0xa8,0xc,0x98,0xa8,0xc,0x98,0xa8,0x10,0x98,0xb0,0x10,0x98,0xb0,
    0x10,0x98,0xb0,0x10,0x98,0xb0,0x10,0x90,0xb0,0x10,0x90,0xb8,0x14,0x90,0xb8,0x14,
    0x88,0xb8,0x18,0x88,0xb8,0x18,0x88,0xb8,0x18,0x88,0xc0,0x1c,0x88,0xc0,0x1c,0x88,
    0xc0,0x1c,0x80,0xc0,0x1c,0x80,0xc8,0x20,0x80,0xc8,0x20,0x80,0xc8,0x20,0x80,0xc8,
    0x20,0x80,0xc8,0x20,0x78,0xc8,0x24,0x78,0xc8,0x28,0x78,0xd0,0x28,0x70,0xd0,0x28,
    0x70,0xd0,0x28,0x70,0xd0,0x2c,0x70,0xd8,0x2c,0x68,0xd8,0x2c,0x68,0xd8,0x30,0x68,
    0xd8,0x30,0x68,0xd8,0x30,0x60,0xd8,0x34,0x60,0xd8,0x34,0x60,0xd8,0x38,0x60,0xd8,
    0x38,0x58,0xe0,0x3c,0x58,0xe0,0x3c,0x58,0xe0,0x3c,0x58,0xe0,0x40,0x50,0xe0,0x40,
    0x50,0xe0,0x40,0x50,0xe0,0x44,0x48,0xe0,0x44,0x48,0xe8,0x48,0x48,0xe8,0x48,0x48,
    0xe8,0x4c,0x40,0xe8,0x4c,0x40,0xe8,0x50,0x40,0xe8,0x50,0x40,0xe8,0x50,0x38,0xf0,
    0x54,0x38,0xf0,0x54,0x38,0xf0,0x58,0x38,0xf0,0x58,0x38,0xf0,0x58,0x30,0xf0,0x5c,
    0x30,0xf0,0x5c,0x30,0xf0,0x60,0x28,0xf0,0x60,0x28,0xf0,0x64,0x28,0xf0,0x64,0x28,
    0xf8,0x64,0x20,0xf8,0x68,0x20,0xf8,0x68,0x20,0xf8,0x6c,0x20,0xf8,0x6c,0x20,0xf8,
    0x70,0x20,0xf8,0x70,0x18,0xf8,0x74,0x18,0xf8,0x74,0x18,0xf8,0x74,0x10,0xf8,0x78,
    0x10,0xf8,0x78,0x10,0xf8,0x7c,0x10,0xf8,0x7c,0x10,0xf8,0x80,0x10,0xf8,0x80,0x10,
    0xf8,0x80,0x8,0xf8,0x84,0x8,0xf8,0x88,0x8,0xf8,0x88,0x0,0xf8,0x88,0x0,0xf8,
    0x8c,0x0,0xf8,0x8c,0x0,0xf8,0x8c,0x0,0xf8,0x90,0x0,0xf8,0x90,0x0,0xf8,0x90,
    0x0,0xf8,0x94,0x0,0xf8,0x94,0x0,0xf8,0x98,0x0,0xf8,0x98,0x0,0xf8,0x9c,0x0,
    0xf8,0x9c,0x0,0xf8,0x9c,0x0,0xf8,0xa0,0x0,0xf8,0xa4,0x0,0xf8,0xa4,0x0,0xf8,
    0xa4,0x0,0xf8,0xa8,0x0,0xf8,0xa8,0x0,0xf8,0xa8,0x0,0xf8,0xac,0x0,0xf8,0xac,
    0x0,0xf8,0xb0,0x0,0xf8,0xb0,0x0,0xf8,0xb4,0x0,0xf8,0xb4,0x0,0xf8,0xb8,0x0,
    0xf8,0xb8,0x0,0xf8,0xb8,0x0,0xf8,0xbc,0x0,0xf8,0xbc,0x0,0xf8,0xbc,0x0,0xf8,
    0xc0,0x0,0xf8,0xc0,0x0,0xf8,0xc0,0x0,0xf8,0xc4,0x0,0xf8,0xc4,0x0,0xf8,0xc8,
    0x0,0xf8,0xc8,0x0,0xf8,0xc8,0x0,0xf8,0xcc,0x0,0xf8,0xcc,0x8,0xf8,0xcc,0x8,
    0xf8,0xd0,0x8,0xf8,0xd0,0x10,0xf8,0xd0,0x10,0xf8,0xd4,0x10,0xf8,0xd4,0x10,0xf8,
    0xd4,0x18,0xf8,0xd8,0x18,0xf8,0xd8,0x18,0xf8,0xdc,0x20,0xf8,0xdc,0x20,0xf8,0xdc,
    0x20,0xf8,0xe0,0x28,0xf8,0xe0,0x28,0xf8,0xe0,0x28,0xf8,0xe0,0x30,0xf8,0xe4,0x30,
    0xf8,0xe4,0x38,0xf8,0xe4,0x38,0xf8,0xe8,0x40,0xf8,0xe8,0x40,0xf8,0xe8,0x40,0xf8,
    0xec,0x48,0xf8,0xec,0x48,0xf8,0xec,0x50,0xf8,0xec,0x50,0xf8,0xec,0x58,0xf8,0xf0,
    0x58,0xf8,0xf0,0x60,0xf8,0xf0,0x60,0xf8,0xf0,0x68,0xf8,0xf4,0x70,0xf8,0xf4,0x78,
    0xf8,0xf4,0x78,0xf8,0xf4,0x80,0xf8,0xf4,0x88,0xf8,0xf8,0x88,0xf8,0xf8,0x90,0xf8,
    0xf8,0x98,0xf8,0xf8,0xa0,0xf8,0xfc,0xa0,0xf8,0xfc,0xa8,0xf8,0xfc,0xb0,0xf0,0xfc,
    0xb8,0xf0,0xfc,0xc0,0xf0,0xfc,0xc8,0xf0,0xfc,0xc8,0xf0,0xfc,0xd0,0xf0,0xfc,0xd8,
    0xf0,0xfc,0xe0,0xf0,0xfc,0xe8,0xf0,0xfc,0xf0,0xf0,0xfc,0xf0,0xf0,0xfc,0xf0
};//铁红
const static unsigned char colormap_RainBowHC888[256*3] =
{
    0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x8,0x0,0x8,0x8,0x0,0x8,0x8,0x0,
    0x10,0x10,0x0,0x10,0x18,0x0,0x18,0x20,0x0,0x18,0x20,0x0,0x20,0x28,0x0,0x20,
    0x28,0x0,0x28,0x30,0x0,0x30,0x30,0x0,0x30,0x38,0x0,0x38,0x40,0x0,0x40,0x48,
    0x0,0x48,0x48,0x0,0x48,0x50,0x0,0x50,0x58,0x0,0x50,0x58,0x0,0x58,0x60,0x0,
    0x58,0x68,0x0,0x60,0x68,0x0,0x68,0x68,0x0,0x68,0x70,0x0,0x70,0x78,0x0,0x78,
    0x80,0x0,0x80,0x88,0x0,0x88,0x88,0x0,0x88,0x90,0x0,0x90,0x98,0x0,0x98,0xa0,
    0x0,0x98,0xa0,0x0,0xa0,0xa8,0x0,0xa8,0xa8,0x0,0xa8,0xb0,0x0,0xb0,0xb0,0x0,
    0xb0,0xb8,0x0,0xb8,0xc0,0x0,0xc0,0xc8,0x0,0xc8,0xc8,0x0,0xc8,0xd0,0x0,0xd0,
    0xd0,0x0,0xd0,0xd8,0x0,0xd8,0xd8,0x0,0xd8,0xe0,0x0,0xe0,0xe0,0x0,0xe0,0xd8,
    0x0,0xe0,0xd8,0x0,0xe0,0xd8,0x0,0xe0,0xd0,0x0,0xe0,0xd0,0x0,0xe0,0xc8,0x0,
    0xd8,0xc8,0x0,0xd8,0xc0,0x0,0xd8,0xc0,0x0,0xd8,0xb8,0x0,0xd8,0xb0,0x0,0xd0,
    0xb0,0x0,0xd0,0xb0,0x0,0xd0,0xa8,0x0,0xd0,0xa8,0x0,0xd0,0xa0,0x0,0xc8,0x98,
    0x0,0xc8,0x98,0x0,0xc8,0x90,0x0,0xc8,0x90,0x0,0xc8,0x88,0x0,0xc8,0x88,0x0,
    0xc0,0x88,0x0,0xc0,0x80,0x0,0xc0,0x78,0x0,0xc0,0x78,0x0,0xc0,0x70,0x0,0xb8,
    0x68,0x0,0xb8,0x60,0x0,0xb8,0x60,0x0,0xb8,0x58,0x0,0xb8,0x58,0x0,0xb0,0x50,
    0x0,0xb0,0x48,0x0,0xb0,0x40,0x0,0xa8,0x38,0x0,0xa8,0x38,0x0,0xa8,0x30,0x0,
    0xa8,0x28,0x0,0xa0,0x20,0x0,0xa0,0x18,0x0,0xa0,0x8,0x0,0x98,0x0,0x0,0x98,
    0x0,0x4,0x98,0x0,0x8,0x98,0x0,0x18,0xa0,0x0,0x20,0xa0,0x0,0x30,0xa8,0x0,
    0x40,0xb0,0x0,0x4c,0xb0,0x0,0x5c,0xb8,0x0,0x60,0xb8,0x0,0x6c,0xb8,0x0,0x78,
    0xb8,0x0,0x88,0xc0,0x0,0x98,0xc8,0x0,0xa0,0xc8,0x0,0xb0,0xd0,0x0,0xb8,0xd0,
    0x0,0xc4,0xd8,0x0,0xc8,0xd8,0x0,0xd4,0xd8,0x0,0xe0,0xe0,0x0,0xdc,0xd8,0x0,
    0xd8,0xd0,0x0,0xcc,0xc0,0x0,0xc8,0xb8,0x0,0xc4,0xb0,0x0,0xc0,0xa8,0x0,0xbc,
    0xa8,0x0,0xb8,0xa0,0x0,0xb0,0x98,0x0,0xa8,0x88,0x0,0xa4,0x80,0x0,0xa0,0x80,
    0x0,0x9c,0x78,0x0,0x98,0x70,0x0,0x98,0x68,0x0,0x90,0x60,0x0,0x8c,0x58,0x0,
    0x80,0x48,0x0,0x80,0x48,0x0,0x7c,0x48,0x0,0x78,0x40,0x0,0x74,0x38,0x0,0x70,
    0x30,0x0,0x68,0x28,0x0,0x64,0x20,0x0,0x64,0x20,0x0,0x60,0x18,0x0,0x5c,0x10,
    0x0,0x58,0x10,0x0,0x50,0x0,0x0,0x4c,0x0,0x8,0x54,0x0,0x10,0x58,0x0,0x18,
    0x64,0x0,0x20,0x64,0x0,0x20,0x64,0x0,0x28,0x68,0x0,0x30,0x70,0x0,0x38,0x74,
    0x0,0x38,0x78,0x0,0x40,0x7c,0x0,0x50,0x80,0x0,0x58,0x88,0x0,0x60,0x90,0x0,
    0x68,0x90,0x0,0x68,0x94,0x0,0x78,0x9c,0x0,0x78,0xa0,0x0,0x80,0xa8,0x0,0x88,
    0xac,0x0,0x98,0xb8,0x0,0xa0,0xb8,0x0,0xa8,0xbc,0x0,0xb0,0xc0,0x0,0xb8,0xcc,
    0x0,0xc0,0xd0,0x0,0xc8,0xd4,0x0,0xd0,0xdc,0x0,0xd8,0xe0,0x0,0xe0,0xe0,0x0,
    0xe0,0xdc,0x0,0xd8,0xd0,0x0,0xd0,0xc4,0x0,0xc8,0xb4,0x0,0xc8,0xac,0x0,0xc0,
    0x9c,0x0,0xb8,0x94,0x0,0xb8,0x94,0x0,0xb8,0x88,0x0,0xb0,0x7c,0x0,0xb0,0x74,
    0x0,0xa8,0x68,0x0,0xa0,0x60,0x0,0xa0,0x58,0x0,0x98,0x4c,0x0,0x90,0x40,0x0,
    0x98,0x40,0x0,0x90,0x30,0x0,0x88,0x24,0x0,0x80,0x18,0x0,0x78,0x10,0x0,0x78,
    0x4,0x0,0x78,0x0,0x0,0x78,0x0,0x0,0x80,0x0,0x0,0x80,0x4,0x0,0x88,0x4,
    0x0,0x88,0x8,0x8,0x90,0xc,0x8,0x90,0xc,0x8,0x98,0x10,0x10,0x98,0x10,0x10,
    0xa0,0x14,0x10,0xa0,0x14,0x10,0xa8,0x14,0x18,0xa8,0x18,0x18,0xb0,0x1c,0x18,0xb0,
    0x1c,0x18,0xb8,0x20,0x20,0xb8,0x20,0x20,0xc0,0x20,0x20,0xc0,0x24,0x20,0xc0,0x24,
    0x28,0xc8,0x24,0x28,0xc8,0x28,0x28,0xd0,0x2c,0x28,0xd0,0x2c,0x28,0xd8,0x30,0x30,
    0xd8,0x34,0x30,0xd8,0x3c,0x38,0xd8,0x40,0x40,0xd8,0x48,0x48,0xd8,0x50,0x50,0xd8,
    0x58,0x58,0xe0,0x64,0x60,0xe0,0x64,0x68,0xe0,0x6c,0x68,0xe0,0x70,0x70,0xe0,0x78,
    0x78,0xe0,0x80,0x80,0xe8,0x88,0x88,0xe8,0x8c,0x90,0xe8,0x90,0x90,0xe8,0x98,0x98,
    0xe8,0x9c,0x98,0xe8,0xa4,0xa0,0xe8,0xac,0xa8,0xe8,0xb0,0xb0,0xf0,0xb8,0xb8,0xf0,
    0xb8,0xb8,0xf0,0xc0,0xb8,0xf0,0xc0,0xc0,0xf0,0xc8,0xc0,0xf0,0xcc,0xc8,0xf0,0xd0,
    0xc8,0xf0,0xd4,0xd0,0xf0,0xd8,0xd8,0xf0,0xd8,0xd8,0xf0,0xe0,0xe0,0xf8,0xe4,0xe0,
    0xf8,0xe8,0xe8,0xf8,0xe8,0xe8,0xf8,0xf0,0xf0,0xf8,0xf0,0xf0,0xf8,0xf4,0xf0
};//高对比彩虹


int keyBoardNum=0;
int KeyTestNew();
int isOn=1;
int counter=0;
int uvcState=1;
int nextTrans=0;
short* collectedData=NULL; 
unsigned short* tempBuffer=NULL; 
int rightData=1;
bool needRecal=0;
sem_t sem;
int KeyTestNew()
{
        struct termios oldt, newt;
        int c, oldf;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
        c = getchar();
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        fcntl(STDIN_FILENO, F_SETFL, oldf);
        if (EOF != c) {
            if (c >= 'a' && c <= 'z') {
                return (c - 'a' + 'A');
            } else {
                return c;
            }
        } else {
            return 0;
        }
}

//SetMethodTwoParameter(13,0,3,3,50,50);//目标突出
int methodTwoParameterA=30;
int methodTwoParameterB=1;
int methodTwoParameterC=1;
int methodTwoParameterD=1;
int methodTwoParameterBright=50;
int methodTwoParameterContra=50;
void on_Trackbar(int, void*)
{
    SetMethodTwoParameter(methodTwoParameterA,methodTwoParameterB,methodTwoParameterC,methodTwoParameterD,methodTwoParameterBright,methodTwoParameterContra);
}


float divTmp1=25;//divTmp1摄氏度以下黑白显示，可根据实际温度需求修改
float divTmp2=40;//divTmp1-divTmp2黄到红色渐变，divTmp2以上红色，可根据实际温度需求修改
int divTmp1x10=250;
int divTmp2x10=400;
int divTmpPercent=50;
int divTmpNuc1=6500;
int divTmpNuc2=9000;
int rangeMode=120;
void on_Trackbar_divTmp(int, void*)
{
    int i=0,divTmpNuc1State=1;
    divTmp1=divTmp1x10/10;
    divTmp2=divTmp2x10/10;
    for(i=0;i<16384;i++)
    {
        if(divTmp1<temperatureTable[i]&&divTmpNuc1State)
        {
            divTmpNuc1=i;
            divTmpNuc1State=0;
            printf("divTmpNuc1:%d\n",divTmpNuc1);
        }
        if(divTmp2<temperatureTable[i])
        {
            divTmpNuc2=i;
            printf("divTmpNuc2:%d\n",divTmpNuc2);
            break;
        }
    }
    SetDivTemp(divTmpNuc1,divTmpNuc2);    
}
void on_Trackbar_PaletteIndex (int, void*)
{
    
}

void* renderThread(void *arg);
void* bulkThreadPot(void *arg);
void* bulkThreadFirmware(void *arg);
void* handlerThread(void *arg)
{
    int palette05=0x8800;
    int temperatueParameter=0; 
    while(isOn)
    {
        sem_wait(&sem);
        printf("keyBoardNum:%d\n",keyBoardNum);
        if(keyBoardNum==10) { if(uvcState==1) v4l2_control(0x8000);}//enter:shutter
        if(keyBoardNum==48) { 
                                if(uvcState==1) v4l2_control(0xFF01);
                                uvcState=0;
                                pthread_attr_t attr;
                                pthread_attr_init(&attr);
                                pthread_attr_setdetachstate(&attr,  PTHREAD_CREATE_DETACHED);                          
                                int ret=pthread_create(&bulkThreadPotId,&attr,bulkThreadPot,NULL);                               
                                pthread_attr_destroy( &attr );                                
                             }//0 change to bulk Pot
        if(keyBoardNum==56) { 
                                if(uvcState==1) v4l2_control(0x80FA);
                                
                                uvcState=0;
                                sleep(5);
                                pthread_attr_t attr;
                                pthread_attr_init(&attr);
                                pthread_attr_setdetachstate(&attr,  PTHREAD_CREATE_DETACHED);                          
                                int ret=pthread_create(&bulkThreadFirmwareId,&attr,bulkThreadFirmware,NULL);                               
                                pthread_attr_destroy( &attr );                                
                             }//8 change to bulk Firmware
         if(keyBoardNum==57) { 
                             
                             }//9                                                               
        if(keyBoardNum==49) {
                                if(collectedData==NULL)
                                {   
                                    printf("collectedData address1:%X\n",collectedData);
                                    collectedData=(short*)malloc(IMAGEHEIGHT*IMAGEWIDTH*sizeof(unsigned short));
                                    printf("collectedData address2:%X\n",collectedData);
                                    memset(collectedData,0,IMAGEHEIGHT*IMAGEWIDTH*sizeof(short));                                
                                }
                                counter=1;
                            }//1:collect pot data
        if(keyBoardNum==50) { if(uvcState==1){rightData=0;v4l2_control(0x8035);usleep(200);savePara();usleep(100000);rightData=1;}printf("en pot\n"); }//2:en pot
        if(keyBoardNum==51) { if(uvcState==1){rightData=0;v4l2_control(0x8036);usleep(200);savePara();usleep(100000);rightData=1;}printf("invalible pot\n");}//3:invalible pot
        if(keyBoardNum==52) { if(uvcState==1){rightData=0;usleep(200);savePara();usleep(100000);rightData=1;} }//4:save para
        if(keyBoardNum==53) { if(uvcState==1){setPoint(0,1,0);} }//5 not suppot(0,0,x)
        if(keyBoardNum==54) { if(uvcState==1){setPoint(IMAGEWIDTH-2,0,1);} }//6 not suppot(0,0,x)
        if(keyBoardNum==55) { if(uvcState==1){setPoint(IMAGEWIDTH-2,IMAGEHEIGHT-6,2);} }//7 not suppot(0,0,x)
        if(keyBoardNum==81) { if(uvcState==1){palette=0;} }//q white hot
        if(keyBoardNum==87) { if(uvcState==1){palette=1;} }//w black hot
        if(keyBoardNum==69) { if(uvcState==1){palette=2;} }//e Iron Rainbow
        if(keyBoardNum==82) { if(uvcState==1){palette=3;} }//r Rainbow 1
        if(keyBoardNum==84) { if(uvcState==1){palette=4;} }//t Rainbow 2
        if(keyBoardNum==89) { if(uvcState==1){palette=5;} }//y HDR rainbow
        if(keyBoardNum==85) { if(uvcState==1){palette=6;} }//u High Contrast rainbow
        if(keyBoardNum==73) { if(uvcState==1){palette=7;} }//i lava Rainbow
        if(keyBoardNum==79) { if(uvcState==1){palette=8;} }//o th Iron Rainbow
        
        
        //if(keyBoardNum==73) { if(uvcState==1){SetUserPalette((unsigned char*)colormap_Iron888,100); palette=100;} }//i lava Rainbow
        //if(keyBoardNum==79) { if(uvcState==1){SetUserPalette((unsigned char*)colormap_RainBowHC888,100); palette=100;} }//o th Iron Rainbow
        
        
        if(keyBoardNum==80) { if(uvcState==1){if(displayMode==0) {displayMode=1;} else if(displayMode==1) {displayMode=2;}else if(displayMode==2) {displayMode=0;}} }//p  change displayMode
        if(keyBoardNum==65) {
                                uvcState=1;
                                pthread_attr_t attr;
                                pthread_attr_init(&attr);
                                pthread_attr_setdetachstate(&attr,  PTHREAD_CREATE_DETACHED);                          
                                int ret=pthread_create(&renderThreadId,&attr,renderThread,NULL);
                                pthread_attr_destroy( &attr );
                            }//a open uvc device
        if(keyBoardNum==83) {uvcState=0;}//s close uvc device
        if(keyBoardNum==68) { if(uvcState==1){if(computeMethod<4){computeMethod++;}else {computeMethod=0;} if(computeMethod==3||computeMethod==4) needRecal=1; } printf("computeMethod:%d\n",computeMethod);}//d change computeMethod 
        if(keyBoardNum==70) {if(uvcState==1);v4l2_control(0x8021); rangeMode=400;sleep(1); v4l2_control(0x8000); sleep(1);needRecal=1;sleep(10); v4l2_control(0x8000); sleep(1);needRecal=1;}//f change rangeMode 400
        if(keyBoardNum==71) {if(uvcState==1);v4l2_control(0x8020); rangeMode=120;sleep(1); v4l2_control(0x8000); sleep(1);needRecal=1;sleep(10); v4l2_control(0x8000); sleep(1);needRecal=1;}//g change rangeMode 120 
        if(keyBoardNum==72) {
                                if(uvcState==1&&temperatueParameter<6)
                                {
                                    switch(temperatueParameter)
                                    {
                                        case 0:
                                            sendCorrection(2.1f);
                                        break;
                                        case 1:
                                            sendAmb(26.0f);
                                        break;
                                        case 2:
                                            sendHumidity(0.47f);
                                        break;                                          
                                        case 3:
                                            sendEmissivity(1.0f);
                                        break; 
                                        default:
                                            sendDistance(3);
                                        break;                                                                               
                                    }
                                     sleep(1);
                                     needRecal=1;
                                     temperatueParameter++;
                                }
                                else
                                {
                                    temperatueParameter=0;
                                }
                            }//h send temperatue parameter 
        if(keyBoardNum==90) { if(uvcState==1&&OUTPUTMODE==5){if(palette05<0x8807){palette05++;}else {palette05=0x8800;} v4l2_control(palette05);} }//z 8005 change palette
    }

}
void* bulkThreadPot(void *arg)
{
    uint16_t VID = 0x1514;
    uint16_t PID = 0xFFFF;
    Usb *myUsb = new Usb(VID,PID);
    if(!myUsb->Init())
        cout<<"Init Erro"<<endl;
    myUsb->ListDev();
    bool openResult=0;
    do
    {
        myUsb->ListDev();
        openResult=myUsb->Open();
        sleep(1);
    }
    while(!openResult);

    if(!myUsb->Claim(0))
        cout<<"Claim Erro"<<endl;     

	sleep(2);
	typedef struct _USBCB  /* USB command block */
	{
	  unsigned short u32_Command;  /* command to execute */
	  unsigned short u32_Data;  /* generic data field */
	  unsigned int u32_Count;  /* number of bytes to transfer */
	} USBCB, *PUSBCB;

	unsigned short pBuffer=0;
	printf("collectedData address3:%X\n",collectedData);
	unsigned char* pData=(unsigned char*)collectedData;
	int IoStatus=0;
	USBCB usbcb;	
	usbcb.u32_Command =0x0001|(0x0b<<8) ;//USB_TEMP_TRANS;	// command
	usbcb.u32_Count = 0;		// number of bytes
	usbcb.u32_Data = 0x0;		// doesn't matter, device determines location
	printf("sizeof(USBCB):%d\n",sizeof(USBCB));
	IoStatus = myUsb->Write((unsigned char*)&usbcb,sizeof(USBCB),1000);
	if (IoStatus!=0)
	{
	    printf("send write pot err:%d\n",IoStatus);
	}
	sleep(1);
	IoStatus = myUsb->Read((unsigned char*)&pBuffer,2,1000);
	printf("read bulk write pot:%d\n",pBuffer);
	if (IoStatus!=0)
	{
	    printf("read bulk write pot err:%d\n",IoStatus);
	}
	sleep(1);

	for(int i=0; i*64<(IMAGEHEIGHT-4)*2* IMAGEWIDTH; i++)
	{
	    IoStatus = myUsb->Write(pData,64,2000);
	    // check for error
	    if (IoStatus!=0)
	    {
		printf("Write bulk err:%d\n",IoStatus);
	    }
	    usleep(1000);
	    IoStatus = myUsb->Read((unsigned char*)&pBuffer,2,1000);
	    usleep(50000);
	    //printf("read bulk buffer:%d,i:%d\n",pBuffer,i);
	    while(IoStatus!=0)
	    {
	        IoStatus = myUsb->Write(pData,64,2000);
	        usleep(100000);
	        IoStatus = myUsb->Read((unsigned char*)&pBuffer,2,1000);
	        //usleep(100000);
	        printf("re write read:%d,i:%d\n",pBuffer,i);
	    }
	    pData+=64;
	}

    unsigned char data[2];
    data[0]=0Xff;
    data[1]=0xff;
    myUsb->Write(data,2,1000);
    printf("change to uvc,input a to open uvc device\n");
    myUsb->Close();
    myUsb->Release();
    if(myUsb!=NULL) delete(myUsb);myUsb=NULL;
    if(collectedData!=NULL){ free(collectedData);collectedData=NULL; }
    pthread_exit(0);
}
void* bulkThreadFirmware(void *arg)
{

    FILE *File_input;
    unsigned long nPackLen = 2*1024*1024;//先分配2MB缓冲
    unsigned long lActLen = 0;
	unsigned char* pFileDatAll = new unsigned char[nPackLen];
	memset(pFileDatAll, 0, nPackLen);
	unsigned char* pFileDat = pFileDatAll;	 
	printf("dwLength000:%n\n");               
    File_input=fopen("./T3S-312-20190826.spi", "rb");
    if(File_input == NULL)
    {
       printf("errno fopen\n");
    }
    else 
    {
       printf("File Open successed!\n");
    }
    int ret=fseek(File_input,0,SEEK_SET);
    printf("ret:%d\n",ret);
    int curpos = ftell(File_input);
    printf("curpos:%d\n",curpos);
    fseek(File_input,0,SEEK_END);
    int dwLength = ftell(File_input);
    printf("dwLength2:%d\n",dwLength);
    ret=fseek(File_input,0,SEEK_SET);
	if(dwLength > nPackLen)
	{
	    printf("文件大小超过2MB，暂不处理!");
	    return 0;

	}
    fread(pFileDat,1,dwLength,File_input);          
                
    uint16_t VID = 0x1514;
    uint16_t PID = 0xFFFF;
    Usb *myUsb = new Usb(VID,PID);
    if(!myUsb->Init())
        cout<<"Init Erro"<<endl;
    myUsb->ListDev();
    bool openResult=0;
    do
    {
        myUsb->ListDev();
        openResult=myUsb->Open();
        sleep(1);
    }
    while(!openResult);

    if(!myUsb->Claim(0))
        cout<<"Claim Erro"<<endl;     

	sleep(2);
	typedef struct _USBCB  /* USB command block */
	{
	  unsigned short u32_Command;  /* command to execute */
	  unsigned short u32_Data;  /* generic data field */
	  unsigned int u32_Count;  /* number of bytes to transfer */
	} USBCB, *PUSBCB;

	unsigned short pBuffer=0;
	int IoStatus=0;
	USBCB usbcb;	
	usbcb.u32_Command =0x0001|(0x06<<8) ;//USB_TEMP_TRANS;	// command
	usbcb.u32_Count = 0;		// number of bytes
	usbcb.u32_Data = 0x0;		// doesn't matter, device determines location
	printf("sizeof(USBCB):%d\n",sizeof(USBCB));
	IoStatus = myUsb->Write((unsigned char*)&usbcb,sizeof(USBCB),1000);
	if (IoStatus!=0)
	{
	    printf("send write firmware err:%d\n",IoStatus);
	}
	else
	{
	    printf("Write firmware success\n");
	}
	sleep(2);
	for(int i=0; i*64<dwLength; i++)
	{
        if(i<50)
	    {
            printf("pFileDat%d:%x,%x,%x\n",i,pFileDat[0],pFileDat[1],pFileDat[2]);
        }
	    IoStatus = myUsb->Write(pFileDat,64,5000000);
	    // check for error
	    if (IoStatus!=0)
	    {
		    printf("Write firmware err:%d\n",IoStatus);
	    }
	    else
	    {   
	        printf("Write firmware success:%d\n",i);
	    }
	    usleep(5000);
	    IoStatus = myUsb->Read((unsigned char*)&pBuffer,2,5000000);
	    usleep(5000);
	    printf("read firmware buffer:%d,i:%d\n",pBuffer,i);
	    pFileDat+=64;
	}
	
	fclose(File_input);
	delete[] pFileDatAll;
	
    unsigned char data[2];
    data[0]=0Xff;
    data[1]=0xff;
    myUsb->Write(data,2,1000);
    printf("change to uvc,input a to open uvc device\n");
    myUsb->Close();
    myUsb->Release();
    if(myUsb!=NULL) delete(myUsb);myUsb=NULL;
    pthread_exit(0);
}


unsigned int n_buffers=0;  
void* renderThread(void *arg)//
{
    if(traversalVideo() == FALSE)       //打开摄像头
    {
        printf("Init fail~~\n");
        uvcState=0;
    }

    if(v4l2_grab() == FALSE)
    {
        printf("grab fail~~\n");
        uvcState=0;
    }
    delayy=0;
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;           //Stream 或者Buffer的类型。此处肯定为V4L2_BUF_TYPE_VIDEO_CAPTURE  Stream or Buffer Type, constantly as V4L2_BUF_TYPE_VIDEO_CAPTURE
    buf.memory = V4L2_MEMORY_MMAP;                    //Memory Mapping模式，则此处设置为：V4L2_MEMORY_MMAP   Memory Mapping mode，set as V4L2_MEMORY_MMAP

    if(OUTPUTMODE==4)
    {
        if(v4l2_control(0x8004) == FALSE)//控制机芯切换为8004 原始数据输出   Switch to 8004 mode, output raw data 
        {
            printf("control fail~~\n");
            //uvcState=0;
        }
        //初始化专业级图像算法 Initialize Professional Algorithm
        irBuffers = (irBuffer*)malloc(4 * sizeof(*irBuffers));
        if(!irBuffers)
        {
            printf("Out of memory\n");
            return 0;
        }
        SimplePictureProcessingInit(IMAGEWIDTH,(IMAGEHEIGHT-4));
        SetMethodTwoParameter(methodTwoParameterA,methodTwoParameterB,methodTwoParameterC,methodTwoParameterD,methodTwoParameterBright,methodTwoParameterContra);;//均衡
        for(n_buffers = 0; n_buffers < 4; n_buffers++)
        {

            irBuffers[n_buffers].midVar=(size_t**)calloc (7,sizeof(size_t*));
            SimplePictureProcessingInitMidVar(irBuffers[n_buffers].midVar);
        }
        //end -初始化高性能图像算法
        temperatureData=(float*)calloc(IMAGEWIDTH*(IMAGEHEIGHT-4)+10,sizeof(float));
        temperatureDataFixed=(float*)calloc(IMAGEWIDTH*(IMAGEHEIGHT-4)+10,sizeof(float));
    }
    else if(OUTPUTMODE==5)
    {
        if(v4l2_control(0x8005) == FALSE)//控制机芯切换为8005 yuyv输出    Switch to 8005 mode, output yuyv
        {
            printf("control fail~~\n");
            //uvcState=0;
        }
        temperatureData=(float*)calloc(11,sizeof(float));
        temperatureDataFixed=(float*)calloc(11,sizeof(float));       
    }

    int i = 100;
    double t;
    long long int extra_time = 0;
    long long int cur_time = 0;
    long long int last_time = 0;  
    // 生成的rgb图像，cv使用bgr格式，所以除了黑热和白热，其他伪彩存在通道偏差，需要再转换一下。
    // formed rgba image, CV with bgr format. So there is channel deviation in pseudo color, need to convert,except Black Hot and White Hot.  
    cv::Mat bgrImg(IMAGEHEIGHT-4, IMAGEWIDTH,CV_8UC4);
    cv::Mat rgbImgBuffer[4];
    for(n_buffers = 0; n_buffers < 4; n_buffers++)
    {
        rgbImgBuffer[n_buffers].create(IMAGEHEIGHT-4, IMAGEWIDTH,CV_8UC4);
    }
    // 灰度图像。luminous image.
    cv::Mat lumiImg(IMAGEHEIGHT-4, IMAGEWIDTH,CV_8UC1);
    //8005模式输出的yuyv图像。 8005 mode output yuyv image.
    cv::Mat yuvImg; 
    yuvImg.create(IMAGEHEIGHT-4, IMAGEWIDTH, CV_8UC2);
    //测温相关参数，详见thermometry.h    Refer to thermometry.h for relevant parameters
    
    int haveChangedrangeMode=0;
    float floatFpaTmp;
    float correction;
    float Refltmp;
    float ambient;
    float humi;
    float emiss;
    unsigned short distance;
    int cameraLens=130;
    float shutterFix=0;
    //end -测温相关参数

    char sn[32];//camera序列码   camera serial number
    char cameraSoftVersion[16];//camera软件版本   camera Software Version
    unsigned short shutTemper;
    float floatShutTemper;//快门温度    Shutter Temperature
    unsigned short coreTemper;
    float floatCoreTemper;//外壳温度   Shell temperature
    printf("renderThread 0\n");
    while(uvcState)
    {
        for(n_buffers = 0; n_buffers < 4; n_buffers++)
        {
            t = (double)cvGetTickCount();
            //出队  out of queue
            buf.index = n_buffers;
            ioctl(fd, VIDIOC_DQBUF, &buf);
            delayy++;
            //printf ("delayy:%d\n", delayy);
            //printf("buf bytesused:%d",buf.bytesused);
            if(buf.bytesused!=IMAGEHEIGHT*IMAGEWIDTH*2||!rightData)//数据不完整直接入队  incomplete data ,enqueueing cycle
            {
                ioctl(fd, VIDIOC_QBUF, &buf);
                printf("throw data\n");
            }
            if(rightData)
            {
                //查看采集数据的时间戳之差，单位为微妙   Check the timestamp of data capturing, unit: microsecond 
                buffers[n_buffers].timestamp = buf.timestamp.tv_sec*1000000+buf.timestamp.tv_usec;
                cur_time = buffers[n_buffers].timestamp;
                extra_time = cur_time - last_time;
                last_time = cur_time;
                //printf("time_deta:%lld\n\n",extra_time);
                //printf("buf_len:%d\n",buffers[n_buffers].length);

                unsigned short* orgData=(unsigned short *)buffers[n_buffers].start;
                unsigned short* fourLinePara=orgData+IMAGEWIDTH*(IMAGEHEIGHT-4);//后四行参数  Parameters in last four lines
                if(counter!=0&&OUTPUTMODE==4)
                {
                    //printf("collectedData address4:%X\n",collectedData);
                    collectData(&counter,orgData,collectedData,IMAGEHEIGHT-4,IMAGEWIDTH,&tempBuffer);
                    //printf("collectData counter:%d,tempBuffer:%X\n",counter,tempBuffer);
                }

                int amountPixels=0;
                switch (IMAGEWIDTH)
                {
                    case 384:
                        amountPixels=IMAGEWIDTH*(4-1);
                        break;
                    case 240:
                        amountPixels=IMAGEWIDTH*(4-3);
                        break;
                    case 256:
                        amountPixels=IMAGEWIDTH*(4-3);
                        break;
                    case 640:
                        amountPixels=IMAGEWIDTH*(4-1);
                        break;
                }
                memcpy(&shutTemper,fourLinePara+amountPixels+1,sizeof(unsigned short));
                //printf("cpyPara  shutTemper:%d ",shutTemper);
                floatShutTemper=shutTemper/10.0f-273.15f;//快门温度  Shutter Temperature
                memcpy(&coreTemper,fourLinePara+amountPixels+2,sizeof(unsigned short));//外壳  Shell temperature
                //printf("cpyPara  coreTemper:%d ",coreTemper);
                floatCoreTemper=coreTemper/10.0f-273.15f;
                //printf("cpyPara  floatShutTemper:%f,floatCoreTemper:%f,floatFpaTmp:%f\n",floatShutTemper,floatCoreTemper,floatFpaTmp);
                memcpy((unsigned short*)cameraSoftVersion,fourLinePara+amountPixels+24,16*sizeof(uint8_t));//camera soft version
                //printf("cameraSoftVersion:%s\n",cameraSoftVersion);
                memcpy((unsigned short*)sn,fourLinePara+amountPixels+32,32*sizeof(uint8_t));//SN
                //printf("sn:%s\n",sn);
                int userArea=amountPixels+127;
                memcpy(&correction,fourLinePara+userArea,sizeof( float));//修正  Correction
                userArea=userArea+2;
                memcpy(&Refltmp,fourLinePara+userArea,sizeof( float));//反射温度   Reflection temperature 
                userArea=userArea+2;
                memcpy(&ambient,fourLinePara+userArea,sizeof( float));//环境温度    Ambient temperature
                userArea=userArea+2;
                memcpy(&humi,fourLinePara+userArea,sizeof( float));//湿度   Humidity
                userArea=userArea+2;
                memcpy(&emiss,fourLinePara+userArea,sizeof( float));//发射率   Emissivity
                userArea=userArea+2; 
                memcpy(&distance,fourLinePara+userArea,sizeof(unsigned short));//距离   Distance
                //printf ("ambient:%f,correction:%f,distance:%d,emiss:%f,Refltmp:%f,humi:%f\n", ambient,correction,distance,emiss,Refltmp,humi);

                //printf("delayy:%d\n",delayy);
                if(delayy%4500==30)//每三分钟打一次快门，人体高精度版本每分钟打一次   Shutter calibrates once every three minutes, for body temperature measuring, once per minute
                {
                    if(v4l2_control(0x8000) == FALSE)
                    {
                        printf("shutter fail~~\n");
                    }
                }
                if(delayy%1500==55||needRecal)//打快门前5帧重新计算table，打快门时不可计算表，数据可能有错误   Recalculate the table during the five frames before shutter calibrates, otherwise may be errors 
                {
                    //用后四行参数来计算表   Calculate table with last four lines
                    thermometryT4Line(IMAGEWIDTH,IMAGEHEIGHT,temperatureTable,fourLinePara,&floatFpaTmp,&correction,&Refltmp,&ambient,&humi,&emiss,&distance,cameraLens,shutterFix,rangeMode);
                    needRecal=0;
                    if(computeMethod==3||computeMethod==4)
                    {
                        int i=0,divTmpNuc1State=1;
                        for(i=0;i<16384;i++)
                        {
                            if(divTmp1<temperatureTable[i]&&divTmpNuc1State)
                            {
                                divTmpNuc1=i;
                                divTmpNuc1State=0;
                                printf("divTmpNuc1:%d\n",divTmpNuc1);
                            }
                            if(divTmp2<temperatureTable[i])
                            {
                                divTmpNuc2=i;
                                printf("divTmpNuc2:%d\n",divTmpNuc2);
                                break;
                            }
                        }
                        SetDivTemp(divTmpNuc1,divTmpNuc2);
                    }
                    if(delayy>9000)
                    {
                        delayy=0;
                    }
                }


                //if(delayy%25==10)//一般一秒钟查一次表确定对应点的温度，首次查表要计算完table
                //Usually search temperature of points with table every second. Calculate the whole table when first time searching.
                // {

                //根据8004或者8005模式来查表，8005模式下仅输出11个参数(参考thermometry.h)，8004模式下数据以上参数+全局温度数据
                //Search on the table with 8004 or 8005 mode, 8005 mode only outputs the 11 parameters(refer to thermometry.h) , 8004 mode include above parameters with overall temperature data
                if(OUTPUTMODE==4)
                {
                    thermometrySearch(IMAGEWIDTH,IMAGEHEIGHT,temperatureTable,orgData,temperatureData,rangeMode,OUTPUTMODE);
                    //printf("centerTmp:%.2f,maxTmp:%.2f,minTmp:%.2f,avgTmp:%.2f,point1Tmp:%.2f\n",temperatureData[0],temperatureData[3],temperatureData[6],temperatureData[9],temperatureData[7]);
                    /*char str[51];
                    memset(str,0,51);
                    sprintf(str,",%d,%d,%d,%.2f,%.2f,%.2f,%d,%d,%d,\n",searchTime,shutterTime,tableTime,floatShutTemper,floatFpaTmp,temperatureData[0],detectAvg,avg,centerNuc);
                    fstream File_output1;
                    File_output1.open("./temperatureDataVideo.dat", ios::out|ios::app|ios::binary);
                    File_output1.write((const char*)str,sizeof(str));
                    File_output1.close();*/
                }
                else if(OUTPUTMODE==5)
                {
                    thermometrySearchCMM(IMAGEWIDTH,IMAGEHEIGHT,temperatureTable,fourLinePara,temperatureData,rangeMode);
                    //printf("centerTmp:%.2f,maxTmp:%.2f,minTmp:%.2f,avgTmp:%.2f,point1Tmp:%.2f,point2Tmp:%.2f,point3Tmp:%.2f\n",temperatureData[0],temperatureData[3],temperatureData[6],temperatureData[10],temperatureData[7],temperatureData[8],temperatureData[9]);
                } 
                //指定要查询的个数及数据位置，仅可输入8004模式下的数据   Set the count of query and data address, only for data in 8004 mode
                /*int count=20;
                unsigned short* queryData=orgData+IMAGEWIDTH*(IMAGEHEIGHT-4)/2;
                float* temperatureData3=(float*)calloc(count,sizeof(float));
                thermometrySearchSingle(IMAGEWIDTH,IMAGEHEIGHT,temperatureTable,fourLinePara,count,queryData,temperatureData3,rangeMode);
                printf("temperatureData3[0]:%.2f,temperatureData3[1]:%.2f,temperatureData3[2]:%.2f\n",temperatureData3[0],temperatureData3[1],temperatureData3[2]);
                free(temperatureData3);*/

                //}                                         
                //thermFix (temperatureData,10, temperatureDataFixed,ambient,Refltmp,0.80f,0.99f);
                //printf("centerTmpFixed:%.2f,maxTmpFixed:%.2f,minTmpFixed:%.2f\n",temperatureDataFixed[0],temperatureDataFixed[3],temperatureDataFixed[6]);
                amountPixels=IMAGEWIDTH*(IMAGEHEIGHT-4);
                unsigned short detectAvg=orgData[amountPixels];
                //printf("cpyPara  detectAvg:%d ",detectAvg);
                amountPixels++;
                unsigned short fpaTmp=orgData[amountPixels];
                amountPixels++;
                unsigned short maxx1=orgData[amountPixels];
                amountPixels++;
                unsigned short maxy1=orgData[amountPixels];
                amountPixels++;
                unsigned short max=orgData[amountPixels];
                //printf("cpyPara  max:%d ",max);
                amountPixels++;
                unsigned short minx1=orgData[amountPixels];
                amountPixels++;
                unsigned short miny1=orgData[amountPixels];
                amountPixels++;
                unsigned short min=orgData[amountPixels];
                amountPixels++;
                unsigned short avg=orgData[amountPixels];

                //以下用于8004模式中计算出rgba来显示，opencv使用bgra来显示，故而通道有差异。
                //Following codes for computing rgba outputs in 8004 mode; using OpenCV to show bgra, so there are differences in channel
                if(OUTPUTMODE==4)
                {
                    switch (computeMethod)
                    {
                        case 1:
                            Compute(orgData,rgbImgBuffer[n_buffers].data,palette,irBuffers[n_buffers].midVar,IRRGBA);//0-6 to change platte
                        break;                        
                        case 2:
                            ComputeMethodTwo(orgData,rgbImgBuffer[n_buffers].data,palette,irBuffers[n_buffers].midVar,IRRGBA);
                        break;                   
                        case 3:
                            ComputeDivTemp(orgData,rgbImgBuffer[n_buffers].data,0,irBuffers[n_buffers].midVar,IRRGBA);
                        break;
                        case 4:
                            ComputeDivTempType2(orgData,rgbImgBuffer[n_buffers].data,palette,lowTmpPaletteIndex,irBuffers[n_buffers].midVar,displayMode,IRRGBA);
                        break;
                        case 0:
                             /**
                             * 线性图像算法 linear algorithm
	                         * 图像效果不及专业级算法，但是处理效率快，对主频几乎没要求 
                             * Poor images than professional algorithm, but runs faster and no requirement in basic frequency
                             *
                             */
                            unsigned char* orgOutput = rgbImgBuffer[n_buffers].data;
                            int ro = (max - min)>0?(max - min):1;
                            int avgSubMin=(avg-min)>0?(avg-min):1;
                            int maxSubAvg=(max-avg)>0?(max-avg):1;
                            int ro1=(avg-min)>97?97:(avg-min);
                            int ro2=(max-avg)>157?157:(max-avg);
                            if(palette==2)
                            {
                                //int ga=(max - min)>254?254:(max - min);
                                for(int i=0; i<IMAGEHEIGHT-4; i++)
                                {
                                    for(int j=0; j<IMAGEWIDTH; j++)
                                    {
                                        //printf("i:%d,j:%d\n",i,j);
                                        //黑白：灰度值0-254单通道。 paletteIronRainbow：（0-254）×3三通道。两个都是255，所以使用254
                                        //Black&WHite: Grayscale (0-254), single channel.  paletteIronRainbow (0-254), triple channels. Both are 255, so take 254
                                        int gray=0;
                                        if(orgData[i*IMAGEWIDTH+j]>avg)
                                        {
                                            gray = (int)(ro2*(orgData[i*IMAGEWIDTH+j]-avg)/maxSubAvg+97);
                                        }
                                        else
                                        {
                                            gray = (int)(ro1*(orgData[i*IMAGEWIDTH+j]-avg)/avgSubMin+97);
                                        }
                                        lumiImg.at<uchar>(i,j) = (uchar)gray;
                                        int intGray=(int)gray;
                                        int paletteNum=3*intGray;
                                        orgOutput[4*(i*IMAGEWIDTH+j)]=(unsigned char)paletteIronRainbow[paletteNum+2];
                                        orgOutput[4*(i*IMAGEWIDTH+j)+1]=(unsigned char)paletteIronRainbow[paletteNum+1];
                                        orgOutput[4*(i*IMAGEWIDTH+j)+2]=(unsigned char)paletteIronRainbow[paletteNum];
                                        orgOutput[4*(i*IMAGEWIDTH+j)+3]=1;
                                    }
                                }
                                cv::imshow("lumiImg", lumiImg);
                            }
                            else if(palette==6)
                            {
                                orgOutput = rgbImgBuffer[n_buffers].data;
                                ro = (max - min)>0?(max - min):1;
                                avgSubMin=(avg-min)>0?(avg-min):1;
                                maxSubAvg=(max-avg)>0?(max-avg):1;
                                ro1=(avg-min)>170?170:(avg-min);
                                ro2=(max-avg)>276?276:(max-avg);
                                for(int i=0; i<IMAGEHEIGHT-4; i++)
                                {
                                    for(int j=0; j<IMAGEWIDTH; j++)
                                    {
                                        //printf("i:%d,j:%d\n",i,j);
                                        // paletteHighContrast（0-448）×3三通道，所以使用447  triple channels, so take 447
                                        int gray=0;
                                        if(orgData[i*IMAGEWIDTH+j]>avg)
                                        {
                                            gray = (int)(ro2*(orgData[i*IMAGEWIDTH+j]-avg)/maxSubAvg+170);
                                        }
                                        else
                                        {
                                            gray = (int)(ro1*(orgData[i*IMAGEWIDTH+j]-avg)/avgSubMin+170);
                                        }
                                        int intGray=(int)gray;
                                        int paletteNum=3*intGray;
                                        orgOutput[4*(i*IMAGEWIDTH+j)]=(unsigned char)paletteHighContrast[paletteNum+2];
                                        orgOutput[4*(i*IMAGEWIDTH+j)+1]=(unsigned char)paletteHighContrast[paletteNum+1];
                                        orgOutput[4*(i*IMAGEWIDTH+j)+2]=(unsigned char)paletteHighContrast[paletteNum];
                                        orgOutput[4*(i*IMAGEWIDTH+j)+3]=1;
                                    }
                                }
                            }
                            else
                            {
                                //int ga=(max - min)>254?254:(max - min);
                                for(int i=0; i<IMAGEHEIGHT-4; i++)
                                {
                                    for(int j=0; j<IMAGEWIDTH; j++)
                                    {
                                        //printf("i:%d,j:%d\n",i,j);
                                        //黑白：灰度值0-254单通道。 paletteIronRainbow：（0-254）×3三通道。两个都是255，所以使用254
                                        //Black&WHite: Grayscale (0-254), single channel.  paletteIronRainbow (0-254), triple channels. Both are 255, so take 254
                                        int gray=0;
                                        if(orgData[i*IMAGEWIDTH+j]>avg)
                                        {
                                            gray = (int)(ro2*(orgData[i*IMAGEWIDTH+j]-avg)/maxSubAvg+97);
                                        }
                                        else
                                        {
                                            gray = (int)(ro1*(orgData[i*IMAGEWIDTH+j]-avg)/avgSubMin+97);
                                        }
                                        lumiImg.at<uchar>(i,j) = (uchar)gray;
                                        int intGray=(int)gray;
                                        int paletteNum=3*intGray;
                                        orgOutput[4*(i*IMAGEWIDTH+j)]=gray;
                                        orgOutput[4*(i*IMAGEWIDTH+j)+1]=gray;
                                        orgOutput[4*(i*IMAGEWIDTH+j)+2]=gray;
                                        orgOutput[4*(i*IMAGEWIDTH+j)+3]=1;
                                    }
                                }
                            }
                            //end-线性算法
                        break;                                          
                    } 
                    cv::cvtColor(rgbImgBuffer[n_buffers], bgrImg,  cv::COLOR_RGB2BGR);                  
                }
                //以下用于8005模式中yuyv输出转bgr，使用的是opencv
                //Following codes for converting yuyv outputs into bgr in 8005 mode, using OpenCV
                else if(OUTPUTMODE==5)
                {
                    memcpy(yuvImg.data,(unsigned char*)orgData,(IMAGEHEIGHT-4)*2* IMAGEWIDTH*sizeof(unsigned char));
                    cv::cvtColor(yuvImg, bgrImg,  cv::COLOR_YUV2BGR_YUYV);
                }

                cv::imshow("methodTwo", bgrImg);
                //cv::imshow("methodTwo", rgbImgBuffer[n_buffers]);

                 //在 driver 内部管理着两个 buffer queues ，一个输入队列，一个输出队列。
                //对于 capture device 来说，当输入队列中的 buffer
                //被塞满数据以后会自动变为输出队列，
                //There are two buffer queues inside driver: input queues and output queues
                //For capture device, when buffer in input queue is fulled by data, it will transfer to output queue
                //重新调用 VIDIOC_QBUF 将 buffer 重新放进输入队列.  Calling VIDIOC_DQBUF again, place buffer into output queue               
                ioctl(fd,VIDIOC_QBUF,&buf);                      
                if((cv::waitKey(1)&255) == 27)    exit(0);
                t=(double)cvGetTickCount()-t;
                //printf("used time is %gms\n",(t/(cvGetTickFrequency()*1000)));

            }   
        }
    }

    printf("renderThread 1\n");
    for(n_buffers = 0; n_buffers < 4; n_buffers++)
    {
        //释放专业图像算法占用的资源   Release buffers captured by professional image algorithm
        if(OUTPUTMODE==4)
        {
            SimplePictureProcessingDeinit();
            if(irBuffers[n_buffers].midVar!=NULL)
            {
                SimplePictureProcessingDeinitMidVar(irBuffers[n_buffers].midVar);
                free(irBuffers[n_buffers].midVar);
                irBuffers[n_buffers].midVar=NULL;
            }
        }
        //end -释放专业图像算法占用的资源   Release buffers

        if(temperatureData!=NULL)
        {
            free(temperatureData);
            temperatureData=NULL;
        }
        if(temperatureDataFixed!=NULL)
        {
            free(temperatureDataFixed);
            temperatureDataFixed=NULL;
        }
    }
    printf("fd :%d\n",fd);
    if(fd!=0)
    {
        v4l2_release();         // 停止视频采集命令，应用程序调用VIDIOC_ STREAMOFF停止视频采集命令后，视频设备驱动程序不在采集视频数据。
                            //Calling VIDIOC_ STREAMOFF to stop capturing video, video device driver stops capturing video
    }
    printf("pthread_exit\n");
    pthread_exit(0);

}
int main()
{

    namedWindow("methodTwo",WINDOW_AUTOSIZE);
    namedWindow("lumiImg", 0);
    //namedWindow("methodTwo",0);
    createTrackbar("a","methodTwo",&methodTwoParameterA,500,on_Trackbar);
    createTrackbar("b","methodTwo",&methodTwoParameterB,50,on_Trackbar);
    createTrackbar("c","methodTwo",&methodTwoParameterC,500,on_Trackbar);
    createTrackbar("d","methodTwo",&methodTwoParameterD,500,on_Trackbar);
    createTrackbar("bright","methodTwo",&methodTwoParameterBright,100,on_Trackbar);
    createTrackbar("contra","methodTwo",&methodTwoParameterContra,100,on_Trackbar);
    createTrackbar("lowTmpPaletteIndex","methodTwo",&lowTmpPaletteIndex,220,on_Trackbar_PaletteIndex);
    createTrackbar("divTmp1x10","methodTwo",&divTmp1x10,1000,on_Trackbar_divTmp);
    createTrackbar("divTmp2x10","methodTwo",&divTmp2x10,1200,on_Trackbar_divTmp);
    on_Trackbar_PaletteIndex(0,0);
    on_Trackbar(0, 0);
    on_Trackbar_divTmp(0,0); 

    
    
    int ret=0;
    
    ret=pthread_create(&handlerThreadId,NULL,handlerThread,NULL);
    ret = sem_init(&sem, 0, 0);
    
    while(isOn)
    {
        int nNub = KeyTestNew();
        switch (nNub)
        {
            case 10://enter:shutter
            case 48://0 change to bulk pot
            case 49://1:collect pot data
            case 50://2:en pot
            case 51://3:invalible pot
            case 52://4:save para
            case 53://5 not suppot(0,0,x)
            case 54://6 not suppot(0,0,x)
            case 55://7 not suppot(0,0,x)
            case 56://8 bulk firm ware
            case 57://9 nextTrans
            case 81://q white hot
            case 87://w black hot
            case 69://e Iron Rainbow
            case 82://r Rainbow 1
            case 84://t Rainbow 2
            case 89://y HDR rainbow
            case 85://u High Contrast rainbow
            case 73://i lava Rainbow
            case 79://o th Iron Rainbow
            case 80://p change displayMode
            case 65://a open uvc device
            case 83://s close uvc device
            case 68://d change computeMethod
            case 70://f change rangemode 400
            case 71://g change rangemode 120
            case 72://h send temperatue parameter
            case 90://z 8005 change pallets
                keyBoardNum=nNub;
                sem_post(&sem);
            break;
            
        }
            

    }
    pthread_join(handlerThreadId,NULL);
    sem_destroy(&sem);
    return 0;
}
int v4l2_release()
{
    unsigned int n_buffers;
    enum v4l2_buf_type type;

    //关闭流  Video stream OFF
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(fd, VIDIOC_STREAMOFF, &type);

    //关闭内存映射   Memory map OFF
    for(n_buffers=0; n_buffers<4; n_buffers++)
    {
        munmap(buffers[n_buffers].start,buffers[n_buffers].length);
    }

    //释放自己申请的内存   Free buffers
    free(buffers);

    //关闭设备   Device OFF
    close(fd);
    return TRUE;
}


int init_v4l2(string videoX)
{
    const char* videoXConst=videoX.c_str();
    if ((fd = open(videoXConst, O_RDWR)) == -1)                //打开video1  Open Video1
    {
        printf("Opening video device error\n");
        return FALSE;
    }
    printf("init_v4l2 fd :%d\n",fd);
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == -1)                // 查询视频设备的功能   Query capabilities of video device
    {
        printf("unable Querying Capabilities\n");
        return FALSE;
    }
    else

    {
        printf( "Driver Caps:\n"
                "  Driver: \"%s\"\n"
                "  Card: \"%s\"\n"
                "  Bus: \"%s\"\n"
                "  Version: %d\n"
                "  Capabilities: %x\n",
                cap.driver,
                cap.card,
                cap.bus_info,
                cap.version,
                cap.capabilities);
        string str="";
        str=(char*)cap.card;
        /*if(!str.find("T3S")){
        	close(fd);
        	return FALSE;
        }*/
    }
    /* if((cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) == V4L2_CAP_VIDEO_CAPTURE){//是否支持V4L2_CAP_VIDEO_CAPTURE
         printf("Camera device %s: support capture\n",FILE_VIDEO1);
     }
     if((cap.capabilities & V4L2_CAP_STREAMING) == V4L2_CAP_STREAMING){//是否支持V4L2_CAP_STREAMING
         printf("Camera device %s: support streaming.\n",FILE_VIDEO1);
     }
    */
    fmtdesc.index = 0;
    fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    printf("Support format: \n");
    while(ioctl(fd,VIDIOC_ENUM_FMT,&fmtdesc) != -1)         // 获取当前视频设备支持的视频格式   Capture current video format of the device 
    {
        printf("\t%d. %s\n",fmtdesc.index+1,fmtdesc.description);
        fmtdesc.index++;
    }
    //set fmt
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = IMAGEWIDTH;
    fmt.fmt.pix.height = IMAGEHEIGHT;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;//使用V4L2_PIX_FMT_YUYV  Calling V4L2_PIX_FMT_YUYV
    //fmt.fmt.pix.field = V4L2_FIELD_NONE;
    fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;
    if (ioctl(fd, VIDIOC_S_FMT, &fmt) == -1)     // 设置视频设备的视频数据格式，例如设置视频图像数据的长、宽，图像格式（JPEG、YUYV格式）
    {                                           //Set video data format, such as Width, Height of image, format (JPEG, YUYV etc)
        printf("Setting Pixel Format error\n");
        return FALSE;
    }
    if(ioctl(fd,VIDIOC_G_FMT,&fmt) == -1)    //获取图像格式   Get video format
    {
        printf("Unable to get format\n");
        return FALSE;
    }
    IMAGEWIDTH = fmt.fmt.pix.width;//更正宽  fix Width
    IMAGEHEIGHT = fmt.fmt.pix.height;//更正高   fix Height
    printf("IMAGEWIDTH:%d,IMAGEHEIGHT:%d\n",IMAGEWIDTH,IMAGEHEIGHT);
    memset(&stream_para, 0, sizeof(struct v4l2_streamparm));
    stream_para.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    stream_para.parm.capture.timeperframe.denominator = 25;
    stream_para.parm.capture.timeperframe.numerator = 1;

    if(ioctl(fd, VIDIOC_S_PARM, &stream_para) == -1)
    {
        printf("Unable to set frame rate\n");
        return FALSE;
    }
    if(ioctl(fd, VIDIOC_G_PARM, &stream_para) == -1)
    {
        printf("Unable to get frame rate\n");
        return FALSE;
    }
    {
        printf("numerator:%d\ndenominator:%d\n",stream_para.parm.capture.timeperframe.numerator,stream_para.parm.capture.timeperframe.denominator);
    }
//        else

    /*        {
                printf("fmt.type:\t%d\n",fmt.type);         //可以输出图像的格式   Output image format
                printf("pix.pixelformat:\t%c%c%c%c\n",fmt.fmt.pix.pixelformat & 0xFF,(fmt.fmt.pix.pixelformat >> 8) & 0xFF,\
                       (fmt.fmt.pix.pixelformat >> 16) & 0xFF, (fmt.fmt.pix.pixelformat >> 24) & 0xFF);
                printf("pix.height:\t%d\n",fmt.fmt.pix.height);
                printf("pix.field:\t%d\n",fmt.fmt.pix.field);
            }
    */
    return TRUE;
}

int v4l2_grab(void)
{
    //struct v4l2_requestbuffers req = {0};
    //4  request for 4buffers 缓存不可少于两个   count of buffer >= 2
    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd, VIDIOC_REQBUFS, &req) == -1)        //开启内存映射或用户指针I/O    Start memory map or pointer I/O
    {
        printf("Requesting Buffer error\n");
        return FALSE;
    }
    //5 mmap for buffers
    buffers = (buffer*)malloc(req.count * sizeof(*buffers));
    if(!buffers)
    {
        printf("Out of memory\n");
        return FALSE;
    }
    unsigned int n_buffers;
    for(n_buffers = 0; n_buffers < req.count; n_buffers++)
    {
        //struct v4l2_buffer buf = {0};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = n_buffers;
        if(ioctl(fd, VIDIOC_QUERYBUF, &buf) == -1)  // 查询已经分配的V4L2的视频缓冲区的相关信息，包括视频缓冲区的使用状态、
        {
            //在内核空间的偏移地址、缓冲区长度等。 Search info of allocated V4L2 video buffer, including status, offset address,buffer length etc  
            printf("Querying Buffer error\n");
            return FALSE;
        }
        buffers[n_buffers].length = buf.length;

        buffers[n_buffers].start = (unsigned char*)mmap (NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);

        if(buffers[n_buffers].start == MAP_FAILED)
        {
            printf("buffer map error\n");
            return FALSE;
        }
    }
    //6 queue
    for(n_buffers = 0; n_buffers <req.count; n_buffers++)
    {
        buf.index = n_buffers;
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        if(ioctl(fd,VIDIOC_QBUF,&buf))     // 投放一个空的视频缓冲区到视频缓冲区输入队列中  Put in empty video buffer to input queue of video buffer
        {
            printf("query buffer error\n");
            return FALSE;
        }
    }
    //7 starting
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(ioctl(fd,VIDIOC_STREAMON,&type) == -1)  //
    {
        printf("stream on error\n");
        return FALSE;
    }
    return TRUE;
}
int v4l2_control(int value)
{
    ctrl.id=V4L2_CID_ZOOM_ABSOLUTE;
    ctrl.value=value;//change output mode 0x8004/0x8005
    int result=0;
    //shutter 0x8000
    if (ioctl(fd, VIDIOC_S_CTRL, &ctrl) == -1)
    {
        printf("v4l2_control error:%d\n",result);
        return FALSE;
    }
    return TRUE;
}
int traversalVideo(void)
{
    string device="/dev";
    string video="video";
    char* KEY_PTR=(char *)video.c_str();
    char* FILE_PTR=(char *)device.c_str();

    DIR *dir;
    struct dirent *ptr;
    char base[1000];

    if ((dir=opendir(FILE_PTR)) == NULL)
    {
        perror("Open dir error...");
        exit(1);
    }

    while ((ptr=readdir(dir)) != NULL)
    {
        string name="";
        name=(char*)ptr->d_name;
        if(name.find("video")!= string::npos)
        {
            string allName=device+"/"+name;
            if(init_v4l2(allName))
            {
                closedir(dir);
                return 1;
            }
        }
        printf("d_name:%s/%s\n",FILE_PTR,ptr->d_name);
    }

    closedir(dir);
    return 0;

}

void sendFloatCommand(int position, unsigned char value0, unsigned char value1, unsigned char value2, unsigned char value3, int interval0,
                      int interval1, int interval2, int interval3, int interval4)
{
    int psitionAndValue0 = (position << 8) | (0x000000ff & value0);
    printf("psitionAndValue0:%X\n",psitionAndValue0);
    //v4l2_control(psitionAndValue0);
    if(v4l2_control(psitionAndValue0) == FALSE)
    {
        printf("control fail psitionAndValue0~~\n");
        //exit(EXIT_FAILURE);
    }
    usleep(50000);
    int psitionAndValue1 = ((position + 1) << 8) | (0x000000ff & value1);
    printf("psitionAndValue1:%X\n",psitionAndValue1);
    if(v4l2_control(psitionAndValue1) == FALSE)
    {
        printf("control fail psitionAndValue1~~\n");
        //exit(EXIT_FAILURE);
    }
    usleep(50000);
    int psitionAndValue2 = ((position + 2) << 8) | (0x000000ff & value2);
    printf("psitionAndValue2:%X\n",psitionAndValue2);
    if(v4l2_control(psitionAndValue2) == FALSE)
    {
        printf("control fail psitionAndValue2~~\n");
        //exit(EXIT_FAILURE);
    }
    usleep(50000);
    int psitionAndValue3 = ((position + 3) << 8) | (0x000000ff & value3);
    printf("psitionAndValue3:%X\n",psitionAndValue3);
    if(v4l2_control(psitionAndValue3) == FALSE)
    {
        printf("control fail psitionAndValue3~~\n");
        exit(EXIT_FAILURE);
    }
    usleep(50000);
}
void sendUshortCommand(int position, unsigned char value0, unsigned char value1)
{
    int psitionAndValue0 = (position << 8) | (0x000000ff & value0);
    printf("psitionAndValue0:%X\n",psitionAndValue0);
    //v4l2_control(psitionAndValue0);
    if(v4l2_control(psitionAndValue0) == FALSE)
    {
        printf("control fail psitionAndValue0~~\n");
        //exit(EXIT_FAILURE);
    }
    int psitionAndValue1 = ((position + 1) << 8) | (0x000000ff & value1);
    printf("psitionAndValue1:%X\n",psitionAndValue1);
    usleep(50000);
    if(v4l2_control(psitionAndValue1) == FALSE)
    {
        printf("control fail psitionAndValue1~~\n");
        //exit(EXIT_FAILURE);
    }
    usleep(50000);
}
void sendByteCommand(int position, unsigned char value0, int interval0)
{
    int psitionAndValue0 = (position << 8) | (0x000000ff & value0);
    v4l2_control(psitionAndValue0);
    usleep(50000);
}

void sendCorrection(float correction)
{
    unsigned char iputCo[4];
    memcpy(iputCo,&correction,sizeof(float));
    sendFloatCommand(0 * 4, iputCo[0], iputCo[1], iputCo[2], iputCo[3], 20, 40, 60, 80, 120);
    printf("sendCorrection 0:%d,1:%d,2:%d,3:%d\n",iputCo[0],iputCo[1],iputCo[2],iputCo[3]);

}
void sendReflection(float reflection)
{
    unsigned char iputRe[4];
    memcpy(iputRe,&reflection,sizeof(float));
    sendFloatCommand(1 * 4, iputRe[0], iputRe[1], iputRe[2], iputRe[3], 20, 40, 60, 80, 120);

}
void sendAmb(float amb)
{
    unsigned char iputAm[4];
    memcpy(iputAm,&amb,sizeof(float));
    sendFloatCommand(2 * 4, iputAm[0], iputAm[1], iputAm[2], iputAm[3], 20, 40, 60, 80, 120);

}
void sendHumidity(float humidity)
{
    unsigned char iputHu[4];
    memcpy(iputHu,&humidity,sizeof(float));
    sendFloatCommand(3 * 4, iputHu[0], iputHu[1], iputHu[2], iputHu[3], 20, 40, 60, 80, 120);

}
void sendEmissivity(float emiss)
{
    unsigned char iputEm[4];
    memcpy(iputEm,&emiss,sizeof(float));
    sendFloatCommand(4 * 4, iputEm[0], iputEm[1], iputEm[2], iputEm[3], 20, 40, 60, 80, 120);
}

void sendDistance(unsigned short distance)
{
    unsigned char iputDi[2];
    memcpy(iputDi,&distance,sizeof(unsigned short));
    sendUshortCommand(5 * 4,iputDi[0],iputDi[1]);
}
void savePara()
{
    v4l2_control(0x80ff);
}
void setPoint(int viewX1,int viewY1,int indexOfPoint)
{
    int x1=0,x2=0,y1=0,y2=0;
    if(IMAGEWIDTH==640)
    {
        switch (indexOfPoint)
        {
            case 0:
                x1=0xf300+(viewX1&0x00ff);//low 8bit
                x2=0xf400+(viewX1>>8);//high 8bit
                y1=0xf500+(viewY1&0x00ff);
                y2=0xf600+(viewY1>>8);
                break;
            case 1:
                x1=0xf700+(viewX1&0x00ff);//low 8bit
                x2=0xf800+(viewX1>>8);//high 8bit
                y1=0xf900+(viewY1&0xff);
                y2=0xfa00+(viewY1>>8);
                break;
            case 2:
                x1=0xfb00+(viewX1&0x00ff);//low 8bit
                x2=0xfc00+(viewX1>>8);//high 8bit
                y1=0xfd00+(viewY1&0x00ff);
                y2=0xfe00+(viewY1>>8);
                break;
        }
        printf("x:0x%x,x1:0x%x,x2:0x%x,y:0x%x,y1:0x%x,y2:0x%x\n",viewX1,x1,x2,viewY1,y1,y2);
        v4l2_control(x1);
        usleep(50000);
        v4l2_control(x2);
        usleep(50000);
        v4l2_control(y1);
        usleep(50000);
        v4l2_control(y2);  
    }
    else if(IMAGEWIDTH==384)
    {
        switch (indexOfPoint)
        {
            case 0:
                x1=0xf300+viewX1;
                y1=0xf500+viewY1;
                break;
            case 1:
                x1=0xf700+viewX1;
                y1=0xf900+viewY1;
                break;
            case 2:
                x1=0xfb00+viewX1;
                y1=0xfd00+viewY1;
                break;
        }
        v4l2_control(x1);
        usleep(50000);
        v4l2_control(y1);  
    }    
    else
    {
        switch (indexOfPoint)
        {
            case 0:
                x1=0xf000+viewX1;
                y1=0xf200+viewY1;
                break;
            case 1:
                x1=0xf400+viewX1;
                y1=0xf600+viewY1;
                break;
            case 2:
                x1=0xf800+viewX1;
                y1=0xfa00+viewY1;
                break;
        }
        v4l2_control(x1);
        usleep(50000);
        v4l2_control(y1);  
    }

}
