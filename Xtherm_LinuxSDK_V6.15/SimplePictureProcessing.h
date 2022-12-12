#ifndef SIMPLE_PICTURE_PROCESSING_H
#define SIMPLE_PICTURE_PROCESSING_H



#include<stdio.h>
#include<stdlib.h>
#include <stdbool.h>
#ifdef __cplusplus
extern "C"  //C++
{
#endif
    void SimplePictureProcessingInit(int width,int height);//初始化Initialize
    void SimplePictureProcessingInitMidVar(size_t** midVar);//初始化中间量initialize Mid Variables
	 /**
     *设置方法二参数
	 * @para a：该值越高，能够看到的背景信息越明显，但是噪声也越严重。该值越低，对背景会抑制，目标突出。范围0-500,推荐值30。
	 * @para b：该值越高，可使比较少的点也能明显显示。当b趋向于0时，可减少高温物体进入，画面变暗的现象。范围0-50,推荐值29.
	 * @para c：高亮度点数调整，该值越大，高亮度点越多。范围0-500,推荐值3
	 * @para d：低亮度点数调整，该值越大，低亮度点越多。范围0-500,推荐值3
     * @para bright：亮度。范围0-100,推荐值50
     * @para contra：对比度。范围0-100,推荐值50
	 */
    void SetMethodTwoParameter(int a,int b,int c,int d,int bright,int contra);
    void SetParameter(float a,float b,float c,float d,float e,float f);
    void SetDivTemp(int div1,int div2);
	//设置参数，参数已经调校好，请勿修改 Set parameters, all set, please dont modify

	 /**
	 * 使用专业图像算法将8004数据转变为图像 Transfer 8004 data into graphs via algorithm
	 * @para input：输入指针，指向8004数据开始 Input pointer, point to 8004 data
	 * @para output：输出指针，指向rgba图像开始 Output pointer, point to rgba graph
	 * @para kindOfPalette1: 0：白热。White Hot 1：黑热。Black Hot 2：铁虹。 Iron Raindow 3：彩虹1。Rainbow 1 
	 * 4、彩虹2. Rainbow 2 5:高对比彩虹1.High-contrast rainbow 1 6:高对比彩虹2. High-contrast rainbow 2 >=7、用户色板 Customize palette
	 */
    void Compute(unsigned short* input,unsigned char* output,int kindOfPalette1,size_t** midVar,int format);
    void ComputeMethodTwo(unsigned short* input,unsigned char* output,int kindOfPalette1,size_t** midVar,int format);//0:rgba 1:rgb
    void ComputeDivTemp(unsigned short* input,unsigned char* output,int kindOfPalette1,size_t** midVar,int format);
    void ComputeDivTempType2(unsigned short* input,unsigned char* output,int kindOfPalette1,int lowTmpPaletteIndex,size_t** midVar,int displayMode,int format);
	 /**
	 * 设置用户色板 Set User Palette
	 * @para palette：输入指针，指向用户色板开始 input pointer, point to user palette
	 * @para typeOfPalette：需要>=7 needs >=7
	 */
    void SetUserPalette(unsigned char* palette,int typeOfPalette);
    void SimplePictureProcessingDeinit();//释放资源 Release resources 
    void SimplePictureProcessingDeinitMidVar(size_t** midVar);//释放中间变量 Release mid variables

	/**
	 * 获得色板Capture palette
	 * @para type：
	 * (0)：256*3 铁虹 iorn rainbow
	 * (1)：256*3 彩虹1 Rainbow 1
	 * (2)：224*3 彩虹2 Rainbow 2
	 * (3)：448*3 高对比彩虹1 High-Contrast Rainbow 1
	 * (4)：448*3 高对比彩虹2 High-Contrast Rainbow 2
	 */
    const unsigned char* getPalette(int type);


#ifdef __cplusplus
}
#endif

#endif
