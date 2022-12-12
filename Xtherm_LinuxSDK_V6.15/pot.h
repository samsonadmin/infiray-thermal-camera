#ifndef POT_H_
#define POT_H_
#define DEBUG 0
#include <stdbool.h>
#ifdef __cplusplus
extern "C"  //C++
{
#endif

void  collectData(int* counter,unsigned short* src,short* dest,int height,int width,unsigned short** tempBuffer);



#ifdef __cplusplus
}
#endif

#endif

