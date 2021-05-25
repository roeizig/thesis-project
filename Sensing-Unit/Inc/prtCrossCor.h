#ifndef __PRTCROSSCOR_H
#define __PRTCROSSCOR_H
#include "arm_math.h"

#define capturedPoints 4096
#define capturedPoints2 4096
#define Pi 3.1415926535f

#define micDistance 0.043 			// [meter]		Value on final system
#define SoS 346.13 					// [meter/second]
#define samplingFreq 143770 		// [samples/second]


#define shift 41					// Maximum possible shift of max correlation


arm_status partialXCorr (float32_t* pSrcA, uint32_t* cpltCntr, float32_t* pSrcB, float32_t* pDst);

float32_t AOADetection (float32_t* xcor, int32_t*	maxShift); 

void convert2Float (uint16_t* input, float32_t* output, uint16_t length);

void ADCToFloat	(uint16_t * pSrc, float32_t * pDst, uint32_t blockSize);


#endif
