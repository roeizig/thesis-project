/******************************************************************************

                           Partial Cross-Correlation Function
                
			

*******************************************************************************/

#include "arm_math.h"
#include "main.h"
#include "prtCrossCor.h"


const float64_t ADC2Float = (2.0f/4095.0f);	 // (ADCvalue * (2.0 / 0xFFF)) - 1.0  @  12 bits range, -1.0V to 1.0V normalized voltage



/*
		************************************************************************************************************************************************
*/
	
	float32_t AOADetection (
		float32_t* xcor,
		int32_t*	maxShift
		)
	{
		
		
		float32_t maxCor = 0;								// The maximum value of the cross-corelation
		uint32_t maxCorIndex = 0;							// The index of the maximum value of the cross-corelation
		float32_t tau, theta, alpha = 0;					// The time delay between ears, the angle from ears to signal's source, and the angle to turn to be perpendicular to source
		float32_t tmp = 0;									// Temporary calculation variable
		
		arm_max_f32(xcor, 2*shift+1 , &maxCor, &maxCorIndex);
		*maxShift = (maxCorIndex - shift);
		tau = samplingFreq*micDistance; // NOT ACCTUALLY TAU! JUST USING THE VARIABLE HERE
		tmp = *maxShift * SoS/tau;
		if ((tmp > 1) || (tmp < -1)) {
			tmp = ((tmp > 0) - (tmp < 0));
		}
		theta = acos(tmp);
		alpha = 90 - (theta * 180U / Pi);
		return alpha;
	}

/*
		************************************************************************************************************************************************
*/

	void convert2Float (
		uint16_t* input,
		float32_t* output,
		uint16_t length
		)
	{
		for (uint16_t i = 0; i < length; i++)	{
		output[i] = (input[i] * ADC2Float);
		}
	}

/*
		************************************************************************************************************************************************
*/

/**
 * @brief  Converts the elements of the uint16_t vector with 12 bit values to floating-point vector.
 * @param[in]       *pSrc points to the uint16_t input vector
 * @param[out]      *pDst points to the floating-point output vector
 * @param[in]       blockSize length of the input vector
 * @return none.
 *
 * \par Description:
 *
 * The equation used for the conversion process is:
 *
 * <pre>
 * 	pDst[n] = (float32_t) pSrc[n] * ADC2Float - 1.0f ;   0 <= n < blockSize.
 * </pre>
 *
 */


void ADCToFloat(
	uint16_t* pSrc,
	float32_t* pDst,
	uint32_t blockSize)
{

  uint16_t *pIn = pSrc;                          /* Src pointer */
  uint32_t blkCnt;                               /* loop counter */


  /* Run the below code for Cortex-M4 and Cortex-M3 */

  /*loop Unrolling */
  blkCnt = blockSize >> 2U;

  /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.
   ** a second loop below computes the remaining 1 to 3 samples. */
  while (blkCnt > 0U)
  {
    /* convert from uint12_t to float and then store the results in the destination buffer */
	*pDst++ = ((float32_t) *pIn++ / 2047.5f) - 1.0f;
    *pDst++ = ((float32_t) *pIn++ / 2047.5f) - 1.0f;
    *pDst++ = ((float32_t) *pIn++ / 2047.5f) - 1.0f;
    *pDst++ = ((float32_t) *pIn++ / 2047.5f) - 1.0f;

    /* Decrement the loop counter */
    blkCnt--;
  }

	
  /* If the blockSize is not a multiple of 4, compute any remaining output samples here.
   ** No loop unrolling is used. */
  blkCnt = blockSize % 0x4U;


  while (blkCnt > 0U)
  {
    /* convert from 12-bit uint to float and then store the results in the destination buffer */
    *pDst++ = ((float32_t) *pIn++ * ADC2Float) - 1.0f;

    /* Decrement the loop counter */
    blkCnt--;
  }
}

/*
		************************************************************************************************************************************************
*/

