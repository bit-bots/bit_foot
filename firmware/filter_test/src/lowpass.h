/******************************* SOURCE LICENSE *********************************
Copyright (c) 2019 MicroModeler.

A non-exclusive, nontransferable, perpetual, royalty-free license is granted to the Licensee to 
use the following Information for academic, non-profit, or government-sponsored research purposes.
Use of the following Information under this License is restricted to NON-COMMERCIAL PURPOSES ONLY.
Commercial use of the following Information requires a separately executed written license agreement.

This Information is distributed WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

******************************* END OF LICENSE *********************************/

// A commercial license for MicroModeler DSP can be obtained at http://www.micromodeler.com/launch.jsp

// Begin header file, lowpass.h

#ifndef LOWPASS_H_ // Include guards
#define LOWPASS_H_

static const int lowpass_numStages = 2;
static const int lowpass_coefficientLength = 10;
extern float lowpass_coefficients[10];

typedef struct
{
	float state[8];
	float output;
} lowpassType;

typedef struct
{
	float *pInput;
	float *pOutput;
	float *pState;
	float *pCoefficients;
	short count;
} lowpass_executionState;


lowpassType *lowpass_create( void );
void lowpass_destroy( lowpassType *pObject );
 void lowpass_init( lowpassType * pThis );
 void lowpass_reset( lowpassType * pThis );
#define lowpass_writeInput( pThis, input )  \
	lowpass_filterBlock( pThis, &(input), &(pThis)->output, 1 );

#define lowpass_readOutput( pThis )  \
	(pThis)->output

 int lowpass_filterBlock( lowpassType * pThis, float * pInput, float * pOutput, unsigned int count );
#define lowpass_outputToFloat( output )  \
	(output)

#define lowpass_inputFromFloat( input )  \
	(input)

 void lowpass_filterBiquad( lowpass_executionState * pExecState );
#endif // LOWPASS_H_
	
