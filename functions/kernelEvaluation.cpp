/*  NORMALIZECOLS.C  Compute Density Function and a Posteriori Variance
 * Syntax: [densityFunction,autocovariance] = aPosterioriVariance(
 *          xAxis,yAxis,inputLocations,estimatedCoefficients,autocovarience)
 */

#include <math.h>
#include "string.h"
#include "stdio.h"
#include "mex.h"
#include "blas.h"

#define IS_REAL_2D_FULL_DOUBLE(P) (!mxIsComplex(P) && \
        mxGetNumberOfDimensions(P) == 2 && !mxIsSparse(P) && mxIsDouble(P))
#define IS_REAL_SCALAR(P) (IS_REAL_2D_FULL_DOUBLE(P) && mxGetNumberOfElements(P) == 1)

        
        
/* The computational routine */
void vectorExponential(double xAxis, double yAxis, double *inputLocations, double *rowVector, mwSize n, double scale)
{
    mwSize i;
    /* multiply each element y by x */
    for (i=0; i < n; i++) {
        rowVector[i] = exp(- (pow( xAxis - inputLocations[i]   , 2 ) +
                              pow( yAxis - inputLocations[i+n] , 2 ))/ (pow(scale,2)) );
        
    }
}
        

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    /* Macros for the ouput and input arguments */
    #define KERNELROW_OUT plhs[0]
    #define XAXIS_IN prhs[0] 
    #define YAXIS_IN prhs[1] 
    #define INPUTLOCATIONS prhs[2]  
    #define SCALE prhs[3]


    double *xAxis, *yAxis, *inputLocations, *estimatedCoefficients;
    double *autoCovariance, *densityFunction, *aPosterioriVariance; 
    int mXAxis, mYAxis, mInputLocations, mEstimatedCoefficients, mAutoCovariance;
    int nXAxis, nYAxis, nInputLocations, nEstimatedCoefficients, nAutoCovariance;
    double *onesX,*onesY,*kernel,*kernelRow,scale;

    
    if(nrhs != 4) 
        /* Check the number of arguments */ 
        mexErrMsgTxt("Wrong number of input arguments.");
    else if(nlhs != 1)
        mexErrMsgTxt("Wrong number of output arguments.");
    
    
    if(!IS_REAL_2D_FULL_DOUBLE(XAXIS_IN)) /* Check  XAXIS_IN*/
        mexErrMsgTxt("A must be a real 2D full double array.");
    
    if(!IS_REAL_2D_FULL_DOUBLE(YAXIS_IN)) /* Check YAXIS_IN */
        mexErrMsgTxt("A must be a real 2D full double array.");
    
    if(!IS_REAL_2D_FULL_DOUBLE(INPUTLOCATIONS)) /* Check INPUTLOCATIONS */
        mexErrMsgTxt("A must be a real 2D full double array.");

    if(!IS_REAL_SCALAR(SCALE)) 
        mexErrMsgTxt("StdScale must be a real double scalar.");
    
    
    
    mXAxis = mxGetM(XAXIS_IN); /* Get the dimensions */
    nXAxis = mxGetN(XAXIS_IN);
    xAxis = mxGetPr(XAXIS_IN); /* Get the pointer to the dataof XAXIS */
    
    mYAxis = mxGetM(YAXIS_IN); /* Get the dimensions */
    nYAxis = mxGetN(YAXIS_IN);
    yAxis = mxGetPr(YAXIS_IN); /* Get the pointer to the dataof YAXIS_IN */
    
    mInputLocations = mxGetM(INPUTLOCATIONS); /* Get the dimensions */
    nInputLocations = mxGetN(INPUTLOCATIONS);
    inputLocations = mxGetPr(INPUTLOCATIONS); /* Get the pointer to the dataof INPUTLOCATIONS */
    
    scale = mxGetScalar(SCALE); /* Get the pointer to the dataof SCALE */
    

   
    KERNELROW_OUT = mxCreateDoubleMatrix(mInputLocations, mXAxis*mXAxis , mxREAL);
    kernel = mxGetPr(KERNELROW_OUT);    
    
    onesX     = (double*) malloc(mInputLocations * sizeof(double));
    onesY     = (double*) malloc(mInputLocations * sizeof(double));
    kernelRow = (double*) malloc(mInputLocations * sizeof(double));
        
    for(int i = 0; i < mXAxis ; i ++)
    {
        for(int j = 0; j < mXAxis ; j++)
        {

            // Kernel Evaluation: exp[-((x-xloc)^2 + (y - yloc)^2)/0.02]
            vectorExponential(xAxis[i], yAxis[j], inputLocations, kernelRow, mInputLocations, scale);
            memcpy((kernel+i*mXAxis*mInputLocations+j*mInputLocations),kernelRow,mInputLocations * sizeof(double));
            
            
        }
    }
    
    free(onesX);
    free(onesY);
    return; 
}
