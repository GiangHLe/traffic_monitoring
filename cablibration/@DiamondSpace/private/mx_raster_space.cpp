#include "mex.h"
#include <cmath>
#include <cstdio>

using namespace std;

//Lines maju line v stlpcoch a su single!!!!!

void lines_end_points(float * line, int * endpoints, float space_c, int numLines);
void rasterize_lines(float * line, int * endpoints, int * space, int SpaceSize, int numLines);
inline void lineH(int x0, int y0, int x1, int y1, int * space, int * y_steps, int weight);
inline void lineV(int x0, int y0, int x1, int y1, int * space, int * y_steps, int weight);

template <typename T> int sgn(T val) 
{
    return (T(0) <= val) - (val < T(0));
}

int myround(float x) 
{
    return ((x>=0) ? int(x + 0.5) : int(x - 0.5));
} 

void mexFunction(int nlhs, mxArray * plhs[], int nrhs, const mxArray * prhs[])
{
    #define SpaceSize prhs[0]
    #define Lines prhs[1]        
    
    //Create Space fills with zeros and size SpaceSize
    int cSpaceSize[2] = {mxGetScalar(SpaceSize),mxGetScalar(SpaceSize)};
    mxArray * Space = mxCreateNumericArray(2, cSpaceSize, mxINT32_CLASS, mxREAL);
    int * pSpace = (int*)mxGetData(Space); 
    
    //Get Lines data
    float * LinesData = (float*)mxGetData(Lines); 
    float space_c = (cSpaceSize[0] - 1.f)/2;

    int numLines = mxGetN(Lines);

    //int cE[2] = {8, numLines};
    //mxArray * mxE = mxCreateNumericArray(2, cE, mxINT32_CLASS, mxREAL);
    //int * EndPoints = (int*)mxGetData(mxE); 
    int * EndPoints =  (int*) malloc(sizeof(int)*8*numLines);

    //Get all EndPoints
    lines_end_points(LinesData, EndPoints, space_c, numLines);
    
    //Rasterize
    rasterize_lines(LinesData, EndPoints, pSpace, cSpaceSize[0], numLines);
    
    free(EndPoints);
    plhs[0] = Space;
    //plhs[0] = mxE;
}

void rasterize_lines(float * line, int * endpoints, int * space, int cSpaceSize, int numLines)
{ 
    int * v_steps = (int*)malloc(sizeof(int)*cSpaceSize);
    for(int i = 0; i < cSpaceSize; i++) 
        v_steps[i] = i*cSpaceSize;

    for(int i = 0; i < numLines; i++)
    {
        int * end = endpoints + i*8;
        int weight =  int(*(line + i*4 + 3));

        for(int j=0; j<6; j+=2)
        {
            if(abs(end[j+3] - end[j+1]) > abs(end[j+2] - end[j]))
                lineV(end[j], end[j+1], end[j+2], end[j+3], space, v_steps, weight);
            else
                lineH(end[j], end[j+1], end[j+2], end[j+3], space, v_steps, weight);        
        }        
        space[v_steps[end[7]] + end[6]] += weight;
    }
    free(v_steps);
}

void lines_end_points(float * line, int * endpoints, float space_c, int numLines)
{
    int center = myround(space_c);
    for(int i = 0; i < numLines; i++)
    {
        float a = *(line + i*4);
        float b = *(line + i*4 + 1); 
        float c = *(line + i*4 + 2);
    
        int * end = endpoints + i*8;
        
        float alpha = float(sgn(a*b));
        float beta = float(sgn(b*c));
        float gamma = float(sgn(a*c));
        
        //mexPrintf("%f, %f, %f\n", alpha, beta, gamma);
    
        float a_x = 0;
        if (a != 0) a_x = alpha*a / (c + gamma*a);
        float b_x = 0;
        if (c != 0) b_x = -alpha*c / (c + gamma*a);
        
        //mexPrintf("%f, %f\n", a_x, b_x);
       
        end[1] = myround((a_x + 1) * space_c);
        end[0] = myround((b_x + 1) * space_c);

        end[3] = myround((b / (c + beta*b) + 1) * space_c);
        end[2] = center;

        end[5] = center;
        end[4] = myround((b / (a + alpha*b) + 1) * space_c);

        end[7] = myround((-a_x + 1) * space_c);
        end[6] = myround((-b_x + 1) * space_c);   
                
        if (abs(c) < 0.00000001)
        {
            end[2] = end[0];
            end[3] = end[1];     
        }
                
        //printf("[%d %d] [%d %d] [%d %d] [%d %d]\n",end[0],end[1],end[2],end[3],end[4],end[5],end[6],end[7]);
    }
    
    
}

inline void lineH(int x0, int y0, int x1, int y1, int * space, int * y_steps, int weight)
{
    float slope = (float)(y1 - y0)/(x1 - x0);

	//float y_iter = y0 + 0.5f;     
    float y_start = y0 + 0.5f; 
    float y_iter = y_start;
    
    int step = (x0 < x1) ? 1 : -1;
    slope *= step;
    
    for(int x = x0, c = 1; x != x1; x+=step, c++)
	{   
        //space[y_steps[int(y_iter)] + x] += weight;        
        space[y_steps[int(y_iter)] + x] += 1;        
        //y_iter += slope;		  		
        y_iter = y_start + c*slope;
	}
    
}

inline void lineV(int x0, int y0, int x1, int y1, int * space, int * y_steps, int weight)
{
    float slope = (x1 - x0)/(float)(y1 - y0);

	//float x_iter = x0 + 0.5f; 
    float x_start = x0 + 0.5f; 
    float x_iter = x_start;
    int step = (y0 < y1) ? 1 : -1;
    slope *= step;

    for(int y = y0, c = 1; y != y1; y+=step, c++)
	{	
        //space[y_steps[y] + int(x_iter)] += weight;
        space[y_steps[y] + int(x_iter)] += 1;
        //x_iter += slope;		  
        x_iter = x_start + c*slope;        
	}     
}
