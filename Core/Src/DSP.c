#include <main.h>
#include "DSP.h"
#include "math.h"
#include "stdbool.h"

double y_value[ARRSIZE] = {0};
double cool_sin(double arg)
{
  double temp = arg - (arg*arg*arg)/6 + (arg*arg*arg*arg*arg)/120 - (arg*arg*arg*arg*arg*arg*arg)/5040;
  return temp;
}

void dsp(void)
{
  for(uint16_t i=0; i<ARRSIZE; i++)
  {
   y_value[i]=1.0f+Sin7((double)i*2.0f*3.14f/ARRSIZE) /*+ (rand()%1000)/1000.0f*/;
  }
}

double Sin7(double x)
{
	x *= 0.63661977236758134308; // 2/Pi
	int sign = x < 0.0;
	x = sign ? -x : x;
	int xf = (int)x;
	x -= xf;
	if ((xf & 1) == 1)
		x = 1 - x;
	int per = ((xf >> 1) & 1) == 1;
	double xx = x * x;
	double y = x * (1.5707903005870776 + xx * (-0.6458858977085938 + 
			xx*(0.07941798513358536 - 0.0043223880120647346 * xx)));
	return sign ^ per ? -y : y;
}

double movingAvg(moving_average_t* av_obj, double new_element)
{
  //Subtract the oldest number from the prev sum, add the new number
  av_obj->sum = av_obj->sum - av_obj->buffer[av_obj->pos] + new_element;
  //Assign the nextNum to the position in the array
  av_obj->buffer[av_obj->pos] = new_element;
  //Increment position internaly
  av_obj->pos++;
  if (av_obj->pos >= av_obj->length){
    av_obj->pos = 0;
    av_obj->is_filled = true;
  }
  //return the average
  return av_obj->sum / (av_obj->is_filled ? av_obj->length:av_obj->pos);
}

moving_average_t* allocate_moving_average(int len)
{
    moving_average_t* av_obj = malloc(sizeof(moving_average_t));
    av_obj->sum       = 0;
    av_obj->pos       = 0;
    av_obj->length    = len;
    av_obj->is_filled = false;
    av_obj->buffer = malloc(len * sizeof(double));
    return av_obj;
}

void free_moving_average(moving_average_t* av_obj)
{
    free(av_obj->buffer);
    av_obj->buffer = NULL;
    free(av_obj);
}

double movAvg(double *ptrArrNumbers, double *ptrSum, int pos, double len, double nextNum)
{
  *ptrSum = *ptrSum - ptrArrNumbers[pos] + nextNum;
  ptrArrNumbers[pos] = nextNum;
  return *ptrSum / len;
}
//double movingAvg2(double *ptrArrNumbers, double *ptrSum, int pos, int len, double nextNum)
//{
//  //Subtract the oldest number from the prev sum, add the new number
//  *ptrSum = *ptrSum - ptrArrNumbers[pos] + nextNum;
//  //Assign the nextNum to the position in the array
//  ptrArrNumbers[pos] = nextNum;
//  //return the average
//  return *ptrSum / len;
//}
