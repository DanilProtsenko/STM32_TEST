#ifndef DSP_H_
#define DSP_H_
#include "main.h"
#include "stdbool.h"

void dsp(void);
void filtr(double* arr, double n, int size);
double Sin7(double x);

typedef struct moving_average
{
    double sum;
    int pos;
    double* buffer;
    int length;
    bool is_filled;
}moving_average_t;

double movingAvg(moving_average_t* av_obj, double new_element);
moving_average_t* allocate_moving_average(int len);
void free_moving_average(moving_average_t * av_obj);
double movAvg(double *ptrArrNumbers, double *ptrSum, int pos, double len, double nextNum);
#endif /* DSP_H_  */

