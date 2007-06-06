#include <math.h>

void get_calibrated_settings(int *range, float *slope, float *height, float *max_membership){  
	int size_of_range = 7;
	int i,j;
	for (i = 1, j = 0 ; i< size_of_range - 1; i++, j++){
		height[j] = abs(range[i-1] - range[i+1]);
		*(slope +j*(size_of_range-2)) = height[j] / (float) (abs(range[i] - range[i-1]));
		*(slope +j*(size_of_range-2) + 1) = height[j] / (float) (abs(range[i+1] - range[i])) ;

		max_membership[j] = height[j] * height[j] / 2;
	}

}

int getIndex(int sample_diff, int *range, float *slope, float *height, float *max_membership){
    int size_of_range = 7;
    int i, j;
    float membership_value[5];
    float sum;
    float divisor;
    int dist1, dist2, h, membership;
    float x2, dist1b, dist2b;
 
    /* test membeship here*/

    sum = 0;
    for (i = 1, j = 0; i < size_of_range - 1; i++, j++){
        if (sample_diff > range[i]){
            dist1 = range[i+1] - sample_diff; 
            dist2 = sample_diff - range[i];
            h = (int) height[j] - (slope +j*(size_of_range-2) + 1)*dist2;
            x2 = (h / *(slope +j*(size_of_range-2))) + range[i-1];
            dist1b = x2 - range[i-1];
            dist2b = range[i] - x2;
        } else {
            dist1 = sample_diff - range[i-1];
            dist2 = range[i] - sample_diff;
            h = *(slope +j*(size_of_range-2))*dist1;
            x2 = (h - height[j]) / - *(slope +j*(size_of_range-2) + 1) + range[i];
            dist1b = range[i+1] - x2;
            dist2b = x2-range[i];
        }
        membership = (h*dist2 + h*dist2b) + (h*dist1/2 + h*dist1b/2);

        if (sample_diff > range[i+1] || sample_diff < range[i-1])
            membership = 0; 

        membership_value[j] = membership / (float) max_membership[j];

        sum += i * membership_value[j];

    }

    divisor = 0;
    for (j = 0; j < 5; j++)
        divisor += membership_value[j];
    if (divisor == 0)
	return 0;
    
    return sum/divisor - 1;
} 