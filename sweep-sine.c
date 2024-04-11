/* Output a slow sine wave as S16_LE samples to stdout
   forever until interrupted.
*/

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

const double time_per_sample = 1.0 / 273000;  // OPV_RPC sample rate

const double wiggle_freq = 0.1;       // 10 second period sweeping tone back and forth
const double wiggle_extent = 0x7FF0;  // plus-and-minus maximum value sweep, with a little headroom

double wiggle_radians_per_sample = 2 * M_PI * wiggle_freq * time_per_sample;

int main(void)
{
    static double wiggle = 0.0;
    double deviation;
    int16_t sample;

    while(1)
    {
        wiggle = fmod(wiggle + wiggle_radians_per_sample, 2 * M_PI);
        deviation = sin(wiggle) * wiggle_extent;
        sample = (int16_t)deviation;
        fwrite(&sample, 2, 1, stdout);
    }
    
}