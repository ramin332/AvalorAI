#include <stdio.h>
#include <stdlib.h>
#include <time.h>
double randomBoxMuller()
{
    double u1 = rand() / (double)RAND_MAX; // Generate a random number between 0 and 1
    double u2 = rand() / (double)RAND_MAX; // Generate another random number between 0 and 1

    double z1 = sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2); // Apply Box-Muller transform
    double z2 = sqrt(-2.0 * log(u1)) * sin(2.0 * M_PI * u2); // Apply Box-Muller transform

    // Transform the generated numbers to the desired range
    z1 = (z1 + 3) / 6; // Adjust the mean and standard deviation
    z2 = (z2 + 3) / 6; // Adjust the mean and standard deviation

    // Return one of the generated numbers
    return z1;
}


short levy_distribution(int seed, double gamma_levy)
{   double u= randomBoxMuller();
    //double u = ((double)rand() / RAND_MAX);  // Generate a random number between 0 and 1
    double levy = gamma_levy*pow(u, -1.9048);
    if (levy >= (gamma_levy/33)*800)
    {
        levy = (gamma_levy/33)*800;
    }
    //printf("gamma_levy is %f [constant], levy is[stepsize] %f\n", gamma_levy, levy);
    return levy;
}
