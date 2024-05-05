#include <math.h>
#include <stdlib.h>
#include <time.h>

double randn(double mu, double sigma, int custom_seed)
{
  double U1, U2, W, mult;
  static double X1, X2;
  static int call = 0;
  srand(time(0) + custom_seed);
  if (call == 1)
  {
    call = !call;
    double normal1= (mu + sigma * (double)X2);
    //printf("normal1 is %f\n", normal1);
    return (normal1);
  }

  do
  {
    U1 = -1 + ((double)rand() / RAND_MAX) * 2;
    U2 = -1 + ((double)rand() / RAND_MAX) * 2;
    W = pow(U1, 2) + pow(U2, 2);
  } 
  while (W >= 1 || W == 0);
  mult = sqrt((-2 * log(W)) / W);
  X1 = U1 * mult;
  X2 = U2 * mult;

  call = !call;
  double normal2=(mu + sigma * (double)X1);
    //printf("normal2 is %f\n", normal2);
  return (normal2);
  

}