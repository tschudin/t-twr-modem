// fht.h

/* adapted from "FHT John Bryan 2017",
   https://www.theradixpoint.com/hartley/fht_in_c.html

   specifically: dynamic caching of cos, sin and bit reversal tables,
   Hann windowing, DC offset, boosting of higher freq (power)
*/

#define FHT_MAX_LOGN 10 // prepare for FHT up to 1024 elements (2^10)


float* fht_run(float *x, float *xo, int N, int p)
{
    // cache the cos and sin values for this p
    static float* trigo[FHT_MAX_LOGN];
    if (trigo[p] == NULL) {
        trigo[p] = (float*) malloc(N * sizeof(float));
        for (int i = 0; i < N/2; i++) {
            float r = 2 * PI * i / N;
            trigo[p][i]     = cos(r);
            trigo[p][i+N/2] = sin(r);
        }
    }
    float *costbl = trigo[p], *sintbl = trigo[p] + N/2;

    // cache the bit reversal vector for this p
    static uint16_t* bitrev[FHT_MAX_LOGN];
    if (bitrev[p] == NULL) {
        uint16_t *r = (uint16_t*) calloc(N, sizeof(uint16_t));
        for (int M = 1; M < N; M *= 2) {
            for (int k = 0; k < M; k++) {
                int T = 2 * r[k];
                r[k] = T;
                r[k+M] = T+1;
            }
        }
        bitrev[p] = r;
    }

    // do the bit reversal
    uint16_t *r = bitrev[p];
    for (int n = 1; n < (N-1); n++) {
        if (r[n] > n) { // swap
            float temp = x[n];
            x[n] = x[r[n]];
            x[r[n]] = temp;
        }
    }

    // fast hartley transform 
    int Np = 2, Bp = N/2, tss = N/2;
    for (int P = 0; P < p; P++) {
        int Npp = Np/2, baseT = 0;
        for (int b = 0; b < Bp; b++) {
            int baseB = baseT + Npp, baseBB = baseT + Np;
            for (int n = 0; n < Npp; n++) {
                float xcs, *x1, *x2;
                if (P%2 == 0)
                    x1 = xo, x2 = x;
                else
                    x1 = x, x2 = xo;
                if (n == 0)
                    xcs = x2[baseB+n];
                else {
                    int tf = n * tss, Nmn = (baseBB - n) % baseBB;
                    xcs = (x2[baseB+n] * costbl[tf]) + (x2[Nmn] * sintbl[tf]);
                }
                x1[baseT+n] = x2[baseT+n] + xcs;
                x1[baseB+n] = x2[baseT+n] - xcs;
            }
            baseT = baseT + Np;
       }
       Np *= 2;
       Bp /= 2;
       tss /= 2;
    }

    return (p%2 == 0) ? x : xo;
}


// Fast Hartley Transform returning the power in each bin, including
// - optional padding (pre- and appending N/2 zeros)
// - applying Hann window
// - gradually boosting higher frequencies

void fht(const int16_t *signal_in, int N, bool padding,
         float boost, float *power_vect, float *total_power)
{
    // uint64_t start = esp_timer_get_time() - start;

    int p = 1;
    while ((1 << p) < N) p++;

    // cache the Hann window function for this p
    static float* hann[FHT_MAX_LOGN];
    if (hann[p] == NULL) {
        hann[p] = (float*) malloc(N * sizeof(float));
        for (int i = 0; i < N; i++) {
            float a = sin(PI * (i+0.5) / N);
            hann[p][i] = a * a;
            // Serial.printf("hann %g\r\n", a*a);
        }
    }

    float *data0 = (float*) alloca(N * sizeof(float));
    // apply Hann windowing
    float *h = hann[p];
    for (int i = 0; i < N; i++)
        data0[i] = h[i] * (float)signal_in[i];

    float *data1, *data2;
    if (padding) {
      data1 = (float*) alloca(2*N * sizeof(float));
      data2 = (float*) alloca(2*N * sizeof(float));
      for (int i = 0; i < N/2; i++)
        data1[i] = data1[2*N-1 - i] = 0;
      for (int i = 0; i < N; i++)
        data1[N/2 + i] = data0[i];
      N *= 2;
      p++;
    } else {
      data1 = data0;
      data2 = (float*) alloca(N * sizeof(float));
    }
    
    float *y = fht_run(data1, data2, N, p);

    power_vect[0] = abs(y[0]); // DC component
    if (total_power)
        *total_power = 0;
    float f = 1; // a simple equalizer (boost higher frequencies
    for (int i = 1; i <= N/2; i++) {
        float u, v;
        /*
          u = y[i] + y[N - 1 - i], v = y[i] - y[N - 1 - i];
          // (a+b)(a+b) + (a-b)*(a-b) =
          // a^2+2ab+b^2 + (a^2-2ab+b^2) =
          // 2a^2 + 2b^2 --> can also just squary the bins
        */
        u = y[i];
        v = y[N - 1 - i];
        power_vect[i] = f * sqrt(u*u + v*v) / N; // normalize and boost
        if (total_power)
            *total_power += power_vect[i];
        f *= boost;
    }

    // start = esp_timer_get_time() - start;
    // Serial.printf("// FH transform N=%d in %d usec\r\n", N, (int) start);
}

// eof
