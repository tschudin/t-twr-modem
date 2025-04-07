// codec2-ldpc.c

// Three LDPCs for up to 1024 bits, adding 256, 512 or 1024 redundancy bits
// The choice is made by setting the LDPD_PARITy_LENGTH macro

// Most code extracted from https://github.com/drowe67/codec2

#include <string.h>
#include <stdio.h>

#include "phi0.c"
#include "mpdecode_core.h"

// ---------------------------------------------------------------------------

#if LDPC_PARITY_LENGTH == 256
#  include "rnd_1024_256.c"
#  define LDPC_WORD_LENGTH       (1280)

   struct LDPC ldpc_code =
    {
        "RND_1024_256",
        RND_MAX_ITER,
        0,
        1,
        1,
        RND_CODELENGTH,
        RND_NUMBERPARITYBITS,
        RND_NUMBERROWSHCOLS,
        RND_MAX_ROW_WEIGHT,
        RND_MAX_COL_WEIGHT,
        (uint16_t *)RND_H_rows,
        (uint16_t *)RND_H_cols
    };

#endif

#if LDPC_PARITY_LENGTH == 512
#  include "HRAa_1536_512.c"
#  define LDPC_WORD_LENGTH       (1536)

   struct LDPC ldpc_code =
    /* used for 4FSK/LLR experiments */
    {
        "HRAa_1536_512",
        HRAa_1536_512_MAX_ITER,
        0,
        1,
        1,
        HRAa_1536_512_CODELENGTH,
        HRAa_1536_512_NUMBERPARITYBITS,
        HRAa_1536_512_NUMBERROWSHCOLS,
        HRAa_1536_512_MAX_ROW_WEIGHT,
        HRAa_1536_512_MAX_COL_WEIGHT,
        (uint16_t *)HRAa_1536_512_H_rows,
        (uint16_t *)HRAa_1536_512_H_cols
    };
#endif

#if LDPC_PARITY_LENGTH == 1024
#  include "H_1024_2048_4f.c"
#  define LDPC_WORD_LENGTH       (2048)

   struct LDPC ldpc_code =
    /* Another fine code from Bill VK5DSK - also useful for HF data */ 
    {
        "H_1024_2048_4f",
        H_1024_2048_4f_MAX_ITER,
        0,
        1,
        1,
        H_1024_2048_4f_CODELENGTH,
        H_1024_2048_4f_NUMBERPARITYBITS,
        H_1024_2048_4f_NUMBERROWSHCOLS,
        H_1024_2048_4f_MAX_ROW_WEIGHT,
        H_1024_2048_4f_MAX_COL_WEIGHT,
        (uint16_t *)H_1024_2048_4f_H_rows,
        (uint16_t *)H_1024_2048_4f_H_cols
    };
#endif

#define LDPC_DATA_LENGTH       (LDPC_WORD_LENGTH - LDPC_PARITY_LENGTH)

// ---------------------------------------------------------------------------

void
ldpc_setup()
{
    codec2_ldpc_setup(&ldpc_code);
}

// ---------------------------------------------------------------------------

// input: up to 1024 bits / output: LDPC_PARITY_LENGTH parity bits depending on selected code

int ldpc_encode_1024(uint8_t *pbits, uint8_t *ibits, int cnt)
{
  uint8_t *data = (uint8_t*) alloca(LDPC_DATA_LENGTH);

  memcpy(data, ibits, cnt);
  memset(data+cnt, 1, LDPC_DATA_LENGTH-cnt); // fill in the unused bits

  codec2_ldpc_encode(&ldpc_code, data, pbits);

  return LDPC_PARITY_LENGTH;
}

// ---------------------------------------------------------------------------

// input: 'cnt+LDPC_PARITY_LENGTH' LLR values (4./-4.), output: cnt bits
// the fct returns the number of decoding iterations
// note: both in- and output arrays must have place for WORD_LENGTH entries

int ldpc_decode_1024(uint8_t *obits, float *llr, int cnt,
                     int *parityCheckCount) // returns iter cnt
{
  memmove(llr + LDPC_DATA_LENGTH, llr + cnt, LDPC_PARITY_LENGTH*sizeof(float));
  for (int i = cnt; i < LDPC_DATA_LENGTH; i++)
      llr[i] = -10.;  // fill in the unused bits (set to 1)

  return codec2_ldpc_decode(&ldpc_code, obits, llr, parityCheckCount);
}

// ---------------------------------------------------------------------------

#include "mpdecode_core.c"

// eof
