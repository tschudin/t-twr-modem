/*
  FILE...: mpdecode_core.h
  AUTHOR.: David Rowe
  CREATED: Sep 2016

  C-callable core functions for MpDecode, so they can be used for
  Octave and C programs.  Also some convenience functions to help use
  the C-callable LDPC decoder in C programs.
*/

#ifndef __MPDECODE_CORE__
#define __MPDECODE_CORE__

#include <stdint.h>

struct LDPC {
    char name[32];
    int max_iter;
    int dec_type;
    int q_scale_factor;
    int r_scale_factor;
    int CodeLength;
    int NumberParityBits;
    int NumberRowsHcols;
    int max_row_weight;
    int max_col_weight;

    uint16_t *H_rows;
    uint16_t *H_cols;
    
    /* these two are fixed to code params */
    int ldpc_data_bits_per_frame;
    int ldpc_coded_bits_per_frame;

    /* support for partial use of data bits in codeword and unequal protection schemes */
    int protection_mode; 
    int data_bits_per_frame;
    int coded_bits_per_frame;
};

void codec2_ldpc_setup(struct LDPC *c);
void codec2_ldpc_encode(struct LDPC *ldpc, unsigned char ibits[], unsigned char pbits[]);

int codec2_ldpc_decode(struct LDPC *ldpc, uint8_t out_char[], float input[], int *parityCheckCount);

void sd_to_llr(float llr[], float sd[], int n);

#endif
