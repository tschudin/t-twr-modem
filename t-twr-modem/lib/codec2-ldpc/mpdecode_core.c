/*
  FILE...: mpdecode_core.c
  AUTHOR.: Matthew C. Valenti, Rohit Iyer Seshadri, David Rowe
  CREATED: Sep 2016

  adapted (static mem alloc etc) Mar 2025 <christian.tschudin@unibas.ch>
*/

#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <assert.h>

#include "mpdecode_core.h"
#ifndef USE_ORIGINAL_PHI0
#include "phi0.h"
#endif

// #include "debug_alloc.h"
#define CALLOC(N,SZ) calloc(N,SZ)
#define FREE(P)      free(P)

#ifdef __EMBEDDED__
#include "machdep.h"
#endif

// c_nodes will be an array of NumberParityBits of struct c_node
// Each c_node contains an array of <degree> c_sub_node elements
// This structure reduces the indexing caluclations in SumProduct()

struct c_sub_node { // Order is important here to keep total size small.
  uint16_t index;   // Values from H_rows (except last 2 entries)
  uint16_t socket;  // The socket number at the v_node
  float    message; // modified during operation!
};

struct c_node {
  int degree;       // A count of elements in the following arrays
  struct c_sub_node *subs;
};

// v_nodes will be an array of CodeLength of struct v_node

struct v_sub_node {
  uint16_t index;  //    the index of a c_node it is connected to
                   //    Filled with values from H_cols (except last 2 entries)
  uint16_t socket; //    socket number at the c_node
  float message;   //    Loaded with input data
                   //    modified during operation!
  uint8_t sign;    //    1 if input is negative
                   //    modified during operation!
};

struct v_node {
  int degree;       // A count of ???
  float initial_value;
  struct v_sub_node *subs;
};

#ifdef USE_ORIGINAL_PHI0
/* Phi function */
static float phi0( float x )
{
  float z;

  if (x>10)
    return( 0 );
  else if (x< 9.08e-5 )
    return( 10 );
  else if (x > 9)
    return( 1.6881e-4 );
  /* return( 1.4970e-004 ); */
  else if (x > 8)
    return( 4.5887e-4 );
  /* return( 4.0694e-004 ); */
  else if (x > 7)
    return( 1.2473e-3 );
  /* return( 1.1062e-003 ); */
  else if (x > 6)
    return( 3.3906e-3 );
  /* return( 3.0069e-003 ); */
  else if (x > 5)
    return( 9.2168e-3 );
  /* return( 8.1736e-003 ); */
  else {
    z = (float) exp(x);
    return( (float) log( (z+1)/(z-1) ) );
  }
}
#endif

// ---------------------------------------------------------------------------

static struct c_node the_c_nodes[LDPC_PARITY_LENGTH];
static struct v_node the_v_nodes[LDPC_WORD_LENGTH];

void codec2_ldpc_setup(struct LDPC *c)
{
    int DataLength = c->CodeLength - c->NumberParityBits;
    int i, j, k, count, cnt, c_index, v_index, H1;

    int shift = (c->NumberParityBits + c->NumberRowsHcols) -
                 c->CodeLength;
    if (c->NumberRowsHcols == c->CodeLength) {
        H1=0;
        shift=0;
    } else {
        H1=1;
    }

    /* first determine the degree of each c-node */

    if (shift ==0){
        for (i=0;i<c->NumberParityBits;i++) {
            count = 0;
            for (j=0;j<c->max_row_weight;j++) {
                if ( c->H_rows[i+j*c->NumberParityBits] > 0 ) {
                    count++;
                }
            }
            the_c_nodes[i].degree = count;
            if (H1){
                if (i==0){
                    the_c_nodes[i].degree=count+1;
                }
                else{
                    the_c_nodes[i].degree=count+2;
                }
            }
        }
    }
    else{
        cnt=0;
        for (i=0;i<(c->NumberParityBits/shift);i++) {
            for (k=0;k<shift;k++){
                count = 0;
                for (j=0;j<c->max_row_weight;j++) {
                    if ( c->H_rows[cnt+j*c->NumberParityBits] > 0 ) {
                        count++;
                    }
                }
                the_c_nodes[cnt].degree = count;
                if ((i==0)||(i==(c->NumberParityBits/shift)-1)){
                    the_c_nodes[cnt].degree=count+1;
                }
                else{
                    the_c_nodes[cnt].degree=count+2;
                }
                cnt++;
            }
        }
    }

    if (H1){
        if (shift ==0){
            for (i=0;i<c->NumberParityBits;i++) {
                // Allocate sub nodes
              if (the_c_nodes[i].degree <= 0)
                printf("neg calloc\n");
              the_c_nodes[i].subs = (struct c_sub_node*) CALLOC(the_c_nodes[i].degree, sizeof(struct c_sub_node));
                assert(the_c_nodes[i].subs);

	        // Populate sub nodes
                for (j=0;j<the_c_nodes[i].degree-2;j++) {
                    uint16_t x = c->H_rows[i+j*c->NumberParityBits];
                    if (x == 0) {
                        printf("** zero for i=%d j=%d\n", i, j);
                        // break;
                    }
                    the_c_nodes[i].subs[j].index = (c->H_rows[i+j*c->NumberParityBits] - 1);
                }
                j=the_c_nodes[i].degree-2;

                if (i==0){
                    uint16_t x = c->H_rows[i+j*c->NumberParityBits];
                    if (x == 0) {
                        printf("** (i==0) zero for i=%d j=%d\n", i, j);
                        // break;
                    }
                    the_c_nodes[i].subs[j].index = (c->H_rows[i+j*c->NumberParityBits] - 1);
                }
                else {
                    uint16_t x = c->H_rows[i+j*c->NumberParityBits];
                    the_c_nodes[i].subs[j].index = DataLength+i-1;
                }

                j=the_c_nodes[i].degree-1;
                the_c_nodes[i].subs[j].index = DataLength+i;

            }
        }
        if (shift >0){
            cnt=0;
            for (i=0;i<(c->NumberParityBits/shift);i++){

                for (k =0;k<shift;k++){

                    // Allocate sub nodes
                  the_c_nodes[cnt].subs = (struct c_sub_node*) CALLOC(the_c_nodes[cnt].degree, sizeof(struct c_sub_node));
                    assert(the_c_nodes[cnt].subs);

	            // Populate sub nodes
                    for (j=0;j<the_c_nodes[cnt].degree-2;j++) {
                        the_c_nodes[cnt].subs[j].index = (c->H_rows[cnt+j*c->NumberParityBits] - 1);
                    }
                    j=the_c_nodes[cnt].degree-2;
                    if ((i ==0)||(i==(c->NumberParityBits/shift-1))){
                        the_c_nodes[cnt].subs[j].index = (c->H_rows[cnt+j*c->NumberParityBits] - 1);
                    }
                    else{
                        the_c_nodes[cnt].subs[j].index = DataLength+k+shift*(i);
                    }
                    j=the_c_nodes[cnt].degree-1;
                    the_c_nodes[cnt].subs[j].index = DataLength+k+shift*(i+1);
                    if (i== (c->NumberParityBits/shift-1))
                        {
                            the_c_nodes[cnt].subs[j].index = DataLength+k+shift*(i);
                        }
                    cnt++;
                }
            }
        }

    } else {
        for (i=0;i<c->NumberParityBits;i++) {
            // Allocate sub nodes
          the_c_nodes[i].subs = (struct c_sub_node*) CALLOC(the_c_nodes[i].degree, sizeof(struct c_sub_node));
            assert(the_c_nodes[i].subs);

	    // Populate sub nodes
            for (j=0;j<the_c_nodes[i].degree;j++){
                the_c_nodes[i].subs[j].index = (c->H_rows[i+j*c->NumberParityBits] - 1);
            }
        }
    }


    /* determine degree of each v-node */

    for(i=0;i<(DataLength+shift);i++){
        count=0;
        for (j=0;j<c->max_col_weight;j++) {
            if ( c->H_cols[i+j*c->NumberRowsHcols] > 0 ) {
                count++;
            }
        }
        the_v_nodes[i].degree = count;
    }

    for(i=DataLength+shift;i<c->CodeLength;i++){
        count=0;
        if (H1){
            if(i!=c->CodeLength-1){
                the_v_nodes[i].degree=2;
            }  else{
                the_v_nodes[i].degree=1;
            }

        } else{
            for (j=0;j<c->max_col_weight;j++) {
                if ( c->H_cols[i+j*c->NumberRowsHcols] > 0 ) {
                    count++;
                }
            }
            the_v_nodes[i].degree = count;
        }
    }

    if (shift>0){
        the_v_nodes[c->CodeLength-1].degree = the_v_nodes[c->CodeLength-1].degree+1;
    }


    /* set up v_nodes */

    for (i=0;i<c->CodeLength;i++) {
        // Allocate sub nodes
        the_v_nodes[i].subs = (struct v_sub_node*) CALLOC(the_v_nodes[i].degree, sizeof(struct v_sub_node));
        assert(the_v_nodes[i].subs);

	// Populate sub nodes

        /* index tells which c-nodes this v-node is connected to */
        // v_nodes[i].initial_value = input[i]; --> happens in init()
        count=0;

        for (j=0;j<the_v_nodes[i].degree;j++) {
            if ((H1)&& (i>=DataLength+shift)){
                the_v_nodes[i].subs[j].index=i-(DataLength+shift)+count;
                if (shift ==0){
                    count=count+1;
                }
                else{
                    count=count+shift;
                }
            } else  {
                the_v_nodes[i].subs[j].index = (c->H_cols[i+j*c->NumberRowsHcols] - 1);
            }

            /* search the connected c-node for the proper message value */
            for (c_index=0;c_index<the_c_nodes[ the_v_nodes[i].subs[j].index ].degree;c_index++)
                if ( the_c_nodes[ the_v_nodes[i].subs[j].index ].subs[c_index].index == i ) {
                    the_v_nodes[i].subs[j].socket = c_index;
                    break;
                }
        }
    }

    /* now finish setting up the c_nodes */
    for (i=0;i<c->NumberParityBits;i++) {
        /* index tells which v-nodes this c-node is connected to */
        for (j=0;j<the_c_nodes[i].degree;j++) {
            /* search the connected v-node for the proper message value */
            for (v_index=0;v_index<the_v_nodes[ the_c_nodes[i].subs[j].index ].degree;v_index++)
                if (the_v_nodes[ the_c_nodes[i].subs[j].index ].subs[v_index].index == i ) {
                    the_c_nodes[i].subs[j].socket = v_index;
                    break;
                }
        }
    }

}

void init_c_v_nodes(struct LDPC *c, float *input)
{
    for (int i = 0; i < c->NumberParityBits; i++) {
        for (int j = 0; j < the_c_nodes[i].degree; j++) {
            the_c_nodes[i].subs[j].message = 0;
        }
    }

    for (int i = 0; i < c->CodeLength; i++) {
        the_v_nodes[i].initial_value = input[i];
        for (int j = 0; j < the_v_nodes[i].degree; j++) {
            /* initialize v-node with received LLR */
            if (c->dec_type == 1)
                the_v_nodes[i].subs[j].message = fabs(input[i]);
            else
                the_v_nodes[i].subs[j].message = phi0( fabs(input[i]) );
            the_v_nodes[i].subs[j].sign = (input[i] < 0) ? 1 : 0;
        }
    }
}


///////////////////////////////////////
/* function for doing the MP decoding */
// Returns the iteration count
int SumProduct( int       *parityCheckCount,
                char     DecodedBits[],
                struct c_node c_nodes[],
                struct v_node v_nodes[],
                int       CodeLength,
                int       NumberParityBits,
                int       max_iter,
                float    r_scale_factor,
                float    q_scale_factor  )
{
  int result;
  // int bitErrors;
  int i,j, iter;
  float phi_sum;
  int sign;
  float temp_sum;
  float Qi;
  int   ssum;


  result = max_iter;
  for (iter=0;iter<max_iter;iter++) {

    for(i=0; i<CodeLength; i++) DecodedBits[i] = 0; // Clear each pass!
    // bitErrors = 0;

    /* update r */
    ssum = 0;
    for (j=0;j<NumberParityBits;j++) {
      sign = v_nodes[ c_nodes[j].subs[0].index ].subs[ c_nodes[j].subs[0].socket ].sign;
      phi_sum = v_nodes[ c_nodes[j].subs[0].index ].subs[ c_nodes[j].subs[0].socket ].message;

      for (i=1;i<c_nodes[j].degree;i++) {
        // Compiler should optomize this but write the best we can to start from.
        struct c_sub_node *cp = &c_nodes[j].subs[i];
        struct v_sub_node *vp = &v_nodes[ cp->index ].subs[ cp->socket ];
	    phi_sum += vp->message;
	    sign ^= vp->sign;
      }

      if (sign==0) ssum++;

      for (i=0;i<c_nodes[j].degree;i++) {
        struct c_sub_node *cp = &c_nodes[j].subs[i];
        struct v_sub_node *vp = &v_nodes[ cp->index ].subs[ cp->socket ];
	    if ( sign ^ vp->sign ) {
	      cp->message = -phi0( phi_sum - vp->message ); // *r_scale_factor;
        } else
	      cp->message =  phi0( phi_sum - vp->message ); // *r_scale_factor;
      }
    }

    /* update q */
    for (i=0;i<CodeLength;i++) {

      /* first compute the LLR */
      Qi = v_nodes[i].initial_value;
      for (j=0;j<v_nodes[i].degree;j++) {
        struct v_sub_node *vp = &v_nodes[i].subs[j];
	    Qi += c_nodes[ vp->index ].subs[ vp->socket ].message;
      }

      /* make hard decision */
      if (Qi < 0) {
            DecodedBits[i] = 1;
      }

      /* now subtract to get the extrinsic information */
      for (j=0;j<v_nodes[i].degree;j++) {
        struct v_sub_node *vp = &v_nodes[i].subs[j];
	    temp_sum = Qi - c_nodes[ vp->index ].subs[ vp->socket ].message;

	    vp->message = phi0( fabs( temp_sum ) ); // *q_scale_factor;
        if (temp_sum > 0)
	      vp->sign = 0;
        else
	      vp->sign = 1;
      }
    }

    // count the number of PC satisfied and exit if all OK
    *parityCheckCount = ssum;
    if (ssum==NumberParityBits)  {
      result = iter + 1;
      break;
    }
  }

  return(result);
}

// ---------------------------------------------------------------------------

void codec2_ldpc_encode( struct LDPC *c,
                         unsigned char ibits[], unsigned char pbits[] )
{
    unsigned int p, i, tmp, par, prev=0;
    int          ind;
    uint16_t     *H_rows = c->H_rows;

    for (p=0; p<c->NumberParityBits; p++) {
        par = 0;

        for (i=0; i<c->max_row_weight; i++) {
            ind = H_rows[p + i*c->NumberParityBits];
            if (ind) par = par + ibits[ind-1];
        }
        tmp = par + prev;
        tmp &= 1;    // only retain the lsb

        prev = tmp;
        pbits[p] = tmp;
    }
}


int codec2_ldpc_decode( struct LDPC *ldpc, uint8_t out_char[],
                        float input[], int *parityCheckCount )
{
    init_c_v_nodes(ldpc, input);

    return SumProduct( parityCheckCount, (char*) out_char,
                       the_c_nodes, the_v_nodes,
                       ldpc->CodeLength, ldpc->NumberParityBits,
                       ldpc->max_iter,
                       ldpc->r_scale_factor, ldpc->q_scale_factor );
}


void sd_to_llr(float llr[], float sd[], int n) {
    double sum, mean, sign, sumsq, estvar, estEsN0, x;
    int i;

    /* convert SD samples to LLRs -------------------------------*/

    sum = 0.0;
    for(i=0; i<n; i++)
        sum += fabs(sd[i]);
    mean = sum/n;

    /* find variance from +/-1 symbol position */

    sum = sumsq = 0.0;
    for(i=0; i<n; i++) {
        sign = (sd[i] > 0.0L) - (sd[i] < 0.0L);
        x = ((double)sd[i]/mean - sign);
        sum += x;
        sumsq += x*x;
    }
    estvar = (n * sumsq - sum * sum) / (n * (n - 1));
    //fprintf(stderr, "mean: %f var: %f\n", mean, estvar);

    estEsN0 = 1.0/(2.0L * estvar + 1E-3);
    for(i=0; i<n; i++)
        llr[i] = 4.0L * estEsN0 * sd[i];
}

// eof
