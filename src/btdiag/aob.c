/* aob.c */
#include "aob.h"

btreal eval_aob(AOB* aob, btreal rk, vect_n *yk){
   // yhk = Ca * ((Fc * Sp) + (Gama * rkp))
   // S = (Fc * Sp) + (Gama * rkp) + (Kk * (yk - yhk))
   // out = L_NAOB * S
   
   setcol_mn(aob->yk, yk, 0); // Copy the vector into a matrix to perform calculation
   
   // Using S->ret to store common factor: ((Fc * Sp) + (Gama * rkp))
   add_mn(aob->S->ret, mul_mn(aob->S->ret, aob->Fc, aob->S), scale_mn(aob->rkp, aob->Gama));
   mul_mn(aob->yhk, aob->Ca, aob->S->ret);
   add_mn(aob->S, aob->S->ret, mul_mn(aob->S, aob->Kk, add_mn(aob->yhk->ret, aob->yk, scale_mn(-1.0,aob->yhk))));
   
   aob->rkp = rk; // Store this rk into rk_previous
   
   return mul_mn(aob->Mscalar, aob->L_NAOB, aob->S)->q[0];
}

int new_aob(char *fn, AOB *aob, btreal *Lc, btreal *m, btreal *Ko, btreal *K2){
   FILE *inp;
   int i, sd, nout;
   
   if((inp = fopen(fn, "r")) == NULL) {
      printf("\nCould not open file: %s\n", fn);
      exit(0);
   }
   fscanf(inp, "%*s%d %*s%d %*s%lf %*s%lf %*s%lf %*s%lf", &sd, &nout, m, Lc, Ko, K2);

   
   // State vars
   aob->rkp = 0.0;
   aob->yk = new_mn(nout, 1);
   aob->yhk = new_mn(nout, 1);
   aob->S = new_mn(sd, 1);
   zero_mn(aob->S);
   aob->Mscalar = new_mn(1,1);
   
   aob->Fo = new_mn(sd, sd); // Unused
   aob->Fc = new_mn(sd, sd);
   aob->Ca = new_mn(nout, sd);
   aob->Pzero =  new_mn(sd, sd); // Unused
   aob->Qnoise =  new_mn(sd, sd); // Unused
   aob->Rnoise = new_mn(nout,nout); // Unused
   aob->L_NAOB = new_mn(1, sd);
   aob->Gama = new_mn(sd, 1);
   aob->Kk = new_mn(sd, nout);
  
   fscanf(inp, "%*s");
   for(i = 0; i < (sd*sd); i++)
      fscanf(inp, "%lf", &aob->Fo->q[i]);

   fscanf(inp, "%*s");
   for(i = 0; i < (sd*sd); i++)
      fscanf(inp, "%lf", &aob->Fc->q[i]);

   fscanf(inp, "%*s");
   for(i = 0; i < (sd*nout); i++)
      fscanf(inp, "%lf", &aob->Ca->q[i]);

   fscanf(inp, "%*s");
   for(i = 0; i < (sd*sd); i++)
      fscanf(inp, "%lf", &aob->Pzero->q[i]);

   fscanf(inp, "%*s");
   for(i = 0; i < (sd*sd); i++)
      fscanf(inp, "%lf", &aob->Qnoise->q[i]);

   fscanf(inp, "%*s");
   for(i = 0; i < (nout*nout); i++)
      fscanf(inp, "%lf", &aob->Rnoise->q[i]);

   fscanf(inp, "%*s");
   for(i = 0; i < sd; i++)
      fscanf(inp, "%lf", &aob->L_NAOB->q[i]);

   fscanf(inp, "%*s");
   for(i = 0; i < sd; i++)
      fscanf(inp, "%lf", &aob->Gama->q[i]);

   fscanf(inp, "%*s");
   for(i = 0; i < (sd*nout); i++)
      fscanf(inp, "%lf", &aob->Kk->q[i]);

   fclose(inp);
   
   return(nout);
}


