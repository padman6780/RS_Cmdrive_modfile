#include <stdio.h>
#include "hocdec.h"
extern int nrnmpi_myid;
extern int nrn_nobanner_;
#if defined(__cplusplus)
extern "C" {
#endif

extern void _dcdt_reg(void);
extern void _HH_traub_reg(void);
extern void _IM_cortex_reg(void);

void modl_reg(){
  if (!nrn_nobanner_) if (nrnmpi_myid < 1) {
    fprintf(stderr, "Additional mechanisms from files\n");

    fprintf(stderr," \"dcdt.mod\"");
    fprintf(stderr," \"HH_traub.mod\"");
    fprintf(stderr," \"IM_cortex.mod\"");
    fprintf(stderr, "\n");
  }
  _dcdt_reg();
  _HH_traub_reg();
  _IM_cortex_reg();
}

#if defined(__cplusplus)
}
#endif
