#ifndef ANDORPROTO_H

/* sdk prototypes and #defines */
#include "atmcdLXd.h"

// mnemonic values
#define FAILcamSelect  -1
#define ZEROcamSelect   0

// the Andor DRV_ macros begin with 20000
// we fake some other numbers
#define DRV_nosuch     1        /* Andor SDK2 does not define such a value */
#define DRV_notPassed  2        /* Andor SDK2 does not define such a value */

char*   drvdef(
int     value   /* one of the Andor #define DRV_ macros in atmcdLXd.h */
);

char*   acCameratype(
int     value   /* one of the Andor #define AC_CAMERATYHPE_ macros in atmcdLXd.h */
);

char*   acReadModes(
int     mask    /* value of ul_ReadModes lookup into AC_READMODE_ macros in atmcdLXd.h */
);

int CameraSelect (
int     iNumArgs,
char*   szArgList[]
);

char*   emgaindef(
int     emgainmode
);

#define ANDORPROTO_H
#endif /* ANDORPROTO_H */
