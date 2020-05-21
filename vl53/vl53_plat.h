/*------------------------------------------------------------------------
| vl53_plat.h  --  libvl53 platform and footprint selection
|
| New material Copyright (c) 2020 Gord Seiferling.
| Preexisting material Copyright © 2016, STMicroelectronics International N.V.
| Refer to license-lib.txt.
|
| Platform-specific includes needed only for library modules.
*/

#ifndef _VL53_PLAT_H_
#define _VL53_PLAT_H_

#ifdef __QNX__

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>

#else

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "libbg.h"

#endif

/*---  compile-time options to control driver footprint  ---*/
#define AllStrings
//#define AllFunction
#define SigmaEst
#define SigmaFloat
#define I2c_Debug

#endif

