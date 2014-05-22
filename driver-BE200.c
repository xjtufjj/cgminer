/* cgminer UART driver for BE200 */
/* Copyright 2014, BE200 Dev Team */

/* This program is free software; you can redistribute it and/or modify it */
/* under the terms of the GNU General Public License as published by the Free */
/* Software Foundation; either version 3 of the License, or (at your option) */
/* any later version.  See COPYING for more details. */

#include <float.h>
#include <limits.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <strings.h>
#include <sys/time.h>
#include <unistd.h>
#include <math.h>

#include "logging.h"
#include "miner.h"
#include "util.h"
#include "compat.h"
#include "usbutils.h"

#include "driver-BE200.h"



