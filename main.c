#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <memory.h>
#include <fcntl.h>
#include <errno.h>
#include <emmintrin.h>
#include <xmmintrin.h>
#include <time.h>
#ifndef _MSC_VER
#include <unistd.h>
#endif

void decode(const char *fname);
void encode(const char *fname);

void main(int argc, char *argv[])
{
	//encode("ZCZC-EAS-RWT-012057-012081+0030-2780415-WTSP/TV-", "my-same1.raw");
	decode("my-same1.raw");
}