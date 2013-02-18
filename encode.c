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

/*
* Bit Parameters
* The following definitions of a bit are based on a bit period equaling
* 1920 microseconds (± one microsecond).
* a.) The speed is 520.83 bits per second
* b.)  Logic zero is 1562.5 Hz.
* c.)  Logic one is 2083.3 Hz
*
* preamble is 0xAB sent on wire as LSB first
*     11010101
* start of message header begins with ZCZC
      0101101011000010
* message ends with NNNN
*     01110010
*
*	ZCZC-EAS-RWT-012057-012081-012101-012103-012115+0030-2780415-WTSP/TV-
*/

#define FREQ_MARK  2083.3                 // binary 1 freq, in Hz
#define FREQ_SPACE 1562.5                 // binary 0 freq, in Hz
#define FREQ_SAMP  22050                  // req'd input sampling rate, in Hz
#define BAUD       520.83                 // symbol rate, in Hz
#define PREAMBLE   ((unsigned char)0xAB)  // preamble byte, MSB first
#define HEADER_BEGIN "ZCZC"               // message begin
#define EOM "NNNN"                        // message end
#define CORRLEN ((int)(FREQ_SAMP/BAUD))

static short silence[FREQ_SAMP] = { 0 };
static void generate_byte(unsigned char data, unsigned short *stream);

void encode(const char *message, const char *fname)
{
	int fd, i, rep, ret;
	short buffer[CORRLEN * 8];
	unsigned char full_message[268 + 2 + 1];
	unsigned char footer[7];

#ifdef _MSC_VER
	if ((fd = open(fname, O_WRONLY | O_CREAT | O_BINARY)) < 0) {
#else
	if ((fd = open(fname, O_WRONLY | O_CREAT)) < 0) {
#endif
		return;
	}

	memset(full_message, 0, 268 + 2 + 1);
	full_message[0] = PREAMBLE;
	full_message[1] = PREAMBLE;
	memcpy(&full_message[2], message, strlen(message));

	footer[0] = PREAMBLE;
	footer[1] = PREAMBLE;
	footer[2] = 'N';
	footer[3] = 'N';
	footer[4] = 'N';
	footer[5] = 'N';
	footer[6] = 0x00;

	for(rep = 0; rep < 3; rep++)
	{
		for(i = 0; i < strlen(full_message); i++)
		{
			generate_byte(full_message[i], buffer);
			ret = write(fd, buffer, sizeof(short)*CORRLEN*8);
		}

		write(fd, silence, sizeof(short)*FREQ_SAMP);
	}

	//2 second pause
	write(fd, silence, sizeof(short)*FREQ_SAMP);
	write(fd, silence, sizeof(short)*FREQ_SAMP);

	//the audio!

	//2 second pause
	write(fd, silence, sizeof(short)*FREQ_SAMP);
	write(fd, silence, sizeof(short)*FREQ_SAMP);

	//the footer
	for(rep = 0; rep < 3; rep++)
	{
		for(i = 0; i < strlen(footer); i++)
		{
			generate_byte(footer[i], buffer);
			write(fd, buffer, sizeof(short)*CORRLEN*8);
		}

		write(fd, silence, sizeof(short)*FREQ_SAMP);
	}

	close(fd);
}

static void generate_byte(unsigned char data, unsigned short *stream)
{
	float f;
	unsigned short *ptr;
	int offset = 0, i, b, bit;

	ptr = stream;

	for(b = 0; b < 8; b++)
	{
		bit = (data >> b) & 0x01;

		for(f = 0, i = 0; i < CORRLEN; i++)
		{
			(*ptr++) = (short)(32768.0 * sin(f));

			if(bit)
				f += (float)(2.0*3.14159265359*FREQ_MARK/FREQ_SAMP);
			else
				f += (float)(2.0*3.14159265359*FREQ_SPACE/FREQ_SAMP);
		}
	}
}