/*
*      demod_eas.c -- Emergency Alert System demodulator
*
*      See http://www.nws.noaa.gov/nwr/nwrsame.htm
*
*      Copyright (C) 2013
*          M. Weber <mweber@gatech.edu>
*
*      Based on
*
*      Copyright (C) 2000
*          A. Maitland Bottoms <bottoms@debian.org>
*
*      Licensed under same terms and based upon the
*         demod_afsk12.c -- 1200 baud AFSK demodulator
*
*         Copyright (C) 1996
*          Thomas Sailer (sailer@ife.ee.ethz.ch, hb9jnx@hb9w.che.eu)
*
*      This program is free software; you can redistribute it and/or modify
*      it under the terms of the GNU General Public License as published by
*      the Free Software Foundation; either version 2 of the License, or
*      (at your option) any later version.
*
*      This program is distributed in the hope that it will be useful,
*      but WITHOUT ANY WARRANTY; without even the implied warranty of
*      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*      GNU General Public License for more details.
*
*      You should have received a copy of the GNU General Public License
*      along with this program; if not, write to the Free Software
*      Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

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
*/

static float mac(const float* a, const float* b, unsigned int size);
static float fsqr(float f);

#define MAX(a,b) (((a)>(b))?(a):(b))
#define MIN(a,b) (((a)<(b))?(a):(b))

enum EAS_L2_State
{
   EAS_L2_IDLE = 0,
   EAS_L2_HEADER_SEARCH = 1,
   EAS_L2_READING_MESSAGE = 2,
   EAS_L2_READING_EOM = 3,
};

static char last_message[269];
static char msg_buf[4][269];
static char head_buf[4];
static unsigned long headlen;
static unsigned long msglen;
static unsigned long msgno;

static int frame_state;

#define FREQ_MARK  2083.3                 // binary 1 freq, in Hz
#define FREQ_SPACE 1562.5                 // binary 0 freq, in Hz
#define FREQ_SAMP  22050                  // req'd input sampling rate, in Hz
#define BAUD       520.83                 // symbol rate, in Hz
#define PREAMBLE   ((unsigned char)0xAB)  // preamble byte, MSB first
#define HEADER_BEGIN "ZCZC"               // message begin
#define EOM "NNNN"                        // message end

// Storage options
#define MAX_MSG_LEN 268                   // maximum length of EAS message
#define MAX_HEADER_LEN 4                  // header length (begin vs end)
#define MAX_STORE_MSG 3                   // # of msgs to store and compare

// Signal processing options
#define DLL_GAIN_UNSYNC 1/2.0             // DLL gain when unsynchronized
#define DLL_GAIN_SYNC 1/2.0               // DLL gain when synchronized
#define DLL_MAX_INC 8192                  // max DLL per-sample shift
#define INTEGRATOR_MAXVAL 10              // sampling integrator bounds
#define MIN_IDENTICAL_MSGS 2              // # of msgs which must be identical

#define CORRLEN ((int)(FREQ_SAMP/BAUD))
#define SPHASEINC (0x10000u*BAUD/FREQ_SAMP)

static float eascorr_mark_i[CORRLEN];
static float eascorr_mark_q[CORRLEN];
static float eascorr_space_i[CORRLEN];
static float eascorr_space_q[CORRLEN];

static void eas_init();
static void eas_demod(float *buffer, int length);

void main(int argc, char *argv[])
{
	int fd;
	int i;
	short buffer[8192];
	float fbuf[16384];
	unsigned int fbuf_cnt = 0;
	short *sp;
	const char *fname = "same3.raw";

	eas_init();

#ifdef _MSC_VER
        if ((fd = open(fname, O_RDONLY | O_BINARY)) < 0) {
#else
        if ((fd = open(fname, O_RDONLY)) < 0) {
#endif
		return;
	}

	for(;;)
	{
		i = read(fd, sp = buffer, sizeof(buffer));

		if(i < 0 && errno != EAGAIN) {
			perror("read");
			exit(4);
		}

		if(!i)
			break;

		if(i > 0)
		{
			for(; i >= sizeof(buffer[0]); i -= sizeof(buffer[0]), sp++)
				fbuf[fbuf_cnt++] = (*sp) * (1.0f/32768.0f);

			if(i)
				fprintf(stderr, "warning: noninteger number of samples read\n");

			if(fbuf_cnt > CORRLEN)
			{
				eas_demod(fbuf, fbuf_cnt-CORRLEN);
				memmove(fbuf, fbuf+fbuf_cnt-CORRLEN, CORRLEN*sizeof(fbuf[0]));
				fbuf_cnt = CORRLEN;
			}
		}
	}

	close(fd);
}

static void eas_init()
{
	float f;
	int i;

	for(f = 0, i = 0; i < CORRLEN; i++) {
		eascorr_mark_i[i] = (float)cos(f);
		eascorr_mark_q[i] = (float)sin(f);
		f += (float)(2.0*3.14159265359*FREQ_MARK/FREQ_SAMP);
	}
	for(f = 0, i = 0; i < CORRLEN; i++) {
		eascorr_space_i[i] = (float)cos(f);
		eascorr_space_q[i] = (float)sin(f);
		f += (float)(2.0*3.14159265359*FREQ_SPACE/FREQ_SAMP);
	}
}

static char eas_allowed(char data)
{
	// determine if a character is allowed in an EAS frame
	// returns true if it is

	// high-byte ASCII characters are forbidden
	if(data & 0x80)
		return 0;
	if(data == 13 || data == 10)
		// LF and CR are allowed
		return 1;
	if(data >= 32 || data <= 126)
		// These text and punctuation characters are allowed
		return 1;

	// all other characters forbidden
	return 0;
}

static void process_frame_char(char data)
{
	int i, j = 0;
	char *ptr = 0;
	int have_complete_set_of_messages;
	
	if(data)
	{
		// if we're idle, now we're looking for a header
		if(frame_state == EAS_L2_IDLE)
			frame_state = EAS_L2_HEADER_SEARCH;
		
		if(frame_state == EAS_L2_HEADER_SEARCH && headlen < MAX_HEADER_LEN)
		{
			// put it in the header buffer if we have room
			head_buf[headlen] = data;
			headlen++;
		}
		
		if(frame_state == EAS_L2_HEADER_SEARCH && headlen >= MAX_HEADER_LEN)
		{
			// test first 4 bytes to see if they are a header
			if(!strncmp(head_buf, HEADER_BEGIN, headlen))
				// have found header. keep reading
				frame_state = EAS_L2_READING_MESSAGE;
			else if(!strncmp(head_buf, EOM, headlen))
				// have found EOM
				frame_state = EAS_L2_READING_EOM;
			else
			{
				// not valid, abort and clear buffer
				frame_state = EAS_L2_IDLE;
				headlen = 0;
			}
		}
		else if(frame_state == EAS_L2_READING_MESSAGE && msglen <= MAX_MSG_LEN)
		{
			// space is available; store in message buffer
			msg_buf[msgno][msglen] = data;
			msglen++;
		}
	}
	else
	{
		// the header has ended
		// fill the rest of the buffer will NULs
		memset(&msg_buf[msgno][msglen], '\0', MAX_MSG_LEN - msglen); 
		//msg_buf[msgno][msglen] = '\0';

		if(frame_state == EAS_L2_READING_MESSAGE)
		{
			// All EAS messages should end in a minus sign("-")
			// trim any trailing characters
			ptr = strrchr(&msg_buf[msgno], '-');
			if(ptr)
			{
				// found. make the next character zero
				*(ptr+1) = '\0';
			}
			
			// display message if verbosity permits
			//verbprintf(7, "\n");
			printf("EAS (part): %s%s\n", HEADER_BEGIN, msg_buf[msgno]);
			
			// increment message number
			msgno += 1;
			if(msgno >= MAX_STORE_MSG)
				msgno = 0;

			have_complete_set_of_messages = 1;

			for(i = 0; i < MAX_STORE_MSG; i++)
			{
				if(msg_buf[i][0] == '\0')
				{
					have_complete_set_of_messages = 0;
					break;
				}
			}
			
			if(have_complete_set_of_messages)
			{
				// check for message agreement; 2 of 3 must agree
				for(i = 0; i < MAX_STORE_MSG; i++)
				{
					// if this message is empty or matches the one we've just 
					// alerted the user to, ignore it.
					if(msg_buf[i][0] == '\0')
						continue;

					for(j = i+1; j < MAX_STORE_MSG; j++)
					{
						// test if messages are identical and not a dupe of the
						// last message
						if(!strncmp(msg_buf[i], msg_buf[j], MAX_MSG_LEN))
						{
							// store message to prevent dupes
							strncpy(last_message, msg_buf[j], MAX_MSG_LEN);
							
							// raise the alert and discontinue processing
							//verbprintf(7, "\n");
							printf("EAS: %s%s\n", HEADER_BEGIN, last_message);
							i = MAX_STORE_MSG;
							break;
						}
					}
				}
			}
		}
		else if(frame_state == EAS_L2_READING_EOM)
		{
			// raise the EOM
			printf("EAS: %s\n", EOM);
			msgno = 0;

			for(i = 0; i < MAX_STORE_MSG; i++)
				msg_buf[i][0] = '\0';
		}

		// go back to idle
		frame_state = EAS_L2_IDLE;
		msglen = 0;
		headlen = 0;
	}
}

static void eas_demod(float *buffer, int length)
{
	static unsigned int shift_reg;
	static unsigned int sphase;
	static unsigned char current_kar = 0;
	static unsigned char bit_counter = 0;
	static int dcd_integrator;
	static int decoder_synced = 0;

	float f;
	float dll_gain;

	for(; length >= 0; length--, buffer++)
	{
		f = fsqr(mac(buffer, eascorr_mark_i, CORRLEN)) +
			fsqr(mac(buffer, eascorr_mark_q, CORRLEN)) -
			fsqr(mac(buffer, eascorr_space_i, CORRLEN)) -
			fsqr(mac(buffer, eascorr_space_q, CORRLEN));

		// f > 0 if a mark is detected
		// keep the last few correlator samples in shift_reg
		// when we've synchronized to the bit transitions, the shift_reg
		// will have (nearly) a single value per symbol
		shift_reg <<= 1;
		shift_reg |= (f > 0);

		// the integrator is positive for 1 bits, and negative for 0 bits
		if(f > 0 && (dcd_integrator < INTEGRATOR_MAXVAL))
		{
			dcd_integrator += 1;
		}
		else if(f < 0 && dcd_integrator > -INTEGRATOR_MAXVAL)
		{
			dcd_integrator -= 1;
		}
		
		// check if transition occurred on time
		if(frame_state != EAS_L2_IDLE)
			dll_gain = DLL_GAIN_SYNC;
		else
			dll_gain = DLL_GAIN_UNSYNC;

		// want transitions to take place near 0 phase
		if((shift_reg ^ (shift_reg >> 1)) & 1)
		{
			if(sphase < (0x8000u-(SPHASEINC/8)))
			{
				// before center; check for decrement
				if(sphase > (SPHASEINC/2))
				{
					sphase -= MIN((int)((sphase)*dll_gain), DLL_MAX_INC);
					//verbprintf(10,"|-%d|", MIN((int)((sphase)*dll_gain), DLL_MAX_INC));
				}
			}
			else
			{
				// after center; check for increment
				if(sphase < (0x10000u - SPHASEINC/2))
				{
					sphase += MIN((int)((0x10000u - sphase)* dll_gain), DLL_MAX_INC);
					//verbprintf(10,"|+%d|", MIN((int)((0x10000u - sphase)* dll_gain), DLL_MAX_INC));
				}
			}
		}

		sphase += (unsigned int)SPHASEINC;
		
		// end of bit period?
		if(sphase >= 0x10000u)
		{
			sphase = 1;
			current_kar >>= 1;
			
			// if at least half of the values in the integrator are 1, 
			// declare a 1 received
			current_kar |= ((dcd_integrator >= 0) << 7) & 0x80;
			
			// check for sync sequence
			// do not resync when we're reading a message!
			if(current_kar == PREAMBLE && frame_state != EAS_L2_READING_MESSAGE)
			{
				// sync found; declare current offset as byte sync
				decoder_synced = 1;
				bit_counter = 0;
				//verbprintf(9, " sync");
			}
			else if(decoder_synced)
			{
				bit_counter++;

				if(bit_counter == 8)
				{
					if(eas_allowed((char)current_kar))
					{
						process_frame_char((char)current_kar);
					}
					else
					{
						//lose sync
						decoder_synced = 0;
						process_frame_char(0x00);
					}

					bit_counter = 0;
				}
			}
		}
	}
}

static float mac(const float* a, const float* b, unsigned int size)
{
	unsigned int i;
	float z = 0.0f, fres = 0.0f;
#ifdef WIN32
	__declspec(align(16)) float ftmp[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
#else
	float ftmp[] __attribute__ ((aligned (16))) = { 0.0f, 0.0f, 0.0f, 0.0f };   
#endif
	__m128 mres, mv1, mv2;

	if((size / 4) != 0) {
		mres = _mm_load_ss(&z);

		for(i = 0; i < size / 4; i++)
			mres = _mm_add_ps(mres, _mm_mul_ps(_mm_loadu_ps(&a[4*i]), _mm_loadu_ps(&b[4*i])));

		//mres = a,b,c,d
		mv1 = _mm_movelh_ps(mres, mres);     //a,b,a,b
		mv2 = _mm_movehl_ps(mres, mres);     //c,d,c,d
		mres = _mm_add_ps(mv1, mv2);         //res[0],res[1]

		_mm_store_ps(ftmp, mres);

		fres = ftmp[0] + ftmp[1];
	}

	if((size % 4) != 0) {
		for(i = size - size % 4; i < size; i++)
			fres += a[i] * b[i];
	}

	return fres;
}

static float fsqr(float f)
{
	return f*f;
}
