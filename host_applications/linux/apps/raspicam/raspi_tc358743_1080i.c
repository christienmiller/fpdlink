/*
Copyright (c) 2015, Raspberry Pi Foundation
Copyright (c) 2015, Dave Stevenson
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
//Hacked firmware, so can use a mmal_connection
//#define MANUAL_CONNECTION

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>

//#include <wiringPi.h>
//#include <wiringPiI2C.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
//#include "i2c-dev.h"

#include "interface/vcos/vcos.h"
#include "bcm_host.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_connection.h"

#include <sys/ioctl.h>
#include "tc358743_regs.h"

// Do the GPIO waggling from here, except that needs root access, plus
// there is variation on pin allocation between the various Pi platforms.
// Provided for reference, but needs tweaking to be useful.
//#define DO_PIN_CONFIG

#define u8  uint8_t
#define u16 uint16_t
#define u32 uint32_t

struct sensor_regs {
   uint16_t reg;
   uint8_t  data;
};

#define WIDTH 1920
#define HEIGHT 540
#define ENCODING MMAL_ENCODING_BGR24 //MMAL_ENCODING_I420// //MMAL_ENCODING_YUYV

#define UNPACK MMAL_CAMERA_RX_CONFIG_UNPACK_NONE
#define PACK MMAL_CAMERA_RX_CONFIG_PACK_NONE

#define I2C_ADDR 0x0F

#define CSI_IMAGE_ID 0x00
#define CSI_DATA_LANES 2

static void i2c_rd(int fd, uint16_t reg, uint8_t *values, uint32_t n)
{
	int err;
	uint8_t buf[2] = { reg >> 8, reg & 0xff };
	struct i2c_rdwr_ioctl_data msgset;
	struct i2c_msg msgs[2] = {
		{
			.addr = I2C_ADDR,
			.flags = 0,
			.len = 2,
			.buf = buf,
		},
		{
			.addr = I2C_ADDR,
			.flags = I2C_M_RD,
			.len = n,
			.buf = values,
		},
	};

	msgset.msgs = msgs;
	msgset.nmsgs = 2;

	err = ioctl(fd, I2C_RDWR, &msgset);
}

static void i2c_wr(int fd, uint16_t reg, uint8_t *values, uint32_t n)
{
	uint8_t data[1024];
	int err, i;
	struct i2c_msg msg;
	struct i2c_rdwr_ioctl_data msgset;

	if ((2 + n) > sizeof(data))
		vcos_log_error("i2c wr reg=%04x: len=%d is too big!\n",
			  reg, 2 + n);

	msg.addr = I2C_ADDR;
	msg.buf = data;
	msg.len = 2 + n;
	msg.flags = 0;

	data[0] = reg >> 8;
	data[1] = reg & 0xff;

	for (i = 0; i < n; i++)
		data[2 + i] = values[i];

	msgset.msgs = &msg;
	msgset.nmsgs = 1;

	err = ioctl(fd, I2C_RDWR, &msgset);
	if (err != 1) {
		vcos_log_error("%s: writing register 0x%x from 0x%x failed\n",
				__func__, reg, I2C_ADDR);
		return;
	}
}

static u8 i2c_rd8(int fd, u16 reg)
{
	u8 val;

	i2c_rd(fd, reg, &val, 1);

	return val;
}

static void i2c_wr8(int fd, u16 reg, u8 val)
{
	i2c_wr(fd, reg, &val, 1);
}

static void i2c_wr8_and_or(int fd, u16 reg,
		u8 mask, u8 val)
{
	i2c_wr8(fd, reg, (i2c_rd8(fd, reg) & mask) | val);
}

static u16 i2c_rd16(int fd, u16 reg)
{
	u16 val;

	i2c_rd(fd, reg, (u8 *)&val, 2);

	return val;
}

static void i2c_wr16(int fd, u16 reg, u16 val)
{
	i2c_wr(fd, reg, (u8 *)&val, 2);
}

static void i2c_wr16_and_or(int fd, u16 reg, u16 mask, u16 val)
{
	i2c_wr16(fd, reg, (i2c_rd16(fd, reg) & mask) | val);
}

static u32 i2c_rd32(int fd, u16 reg)
{
	u32 val;

	i2c_rd(fd, reg, (u8 *)&val, 4);

	return val;
}

static void i2c_wr32(int fd, u16 reg, u32 val)
{
	i2c_wr(fd, reg, (u8 *)&val, 4);
}

#define CC_RGB_PASSTHROUGH      1
#define CC_RGB_YUV422           2
#define CC_RGB_YUV444           3
#define CC_YUV444_YUV422        4
#define CC_YUV422_YUV444        5
#define COLOR_CONVERSION CC_RGB_PASSTHROUGH

#if COLOR_CONVERSION == CC_RGB_PASSTHROUGH
   // RGB through mode
   #define r8576  0x00 // 0000 0000 -- RGB full
   #define r8573  0x00 // 00000000 -- RGB through
   #define r8574  0x00
   #define r0004  0x0e24 // 0000 1110 0010 0111
#elif COLOR_CONVERSION == CC_RGB_YUV422
   #define r8574  0x08
   #define r8573  /* 11000001 */ 0xC1
   #define r8576  0x60
   #define r0004  0x0ee4
#elif COLOR_CONVERSION == CC_RGB_YUV444
   #define r8574  0x08
   #define r8573  /* 00000001 */ 0x01
   #define r8576  0x60
   #define r0004  0x0e24
#elif COLOR_CONVERSION == CC_YUV444_YUV422
   #define r8574  0x08
   #define r8573  /* 00000001 */ 0x80
   #define r8576  0x00
   #define r0004  0x0ee4
#elif COLOR_CONVERSION == CC_YUV422_YUV444
   #define r8574  0x08
   #define r8573  /* 00000001 */ 0x00
   #define r8576  0x00
   #define r0004  0x0e24
#endif

struct cmds_t {
	uint16_t addr;
	uint32_t value;
	int num_bytes;
};
struct cmds_t cmds[] = 
{
   // Turn off power and put in reset
   // handle.first_boot = VC_FALSE;
	{0x0002, 0x0F00, 2},		// Assert Reset, [0] = 0: Exit Sleep, wait

    {0x0000, 1, 0xFFFF}, 		// V054 requires us to wait 1ms for PLL to lock
	{0x0002, 0x0000, 2},		// Release Reset, Exit Sleep
	{0x0006, 0x0008, 2},		// FIFO level
	{0x0008, 0x005f, 2},		// Audio buffer level -- 96 bytes = 0x5F + 1
	{0x0014, 0xFFFF, 2},	    // Clear HDMI Rx, CSI Tx and System Interrupt Status
	{0x0016, 0x051f, 2},		// Enable HDMI-Rx Interrupt (bit 9), Sys interrupt (bit 5). Disable others. 11-15, 6-7 reserved
	{0x0020, 0x8111, 2},	       // PRD[15:12], FBD[8:0]
	{0x0022, 0x0213, 2},		// FRS[11:10], LBWS[9:8]= 2, Clock Enable[4] = 1,  ResetB[1] = 1,  PLL En[0]
	{0x0004, r0004,  2},		// PwrIso[15], 422 output, send infoframe
	{0x0140, 0x0,    4},		//Enable CSI-2 Clock lane
	{0x0144, 0x0,    4},		//Enable CSI-2 Data lane 0
	{0x0148, 0x0,    4},		//Enable CSI-2 Data lane 1
	{0x014C, 0x1,    4},		//Disable CSI-2 Data lane 2
	{0x0150, 0x1,    4},		//Disable CSI-2 Data lane 3

	{0x0210, 0x00002988, 4},  	// LP11 = 100 us for D-PHY Rx Init
	{0x0214, 0x00000005, 4},	// LP Tx Count[10:0]
	{0x0218, 0x00001d04, 4},	// TxClk_Zero[15:8]
	{0x021C, 0x00000002, 4},   // TClk_Trail =
	{0x0220, 0x00000504, 4},   // HS_Zero[14:8] =
	{0x0224, 0x00004600, 4},   // TWAKEUP Counter[15:0]
	{0x0228, 0x0000000A, 4},   // TxCLk_PostCnt[10:0]
	{0x022C, 0x00000004, 4},   // THS_Trail =
	{0x0234, 0x0000001F, 4},   // Enable Voltage Regulator for CSI (4 Data + Clk) Lanes
	{0x0204, 0x00000001, 4},   // Start PPI

	{0x0518, 0x00000001, 4},   // Start CSI-2 Tx
	{0x0500, 0xA3008082, 4},   // SetBit[31:29]
	   //
	   //    1010 0011 0000 0000    1000 0000 1010 0010

	{0x8502, 0x01, 1},		// Enable HPD DDC Power Interrupt
	{0x8512, 0xFE, 1},		// Disable HPD DDC Power Interrupt Mask
	{0x8513, (uint8_t) ~0x20, 1},		// Receive interrupts for video format change (bit 5)
	{0x8515, (uint8_t) ~0x02, 1},		// Receive interrupts for format change (bit 1)

	{0x8531, 0x01, 1},		// [1] = 1: RefClk 42 MHz, [0] = 1, DDC5V Auto detection
	{0x8540, 0x0A8C, 2},	// SysClk Freq count with RefClk = 27 MHz (0x1068 for 42 MHz, default)
	{0x8630, 0x00041eb0, 4},   // Audio FS Lock Detect Control [19:0]: 041EB0 for 27 MHz, 0668A0 for 42 MHz (default)
	{0x8670, 0x01, 1},		   // SysClk 27/42 MHz: 00:= 42 MHz

	{0x8532, 0x80, 1},		// PHY_AUTO_RST[7:4] = 1600 us, PHY_Range_Mode = 12.5 us
	{0x8536, 0x40, 1},		// [7:4] Ibias: TBD, [3:0] BGR_CNT: Default
	{0x853F, 0x0A, 1},		// [3:0] = 0x0a: PHY TMDS CLK line squelch level: 50 uA

	{0x8543, 0x32, 1},		// [5:4] = 2'b11: 5V Comp, [1:0] = 10, DDC 5V active detect delay setting: 100 ms
	{0x8544, 0x10, 1},		// DDC5V detection interlock -- enable
	{0x8545, 0x31, 1},		//  [5:4] = 2'b11: Audio PLL charge pump setting to Normal, [0] = 1: DAC/PLL Power On
	{0x8546, 0x2D, 1},		// [7:0] = 0x2D: AVMUTE automatic clear setting (when in MUTE and no AVMUTE CMD received) 45 * 100 ms

	{0x85C7, 0x01, 1},		// [6:4] EDID_SPEED: 100 KHz, [1:0] EDID_MODE: Internal EDID-RAM & DDC2B mode
	{0x85CB, 0x01, 1},		// EDID Data size read from EEPROM EDID_LEN[10:8] = 0x01, 256-Byte
};
#define NUM_REGS_CMD (sizeof(cmds)/sizeof(cmds[0]))

//#define TOSHH2C_720P

#ifdef TOSHH2C_720P
// 720P with quantization flag
static unsigned char TOSHH2C_DEFAULT_EDID[] = //{
   "\x00\xff\xff\xff\xff\xff\xff\x00\x52\x62\x88\x88\x00\x88\x88\x88"
   "\x1c\x15\x01\x03\x80\x00\x00\x78\x0a\xEE\x91\xA3\x54\x4C\x99\x26"
   "\x0F\x50\x54\x00\x00\x00\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01"
   "\x01\x01\x01\x01\x01\x01\x01\x1d\x00\x72\x51\xd0\x1e\x20\x6e\x28"
   "\x55\x00\xc4\x8e\x21\x00\x00\x1e\x8c\x0a\xd0\x8a\x20\xe0\x2d\x10"
   "\x10\x3e\x96\x00\x13\x8e\x21\x00\x00\x1e\x00\x00\x00\xfc\x00\x54"
   "\x6f\x73\x68\x69\x62\x61\x2d\x48\x32\x43\x0a\x20\x00\x00\x00\xFD"
   "\x00\x3b\x3d\x0f\x2e\x0f\x1e\x0a\x20\x20\x20\x20\x20\x20\x01\x67"
   "\x02\x03\x1A\x42\x47\x84\x13\x03\x02\x07\x06\x01\x23\x09\x07\x07"
   "\x66\x03\x0c\x00\x30\x00\x80\xE3\x00\x7F\x8c\x0a\xd0\x8a\x20\xe0"
   "\x2d\x10\x10\x3e\x96\x00\xc4\x8e\x21\x00\x00\x18\x8c\x0a\xd0\x8a"
   "\x20\xe0\x2d\x10\x10\x3e\x96\x00\x13\x8e\x21\x00\x00\x18\x8c\x0a"
   "\xa0\x14\x51\xf0\x16\x00\x26\x7c\x43\x00\x13\x8e\x21\x00\x00\x98"
   "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
   "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
   "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x20";
#else

// 480P EDID
static unsigned char TOSHH2C_DEFAULT_EDID[] = //{
"\x00\xFF\xFF\xFF\xFF\xFF\xFF\x00\x52\x62\x88\x88\x00\x88\x88\x88"
"\x1C\x15\x01\x03\x80\x00\x00\x78\x0A\xEE\x91\xA3\x54\x4C\x99\x26"
"\x0F\x50\x54\x00\x00\x00\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01"
"\x01\x01\x01\x01\x01\x01\x8C\x0A\xD0\x8A\x20\xE0\x2D\x10\x10\x3E"
"\x96\x00\x13\x8E\x21\x00\x00\x1E\x00\x00\x00\xFC\x00\x54\x6F\x73"
"\x68\x69\x62\x61\x2D\x48\x32\x43\x0A\x20\x00\x00\x00\xFD\x00\x3B"
"\x3D\x0F\x2E\x0F\x00\x0A\x20\x20\x20\x20\x20\x20\x00\x00\x00\x10"
"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x01\xC8";

#endif



struct cmds_t cmds2[] = 
{
	/* HDMI specification requires HPD to be pulsed low for 100ms when EDID changed */
	{0x8544, 0x01, 1},		// DDC5V detection interlock -- disable
	{0x8544, 0x00, 1},		// DDC5V detection interlock -- pulse low

	{0x0000, 100, 0xFFFF},	// sleep
	{0x8544, 0x10, 1},		// DDC5V detection interlock -- enable
	 
	{0x85D1, 0x01, 1},		   // Key loading command
	{0x8560, 0x24, 1},		   // KSV Auto Clear Mode
	{0x8563, 0x11, 1},		   // EESS_Err auto-unAuth
	{0x8564, 0x0F, 1},		// DI_Err (Data Island Error) auto-unAuth

	   // RGB888 to YUV422 conversion (must)
	{0x8574, r8574, 1},		
	{0x8573, r8573, 1},		   // OUT YUV444[7]
	   // 1010 0001
	{0x8576, r8576, 1},		   // [7:5] = YCbCr601 Limited ? 3'b011 : 3'b101 (YCbCr 709 Limited)

	{0x8600, 0x00, 1},		// Forced Mute Off, Set Auto Mute On
	{0x8602, 0xF3, 1},		// AUTO Mute (AB_sel, PCM/NLPCM_chg, FS_chg, PX_chg, PX_off, DVI)
	{0x8603, 0x02, 1},		   // AUTO Mute (AVMUTE)
	{0x8604, 0x0C, 1},		// AUTO Play (mute-->BufInit-->play)
	{0x8606, 0x05, 1},		   // BufInit start time = 0.5sec
	{0x8607, 0x00, 1},		   // Disable mute
	{0x8620, 0x00, 1},		// [5] = 0: LPCM/NLPCMinformation extraction from Cbit
	{0x8640, 0x01, 1},		   // CTS adjustment=ON
	{0x8641, 0x65, 1},		// Adjustment level 1=1000ppm, Adjustment level 2=2000ppm
	{0x8642, 0x07, 1},		   // Adjustment level 3=4000ppm
	{0x8652, 0x02, 1},		// Data Output Format: [6:4] = 0, 16-bit, [1:0] = 2, I2S Format
	{0x8665, 0x10, 1},		// [7:4] 128 Fs Clock divider  Delay 1 * 0.1 s, [0] = 0: 44.1/48 KHz Auto switch setting

	{0x8709, 0xFF, 1},		// ""FF"": Updated secondary Pkts even if there are errors received
	{0x870B, 0x2C, 1},		// [7:4]: ACP packet Intervals before interrupt, [3:0] AVI packet Intervals, [] * 80 ms
	{0x870C, 0x53, 1},		// [6:0]: PKT receive interrupt is detected,storage register automatic clear, video signal with RGB and no repeat are set
	{0x870D, 0x01, 1},		// InFo Pkt: [7]: Correctable Error is included, [6:0] # of Errors before assert interrupt
	{0x870E, 0x30, 1},		// [7:4]: VS packet Intervals before interrupt, [3:0] SPD packet Intervals, [] * 80 ms
	{0x9007, 0x10, 1},		// [5:0]  Auto clear by not receiving 16V GBD
	{0x854A, 0x01, 1},		// HDMIRx Initialization Completed, THIS MUST BE SET AT THE LAST!

	{0x0004, r0004 | 0x03, 2},        // Enable tx buffers
};
#define NUM_REGS_CMD2 (sizeof(cmds2)/sizeof(cmds2[0]))

struct cmds_t stop_cmds[] =
{
{0x8544, 0x01, 1},   // regain manual control
{0x8544, 0x00, 1},   // disable HPD
//{0x0014, 0x8000, 0x12}, //power island enable (or'ed with register)
//{0x0000, 10, 0xFFFF},	//Sleep 10ms
//{0x0002, 0x0001, 2}, // put to sleep
};
#define NUM_REGS_STOP (sizeof(stop_cmds)/sizeof(stop_cmds[0]))

void start_camera_streaming(void)
{
   int fd, i;

#ifdef DO_PIN_CONFIG
   wiringPiSetupGpio();
   pinModeAlt(0, INPUT);
   pinModeAlt(1, INPUT);
   //Toggle these pin modes to ensure they get changed.
   pinModeAlt(28, INPUT);
   pinModeAlt(28, 4);	//Alt0
   pinModeAlt(29, INPUT);
   pinModeAlt(29, 4);	//Alt0
   digitalWrite(41, 1); //Shutdown pin on B+ and Pi2
   digitalWrite(32, 1); //LED pin on B+ and Pi2
#endif
   fd = open("/dev/i2c-1", O_RDWR);
   if (!fd)
   {
      vcos_log_error("Couldn't open I2C device");
      return;
   }
   if(ioctl(fd, I2C_SLAVE, 0x0F) < 0)
   {
      vcos_log_error("Failed to set I2C address");
      return;
   }
   for (i=0; i<NUM_REGS_CMD; i++)
   {
	   switch(cmds[i].num_bytes)
	   {
			case 1:
				i2c_wr8(fd, cmds[i].addr, (uint8_t)cmds[i].value);
				break;
			case 2:
				i2c_wr16(fd, cmds[i].addr, (uint16_t)cmds[i].value);
				break;
			case 4:
				i2c_wr32(fd, cmds[i].addr, cmds[i].value);
				break;
			case 0x11:
				i2c_wr8_and_or(fd, cmds[i].addr, 0xFF, (uint8_t)cmds[i].value);
				break;
			case 0x12:
				i2c_wr16_and_or(fd, cmds[i].addr, 0xFFFF, (uint16_t)cmds[i].value);
				break;
			case 0xFFFF:
				vcos_sleep(cmds[i].value);
				break;
			default:
				vcos_log_error("%u bytes specified in entry %d - not supported", cmds[i].num_bytes, i);
				break;
		}
   }
      // default is 256 bytes, 256 / 16
   // write in groups of 16
   {
      int i, j;
      unsigned char checksum = 0;
      for (i = 0; i < sizeof(TOSHH2C_DEFAULT_EDID); i += 16)
      {
         for (j = 0; j < 15; j++)
         {
            checksum -= TOSHH2C_DEFAULT_EDID[i + j];
         }
         // if checksum byte
         if (i == (7 * 16) || i == (15 * 16))
         {
            TOSHH2C_DEFAULT_EDID[i + 15] = checksum;
            checksum = 0;
         }
         else
         {
            checksum -= TOSHH2C_DEFAULT_EDID[i + 15];
         }
         i2c_wr(fd, 0x8C00 + i, &TOSHH2C_DEFAULT_EDID[i], 16);
      }
   }
   for (i=0; i<NUM_REGS_CMD2; i++)
   {
	   switch(cmds2[i].num_bytes)
	   {
			case 1:
				i2c_wr8(fd, cmds2[i].addr, (uint8_t)cmds2[i].value);
				break;
			case 2:
				i2c_wr16(fd, cmds2[i].addr, (uint16_t)cmds2[i].value);
				break;
			case 4:
				i2c_wr32(fd, cmds2[i].addr, cmds2[i].value);
				break;
			case 0x11:
				i2c_wr8_and_or(fd, cmds2[i].addr, 0xFF, (uint8_t)cmds2[i].value);
				break;
			case 0x12:
				i2c_wr16_and_or(fd, cmds2[i].addr, 0xFFFF, (uint16_t)cmds2[i].value);
				break;
			case 0xFFFF:
				vcos_sleep(cmds2[i].value);
				break;
			default:
				vcos_log_error("%u bytes specified in entry %d - not supported", cmds2[i].num_bytes, i);
				break;
		}
   }

   close(fd);
}

void stop_camera_streaming(void)
{
   int fd, i;
   fd = open("/dev/i2c-1", O_RDWR);
   if (!fd)
   {
      vcos_log_error("Couldn't open I2C device");
      return;
   }
   if(ioctl(fd, I2C_SLAVE, 0x36) < 0)
   {
      vcos_log_error("Failed to set I2C address");
      return;
   }
   for (i=0; i<NUM_REGS_STOP; i++)
   {
	   switch(stop_cmds[i].num_bytes)
	   {
			case 1:
				i2c_wr8(fd, stop_cmds[i].addr, (uint8_t)stop_cmds[i].value);
				break;
			case 2:
				i2c_wr16(fd, stop_cmds[i].addr, (uint16_t)stop_cmds[i].value);
				break;
			case 4:
				i2c_wr32(fd, stop_cmds[i].addr, stop_cmds[i].value);
				break;
			case 0x11:
				i2c_wr8_and_or(fd, stop_cmds[i].addr, 0xFF, (uint8_t)stop_cmds[i].value);
				break;
			case 0x12:
				i2c_wr16_and_or(fd, stop_cmds[i].addr, 0xFFFF, (uint16_t)stop_cmds[i].value);
				break;
			case 0xFFFF:
				vcos_sleep(stop_cmds[i].value);
				break;
			default:
				vcos_log_error("%u bytes specified in entry %d - not supported", stop_cmds[i].num_bytes, i);
				break;
		}
   }
   close(fd);
#ifdef DO_PIN_CONFIG
   digitalWrite(41, 0); //Shutdown pin on B+ and Pi2
   digitalWrite(32, 0); //LED pin on B+ and Pi2
#endif
}

int running = 0;
#ifdef MANUAL_CONNECTION
static void callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
	static int count = 0;
	vcos_log_error("Buffer %p returned, filled %d, timestamp %llu, flags %04X", buffer, buffer->length, buffer->pts, buffer->flags);
	if(running)
	{
		if(!(buffer->flags&MMAL_BUFFER_HEADER_FLAG_CODECSIDEINFO) &&
                   (((count++)%15)==0))
		{
			// Save every 15th frame
			// SD card access is to slow to do much more.
			FILE *file;
			char filename[20];
			sprintf(filename, "raw%04d.raw", count);
			file = fopen(filename, "wb");
			if(file)
			{
				fwrite(buffer->data, buffer->length, 1, file);
				fclose(file);
			}
		}
		buffer->length = 0;
		mmal_port_send_buffer(port, buffer);
	}
	else
		mmal_buffer_header_release(buffer);
}
#endif

int main (void)
{
	MMAL_COMPONENT_T *rawcam, *render, *resizer;
	MMAL_STATUS_T status;
	MMAL_PORT_T *output, *input, *resizer_input, *resizer_output;
	MMAL_POOL_T *pool;
	MMAL_PARAMETER_CAMERA_RX_CONFIG_T rx_cfg = {{MMAL_PARAMETER_CAMERA_RX_CONFIG, sizeof(rx_cfg)}};
	MMAL_PARAMETER_CAMERA_RX_TIMING_T rx_timing = {{MMAL_PARAMETER_CAMERA_RX_TIMING, sizeof(rx_timing)}};

	bcm_host_init();
	vcos_log_register("RaspiRaw", VCOS_LOG_CATEGORY);

	status = mmal_component_create("vc.ril.rawcam", &rawcam);
	if(status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to create rawcam");
		return -1;
	}
	status = mmal_component_create("vc.ril.video_render", &render);
	if(status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to create render");
		return -1;
	}
	
	status = mmal_component_create("vc.ril.resize", &resizer); //vc.ril.resize
	if(status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to create resizer");
		return -1;
	}
	output = rawcam->output[0];
	resizer_input = resizer->input[0];
	resizer_output = resizer->output[0];
	input = render->input[0];

	/*  attempt at setting input port (render) to fullscreen) 
      MMAL_DISPLAYREGION_T param;
      param.hdr.id = MMAL_PARAMETER_DISPLAYREGION;
      param.hdr.size = sizeof(MMAL_DISPLAYREGION_T);

      //param.set = MMAL_DISPLAY_SET_LAYER;
      //param.layer = 2;

      param.set |= MMAL_DISPLAY_SET_ALPHA;
      param.alpha = 100;



      //param.set |= MMAL_DISPLAY_SET_FULLSCREEN;
      //param.fullscreen = 1;

	  MMAL_RECT_T previewWindow; 
      previewWindow.x = 0;
	  previewWindow.y = 0;
	  previewWindow.width = 500;
	  previewWindow.height = 500;
      param.set |= (MMAL_DISPLAY_SET_DEST_RECT | MMAL_DISPLAY_SET_FULLSCREEN);
      param.fullscreen = 0;
      param.dest_rect = previewWindow;
      
	
	vcos_log_error("attempting to set render to fullscreen");
	  status = mmal_port_parameter_set(input, &param.hdr);
	  if (status != MMAL_SUCCESS && status != MMAL_ENOSYS)
      {
         vcos_log_error("unable to set preview port parameters (%u)", status);
      }
	/* attempt at setting input port (render) to fullscreen) */
	
	// change pixel aspect ratio to stetch the height of the video, needed for 1080i and 480p -- HACK !!
	//input->format->es->video.par.num = 1;
	//input->format->es->video.par.den = 2;
	//status = mmal_port_format_commit(input);
	  if (status != MMAL_SUCCESS && status != MMAL_ENOSYS)
      {
         vcos_log_error("unable to set preview port parameters (%u)", status);
      }
	status = mmal_port_parameter_get(output, &rx_cfg.hdr);
	if(status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to get cfg");
		goto component_destroy;
	}
	rx_cfg.image_id = CSI_IMAGE_ID;
	rx_cfg.data_lanes = CSI_DATA_LANES;
	rx_cfg.unpack = UNPACK;
	rx_cfg.pack = PACK;
	rx_cfg.data_lanes = 2;
	rx_cfg.embedded_data_lines = 128;
	vcos_log_error("Set pack to %d, unpack to %d", rx_cfg.unpack, rx_cfg.pack);
	status = mmal_port_parameter_set(output, &rx_cfg.hdr);
	if(status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to set cfg");
		goto component_destroy;
	}


	vcos_log_error("Enable rawcam....");

	status = mmal_component_enable(rawcam);
	if(status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to enable");
		goto component_destroy;
	}
	status = mmal_port_parameter_set_boolean(output, MMAL_PARAMETER_ZERO_COPY, MMAL_TRUE);
	if(status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to set zero copy");
		goto component_disable;
	}

	vcos_log_error("Enable resizer....");

	status = mmal_component_enable(resizer);
	if(status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to enable");
		goto component_destroy;
	}
	status = mmal_port_parameter_set_boolean(resizer_output, MMAL_PARAMETER_ZERO_COPY, MMAL_TRUE);
	if(status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to set zero copy");
		goto component_disable;
	}





	output->format->es->video.crop.width = WIDTH;
	output->format->es->video.crop.height = HEIGHT;
	output->format->es->video.width = VCOS_ALIGN_UP(WIDTH, 32);
	output->format->es->video.height = VCOS_ALIGN_UP(HEIGHT, 16);
	output->format->encoding = ENCODING; //MMAL_ENCODING_I420; //ENCODING;
	output->format->es->video.par.num = 1;
	output->format->es->video.par.den = 2;
	vcos_log_error("output pixel aspect ratio is %d/%d", output->format->es->video.par.num, output->format->es->video.par.den);
	vcos_log_error("input pixel aspect ratio is %d/%d", input->format->es->video.par.num, input->format->es->video.par.den);
	vcos_log_error("output p_fmt_commit...");
	status = mmal_port_format_commit(output);

	if(status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed port_format_commit");
		goto component_disable;
	}

 	output->buffer_size = output->buffer_size_recommended;
 	output->buffer_num = 8; //output->buffer_num_recommended;
     vcos_log_error("buffer size is %d bytes, num %d", output->buffer_size, output->buffer_num);
	status = mmal_port_format_commit(output);
	if(status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed port_format_commit");
		goto component_disable;
	}
	// copy the format of the rawcam output to the resizer input
	mmal_format_copy(resizer_input->format,output->format);
	//resizer_input->format->encoding = MMAL_ENCODING_BGR24;
	resizer_input->buffer_num = 8;
	vcos_log_error("Setting resizer input port format to the same as the rawcam output");
	status = mmal_port_format_commit(resizer_input);
	if (status != MMAL_SUCCESS)
	{
		printf("Couldn't set resizer input port format : error %d", status);
	}
	
	// copy the resizer input to the resizer output
	mmal_format_copy(resizer_output->format,resizer_input->format);
	//resizer_output->format->es->encoding = MMAL_ENCODING_I420;
	resizer_output->format->es->video.width = 720;
	resizer_output->format->es->video.height = 480;
	resizer_output->format->es->video.crop.x = 0;
	resizer_output->format->es->video.crop.y = 0;
	resizer_output->format->es->video.crop.width = 720;
	resizer_output->format->es->video.crop.height = 480;
	vcos_log_error("Setting resizer output port format");
	status = mmal_port_format_commit(resizer_output);
	if (status != MMAL_SUCCESS)
	{
		printf("Couldn't set resizer output port format : error %d", status);
	}

	vcos_log_error("rawcam supported encodings:");
	display_supported_encodings(output);
	vcos_log_error("resizer input supported encodings:");
	display_supported_encodings(resizer_input);
	vcos_log_error("resizer output supported encodings:");
	display_supported_encodings(resizer_output);
	vcos_log_error("render supported encodings:");
	display_supported_encodings(input);

	
	//If video_render does ignore MMAL_BUFFER_HEADER_FLAG_CODECSIDEINFO (firmware hack!), can
   //use a mmal_connection
   MMAL_CONNECTION_T *connection[2] = {0};

	status = mmal_port_parameter_set_boolean(input, MMAL_PARAMETER_ZERO_COPY, MMAL_TRUE);
	if(status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to set zero copy on video_render");
		goto component_disable;
	}
	vcos_log_error("Create connections....");
	vcos_log_error("Create connection rawcam output to resizer input....");
	status =  mmal_connection_create(&connection[0], output, input, MMAL_CONNECTION_FLAG_TUNNELLING); //resizer_input
	//vcos_log_error("Create connection resizer output to render input....");
	//status =  mmal_connection_create(&connection[1], resizer_output, input, MMAL_CONNECTION_FLAG_TUNNELLING);
	
   if (status == MMAL_SUCCESS)
   {
	vcos_log_error("Enable connection[0]...");
    vcos_log_error("buffer size is %d bytes, num %d", output->buffer_size, output->buffer_num);
      status =  mmal_connection_enable(connection[0]);
      if (status != MMAL_SUCCESS)
      {
         mmal_connection_destroy(connection[0]);
		}
	//vcos_log_error("Enable connection[1]...");
    //vcos_log_error("buffer size is %d bytes, num %d", output->buffer_size, output->buffer_num);
     //status =  mmal_connection_enable(connection[1]);
      //if (status != MMAL_SUCCESS)
      //{
       //  mmal_connection_destroy(connection[1]);
	//	}
	}


	vcos_log_error("All done. Start streaming...");
	//start_camera_streaming();
	vcos_log_error("View!");

	vcos_sleep(10000);
	running = 0;

	vcos_log_error("Stopping streaming...");
	//stop_camera_streaming();

port_disable:

	mmal_connection_disable(connection[0]);
	mmal_connection_destroy(connection[0]);

component_disable:
	status = mmal_component_disable(rawcam);
	if(status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to disable rawcam");
	}
	status = mmal_component_disable(render);
	if(status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to disable render");
	}
component_destroy:
	mmal_component_destroy(rawcam);
	mmal_component_destroy(render);
	return 0;
}







	#define MAX_ENCODINGS_NUM 20
	typedef struct {
		MMAL_PARAMETER_HEADER_T header;
		MMAL_FOURCC_T encodings[MAX_ENCODINGS_NUM];
	} MMAL_SUPPORTED_ENCODINGS_T;

void display_supported_encodings(MMAL_PORT_T *port)
{
	


	
   MMAL_SUPPORTED_ENCODINGS_T sup_encodings = {{MMAL_PARAMETER_SUPPORTED_ENCODINGS, sizeof(sup_encodings)}, {0}};
   if (mmal_port_parameter_get(port, &sup_encodings.header) == MMAL_SUCCESS)
   {
      int i;
      int num_encodings = (sup_encodings.header.size - sizeof(sup_encodings.header)) /
          sizeof(sup_encodings.encodings[0]);
      for (i=0; i<num_encodings; i++)
      {
         switch (sup_encodings.encodings[i])
         {
			case MMAL_ENCODING_I420:
				vcos_log_error("MMAL_ENCODING_I420");
			break;
			case MMAL_ENCODING_I420_SLICE:
				vcos_log_error("MMAL_ENCODING_I420_SLICE");
			break;
			case MMAL_ENCODING_YV12:
				vcos_log_error("MMAL_ENCODING_YV12");
			break;
			case MMAL_ENCODING_I422:
				vcos_log_error("MMAL_ENCODING_I422");
			break;
			case MMAL_ENCODING_I422_SLICE:
				vcos_log_error("MMAL_ENCODING_I422_SLICE");
			break;
			case MMAL_ENCODING_YUYV:
				vcos_log_error("MMAL_ENCODING_YUYV");
			break;
			case MMAL_ENCODING_YVYU:
				vcos_log_error("MMAL_ENCODING_YVYU");
			break;
			case MMAL_ENCODING_UYVY:
				vcos_log_error("MMAL_ENCODING_UYVY");
			break;
			case MMAL_ENCODING_VYUY:
				vcos_log_error("MMAL_ENCODING_VYUY");
			break;
			case MMAL_ENCODING_NV12:
				vcos_log_error("MMAL_ENCODING_NV12");
			break;
			case MMAL_ENCODING_NV21:
				vcos_log_error("MMAL_ENCODING_NV21");
			break;
			case MMAL_ENCODING_ARGB:
				vcos_log_error("MMAL_ENCODING_ARGB");
			break;
			case MMAL_ENCODING_RGBA:
				vcos_log_error("MMAL_ENCODING_RGBA");
			break;
			case MMAL_ENCODING_ABGR:
				vcos_log_error("MMAL_ENCODING_ABGR");
			break;
			case MMAL_ENCODING_BGRA:
				vcos_log_error("MMAL_ENCODING_BGRA");
			break;
			case MMAL_ENCODING_RGB16:
				vcos_log_error("MMAL_ENCODING_RGB16");
			break;
			case MMAL_ENCODING_RGB24:
				vcos_log_error("MMAL_ENCODING_RGB24");
			break;
			case MMAL_ENCODING_RGB32:
				vcos_log_error("MMAL_ENCODING_RGB32");
			break;
			case MMAL_ENCODING_BGR16:
				vcos_log_error("MMAL_ENCODING_BGR16");
			break;
			case MMAL_ENCODING_BGR24:
				vcos_log_error("MMAL_ENCODING_BGR24");
			break;
			case MMAL_ENCODING_BGR32:
				vcos_log_error("MMAL_ENCODING_BGR32");
			break;
		 }
      }
   }
   else
   {
	   vcos_log_error("Failed to get supported encodings");
   }
}
