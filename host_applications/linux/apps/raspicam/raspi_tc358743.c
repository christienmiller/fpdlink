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

#define WIDTH 1280
#define HEIGHT 720
#define ENCODING MMAL_ENCODING_BGR24 //MMAL_ENCODING_YUYV

#define UNPACK MMAL_CAMERA_RX_CONFIG_UNPACK_NONE
#define PACK MMAL_CAMERA_RX_CONFIG_PACK_NONE

#define I2C_ADDR 0x0c //0x2c // DS90DU940 I2C ADDRESS

#define CSI_ADDR 0x6c // CSI PORT ADDRESS
#define CSI_DATA 0x6d // CSI DATA ADDRESS
#define CSI_EN_PORT0 0x13 // CSI_PORT0 OFFSET
#define CSI_ENABLED 0x3f
#define CSI_DISABLE 0x3e

#define CSI_IMAGE_ID 0x00 //0x24
#define CSI_DATA_LANES 2

	MMAL_COMPONENT_T *rawcam, *render;
	MMAL_STATUS_T status;
	MMAL_PORT_T *output, *input;
	MMAL_POOL_T *pool;
	MMAL_PARAMETER_CAMERA_RX_CONFIG_T rx_cfg = {{MMAL_PARAMETER_CAMERA_RX_CONFIG, sizeof(rx_cfg)}};
	MMAL_PARAMETER_CAMERA_RX_TIMING_T rx_timing = {{MMAL_PARAMETER_CAMERA_RX_TIMING, sizeof(rx_timing)}};
	MMAL_CONNECTION_T *connection;

static void i2c_rd(int fd, uint8_t reg, uint8_t *values, uint32_t n)
{
	int err;
	uint8_t buf[2] = { reg >> 8, reg & 0xff };
	struct i2c_rdwr_ioctl_data msgset;
	struct i2c_msg msgs[2] = {
		{
			.addr = I2C_ADDR,
			.flags = 0,
			.len = 1,
			.buf = &reg,//buf, // force 8 bit registers not 16
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

static void i2c_wr(int fd, uint8_t reg, uint8_t *value, uint32_t n)
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
	msg.len = 1 + n;
	msg.flags = 0;

	data[0] = reg;  // force 8 bit registers not 16
	data[1] = *value;

	msgset.msgs = &msg;
	msgset.nmsgs = 1;

	err = ioctl(fd, I2C_RDWR, &msgset);
	if (err != 1) {
		vcos_log_error("%s: writing register 0x%x from 0x%x failed\n",
				__func__, reg, I2C_ADDR);
		return;
	}
}



static u8 i2c_rd8(int fd, u8 reg)
{
	u8 val;

	i2c_rd(fd, reg, &val, 1);

	return val;
}

static void i2c_wr8(int fd, u8 reg, u8 val)
{
	i2c_wr(fd, reg, &val, 1);
}

static void i2c_wr8_and_or(int fd, u8 reg,
		u8 mask, u8 val)
{
	i2c_wr8(fd, reg, (i2c_rd8(fd, reg) & mask) | val);
}

static u16 i2c_rd16(int fd, u8 reg)
{
	u16 val;

	i2c_rd(fd, reg, (u8 *)&val, 2);

	return val;
}

static void i2c_wr16(int fd, u8 reg, u16 val)
{
	i2c_wr(fd, reg, (u8 *)&val, 2);
}

static void i2c_wr16_and_or(int fd, u8 reg, u16 mask, u16 val)
{
	i2c_wr16(fd, reg, (i2c_rd16(fd, reg) & mask) | val);
}

static u32 i2c_rd32(int fd, u8 reg)
{
	u32 val;

	i2c_rd(fd, reg, (u8 *)&val, 4);

	return val;
}

static void i2c_wr32(int fd, u8 reg, u32 val)
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


uint8_t get_hdmi_freq(void)
{
   int fd;
   uint8_t freq;
   
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
   
   // enable csi2 video
   freq = i2c_rd8(fd, 0x5f); // find HDMI Frequency
   
   vcos_log_error("HDMI Frequecy is %d MHz", freq);
   
   close(fd);
   return freq;

}

uint16_t get_hdmi_height(void)
{
   int fd;
   uint8_t height_lsb;
   uint16_t height_msb;
   uint16_t height;
   
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
   
   i2c_wr8(fd, 0x49, 0x68); // select address LSB
   i2c_wr8(fd, 0x4a, 0x01); // select address MSB 
   i2c_wr8(fd, 0x48, 0x03); // start APB read

   height_lsb = i2c_rd8(fd, 0x4b); // find HDMI width lsb
   height_msb = i2c_rd8(fd, 0x4c); // find HDMI width msb
   height = (height_msb << 8) | height_lsb;

   vcos_log_error("HDMI height is %d pixels", height);
   
   close(fd);
   return height;

}

void create_mmal_connection(int width, int height)
{
	int i;

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
		vcos_log_error("Failed to create rawcam");
		return -1;
	}
	output = rawcam->output[0];
	input = render->input[0];

	status = mmal_port_parameter_get(output, &rx_cfg.hdr);
	if(status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to get cfg");
		mmal_component_destroy(rawcam);
		mmal_component_destroy(render);	
	
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
		mmal_component_destroy(rawcam);
		mmal_component_destroy(render);	
	
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
	}

	vcos_log_error("Enable rawcam....");

	status = mmal_component_enable(rawcam);
	if(status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to enable");
		mmal_component_destroy(rawcam);
		mmal_component_destroy(render);	
	
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
	}
	status = mmal_port_parameter_set_boolean(output, MMAL_PARAMETER_ZERO_COPY, MMAL_TRUE);
	if(status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to set zero copy");
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
	}

	output->format->es->video.crop.width = width;
	output->format->es->video.crop.height = height;
	output->format->es->video.width = VCOS_ALIGN_UP(width, 32);
	output->format->es->video.height = VCOS_ALIGN_UP(height, 16);
	output->format->encoding = ENCODING;
	vcos_log_error("output p_fmt_commit...");
	status = mmal_port_format_commit(output);
	if(status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed port_format_commit");
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
	}

 	output->buffer_size = output->buffer_size_recommended;
 	output->buffer_num = 8; //output->buffer_num_recommended;
     vcos_log_error("buffer size is %d bytes, num %d", output->buffer_size, output->buffer_num);
	status = mmal_port_format_commit(output);
	if(status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed port_format_commit");
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
	}

	//If video_render does ignore MMAL_BUFFER_HEADER_FLAG_CODECSIDEINFO (firmware hack!), can
   //use a mmal_connection


	status = mmal_port_parameter_set_boolean(input, MMAL_PARAMETER_ZERO_COPY, MMAL_TRUE);
	if(status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to set zero copy on video_render");
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
	}
	vcos_log_error("Create connection....");
   status =  mmal_connection_create(&connection, output, input, MMAL_CONNECTION_FLAG_TUNNELLING);

   if (status == MMAL_SUCCESS)
   {
	vcos_log_error("Enable connection...");
    vcos_log_error("buffer size is %d bytes, num %d", output->buffer_size, output->buffer_num);
      status =  mmal_connection_enable(connection);
      if (status != MMAL_SUCCESS)
         mmal_connection_destroy(connection);
   }


	vcos_log_error("All done. Start streaming...");

}

void destroy_mmal_conncetion(void)
{
	vcos_log_error("Stopping streaming...");

	mmal_connection_disable(connection);
	mmal_connection_destroy(connection);

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

	mmal_component_destroy(rawcam);
	mmal_component_destroy(render);	
}

int main (void)
{

	uint8_t freq;
	uint16_t height;
	//create_mmal_connection(1280, 720);
	int running = 0;

	while (1)
	{
		freq = get_hdmi_freq();

		if (freq != 0 && running == 0) // we must have good video and we need to create the mmal connection
		{
		running = 1;
		height = get_hdmi_height();
		
			switch (height)
			{
				case 480: 
					create_mmal_connection(720, 480);
					vcos_log_error("Found good video, starting connection to mmal at 720x480");
				break;
					
				case 720:
					create_mmal_connection(1280, 720);
					vcos_log_error("Found good video, starting connection to mmal at 1280x720");
				break;

				case 1080:
					create_mmal_connection(1920, 1080);
					vcos_log_error("Found good video, starting connection to mmal at 1920x1080");
				break;
			}
		}
		else if (freq == 0 && running == 1) // we have a mmal connection but video was lost, kill the connection
		{
			vcos_log_error("We have lost video, killing connection");
			destroy_mmal_conncetion();
			running = 0; // reset the running state;
			vcos_sleep(5000); // lets wait 5 seconds to let the connection die
		}

		vcos_sleep(1000);
	}

}
