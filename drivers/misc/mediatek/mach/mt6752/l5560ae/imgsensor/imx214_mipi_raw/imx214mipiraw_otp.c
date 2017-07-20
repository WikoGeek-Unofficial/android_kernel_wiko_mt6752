// add by Suny , for IMX214  otp
// ++

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h> 
#include <asm/atomic.h>
#include <linux/xlog.h>
//#include <asm/system.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "imx214mipiraw_Sensor.h"

extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
//#define IMX214_write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para , 1, IMX214MIPI_WRITE_ID)
extern void IMX214_write_cmos_sensor(u16 addr, u32 para);
#define SENSORDB(fmt, arg...) printk( "[IMX214MIPIRaw] "  fmt, ##arg)
#if 0
static kal_uint16 IMX214_read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
    iReadReg((u16) addr ,(u8*)&get_byte,IMX214MIPI_WRITE_ID);
    return get_byte;
}
#else
extern kal_uint16 IMX214_read_cmos_sensor(kal_uint32 addr);
#endif

static uint32_t RG_Ratio_typical=0x12A;//141111 llf
static uint32_t BG_Ratio_typical=0x12C;//141111 llf


kal_uint16 r_current,g_current,b_current;

#define  IMX214_OTP_SUPPORT
#ifdef IMX214_OTP_SUPPORT
//read otp data by the given page
//tempbank:the first group for read
//address:the first address for read
//iBuffer:the return data buffer
//buffersize: buffer size

static kal_bool IMX214MIPI_ReadOtp(int tempbank,kal_uint16 address,kal_uint8* iBuffer,int buffersize)
{
	kal_uint16 reVal;
	kal_uint16 reg;
	int i = 0;
	int k = 0;
	while (k<buffersize)
	{
		i=0;
		IMX214_write_cmos_sensor(0x0A02, tempbank);
		IMX214_write_cmos_sensor(0x0A00, 0x01);
		msleep(100);
		reg = IMX214_read_cmos_sensor(0x0A01);
		if ((reg & 0x01) ==0x01)
		{
			SENSORDB("读OTP 成功!");
		}
		else
		{
			SENSORDB("读OTP 失败!");
			return KAL_FALSE;
		}

		while(i<64)
		{
			reVal = IMX214_read_cmos_sensor(address+i);
			*(iBuffer+k) = reVal;
			i++;
			k++;
			if (k>=buffersize)
			{
				break;
			}
		}
		tempbank++;
	}

	return KAL_TRUE;
}

static void TestAWBforIMX214(kal_uint16 RoverG_dec,kal_uint16 BoverG_dec,kal_uint16 GboverGr_dec)
{
	kal_uint16 RoverG_dec_base,BoverG_dec_base,GboverGr_dec_base;
	kal_uint16 R_test,B_test,G_test;
	kal_uint16 R_test_H3,R_test_L8,B_test_H3,B_test_L8,G_test_H3,G_test_L8;
	
	RoverG_dec_base = RG_Ratio_typical;//the typcical value
	BoverG_dec_base = BG_Ratio_typical;//the typcical value
	GboverGr_dec_base = 0;//the typcical value
	
	kal_uint32 G_test_R, G_test_B;
	
	if(BoverG_dec < BoverG_dec_base)
	{
		if (RoverG_dec < RoverG_dec_base)
		{
			G_test = 0x100;
			B_test = 0x100 * BoverG_dec_base / BoverG_dec;
			R_test = 0x100 * RoverG_dec_base / RoverG_dec;
		}
		else
		{
			R_test = 0x100;
			G_test = 0x100 * RoverG_dec / RoverG_dec_base;
			B_test = G_test * BoverG_dec_base / BoverG_dec;
		}
	}
	else
	{
		if (RoverG_dec < RoverG_dec_base)
		{
			B_test = 0x100;
			G_test = 0x100 * BoverG_dec / BoverG_dec_base;
			R_test = G_test * RoverG_dec_base / RoverG_dec;
		}
		else
		{
			G_test_B = BoverG_dec * 0x100 / BoverG_dec_base;
			G_test_R = RoverG_dec * 0x100 / RoverG_dec_base;
			if(G_test_B > G_test_R )
			{
				B_test = 0x100;
				G_test = G_test_B;
				R_test = G_test_B * RoverG_dec_base / RoverG_dec;
			}
			else
			{
				R_test = 0x100;
				G_test = G_test_R;
				B_test = G_test_R * BoverG_dec_base / BoverG_dec;
			}
		}
	}
	if(R_test < 0x100)
	{
		R_test = 0x100;
	}
	if(G_test < 0x100)
	{
		G_test = 0x100;
	}
	if(B_test < 0x100)
	{
		B_test = 0x100;
	}
	R_test_H3 =( R_test>>8)&0x0F;
	R_test_L8 = R_test &0xFF;
	B_test_H3 = (B_test>>8)&0x0F;
	B_test_L8 = B_test &0xFF;
	G_test_H3 = (G_test>>8)&0x0F;
	G_test_L8 = G_test &0xFF;
	
	//reset the digital gain
	IMX214_write_cmos_sensor(0x020E, G_test_H3);
	IMX214_write_cmos_sensor(0x020F,G_test_L8);
	IMX214_write_cmos_sensor(0x0210, R_test_H3);
	IMX214_write_cmos_sensor(0x0211,R_test_L8);
	IMX214_write_cmos_sensor(0x0212, B_test_H3);
	IMX214_write_cmos_sensor(0x0213,B_test_L8);
	IMX214_write_cmos_sensor(0x0214, G_test_H3);	//zl:why rewrite?
	IMX214_write_cmos_sensor(0x0215,G_test_L8);
	
	SENSORDB("R_test=0x%x,G_test=0x%x,B_test=0x%x",R_test,G_test,B_test);
}

//read module information and awb data
static kal_bool IMX214MIPI_ReadAWBFromOtp()
{
	kal_uint16 awbGroupbank[] = {0x02,0x01,0x00};//the three fist group for AWB data
	kal_uint16 address = 0x0A04;
	kal_uint16 MID = 0x01;
	kal_uint16 r,g,b;
	int i;
	kal_uint8 Temp[42]={0};
	kal_uint16 temp3;
	int index = -1;
	
	//1.check valid group
	for(i=0;i<3;i++)
	{
		//select OTP page address for read
		IMX214_write_cmos_sensor(0x0A02,awbGroupbank[i]);//select page
		//turn on OTP read mode
		IMX214_write_cmos_sensor(0x0A00,0x01);//read mode:0x0A00 = 0x01 
		//check status(bit0:0x01 read ready)
		temp3 = IMX214_read_cmos_sensor(0x0A01);//Status check : 0x0A01 = 0x01 (bit0 1:read ready)
		if ((temp3&0x01) ==0x01)
		{
			SENSORDB("读OTP AWB 成功!\n");//显示
		}
		else
		{
			SENSORDB("读OTP AWB 失败!");
			return KAL_FALSE;
		}
		
		//0x0A04-0x0A43 are the register address area for each page
		//check program flag by read ox0A04
		temp3 = IMX214_read_cmos_sensor(0x0A04);
		//program flag is not zero,the current group is valid
		if (temp3==1)
		{
			index = i;
			break;
		}
	}
	if (index == -1)
	{
		SENSORDB("OTP 中没有发现AWB 数据。");
		return KAL_FALSE;
	}
	
	//2.read data from the valid group
	int tempbank = awbGroupbank[index];
	if(KAL_FALSE == IMX214MIPI_ReadOtp(tempbank,address,Temp,42))
	{
		SENSORDB("读取Otp 数据时，I2C 通信出现错误。");
		return KAL_FALSE;
	}
	
	//3.check sum
	kal_uint16 sum=0;
	for(i=2;i<42;i++)
	{
		sum = sum + Temp[i];
	}
	if (Temp[1] != (sum%255)+1)      
	{
		SENSORDB("OTP 中发现AWB 数据有误。");
		//printk("xxxxcheck sum value:Temp[1] = %d,(sum\%255) = %d\n",Temp[1],sum%255);  
		return KAL_FALSE;
	}

	//printk("check sum value:Temp[1] = %d,(sum\%255) = %d\n",Temp[1],sum%255);  
	
	//4.check module information
	if(Temp[6]!=MID)
	{
		SENSORDB("此模组非Sunny 模组");
		return KAL_FALSE;
	}
	
	//4.get awb data
	r = (Temp[15]<<8)|Temp[16];
	b = (Temp[17]<<8)|Temp[18];
	g = (Temp[19]<<8)|Temp[20];
	
	SENSORDB("mid=0x%x",Temp[6]);
	SENSORDB("r=0x%x,g=0x%x,b=0x%x",r,g,b);

    r_current=r;
    g_current=g;
    b_current=b;
   
	
	//5.set new digital gain value
	//TestAWBforIMX214(r,b,g);
	return KAL_TRUE;
}


//read AF data
static kal_bool IMX214MIPI_ReadAFFromOtp()
{
	kal_uint16 AFGroupbank[] = {0x0B,0x0A,0x09};//the three fist group for AWB data
	kal_uint16 address=0x0A04;
	int i;
	kal_uint8 Temp[32]={0};
	kal_uint16 temp3,temp4;
	int index = -1;
	
	//1.check valid group
	for(i=0;i<3;i++)
	{
		//select OTP page address for read
		IMX214_write_cmos_sensor(0x0A02,AFGroupbank[i]);//select page
		//turn on OTP read mode
		IMX214_write_cmos_sensor(0x0A00,0x01);//read mode:0x0A00 = 0x01 
		//check status(bit0:0x01 read ready)
		temp3 = IMX214_read_cmos_sensor(0x0A01);//Status check : 0x0A01 = 0x01 (bit0 1:read ready)
		if ((temp3&0x01) ==0x01)
		{
			SENSORDB("读OTP AF 成功!\n");//显示
		}
		else
		{
			SENSORDB("读OTP AF 失败!");
			return KAL_FALSE;
		}
		
		//0x0A04-0x0A43 are the register address area for each page
		//check program flag by read ox0A04
		temp3 = IMX214_read_cmos_sensor(0x0A04);
		temp4 = IMX214_read_cmos_sensor(0x0A14);
		//program flag is not zero,the current group is valid
		if (temp3==1&&temp4==1)
		{
			index = i;
			break;
		}
	}
	if (index == -1)
	{
		SENSORDB("OTP 中没有发现AF 数据。");
		return KAL_FALSE;
	}
	
	//2.read AF  Macro data from the valid group
	int tempbank = AFGroupbank[index];
	if(KAL_FALSE == IMX214MIPI_ReadOtp(tempbank,address,Temp,32))
	{
		SENSORDB("读取Otp 数据时，I2C 通信出现错误。");
		return KAL_FALSE;
	}
	
	//3.check sum of Macro
	kal_uint16 sum=0;
	for(i=2;i<7;i++)
	{
		sum = sum + Temp[i];
	}
	if (Temp[1] != (sum%255)+1)
	{
		SENSORDB("OTP 中发现AF Macro  数据有误。");
		return KAL_FALSE;
	}
	
	//4.check sum of Infinity
	kal_uint16 sum1=0;
	for(i=18;i<23;i++)
	{
		sum1 = sum1 + Temp[i];
	}
	if (Temp[17] != (sum1%255)+1)
	{
		SENSORDB("OTP 中发现AF Infinity  数据有误。");
		return KAL_FALSE;
	}

	return KAL_TRUE;
}
//the main functiion to call
#if 0
kal_bool OnReadOtpIMX214(void)
{

	
	if (KAL_FALSE == IMX214MIPI_ReadAWBFromOtp())
	{
		return KAL_FALSE;
	}
	if(KAL_FALSE==IMX214MIPI_ReadAFFromOtp())
	{
		return KAL_FALSE;
	}
	return KAL_TRUE;
}
#endif

kal_bool IMX214_check_OTP_MID_sunny(void)
{
	if (KAL_FALSE == IMX214MIPI_ReadAWBFromOtp())
	{
		return KAL_FALSE;
	}
    else
        return KAL_TRUE;

}

kal_bool IMX214_check_OTP_AF(void)
{
	if (KAL_FALSE == IMX214MIPI_ReadAFFromOtp())
	{
		return KAL_FALSE;
	}
    else
        return KAL_TRUE;

}

void IMX214_OTP_apply_awb(void)
{
	TestAWBforIMX214(r_current,b_current,g_current);
    
}

#endif
