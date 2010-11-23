#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <mach/mxc_i2c.h>
#include "vs6624_regs.h"

# include "vs6624_patch.c"


int vs6624_ReadByte(struct i2c_adapter *adap, unsigned short offset, unsigned char *data)
{
	int    ret = 0;
	struct i2c_msg rmsg[2];
	int    num_msgs;
	char   buf[2];

	buf[0] = (offset & 0xFF00) >> 8;
	buf[1] = (offset & 0x00FF);

	/* Read Byte with 16-Bit Pointer */

	rmsg[0].addr = VS6624_I2C_ADDRESS;
	rmsg[0].flags = 0;          /* write */
	rmsg[0].len = 2;
	rmsg[0].buf = buf;

	rmsg[1].addr = VS6624_I2C_ADDRESS;
	rmsg[1].flags = I2C_M_RD;   /* read */ 
	rmsg[1].len = 1;
	rmsg[1].buf = data;

	num_msgs = 2;

	ret = i2c_transfer(adap, rmsg, num_msgs);

	if (ret == 2) {
		ret = 0;
	}
	else {
		printk (KERN_ERR "vs6624_ReadByte() - i2c_transfer failed.\n");		
		//Rework: add conditional debug messages here
		ret = -1;
	}
	return ret;
}

int vs6624_WriteByte(struct i2c_adapter *adap, unsigned short offset, unsigned char data)
{
	int ret;
	char   buf[2];
	struct i2c_msg wmsg[2];
	int    num_msgs;

	buf[0] = (offset & 0xFF00) >> 8;
	buf[1] = (offset & 0x00FF);

	wmsg[0].addr = VS6624_I2C_ADDRESS;
	wmsg[0].flags = 0;          /* write */
	wmsg[0].len = 2;
	wmsg[0].buf = buf;

	wmsg[1].addr = VS6624_I2C_ADDRESS;
	wmsg[1].flags = 0;   /* write */ 
	wmsg[1].len = 1;
	wmsg[1].buf = &data;

	num_msgs = 2;

	ret = i2c_transfer (adap, wmsg, num_msgs);

	if (ret == 2) {
		ret = 0;
	}
	else {
		printk (KERN_ERR "vs6624_WriteByte() - i2c_transfer failed.\n");		
		//Rework: add conditional debug messages here
		ret = -1;
	}
	return ret;
}

int  vs6624_WriteSequence(struct i2c_adapter *adap, const unsigned short array[][2], unsigned short len)
{
	u16 x;

	for (x = 0; x < len; x++)
		vs6624_WriteByte (adap, array[x][0], (unsigned char) array[x][1]);

	return 0;
}


static unsigned char get_reg(struct i2c_adapter *adap, unsigned short offset)
{
        unsigned char buf[1];

        vs6624_ReadByte (adap, offset, &buf[0]);

        return buf[0];
}

void
vs6624_set_color(struct i2c_adapter *adap, int bright, int saturation, int red, int green, int blue)
{
        switch (saturation) {
        case 150:
                vs6624_WriteByte(adap, bColourSaturation0, 0xFF & 0xFF);
                break;
        case 75:
                vs6624_WriteByte(adap, bColourSaturation0, 0xC0 & 0xFF);
                break;
        case 50:
                vs6624_WriteByte(adap, bColourSaturation0, 0x80 & 0xFF);
                break;
        case 25:
                vs6624_WriteByte(adap, bColourSaturation0, 0x40 & 0xFF);
                break;
        default:
                vs6624_WriteByte(adap, bColourSaturation0, 0x78 & 0xFF);
                break;
        }
}

void
vs6624_get_color(struct i2c_adapter *adap, int *bright, int *saturation, int *red, int *green, int *blue)
{
        *saturation = (int) get_reg( adap, bColourSaturation0);
        switch (*saturation) {
        case 0xFF:
                *saturation = 150;
                break;
        case 0xC0:
                *saturation = 75;
                break;
        case 0x80:
                *saturation = 50;
                break;
        case 0x40:
                *saturation = 25;
                break;
        default:
                *saturation = 100;
                break;
        }
}



static int vs_probe(struct i2c_adapter *adap)
{

        unsigned char buf[2];

        vs6624_ReadByte (adap, DeviceID_MSB, &buf[0]);
        vs6624_ReadByte (adap, DeviceID_LSB, &buf[1]);

        if ((((buf[0] & 0xFF) << 8) | (buf[1] & 0xFF)) == VS6624_ID) {

                printk(KERN_INFO "%s: Firmware Version %d.%d \n",
                SENSOR_NAME, get_reg(adap, bFirmwareVsnMajor), get_reg (adap, bFirmwareVsnMinor));
                printk(KERN_INFO "%s: Patch Version %d.%d \n",
                SENSOR_NAME, get_reg(adap, bPatchVsnMajor), get_reg(adap, bPatchVsnMinor));

                return 0;
        }

        printk (KERN_ERR "vs_probe: No VS6624 found.\n");
        printk (KERN_ERR "vs_probe: buf[0] = 0x%x\n", (buf[0] & 0xFF) << 8);
        printk (KERN_ERR "vs_probe: buf[1] = 0x%x\n", (buf[1] & 0xFF));
        printk (KERN_ERR "vs_probe: DeviceID = %d\n", ((buf[0] & 0xFF) << 8) | (buf[1] & 0xFF));

        return -ENODEV;
}

void vs6624_patch (struct i2c_adapter *adap)
{
        printk (KERN_ERR "vs6624_patch() - enter\n");
        msleep(100);

        vs6624_WriteByte (adap, PWR_MAN_SETUP_MODE_SELECT, 0x0);
        msleep(10);
        vs_probe (adap);

        printk (KERN_ERR "vs6624_patch() - applying patch p1\n");
        vs6624_WriteSequence (adap, patch_p1, sizeof(patch_p1) / (sizeof(u16) * 2));
        msleep(50);

        printk (KERN_ERR "vs6624_patch() - applying patch p2\n");
        vs6624_WriteSequence (adap, patch_p2, sizeof(patch_p2) / (sizeof(u16) * 2));

        vs6624_WriteByte (adap, PWR_MAN_SETUP_MODE_SELECT, 0x2);
        msleep(100);

        vs6624_WriteByte (adap, PWR_MAN_DIO_ENABLE, 0x1);
        msleep(1);

        printk(KERN_INFO "MODE: %d \n", get_reg(adap, bState));       //pjg

        vs_probe (adap);


        // Flicker correction
        vs6624_WriteByte (adap, bLightingFrequencyHz, 0x64);       // AC frequency == 100
        
        // Pan step size
        vs6624_WriteByte (adap, uwPanStepHSizeMSB0, 0x0);          // H pan step == 15
        vs6624_WriteByte (adap, uwPanStepHSizeLSB0, 0xf);
        vs6624_WriteByte (adap, uwPanStepVSizeMSB0, 0x0);          // V pan step == 15
        vs6624_WriteByte (adap, uwPanStepVSizeLSB0, 0xf);
        
        vs6624_WriteByte (adap, bSyncCodeSetup, 0x21);             // SYNC //pjg
        vs6624_WriteByte (adap, bHSyncSetup, 0xF);                 // Active lines only, Automatic //pjg
        vs6624_WriteByte (adap, bVSyncSetup, 0x7);                 // Active lines only, Automatic //pjg

        vs6624_WriteByte (adap, uwDesiredFrameRate_Num_MSB, 0x0);
        vs6624_WriteByte (adap, uwDesiredFrameRate_Num_LSB, 0x0F);    // frame rate numerator == 15 MTW
        vs6624_WriteByte (adap, bDesiredFrameRate_Den, 0x1);          // frame rate denominator == 1

        printk(KERN_INFO "MODE: %d \n", get_reg(adap, bState)); //pjg

        vs6624_WriteByte (adap, uwExternalClockFrequencyMhzNumeratorMSB, 0x0);
        vs6624_WriteByte (adap, uwExternalClockFrequencyMhzNumeratorLSB, 12);
        vs6624_WriteByte (adap, bExternalClockFrequencyMhzDenominator, 0x1);

        vs6624_WriteByte (adap, bPClkSetup, 0x85);                    // Pix Clk Mode == free run

        printk(KERN_INFO "MODE: %d \n", get_reg(adap, bState)); //pjg
        vs6624_WriteByte (adap, bUserCommand, 0x2);   // RUN

        printk(KERN_INFO "MODE: %d \n", get_reg(adap, bState)); //pjg
        printk (KERN_ERR "vs6624_patch() - writing setup patch\n");
        vs6624_WriteSequence (adap, patch_run_setup, sizeof(patch_run_setup) / (sizeof(u16) * 2));


        printk(KERN_INFO "MODE: %d \n", get_reg(adap, bState)); //pjg

        vs_probe (adap); //pjg
        printk (KERN_ERR "vs6624_patch() - exit\n");
	return;
}



static int ReadByteV(struct i2c_adapter *adap, unsigned short offset, char* name)
{
	unsigned char tmp[1];
	vs6624_ReadByte(adap, offset, &tmp[0]);
	printk (KERN_ERR "vs6624 offset = %04X,  data = %02X  %s\n", offset, tmp[0], name);
	return 0;

}


void vs6624_dump_regs(struct i2c_adapter *adap) 
{

	printk (KERN_ERR "vs6624_dump_regs() - enter\n");

	ReadByteV(adap, 0xc044,"PWR_MAN_DIO_ENABLE			    ");
	ReadByteV(adap, 0x0001,"uwDeviceId				    ");
	ReadByteV(adap, 0x0001,"DeviceID_MSB				    ");
	ReadByteV(adap, 0x0002,"DeviceID_LSB				    ");
	ReadByteV(adap, 0x0004,"bFirmwareVsnMajor			    ");
	ReadByteV(adap, 0x0006,"bFirmwareVsnMinor			    ");
	ReadByteV(adap, 0x0008,"bPatchVsnMajor			    ");	
	ReadByteV(adap, 0x000a,"bPatchVsnMinor			    ");	
	ReadByteV(adap, 0x0180,"bUserCommand				    ");
	ReadByteV(adap, 0x0186,"bManualNextState			    ");
	ReadByteV(adap, 0x0200,"bNextState				    ");
	ReadByteV(adap, 0x0202,"bState				    ");	
	ReadByteV(adap, 0x0280,"fMeteringOn				    ");
	ReadByteV(adap, 0x0282,"fExitOnStable				    ");
	ReadByteV(adap, 0x0284,"bStreamLength				    ");
	ReadByteV(adap, 0x0300,"fIsColdStart				    ");
	ReadByteV(adap, 0x0302,"bNonViewLive_ActivePipeSetupBank	    ");
	ReadByteV(adap, 0x0304,"bSnapShoot_ActivePipeSetupBank	    ");	
	ReadByteV(adap, 0x0306,"fSnapShoot_NoWaiting			    ");
	ReadByteV(adap, 0x0308,"SensorMode				    ");
	ReadByteV(adap, 0x0380,"bImageSize0				    ");
	ReadByteV(adap, 0x0383,"uwManualHSize0			    ");	
	ReadByteV(adap, 0x0383,"uwManualHSizeMSB0			    ");
	ReadByteV(adap, 0x0384,"uwManualHSizeLSB0			    ");
	ReadByteV(adap, 0x0387,"uwManualVSize0			    ");	
	ReadByteV(adap, 0x0387,"uwManualVSizeMSB0			    ");
	ReadByteV(adap, 0x0388,"uwManualVSizeLSB0			    ");
	ReadByteV(adap, 0x038b,"uwZoomStepHSize0			    ");
	ReadByteV(adap, 0x038b,"uwZoomStepHSizeMSB0			    ");
	ReadByteV(adap, 0x038c,"uwZoomStepHSizeLSB0			    ");
	ReadByteV(adap, 0x038f,"uwZoomStepVSize0			    ");
	ReadByteV(adap, 0x038f,"uwZoomStepVSizeMSB0			    ");
	ReadByteV(adap, 0x0390,"uwZoomStepVSizeLSB0			    ");
	ReadByteV(adap, 0x0392,"bZoomControl0				    ");
	ReadByteV(adap, 0x0395,"uwPanStepHSize0			    ");	
	ReadByteV(adap, 0x0395,"uwPanStepHSizeMSB0			    ");
	ReadByteV(adap, 0x0396,"uwPanStepHSizeLSB0			    ");
	ReadByteV(adap, 0x0399,"uwPanStepVSize0			    ");	
	ReadByteV(adap, 0x0399,"uwPanStepVSizeMSB0			    ");
	ReadByteV(adap, 0x039a,"uwPanStepVSizeLSB0			    ");
	ReadByteV(adap, 0x039c,"bPanControl0				    ");
	ReadByteV(adap, 0x039e,"bCropControl0				    ");
	ReadByteV(adap, 0x03a1,"uwManualCropHorizontalStart0		    ");
	ReadByteV(adap, 0x03a5,"uwManualCropHorizontalSize0		    ");
	ReadByteV(adap, 0x03a9,"uwManualCropVerticalStart0		    ");
	ReadByteV(adap, 0x03ad,"uwManualCropVerticalSize0		    ");
	ReadByteV(adap, 0x03a1,"bCropHStartMSB0			    ");	
	ReadByteV(adap, 0x03a2,"bCropHStartLSB0			    ");	
	ReadByteV(adap, 0x03a9,"bCropVStartMSB0			    ");	
	ReadByteV(adap, 0x03aa,"bCropVStartLSB0			    ");	
	ReadByteV(adap, 0x03a5,"bCropHSizeMSB0			    ");	
	ReadByteV(adap, 0x03a6,"bCropHSizeLSB0			    ");	
	ReadByteV(adap, 0x03ad,"bCropVSizeMSB0			    ");	
	ReadByteV(adap, 0x03ae,"bCropVSizeLSB0			    ");	
	ReadByteV(adap, 0x03b0,"bDataFormat0				    ");
	ReadByteV(adap, 0x03b2,"bBayerOutputAlignment0		    ");	
	ReadByteV(adap, 0x03b4,"bContrast0				    ");
	ReadByteV(adap, 0x03b6,"bColourSaturation0			    ");
	ReadByteV(adap, 0x03b8,"bGamma0				    ");	
	ReadByteV(adap, 0x03ba,"fHorizontalMirror0			    ");
	ReadByteV(adap, 0x03bc,"fVerticalFlip0			    ");	
	ReadByteV(adap, 0x03be,"bChannelID0				    ");
	ReadByteV(adap, 0x0400,"bImageSize1				    ");
	ReadByteV(adap, 0x0403,"uwManualHSize1			    ");	
	ReadByteV(adap, 0x0407,"uwManualVSize1			    ");	
	ReadByteV(adap, 0x040b,"uwZoomStepHSize1			    ");
	ReadByteV(adap, 0x040f,"uwZoomStepVSize1			    ");
	ReadByteV(adap, 0x0412,"bZoomControl1				    ");
	ReadByteV(adap, 0x0415,"uwPanStepHSize1			    ");	
	ReadByteV(adap, 0x0419,"uwPanStepVSize1			    ");	
	ReadByteV(adap, 0x041c,"bPanControl1				    ");
	ReadByteV(adap, 0x041e,"bCropControl1				    ");
	ReadByteV(adap, 0x0421,"uwManualCropHorizontalStart1		    ");
	ReadByteV(adap, 0x0425,"uwManualCropHorizontalSize1		    ");
	ReadByteV(adap, 0x0429,"uwManualCropVerticalStart1		    ");
	ReadByteV(adap, 0x042d,"uwManualCropVerticalSize1		    ");
	ReadByteV(adap, 0x0421,"bCropHStartMSB1			    ");	
	ReadByteV(adap, 0x0422,"bCropHStartLSB1			    ");	
	ReadByteV(adap, 0x0429,"bCropVStartMSB1			    ");	
	ReadByteV(adap, 0x042a,"bCropVStartLSB1			    ");	
	ReadByteV(adap, 0x0425,"bCropHSizeMSB1			    ");	
	ReadByteV(adap, 0x0426,"bCropHSizeLSB1			    ");	
	ReadByteV(adap, 0x042d,"bCropVSizeMSB1			    ");	
	ReadByteV(adap, 0x042e,"bCropVSizeLSB1			    ");	
	ReadByteV(adap, 0x0430,"bDataFormat1				    ");
	ReadByteV(adap, 0x0432,"bBayerOutputAlignment1		    ");	
	ReadByteV(adap, 0x0434,"bContrast1				    ");
	ReadByteV(adap, 0x0436,"bColourSaturation1			    ");
	ReadByteV(adap, 0x0438,"bGamma1				    ");	
	ReadByteV(adap, 0x043a,"fHorizontalMirror1			    ");
	ReadByteV(adap, 0x043c,"fVerticalFlip1			    ");	
	ReadByteV(adap, 0x043e,"bChannelID1				    ");
	ReadByteV(adap, 0x0480,"fEnable				    ");	
	ReadByteV(adap, 0x0482,"bInitialPipeSetupBank			    ");
	ReadByteV(adap, 0x0500,"CurrentPipeSetupBank			    ");
	ReadByteV(adap, 0x0580,"bTimeToPowerdown			    ");
	ReadByteV(adap, 0x058a,"fVRegSleep				    ");
	ReadByteV(adap, 0x058c,"fSmoothLineReading			    ");
	ReadByteV(adap, 0x0605,"uwExternalClockFrequencyMhzNumerator	    ");
	ReadByteV(adap, 0x0605,"uwExternalClockFrequencyMhzNumeratorMSB   ");	
	ReadByteV(adap, 0x0606,"uwExternalClockFrequencyMhzNumeratorLSB   ");	
	ReadByteV(adap, 0x0608,"bExternalClockFrequencyMhzDenominator	    ");  
	ReadByteV(adap, 0x0681,"fpExternalClockFrequencyMhz		    ");
	ReadByteV(adap, 0x0880,"bSysClkMode				    ");
	ReadByteV(adap, 0x0882,"bMode					    ");
	ReadByteV(adap, 0x0c80,"bLightingFrequencyHz			    ");
	ReadByteV(adap, 0x0c82,"fFlickerCompatibleFrameLength		    ");
	ReadByteV(adap, 0x0d05,"fpFlickerFreePeriod_us		    ");	
	ReadByteV(adap, 0x0d08,"fAntiFlickerEnabled			    ");
	ReadByteV(adap, 0x0d81,"uwDesiredFrameRate_Num		    ");	
	ReadByteV(adap, 0x0d81,"uwDesiredFrameRate_Num_MSB		    ");
	ReadByteV(adap, 0x0d82,"uwDesiredFrameRate_Num_LSB		    ");
	ReadByteV(adap, 0x0d84,"bDesiredFrameRate_Den			    ");
	ReadByteV(adap, 0x0e01,"fpRequestedFrameRate_Hz		    ");	
	ReadByteV(adap, 0x0e01,"fpRequestedFrameRate_Hz_MSB		    ");
	ReadByteV(adap, 0x0e02,"fpRequestedFrameRate_Hz_LSB		    ");
	ReadByteV(adap, 0x0e05,"fpMaxFrameRate_Hz			    ");
	ReadByteV(adap, 0x0e09,"fpMinFrameRate_Hz			    ");
	ReadByteV(adap, 0x0e0c,"fChangePending			    ");	
	ReadByteV(adap, 0x0e0f,"uwRequiredFrameLength_lines		    ");
	ReadByteV(adap, 0x0e12,"ClipFrameRate				    ");
	ReadByteV(adap, 0x0e80,"fDisableFrameRateDamper		    ");	
	ReadByteV(adap, 0x0e82,"bImpliedGainThresholdLow_num		    ");
	ReadByteV(adap, 0x0e84,"bImpliedGainThresholdLow_den		    ");
	ReadByteV(adap, 0x0e86,"bImpliedGainThresholdHigh_num		    ");
	ReadByteV(adap, 0x0e88,"bImpliedGainThresholdHigh_den		    ");
	ReadByteV(adap, 0x0e8a,"bUserMinimumFrameRate_Hz		    ");
	ReadByteV(adap, 0x0e8c,"bUserMaximumFrameRate_Hz		    ");
	ReadByteV(adap, 0x0e8e,"bRelativeChange_num			    ");
	ReadByteV(adap, 0x0e90,"bRelativeChange_den			    ");
	ReadByteV(adap, 0x0e92,"fDivorceMinFrameRateFromMaxIntegration    ");	
	ReadByteV(adap, 0x0f01,"fpImpliedGain				    ");
	ReadByteV(adap, 0x0f05,"uwMaximumFrameLength_lines		    ");
	ReadByteV(adap, 0x0f09,"uwMinimumFrameLength_lines		    ");
	ReadByteV(adap, 0x0f0d,"uwFrameLengthChange_lines		    ");
	ReadByteV(adap, 0x0f11,"fpDesiredAutomaticFrameRate_Hz	    ");	
	ReadByteV(adap, 0x0f15,"uwCurrentFrameLength_lines		    ");
	ReadByteV(adap, 0x0f19,"uwDesiredFrameLength_lines		    ");
	ReadByteV(adap, 0x0f1c,"fAutomaticFrameRateStable		    ");
	ReadByteV(adap, 0x0f1e,"fAutomaticFrameRateClip		    ");	
	ReadByteV(adap, 0x0f81,"uwXOffset				    ");
	ReadByteV(adap, 0x0f85,"uwYOffset				    ");
	ReadByteV(adap, 0x0f89,"uwXSize				    ");	
	ReadByteV(adap, 0x0f8d,"uwYSize				    ");	
	ReadByteV(adap, 0x1180,"ExposureControls_bMode		    ");	
	ReadByteV(adap, 0x1182,"bExposureMetering			    ");
	ReadByteV(adap, 0x1184,"bManualExposureTime_s_num		    ");
	ReadByteV(adap, 0x1186,"bManualExposureTime_s_den		    ");
	ReadByteV(adap, 0x1189,"fpManualDesiredExposureTime_us	    ");	
	ReadByteV(adap, 0x1190,"iExposureCompensation			    ");
	ReadByteV(adap, 0x1195,"uwDirectModeCoarseIntegration_lines	    ");
	ReadByteV(adap, 0x1199,"uwDirectModeFineIntegration_pixels	    ");
	ReadByteV(adap, 0x119d,"uwDirectModeCodedAnalogGain		    ");
	ReadByteV(adap, 0x11a1,"fpDirectModeDigitalGain		    ");	
	ReadByteV(adap, 0x11a5,"uwFlashGunModeCoarseIntegration_lines	    ");
	ReadByteV(adap, 0x11a9,"uwFlashGunModeFineIntegration_pixels	    ");
	ReadByteV(adap, 0x11ad,"uwFlashGunModeCodedAnalogGain		    ");
	ReadByteV(adap, 0x11b1,"fpFlashGunModeDigitalGain		    ");
	ReadByteV(adap, 0x11b4,"fFreezeAutoExposure			    ");
	ReadByteV(adap, 0x11b7,"fpUserMaximumIntegrationTime_us	    ");	
	ReadByteV(adap, 0x11bb,"fpRecommendFlashGunAnalogGainThreshold    ");	
	ReadByteV(adap, 0x11be,"fEnableHighClipForDesiredExposureTime	    ");
	ReadByteV(adap, 0x11c0,"bAntiFlickerMode			    ");
	ReadByteV(adap, 0x1201,"fpMaximumStep				    ");
	ReadByteV(adap, 0x1205,"fpMinimumStep				    ");
	ReadByteV(adap, 0x1209,"fpMinimumDesiredExposureTime_us	    ");	
	ReadByteV(adap, 0x120d,"fpStepProportion			    ");
	ReadByteV(adap, 0x1211,"fpMaximumNegativeStepThreshold	    ");	
	ReadByteV(adap, 0x1215,"fpRelativeOnTargetStabilityThreshold	    ");
	ReadByteV(adap, 0x1219,"fpDigitalGainFloor			    ");
	ReadByteV(adap, 0x121d,"fpDigitalGainCeiling			    ");
	ReadByteV(adap, 0x1221,"fpRelativeIntTimeHysThreshold		    ");
	ReadByteV(adap, 0x1225,"fpRelativeDigitalGainHysThreshold	    ");
	ReadByteV(adap, 0x1229,"fpRelativeCompilationProblemThreshold	    ");
	ReadByteV(adap, 0x122d,"fpRoundUpBunchFudge			    ");
	ReadByteV(adap, 0x1231,"fpFineClampThreshold			    ");
	ReadByteV(adap, 0x1235,"fpMaximumManualExposureTime_s		    ");
	ReadByteV(adap, 0x1239,"fpRelativeStabilityThresholdForAutoFocus  ");
	ReadByteV(adap, 0x123c,"bLeakShift				    ");
	ReadByteV(adap, 0x1281,"fpLeakyEnergy				    ");
	ReadByteV(adap, 0x1285,"fpRelativeStep			    ");	
	ReadByteV(adap, 0x1309,"uwCoarseIntegrationPending_lines	    ");
	ReadByteV(adap, 0x130d,"uwFineIntegrationPending_pixels	    ");	
	ReadByteV(adap, 0x1311,"fpAnalogGainPending			    ");
	ReadByteV(adap, 0x1315,"fpDigitalGainPending			    ");
	ReadByteV(adap, 0x1319,"fpDesiredExposureTime_us		    ");
	ReadByteV(adap, 0x131d,"fpCompiledExposureTime_us		    ");
	ReadByteV(adap, 0x132b,"uwCodedAnalogGainPending		    ");
	ReadByteV(adap, 0x1480,"bWhiteBalanceMode			    ");
	ReadByteV(adap, 0x1482,"bManualRedGain			    ");	
	ReadByteV(adap, 0x1484,"bManualGreenGain			    ");
	ReadByteV(adap, 0x1486,"bManualBlueGain			    ");	
	ReadByteV(adap, 0x148b,"fpFlashRedGain			    ");	
	ReadByteV(adap, 0x148f,"fpFlashGreenGain			    ");
	ReadByteV(adap, 0x1493,"fpFlashBlueGain			    ");	
	ReadByteV(adap, 0x1500,"bStatus				    ");	
	ReadByteV(adap, 0x1505,"fpRedGain				    ");
	ReadByteV(adap, 0x1509,"fpGreenGain				    ");
	ReadByteV(adap, 0x150d,"fpBlueGain				    ");
	ReadByteV(adap, 0x1581,"fpStableTotalStepThreshold		    ");
	ReadByteV(adap, 0x1585,"fpMinimumRelativeStep			    ");
	ReadByteV(adap, 0x1589,"fpMaximumRelativeStep			    ");
	ReadByteV(adap, 0x1601,"fpRedA				    ");	
	ReadByteV(adap, 0x1605,"fpBlueA				    ");	
	ReadByteV(adap, 0x1609,"fpRedB				    ");	
	ReadByteV(adap, 0x160d,"fpBlueB				    ");	
	ReadByteV(adap, 0x1611,"fpMaximumDistanceAllowedFromLocus	    ");
	ReadByteV(adap, 0x1614,"fEnableConstrainedWhiteBalance	    ");	
	ReadByteV(adap, 0x1616,"bACCSRCCtrl				    ");
	ReadByteV(adap, 0x1681,"fpOutputRedGain			    ");	
	ReadByteV(adap, 0x1685,"fpOutputGreenGain			    ");
	ReadByteV(adap, 0x1689,"fpOutputBlueGain			    ");
	ReadByteV(adap, 0x168c,"fAreGainsConstrained			    ");
	ReadByteV(adap, 0x1701,"fpGradientOfLocusAB			    ");
	ReadByteV(adap, 0x1705,"fpDistanceOfInputPointFromLocusAB	    ");
	ReadByteV(adap, 0x1709,"fpConstrainedRedPoint			    ");
	ReadByteV(adap, 0x170d,"fpConstrainedBluePoint		    ");	
	ReadByteV(adap, 0x1880,"bMaxNumberOfFramesToWaitForStability	    ");
	ReadByteV(adap, 0x1900,"fWhiteBalanceStable			    ");
	ReadByteV(adap, 0x1902,"fExposureStable			    ");	
	ReadByteV(adap, 0x1904,"fDarkCalStable			    ");	
	ReadByteV(adap, 0x1906,"fStable				    ");	
	ReadByteV(adap, 0x1908,"fForcedStablility			    ");
	ReadByteV(adap, 0x1985,"fpRedTilt				    ");
	ReadByteV(adap, 0x1989,"fpGreenTilt				    ");
	ReadByteV(adap, 0x198d,"fpBlueTilt				    ");
	ReadByteV(adap, 0x1990,"bBlackCorrectionOffset		    ");	
	ReadByteV(adap, 0x1a01,"uwSensorAnalogGainFloor		    ");	
	ReadByteV(adap, 0x1a05,"uwSensorAnalogGainCeiling		    ");
	ReadByteV(adap, 0x1a80,"bFlashMode				    ");
	ReadByteV(adap, 0x1a83,"uwFlashOffLine			    ");	
	ReadByteV(adap, 0x1b00,"fFlashRecommended			    ");
	ReadByteV(adap, 0x1b02,"fFlashGrabComplete			    ");
	ReadByteV(adap, 0x1d01,"uwHorizontalOffset			    ");
	ReadByteV(adap, 0x1d05,"uwVerticalOffset			    ");
	ReadByteV(adap, 0x1d08,"iR2RCoefficient			    ");	
	ReadByteV(adap, 0x1d0a,"iR2GRCoefficient			    ");
	ReadByteV(adap, 0x1d0c,"iR2GBCoefficient			    ");
	ReadByteV(adap, 0x1d0e,"iR2BCoefficient			    ");	
	ReadByteV(adap, 0x1d10,"iR4RCoefficient			    ");	
	ReadByteV(adap, 0x1d12,"iR4GRCoefficient			    ");
	ReadByteV(adap, 0x1d14,"iR4GBCoefficient			    ");
	ReadByteV(adap, 0x1d16,"iR4BCoefficient			    ");	
	ReadByteV(adap, 0x1d80,"ScythefDisableFilter			    ");
	ReadByteV(adap, 0x1e00,"JackfDisableFilter			    ");
	ReadByteV(adap, 0x1e80,"bAntiAliasFilterSuppress		    ");
	ReadByteV(adap, 0x1f00,"ColourMatrixDamperfDisable		    ");
	ReadByteV(adap, 0x1f03,"fpLowThreshold			    ");	
	ReadByteV(adap, 0x1f07,"fpHighThreshold			    ");	
	ReadByteV(adap, 0x1f0b,"fpMinimumOutput			    ");	
	ReadByteV(adap, 0x1f81,"fpGInR				    ");	
	ReadByteV(adap, 0x1f85,"fpBInR				    ");	
	ReadByteV(adap, 0x1f89,"fpRInG				    ");	
	ReadByteV(adap, 0x1f8d,"fpBInG				    ");	
	ReadByteV(adap, 0x1f91,"fpRInB				    ");	
	ReadByteV(adap, 0x1f95,"fpGInB				    ");	
	ReadByteV(adap, 0x2000,"bUserPeakGain				    ");
	ReadByteV(adap, 0x2002,"fDisableGainDamping			    ");
	ReadByteV(adap, 0x2005,"fpDamperLowThreshold_Gain		    ");
	ReadByteV(adap, 0x2009,"fpDamperHighThreshold_Gain		    ");
	ReadByteV(adap, 0x200d,"fpMinimumDamperOutput_Gain		    ");
	ReadByteV(adap, 0x2010,"bUserPeakLoThresh			    ");
	ReadByteV(adap, 0x2012,"fDisableCoringDamping			    ");
	ReadByteV(adap, 0x2014,"bUserPeakHiThresh			    ");
	ReadByteV(adap, 0x2017,"fpDamperLowThreshold_Coring		    ");
	ReadByteV(adap, 0x201b,"fpDamperHighThreshold_Coring		    ");
	ReadByteV(adap, 0x201f,"fpMinimumDamperOutput_Coring		    ");
	ReadByteV(adap, 0x2022,"bBlockControl				    ");
	ReadByteV(adap, 0x2280,"fGammaManuCtrl0			    ");	
	ReadByteV(adap, 0x2282,"bRPeakGamma0				    ");
	ReadByteV(adap, 0x2284,"bGPeakGamma0				    ");
	ReadByteV(adap, 0x2286,"bBPeakGamma0				    ");
	ReadByteV(adap, 0x2288,"bRUnPeakGamma0			    ");	
	ReadByteV(adap, 0x228a,"bGUnPeakGamma0			    ");	
	ReadByteV(adap, 0x228c,"bBUnPeakGamma0			    ");	
	ReadByteV(adap, 0x2294,"bYuvSetup  MTW                      ");
	ReadByteV(adap, 0x2300,"fGammaManuCtrl1			    ");	
	ReadByteV(adap, 0x2302,"bRPeakGamma1				    ");
	ReadByteV(adap, 0x2304,"bGPeakGamma1				    ");
	ReadByteV(adap, 0x2306,"bBPeakGamma1				    ");
	ReadByteV(adap, 0x2308,"bRUnPeakGamma1			    ");	
	ReadByteV(adap, 0x230a,"bGUnPeakGamma1			    ");	
	ReadByteV(adap, 0x230c,"bBUnPeakGamma1			    ");	
	ReadByteV(adap, 0x2381,"uwLumaExcursion0			    ");
	ReadByteV(adap, 0x2385,"uwLumaMidpointTimes20			    ");
	ReadByteV(adap, 0x2389,"uwChromaExcursion0			    ");
	ReadByteV(adap, 0x238d,"uwChromaMidpointTimes20		    ");	
	ReadByteV(adap, 0x2401,"uwLumaExcursion1			    ");
	ReadByteV(adap, 0x2405,"uwLumaMidpointTimes21			    ");
	ReadByteV(adap, 0x2409,"uwChromaExcursion1			    ");
	ReadByteV(adap, 0x240d,"uwChromaMidpointTimes21		    ");	
	ReadByteV(adap, 0x2480,"FadeToBlackfDisable			    ");
	ReadByteV(adap, 0x2483,"fpBlackValue				    ");
	ReadByteV(adap, 0x2487,"fpDamperLowThreshold			    ");
	ReadByteV(adap, 0x248b,"fpDamperHighThreshold			    ");
	ReadByteV(adap, 0x248f,"fpDamperOutput			    ");	
	ReadByteV(adap, 0x2580,"bCodeCheckEn				    ");
	ReadByteV(adap, 0x2582,"bBlankFormat				    ");
	ReadByteV(adap, 0x2584,"bSyncCodeSetup			    ");	
	ReadByteV(adap, 0x2586,"bHSyncSetup				    ");
	ReadByteV(adap, 0x2588,"bVSyncSetup				    ");
	ReadByteV(adap, 0x258a,"bPClkSetup				    ");
	ReadByteV(adap, 0x258c,"fPclkEn				    ");	
	ReadByteV(adap, 0x258e,"bOpfSpSetup				    ");
	ReadByteV(adap, 0x2590,"bBlankData_MSB			    ");	
	ReadByteV(adap, 0x2592,"bBlankData_LSB			    ");	
	ReadByteV(adap, 0x2594,"bRgbSetup				    ");
	ReadByteV(adap, 0x2596,"bYuvSetup				    ");
	ReadByteV(adap, 0x2598,"bVsyncRisingCoarseH			    ");
	ReadByteV(adap, 0x259a,"bVsyncRisingCoarseL			    ");
	ReadByteV(adap, 0x259c,"bVsyncRisingFineH			    ");
	ReadByteV(adap, 0x259e,"bVsyncRisingFineL			    ");
	ReadByteV(adap, 0x25a0,"bVsyncFallingCoarseH			    ");
	ReadByteV(adap, 0x25a2,"bVsyncFallingCoarseL			    ");
	ReadByteV(adap, 0x25a4,"bVsyncFallingFineH			    ");
	ReadByteV(adap, 0x25a6,"bVsyncFallingFineL			    ");
	ReadByteV(adap, 0x25a8,"bHsyncRisingH				    ");
	ReadByteV(adap, 0x25aa,"bHsyncRisingL				    ");
	ReadByteV(adap, 0x25ac,"bHsyncFallingH			    ");	
	ReadByteV(adap, 0x25ae,"bHsyncFallingL			    ");	
	ReadByteV(adap, 0x25b0,"bOutputInterface			    ");
	ReadByteV(adap, 0x25b2,"bCCPExtraData				    ");
	ReadByteV(adap, 0x2600,"NoRAfDisable				    ");
	ReadByteV(adap, 0x2602,"bUsage				    ");	
	ReadByteV(adap, 0x2604,"bSplit_Kn				    ");
	ReadByteV(adap, 0x2606,"bSplit_Nl				    ");
	ReadByteV(adap, 0x2608,"bTight_Green				    ");
	ReadByteV(adap, 0x260a,"fDisableNoraPromoting			    ");
	ReadByteV(adap, 0x260d,"DamperLowThreshold			    ");
	ReadByteV(adap, 0x2611,"DamperHighThreshold			    ");
	ReadByteV(adap, 0x2615,"MinimumDamperOutput			    ");

	return;			        
}		        
