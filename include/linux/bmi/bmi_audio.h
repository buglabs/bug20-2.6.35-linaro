/*
 * File:         include/linux/bmi/bmi_audio.h
 * Author:       Peter Giacomini <p.giacomini@encadis.com>
 *
 * 		This is the application header file for the BMI bus audio plug-in
 * 		module on the MX31 BUG platform.
 */

#ifndef BMI_AUDIO_H
#define BMI_AUDIO_H

#include <linux/bmi/bmi_ioctl.h>

// GPIO	defines
typedef enum {
	GPIO_SPARE,	// unused
	GPIO_RESET,	// CODEC reset
	GPIO_GREEN,	// green LED
	GPIO_RED,	// red LED
} BMI_AUDIO_GPIO;

// GETSTAT defines
typedef enum {
	GETSTAT_AMP	= 0x001,	// IOX bit 0 - amplifier off (O - low active)
	GETSTAT_ISPARE	= 0x002,	// IOX bit 1 - spare
	GETSTAT_VOLP	= 0x004,	// IOX bit 2 - VOLP (I - interrupt)
	GETSTAT_VOLD	= 0x008,	// IOX bit 3 - VOLD (I - interrupt)
	GETSTAT_HP_INS	= 0x010,	// IOX bit 4 - HP_INS (I - interrupt)
	GETSTAT_MIC_INS	= 0x020,	// IOX bit 5 - MIC_INS (I - interrupt)
	GETSTAT_LI_INS	= 0x040,	// IOX bit 6 - LI_INS (I - interrupt)
	GETSTAT_LO_INS	= 0x080,	// IOX bit 7 - LO_INS (I - interrupt)
	GETSTAT_GSPARE	= 0x100,	// unused
	GETSTAT_RESET	= 0x200,	// CODEC reset
	GETSTAT_GREEN	= 0x400,	// green LED
	GETSTAT_RED	= 0x800,	// red LED
} BMI_AUDIO_GETSTAT;

// input event bit defintions
typedef enum {
	HEADPHONE_INSERTED         = 0x001,	// Detected headphone insertion
	MICROPHONE_INSERTED        = 0x002,	// Detected microphone insertion
	LINEOUT_INSERTED           = 0x004,	// Detected line out insertion
	LINEIN_INSERTED            = 0x008,	// Detected line in insertion
	VOLUME_DOWN                = 0x010,	// volume down button pressed
	VOLUME_UP                  = 0x020,	// volume up button pressed
} BMI_AUDIO_EVENT;

// module numbers
typedef enum {
	BMI_AUDIO_M1,		// PIM 1
	BMI_AUDIO_M2,		// PIM 2
	BMI_AUDIO_M3,		// PIM 3
	BMI_AUDIO_M4,		// PIM 4
	BMI_AUDIO_PIM_NUM,	// Number of PIMs
} BMI_MODULE_NUMBERS;

// TI '3105 CODEC registers
// Page 0
#define CODEC_PAGE_SEL			0x0			// page select
#define CODEC_RESET			0x1			// reset (self-clearing)
	#define CODEC_RESET_RESET	0x1			// reset (self-clearing)
#define CODEC_SAMPLE_RATE		0x2			// ADC/DAC sample rate
	#define CODEC_SR1		0x0			// ADC/DAC sample rate
	#define CODEC_SR1_5		0x1			// ADC/DAC sample rate
	#define CODEC_SR2		0x2			// ADC/DAC sample rate
	#define CODEC_SR2_5		0x3			// ADC/DAC sample rate
	#define CODEC_SR3		0x4			// ADC/DAC sample rate
	#define CODEC_SR3_5		0x5			// ADC/DAC sample rate
	#define CODEC_SR4		0x6			// ADC/DAC sample rate
	#define CODEC_SR4_5		0x7			// ADC/DAC sample rate
	#define CODEC_SR5		0x8			// ADC/DAC sample rate
	#define CODEC_SR5_5		0x9			// ADC/DAC sample rate
	#define CODEC_SR6		0xA			// ADC/DAC sample rate
	#define CODEC_SR_SHIFT		(4)			// ADC shift
#define CODEC_PLLA			0x3			// PLL Programming A
	#define CODEC_PLLA_EN		0x80			// PLL enabled
	#define CODEC_PLLA_DIS		0x00			// PLL disabled
	#define CODEC_PLLA_Q(x)		(((x) & 0xF) << 3)	// PLL Q
	#define CODEC_PLLA_P(x)		((x) & 0x7)		// PLL P
#define CODEC_PLLB			0x4			// PLL Programming B
	#define CODEC_PLLB_J(x)		(((x) & 0x3F) << 2)	// PLL J
#define CODEC_PLLC_DMSB			0x5			// PLL D MSB
#define CODEC_PLLD_DLSB			0x6			// PLL D LSB
	#define CODEC_PLLD_D(x)		(((x) & 0x3F) << 2)	// PLL D LSB
#define CODEC_DATAPATH			0x7			// Datapath set up
	#define CODEC_DP_48		(0x00)			// 48 kHz
	#define CODEC_DP_44		(0x80)			// 44.1 kHz
	#define CODEC_ADR_DIS		(0x00)			// ADC Dual Rate
	#define CODEC_ADR_EN		(0x40)			// ADC Dual Rate
	#define CODEC_DDR_DIS		(0x00)			// DAC Dual Rate
	#define CODEC_DDR_EN		(0x20)			// DAC Dual Rate
	#define CODEC_DP_MUTED		(0x00)			// DAC Data Path
	#define CODEC_DP_NORMAL		(0x01)			// DAC Data Path
	#define CODEC_DP_REVERSE	(0x02)			// DAC Data Path
	#define CODEC_DP_MONO		(0x03)			// DAC Data Path
	#define CODEC_DP_L(x)		((x) << 3)		// DAC Data Path Left
	#define CODEC_DP_R(x)		((x) << 1)		// DAC Data Path Right
#define CODEC_AIFA			0x8			// Audio serial data IF control
	#define CODEC_AIFA_BCLK_S	0x00			// BCLK is input
	#define CODEC_AIFA_BCLK_M	0x80			// BCLK is output
	#define CODEC_AIFA_WCLK_S	0x00			// WCLK is input
	#define CODEC_AIFA_WCLK_M	0x40			// WCLK is output
	#define CODEC_AIFA_DOUT_N	0x00			// Dout not tri-state
	#define CODEC_AIFA_DOUT_TS	0x20			// Dout tri-states
	#define CODEC_AIFA_CLK_G	0x00			// CLKs gated (Master mode only)
	#define CODEC_AIFA_CLK_F	0x10			// CLKs free run (Master mode only)
	#define CODEC_AIFA_FX_OFF	0x00			// disable 3-D EFX
	#define CODEC_AIFA_FX_ON	0x04			// enable 3-D EFX
#define CODEC_AIFB			0x9			// Audio serial data IF control
	#define CODEC_AIFB_I2S		0x00			// MODE = I2S
	#define CODEC_AIFB_DSP		0x40			// MODE = DSP
	#define CODEC_AIFB_RJ		0x80			// MODE = Right Justified
	#define CODEC_AIFB_LJ		0xC0			// MODE = Left Justified
	#define CODEC_AIFB_16		0x00			// World Length = 16 bits
	#define CODEC_AIFB_20		0x10			// World Length = 20 bits
	#define CODEC_AIFB_24		0x20			// World Length = 24 bits
	#define CODEC_AIFB_32		0x30			// World Length = 32 bits
	#define CODEC_AIFB_256S		0x08			// 256-clock transfer mode (TDM)
	#define CODEC_AIFB_DSYNC	0x04			// DAC resync
	#define CODEC_AIFB_ASYNC	0x02			// ADC resync
	#define CODEC_AIFB_MSYNC	0x01			// resync with soft-mute
#define CODEC_AIF_WORD_OFFSET		0xA			// data bit offset in frame
#define CODEC_OVERFLOW			0xB			// Overflow flags
	#define CODEC_OF_LADC		0x80			// Left ADC
	#define CODEC_OF_RADC		0x40			// Right ADC
	#define CODEC_OF_LDAC		0x20			// Left DAC
	#define CODEC_OF_RDAC		0x10			// Right DAC
	#define CODEC_OF_PLLR(x)	((x) & 0xF)		// PLL R
#define CODEC_FILT_CONTROL		0xC			// Filter Control
	#define CODEC_FC_LADC_HP45	0x40			// Left ADC Filter Control
	#define CODEC_FC_LADC_HP125	0x80			// Left ADC Filter Control
	#define CODEC_FC_LADC_HP25	0xC0			// Left ADC Filter Control
	#define CODEC_FC_RADC_HP45	0x10			// Right ADC Filter Control
	#define CODEC_FC_RADC_HP125	0x20			// Right ADC Filter Control
	#define CODEC_FC_RADC_HP25	0x30			// Right ADC Filter Control
#define CODEC_HS			0xE			// Headset/Button
	#define CODEC_HS_COUPLED	0x80			// HP outputs AC-Coupled
	#define CODEC_HS_ADIFF		0x40			// Output A differential
	#define CODEC_HS_HSDET		0x10			// headset detected
	#define CODEC_HS_BDIFF		0x08			// Output B differential
#define CODEC_LADC_PGA			0xF			// Left ADC PGA
#define CODEC_RADC_PGA			0x10			// Right ADC PGA
	#define CODEC_ADC_PGA_MUTE	0x80			// muted
	#define CODEC_ADC_PGA_G(x)	((x) & 0x7F)		// gain (0 to 59.5 dB)
#define CODEC_M3_LPGA			0x11			// MIC3 -> LADC PGA
#define CODEC_M3_RPGA			0x12			// MIC3 -> RADC PGA
	#define CODEC_M3_PGA_LOFF	(0xF << 4)		// L input off
	#define CODEC_M3_PGA_ROFF	(0xF	)		// R input off
	#define CODEC_M3_PGA_L(x)	(((x) & 0xF) << 4)	// L input level (0 to -12 dB)	
	#define CODEC_M3_PGA_R(x)	((x) & 0xF)		// R input level (0 to -12 dB) 
#define CODEC_L1L_LPGA			0x13			// L1 Left -> LADC PGA
#define CODEC_L2L_LPGA			0x14			// L2 Left -> LADC PGA
#define CODEC_L1R_LPGA			0x15			// L1 Right -> LADC PGA
#define CODEC_L1R_RPGA			0x16			// R1 Right -> RADC PGA
#define CODEC_L2R_RPGA			0x17			// L2 Right -> RADC PGA
#define CODEC_L1L_RPGA			0x18			// L1 Left -> RADC PGA
	#define CODEC_L_PGA(x)		(((x) & 0xF) << 3)	// input level (0 to -12 dB)	
	#define CODEC_LX_PGA_PU		0x04			// L1 power up
	#define CODEC_L1L_PGA_SS(x)	((x) & 0x3)		// L1 soft stepping
	#define CODEC_L2L_LPGA_BIASED	0x04			// L2 Left weak bias
#define CODEC_MIC_BIAS			0x19			// Mic Bias
	#define CODEC_MIC_BIAS_PD	0x00			// powered down
	#define CODEC_MIC_BIAS_2V	0x40			// 2V
	#define CODEC_MIC_BIAS_2P5V	0x80			// 2.5V
	#define CODEC_MIC_BIAS_AVDD	0xC0			// AVDD
#define CODEC_MIC_LAGC_A		0x1A			// L AGC A
#define CODEC_MIC_RAGC_A		0x1D			// R AGC A
	#define CODEC_MIC_AGC_EN	0x80			// enable
	#define CODEC_MIC_AGC_TL(x)	(((x) & 0x7) << 4)	// target level (-5.5 to -24 dB)
	#define CODEC_MIC_AGC_AT(x)	(((x) & 0x3) << 2)	// attack time (8 to 20 ms)
	#define CODEC_MIC_AGC_DT(x)	((x) & 0x3)		// decay time (100 to 500 ms)
#define CODEC_MIC_LAGC_B		0x1B			// L AGC B
#define CODEC_MIC_RAGC_B		0x1E			// R AGC B
	#define CODEC_MIC_AGC_MG(x)	(((x) & 0x7F) << 1)	// max gain (0 to 59.5 dB)
#define CODEC_MIC_LAGC_C		0x1C			// L AGC C
#define CODEC_MIC_RAGC_C		0x1F			// R AGC C
	#define CODEC_MIC_AGC_H(x)	(((x) & 0x3) << 6)	// NG hysteresis (1 to 3 dB, off)
	#define CODEC_MIC_AGC_T(x)	(((x) & 0x3) << 6)	// NG Threshold (off to -90 dB)
	#define CODEC_MIC_AGC_SC	0x1			// clip stepping enable
#define CODEC_MIC_LAGC_GAIN		0x20			// L AGC gain (-12 tp 59.5 dB)
#define CODEC_MIC_RAGC_GAIN		0x21			// R AGC gain (-12 tp 59.5 dB)
#define CODEC_MIC_LAGC_NGD		0x22			// L AGC NG debounce
#define CODEC_MIC_RAGC_NGD		0x23			// R AGC NG debounce
	#define CODEC_MIC_AGC_NGD_D(x)	(((x) & 0x1F) << 3)	// detect(0 to 1536 ms)
	#define CODEC_MIC_AGC_NGD_C(x)	((x) & 0x7)		// control (0 to 32 ms)
#define CODEC_ADC_FLAG			0x24			// ADC flag
	#define CODEC_ADC_FLAG_LPGA_S	0x80			// L ADC PGA gain equal
	#define CODEC_ADC_FLAG_LPWR_S	0x40			// L ADC powered-up
	#define CODEC_ADC_FLAG_LSIGD	0x20			// L AGC signal detected
	#define CODEC_ADC_FLAG_LSAT	0x10			// L AGC saturation detected
	#define CODEC_ADC_FLAG_RPGA_S	0x08			// R ADC PGA gain equal
	#define CODEC_ADC_FLAG_RPWR_S	0x04			// R ADC powered-up
	#define CODEC_ADC_FLAG_RSIGD	0x02			// R AGC signal detected
	#define CODEC_ADC_FLAG_RSAT	0x01			// R AGC saturation detected
#define CODEC_DAC_PWR			0x25			// DAC power and output driver
#define CODEC_DAC_HPWR			0x26			// high-power output driver
	#define CODEC_DAC_PWR_L_EN	0x80			// L power up
	#define CODEC_DAC_PWR_R_EN	0x40			// R power up
	#define CODEC_DAC_PWR_HP_DIFF	0x00			// differential of HPLOUT
	#define CODEC_DAC_PWR_HP_VCM	0x10			// constant VCM
	#define CODEC_DAC_PWR_HP_ISE	0x20			// independant single ended
	#define CODEC_DAC_HPWR_HPL_DIFF	0x18			// short circuit protection
	#define CODEC_DAC_HPWR_SS	0x04			// short circuit protection
	#define CODEC_DAC_HPWR_SS_C	0x00			// 	limit current
	#define CODEC_DAC_HPWR_SS_P	0x02			// 	power down
#define CODEC_DAC_HPOS			0x28			// high-power output stage
	#define CODEC_DAC_HPOS_CM1P35	0x00			// common mode = 1.35V
	#define CODEC_DAC_HPOS_CM1P5	0x40			// common mode = 1.5V
	#define CODEC_DAC_HPOS_CM1P65	0x80			// common mode = 1.65V
	#define CODEC_DAC_HPOS_CM1P8	0xC0			// common mode = 1.8V
	#define CODEC_DAC_HPOS_L2L_BYP	0x00			// L2 L bypass disabled
	#define CODEC_DAC_HPOS_L2L_SE	0x10			// L2 L bypass = L2LP
	#define CODEC_DAC_HPOS_L2L_BYP	0x00			// L2 R bypass disabled
	#define CODEC_DAC_HPOS_L2R_SE	0x04			// L2 R bypass = L2RP
	#define CODEC_DAC_HPOS_FS	0x00			// soft stepping: 1 / fs
	#define CODEC_DAC_HPOS_2FS	0x01			// soft stepping: 1 / 2 fs
	#define CODEC_DAC_HPOS_SS_DIS	0x02			// soft stepping disabled
#define CODEC_DAC_OS			0x29			// output switching
	#define CODEC_DAC_OS_L1		0x00			// L = L1
	#define CODEC_DAC_OS_L3		0x40			// L = L3
	#define CODEC_DAC_OS_L2		0x80			// L = L2
	#define CODEC_DAC_OS_R1		0x00			// R = R1
	#define CODEC_DAC_OS_R3		0x10			// R = R3
	#define CODEC_DAC_OS_R2		0x20			// R = R2
	#define CODEC_DAC_OS_VOL_S	0x00			// volume separate
	#define CODEC_DAC_OS_VOL_R	0x01			// 	L follows R
	#define CODEC_DAC_OS_VOL_L	0x02			// 	R follows L
#define CODEC_DAC_PR			0x2A			// pop reduction
	#define CODEC_DAC_PR_DEL(x)	(((x) & 0xF) << 4)	// delay (0 us to 4 s)
	#define CODEC_DAC_PR_RU(x)	(((x) & 0x3) << 2)	// ramp up (0 to 4 ms)
	#define CODEC_DAC_CM_AVDD	0x00			// common mode from AVDD
	#define CODEC_DAC_CM_BG		0x02			// common mode from band gap
#define CODEC_DAC_LVOL			0x2B			// Left volume
#define CODEC_DAC_RVOL			0x2C			// Right volume
	#define CODEC_DAC_VOL_MUTE	0x80			// muted
	#define CODEC_DAC_VOL(x)	((x) & 0x7F)		// volume (0 to -63.5 dB)
#define CODEC_L2L_HPL			0x2D			// L2L -> HPLOUT
#define CODEC_PGAL_HPL			0x2E			// PGAL -> HPLOUT
#define CODEC_DACL1_HPL			0x2F			// DACL1 -> HPLOUT
#define CODEC_L2R_HPL			0x30			// L2R -> HPLOUT
#define CODEC_PGAR_HPL			0x31			// PGAR -> HPLOUT
#define CODEC_DACR1_HPL			0x32			// DACLR -> HPLOUT
#define CODEC_L2L_HPLCOM		0x34			// L2L -> HPLCOM
#define CODEC_PGAL_HPLCOM		0x35			// PGAL -> HPLCOM
#define CODEC_DACL1_HPLCOM		0x36			// DACL1 -> HPLCOM
#define CODEC_L2R_HPLCOM		0x37			// L2R -> HPLCOM
#define CODEC_PGAR_HPLCOM		0x38			// PGAR -> HPLCOM
#define CODEC_DACR1_HPLCOM		0x39			// DACR1 -> HPLCOM
#define CODEC_L2L_HPR			0x3B			// L2L -> HPROUT
#define CODEC_PGAL_HPR			0x3C			// PGAL -> HPROUT
#define CODEC_DACL1_HPR			0x3D			// DACL1 -> HPROUT
#define CODEC_L2R_HPR			0x3E			// L2R -> HPROUT
#define CODEC_PGAR_HPR			0x3F			// PGAR -> HPROUT
#define CODEC_DACR1_HPR			0x40			// DACLR -> HPROUT
#define CODEC_L2L_HPRCOM		0x42			// L2L -> HPRCOM
#define CODEC_PGAL_HPRCOM		0x43			// PGAL -> HPRCOM
#define CODEC_DACL1_HPRCOM		0x44			// DACL1 -> HPRCOM
#define CODEC_L2R_HPRCOM		0x45			// L2R -> HPRCOM
#define CODEC_PGAR_HPRCOM		0x46			// PGAR -> HPRCOM
#define CODEC_DACR1_HPRCOM		0x47			// DACLR -> HPRCOM
#define CODEC_L2L_LLOPM			0x50
#define CODEC_PGAL_LLOPM		0x51
#define CODEC_DACL1_LLOPM		0x52
#define CODEC_L2R_LLOPM			0x53
#define CODEC_PGAR_LLOPM		0x54
#define CODEC_DACR1_LLOPM		0x55
#define CODEC_L2L_RLOPM			0x57
#define CODEC_PGA_RLOPM			0x58
#define CODEC_DACL1_RLOPM		0x59
#define CODEC_L2R_RLOPM			0x5A
#define CODEC_PGAR_RLOPM		0x5B
#define CODEC_DACR1_RLOPM		0x5C
	#define CODEC_HP_EN		0x80			// enabled
	#define CODEC_HP_VOL(x)		((x) & 0x7F)		// see datasheet Table 6
#define CODEC_HPLOUT			0x33			// HPLOUT output level
#define CODEC_HPLCOM			0x3A			// HPLCOM output level
#define CODEC_HPROUT			0x41			// HPROUT output level
#define CODEC_HPRCOM			0x48			// HPRCOM output level
#define CODEC_LLOPM			0x56			// LLOPM output level
#define CODEC_RLOPM			0x5D			// RLOPM output level
	#define CODEC_HPX_LC(x)		(((x) & 0xF) << 4)	// output level
	#define CODEC_HPX_EN		0x08			// not muted
	#define CODEC_HPX_PD		0x04			// power down enable
	#define CODEC_HPX_STAT		0x02			// gain not applied
	#define CODEC_HPX_PC		0x01			// fully powered up
#define CODEC_PSR			0x5E			// Power Status
	#define CODEC_PSR_LDPS		0x80			// L DAC
	#define CODEC_PSR_DDPS		0x40			// R DAC
	#define CODEC_PSR_LLOPM		0x10			// L LOPM
	#define CODEC_PSR_RLOPM		0x08			// R LOPM
	#define CODEC_PSR_HPLOUT	0x04			// HPLOUT
	#define CODEC_PSR_HPLCOM	0x02			// HPLCOM
#define CODEC_SS			0x5F			// driver short circuit
	#define CODEC_SS_HPLOUT		0x80
	#define CODEC_SS_HPROUT		0x40
	#define CODEC_SS_HPLCOM		0x20
	#define CODEC_SS_HPRCOM		0x10
	#define CODEC_SS_HPLCOM_PS	0x08
	#define CODEC_SS_HPRCOM_PS	0x04
#define CODEC_S_INT			0x60			// sticky interrupt
#define CODEC_RT_INT			0x61			// real-time interrupt
	#define CODEC_INT_HPLOUT_SS	0x80
	#define CODEC_INT_HPROUT_SS	0x40
	#define CODEC_INT_HPLCOM_SS	0x20
	#define CODEC_INT_HPRCOM_SS	0x10
	#define CODEC_INT_HS_DET	0x04
	#define CODEC_INT_LAGC_NG	0x02
	#define CODEC_INT_RAGC_NG	0x01
#define CODEC_CLK			0x65			// clock source
	#define CODEC_CLK_PLLDIV	0x00
	#define CODEC_CLK_CLKDIV	0x01
#define CODEC_CLKGEN			0x66			// clock generation
	#define CODEC_CLKGEN_C_M	0x02			// MCLK -> CLK
	#define CODEC_CLKGEN_C_G	0x42			// GPIO2 -> CLK
	#define CODEC_CLKGEN_C_B	0x82			// BCLK -> CLK
	#define CODEC_CLKGEN_P_M	0x02			// MCLK -> PLL
	#define CODEC_CLKGEN_P_G	0x12			// GPIO2 -> PLL
	#define CODEC_CLKGEN_P_B	0x22			// BCLK -> PLL
#define CODEC_LAGC_ATT			0x67			// L AGC Attack
#define CODEC_RAGC_ATT			0x69			// R AGC Attack
	#define CODEC_AGC_ATT_R26	0x00			// source reg 36
	#define CODEC_AGC_ATT_R103	0x80			// source reg 103
	#define CODEC_AGC_ATT_T(x)	(((x) & 0x3) << 5)	// time
	#define CODEC_AGC_ATT_M(x)	(((x) & 0xF) << 2)	// multiplication
#define CODEC_LAGC_DEC			0x68			// L AGC Decay
#define CODEC_RAGC_DEC			0x6A			// R AGC Decay
	#define CODEC_AGC_DEC_R26	0x00			// source reg 36
	#define CODEC_AGC_DEC_R104	0x80			// source reg 104
	#define CODEC_AGC_DEC_T(x)	(((x) & 0x3) << 5)	// time
	#define CODEC_AGC_DEC_M(x)	(((x) & 0xF) << 2)	// multiplication
#define CODEC_DP_I2C			0x6B			// digital path and I2C
	#define CODEC_DP_I2C_LHPF_EN	0x80	
	#define CODEC_DP_I2C_RHPF_EN	0x40	
	#define CODEC_ADC_DFLDRD	0x00
	#define CODEC_ADC_DFLDRA	0x10
	#define CODEC_ADC_DFLARD	0x20
	#define CODEC_ADC_DFLARA	0x30
	#define CODEC_ADC_F_EN		0x08
	#define CODEC_I2C_ERR_DIS	0x04
	#define CODEC_I2C_HANG		0x01
#define CODEC_PASB			0x6C			// passive analog bypass
	#define CODEC_PASB_L2RP_RLOP	0x40
	#define CODEC_PASB_L1RP_RLOP	0x10
	#define CODEC_PASB_L2LP_LLOP	0x04
	#define CODEC_PASB_L1LP_LLOP	0x01
#define CODEC_DAC_QCA			0x6D			// DAC current adjust
#define CODEC_DAC_QCA_50		0x40
#define CODEC_DAC_QCA_100		0xC0
// Page 1
#define CODEC_EF_LN0M			0x1
#define CODEC_EF_LN0L			0x2
#define CODEC_EF_LN1M			0x3
#define CODEC_EF_LN1L			0x4
#define CODEC_EF_LN2M			0x5
#define CODEC_EF_LN2L			0x6
#define CODEC_EF_LN3M			0x7
#define CODEC_EF_LN3L			0x8
#define CODEC_EF_LN4M			0x9
#define CODEC_EF_LN4L			0xA
#define CODEC_EF_LN5M			0xB
#define CODEC_EF_LN5L			0xC

#define CODEC_EF_LD1M			0xD
#define CODEC_EF_LD1L			0xE
#define CODEC_EF_LD2M			0xF
#define CODEC_EF_LD2L			0x10
#define CODEC_EF_LD4M			0x11
#define CODEC_EF_LD4L			0x12
#define CODEC_EF_LD5M			0x13
#define CODEC_EF_LD5L			0x14

#define CODEC_DF_LN0M			0x15
#define CODEC_DF_LN0L			0x16
#define CODEC_DF_LN1M			0x17
#define CODEC_DF_LN1L			0x18

#define CODEC_DF_LD1M			0x19
#define CODEC_DF_LD1L			0x1A

#define CODEC_EF_RN0M			0x1B
#define CODEC_EF_RN0L			0x1C
#define CODEC_EF_RN1M			0x1D
#define CODEC_EF_RN1L			0x1E
#define CODEC_EF_RN2M			0x1F
#define CODEC_EF_RN2L			0x20
#define CODEC_EF_RN3M			0x21
#define CODEC_EF_RN3L			0x22
#define CODEC_EF_RN4M			0x23
#define CODEC_EF_RN4L			0x24
#define CODEC_EF_RN5M			0x25
#define CODEC_EF_RN5L			0x26

#define CODEC_EF_RD1M			0x27
#define CODEC_EF_RD1L			0x28
#define CODEC_EF_RD2M			0x29
#define CODEC_EF_RD2L			0x2A
#define CODEC_EF_RD4M			0x2B
#define CODEC_EF_RD4L			0x2C
#define CODEC_EF_RD5M			0x2D
#define CODEC_EF_RD5L			0x2E

#define CODEC_DF_RN0M			0x2F
#define CODEC_DF_RN0L			0x30
#define CODEC_DF_RN1M			0x31
#define CODEC_DF_RN1L			0x32

#define CODEC_DF_RD1M			0x33
#define CODEC_DF_RD1L			0x34

#define CODEC_3DAM			0x35
#define CODEC_3DAL			0x36

#define CODEC_LHPN0M			0x41
#define CODEC_LHPN0L			0x42
#define CODEC_LHPN1M			0x43
#define CODEC_LHPN1L			0x44
#define CODEC_LHPD1M			0x45
#define CODEC_LHPD1L			0x46

#define CODEC_RHPN0M			0x47
#define CODEC_RHPN0L			0x48
#define CODEC_RHPN1M			0x49
#define CODEC_RHPN1L			0x4A
#define CODEC_RHPD1M			0x4B
#define CODEC_RHPD1L			0x4C

struct codec_xfer {
	unsigned char page;
	unsigned char reg;
	unsigned char data;
} codec_xfer;

// IOCTL commands for BMI AUDIO driver
#define BMI_AUDIO_RLEDOFF    _IO(BMI_AUDIO_IOCTL, 0x1) // Turn off red LED
#define BMI_AUDIO_RLEDON     _IO(BMI_AUDIO_IOCTL, 0x2) // Turn on red LED
#define BMI_AUDIO_GLEDOFF    _IO(BMI_AUDIO_IOCTL, 0x3) // Turn off green LED
#define BMI_AUDIO_GLEDON     _IO(BMI_AUDIO_IOCTL, 0x4) // Turn on green LED
#define BMI_AUDIO_SPKOFF     _IO(BMI_AUDIO_IOCTL, 0x5) // Turn off speaker
#define BMI_AUDIO_SPKON      _IO(BMI_AUDIO_IOCTL, 0x6) // Turn on speaker
#define BMI_AUDIO_GETSTAT    _IOR(BMI_AUDIO_IOCTL, 0x9, unsigned int *)	// READ IOX register
#define BMI_AUDIO_SETRST     _IO(BMI_AUDIO_IOCTL, 0xA) // Set RESET to '0'
#define BMI_AUDIO_CLRRST     _IO(BMI_AUDIO_IOCTL, 0xB) // Set RESET to '1'
#define BMI_AUDIO_ACTIVATE   _IO(BMI_AUDIO_IOCTL, 0xC) // Activate a module for audio capture
#define BMI_AUDIO_DEACTIVATE _IO(BMI_AUDIO_IOCTL, 0xD) // Deactivate a module for audio capture
#define BMI_AUDIO_WCODEC     _IOW(BMI_AUDIO_IOCTL, 0xE, struct codec_xfer *)	// write CODEC register
#define BMI_AUDIO_RCODEC     _IOR(BMI_AUDIO_IOCTL, 0xF, struct codec_xfer *)	// read CODEC register
#define BMI_AUDIO_SUSPEND    _IO(BMI_AUDIO_IOCTL, 0x10)
#define BMI_AUDIO_RESUME     _IO(BMI_AUDIO_IOCTL, 0x11)

#endif	/* BMI_AUDIO_H */

