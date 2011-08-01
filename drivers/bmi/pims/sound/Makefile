#
# BMI PIMS
#

obj-$(CONFIG_BMI_AUDIO)		+= bmi_audio.o

all:
	make -C ../../../.. M=$(PWD) CONFIG_BMI_AUDIO=m modules

clean:
	make -C ../../../.. M=$(PWD) CONFIG_BMI_AUDIO=m clean


upload: all
	scp -qB bmi_audio.ko bug:
