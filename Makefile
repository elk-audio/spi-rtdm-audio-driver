obj-m += spi_audio_rtdm.o
spi_audio_rtdm-objs := src/audio_rtdm_driver.o src/audio_rtdm_spi.o
SRC := $(shell pwd)
INCLUDE_DIRS := $(SRC)/audio-control-protocol/include

all:
	$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=${CROSS_COMPILE} -C $(KERNEL_PATH) M=$(SRC) -I $(INCLUDE_DIRS) modules

modules_install:
	$(MAKE) -C $(KERNEL_PATH) M=$(SRC) modules_install

clean:
	$(MAKE) -C $(KERNEL_PATH) M=$(SRC) clean
	@rm -f *.o
	@rm -f *.o.*
