
KERN_DIR := $(HOME)/raspberrypi/linux
ARCH ?= arm64
CROSS_COMPILE ?= aarch64-linux-gnu-

MODULE_NAME:=panel-boe-zv039wvq-n80

all:
	make -C $(KERN_DIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) M=`pwd` modules
	dtc -@ -Hepapr -I dts -O dtb -o vc4-kms-dsi-boe-zv039wvq-n80-overlay.dtbo vc4-kms-dsi-boe-zv039wvq-n80-overlay.dts
clean:
	make -C $(KERN_DIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) M=`pwd` modules clean
	rm -rf *.dtbo

obj-m += $(MODULE_NAME).o
#$(MODULE_NAME)-y += anel-boe-zv039wvq-n80.o
