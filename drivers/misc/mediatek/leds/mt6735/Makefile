
obj-$(CONFIG_MTK_LEDS) := leds.o
ccflags-y += -I$(srctree)/drivers/misc/mediatek/video/include
obj-$(CONFIG_MTK_LEDS) += ktd2037.o

ifeq ($(VANZO_DELAY_BACKLIGHT),yes)
subdir-ccflags-y += -DVANZO_DELAY_BACKLIGHT
endif
