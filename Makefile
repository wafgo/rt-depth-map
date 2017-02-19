#CROSS_COMPILE = /opt/petalinux/2016.3/sysroots/x86_64-petalinux-linux/usr/bin/arm-xilinx-linux-gnueabi/arm-xilinx-linux-gnueabi-
SYSROOT = /opt/petalinux/2016.3/sysroots/cortexa9hf-neon-xilinx-linux-gnueabi
ROOTFS := /home/sefo/mounts/nfs/zrfs2/home/root/
CC = $(CROSS_COMPILE)g++

ifeq ($(CROSS_COMPILE),)
CFLAGS += -Iinclude -I/usr/local/include/opencv -I/usr/local/include -L/usr/local/lib -g3 -MMD
else
CFLAGS += --sysroot=$(SYSROOT) -Iinclude -I$(SYSROOT)/usr/include/opencv -I$(SYSROOT)/usr/include -L$(SYSROOT)/usr/lib -march=armv7-a -mfloat-abi=hard -mcpu=cortex-a9
endif

LIBS += -lopencv_stitching -lopencv_superres -lopencv_videostab -lopencv_aruco -lopencv_bgsegm -lopencv_bioinspired \
		-lopencv_ccalib -lopencv_dnn -lopencv_dpm -lopencv_fuzzy -lopencv_line_descriptor -lopencv_optflow -lopencv_plot \
		-lopencv_reg -lopencv_saliency -lopencv_stereo -lopencv_structured_light -lopencv_rgbd -lopencv_surface_matching \
		-lopencv_tracking -lopencv_datasets -lopencv_text -lopencv_face -lopencv_xfeatures2d -lopencv_shape -lopencv_video \
		-lopencv_ximgproc -lopencv_calib3d -lopencv_features2d -lopencv_flann -lopencv_xobjdetect -lopencv_objdetect -lopencv_ml \
		-lopencv_xphoto -lopencv_highgui -lopencv_videoio -lopencv_imgcodecs -lopencv_photo -lopencv_imgproc -lopencv_core -ljpeg


obj-y := main.o mjpeg.o
target := rt-depth-map.elf

.PHONY: all clean

all: $(target)
	
copy_to_target: clean all
ifeq ($(CROSS_COMPILE),)
	@echo "need a cross compile build"
else
	@echo "[CP] $(target) to $(ROOTFS)"
	@cp $(target) $(ROOTFS)
endif
	
$(target): $(obj-y)
	@echo "[LD] $@ from $?"
	@$(CC) $(CFLAGS) -o $@ $(obj-y) $(LIBS)
	
clean:
	@echo "[RM]" 
	@rm -f *.o *.elf *.d
	
%.o: %.cpp
	@echo "[CC] $<"
	@$(CC) $(CFLAGS) -c $< -o $@

%.d : ;

.PRECIOUS: %.d

-include $(obj-y:.o=.d)
