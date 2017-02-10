CC = g++
CFLAGS += -Iinclude -I/usr/local/include/opencv -I/usr/local/include -L/usr/local/lib -g3
LIBS += -lopencv_stitching -lopencv_superres -lopencv_videostab -lopencv_aruco -lopencv_bgsegm -lopencv_bioinspired \
		-lopencv_ccalib -lopencv_dnn -lopencv_dpm -lopencv_fuzzy -lopencv_line_descriptor -lopencv_optflow -lopencv_plot \
		-lopencv_reg -lopencv_saliency -lopencv_stereo -lopencv_structured_light -lopencv_rgbd -lopencv_surface_matching \
		-lopencv_tracking -lopencv_datasets -lopencv_text -lopencv_face -lopencv_xfeatures2d -lopencv_shape -lopencv_video \
		-lopencv_ximgproc -lopencv_calib3d -lopencv_features2d -lopencv_flann -lopencv_xobjdetect -lopencv_objdetect -lopencv_ml \
		-lopencv_xphoto -lopencv_highgui -lopencv_videoio -lopencv_imgcodecs -lopencv_photo -lopencv_imgproc -lopencv_core -ljpeg


obj-y := main.o mjpeg.o
target := rt-depth-map.elf

all: $(obj-y)
	$(CC) $(CFLAGS) -o $(target) $(obj-y) $(LIBS)
	
clean:
	rm -f *.o *.elf *.jpg *.yml
	
%.o: %.cpp
	$(CC) $(CFLAGS) -c $< -o $@