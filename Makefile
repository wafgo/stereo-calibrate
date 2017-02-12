CC = g++
CAM = ELP
#CAM = ZED
CFLAGS += -D$(CAM) -I/usr/local/include/opencv -I/usr/local/include -L/usr/local/lib -MMD
LIBS += -lopencv_stitching -lopencv_superres -lopencv_videostab -lopencv_aruco -lopencv_bgsegm -lopencv_bioinspired \
		-lopencv_ccalib -lopencv_dnn -lopencv_dpm -lopencv_fuzzy -lopencv_line_descriptor -lopencv_optflow -lopencv_plot \
		-lopencv_reg -lopencv_saliency -lopencv_stereo -lopencv_structured_light -lopencv_rgbd -lopencv_surface_matching \
		-lopencv_tracking -lopencv_datasets -lopencv_text -lopencv_face -lopencv_xfeatures2d -lopencv_shape -lopencv_video \
		-lopencv_ximgproc -lopencv_calib3d -lopencv_features2d -lopencv_flann -lopencv_xobjdetect -lopencv_objdetect -lopencv_ml \
		-lopencv_xphoto -lopencv_highgui -lopencv_videoio -lopencv_imgcodecs -lopencv_photo -lopencv_imgproc -lopencv_core -ljpeg


obj-y := calibrate.o mjpeg.o
target := stereo-calibrate.elf
cal_dir := $(addprefix CAL_,$(shell date '+%d-%b-%Y-%H-%M-%s'))
backup_dir := backup/$(CAM)/$(cal_dir)

.PHONY: all create_backup clean

all: $(target)

$(target): $(obj-y)
	@echo "[LD] $@ from $?"
	@$(CC) $(CFLAGS) -o $(target) $(obj-y) $(LIBS)
	
create_backup:
	@echo "[BACKUP] to $(backup_dir)"
	@mkdir $(backup_dir) && cp *.jpg extrinsics.yml intrinsics.yml $(backup_dir)
	
clean:
	@echo [RM]
	@rm -f *.o *.elf *.jpg *.yml *.d
	
%.o: %.cpp
	@echo "[CC] $<"
	@$(CC) $(CFLAGS) -c $< -o $@
	
%.d : ;

.PRECIOUS: %.d

-include $(obj-y:.o=.d)
