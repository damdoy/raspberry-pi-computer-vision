CC := gcc
build_files := main.c ../common/image.c ../common/camera_mmal.c ../common/edge_detect.c
output := -o hough
opti := -O2
opti_rpi3 := -march=armv8-a -mtune=cortex-a53
includes := -I /opt/vc/include/ -I/opt/vc/include/interface/mmal/ -L/opt/vc/lib/ -lmmal_util -lmmal_core -lbcm_host -lmmal_vc_client -Wl,--whole-archive -lmmal_components -Wl,--no-whole-archive -lmmal_core -lpthread -lm

all:
	${CC} ${build_files} ${output} ${opti} ${includes}

rpi3:
	${CC} ${build_files} ${output} ${opti} ${opti_rpi3} ${includes}
