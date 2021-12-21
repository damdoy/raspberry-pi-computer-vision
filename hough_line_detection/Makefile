all:
	gcc main.c ../common/image.c ../common/edge_detect.c -o hough -O2 -I/opt/vc/include/ -I/opt/vc/include/interface/mmal/ -L/opt/vc/lib/ -lmmal_util -lmmal_core -lbcm_host -lmmal_vc_client -Wl,--whole-archive -lmmal_components -Wl,--no-whole-archive -lmmal_core -lpthread -lm
