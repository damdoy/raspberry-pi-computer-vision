# FAST feature detection on Raspberry Pi

Simple and easy FAST feature detection implementation

The features points are found on multiple pyramid levels and then displayed on the captured image as red circles of different sizes depending on the pyramid level. The image is then displayed on the framebuffer.

Performance: 8Hz at 800x600 using one core on a Raspberry Pi 3 on 3 pyramid levels

![example](example.jpg)