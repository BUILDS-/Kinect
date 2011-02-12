#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <stdio.h>
#include "libfreenect_cv.h"

IplImage *GlViewColor(IplImage *depth)
{
	static IplImage *image = 0;
	if (!image) image = cvCreateImage(cvSize(640,480), 8, 3);
	unsigned char *depth_mid = image->imageData;
	int i;
	for (i = 0; i < 640*480; i++) {
		int lb = ((short *)depth->imageData)[i] % 256;
		int ub = ((short *)depth->imageData)[i] / 256;
		switch (ub) {
			case 0:
				depth_mid[3*i+2] = 255;
				depth_mid[3*i+1] = 255-lb;
				depth_mid[3*i+0] = 255-lb;
				break;
			case 1:
				depth_mid[3*i+2] = 255;
				depth_mid[3*i+1] = lb;
				depth_mid[3*i+0] = 0;
				break;
			case 2:
				depth_mid[3*i+2] = 255-lb;
				depth_mid[3*i+1] = 255;
				depth_mid[3*i+0] = 0;
				break;
			case 3:
				depth_mid[3*i+2] = 0;
				depth_mid[3*i+1] = 255;
				depth_mid[3*i+0] = lb;
				break;
			case 4:
				depth_mid[3*i+2] = 0;
				depth_mid[3*i+1] = 255-lb;
				depth_mid[3*i+0] = 255;
				break;
			case 5:
				depth_mid[3*i+2] = 0;
				depth_mid[3*i+1] = 0;
				depth_mid[3*i+0] = 255-lb;
				break;
			default:
				depth_mid[3*i+2] = 0;
				depth_mid[3*i+1] = 0;
				depth_mid[3*i+0] = 0;
				break;
		}
	}
	return image;
}

int main(int argc, char **argv)
{
  int angle = 0;
  while (1) {
    switch(cvWaitKey(10)){

    case 113:
      exit(0);
    case 'w':
      angle++;
      if(angle > 30)
	angle = 30;
      set_tilt_cv(angle,0);
      break;
    case 'x':
      angle--;
      if(angle < -30)
	angle = -30;
      set_tilt_cv(angle,0);
      break;
    case 's':
      angle = 0;
      set_tilt_cv(angle,0);
      break;
    case 'e':
      angle += 10;
      if(angle > 30)
	angle = 30;
      set_tilt_cv(angle,0);
      break;
    case 'c':
      angle -=10;
      if(angle < -30)
	angle = -30;
      set_tilt_cv(angle,0);
      break;
    default:
      ;IplImage *image = freenect_sync_get_rgb_cv(0);
      if (!image) {
	printf("Error: Kinect not connected?\n");
	return -1;
      }
      cvCvtColor(image, image, CV_RGB2BGR);
      cvCircle(image,cvPoint(110,60),35,cvScalar(0, 0, 255, 0),-1, 8, 0);
      IplImage *depth = freenect_sync_get_depth_cv(0);
      if (!depth) {
	printf("Error: Kinect not connected?\n");
	return -1;
      }
      cvShowImage("RGB", image);

      cvShowImage("Depth", GlViewColor(depth));
      break;
    }
  }
}
