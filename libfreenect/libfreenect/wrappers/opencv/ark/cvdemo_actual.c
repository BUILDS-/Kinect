#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <stdio.h>
#include "libfreenect_cv.h"


void OverlayImage(IplImage* src, IplImage* overlay, CvPoint location, CvScalar S, CvScalar D)

{
  int x;
  int y;
  int i;
  for(x=0;x<overlay->width;x++)

    {

      if(x+location.x>=src->width) continue;

      for(y=0;y<overlay->height;y++)

	{

	  if(y+location.y>=src->height) continue;

	  CvScalar source = cvGet2D(src, y+location.y, x+location.x);

	  CvScalar over = cvGet2D(overlay, y, x);



	  CvScalar merged;

	  for(i=0;i<4;i++)

	    merged.val[i] = (S.val[i]*source.val[i]+D.val[i]*over.val[i]);



	  cvSet2D(src, y+location.y, x+location.x, merged);

	}

    }

}

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
  int first = 1;
  int angle = 0;
  double duration = 5;
  IplImage *image = 0;
  IplImage *image2 = 0;
  IplImage *prev = 0;
  IplImage *output = 0;
  IplImage *depth;
  IplImage *diff = 0;
  IplImage *bw = 0;
  IplImage *edge = 0;
  IplImage *edge2 = 0;
  //  if (!prev) prev = cvCreateImageHeader(cvSize(640,480), 8, 3);
  //if (!diff) diff = cvCreateImageHeader(cvSize(640,480), 8, 3);
  diff = cvCreateImage(cvSize(640,480),8,3);
  bw = cvCreateImage(cvSize(640,480),8,1);
  edge = cvCreateImage(cvSize(640,480),8,1);
  edge2 = cvCreateImage(cvSize(640,480),8,1);
  output  = cvCreateImage(cvSize(640,480),8,3);
  cvZero( output );
  
  //cvCvtColor(output, output, CV_RGB2BGR);
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
      // cvWaitKey(700);
      if(first){
	prev = freenect_sync_get_rgb_cv(0);
	//first = 0;
      }
      else
	{
	  prev = cvCloneImage(image2);
	  cvReleaseImage(&image2); 
	}
      image  = freenect_sync_get_rgb_cv(0);
      image2  = cvCloneImage(image);
      if (!image) {
	printf("Error: Kinect not connected?\n");
	return -1;
      }
      cvCvtColor(image, image, CV_RGB2BGR);
      //      cvCvtColor(image2, image2, CV_RGB2BGR);

      cvAbsDiff(image,prev,diff);
      cvCvtColor(diff, bw,CV_BGR2GRAY);
      cvCanny(bw, edge, 29000,30500,7);
      //      cvThreshold(bw,bw,100,254,CV_THRESH_BINARY);
      cvNot(edge,edge2);

      if(!first)
	{
	  cvSubS(output,cvScalar(255,255,255,255),output,0);
	  cvAnd(GlViewColor(depth),GlViewColor(depth),output,edge);
	  //cvRunningAvg(GlViewColor(depth),output,1,edge);
	}
      //cvRunningAvg(image,output,1,edge);

      if(!first)
	{
	  cvReleaseImage(&prev);
	}
      else
	first = 0;
      cvAddWeighted(image, .3, output, .7, 1, image);
      
      //       OverlayImage(image2, output, cvPoint(0, 0), cvScalar(0.8,0.8,0.8,0.8), cvScalar(0.2,0.2,0.2,0.2));
      /*
      CvPoint* points[1];
      CvPoint ptt[5];
      points[0] = &(ptt[0]);
      points[0][0] = cvPoint(100,100);
      points[0][1] = cvPoint(200,100);
      points[0][2] = cvPoint(150,150);
      points[0][3] = cvPoint(150,300);
      points[0][4] = cvPoint(100,250);

      int npts[1];
      npts[0]=5;
	cvPolyLine(image, points, npts, 1,1, cvScalar(100,100,100,230),1, 8,0);
	cvFillPoly(image, points, npts,1, cvScalar(100,100,100,230), 8,0);
      */
      depth = freenect_sync_get_depth_cv(0);
      cvSmooth(depth,depth,CV_BLUR,18,18,2.0,2.0);
      if (!depth) {
	printf("Error: Kinect not connected?\n");
	return -1;
      }
      cvShowImage("RGB", image);
      cvShowImage("Output", output);

      cvShowImage("Depth", GlViewColor(depth));
      break;
    }
  }
}
