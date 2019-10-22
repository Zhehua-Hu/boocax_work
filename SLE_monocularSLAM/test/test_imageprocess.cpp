#include "../include/common.h"

#if TEST_IMAGEPROCESS


int main(){

Mat srcimg = cv::imread("/home/oym/SLE_monocularSLAM/image/2.png",IMREAD_GRAYSCALE);
cout<<srcimg.type()<<endl;
cv::imshow("source_image:",srcimg);
Mat dst = srcimg.clone();
cv::bilateralFilter(srcimg,dst,5,20,40);
cv::imshow("processed_image:",dst);

waitKey(0);

}







#endif
