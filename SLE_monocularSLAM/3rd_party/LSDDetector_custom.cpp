/*M///////////////////////////////////////////////////////////////////////////////////////
 //
 //  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
 //
 //  By downloading, copying, installing or using the software you agree to this license.
 //  If you do not agree to this license, do not download, install,
 //  copy or use the software.
 //
 //
 //                           License Agreement
 //                For Open Source Computer Vision Library
 //
 // Copyright (C) 2014, Biagio Montesano, all rights reserved.
 // Third party copyrights are property of their respective owners.
 //
 // Redistribution and use in source and binary forms, with or without modification,
 // are permitted provided that the following conditions are met:
 //
 //   * Redistribution's of source code must retain the above copyright notice,
 //     this list of conditions and the following disclaimer.
 //
 //   * Redistribution's in binary form must reproduce the above copyright notice,
 //     this list of conditions and the following disclaimer in the documentation
 //     and/or other materials provided with the distribution.
 //
 //   * The name of the copyright holders may not be used to endorse or promote products
 //     derived from this software without specific prior written permission.
 //
 // This software is provided by the copyright holders and contributors "as is" and
 // any express or implied warranties, including, but not limited to, the implied
 // warranties of merchantability and fitness for a particular purpose are disclaimed.
 // In no event shall the Intel Corporation or contributors be liable for any direct,
 // indirect, incidental, special, exemplary, or consequential damages
 // (including, but not limited to, procurement of substitute goods or services;
 // loss of use, data, or profits; or business interruption) however caused
 // and on any theory of liability, whether in contract, strict liability,
 // or tort (including negligence or otherwise) arising in any way out of
 // the use of this software, even if advised of the possibility of such damage.
 //
 //M*/

#include "include/precomp_custom.hpp"
#include <opencv2/core/hal/hal.hpp>


//using namespace cv;
namespace cv
{
namespace line_descriptor
{

Ptr<LSDDetectorC> LSDDetectorC::createLSDDetectorC()
{
  return Ptr<LSDDetectorC>( new LSDDetectorC() );
}

/* compute Gaussian pyramid of input image */
void LSDDetectorC::computeGaussianPyramid( const Mat& image, int numOctaves, int scale )
{
  /* clear class fields */
  gaussianPyrs.clear();

  /* insert input image into pyramid */
  cv::Mat currentMat = image.clone();
  //cv::GaussianBlur( currentMat, currentMat, cv::Size( 5, 5 ), 1 );
  gaussianPyrs.push_back( currentMat );

  /* fill Gaussian pyramid */
  for ( int pyrCounter = 1; pyrCounter < numOctaves; pyrCounter++ )
  {
    /* compute and store next image in pyramid and its size */
    pyrDown( currentMat, currentMat, Size( currentMat.cols / scale, currentMat.rows / scale ) );
    gaussianPyrs.push_back( currentMat );
  }
}

/* check lines' extremes */
inline void checkLineExtremes( cv::Vec4f& extremes, cv::Size imageSize )
{

  if( extremes[0] < 0 )
    extremes[0] = 0;

  if( extremes[0] >= imageSize.width )
    extremes[0] = (float)imageSize.width - 1.0f;

  if( extremes[2] < 0 )
    extremes[2] = 0;

  if( extremes[2] >= imageSize.width )
    extremes[2] = (float)imageSize.width - 1.0f;

  if( extremes[1] < 0 )
    extremes[1] = 0;

  if( extremes[1] >= imageSize.height )
    extremes[1] = (float)imageSize.height - 1.0f;

  if( extremes[3] < 0 )
    extremes[3] = 0;

  if( extremes[3] >= imageSize.height )
    extremes[3] = (float)imageSize.height - 1.0f;
}

/* requires line detection (only one image) */
void LSDDetectorC::detect( const Mat& image, CV_OUT std::vector<KeyLine>& keylines, int scale, int numOctaves, const Mat& mask )
{
  if( mask.data != NULL && ( mask.size() != image.size() || mask.type() != CV_8UC1 ) )
    throw std::runtime_error( "Mask error while detecting lines: please check its dimensions and that data type is CV_8UC1" );

  else
    detectImpl( image, keylines, numOctaves, scale, mask );
}

/* requires line detection (more than one image) */
void LSDDetectorC::detect( const std::vector<Mat>& images, std::vector<std::vector<KeyLine> >& keylines, int scale, int numOctaves,
                          const std::vector<Mat>& masks ) const
{
  /* detect lines from each image */
  for ( size_t counter = 0; counter < images.size(); counter++ )
  {
    if( masks[counter].data != NULL && ( masks[counter].size() != images[counter].size() || masks[counter].type() != CV_8UC1 ) )
      throw std::runtime_error( "Masks error while detecting lines: please check their dimensions and that data types are CV_8UC1" );

    else
      detectImpl( images[counter], keylines[counter], numOctaves, scale, masks[counter] );
  }
}

/* implementation of line detection */
void LSDDetectorC::detectImpl( const Mat& imageSrc, std::vector<KeyLine>& keylines, int numOctaves, int scale, const Mat& mask ) const
{
  cv::Mat image;
  if( imageSrc.channels() != 1 )
    cvtColor( imageSrc, image, COLOR_BGR2GRAY );
  else
    image = imageSrc.clone();

  /*check whether image depth is different from 0 */
  if( image.depth() != 0 )
    throw std::runtime_error( "Error, depth image!= 0" );

  /* create a pointer to self */
  LSDDetectorC *lsd = const_cast<LSDDetectorC*>( this );

  /* compute Gaussian pyramids */
  lsd->computeGaussianPyramid( image, numOctaves, scale );

  /* create an LSD extractor */
  cv::Ptr<cv::LineSegmentDetector> ls = cv::createLineSegmentDetector();

  /* prepare a vector to host extracted segments */
  std::vector<std::vector<cv::Vec4f> > lines_lsd;

  /* extract lines */
  for ( int i = 0; i < numOctaves; i++ )
  {
    std::vector<Vec4f> octave_lines;
    ls->detect( gaussianPyrs[i], octave_lines );
    lines_lsd.push_back( octave_lines );
  }

  /* create keylines */
  int class_counter = -1;
  for ( int octaveIdx = 0; octaveIdx < (int) lines_lsd.size(); octaveIdx++ )
  {
    float octaveScale = pow( (float)scale, octaveIdx );
    for ( int k = 0; k < (int) lines_lsd[octaveIdx].size(); k++ )
    {
      KeyLine kl;
      cv::Vec4f extremes = lines_lsd[octaveIdx][k];

      /* check data validity */
      checkLineExtremes( extremes, gaussianPyrs[octaveIdx].size() );

      /* fill KeyLine's fields */
      kl.startPointX = extremes[0] * octaveScale;
      kl.startPointY = extremes[1] * octaveScale;
      kl.endPointX = extremes[2] * octaveScale;
      kl.endPointY = extremes[3] * octaveScale;
      kl.sPointInOctaveX = extremes[0];
      kl.sPointInOctaveY = extremes[1];
      kl.ePointInOctaveX = extremes[2];
      kl.ePointInOctaveY = extremes[3];
      kl.lineLength = (float) sqrt( pow( extremes[0] - extremes[2], 2 ) + pow( extremes[1] - extremes[3], 2 ) );

      /* compute number of pixels covered by line */
      LineIterator li( gaussianPyrs[octaveIdx], Point2f( extremes[0], extremes[1] ), Point2f( extremes[2], extremes[3] ) );
      kl.numOfPixels = li.count;

      kl.angle = atan2( ( kl.endPointY - kl.startPointY ), ( kl.endPointX - kl.startPointX ) );
      kl.class_id = ++class_counter;
      kl.octave = octaveIdx;
      kl.size = ( kl.endPointX - kl.startPointX ) * ( kl.endPointY - kl.startPointY );
      kl.response = kl.lineLength / max( gaussianPyrs[octaveIdx].cols, gaussianPyrs[octaveIdx].rows );
      kl.pt = Point2f( ( kl.endPointX + kl.startPointX ) / 2, ( kl.endPointY + kl.startPointY ) / 2 );

      keylines.push_back( kl );
    }
  }

  /* delete undesired KeyLines, according to input mask */
  if( !mask.empty() )
  {
    for ( size_t keyCounter = 0; keyCounter < keylines.size(); keyCounter++ )
    {
      KeyLine kl = keylines[keyCounter];
      if( mask.at<uchar>( (int) kl.startPointY, (int) kl.startPointX ) == 0 && mask.at<uchar>( (int) kl.endPointY, (int) kl.endPointX ) == 0 )
      {
        keylines.erase( keylines.begin() + keyCounter );
        keyCounter--;
      }
    }
  }

}

// Overload detect and detectImpl with LSDDetectorC Options
void LSDDetectorC::detect( const Mat& image, CV_OUT std::vector<KeyLine>& keylines, int scale, int numOctaves, LSDOptions opts, const Mat& mask )
{
  if( mask.data != NULL && ( mask.size() != image.size() || mask.type() != CV_8UC1 ) )
    throw std::runtime_error( "Mask error while detecting lines: please check its dimensions and that data type is CV_8UC1" );

  else
    detectImpl( image, keylines, numOctaves, scale, opts, mask );
}



void LSDDetectorC::draw_line(Mat imageSrc,std::vector<KeyLine>& keylines )
{
    RNG rng(0);

    cv::Mat temp_img;

    String title;

    if(imageSrc.channels()!=1)

    {

    cv::Scalar s=cv::Scalar(0,0,0);

    cv::Mat T_img(imageSrc.size(),CV_8UC3,s);

    temp_img=T_img.clone();

    title = "黑幕";

    }

    else

    {

   temp_img=imageSrc.clone();

   title = "原圖";

    }



    std::for_each( keylines.begin(), keylines.end(), [&](KeyLine kl)
    {

    cv::Scalar color = cv::Scalar(255, 255, 255);

    //cv::Scalar color = cv::Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255));

    cv::line(temp_img,cv::Point(kl.startPointX,kl.startPointY),cv::Point(kl.endPointX,kl.endPointY),color,1.5);

    cv::circle(temp_img,cv::Point(kl.startPointX,kl.startPointY),3,cv::Scalar(255,255,255),1);

    cv::circle(temp_img,cv::Point(kl.endPointX,kl.endPointY),3,cv::Scalar(0,255,0),1);

    //  std::cout<<color(1)<<"  "<<color(2)<<"   "<<color(3)<<std::endl;

    });

    cv::imshow(title,temp_img);
}





/*

void LSDDetectorC::calculate_sift(const Mat &imageSrc, const Point2i &pt, std::vector<KeyPoint> &keypoints)
{
    int ratio = 5;
    std::for_each(keypoints.begin(),keypoints.end(), [&](KeyPoint kp)
    {
       if(kp.pt.x>5 && kp.pt.y>5 && kp.pt.x<imageSrc.cols-5 && kp.pt.y < imageSrc.rows-5)
       {

           cv::Mat temp_img(2*ratio+1,2*ratio+1,CV_8UC1);



           for (int i=-ratio;i<ratio+1;i++)

               for (int j=-ratio;j<ratio+1;j++)
               {

                   temp_img.at<int>(i,j)=imageSrc.at<int>(kp.pt.x-ratio+i,kp.pt.y-ratio+j);
               }



        }


    });

}


static const float SIFT_INT_DESCR_FCTR = 512.f;
static const float SIFT_DESCR_SCL_FCTR = 3.f;


 void LSDDetectorC::calcSIFTDescriptor(const Mat &img, Point2f ptf, float ori, float scl, int d, int n, float *dst)
{


    Point pt(cvRound(ptf.x), cvRound(ptf.y));
    float cos_t = cosf(ori*(float)(CV_PI/180));
    float sin_t = sinf(ori*(float)(CV_PI/180));
    float bins_per_rad = n / 360.f;
    float exp_scale = -1.f/(d * d * 0.5f);
    float hist_width = SIFT_DESCR_SCL_FCTR * scl;
    int radius = cvRound(hist_width * 1.4142135623730951f * (d + 1) * 0.5f);
    // Clip the radius to the diagonal of the image to avoid autobuffer too large exception
    radius = std::min(radius, (int) sqrt(((double) img.cols)*img.cols + ((double) img.rows)*img.rows));
    cos_t /= hist_width;
    sin_t /= hist_width;

    int i, j, k, len = (radius*2+1)*(radius*2+1), histlen = (d+2)*(d+2)*(n+2);
    int rows = img.rows, cols = img.cols;

    AutoBuffer<float> buf(len*6 + histlen);
    float *X = buf, *Y = X + len, *Mag = Y, *Ori = Mag + len, *W = Ori + len;
    float *RBin = W + len, *CBin = RBin + len, *hist = CBin + len;

    for( i = 0; i < d+2; i++ )
    {
        for( j = 0; j < d+2; j++ )
            for( k = 0; k < n+2; k++ )
                hist[(i*(d+2) + j)*(n+2) + k] = 0.;
    }

    for( i = -radius, k = 0; i <= radius; i++ )
        for( j = -radius; j <= radius; j++ )
        {
            // Calculate sample's histogram array coords rotated relative to ori.
            // Subtract 0.5 so samples that fall e.g. in the center of row 1 (i.e.
            // r_rot = 1.5) have full weight placed in row 1 after interpolation.
            float c_rot = j * cos_t - i * sin_t;
            float r_rot = j * sin_t + i * cos_t;
            float rbin = r_rot + d/2 - 0.5f;
            float cbin = c_rot + d/2 - 0.5f;
            int r = pt.y + i, c = pt.x + j;

            if( rbin > -1 && rbin < d && cbin > -1 && cbin < d &&
                r > 0 && r < rows - 1 && c > 0 && c < cols - 1 )
            {
                float dx = (float)(img.at<sift_wt>(r, c+1) - img.at<sift_wt>(r, c-1));
                float dy = (float)(img.at<sift_wt>(r-1, c) - img.at<sift_wt>(r+1, c));
                X[k] = dx; Y[k] = dy; RBin[k] = rbin; CBin[k] = cbin;
                W[k] = (c_rot * c_rot + r_rot * r_rot)*exp_scale;
                k++;
            }
        }

    len = k;
    cv::hal::fastAtan2(Y, X, Ori, len, true);
    cv::hal::magnitude32f(X, Y, Mag, len);
    cv::hal::exp32f(W, W, len);

    for( k = 0; k < len; k++ )
    {
        float rbin = RBin[k], cbin = CBin[k];
        float obin = (Ori[k] - ori)*bins_per_rad;
        float mag = Mag[k]*W[k];

        int r0 = cvFloor( rbin );
        int c0 = cvFloor( cbin );
        int o0 = cvFloor( obin );
        rbin -= r0;
        cbin -= c0;
        obin -= o0;

        if( o0 < 0 )
            o0 += n;
        if( o0 >= n )
            o0 -= n;

        // histogram update using tri-linear interpolation
        float v_r1 = mag*rbin, v_r0 = mag - v_r1;
        float v_rc11 = v_r1*cbin, v_rc10 = v_r1 - v_rc11;
        float v_rc01 = v_r0*cbin, v_rc00 = v_r0 - v_rc01;
        float v_rco111 = v_rc11*obin, v_rco110 = v_rc11 - v_rco111;
        float v_rco101 = v_rc10*obin, v_rco100 = v_rc10 - v_rco101;
        float v_rco011 = v_rc01*obin, v_rco010 = v_rc01 - v_rco011;
        float v_rco001 = v_rc00*obin, v_rco000 = v_rc00 - v_rco001;

        int idx = ((r0+1)*(d+2) + c0+1)*(n+2) + o0;
        hist[idx] += v_rco000;
        hist[idx+1] += v_rco001;
        hist[idx+(n+2)] += v_rco010;
        hist[idx+(n+3)] += v_rco011;
        hist[idx+(d+2)*(n+2)] += v_rco100;
        hist[idx+(d+2)*(n+2)+1] += v_rco101;
        hist[idx+(d+3)*(n+2)] += v_rco110;
        hist[idx+(d+3)*(n+2)+1] += v_rco111;
    }

    // finalize histogram, since the orientation histograms are circular
    for( i = 0; i < d; i++ )
        for( j = 0; j < d; j++ )
        {
            int idx = ((i+1)*(d+2) + (j+1))*(n+2);
            hist[idx] += hist[idx+n];
            hist[idx+1] += hist[idx+n+1];
            for( k = 0; k < n; k++ )
                dst[(i*d + j)*n + k] = hist[idx+k];
        }
    // copy histogram to the descriptor,
    // apply hysteresis thresholding
    // and scale the result, so that it can be easily converted
    // to byte array
    float nrm2 = 0;
    len = d*d*n;
    for( k = 0; k < len; k++ )
        nrm2 += dst[k]*dst[k];
    float thr = std::sqrt(nrm2)*SIFT_DESCR_MAG_THR;
    for( i = 0, nrm2 = 0; i < k; i++ )
    {
        float val = std::min(dst[i], thr);
        dst[i] = val;
        nrm2 += val*val;
    }
    nrm2 = SIFT_INT_DESCR_FCTR/std::max(std::sqrt(nrm2), FLT_EPSILON);

#if 1
    for( k = 0; k < len; k++ )
    {
        dst[k] = saturate_cast<uchar>(dst[k]*nrm2);
    }
#else
    float nrm1 = 0;
    for( k = 0; k < len; k++ )
    {
        dst[k] *= nrm2;
        nrm1 += dst[k];
    }
    nrm1 = 1.f/std::max(nrm1, FLT_EPSILON);
    for( k = 0; k < len; k++ )
    {
        dst[k] = std::sqrt(dst[k] * nrm1);//saturate_cast<uchar>(std::sqrt(dst[k] * nrm1)*SIFT_INT_DESCR_FCTR);
    }
#endif
}


static const int SIFT_DESCR_WIDTH = 4;

static const int SIFT_DESCR_HIST_BINS = 8;

void LSDDetectorC::calcDescriptors(const std::vector<Mat> &gpyr, const std::vector<KeyPoint> &keypoints, Mat &descriptors, int nOctaveLayers, int firstOctave)
{
    int d = SIFT_DESCR_WIDTH, n = SIFT_DESCR_HIST_BINS;

    for( size_t i = 0; i < keypoints.size(); i++ )
    {
        KeyPoint kpt = keypoints[i];
        int octave, layer;
        float scale;
        unpackOctave(kpt, octave, layer, scale);
        CV_Assert(octave >= firstOctave && layer <= nOctaveLayers+2);
        float size=kpt.size*scale;
        Point2f ptf(kpt.pt.x*scale, kpt.pt.y*scale);
        const Mat& img = gpyr[(octave - firstOctave)*(nOctaveLayers + 3) + layer];

        float angle = 360.f - kpt.angle;
        if(std::abs(angle - 360.f) < FLT_EPSILON)
            angle = 0.f;
        calcSIFTDescriptor(img, ptf, angle, size*0.5f, d, n, descriptors.ptr<float>((int)i));
    }
}


*/

float LSDDetectorC::calcOrientationHist( const Mat& img, Point pt, int radius,
                                   float sigma, float* hist, int n )
{
    int i, j, k, len = (radius*2+1)*(radius*2+1);

    float expf_scale = -1.f/(2.f * sigma * sigma);
    AutoBuffer<float> buf(len*4 + n+4);
    float *X = buf, *Y = X + len, *Mag = X, *Ori = Y + len, *W = Ori + len;
    float* temphist = W + len + 2;

    for( i = 0; i < n; i++ )
        temphist[i] = 0.f;

    for( i = -radius, k = 0; i <= radius; i++ )
    {
        int y = pt.y + i;
        if( y <= 0 || y >= img.rows - 1 )
            continue;
        for( j = -radius; j <= radius; j++ )
        {
            int x = pt.x + j;
            if( x <= 0 || x >= img.cols - 1 )
                continue;

            float dx = (float)(img.at<float>(y, x+1) - img.at<float>(y, x-1));
            float dy = (float)(img.at<float>(y-1, x) - img.at<float>(y+1, x));
            float xx = (float)(img.at<float>(y, x+1) );
            float yy = (float)(img.at<float>(y-1, x) );

            X[k] = dx; Y[k] = dy; W[k] = (i*i + j*j)*expf_scale;
            k++;
        }
    }

    len = k;

    // compute gradient values, orientations and the weights over the pixel neighborhood
    cv::hal::exp32f(W, W, len);
    cv::hal::fastAtan2(Y, X, Ori, len, true);
    cv::hal::magnitude32f(X, Y, Mag, len);

    for( k = 0; k < len; k++ )
    {
        int bin = cvRound((n/360.f)*Ori[k]);
        if( bin >= n )
            bin -= n;
        if( bin < 0 )
            bin += n;
        temphist[bin] += W[k]*Mag[k];
    }

    // smooth the histogram
    temphist[-1] = temphist[n-1];
    temphist[-2] = temphist[n-2];
    temphist[n] = temphist[0];
    temphist[n+1] = temphist[1];
    for( i = 0; i < n; i++ )
    {
        hist[i] = (temphist[i-2] + temphist[i+2])*(1.f/16.f) +
            (temphist[i-1] + temphist[i+1])*(4.f/16.f) +
            temphist[i]*(6.f/16.f);
    }

    float maxval = hist[0];
    for( i = 1; i < n; i++ )
        maxval = std::max(maxval, hist[i]);

    float SIFT_ORI_PEAK_RATIO = 0.8;

    float mag_thr = (float)(maxval * SIFT_ORI_PEAK_RATIO);

    float angle = 0.0f;
    for( int j = 0; j < n; j++ )
    {
        int l = j > 0 ? j - 1 : n - 1;
        int r2 = j < n-1 ? j + 1 : 0;

        if( hist[j] > hist[l]  &&  hist[j] > hist[r2]  &&  hist[j] >= mag_thr )
        {
            float bin = j + 0.5f * (hist[l]-hist[r2]) / (hist[l] - 2*hist[j] + hist[r2]);
            bin = bin < 0 ? n + bin : bin >= n ? bin - n : bin;
            angle = 360.f - (float)((360.f/n) * bin);
            if(std::abs(angle - 360.f) < FLT_EPSILON)
               angle = 0.f;
        }
    }
  std::cout<<"angle = "<<angle<<std::endl;
       return maxval;
}







void LSDDetectorC::detectImpl( const Mat& imageSrc, std::vector<KeyLine>& keylines, int numOctaves, int scale, LSDOptions opts, const Mat& mask ) const
{
  cv::Mat image;
  if( imageSrc.channels() != 1 )
    cvtColor( imageSrc, image, COLOR_BGR2GRAY );
  else
    image = imageSrc.clone();

  /*check whether image depth is different from 0 */
  if( image.depth() != 0 )
    throw std::runtime_error( "Error, depth image!= 0" );

  /* create a pointer to self */
  LSDDetectorC *lsd = const_cast<LSDDetectorC*>( this );

  /* compute Gaussian pyramids */
  lsd->computeGaussianPyramid( image, numOctaves, scale );

  /* create an LSD extractor */

//  cv::Ptr<cv::line_descriptor::LSDDetectorC> ls = cv::line_descriptor::LSDDetectorC::createLSDDetectorC( opts.refine,
//                                                                       opts.scale,
//                                                                       opts.sigma_scale,
//                                                                       opts.quant,
//                                                                       opts.ang_th,
//                                                                       opts.log_eps,
//                                                                       opts.density_th,
//                                                                       opts.n_bins);

 // static Ptr<LSDDetector> cv::line_descriptor::LSDDetector::createLSDDetector 	( 	LSDParam  	params	);


  cv::Ptr<cv::LineSegmentDetector> ls = cv::createLineSegmentDetector( opts.refine,
                                                                       opts.scale,
                                                                       opts.sigma_scale,
                                                                       opts.quant,
                                                                       opts.ang_th,
                                                                       opts.log_eps,
                                                                       opts.density_th,
                                                                       opts.n_bins);

  /* prepare a vector to host extracted segments */
  std::vector<std::vector<cv::Vec4f> > lines_lsd;

  /* extract lines */
  for ( int i = 0; i < numOctaves; i++ )
  {
    std::vector<Vec4f> octave_lines;
    ls->detect( gaussianPyrs[i], octave_lines );
    lines_lsd.push_back( octave_lines );
  }

  /* create keylines */
  int class_counter = -1;
  for ( int octaveIdx = 0; octaveIdx < (int) lines_lsd.size(); octaveIdx++ )
  {
    float octaveScale = pow( (float)scale, octaveIdx );
    for ( int k = 0; k < (int) lines_lsd[octaveIdx].size(); k++ )
    {
      KeyLine kl;
      cv::Vec4f extremes = lines_lsd[octaveIdx][k];

      /* check data validity */
      checkLineExtremes( extremes, gaussianPyrs[octaveIdx].size() );

      /* check line segment min length */
      double length = (float) sqrt( pow( extremes[0] - extremes[2], 2 ) + pow( extremes[1] - extremes[3], 2 ) );
      if( length > opts.min_length )
      {
          /* fill KeyLine's fields */
          kl.startPointX = extremes[0] * octaveScale;
          kl.startPointY = extremes[1] * octaveScale;
          kl.endPointX   = extremes[2] * octaveScale;
          kl.endPointY   = extremes[3] * octaveScale;
          kl.sPointInOctaveX = extremes[0];
          kl.sPointInOctaveY = extremes[1];
          kl.ePointInOctaveX = extremes[2];
          kl.ePointInOctaveY = extremes[3];
          kl.lineLength = length;
          kl.O_endPointX=kl.endPointX-image.cols;
          kl.O_endPointY=kl.endPointY-image.cols;
          kl.O_endPointX=kl.endPointX-image.cols;
          kl.O_endPointX=kl.endPointX-image.cols;



          /* compute number of pixels covered by line */
          LineIterator li( gaussianPyrs[octaveIdx], Point2f( extremes[0], extremes[1] ), Point2f( extremes[2], extremes[3] ) );
          kl.numOfPixels = li.count;

          kl.angle = atan2( ( kl.endPointY - kl.startPointY ), ( kl.endPointX - kl.startPointX ) );
          kl.class_id = ++class_counter;
          kl.octave = octaveIdx;
          kl.size = ( kl.endPointX - kl.startPointX ) * ( kl.endPointY - kl.startPointY );
          kl.response = kl.lineLength / max( gaussianPyrs[octaveIdx].cols, gaussianPyrs[octaveIdx].rows );
          kl.pt = Point2f( ( kl.endPointX + kl.startPointX ) / 2, ( kl.endPointY + kl.startPointY ) / 2 );

          keylines.push_back( kl );
      }
    }
  }



  /* delete undesired KeyLines, according to input mask */
  if( !mask.empty() )
  {
    for ( size_t keyCounter = 0; keyCounter < keylines.size(); keyCounter++ )
    {
      KeyLine kl = keylines[keyCounter];
      if( mask.at<uchar>( (int) kl.startPointY, (int) kl.startPointX ) == 0 && mask.at<uchar>( (int) kl.endPointY, (int) kl.endPointX ) == 0 )
      {
        keylines.erase( keylines.begin() + keyCounter );
        keyCounter--;
      }
    }
  }

}



void LSDDetectorC::detectline( std::vector<Mat>& gpyr, std::vector<KeyLine>& keylines, LSDOptions opts ) const
{
  /* create a pointer to self */
  LSDDetectorC *lsd = const_cast<LSDDetectorC*>( this );


  /* create an LSD extractor */

  cv::Ptr<cv::LineSegmentDetector> ls = cv::createLineSegmentDetector( opts.refine,
                                                                       opts.scale,
                                                                       opts.sigma_scale,
                                                                       opts.quant,
                                                                       opts.ang_th,
                                                                       opts.log_eps,
                                                                       opts.density_th,
                                                                       opts.n_bins
                                                                        );

  /* prepare a vector to host extracted segments */
  /* extract lines */
    std::vector<std::vector<cv::Vec4f> > lines_lsd;

    /* extract lines */
    for ( int i = 0; i < gpyr.size(); i++ )
    {
      std::vector<Vec4f> octave_lines;
      ls->detect( gpyr[i], octave_lines );
      lines_lsd.push_back( octave_lines );
    }


  /* create keylines */

  for ( int octaveIdx = 0; octaveIdx < (int) lines_lsd.size(); octaveIdx++ )
  {
  // float octaveScale = pow( (float)scale, octaveIdx );
    for ( int k = 0; k < (int) lines_lsd[octaveIdx].size(); k++ )
    {
      KeyLine kl;
      cv::Vec4f extremes = lines_lsd[octaveIdx][k];

      /* check data validity */
      checkLineExtremes( extremes, gpyr[octaveIdx].size() );

      /* check line segment min length */
      double length = (float) sqrt( pow( extremes[0] - extremes[2], 2 ) + pow( extremes[1] - extremes[3], 2 ));
      if( length > opts.min_length )
      {
          /* fill KeyLine's fields */
          kl.startPointX = extremes[0] ;
          kl.startPointY = extremes[1] ;
          kl.endPointX   = extremes[2] ;
          kl.endPointY   = extremes[3] ;
          kl.lineLength = length;
          kl.angle = (atan ((kl.endPointY-kl.startPointY)/(kl.endPointX-kl.startPointX)))/3.1415926*180;
          kl.octave = octaveIdx;
          keylines.push_back( kl );
    }
  }
    Mat temp_img = gpyr[octaveIdx].clone();
//    draw_line(img,keylines);
    //绘制线段
    RNG rng(0);

    std::for_each( keylines.begin(), keylines.end(), [&](KeyLine kl)
    {

    cv::Scalar color = cv::Scalar(255, 255, 255);

    cv::line(temp_img,cv::Point(kl.startPointX,kl.startPointY),cv::Point(kl.endPointX,kl.endPointY),color,1.5);

    cv::circle(temp_img,cv::Point(kl.startPointX,kl.startPointY),3,cv::Scalar(255,255,255),1);

    cv::circle(temp_img,cv::Point(kl.endPointX,kl.endPointY),3,cv::Scalar(0,255,0),1);

    });

#if 1//Debug
    cv::imshow("线段图",temp_img);
#endif

}

}

}


}

