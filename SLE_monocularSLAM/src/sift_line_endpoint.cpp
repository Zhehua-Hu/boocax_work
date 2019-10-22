#include "../include/sift_line_endpoint.h"

namespace cv{


int Sift_Line_Endpoint::descriptorSize() const
{
    return SIFT_DESCR_WIDTH*SIFT_DESCR_WIDTH*SIFT_DESCR_HIST_BINS;
}

int Sift_Line_Endpoint::descriptorType() const
{
    return CV_32F;
}

int Sift_Line_Endpoint::defaultNorm() const
{
    return NORM_L2;
}


static Mat createInitialImage( const Mat& img, bool doubleImageSize, float sigma )
{
    Mat gray, gray_fpt;
    if( img.channels() == 3 || img.channels() == 4 )
        cvtColor(img, gray, COLOR_BGR2GRAY);
    else
        img.copyTo(gray);

//    cv::imshow("gray",gray);

    gray.convertTo(gray_fpt, DataType<sift_wt>::type, SIFT_FIXPT_SCALE, 0);

//   std::cout<<gray_fpt<<endl;

    float sig_diff;

    if( doubleImageSize )
    {
        sig_diff = sqrtf( std::max(sigma * sigma - SIFT_INIT_SIGMA * SIFT_INIT_SIGMA * 4, 0.01f) );
        Mat dbl;
        resize(gray_fpt, dbl, Size(gray.cols*2, gray.rows*2), 0, 0, INTER_LINEAR);
        GaussianBlur(dbl, dbl, Size(), sig_diff, sig_diff);
        return dbl;
    }
    else
    {
        sig_diff = sqrtf( std::max(sigma * sigma - SIFT_INIT_SIGMA * SIFT_INIT_SIGMA, 0.01f) );
        GaussianBlur(gray_fpt, gray_fpt, Size(), sig_diff, sig_diff);
//        cv::imshow("blurred_gray_ftp",gray_fpt);

        return gray_fpt;
    }
}



// 建立高斯金字塔， 组数 nOctaves， 每组层数 nOctaveLayers + 3
void Sift_Line_Endpoint::buildGaussianPyramid(  )
{

    gpyr.resize(nOctaves);

    Mat temp_image = img.clone();

    if(temp_image.channels()!=1){
        cvtColor(temp_image, temp_image, COLOR_BGR2GRAY);
    }

 //   img.convertTo(temp_image, DataType<sift_wt>::type, SIFT_FIXPT_SCALE, 0);

    for( int o = 0; o < nOctaves; o++ )
    {
        Mat& dst = gpyr[o];
        {

        if(o == 0)   dst = temp_image;

        else
            {
            const Mat& src = gpyr[o-1];

            resize(src, dst, Size(src.cols/2, src.rows/2),
                   0, 0, INTER_NEAREST);
        //    GaussianBlur(src, dst, Size(), sigma, sigma);
            }
         }
    }
}




void Sift_Line_Endpoint::buildGaussianPyramid( const Mat& base, std::vector<Mat>& pyr, int nOctaves ) const
{
    std::vector<double> sig(nOctaveLayers + 3);
    pyr.resize(nOctaves*(nOctaveLayers + 3));

    // precompute Gaussian sigmas using the following formula:
    //  \sigma_{total}^2 = \sigma_{i}^2 + \sigma_{i-1}^2
    sig[0] = sigma;
    double k = std::pow( 2., 1. / nOctaveLayers );
    for( int i = 1; i < nOctaveLayers + 3; i++ )
    {
        double sig_prev = std::pow(k, (double)(i-1))*sigma;
        double sig_total = sig_prev*k;
        sig[i] = std::sqrt(sig_total*sig_total - sig_prev*sig_prev);
    }

    for( int o = 0; o < nOctaves; o++ )
    {
        for( int i = 0; i < nOctaveLayers + 3; i++ )
        {
            Mat& dst = pyr[o*(nOctaveLayers + 3) + i];
            if( o == 0  &&  i == 0 )
                dst = base;
            // base of new octave is halved image from end of previous octave
            else if( i == 0 )
            {
                const Mat& src = pyr[(o-1)*(nOctaveLayers + 3) + nOctaveLayers];
                resize(src, dst, Size(src.cols/2, src.rows/2),
                       0, 0, INTER_NEAREST);
            }
            else
            {
                const Mat& src = pyr[o*(nOctaveLayers + 3) + i-1];
                GaussianBlur(src, dst, Size(), sig[i], sig[i]);
            }
        }
    }
}


// 建立Dog金字塔， 组数 nOctaves， 每组层数 nOctaveLayers + 2

void Sift_Line_Endpoint::buildDoGPyramid( const std::vector<Mat>& gpyr, std::vector<Mat>& dogpyr ) const
{
    int nOctaves = (int)gpyr.size()/(nOctaveLayers + 3);
    dogpyr.resize( nOctaves*(nOctaveLayers + 2) );

    for( int o = 0; o < nOctaves; o++ )
    {
        for( int i = 0; i < nOctaveLayers + 2; i++ )
        {
            const Mat& src1 = gpyr[o*(nOctaveLayers + 3) + i];
            const Mat& src2 = gpyr[o*(nOctaveLayers + 3) + i + 1];
            Mat& dst = dogpyr[o*(nOctaveLayers + 2) + i];
            subtract(src2, src1, dst, noArray(), DataType<sift_wt>::type);
        }
    }
}



Sift_Line_Endpoint::Sift_Line_Endpoint( int _nfeatures, int _nOctaveLayers,
           double _contrastThreshold, double _edgeThreshold, double _sigma, Mat _img , int actualNOctaves)
    : nfeatures(_nfeatures), nOctaveLayers(_nOctaveLayers),
    contrastThreshold(_contrastThreshold), edgeThreshold(_edgeThreshold), sigma(_sigma)
{
    if (_img.channels()==1)
        img = _img;

    else   cvtColor(_img,img,CV_BGR2GRAY);

    nOctaves = actualNOctaves > 0 ? actualNOctaves : cvRound(std::log( (double)std::min( img.cols, img.rows ) ) / std::log(2.) - 2);


}



static float calcOrientationHist( const Mat& img, Point pt, int radius,
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

            float dx = (float)(img.at<sift_wt>(y, x+1) - img.at<sift_wt>(y, x-1));
            float dy = (float)(img.at<sift_wt>(y-1, x) - img.at<sift_wt>(y+1, x));

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

    return maxval;
}



void Sift_Line_Endpoint::calc_Orientation_Hist(int radius,float sigma, int n )
{
    Point pt;

    float hist[n];

    img.convertTo(img,CV_32FC1);

   for_each(keypoints.begin(),keypoints.end(),[&](KeyPoint kpt)
    {

     pt.x = kpt.pt.x ;

     pt.y = kpt.pt.y;

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

           float t1 = img.at<sift_wt>(y, x+1);
           float t11 = img.at<sift_wt>(y, x-1);
           float t2 = img.at<sift_wt>(y-1, x);
           float t22 = img.at<sift_wt>(y+1, x);

           float dx = (float)(img.at<sift_wt>(y, x+1) - img.at<sift_wt>(y, x-1));
           float dy = (float)(img.at<sift_wt>(y-1, x) - img.at<sift_wt>(y+1, x));

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

   float mag_thr = (float)(maxval * SIFT_ORI_PEAK_RATIO);
   for( int j = 0; j < n; j++ )
   {
       int l = j > 0 ? j - 1 : n - 1;
       int r2 = j < n-1 ? j + 1 : 0;

       if( hist[j] > hist[l]  &&  hist[j] > hist[r2]  &&  hist[j] >= mag_thr )
       {
           float bin = j + 0.5f * (hist[l]-hist[r2]) / (hist[l] - 2*hist[j] + hist[r2]);
           bin = bin < 0 ? n + bin : bin >= n ? bin - n : bin;
           kpt.angle = 360.f - (float)((360.f/n) * bin);
           if(std::abs(kpt.angle - 360.f) < FLT_EPSILON)
               kpt.angle = 0.f;

       }
   }
   kpt.octave=1114623;
   kpt.response=2;
   selected_keypoints.push_back(kpt);
 // cout<<"-------------------"<<endl;

   });

   KeyPointsFilter::removeDuplicated( keypoints );

   KeyPointsFilter::retainBest(keypoints, 100);


}





 void Sift_Line_Endpoint::calc_Gpyr_Orientation_Hist(int radius,float sigma, int n)
{
     Point pt;

     float hist[n];

     int scale =1;

     int gpyr_layer =0;

     Mat gpyr_image;


    for_each(keypoints.begin(),keypoints.end(),[&](KeyPoint kpt)
     {

      gpyr_layer = kpt.octave;

      gpyr_image = gpyr[gpyr_layer];

      gpyr_image.convertTo(gpyr_image,CV_32FC1);

      scale = 1<<gpyr_layer;

      pt.x = kpt.pt.x / scale;

      pt.y = kpt.pt.y / scale;

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
        if( y <= 0 || y >= gpyr_image.rows - 1 )
            continue;
        for( j = -radius; j <= radius; j++ )
        {
            int x = pt.x + j;
            if( x <= 0 || x >= gpyr_image.cols - 1 )
                continue;

            float t1 = gpyr_image.at<sift_wt>(y, x+1);
            float t11 = gpyr_image.at<sift_wt>(y, x-1);
            float t2 = gpyr_image.at<sift_wt>(y-1, x);
            float t22 = gpyr_image.at<sift_wt>(y+1, x);

            float dx = (float)(gpyr_image.at<sift_wt>(y, x+1) - gpyr_image.at<sift_wt>(y, x-1));
            float dy = (float)(gpyr_image.at<sift_wt>(y-1, x) - gpyr_image.at<sift_wt>(y+1, x));

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

    float mag_thr = (float)(maxval * SIFT_ORI_PEAK_RATIO);
    for( int j = 0; j < n; j++ )
    {
        int l = j > 0 ? j - 1 : n - 1;
        int r2 = j < n-1 ? j + 1 : 0;

        if( hist[j] > hist[l]  &&  hist[j] > hist[r2]  &&  hist[j] >= mag_thr )
        {
            float bin = j + 0.5f * (hist[l]-hist[r2]) / (hist[l] - 2*hist[j] + hist[r2]);
            bin = bin < 0 ? n + bin : bin >= n ? bin - n : bin;
            kpt.angle = 360.f - (float)((360.f/n) * bin);
            if(std::abs(kpt.angle - 360.f) < FLT_EPSILON)
                kpt.angle = 0.f;

        }
    }

    selected_keypoints.push_back(kpt);
  // cout<<"-------------------"<<endl;

    });

    KeyPointsFilter::removeDuplicated( keypoints );

 }




static bool adjustLocalExtrema( const std::vector<Mat>& dog_pyr, KeyPoint& kpt, int octv,
                                int& layer, int& r, int& c, int nOctaveLayers,
                                float contrastThreshold, float edgeThreshold, float sigma )
{
    const float img_scale = 1.f/(255*SIFT_FIXPT_SCALE);
    const float deriv_scale = img_scale*0.5f;
    const float second_deriv_scale = img_scale;
    const float cross_deriv_scale = img_scale*0.25f;

    float xi=0, xr=0, xc=0, contr=0;
    int i = 0;

    for( ; i < SIFT_MAX_INTERP_STEPS; i++ )
    {
        int idx = octv*(nOctaveLayers+2) + layer;
        const Mat& img = dog_pyr[idx];
        const Mat& prev = dog_pyr[idx-1];
        const Mat& next = dog_pyr[idx+1];

        Vec3f dD((img.at<sift_wt>(r, c+1) - img.at<sift_wt>(r, c-1))*deriv_scale,
                 (img.at<sift_wt>(r+1, c) - img.at<sift_wt>(r-1, c))*deriv_scale,
                 (next.at<sift_wt>(r, c) - prev.at<sift_wt>(r, c))*deriv_scale);

        float v2 = (float)img.at<sift_wt>(r, c)*2;
        float dxx = (img.at<sift_wt>(r, c+1) + img.at<sift_wt>(r, c-1) - v2)*second_deriv_scale;
        float dyy = (img.at<sift_wt>(r+1, c) + img.at<sift_wt>(r-1, c) - v2)*second_deriv_scale;
        float dss = (next.at<sift_wt>(r, c) + prev.at<sift_wt>(r, c) - v2)*second_deriv_scale;
        float dxy = (img.at<sift_wt>(r+1, c+1) - img.at<sift_wt>(r+1, c-1) -
                     img.at<sift_wt>(r-1, c+1) + img.at<sift_wt>(r-1, c-1))*cross_deriv_scale;
        float dxs = (next.at<sift_wt>(r, c+1) - next.at<sift_wt>(r, c-1) -
                     prev.at<sift_wt>(r, c+1) + prev.at<sift_wt>(r, c-1))*cross_deriv_scale;
        float dys = (next.at<sift_wt>(r+1, c) - next.at<sift_wt>(r-1, c) -
                     prev.at<sift_wt>(r+1, c) + prev.at<sift_wt>(r-1, c))*cross_deriv_scale;

        Matx33f H(dxx, dxy, dxs,
                  dxy, dyy, dys,
                  dxs, dys, dss);

        Vec3f X = H.solve(dD, DECOMP_LU);

        xi = -X[2];
        xr = -X[1];
        xc = -X[0];

        if( std::abs(xi) < 0.5f && std::abs(xr) < 0.5f && std::abs(xc) < 0.5f )
            break;

        if( std::abs(xi) > (float)(INT_MAX/3) ||
            std::abs(xr) > (float)(INT_MAX/3) ||
            std::abs(xc) > (float)(INT_MAX/3) )
            return false;

        c += cvRound(xc);
        r += cvRound(xr);
        layer += cvRound(xi);

        if( layer < 1 || layer > nOctaveLayers ||
            c < SIFT_IMG_BORDER || c >= img.cols - SIFT_IMG_BORDER  ||
            r < SIFT_IMG_BORDER || r >= img.rows - SIFT_IMG_BORDER )
            return false;
    }

    // ensure convergence of interpolation
    if( i >= SIFT_MAX_INTERP_STEPS )
        return false;

    {
        int idx = octv*(nOctaveLayers+2) + layer;
        const Mat& img = dog_pyr[idx];
        const Mat& prev = dog_pyr[idx-1];
        const Mat& next = dog_pyr[idx+1];
        Matx31f dD((img.at<sift_wt>(r, c+1) - img.at<sift_wt>(r, c-1))*deriv_scale,
                   (img.at<sift_wt>(r+1, c) - img.at<sift_wt>(r-1, c))*deriv_scale,
                   (next.at<sift_wt>(r, c) - prev.at<sift_wt>(r, c))*deriv_scale);
        float t = dD.dot(Matx31f(xc, xr, xi));

        contr = img.at<sift_wt>(r, c)*img_scale + t * 0.5f;
        if( std::abs( contr ) * nOctaveLayers < contrastThreshold )
            return false;

        // principal curvatures are computed using the trace and det of Hessian
        float v2 = img.at<sift_wt>(r, c)*2.f;
        float dxx = (img.at<sift_wt>(r, c+1) + img.at<sift_wt>(r, c-1) - v2)*second_deriv_scale;
        float dyy = (img.at<sift_wt>(r+1, c) + img.at<sift_wt>(r-1, c) - v2)*second_deriv_scale;
        float dxy = (img.at<sift_wt>(r+1, c+1) - img.at<sift_wt>(r+1, c-1) -
                     img.at<sift_wt>(r-1, c+1) + img.at<sift_wt>(r-1, c-1)) * cross_deriv_scale;
        float tr = dxx + dyy;
        float det = dxx * dyy - dxy * dxy;

        if( det <= 0 || tr*tr*edgeThreshold >= (edgeThreshold + 1)*(edgeThreshold + 1)*det )
            return false;
    }

    kpt.pt.x = (c + xc) * (1 << octv);
    kpt.pt.y = (r + xr) * (1 << octv);
    kpt.octave = octv + (layer << 8) + (cvRound((xi + 0.5)*255) << 16);
    kpt.size = sigma*powf(2.f, (layer + xi) / nOctaveLayers)*(1 << octv)*2;
    kpt.response = std::abs(contr);

    return true;
}


void Sift_Line_Endpoint::findScaleSpaceExtrema( const std::vector<Mat>& gauss_pyr, const std::vector<Mat>& dog_pyr)
{
    int nOctaves = (int)gauss_pyr.size()/(nOctaveLayers + 3);
    int threshold = 0.1;//cvFloor(0.5 * contrastThreshold / nOctaveLayers * 255 * SIFT_FIXPT_SCALE);
    const int n = SIFT_ORI_HIST_BINS;
    float hist[n];
    KeyPoint kpt;
    float scale = 0.5;
    for( int o = 0; o < nOctaves; o++ )
    {
        scale *= 2;

        for( int i = 1; i <= nOctaveLayers; i++ )
        {
            int idx = o*(nOctaveLayers+2)+i;
            const Mat& srcimg = dog_pyr[idx];
//          cout<<dog_pyr[idx]<<endl;
            const Mat& prev = dog_pyr[idx-1];
            const Mat& next = dog_pyr[idx+1];

            int step = (int)srcimg.step1();

//            int step = (int)srcimg.rows;

            int rows = srcimg.rows, cols = srcimg.cols;
            for (int k=0; k <keypoints.size() ; k++ )
            {
                int r = cvRound(keypoints.at(k).pt.x/scale);  //  pt.x 代表第x行
                int c = cvRound(keypoints.at(k).pt.y/scale);  //  pt.y 代表第y列

                if( r >=SIFT_IMG_BORDER && r < rows-SIFT_IMG_BORDER && c >= SIFT_IMG_BORDER && c < cols-SIFT_IMG_BORDER)
                {
                const sift_wt* currptr = srcimg.ptr<sift_wt>(r);
                const sift_wt* prevptr = prev.ptr<sift_wt>(r);
                const sift_wt* nextptr = next.ptr<sift_wt>(r);

                sift_wt val = currptr[c];  //srcimg.at<float>(r,c);

                sift_wt temp2 = srcimg.at<float>(r,c);  //srcimg.at<float>(r,c);

                sift_wt temp3 = currptr[c-step-1];
                sift_wt temp4 = currptr[c-step];    // srcimg.at<float>(r-1,c);
                sift_wt temp5 = currptr[c-step+1];

                sift_wt temp33 = srcimg.at<float>(c-1,r-1);
                sift_wt temp44 = srcimg.at<float>(c-1,r);
                sift_wt temp44444 = srcimg.at<float>(c,r-1);

                sift_wt temp55 = srcimg.at<float>(c-1,r+1);

                sift_wt temp3333 = srcimg.at<float>(r-1,c-1);
                sift_wt temp4444 = srcimg.at<float>(r,c-1);
                sift_wt temp444 = srcimg.at<float>(r-1,c);

                sift_wt temp5555 = srcimg.at<float>(r-1,c+1);

                // find local extrema with pixel accuracy
                if( std::abs(val) > threshold &&
                   ((val > 0 && val >= currptr[c-1] && val >= currptr[c+1] &&
                     val >= currptr[c-step-1] && val >= currptr[c-step] && val >= currptr[c-step+1] &&
                     val >= currptr[c+step-1] && val >= currptr[c+step] && val >= currptr[c+step+1] &&
                     val >= nextptr[c] && val >= nextptr[c-1] && val >= nextptr[c+1] &&
                     val >= nextptr[c-step-1] && val >= nextptr[c-step] && val >= nextptr[c-step+1] &&
                     val >= nextptr[c+step-1] && val >= nextptr[c+step] && val >= nextptr[c+step+1] &&
                     val >= prevptr[c] && val >= prevptr[c-1] && val >= prevptr[c+1] &&
                     val >= prevptr[c-step-1] && val >= prevptr[c-step] && val >= prevptr[c-step+1] &&
                     val >= prevptr[c+step-1] && val >= prevptr[c+step] && val >= prevptr[c+step+1]) ||
                    (val < 0 && val <= currptr[c-1] && val <= currptr[c+1] &&
                     val <= currptr[c-step-1] && val <= currptr[c-step] && val <= currptr[c-step+1] &&
                     val <= currptr[c+step-1] && val <= currptr[c+step] && val <= currptr[c+step+1] &&
                     val <= nextptr[c] && val <= nextptr[c-1] && val <= nextptr[c+1] &&
                     val <= nextptr[c-step-1] && val <= nextptr[c-step] && val <= nextptr[c-step+1] &&
                     val <= nextptr[c+step-1] && val <= nextptr[c+step] && val <= nextptr[c+step+1] &&
                     val <= prevptr[c] && val <= prevptr[c-1] && val <= prevptr[c+1] &&
                     val <= prevptr[c-step-1] && val <= prevptr[c-step] && val <= prevptr[c-step+1] &&
                     val <= prevptr[c+step-1] && val <= prevptr[c+step] && val <= prevptr[c+step+1])))
                {
                    int r1 = r, c1 = c, layer = i;
                    if( !adjustLocalExtrema(dog_pyr, kpt, o, layer, r1, c1,
                                            nOctaveLayers, (float)contrastThreshold,
                                            (float)edgeThreshold, (float)sigma) )
                        continue;
                    float scl_octv = kpt.size*0.5f/(1 << o);
                    float omax = calcOrientationHist(gauss_pyr[o*(nOctaveLayers+3) + layer],
                                                     Point(c1, r1),
                                                     cvRound(SIFT_ORI_RADIUS * scl_octv),
                                                     SIFT_ORI_SIG_FCTR * scl_octv,
                                                     hist, n);
                    float mag_thr = (float)(omax * SIFT_ORI_PEAK_RATIO);
                    for( int j = 0; j < n; j++ )
                    {
                        int l = j > 0 ? j - 1 : n - 1;
                        int r2 = j < n-1 ? j + 1 : 0;

                        if( hist[j] > hist[l]  &&  hist[j] > hist[r2]  &&  hist[j] >= mag_thr )
                        {
                            float bin = j + 0.5f * (hist[l]-hist[r2]) / (hist[l] - 2*hist[j] + hist[r2]);
                            bin = bin < 0 ? n + bin : bin >= n ? bin - n : bin;
                            kpt.angle = 360.f - (float)((360.f/n) * bin);
                            if(std::abs(kpt.angle - 360.f) < FLT_EPSILON)
                                kpt.angle = 0.f;
                             selected_keypoints.push_back(kpt);
                        }
                    }
                }

                }
            }
        }


    }

      keypoints.clear();


//    for( int o = 0; o < nOctaves; o++ )
//        for( int i = 1; i <= nOctaveLayers; i++ )
//        {
//            int idx = o*(nOctaveLayers+2)+i;
//            const Mat& img = dog_pyr[idx];
//            const Mat& prev = dog_pyr[idx-1];
//            const Mat& next = dog_pyr[idx+1];
//            int step = (int)img.step1();
//            int rows = img.rows, cols = img.cols;

//            for( int r = SIFT_IMG_BORDER; r < rows-SIFT_IMG_BORDER; r++)
//            {
//                const sift_wt* currptr = img.ptr<sift_wt>(r);
//                const sift_wt* prevptr = prev.ptr<sift_wt>(r);
//                const sift_wt* nextptr = next.ptr<sift_wt>(r);

//                for( int c = SIFT_IMG_BORDER; c < cols-SIFT_IMG_BORDER; c++)
//                {
//                    sift_wt val = currptr[c];

//                    // find local extrema with pixel accuracy
//                    if( std::abs(val) > threshold &&
//                       ((val > 0 && val >= currptr[c-1] && val >= currptr[c+1] &&
//                         val >= currptr[c-step-1] && val >= currptr[c-step] && val >= currptr[c-step+1] &&
//                         val >= currptr[c+step-1] && val >= currptr[c+step] && val >= currptr[c+step+1] &&
//                         val >= nextptr[c] && val >= nextptr[c-1] && val >= nextptr[c+1] &&
//                         val >= nextptr[c-step-1] && val >= nextptr[c-step] && val >= nextptr[c-step+1] &&
//                         val >= nextptr[c+step-1] && val >= nextptr[c+step] && val >= nextptr[c+step+1] &&
//                         val >= prevptr[c] && val >= prevptr[c-1] && val >= prevptr[c+1] &&
//                         val >= prevptr[c-step-1] && val >= prevptr[c-step] && val >= prevptr[c-step+1] &&
//                         val >= prevptr[c+step-1] && val >= prevptr[c+step] && val >= prevptr[c+step+1]) ||
//                        (val < 0 && val <= currptr[c-1] && val <= currptr[c+1] &&
//                         val <= currptr[c-step-1] && val <= currptr[c-step] && val <= currptr[c-step+1] &&
//                         val <= currptr[c+step-1] && val <= currptr[c+step] && val <= currptr[c+step+1] &&
//                         val <= nextptr[c] && val <= nextptr[c-1] && val <= nextptr[c+1] &&
//                         val <= nextptr[c-step-1] && val <= nextptr[c-step] && val <= nextptr[c-step+1] &&
//                         val <= nextptr[c+step-1] && val <= nextptr[c+step] && val <= nextptr[c+step+1] &&
//                         val <= prevptr[c] && val <= prevptr[c-1] && val <= prevptr[c+1] &&
//                         val <= prevptr[c-step-1] && val <= prevptr[c-step] && val <= prevptr[c-step+1] &&
//                         val <= prevptr[c+step-1] && val <= prevptr[c+step] && val <= prevptr[c+step+1])))
//                    {
//                        int r1 = r, c1 = c, layer = i;
//                        if( !adjustLocalExtrema(dog_pyr, kpt, o, layer, r1, c1,
//                                                nOctaveLayers, (float)contrastThreshold,
//                                                (float)edgeThreshold, (float)sigma) )
//                            continue;
//                        float scl_octv = kpt.size*0.5f/(1 << o);
//                        float omax = calcOrientationHist(gauss_pyr[o*(nOctaveLayers+3) + layer],
//                                                         Point(c1, r1),
//                                                         cvRound(SIFT_ORI_RADIUS * scl_octv),
//                                                         SIFT_ORI_SIG_FCTR * scl_octv,
//                                                         hist, n);
//                        float mag_thr = (float)(omax * SIFT_ORI_PEAK_RATIO);
//                        for( int j = 0; j < n; j++ )
//                        {
//                            int l = j > 0 ? j - 1 : n - 1;
//                            int r2 = j < n-1 ? j + 1 : 0;

//                            if( hist[j] > hist[l]  &&  hist[j] > hist[r2]  &&  hist[j] >= mag_thr )
//                            {
//                                float bin = j + 0.5f * (hist[l]-hist[r2]) / (hist[l] - 2*hist[j] + hist[r2]);
//                                bin = bin < 0 ? n + bin : bin >= n ? bin - n : bin;
//                                kpt.angle = 360.f - (float)((360.f/n) * bin);
//                                if(std::abs(kpt.angle - 360.f) < FLT_EPSILON)
//                                    kpt.angle = 0.f;
//                                keypoints.push_back(kpt);
//                            }
//                        }
//                    }
//                }
//            }
//        }
}




static void calcSIFTDescriptor( const Mat& img, Point2f ptf, float ori, float scl,
                               int d, int n, float* dst )
{
    Point pt(cvRound(ptf.x), cvRound(ptf.y));
    float cos_t = cosf(ori*(float)(CV_PI/180));
    float sin_t = sinf(ori*(float)(CV_PI/180));
    float bins_per_rad = n / 360.f;
    float exp_scale = -1.f/(d * d * 0.5f);
    float hist_width = SIFT_DESCR_SCL_FCTR * scl;
    int radius = cvRound(hist_width * 1.4142135623730951f * (d + 1) * 0.5f);  	// 计算图像区域半径mσ(d+1)/2*sqrt(2)
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



static void calcDescriptors(const std::vector<Mat>& gpyr, const std::vector<KeyPoint>& keypoints,
                            Mat& descriptors, int nOctaveLayers, int firstOctave )
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

void Sift_Line_Endpoint::detectAndCompute(InputArray _image, InputArray _mask,
                      std::vector<KeyPoint>& keypoints,
                      OutputArray _descriptors,
                      bool useProvidedKeypoints)
{
    int firstOctave = -1, actualNOctaves = 0, actualNLayers = 0;
    Mat image = _image.getMat(), mask = _mask.getMat();

    if( image.empty() || image.depth() != CV_8U )
        CV_Error( Error::StsBadArg, "image is empty or has incorrect depth (!=CV_8U)" );

    if( !mask.empty() && mask.type() != CV_8UC1 )
        CV_Error( Error::StsBadArg, "mask has incorrect type (!=CV_8UC1)" );

    if( useProvidedKeypoints )
    {
        firstOctave = 0;
        int maxOctave = INT_MIN;
        for( size_t i = 0; i < keypoints.size(); i++ )
        {
            int octave, layer;
            float scale;
            unpackOctave(keypoints[i], octave, layer, scale);
            firstOctave = std::min(firstOctave, octave);
            maxOctave = std::max(maxOctave, octave);
            actualNLayers = std::max(actualNLayers, layer-2);
        }

        firstOctave = std::min(firstOctave, 0);
        CV_Assert( firstOctave >= -1 && actualNLayers <= nOctaveLayers );
        actualNOctaves = maxOctave - firstOctave + 1;
    }

    Mat base = createInitialImage(image, firstOctave < 0, (float)sigma);
    std::vector<Mat> gpyr, dogpyr;
    int nOctaves = actualNOctaves > 0 ? actualNOctaves : cvRound(std::log( (double)std::min( base.cols, base.rows ) ) / std::log(2.) - 2) - firstOctave;

    //double t, tf = getTickFrequency();
    //t = (double)getTickCount();
    buildGaussianPyramid(base, gpyr, nOctaves);
    buildDoGPyramid(gpyr, dogpyr);

    //t = (double)getTickCount() - t;
    //printf("pyramid construction time: %g\n", t*1000./tf);

    if( !useProvidedKeypoints )
    {
        //t = (double)getTickCount();
        findScaleSpaceExtrema(gpyr, dogpyr);
        KeyPointsFilter::removeDuplicated( keypoints );

        if( nfeatures > 0 )
            KeyPointsFilter::retainBest(keypoints, nfeatures);
        //t = (double)getTickCount() - t;
        //printf("keypoint detection time: %g\n", t*1000./tf);

        if( firstOctave < 0 )
            for( size_t i = 0; i < keypoints.size(); i++ )
            {
                KeyPoint& kpt = keypoints[i];
                float scale = 1.f/(float)(1 << -firstOctave);
                kpt.octave = (kpt.octave & ~255) | ((kpt.octave + firstOctave) & 255);
                kpt.pt *= scale;
                kpt.size *= scale;
            }

        if( !mask.empty() )
            KeyPointsFilter::runByPixelsMask( keypoints, mask );
    }
    else
    {
        // filter keypoints by mask
        //KeyPointsFilter::runByPixelsMask( keypoints, mask );
    }

    if( _descriptors.needed() )
    {
        //t = (double)getTickCount();
        int dsize = descriptorSize();
        _descriptors.create((int)keypoints.size(), dsize, CV_32F);
        Mat descriptors = _descriptors.getMat();

        calcDescriptors(gpyr, keypoints, descriptors, nOctaveLayers, firstOctave);
        //t = (double)getTickCount() - t;
        //printf("descriptor extraction time: %g\n", t*1000./tf);
    }
}

void Sift_Line_Endpoint::get_lsdoptions(LSDDetectorC::LSDOptions _opts){
    opts.refine       = _opts.refine;
    opts.scale        = _opts.scale;
    opts.sigma_scale  = _opts.sigma_scale;
    opts.quant        = _opts.quant ;
    opts.ang_th       = _opts.ang_th;
    opts.log_eps      = _opts.log_eps;
    opts.density_th   = _opts.density_th;
    opts.n_bins       = _opts.n_bins;
}

void Sift_Line_Endpoint::get_keypoints( ){


          std::vector<KeyLine> key_lines;


          lsd->detect(img, key_lines,1, 1,opts);

          //sort lines according to their unscaled length
          //sort( key_lines.begin(), key_lines.end(), compare_line_by_unscaled_length() );
          //detection_threshold_ = keyline[int(0.25*double(keyline.size()))].lineLength ;

          detection_threshold_=detection_threshold_ ;

          std::for_each( key_lines.begin(), key_lines.end(), [&](KeyLine kl){
            if( kl.lineLength > detection_threshold_ ){
                keylines.push_back(kl);
            }
          });

       lsd->draw_line(img,keylines);

       std::vector<KeyPoint>  temp_keypoints;

       KeyPoint kp;

      // sort( keylines.begin(), keylines.end(), compare_line_by_unscaled_length() );

       std::for_each( keylines.begin(), keylines.end(), [&](KeyLine kl)
           {
            if(kl.startPointX>=margin && kl.startPointX<=img.cols-margin &&
                 kl.startPointY>=margin && kl.startPointY<=img.rows-margin)
                kp.pt.x=kl.startPointX;
                kp.pt.y=kl.startPointY;
                temp_keypoints.push_back(kp);

            if(kl.endPointX>=margin && kl.endPointX<=img.cols-margin &&
                 kl.endPointY>=margin && kl.endPointY<=img.rows-margin)
                 kp.pt.x=kl.endPointX ;
                 kp.pt.y=kl.endPointY ;
                 temp_keypoints.push_back(kp);
              });

      if(temp_keypoints.size()>10){

      int start_kp_size = (int)keypoints.size();

      keypoints.push_back(temp_keypoints[0]);

      keypoints.push_back(temp_keypoints[(int)temp_keypoints.size()-1]);

      int select_batch=10;


      for (int i =1 ;i<(int)temp_keypoints.size()-1;i++)
      {
          bool add_flag = 0;

          int kp_size = (int)keypoints.size();

          for (int j =start_kp_size ;j<kp_size;j++)
          {
             if(abs((temp_keypoints[i]).pt.x-(keypoints[j]).pt.x)<select_batch &&
                    abs((temp_keypoints[i]).pt.y-(keypoints[j]).pt.y)<select_batch )
             {
                 add_flag=1;
                 continue;
             }
          }

          if(!add_flag)   keypoints.push_back(temp_keypoints[i]);
      }
        }
      cout<<"筛选完毕"<<endl;
}




void Sift_Line_Endpoint::get_gpyrkeypoints()
{

    lsd->detectline(gpyr, keylines,opts);

    std::vector<KeyPoint>  temp_keypoints;

    KeyPoint kp;

   std::for_each( keylines.begin(), keylines.end(), [&](KeyLine kl)
     {
       int layer = kl.octave;

       int scale = 1<<layer;

       kl.endPointX = kl.endPointX * scale;

       kl.endPointY = kl.endPointY * scale;

       kl.startPointX = kl.startPointX * scale;

       kl.startPointY = kl.startPointY * scale;

      if(kl.startPointX>=margin && kl.startPointX<=img.cols-margin &&
           kl.startPointY>=margin && kl.startPointY<=img.rows-margin)
          kp.pt.x=kl.startPointX;
          kp.pt.y=kl.startPointY;
          kp.octave=layer;
          temp_keypoints.push_back(kp);

      if(kl.endPointX>=margin && kl.endPointX<=img.cols-margin &&
           kl.endPointY>=margin && kl.endPointY<=img.rows-margin)
           kp.pt.x=kl.endPointX ;
           kp.pt.y=kl.endPointY ;
           kp.octave=layer;
           temp_keypoints.push_back(kp);
        });

   keylines.clear();

if(temp_keypoints.size()>10){

keypoints.push_back(temp_keypoints[0]);

keypoints.push_back(temp_keypoints[(int)temp_keypoints.size()-1]);

int select_batch=10;

for (int i =1 ;i<(int)temp_keypoints.size()-1;i++)
{
    bool add_flag = 0;

    int kp_size = (int)keypoints.size();

    for (int j =0 ;j<kp_size;j++)
    {
       if(abs((temp_keypoints[i]).pt.x-(keypoints[j]).pt.x)<select_batch &&
              abs((temp_keypoints[i]).pt.y-(keypoints[j]).pt.y)<select_batch &&
               (temp_keypoints[i]).octave == (keypoints[j]).octave)
       {
           add_flag=1;
           continue;
       }
    }

    if(!add_flag)   keypoints.push_back(temp_keypoints[i]);

}

temp_keypoints.clear();
  }

}


void Sift_Line_Endpoint::select_keypoints()
{
    //剔除靠得很近的端点
    int still_think = 0;
}


void Sift_Line_Endpoint::get_sift(){

    cv::imshow("base_img",img);

    Mat base = createInitialImage(img, false, (float)sigma);

//    cv::imshow("base",base);

//    std::cout<<base<<endl;

    std::cout<<"---------------------------------------------"<<endl;

    std::vector<Mat> gpyr, dogpyr;

    int nOctaves = cvRound(log( (double)std::min( base.cols, base.rows ) ) / log(2.) - 2);

    buildGaussianPyramid(base, gpyr, nOctaves);
        // 构建高斯差分金字塔
    buildDoGPyramid(gpyr, dogpyr);

    findScaleSpaceExtrema(gpyr, dogpyr);

    print_kp(0);

    cout<<"----------------------"<<endl;

    //除去重复特征点
    KeyPointsFilter::removeDuplicated( selected_keypoints );

    print_kp(0);

//    cout<<"xxxxxxxxxxxxxxxxxxxx"<<endl;
}


void Sift_Line_Endpoint::print_kp(bool select_flag)
{
    if (select_flag)
    {
    for_each(keypoints.begin(),keypoints.end(), [&](KeyPoint kp){

        cout<<kp.pt.x<<"   "<<kp.pt.y<<"   "<<endl;

             });
    }

    else
    {

        for_each(selected_keypoints.begin(),selected_keypoints.end(), [&](KeyPoint kp){

            cout<<kp.pt.x<<"   "<<kp.pt.y<<"   "<<endl;

                 });
    }

}


void Sift_Line_Endpoint::draw_kp()
{
    for_each(selected_keypoints.begin(),selected_keypoints.end(), [&](KeyPoint kp){

        cv::circle(img,cv::Point(kp.pt.y,kp.pt.x),3,255,1);

       });

      cv::imshow("key_points",img);


}


void Sift_Line_Endpoint::calculator_gpyr_descriptor(){

    int d = SIFT_DESCR_WIDTH, n = SIFT_DESCR_HIST_BINS;

    descriptors = Mat::zeros((int)keypoints.size(), 128  ,CV_32F);

    int scale = 1;

    for( size_t i = 0; i < keypoints.size(); i++ )
    {
        KeyPoint kpt = keypoints[i];
        scale = 1<< kpt.octave;
        Point2f ptf(kpt.pt.x/scale, kpt.pt.y/scale);
        float angle = 360.f - kpt.angle;
        if(std::abs(angle - 360.f) < FLT_EPSILON)
            angle = 0.f;

        Mat des_img;

        gpyr[kpt.octave].convertTo(des_img,CV_32FC1);

        calcSIFTDescriptor(des_img, ptf, angle, 3, d, n, descriptors.ptr<float>((int)i));

    }

}



void Sift_Line_Endpoint::calculator_descriptor()
{

    int d = SIFT_DESCR_WIDTH, n = SIFT_DESCR_HIST_BINS;

    descriptors = Mat::zeros((int)keypoints.size(), 128  ,CV_32F);

    for( size_t i = 0; i < keypoints.size(); i++ )
    {
        KeyPoint kpt = keypoints[i];
        float size=kpt.size;
        Point2f ptf(kpt.pt.x, kpt.pt.y);
        float angle = 360.f - kpt.angle;
        if(std::abs(angle - 360.f) < FLT_EPSILON)
            angle = 0.f;
        calcSIFTDescriptor(img, ptf, angle, 1, d, n, descriptors.ptr<float>((int)i));

    }

    cout<<descriptors<<endl;

}


void Sift_Line_Endpoint::run()
{
    buildGaussianPyramid();

#if(Debug)


        imshow("jing zi ta0",gpyr[0]);
        imshow("jing zi ta1",gpyr[1]);
        imshow("jing zi ta2",gpyr[2]);
        imshow("jing zi ta3",gpyr[3]);
        imshow("jing zi ta4",gpyr[4]);
#endif



    LSDDetectorC::LSDOptions opts;
       opts.refine       = LSD_REFINE_ADV;
       opts.scale        = 0.8;
       opts.sigma_scale  = 0.6;
       opts.quant        = 2.0;
       opts.ang_th       = 22.5;
       opts.log_eps      = 1.0;
       opts.density_th   = 0.6;
       opts.n_bins       = 1024;

       get_lsdoptions(opts);
       get_gpyrkeypoints();
       calc_Gpyr_Orientation_Hist(5,2,36);
       calculator_gpyr_descriptor();
       draw_line();

//       get_keypoints();
//     calc_Orientation_Hist(5,10,36);
//       calculator_descriptor();

}

void Sift_Line_Endpoint::draw_line()
{
    int layer=0;

    int scale =1;

    std::vector<Mat> copy_gpyr = gpyr;

    std::for_each(keylines.begin(), keylines.end(), [&](KeyLine kl)

    {

    layer= kl.octave;

    if(layer >=0){

    scale = 1<<layer;

    cv::Scalar color = cv::Scalar(255, 255, 255);

    cv::line(copy_gpyr[0],cv::Point(kl.startPointX*scale,kl.startPointY*scale),cv::Point(kl.endPointX*scale,kl.endPointY*scale),color,1.5);

    cv::circle(copy_gpyr[0],cv::Point(kl.startPointX*scale,kl.startPointY*scale),3,cv::Scalar(255,255,255),1);

    cv::circle(copy_gpyr[0],cv::Point(kl.endPointX*scale,kl.endPointY*scale),3,cv::Scalar(0,255,0),1);

    }

    });



    std::for_each( keypoints.begin(), keypoints.end(), [&](KeyPoint kp)
    {
    cv::circle(copy_gpyr[0],cv::Point(kp.pt.x,kp.pt.y),3,cv::Scalar(0,255,0),1);
    });

    cv::imshow("jin zi ta segment0", copy_gpyr[0]);
//    cv::imshow("jin zi ta segment1", copy_gpyr[1]);
//    cv::imshow("jin zi ta segment2", copy_gpyr[2]);
//    cv::imshow("jin zi ta segment3", copy_gpyr[3]);
//    cv::imshow("jin zi ta segment4", copy_gpyr[4]);
}

void Sift_Line_Endpoint::set_img(const Mat &image){
    img=image.clone();
}

void Sift_Line_Endpoint::detect_keypoints(const Mat &image, std::vector<KeyPoint> &Kpoints, Mat& desc_){

     img = image.clone();

     buildGaussianPyramid();

    LSDDetectorC::LSDOptions options;
       options.refine       = LSD_REFINE_ADV;
       options.scale        = 0.8;
       options.sigma_scale  = 0.6;
       options.quant        = 2.0;
       options.ang_th       = 22.5;
       options.log_eps      = 1.0;
       options.density_th   = 0.6;
       options.n_bins       = 1024;
       options.min_length   = 10;

     get_lsdoptions(options);

     get_gpyrkeypoints();

//     std::for_each(selected_keypoints.begin(),selected_keypoints.end(),[&](KeyPoint kp){
//                  Kpoints.push_back(kp);
//              });    遍历赋值，效率太低
     Kpoints.assign(keypoints.begin(),keypoints.end());

     calc_Gpyr_Orientation_Hist(5,2,36);

     calculator_gpyr_descriptor();

     desc_=descriptors.clone();

     keypoints.clear();
}




}
