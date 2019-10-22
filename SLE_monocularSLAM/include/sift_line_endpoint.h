#ifndef SIFT_LINE_ENDPOINT_H
#define SIFT_LINE_ENDPOINT_H

#include "../3rd_party/descriptor_custom.hpp"
#include "../3rd_party/precomp.hpp"
#include <stdarg.h>
#include <opencv2/core/hal/hal.hpp>
#include <math.h>
#include "common.h"


using namespace cv::line_descriptor;
using namespace std;

/******************************* Defs and macros *****************************/

// default width of descriptor histogram array
static const int SIFT_DESCR_WIDTH = 4;

// default number of bins per histogram in descriptor array
static const int SIFT_DESCR_HIST_BINS = 8;

// assumed gaussian blur for input image
static const float SIFT_INIT_SIGMA = 0.5f;

// width of border in which to ignore keypoints
static const int SIFT_IMG_BORDER = 5;

// maximum steps of keypoint interpolation before failure
static const int SIFT_MAX_INTERP_STEPS = 5;

// default number of bins in histogram for orientation assignment
static const int SIFT_ORI_HIST_BINS = 36;

// determines gaussian sigma for orientation assignment
static const float SIFT_ORI_SIG_FCTR = 1.5f;

// determines the radius of the region used in orientation assignment
static const float SIFT_ORI_RADIUS = 3 * SIFT_ORI_SIG_FCTR;

// orientation magnitude relative to max that results in new feature
static const float SIFT_ORI_PEAK_RATIO = 0.8f;

// determines the size of a single descriptor orientation histogram
static const float SIFT_DESCR_SCL_FCTR = 3.f;

// threshold on magnitude of elements of descriptor vector
static const float SIFT_DESCR_MAG_THR = 0.2f;

// factor used to convert floating-point descriptor to unsigned char
static const float SIFT_INT_DESCR_FCTR = 512.f;

#if 0
// intermediate type used for DoG pyramids
typedef short sift_wt;
static const int SIFT_FIXPT_SCALE = 48;
#else
// intermediate type used for DoG pyramids
typedef float sift_wt;
static const int SIFT_FIXPT_SCALE = 1;
#endif



namespace cv
{

class  Sift_Line_Endpoint //: public xfeatures2d::SIFT
{
private:

    std::vector<KeyLine> keylines;

    Mat img;

    double detection_threshold_=10;

    LSDDetectorC::LSDOptions opts;

    int nOctaves;

    int margin = 20;

public:

    typedef std::shared_ptr<Sift_Line_Endpoint> Ptr_SLE;

    std::vector<Mat> gpyr;

    Mat get_img()
    {
        return img;
    }

    void set_img(const Mat& image);

    Mat descriptors;

   void calculator_descriptor();

   void calculator_gpyr_descriptor();

   std::vector<KeyPoint> keypoints, selected_keypoints;

   std::vector<KeyPoint> get_kp()
   {
   return keypoints;
   }

    void print_kp(bool select_flag);

    int descriptorSize() const;

    //! returns the descriptor type
    int descriptorType() const;

    //! returns the default norm type
    int defaultNorm() const;

    explicit Sift_Line_Endpoint( int nfeatures = 0, int nOctaveLayers = 3,
                          double contrastThreshold = 0.04, double edgeThreshold = 10,
                          double sigma = 1.6, Mat _img = Mat(5, 5, CV_8UC1, Scalar(128,0,0)),
                                 int actualNOctaves = 1);
    Sift_Line_Endpoint(){

        nfeatures = 0;

        nOctaveLayers = 3;

        contrastThreshold = 0.04;

        edgeThreshold = 10;

        sigma = 1.6;

        nOctaves = 5;

       // img = Mat(5, 5, CV_8UC1, Scalar(128,0,0));

    }

    void get_lsdoptions(LSDDetectorC::LSDOptions _opts);

    void get_keypoints();

    void get_gpyrkeypoints();

    void detect_keypoints(const Mat& image, std::vector<KeyPoint>& Kpoints , Mat& desc_);

    void select_keypoints();

    void get_sift();

    void draw_kp();

    void calc_Gpyr_Orientation_Hist( int radius,float sigma, int n);

    void calc_Orientation_Hist( int radius,float sigma, int n);

    Ptr<LSDDetectorC> lsd = LSDDetectorC::createLSDDetectorC();

    void detectAndCompute(InputArray img, InputArray mask,
                    std::vector<KeyPoint>& keypoints,
                    OutputArray descriptors,
                    bool useProvidedKeypoints = false);

    void buildGaussianPyramid()  ;
    void buildDoGPyramid( const std::vector<Mat>& pyr, std::vector<Mat>& dogpyr ) const;
    void findScaleSpaceExtrema( const std::vector<Mat>& gauss_pyr, const std::vector<Mat>& dog_pyr) ;
    void buildGaussianPyramid(const Mat& base, std::vector<Mat>& pyr , int nOctaves) const;
    void run();
    void draw_line();

protected:
    CV_PROP_RW int nfeatures;
    CV_PROP_RW int nOctaveLayers;
    CV_PROP_RW double contrastThreshold;
    CV_PROP_RW double edgeThreshold;
    CV_PROP_RW double sigma;

};



static inline void
unpackOctave(const KeyPoint& kpt, int& octave, int& layer, float& scale)
{
    octave = kpt.octave & 255;
    layer = (kpt.octave >> 8) & 255;
    octave = octave < 128 ? octave : (-128 | octave);
    scale = octave >= 0 ? 1.f/(1 << octave) : (float)(1 << -octave);
}


}

#endif // SIFT_LINE_ENDPOINT_H
