
#include "main.h"

#include "opencv2/xfeatures2d.hpp"

using namespace cv;
using namespace cv::xfeatures2d;

struct FeatureExtractorImpl
{
    FeatureExtractorImpl()
    {
        const double hessian = 100.0;
        surf = SURF::create(hessian);
        freak = FREAK::create();
    }
    Ptr<Feature2D> surf;
    Ptr<Feature2D> freak;
};

FeatureExtractor::FeatureExtractor()
{
    impl = new FeatureExtractorImpl;
}

vector<Keypoint> FeatureExtractor::detectAndDescribe(const Bitmap &bmp)
{
    Mat cvImage(bmp.getDimY(), bmp.getDimX(), CV_8UC1);
    for (auto &p : bmp)
    {
        const BYTE c = util::boundToByte(((float)p.value.x + (float)p.value.y + (float)p.value.z) / 3.0f);
        cvImage.at<BYTE>((int)p.y, (int)p.x) = c;
    }

    vector<cv::KeyPoint> cvPts;
    impl->surf->detect(cvImage, cvPts);

    const int keyPtCount = (int)cvPts.size();

    Mat descriptors;
    impl->freak->compute(cvImage, cvPts, descriptors);
    
    vector<Keypoint> result(keyPtCount);
    for (int kIndex = 0; kIndex < keyPtCount; kIndex++)
    {
        const auto &k = cvPts[kIndex];
        Keypoint &keypt = result[kIndex];
        keypt.angle = k.angle;
        keypt.response = k.response;
        keypt.size = k.size;
        keypt.pt.x = k.pt.x;
        keypt.pt.y = k.pt.y;
        memcpy(keypt.desc.data, descriptors.ptr(kIndex), 64);
    }
    return result;
}

void KeypointMatcher::match(const vector<Keypoint> &keypointsA, const vector<Keypoint> &keypointsB)
{

}

int FREAKDescriptor::hammingDistance(const FREAKDescriptor &a, const FREAKDescriptor &b)
{
    const UINT64 *aStart = (const UINT64 *)a.data;
    const UINT64 *bStart = (const UINT64 *)b.data;
    int sum = 0;
    for (int i = 0; i < 8; i++)
    {
        sum += helper::countBits(aStart[i] ^ bStart[i]);
    }
    return sum;
}
