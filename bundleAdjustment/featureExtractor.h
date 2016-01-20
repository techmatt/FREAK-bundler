
struct FeatureExtractorImpl;

struct FREAKDescriptor
{
    static int hammingDistance(const FREAKDescriptor &a, const FREAKDescriptor &b);
    BYTE data[64];
};

struct Keypoint
{
    vec2f pt;
    float size;
    float angle;
    float response;

    FREAKDescriptor desc;
};

struct KeypointMatch
{
    int indexA;
    int indexB;
    float distance;
};

class FeatureExtractor
{
public:
    FeatureExtractor();
    vector<Keypoint> detectAndDescribe(const Bitmap &bmp);

    FeatureExtractorImpl *impl;
};

class KeypointMatcher
{
public:
    static void match(const vector<Keypoint> &keypointsA, const vector<Keypoint> &keypointsB);
};