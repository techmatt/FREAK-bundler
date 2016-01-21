
struct BundlerImage
{
    vec3f localPos(const vec2i &depthPixel) const;

    int index;
    
    Bitmap colorImage;
    DepthImage32 depthImage;

    mat4f depthIntrinsicInverse;

    vector<Keypoint> keypoints;
};

struct ImageCorrespondence
{
    int imageA;
    int imageB;

    vec2i ptAPixel; // 2D pixel in image A
    vec3f ptALocal; // 3D point in the local depth frame of A

    vec2i ptBPixel; // 2D pixel in image B
    vec3f ptBLocal; // 3D point in the local depth frame of B

    float keyPtDist;

    float residual;
};

struct ImagePairCorrespondences
{
    struct TransformResult
    {
        double totalError;
        double inlierError;
        int inlierCount;
        int outlierCount;
    };

    void visualize(const string &dir) const;

    BundlerImage *imageA;
    BundlerImage *imageB;

    mat4f transformAToB;

    int transformOutliers;
    int transformInliers;
    double transformInlierError;

    void estimateTransform();
    mat4f estimateTransform(const set<int> &indices);
    TransformResult computeTransformResult(const mat4f &transform);

    vector<ImageCorrespondence> allCorr;
    vector<ImageCorrespondence> inlierCorr;
};
