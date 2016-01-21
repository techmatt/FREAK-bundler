
struct BundlerImage
{
    vec3f localPos(const vec2i &depthPixel, const mat4f &depthIntrinsicInverse) const;

    int index;
    
    Bitmap colorImage;
    DepthImage32 depthImage;

    vector<Keypoint> keypoints;
};

struct ImageCorrespondence
{
    int imageA;
    int imageB;

    vec3f ptALocal; // 3D point in the local depth frame of A
    vec3f ptBLocal; // 3D point in the local depth frame of B

    float keyPtDist;

    float residual;
};

struct ImagePairCorrespondences
{
    void visualize(const string &dir) const;

    BundlerImage *imageA;
    BundlerImage *imageB;

    mat4f transformAToB;

    int transformOutliers;
    int transformInliers;
    double transformInlierError;

    void estimateTransform();
    mat4f estimateTransform(const set<int> &indices);
    double computeTransformError(const mat4f &transform);

    vector<ImageCorrespondence> allCorr;
    vector<ImageCorrespondence> inlierCorr;
};
