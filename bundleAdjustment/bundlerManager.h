
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

struct ImageCorrespondences
{
    int imageA;
    int imageB;

    vector<ImageCorrespondence> data;
};

struct BundlerManager
{
    void loadSensorFile(const string &filename);

    vec2i imagePixelToDepthPixel(const vec2f &imageCoord) const;

    void computeKeypoints();

    void addCorrespondences(int forwardSkip);
    void addCorrespondences(int imageAIndex, int imageBIndex);

    mat4f depthIntrinsicInverse;
    vector<BundlerImage> images;
    vector<ImageCorrespondences> allCorrespondences;
};