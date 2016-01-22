
struct BundlerFrame
{
    BundlerFrame()
    {
        index = -1;

        for (auto &c : camera)
            c = 0.0;

        debugColor = vec3f(util::randomUniformf(), util::randomUniformf(), util::randomUniformf());
        while (debugColor.length() < 0.7f)
            debugColor = vec3f(util::randomUniformf(), util::randomUniformf(), util::randomUniformf());
    }

    mat4f frameToWorldMatrix() const
    {
        mat4f rotation = mat4f::identity();
        mat4f translation = mat4f::translation(vec3f((float)camera[3], (float)camera[4], (float)camera[5]));

        const vec3f axis = vec3f((float)camera[0], (float)camera[1], (float)camera[2]);
        const float angle = axis.length();
        if (angle >= 1e-10f)
        {
            rotation = mat4f::rotation(axis.getNormalized(), math::radiansToDegrees(angle));
        }
        return translation * rotation;
    }

    mat4f worldToFrameMatrix() const
    {
        return frameToWorldMatrix().getInverse();
    }

    bool isValidCamera() const
    {
        return index == 0 || !(camera[0] == 0.0 && camera[5] == 0.0);
    }

    vec3f localPos(const vec2i &depthPixel) const;

    int index;
    
    Bitmap colorImage;
    DepthImage32 depthImage;

    mat4f depthIntrinsicInverse;

    vector<Keypoint> keypoints;

    //camera[0,1,2] are the angle-axis rotation
    //camera[3,4,5] are the translation
    double camera[6];

    Cameraf debugCamera;
    vec3f debugColor;
};

struct ImageCorrespondence
{
    ImageCorrespondence()
    {
        weight = 1.0f;
    }
    int imageA;
    int imageB;

    vec2i ptAPixel; // 2D pixel in image A
    vec3f ptALocal; // 3D point in the local depth frame of A

    vec2i ptBPixel; // 2D pixel in image B
    vec3f ptBLocal; // 3D point in the local depth frame of B

    float weight;

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

    BundlerFrame *imageA;
    BundlerFrame *imageB;

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
