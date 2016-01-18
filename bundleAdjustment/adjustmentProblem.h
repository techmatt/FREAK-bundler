
const bool useSquashFunction = false;

struct Fragment
{
    Fragment()
    {
        index = -1;
        for (auto &c : camera)
            c = 0.0;
        
        debugColor = vec3f(util::randomUniformf(), util::randomUniformf(), util::randomUniformf());
        while (debugColor.length() < 0.7f)
            debugColor = vec3f(util::randomUniformf(), util::randomUniformf(), util::randomUniformf());

        connectedToFragmentZero = false;
    }
    mat4f fragmentToWorldMatrix() const
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
    mat4f worldToFragmentMatrix() const
    {
        return fragmentToWorldMatrix().getInverse();
    }
    bool isValidCamera() const
    {
        return index == 0 || !(camera[0] == 0.0 && camera[5] == 0.0);
    }

    int index;

    //camera[0,1,2] are the angle-axis rotation
    //camera[3,4,5] are the translation
    double camera[6];

    vector<vec3f> keypoints;
    set<int> fragmentNeighbors;
    map<int, mat4f> fragmentNeighborTransform;

    string pointCloudPath;

    Cameraf debugCamera;
    vec3f debugColor;

    bool connectedToFragmentZero;
};

struct CorrespondenceError
{
    CorrespondenceError(const Correspondence &_corr)
        : corr(_corr) {}

    template <typename T> T squash(const T &x) const
    {
        const T a = (T)20.0;
        const T s = (T)0.5;
        return (( T(2.0) + x / a) / (T(1.0) + exp(T(-1.0) * abs(x * a))) - T(1.0)) * s;
    }

    template <typename T>
    bool operator()(const T* const cameraA, const T* const cameraB, T* residuals) const
    {
        // camera[0,1,2] are the angle-axis rotation
        // camera[3,4,5] are the translation
        
        T pA[3] = { T(corr.e[0].fragmentPos.x), T(corr.e[0].fragmentPos.y), T(corr.e[0].fragmentPos.z) };
        T pAWorld[3];
        ceres::AngleAxisRotatePoint(cameraA, pA, pAWorld);
        pAWorld[0] += cameraA[3]; pAWorld[1] += cameraA[4]; pAWorld[2] += cameraA[5];

        T pB[3] = { T(corr.e[1].fragmentPos.x), T(corr.e[1].fragmentPos.y), T(corr.e[1].fragmentPos.z) };
        T pBWorld[3];
        ceres::AngleAxisRotatePoint(cameraB, pB, pBWorld);
        pBWorld[0] += cameraB[3]; pBWorld[1] += cameraB[4]; pBWorld[2] += cameraB[5];
        
        // The error is the difference between the predicted and observed position.
        if (useSquashFunction)
        {
            residuals[0] = squash(pAWorld[0] - pBWorld[0]) * T(corr.weight);
            residuals[1] = squash(pAWorld[1] - pBWorld[1]) * T(corr.weight);
            residuals[2] = squash(pAWorld[2] - pBWorld[2]) * T(corr.weight);
        }
        else
        {
            residuals[0] = (pAWorld[0] - pBWorld[0]) * T(corr.weight);
            residuals[1] = (pAWorld[1] - pBWorld[1]) * T(corr.weight);
            residuals[2] = (pAWorld[2] - pBWorld[2]) * T(corr.weight);
        }
        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(const Correspondence &corr)
    {
        return (new ceres::AutoDiffCostFunction<CorrespondenceError, 3, 6, 6>(
            new CorrespondenceError(corr)));
    }

    Correspondence corr;
};

struct AnchorError
{
    AnchorError(const vec3f &_anchorPoint, float _weight)
        : anchorPoint(_anchorPoint), weight(_weight) {}

    template <typename T>
    bool operator()(const T* const camera, T* residuals) const
    {
        // camera[0,1,2] are the angle-axis rotation
        // camera[3,4,5] are the translation

        T p[3] = { T(anchorPoint.x), T(anchorPoint.y), T(anchorPoint.z) };
        T pWorld[3];
        ceres::AngleAxisRotatePoint(camera, p, pWorld);
        pWorld[0] += camera[3]; pWorld[1] += camera[4]; pWorld[2] += camera[5];

        // The error is the difference between the predicted and observed position.
        residuals[0] = (pWorld[0] - T(anchorPoint.x)) * T(weight);
        residuals[1] = (pWorld[1] - T(anchorPoint.y)) * T(weight);
        residuals[2] = (pWorld[2] - T(anchorPoint.z)) * T(weight);
        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(const vec3f &anchorPoint, float weight)
    {
        return (new ceres::AutoDiffCostFunction<AnchorError, 3, 6>(
            new AnchorError(anchorPoint, weight)));
    }

    vec3f anchorPoint;
    float weight;
};

struct AdjustmentProblem
{
    vector<Fragment> fragments;
    vector<Correspondence> correspondences;
    float anchorWeight;
    Grid2<FragmentMatch> matches;

    void initFromMatches(const string &pointCloudDirPrefix, const string &matchDir, int fragmentCount, const set<int> &excludedMatches);

    void initDebug();

    void solve();

    Grid2f runDenseCheck(const string &cacheFolder, set<int> &excludedMatchesOut, float overlapCutoff);
    void greedyInitialize(const Grid2f &scores);
    int evaluateResiduals(double residualThreshold, int maxToEliminate, const string &residualFilename, set<int> &excludedMatchesOut);

    void saveGlobalCloud(const string &outputFilename, float subsampleRatio) const;
};
