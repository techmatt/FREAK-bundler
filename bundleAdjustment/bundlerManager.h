
struct BundlerManager
{
    void loadSensorFile(const string &filename);

    vec2i imagePixelToDepthPixel(const vec2f &imageCoord) const;

    void computeKeypoints();

    void addCorrespondences(int forwardSkip);
    void addCorrespondences(int imageAIndex, int imageBIndex);

    void visualize(const string &dir, int imageAIndex, int imageBIndex) const;

    vector<BundlerImage> images;
    vector<ImagePairCorrespondences> allCorrespondences;
};