
#include "main.h"

const int debugMaxFrameCount = 4;

vec3f BundlerImage::localPos(const vec2i &depthPixel, const mat4f &depthIntrinsicInverse) const
{
    if (!depthImage.isValidCoordinate(depthPixel))
        return vec3f(numeric_limits<float>::infinity(), numeric_limits<float>::infinity(), numeric_limits<float>::infinity());

    const float depth = depthImage(depthPixel);
    if (!depthImage.isValidValue(depth))
        return vec3f(numeric_limits<float>::infinity(), numeric_limits<float>::infinity(), numeric_limits<float>::infinity());

    const vec4f world = depthIntrinsicInverse * vec4f((float)depthPixel.x * depth, (float)depthPixel.y * depth, depth, 0.0f);
    return world.getVec3();
}

vec2i BundlerManager::imagePixelToDepthPixel(const vec2f &imageCoord) const
{
    // assumes remapped sensor file!
    return math::round(imageCoord);
}

void BundlerManager::loadSensorFile(const string &filename)
{
    BinaryDataStreamFile in(filename, false);
    CalibratedSensorData data;
    in >> data;
    in.closeStream();

    if (data.m_ColorImageWidth != data.m_DepthImageWidth ||
        data.m_ColorImageHeight != data.m_DepthImageHeight ||
        data.m_ColorImages.size() != data.m_DepthImages.size())
    {
        cout << "Sensor file not remapped" << endl;
        return;
    }

    const int width = data.m_ColorImageWidth;
    const int height = data.m_ColorImageHeight;
    const int imageCount = min((int)data.m_ColorImages.size(), debugMaxFrameCount);

    images.resize(imageCount);

    depthIntrinsicInverse = data.m_CalibrationDepth.m_IntrinsicInverse;

    for (auto &image : iterate(images))
    {
        image.value.index = (int)image.index;
        image.value.colorImage.allocate(width, height);
        image.value.depthImage.allocate(width, height);

        memcpy(image.value.colorImage.getData(), data.m_ColorImages[image.index], sizeof(vec4uc) * width * height);
        memcpy(image.value.depthImage.getData(), data.m_DepthImages[image.index], sizeof(float) * width * height);

        SAFE_DELETE_ARRAY(data.m_ColorImages[image.index]);
        SAFE_DELETE_ARRAY(data.m_DepthImages[image.index]);
    }
}

void BundlerManager::addCorrespondences(int forwardSkip)
{
    for (auto &startImage : images)
    {
        addCorrespondences(startImage.index, startImage.index + forwardSkip);
    }
}

void BundlerManager::computeKeypoints()
{
    FeatureExtractor extractor;
    for (auto &i : images)
    {
        i.keypoints = extractor.detectAndDescribe(i.colorImage);
    }
}

void BundlerManager::addCorrespondences(int imageAIndex, int imageBIndex)
{
    if (imageBIndex >= images.size())
        return;

    const BundlerImage &imageA = images[imageAIndex];
    const BundlerImage &imageB = images[imageBIndex];

    KeypointMatcher matcher;
    auto matches = matcher.match(imageA.keypoints, imageB.keypoints);

    ImageCorrespondences correspondences;
    correspondences.imageA = imageAIndex;
    correspondences.imageB = imageBIndex;

    for (auto &match : matches)
    {
        ImageCorrespondence corr;
        
        corr.imageA = imageAIndex;
        corr.imageB = imageBIndex;

        corr.keyPtDist = match.distance;

        const vec2i depthPixelA = imagePixelToDepthPixel(imageA.keypoints[match.indexA].pt);
        const vec2i depthPixelB = imagePixelToDepthPixel(imageB.keypoints[match.indexB].pt);

        corr.ptALocal = imageA.localPos(depthPixelA, depthIntrinsicInverse);
        corr.ptBLocal = imageB.localPos(depthPixelB, depthIntrinsicInverse);

        corr.residual = -1.0f;

        if (corr.ptALocal.x == corr.ptALocal.x &&
            corr.ptBLocal.x == corr.ptBLocal.x)
            correspondences.data.push_back(corr);
    }

    allCorrespondences.push_back(correspondences);
}
