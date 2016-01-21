
#include "main.h"

const int debugMaxFrameCount = 500;
//const int debugMaxFrameCount = numeric_limits<int>::max();

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

    for (auto &image : iterate(images))
    {
        image.value.index = (int)image.index;
        image.value.colorImage.allocate(width, height);
        image.value.depthImage.allocate(width, height);
        image.value.depthIntrinsicInverse = data.m_CalibrationDepth.m_IntrinsicInverse;

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

    ImagePairCorrespondences correspondences;
    correspondences.imageA = &images[imageAIndex];
    correspondences.imageB = &images[imageBIndex];

    for (auto &match : matches)
    {
        ImageCorrespondence corr;
        
        corr.imageA = imageAIndex;
        corr.imageB = imageBIndex;

        corr.keyPtDist = match.distance;

        const vec2i depthPixelA = imagePixelToDepthPixel(imageA.keypoints[match.indexA].pt);
        const vec2i depthPixelB = imagePixelToDepthPixel(imageB.keypoints[match.indexB].pt);

        corr.ptAPixel = math::round(imageA.keypoints[match.indexA].pt);
        corr.ptALocal = imageA.localPos(depthPixelA);

        corr.ptBPixel = math::round(imageB.keypoints[match.indexB].pt);
        corr.ptBLocal = imageB.localPos(depthPixelB);

        corr.residual = -1.0f;

        if (corr.ptALocal.isValid() &&
            corr.ptBLocal.isValid())
            correspondences.allCorr.push_back(corr);
    }

    correspondences.estimateTransform();

    if (correspondences.transformInliers >= constants::minInlierCount)
        allCorrespondences.push_back(correspondences);
}
/*
void BundlerManager::saveImagePairCloud(int imageAIndex, int imageBIndex, const string &plyFilenameOut)
{
    ImagePairCorrespondences *correspondences = nullptr;
    for (auto &i : allCorrespondences)
    {
        if (i.imageA == imageAIndex &&
            i.imageB == imageBIndex)
        {
            correspondences = &i;
        }
    }
    if (correspondences == nullptr)
    {
        cout << "Image pair not found" << endl;
        return;
    }

    PointCloudf cloud;

    vector<TriMeshf> meshes;

    const BundlerImage &imageA = images[imageAIndex];
    const BundlerImage &imageB = images[imageBIndex];

    const vec3f AOffset = vec3f::eX * 0.1f;
    const vec3f BOffset = vec3f::eX * -0.1f;

    const vec4f AColor(1.0f, 0.5f, 0.5f, 1.0f);
    const vec4f BColor(0.5f, 0.5f, 1.0f, 1.0f);

    auto makeColoredBox = [](const vec3f &center, const vec4f &color, float radius) {
        TriMeshf result = ml::Shapesf::box(radius, radius, radius);
        result.transform(mat4f::translation(center));
        result.setColor(color);
        return result;
    };

    auto addMatch = [&](const ImageCorrespondence &m, const vec4f &matchColor, float scale)
    {
        cloud.m_points.push_back(m.ptALocal + AOffset);
        cloud.m_colors.push_back(matchColor);

        cloud.m_points.push_back(m.ptBLocal + BOffset);
        cloud.m_colors.push_back(matchColor);

        meshes.push_back(makeColoredBox(m.ptALocal + AOffset, matchColor, 0.015f * scale));
        meshes.push_back(makeColoredBox(m.ptBLocal + BOffset, matchColor, 0.015f * scale));

        const TriMeshf cylinder = Shapesf::cylinder(m.ptALocal + AOffset, m.ptBLocal + BOffset, 0.007f * scale, 2, 4, matchColor);
        meshes.push_back(cylinder);
    };

    for (auto &m : correspondences->inlierCorr)
    {
        vec4f matchColor(util::randomUniform(0.3f, 1.0f),
            util::randomUniform(0.3f, 1.0f),
            util::randomUniform(0.3f, 1.0f), 1.0f);
        addMatch(m, matchColor, 1.0f);
    }

    for (auto &m : correspondences->allCorr)
    {
        vec4f matchColor(0.5f, 0.5f, 0.5f, 1.0f);
        addMatch(m, matchColor, 0.6f);
    }

    const int stride = 2;
    for (auto &p : imageA.depthImage)
    {
        const vec3f pos = imageA.localPos(vec2i((int)p.x, (int)p.y), depthIntrinsicInverse);
        if (!pos.isValid())
            continue;

        if (p.x % stride != 0 || p.y % stride != 0)
            continue;

        meshes.push_back(makeColoredBox(pos + AOffset, AColor, 0.002f));
        //cloud.m_points.push_back(pos + AOffset);
        //cloud.m_colors.push_back(AColor);
    }

    for (auto &p : imageB.depthImage)
    {
        const vec3f pos = imageB.localPos(vec2i((int)p.x, (int)p.y), depthIntrinsicInverse);
        if (!pos.isValid())
            continue;

        if (p.x % stride != 0 || p.y % stride != 0)
            continue;

        meshes.push_back(makeColoredBox(pos + BOffset, BColor, 0.002f));
        //cloud.m_points.push_back(pos + BOffset);
        //cloud.m_colors.push_back(BColor);
    }

    std::vector<vec3f> unifiedVertices;
    std::vector<unsigned int> unifiedIndices;
    std::vector<vec4f> unifiedColors;
    std::vector<vec2f> unifiedTexCoords;
    std::vector<vec3f> unifiedNormals;

    int meshBaseVertex = 0;
    for (const auto &mesh : meshes)
    {
        for (auto &v : mesh.getVertices())
        {
            unifiedVertices.push_back(v.position);
            unifiedColors.push_back(v.color);
        }
        for (auto &i : mesh.getIndices())
        {
            unifiedIndices.push_back(i.x + meshBaseVertex);
            unifiedIndices.push_back(i.y + meshBaseVertex);
            unifiedIndices.push_back(i.z + meshBaseVertex);
        }
        meshBaseVertex += (int)mesh.getVertices().size();
    }

    TriMeshf unifiedMesh(unifiedVertices, unifiedIndices, unifiedColors, unifiedNormals, unifiedTexCoords);

    MeshIOf::saveToPLY(plyFilenameOut, unifiedMesh.getMeshData());
}
*/