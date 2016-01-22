
#include "main.h"

//const int debugMaxFrameCount = 200;
const int debugMaxFrameCount = numeric_limits<int>::max();

vec2i BundlerManager::imagePixelToDepthPixel(const vec2f &imageCoord) const
{
    // assumes remapped sensor file!
    return math::round(imageCoord);
}

void BundlerManager::loadSensorFile(const string &filename)
{
    cout << "Reading sensor file" << endl;
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
    const int frameCount = min((int)data.m_ColorImages.size(), debugMaxFrameCount);

    cout << "Creating frames" << endl;
    frames.resize(frameCount);

    for (auto &frame : iterate(frames))
    {
        frame.value.index = (int)frame.index;
        frame.value.colorImage.allocate(width, height);
        frame.value.depthImage.allocate(width, height);
        frame.value.depthIntrinsicInverse = data.m_CalibrationDepth.m_IntrinsicInverse;

        memcpy(frame.value.colorImage.getData(), data.m_ColorImages[frame.index], sizeof(vec4uc) * width * height);
        memcpy(frame.value.depthImage.getData(), data.m_DepthImages[frame.index], sizeof(float) * width * height);

        SAFE_DELETE_ARRAY(data.m_ColorImages[frame.index]);
        SAFE_DELETE_ARRAY(data.m_DepthImages[frame.index]);
    }
}

void BundlerManager::addCorrespondences(int forwardSkip)
{
    cout << "Adding correspondesnces (skip=" << forwardSkip << ")" << endl;
    for (auto &startImage : frames)
    {
        addCorrespondences(startImage.index, startImage.index + forwardSkip);
    }
}

void BundlerManager::computeKeypoints()
{
    cout << "Computing image keypoints" << endl;
    FeatureExtractor extractor;
    for (auto &i : frames)
    {
        i.keypoints = extractor.detectAndDescribe(i.colorImage);
    }
}

void BundlerManager::addCorrespondences(int frameAIndex, int frameBIndex)
{
    if (frameBIndex >= frames.size())
        return;

    const BundlerFrame &imageA = frames[frameAIndex];
    const BundlerFrame &imageB = frames[frameBIndex];

    KeypointMatcher matcher;
    auto matches = matcher.match(imageA.keypoints, imageB.keypoints);

    ImagePairCorrespondences correspondences;
    correspondences.imageA = &frames[frameAIndex];
    correspondences.imageB = &frames[frameBIndex];

    for (auto &match : matches)
    {
        ImageCorrespondence corr;
        
        corr.imageA = frameAIndex;
        corr.imageB = frameBIndex;

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

void BundlerManager::solve()
{
    ceres::Problem problem;
    int totalCorrespondences = 0;
    for (const ImagePairCorrespondences &iCorr : allCorrespondences)
    {
        BundlerFrame &frameA = *iCorr.imageA;
        BundlerFrame &frameB = *iCorr.imageB;

        for (const ImageCorrespondence &c : iCorr.inlierCorr)
        {
            ceres::CostFunction* costFunction = CorrespondenceCostFunc::Create(c);
            problem.AddResidualBlock(costFunction, NULL, frameA.camera, frameB.camera);
            totalCorrespondences++;
        }
    }

    for (const Keypoint &keypt : frames[0].keypoints)
    {
        const vec3f framePos = frames[0].localPos(keypt.pt);
        if (!framePos.isValid())
            continue;

        ceres::CostFunction* costFunction = AnchorCostFunc::Create(framePos, 100.0f);
        problem.AddResidualBlock(costFunction, NULL, frames[0].camera);
    }

    cout << "Total correspondences: " << totalCorrespondences << endl;
    
    ceres::Solver::Options options;
    //options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.linear_solver_type = ceres::
        ;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 100000;
    options.max_num_consecutive_invalid_steps = 100;
    options.function_tolerance = 1e-7;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << std::endl;
}

void BundlerManager::saveKeypointCloud(const string &outputFilename) const
{
    PointCloudf cloud;
    
    for (const BundlerFrame &frame : frames)
    {
        const mat4f m = frame.frameToWorldMatrix();
        
        /*for (const Keypoint &keypt : frame.keypoints)
        {
            const vec3f framePos = frame.localPos(keypt.pt);
            if (!framePos.isValid())
                continue;

            cloud.m_points.push_back(m * framePos);
            cloud.m_colors.push_back(vec4f(keypt.color) / 255.0f);
        }*/

        const int stride = 5;
        for (auto &p : frame.depthImage)
        {
            vec2i coord((int)p.x, (int)p.y);
            const vec3f framePos = frame.localPos(coord);
            if (!framePos.isValid())
                continue;

            if (p.x % stride != 0 || p.y % stride != 0)
                continue;

            cloud.m_points.push_back(m * framePos);
            cloud.m_colors.push_back(vec4f(frame.colorImage(coord)) / 255.0f);
        }
    }

    PointCloudIOf::saveToFile(outputFilename, cloud);
}
