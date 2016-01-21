
#include "main.h"

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

void ImagePairCorrespondences::estimateTransform()
{
    if (allCorr.size() < constants::minCorrespondenceCount)
    {
        transformInliers = 0;
        cout << "Too few correspondences: " << allCorr.size() << endl;
        return;
    }

    mat4f bestTransform = mat4f::identity();
    double bestError = std::numeric_limits<double>::max();
    for (int ransacIter = 0; ransacIter < constants::RANSACIters; ransacIter++)
    {
        set<int> indices;
        while (indices.size() < constants::samplesPerRANSAC)
        {
            const int index = util::randomInteger(0, (int)allCorr.size() / 2 + 1);
            indices.insert(index);
        }
        mat4f candidateTransform = estimateTransform(indices);
        double candidateError = computeTransformError(candidateTransform);
        if (candidateError < bestError)
        {
            bestError = candidateError;
            bestTransform = candidateTransform;
        }
    }

    //
    // recompute transform with all inliers
    //
    transformOutliers = 0;
    transformInliers = 0;
    double inlierDistSum = 0.0;
    set<int> inlierIndices;
    for (auto &c : iterate(allCorr))
    {
        const vec3f bPt = bestTransform * c.value.ptALocal;
        const float distSq = vec3f::distSq(bPt, c.value.ptBLocal);
        if (distSq < constants::outlierDistSq)
        {
            transformInliers++;
            inlierDistSum += sqrtf(distSq);
            inlierIndices.insert((int)c.index);
            inlierCorr.push_back(c.value);
        }
        else
        {
            transformOutliers++;
        }
    }
    transformInlierError = inlierDistSum / (double)transformInliers;

    if (transformInliers < 4)
    {
        cout << "Too few inliers to compute transform: " << imageA << "-" << imageB << endl;
    }
    else
        transformAToB = estimateTransform(inlierIndices);

    cout << "inliers: " << transformInliers << " / " << allCorr.size() << ", error: " << transformInlierError << endl;
}

mat4f ImagePairCorrespondences::estimateTransform(const set<int> &indices)
{
    //Matrix4x4<FloatType> kabsch(const std::vector<vec3<FloatType>>& source, const std::vector<vec3<FloatType>>& target, vec3<FloatType>& eigenvalues
    vec3f eigenvalues;
    vector<vec3f> source, target;
    for (int i : indices)
    {
        source.push_back(allCorr[i].ptALocal);
        target.push_back(allCorr[i].ptBLocal);
    }
    return EigenWrapperf::kabsch(source, target, eigenvalues);
}

double ImagePairCorrespondences::computeTransformError(const mat4f &transform)
{
    int outlierCount = 0;
    int inlierCount = 0;
    double inlierDistSum = 0.0;
    for (const ImageCorrespondence &c : allCorr)
    {
        const vec3f bPt = transform * c.ptALocal;
        const float distSq = vec3f::distSq(bPt, c.ptBLocal);
        if (distSq < constants::outlierDistSq)
        {
            inlierCount++;
            inlierDistSum += sqrtf(distSq);
        }
        else
        {
            outlierCount++;
        }
    }
    return inlierDistSum / (double)inlierCount + (double)outlierCount;
}
