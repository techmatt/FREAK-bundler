
class DenseCheck
{
public:
    struct Result
    {
        Result()
        {
            commonPointCount = 0;
            overlapRatio = 0.0f;
            totalAvgResidual = 0.0f;
            overlapAvgResidual = 0.0f;
        }
        int commonPointCount;
        float overlapRatio;
        float totalAvgResidual;
        float overlapAvgResidual;
    };

    static Result run(const std::string &pointCloudFileA, const std::string &pointCloudFileB, const mat4f &transformBToA, float matchDist, float colorMatchDist)
    {
        auto cloudA = PointCloudIOf::loadFromFile(pointCloudFileA);
        auto cloudB = PointCloudIOf::loadFromFile(pointCloudFileB);
        return run(cloudA, cloudB, transformBToA, matchDist, colorMatchDist);
    }

    static Result run(const PointCloudf &pointCloudA, const PointCloudf &pointCloudB, const mat4f &transformBToA, float matchDist, float colorMatchDist)
    {
        if (pointCloudA.m_points.size() < pointCloudB.m_points.size())
        {
            return run(pointCloudB, pointCloudA, transformBToA.getInverse(), matchDist, colorMatchDist);
        }

        const float matchDistSq = matchDist * matchDist;
        const float colorMatchDistSq = colorMatchDist * colorMatchDist;
        
        UniformAccelerator accelA(pointCloudA.m_points, matchDist);

        Result result;

        for (int bIndex = 0; bIndex < pointCloudB.m_points.size(); bIndex++)
        {
            const vec3f &bPt = pointCloudB.m_points[bIndex];

            const vec3f aPt = transformBToA * bPt;
            auto closestAPt = accelA.findClosestPoint(aPt);
            
            const float distSq = vec3f::distSq(closestAPt.second, aPt);
            const float colorDistSq = vec3f::distSq(pointCloudA.m_colors[closestAPt.first].getVec3(), pointCloudB.m_colors[bIndex].getVec3());

            if (distSq <= matchDistSq && colorDistSq <= colorMatchDistSq)
            {
                const float dist = sqrtf(distSq);
                result.commonPointCount++;
                result.totalAvgResidual += dist;
                result.overlapAvgResidual += dist;
            }
            else
            {
                result.totalAvgResidual += matchDist;
            }
        }

        if (result.commonPointCount == 0)
        {
            return Result();
        }

        result.overlapRatio = (float)result.commonPointCount / (float)math::min(pointCloudA.m_points.size(), pointCloudB.m_points.size());
        result.totalAvgResidual /= (float)pointCloudB.m_points.size();
        result.overlapAvgResidual /= (float)result.commonPointCount;
        return result;
    }
};