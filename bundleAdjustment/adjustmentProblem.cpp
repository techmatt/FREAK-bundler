
#include "main.h"

/*
void AdjustmentProblem::initFromMatches(const string &pointCloudDirPrefix, const string &matchDir, int fragmentCount, const set<int> &excludedMatches)
{
    fragments.clear();
    correspondences.clear();
    
    const int correspondenceCountThreshold = 20;
    const int strongCorrespondenceCountThreshold = 15;

    fragments.resize(fragmentCount);

    ofstream statsFile("matchStats.csv");
    statsFile << "FragA,fragB,corr,strong corr,ratio" << endl;
    matches.allocate(fragmentCount, fragmentCount);

    vector<FragmentMatch> goodMatches;

    for (int i = 0; i < fragmentCount; i++)
        for (int j = 0; j < fragmentCount; j++)
        {
            const string filename = matchDir + "match" + to_string(i) + "-" + to_string(j) + ".txt";
            //cout << filename << endl;
            if (util::fileExists(filename) && excludedMatches.count(getMatchHash(i, j)) == 0)
            {
                FragmentMatch match;
                match.loadASCII(filename, i, j);
                matches(i, j) = match;

                int strongCorrespondences = 0;
                for (auto &c : match.correspondences)
                    if (math::min(c.scoreAtoB, c.scoreBtoA) >= correspondenceStrengthCutoff)
                        strongCorrespondences++;
                float strengthRatio = (float)strongCorrespondences / (float)match.correspondences.size();

                    statsFile << i << "," << j << "," << match.correspondences.size() << "," << strongCorrespondences << "," << strengthRatio << endl;
                if (match.correspondences.size() >= correspondenceCountThreshold &&
                    strongCorrespondences >= strongCorrespondenceCountThreshold)
                {
                    goodMatches.push_back(match);
                    fragments[i].fragmentNeighborTransform[j] = match.transformBtoA.getInverse();
                    fragments[j].fragmentNeighborTransform[i] = match.transformBtoA;
                }
            }
        }

    for (int i = 0; i < fragmentCount; i++)
    {
        Fragment &fragment = fragments[i];
        fragment.index = i;
        fragment.pointCloudPath = pointCloudDirPrefix + to_string(i) + ".ply";
    }

    for (auto &match : goodMatches)
    {
        fragments[match.fragmentAIndex].fragmentNeighbors.insert(match.fragmentBIndex);
        fragments[match.fragmentBIndex].fragmentNeighbors.insert(match.fragmentAIndex);

        if (fragments[match.fragmentAIndex].keypoints.size() == 0)
            fragments[match.fragmentAIndex].keypoints = match.keypointsA;

        if (fragments[match.fragmentBIndex].keypoints.size() == 0)
            fragments[match.fragmentBIndex].keypoints = match.keypointsB;

        for (auto &c : match.correspondences)
            correspondences.push_back(c);
    }

    fragments[0].connectedToFragmentZero = true;
    for (int pass = 0; pass < fragmentCount; pass++)
    {
        for (auto &fragment : fragments)
        {
            if (fragment.connectedToFragmentZero)
                for (int neighbor : fragment.fragmentNeighbors)
                    fragments[neighbor].connectedToFragmentZero = true;
        }
    }
}

void AdjustmentProblem::initDebug()
{
    const int worldPointCount = 10;
    const int fragmentCount = 2;
    const int correspondenceCount = worldPointCount;

    vector<vec3f> worldPoints;
    auto r = []() { return util::randomUniform(-1.0f, 1.0f);  };
    for (int i = 0; i < worldPointCount; i++)
        worldPoints.push_back(vec3f(r(), r(), r()));

    fragments.resize(fragmentCount);
    correspondences.resize(correspondenceCount);

    for (int i = 0; i < fragmentCount; i++)
    {
        Fragment &fragment = fragments[i];

        const float theta = i * 90.0f;
        const vec3f eye = mat4f::rotationZ(theta) * vec3f(5.0f, 0.0f, 0.0f);
        fragment.debugCamera = Cameraf(eye, -eye, vec3f(0.0f, 0.0f, 1.0f), 60.0f, 1.0f, 0.01f, 100.0f);
        for (const vec3f &worldPos : worldPoints)
        {
            vec3f fragmentPos = fragment.debugCamera.getCamera() * worldPos;
            fragment.keypoints.push_back(fragmentPos);
        }
    }

    for (int i = 0; i < worldPointCount; i++)
    {
        correspondences[i].weight = 1.0f;

        correspondences[i].e[0].fragmentIndex = 0;
        correspondences[i].e[0].fragmentPos = fragments[0].keypoints[i];

        correspondences[i].e[1].fragmentIndex = 1;
        correspondences[i].e[1].fragmentPos = fragments[1].keypoints[i];
    }

    fragments[0].debugColor = vec3f(0.3f, 0.3f, 1.0f);
    fragments[1].debugColor = vec3f(1.0f, 0.3f, 0.3f);
}

void AdjustmentProblem::solve()
{
    ceres::Problem problem;
    for (const Correspondence &correspondence : correspondences)
    {
        ceres::CostFunction* costFunction = CorrespondenceError::Create(correspondence);

        Fragment &fragA = fragments[correspondence.e[0].fragmentIndex];
        Fragment &fragB = fragments[correspondence.e[1].fragmentIndex];

        problem.AddResidualBlock(costFunction, NULL, fragA.camera, fragB.camera);
    }

    for (auto &pos : fragments[0].keypoints)
    {
        ceres::CostFunction* costFunction = AnchorError::Create(pos, 100.0f);

        problem.AddResidualBlock(costFunction, NULL, fragments[0].camera);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 100000;
    options.max_num_consecutive_invalid_steps = 100;
    options.function_tolerance = 1e-8;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << std::endl;
}

void AdjustmentProblem::saveGlobalCloud(const string &outputFilename, float subsampleRatio) const
{
    PointCloudf cloud;
    for (auto &fragment : fragments)
    {
        if (!fragment.isValidCamera() || !fragment.connectedToFragmentZero)
        {
            cout << "skipping fragment " << fragment.pointCloudPath << endl;
            continue;
        }
        const mat4f m = fragment.fragmentToWorldMatrix();
        PointCloudf fragmentCloud;
        PointCloudIOf::loadFromPLY(fragment.pointCloudPath, fragmentCloud);
        for (const vec3f &v : fragmentCloud.m_points)
        {
            if (util::randomUniform() <= subsampleRatio)
            {
                cloud.m_points.push_back(m * v);
                cloud.m_colors.push_back(vec4f(fragment.debugColor, 1.0f));
            }
        }
    }
    PointCloudIOf::saveToFile(outputFilename, cloud);
}
*/