
#include "main.h"

const float minOverlapScore = 0.3f;
const float correspondenceStrengthCutoff = 0.95f;
const float strengthRatioThreshold = 0.2f;

const float matchDist = 0.05f;
const float colorMatchDist = 0.3f;

int getMatchHash(int a, int b)
{
    if (a > b) swap(a, b);
    return a * 1024 + b;
}

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

void AdjustmentProblem::greedyInitialize(const Grid2f &overlapScores)
{
    util::makeDirectory("greedyPath");

    set<int> addedFragments;
    addedFragments.insert(0);

    vector<pair<int, int>> unacceptablePairs;
    //32-11??
    //23-12
    unacceptablePairs.push_back(make_pair(43, 45));
    unacceptablePairs.push_back(make_pair(43, 44));
    unacceptablePairs.push_back(make_pair(21, 8));
    unacceptablePairs.push_back(make_pair(46, 34));
    unacceptablePairs.push_back(make_pair(32, 27));
    unacceptablePairs.push_back(make_pair(44, 41));
    unacceptablePairs.push_back(make_pair(7, 21));
    unacceptablePairs.push_back(make_pair(38, 23));
    unacceptablePairs.push_back(make_pair(45, 33));
    unacceptablePairs.push_back(make_pair(46, 33));
    unacceptablePairs.push_back(make_pair(16, 33));
    unacceptablePairs.push_back(make_pair(47, 12));
    unacceptablePairs.push_back(make_pair(23, 12));
    unacceptablePairs.push_back(make_pair(15, 11));
    unacceptablePairs.push_back(make_pair(36, 12));
    unacceptablePairs.push_back(make_pair(37, 12));

    int greedyIndex = 0;
    bool done = false;
    while (!done)
    {
        float bestFragmentScore = 0.1f;
        int bestNeighborIndex = -1;
        int bestAnchorIndex = -1;

        for (int startFragment : addedFragments)
        {
            for (int neighborFragment : fragments[startFragment].fragmentNeighbors)
            {
                if (addedFragments.count(neighborFragment) > 0)
                    continue;
                
                float fragmentScore = 0.0f;

                float overlapScore = overlapScores(startFragment, neighborFragment);
                if (overlapScore < minOverlapScore)
                    fragmentScore = -1.0f;
                
                auto &match = matches(startFragment, neighborFragment);
                float correspondenceScore = match.strongCorrespondenceRatio(correspondenceStrengthCutoff);
                if (correspondenceScore <= strengthRatioThreshold)
                    fragmentScore = -1.0f;
                
                if (fragmentScore == 0.0f)
                    fragmentScore = correspondenceScore;

                for (auto &p : unacceptablePairs)
                {
                    if ((startFragment == p.first && neighborFragment == p.second) ||
                        (startFragment == p.second && neighborFragment == p.first))
                        fragmentScore = 0.0f;
                }

                if (fragmentScore >= bestFragmentScore)
                {
                    bestFragmentScore = fragmentScore;
                    bestNeighborIndex = neighborFragment;
                    bestAnchorIndex = startFragment;
                }
            }
        }
        
        if (bestAnchorIndex != -1)
        {
            cout << "Adding " << bestNeighborIndex << " anchored from " << bestAnchorIndex << ": " << bestFragmentScore << endl;
            addedFragments.insert(bestNeighborIndex);
            Fragment &anchorFragment = fragments[bestAnchorIndex];
            Fragment &neighborFragment = fragments[bestNeighborIndex];

            const mat4f transformAnchorToNeighbor = anchorFragment.fragmentNeighborTransform[bestNeighborIndex];
            const mat4f transformWorldToNeighbor = transformAnchorToNeighbor * anchorFragment.worldToFragmentMatrix();
            const vec6f newCamera = transformWorldToNeighbor.getInverse().convertToAxisAngleTranslation();
            
            cout << "  correspondence score: " << matches(bestAnchorIndex, bestNeighborIndex).strongCorrespondenceRatio(correspondenceStrengthCutoff) << endl;
            cout << "  overlap score: " << overlapScores(bestAnchorIndex, bestNeighborIndex) << endl;

            for (int i = 0; i < 6; i++)
                neighborFragment.camera[i] = newCamera[i];

            //mat4f diff = transformWorldToNeighbor - neighborFragment.fragmentToWorldMatrix();
            
            const string plyFilename = to_string(bestNeighborIndex) + "_" + to_string(bestAnchorIndex) + ".ply";
            if (util::fileExists("denseCache/" + plyFilename))
                util::copyFile("denseCache/" + plyFilename, "greedyPath/" + to_string(greedyIndex) + "-" + plyFilename);
        }

        if (bestAnchorIndex == -1)
        {
            cout << "No valid additions found, ending. total fragments: " << addedFragments.size() << endl;
            done = true;
        }

        greedyIndex++;
    }
}

Grid2f AdjustmentProblem::runDenseCheck(const string &cacheFolder, set<int> &excludedMatchesOut, float overlapCutoff)
{
    const bool dumpMeshes = true;

    util::makeDirectory(cacheFolder);
    const size_t fragmentCount = fragments.size();

    Grid2f overlapGrid(fragmentCount, fragmentCount, -1.0f);

#pragma omp parallel for schedule(dynamic,1)
    for (int i = 0; i < fragmentCount; i++)
        for (int j = 0; j < fragmentCount; j++)
            if (fragments[i].fragmentNeighbors.count(j) > 0)
            {
                const string &cacheFilename = cacheFolder + to_string(i) + "_" + to_string(j) + ".txt";
                if (!util::fileExists(cacheFilename))
                {
                    cout << "Running " << cacheFilename << endl;
                    auto result = DenseCheck::run(fragments[i].pointCloudPath, fragments[j].pointCloudPath, fragments[j].fragmentNeighborTransform[i], matchDist, colorMatchDist);
                    ofstream cacheFileOut(cacheFilename);
                    cacheFileOut << result.commonPointCount << ' ' << result.overlapAvgResidual << ' ' << result.overlapRatio << ' ' << result.totalAvgResidual << endl;
                }

                const string &meshFilename = cacheFolder + to_string(i) + "_" + to_string(j) + ".ply";
                if (dumpMeshes && !util::fileExists(meshFilename))
                {
                    cout << "Exporting " << meshFilename << endl;
                    PointCloudf cloudA, cloudB, cloudOut;
                    PointCloudIOf::loadFromPLY(fragments[i].pointCloudPath, cloudA);
                    PointCloudIOf::loadFromPLY(fragments[j].pointCloudPath, cloudB);
                    const mat4f transformBtoA = fragments[j].fragmentNeighborTransform[i];
                    for (const vec3f &v : cloudA.m_points)
                    {
                        cloudOut.m_points.push_back(v);
                        cloudOut.m_colors.push_back(vec4f(0.5f, 0.5f, 1.0f, 1.0f));
                    }
                    for (const vec3f &v : cloudB.m_points)
                    {
                        cloudOut.m_points.push_back(transformBtoA * v);
                        cloudOut.m_colors.push_back(vec4f(1.0f, 0.5f, 0.5f, 1.0f));
                    }
                    PointCloudIOf::saveToFile(meshFilename, cloudOut);
                }
            }

    ofstream file("denseCheck.csv");
    file << "index";
    for (int i = 0; i < fragmentCount; i++)
        file << "," << i;
    file << endl;
    for (int i = 0; i < fragmentCount; i++)
    {
        cout << "Dense check row " << i << endl;
        file << i;
        for (int j = i + 1; j < fragmentCount; j++)
        {
            if (fragments[i].fragmentNeighbors.count(j) > 0)
            {
                const string &cacheFilename = cacheFolder + to_string(i) + "_" + to_string(j) + ".txt";
                if (!util::fileExists(cacheFilename))
                {
                    cout << "Cache failed to initalize" << endl;
                }
                DenseCheck::Result result;
                ifstream cacheFileIn(cacheFilename);
                cacheFileIn >> result.commonPointCount >> result.overlapAvgResidual >> result.overlapRatio >> result.totalAvgResidual;
                file << "," << result.overlapRatio << "_" << result.overlapAvgResidual;

                if (result.overlapRatio < overlapCutoff)
                    excludedMatchesOut.insert(getMatchHash(i, j));

                auto &match = matches(i, j);
                float correspondenceScore = match.strongCorrespondenceRatio(correspondenceStrengthCutoff);
                //cout << i << "," << j << " = " << correspondenceScore << endl;
                if (correspondenceScore <= strengthRatioThreshold)
                    excludedMatchesOut.insert(getMatchHash(i, j));

                overlapGrid(i, j) = result.overlapRatio;
            }
            else
            {
                file << ",x";
            }
        }
        file << std::endl;
    }

    return overlapGrid;
}

int AdjustmentProblem::evaluateResiduals(double residualThreshold, int maxToEliminate, const string &residualFilename, set<int> &excludedMatchesOut)
{
    const size_t fragmentCount = fragments.size();
    Grid2f residualSum(fragmentCount, fragmentCount, 0.0f);
    Grid2i correspondenceCount(fragmentCount, fragmentCount, 0);

    vector< pair<double, int> > potentialExclusions;

    for (auto &c : correspondences)
    {
        const vec3f ptAWorld = fragments[c.e[0].fragmentIndex].fragmentToWorldMatrix() * c.e[0].fragmentPos;
        const vec3f ptBWorld = fragments[c.e[1].fragmentIndex].fragmentToWorldMatrix() * c.e[1].fragmentPos;
        const float residual = vec3f::dist(ptAWorld, ptBWorld);

        const bool useAvg = true;
        auto &r = residualSum(c.e[0].fragmentIndex, c.e[1].fragmentIndex);
        if (useAvg)
        {
            r += residual;
            correspondenceCount(c.e[0].fragmentIndex, c.e[1].fragmentIndex)++;
        }
        else
        {
            r = max(r, residual);
            correspondenceCount(c.e[0].fragmentIndex, c.e[1].fragmentIndex) = 1;
        }
    }

    for (int i = 0; i < fragmentCount; i++)
        for (int j = 0; j < fragmentCount; j++)
        {
            double residual = residualSum(i, j) / correspondenceCount(i, j);
            if (correspondenceCount(i, j) > 0 && residual >= residualThreshold)
                potentialExclusions.push_back(make_pair(residual, getMatchHash(i, j)));
                //excludedMatchesOut.insert(getMatchHash(i, j));
        }

    sort(potentialExclusions.begin(), potentialExclusions.end(),
        [](const pair<double, int> &a, const pair<double, int> &b) { return a.first > b.first; });

    if (potentialExclusions.size() > maxToEliminate)
        potentialExclusions.resize(maxToEliminate);

    for (auto &e : potentialExclusions)
    {
        cout << "Eliminating frame residual = " << e.first << endl;
        excludedMatchesOut.insert(e.second);
    }


    ofstream file(residualFilename);
    file << "index";
    for (int i = 0; i < fragmentCount; i++)
        file << "," << i;
    file << endl;
    for (int i = 0; i < fragmentCount; i++)
    {
        file << i;
        for (int j = 0; j < fragmentCount; j++)
        {
            if (correspondenceCount(i, j) == 0)
                file << ",x";
            else
                file << "," << residualSum(i, j) / correspondenceCount(i, j);
        }
        file << endl;
    }

    return (int)potentialExclusions.size();
}
