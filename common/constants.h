
namespace constants
{
    const double SIFTHessian = 500.0;

    const int minCorrespondenceCount = 20;
    const int minInlierCount = 10;

    const int RANSACSamples = 3;

    const int RANSACEarlyIters = 50;
    const int RANSACEarlyInlierMin = 100;

    const int RANSACFullIters = 1000;
    
    const float outlierDist = 0.05f;
    const float outlierDistSq = outlierDist * outlierDist;

    const double CERESTolerance = 1e-7;

    const string dataDir = R"(C:\Code\FREAK-bundler\data\)";
    const string debugDir = dataDir + "debug/";
}