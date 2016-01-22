
#include "main.h"

struct App
{
    void go();

    BundlerManager bundler;
};

void App::go()
{
    bundler.loadSensorFile(constants::dataDir + "/sensors/sample.sensor");
    bundler.computeKeypoints();
    bundler.addCorrespondences(1);
    bundler.addCorrespondences(2);
    bundler.addCorrespondences(3);
    bundler.addCorrespondences(4);

    bundler.solve();

    bundler.thresholdCorrespondences(0.01);

    bundler.solve();

    bundler.saveKeypointCloud(constants::debugDir + "result.ply");
    bundler.saveResidualDistribution(constants::debugDir + "residuals.csv");
}

int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);

    App app;
    app.go();

    return 0;
}
