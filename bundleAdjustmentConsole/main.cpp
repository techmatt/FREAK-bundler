
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

    bundler.allCorrespondences[0].visualize(constants::debugDir + "0_1/");

    bundler.solve();
    util::makeDirectory(constants::debugDir);
    bundler.saveKeypointCloud(constants::debugDir + "resultA.ply");
    bundler.saveResidualDistribution(constants::debugDir + "residualsA.csv");

    bundler.thresholdCorrespondences(0.01);
    bundler.solve();

    bundler.saveKeypointCloud(constants::debugDir + "resultB.ply");
    bundler.saveResidualDistribution(constants::debugDir + "residualsB.csv");

    bundler.thresholdCorrespondences(0.005);
    bundler.solve();

    bundler.saveKeypointCloud(constants::debugDir + "resultC.ply");
    bundler.saveResidualDistribution(constants::debugDir + "residualsC.csv");
}

int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);

    App app;
    app.go();

    return 0;
}
