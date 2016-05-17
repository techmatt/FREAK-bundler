
#include "main.h"

void Vizzer::init(ApplicationData &app)
{
    assets.init(app.graphics);

    vec3f eye(0.5f, 0.2f, 0.5f);
    vec3f worldUp(0.0f, 1.0f, 0.0f);
    camera = Cameraf(-eye, eye, worldUp, 60.0f, (float)app.window.getWidth() / app.window.getHeight(), 0.01f, 10000.0f);
    camera = Cameraf("-0.774448 1.24485 -1.35404 0.999848 1.80444e-009 -0.0174517 0.0152652 -0.484706 0.874544 -0.00845866 -0.874677 -0.484632 0 1 0 60 1.25 0.01 10000");
    //-0.774448 1.24485 -1.35404 0.999848 1.80444e-009 -0.0174517 0.0152652 -0.484706 0.874544 -0.00845866 -0.874677 -0.484632 0 1 0 60 1.25 0.01 10000
    font.init(app.graphics, "Calibri");

    state.bundler.loadSensorFile(constants::dataDir + "/sensors/sample.sensor");
    state.bundler.computeKeypoints();
    state.bundler.addCorrespondences(1);
    state.bundler.addCorrespondences(2);
    state.bundler.addCorrespondences(4);
    state.bundler.addCorrespondences(6);
    state.bundler.addCorrespondences(12);
    state.bundler.addCorrespondences(20);
    state.bundler.addCorrespondences(30);
    state.bundler.addCorrespondences(40);
    state.bundler.addCorrespondences(70);
    state.bundler.addCorrespondences(100);
    state.bundler.addCorrespondences(150);
    state.bundler.addCorrespondences(200);
    state.bundler.addCorrespondences(250);
    state.bundler.addCorrespondences(300);
    state.bundler.addCorrespondences(350);
    state.bundler.addCorrespondences(400);
    
    state.bundler.solve();
    
    state.bundler.thresholdCorrespondences(0.01);

    state.bundler.solve();

    state.bundler.thresholdCorrespondences(0.005);

    state.bundler.solve();

    state.bundler.saveKeypointCloud(constants::debugDir + "result.ply");
    state.bundler.saveResidualDistribution(constants::debugDir + "residuals.csv");

    state.frameCloudsSmall.resize(state.bundler.frames.size());
    state.frameCloudsBig.resize(state.bundler.frames.size());

    for (auto &frame : iterate(state.bundler.frames))
    {
        vector<TriMeshf> meshesSmall, meshesBig;

        auto makeColoredBox = [](const vec3f &center, const vec4f &color, float radius) {
            TriMeshf result = ml::Shapesf::box(radius, radius, radius);
            result.transform(mat4f::translation(center));
            result.setColor(color);
            return result;
        };

        const int stride = 5;
        for (auto &p : frame.value.depthImage)
        {
            vec2i coord((int)p.x, (int)p.y);
            const vec3f framePos = frame.value.localPos(coord);
            if (!framePos.isValid() || p.x % stride != 0 || p.y % stride != 0)
                continue;

            meshesSmall.push_back(makeColoredBox(frame.value.frameToWorld * framePos, vec4f(frame.value.colorImage(coord)) / 255.0f, 0.002f));
            meshesBig.push_back(makeColoredBox(frame.value.frameToWorld * framePos, vec4f(frame.value.colorImage(coord)) / 255.0f, 0.005f));
        }

        state.frameCloudsSmall[frame.index] = D3D11TriMesh(app.graphics, Shapesf::unifyMeshes(meshesSmall));
        state.frameCloudsBig[frame.index] = D3D11TriMesh(app.graphics, Shapesf::unifyMeshes(meshesBig));
    }

    state.selectedCamera = 0;
}

void Vizzer::renderCamera(ApplicationData &app, const Cameraf &c, const vec3f &color)
{
    const float axisLength = 0.15f;
    const float radius = 0.003f;

    const vec3f eye = c.getEye();
    const vec3f look = c.getLook() * axisLength;
    const vec3f right = c.getRight() * axisLength;
    const vec3f up = c.getUp() * axisLength;
    assets.renderSphere(camera.getCameraPerspective(), eye, radius * 1.5f, math::lerp(color, vec3f(1.0f, 1.0f, 1.0f), 0.25f));
    assets.renderCylinder(camera.getCameraPerspective(), eye, eye + look, radius, math::lerp(color, vec3f(0.0f, 0.0f, 1.0f), 0.25f));
    assets.renderCylinder(camera.getCameraPerspective(), eye, eye + right, radius, math::lerp(color, vec3f(0.0f, 1.0f, 0.0f), 0.25f));
    assets.renderCylinder(camera.getCameraPerspective(), eye, eye + up, radius, math::lerp(color, vec3f(1.0f, 0.0f, 0.0f), 0.25f));
}

void Vizzer::render(ApplicationData &app)
{
    timer.frame();

    for (const auto &f : iterate(state.bundler.frames))
    {
        vec3f cameraColor(0.5f, 0.5f, 0.9f);
        vec3f meshColor(1.0f, 1.0f, 1.0f);

        if (f.index == state.selectedCamera)
        {
            meshColor = vec3f(0.9f, 0.5f, 0.5f);
            cameraColor = vec3f(0.9f, 0.5f, 0.5f);
            assets.renderMesh(state.frameCloudsBig[f.index], camera.getCameraPerspective(), meshColor);
        }

        renderCamera(app, f.value.debugCamera, cameraColor);

        assets.renderMesh(state.frameCloudsSmall[f.index], camera.getCameraPerspective(), meshColor);
    }

    /*const float borderRadius = 0.01f;
    assets.renderCylinder(camera.getCameraPerspective(), vec3f(1.0f, 0.0f, 0.0f), vec3f(1.0f, 1.0f, 0.0f), borderRadius, vec3f(1.0f, 0.0f, 0.0f));
    assets.renderCylinder(camera.getCameraPerspective(), vec3f(1.0f, 1.0f, 0.0f), vec3f(0.0f, 1.0f, 0.0f), borderRadius, vec3f(1.0f, 0.0f, 0.0f));
    assets.renderCylinder(camera.getCameraPerspective(), vec3f(0.0f, 1.0f, 0.0f), vec3f(0.0f, 0.0f, 0.0f), borderRadius, vec3f(1.0f, 0.0f, 0.0f));
    assets.renderCylinder(camera.getCameraPerspective(), vec3f(0.0f, 0.0f, 0.0f), vec3f(1.0f, 0.0f, 0.0f), borderRadius, vec3f(1.0f, 0.0f, 0.0f));*/

    vector<string> text;
    text.push_back(string("FPS: ") + convert::toString(timer.framesPerSecond()));
    
    //if (rand() % 100 == 0)
    //    cout << camera.toString() << endl;

    const bool useText = true;
    if (useText)
        drawText(app, text);
}

void Vizzer::resize(ApplicationData &app)
{
    camera.updateAspectRatio((float)app.window.getWidth() / app.window.getHeight());
}

void Vizzer::drawText(ApplicationData &app, vector<string> &text)
{
    int y = 0;
    for (auto &entry : text)
    {
        font.drawString(app.graphics, entry, vec2i(10, 5 + y++ * 25), 24.0f, RGBColor::Red);
    }
}

void Vizzer::keyDown(ApplicationData &app, UINT key)
{
    if (key == KEY_F) app.graphics.castD3D11().toggleWireframe();
}

void Vizzer::keyPressed(ApplicationData &app, UINT key)
{
    const float distance = 0.1f;
    const float theta = 3.0f;

    //if (key == KEY_Z) physicsWorld.step();

    if(key == KEY_S) camera.move(-distance);
    if(key == KEY_W) camera.move(distance);
    if(key == KEY_A) camera.strafe(-distance);
    if(key == KEY_D) camera.strafe(distance);
	if(key == KEY_E) camera.jump(distance);
	if(key == KEY_Q) camera.jump(-distance);

    if(key == KEY_UP) camera.lookUp(-theta);
    if(key == KEY_DOWN) camera.lookUp(theta);
    if(key == KEY_LEFT) camera.lookRight(-theta);
    if(key == KEY_RIGHT) camera.lookRight(theta);

    if (key == KEY_Z) state.selectedCamera = math::mod(state.selectedCamera - 1, state.bundler.frames.size());
    if (key == KEY_X) state.selectedCamera = math::mod(state.selectedCamera + 1, state.bundler.frames.size());
}

void Vizzer::mouseDown(ApplicationData &app, MouseButtonType button)
{

}

void Vizzer::mouseWheel(ApplicationData &app, int wheelDelta)
{
    const float distance = 0.001f;
    camera.move(distance * wheelDelta);
}

void Vizzer::mouseMove(ApplicationData &app)
{
    const float distance = 0.01f;
    const float theta = 0.4f;

    vec2i posDelta = app.input.mouse.pos - app.input.prevMouse.pos;

    if(app.input.mouse.buttons[MouseButtonRight])
    {
        camera.strafe(distance * posDelta.x);
        camera.jump(-distance * posDelta.y);
    }

    if(app.input.mouse.buttons[MouseButtonLeft])
    {
        camera.lookRight(theta * posDelta.x);
        camera.lookUp(theta * posDelta.y);
    }

}
