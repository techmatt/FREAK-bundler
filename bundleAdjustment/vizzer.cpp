
#include "main.h"

void Vizzer::init(ApplicationData &app)
{
    assets.init(app.graphics);

    vec3f eye(1.0f, 1.0f, 4.5f);
    vec3f worldUp(0.0f, 0.0f, 1.0f);
    camera = Cameraf(-eye, eye, worldUp, 60.0f, (float)app.window.getWidth() / app.window.getHeight(), 0.01f, 10000.0f);

    font.init(app.graphics, "Calibri");

    state.bundler.loadSensorFile(constants::dataDir + "/sensors/sample.sensor");
    state.bundler.computeKeypoints();
    state.bundler.addCorrespondences(1);
    state.bundler.addCorrespondences(2);
    state.bundler.addCorrespondences(3);
    state.bundler.addCorrespondences(4);
    state.bundler.solve();
    state.bundler.saveKeypointCloud(constants::debugDir + "result.ply");

    //state.bundler.saveImagePairCloud(0, 1, "out_0_1.ply");

    int a = 5;

    /*FeatureExtractor extractor;
    Bitmap bmpA = LodePNG::load("imageA.png");
    Bitmap bmpB = LodePNG::load("imageB.png");
    auto keyptsA = extractor.detectAndDescribe(bmpA);
    auto keyptsB = extractor.detectAndDescribe(bmpB);

    KeypointMatcher matcher;
    auto matches = matcher.match(keyptsA, keyptsB);

    auto splat = [](Bitmap &bmp, int x, int y, vec4uc color)
    {
        int radius = 1;
        for (int xOffset = -radius; xOffset <= radius; xOffset++)
            for (int yOffset = -radius; yOffset <= radius; yOffset++)
            {
                if (bmp.isValidCoordinate(x + xOffset, y + yOffset))
                    bmp(x + xOffset, y + yOffset) = color;
            }
    };

    for (auto &match : matches)
    {
        if (match.distance >= 50)
            continue;

        const Keypoint &a = keyptsA[match.indexA];
        const Keypoint &b = keyptsB[match.indexB];

        vec4uc matchColor((BYTE)util::randomInteger(64, 255),
                          (BYTE)util::randomInteger(64, 255),
                          (BYTE)util::randomInteger(64, 255), 255);

        splat(bmpA, math::round(a.pt.x), math::round(a.pt.y), matchColor);
        splat(bmpB, math::round(b.pt.x), math::round(b.pt.y), matchColor);
    }

    LodePNG::save(bmpA, "imageAOut.png");
    LodePNG::save(bmpB, "imageBOut.png");*/
}

void Vizzer::renderCamera(ApplicationData &app, const Cameraf &c, const vec3f &color)
{
    const float axisLength = 1.0f;
    const float radius = 0.1f;

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

    /*for (const Fragment &f : state.data.fragments)
    {
        renderCamera(app, f.debugCamera, f.debugColor);
    }*/

    const float borderRadius = 0.01f;
    assets.renderCylinder(camera.getCameraPerspective(), vec3f(1.0f, 0.0f, 0.0f), vec3f(1.0f, 1.0f, 0.0f), borderRadius, vec3f(1.0f, 0.0f, 0.0f));
    assets.renderCylinder(camera.getCameraPerspective(), vec3f(1.0f, 1.0f, 0.0f), vec3f(0.0f, 1.0f, 0.0f), borderRadius, vec3f(1.0f, 0.0f, 0.0f));
    assets.renderCylinder(camera.getCameraPerspective(), vec3f(0.0f, 1.0f, 0.0f), vec3f(0.0f, 0.0f, 0.0f), borderRadius, vec3f(1.0f, 0.0f, 0.0f));
    assets.renderCylinder(camera.getCameraPerspective(), vec3f(0.0f, 0.0f, 0.0f), vec3f(1.0f, 0.0f, 0.0f), borderRadius, vec3f(1.0f, 0.0f, 0.0f));

    vector<string> text;
    text.push_back(string("FPS: ") + convert::toString(timer.framesPerSecond()));
    
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
    const float distance = 1.0f;
    const float theta = 5.0f;

    //if (key == KEY_Z) physicsWorld.step();

    if(key == KEY_S) camera.move(-distance);
    if(key == KEY_W) camera.move(distance);
    if(key == KEY_A) camera.strafe(-distance);
    if(key == KEY_D) camera.strafe(distance);
	if(key == KEY_E) camera.jump(distance);
	if(key == KEY_Q) camera.jump(-distance);

    if(key == KEY_UP) camera.lookUp(theta);
    if(key == KEY_DOWN) camera.lookUp(-theta);
    if(key == KEY_LEFT) camera.lookRight(theta);
    if(key == KEY_RIGHT) camera.lookRight(-theta);
}

void Vizzer::mouseDown(ApplicationData &app, MouseButtonType button)
{

}

void Vizzer::mouseWheel(ApplicationData &app, int wheelDelta)
{
    const float distance = 0.01f;
    camera.move(distance * wheelDelta);
}

void Vizzer::mouseMove(ApplicationData &app)
{
    const float distance = 0.1f;
    const float theta = 0.4f;

    vec2i posDelta = app.input.mouse.pos - app.input.prevMouse.pos;

    if(app.input.mouse.buttons[MouseButtonRight])
    {
        camera.strafe(-distance * posDelta.x);
        camera.jump(distance * posDelta.y);
    }

    if(app.input.mouse.buttons[MouseButtonLeft])
    {
        camera.lookRight(-theta * posDelta.x);
        camera.lookUp(theta * posDelta.y);
    }

}
