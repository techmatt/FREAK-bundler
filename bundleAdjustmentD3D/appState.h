
struct AppState
{
    BundlerManager bundler;
    vector<D3D11TriMesh> frameCloudsSmall;
    vector<D3D11TriMesh> frameCloudsBig;

    int selectedCamera;
};