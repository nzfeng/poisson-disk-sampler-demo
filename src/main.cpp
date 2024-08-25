#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/poisson_disk_sampler.h"
#include "geometrycentral/surface/surface_point.h"
#include "geometrycentral/surface/vertex_position_geometry.h"

#include "polyscope/point_cloud.h"
#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"

#include "args/args.hxx"
#include "imgui.h"

#include <chrono>
using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;

using namespace geometrycentral;
using namespace geometrycentral::surface;

// == Geometry-central data
std::unique_ptr<ManifoldSurfaceMesh> mesh;
std::unique_ptr<VertexPositionGeometry> geometry;

// Polyscope visualization handle, to quickly add data to the surface
polyscope::SurfaceMesh* psMesh;

// The sampler object
std::unique_ptr<PoissonDiskSampler> poissonSampler;

// parameters
std::string MESHNAME = "input mesh";
double UNIT; // user-defined distance unit to make it easier to reason about sampling radius
double RCOEF = 1.0;
int KCANDIDATES = 30;
int RAVOID = 1;
bool USE_3D_AVOIDANCE = true;
std::vector<SurfacePoint> POINTS_TO_AVOID = {};
std::vector<SurfacePoint> SAMPLES;

std::tuple<Vector3, Vector3> boundingBox(VertexPositionGeometry& geometry_) {

    SurfaceMesh& mesh_ = geometry_.mesh;
    const double inf = std::numeric_limits<double>::infinity();
    Vector3 bboxMin = {inf, inf, inf};
    Vector3 bboxMax = {-inf, -inf, -inf};
    for (Vertex v : mesh_.vertices()) {
        Vector3& pos = geometry_.vertexPositions[v];
        for (int i = 0; i < 3; i++) {
            if (pos[i] <= bboxMin[i]) bboxMin[i] = pos[i];
            if (pos[i] >= bboxMax[i]) bboxMax[i] = pos[i];
        }
    }
    return std::make_tuple(bboxMin, bboxMax);
}

void visualizeSamples() {

    // remove everything
    psMesh->removeAllQuantities();

    std::vector<Vector3> cloud;
    for (const SurfacePoint& pt : SAMPLES) {
        Vector3 pos = pt.interpolate(geometry->inputVertexPositions);
        cloud.push_back(pos);
    }
    // Visualize balls at marked
    auto psCloud = polyscope::registerPointCloud("samples", cloud);
    psCloud->setPointColor(polyscope::render::RGB_BLACK)->setEnabled(true);
}

void updatePointsToAvoid() {

    std::vector<Vector3> cloud;
    for (const SurfacePoint& pt : POINTS_TO_AVOID) {
        Vector3 pos = pt.interpolate(geometry->inputVertexPositions);
        cloud.push_back(pos);
    }
    // Visualize balls at marked
    auto psCloud = polyscope::registerPointCloud("points to avoid", cloud);
    psCloud->setPointColor(polyscope::render::RGB_RED)->setEnabled(true);
}

/*
 * Save points as vertex positions in an OBJ file.
 */
void saveSamples(const std::vector<SurfacePoint>& points, const std::string& filepath) {

    std::fstream file;
    file.open(filepath, std::ios::out | std::ios::trunc);

    if (file.is_open()) {
        for (const SurfacePoint& pt : points) {
            Vector3 pos = pt.interpolate(geometry->inputVertexPositions);
            file << "v " << pos[0] << " " << pos[1] << " " << pos[2] << "\n";
        }
        std::cerr << "File " << filepath << " written succesfully." << std::endl;
        file.close();
    } else {
        std::cerr << "Could not save file '" << filepath << "'!" << std::endl;
    }
}

void myCallback() {

    if (ImGui::Button("Sample")) {

        double radius = RCOEF * UNIT;
        double radiusAvoid = RAVOID * UNIT;
        PoissonDiskOptions options;
        options.minDist = radius;
        options.kCandidates = KCANDIDATES;
        options.minDistAvoidance = radiusAvoid;
        options.pointsToAvoid = POINTS_TO_AVOID;
        options.use3DAvoidance = USE_3D_AVOIDANCE;

        auto t1 = high_resolution_clock::now();
        SAMPLES = poissonSampler->sample(options);
        auto t2 = high_resolution_clock::now();
        auto ms_int = duration_cast<milliseconds>(t2 - t1);
        std::cerr << "Sample time: " << ms_int.count() / 1000. << "s" << std::endl;

        visualizeSamples();

        // saveSamples(SAMPLES, "points_to_avoid.obj");
        // saveSamples(POINTS_TO_AVOID, "points_to_avoid.obj");
    }

    // Options
    ImGui::InputDouble("Scale distance", &RCOEF, 0.0, 0.0, "%.6f");

    ImGui::InputInt("kCandidates", &KCANDIDATES);

    ImGui::Checkbox("Use 3D radius of avoidance", &USE_3D_AVOIDANCE);

    // If selecting points manually, only vertices can be selected.
    if (ImGui::TreeNode("Add points to avoid")) {

        if (ImGui::Button("Push Vertex")) {

            long long int iV = psMesh->selectVertex();
            if (iV != -1) {
                Vertex v = mesh->vertex(iV);
                POINTS_TO_AVOID.push_back(SurfacePoint(v));
                updatePointsToAvoid();
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Pop Vertex")) {
            if (!POINTS_TO_AVOID.empty()) {
                POINTS_TO_AVOID.pop_back();
            }
            updatePointsToAvoid();
        }

        ImGui::InputInt("Radius of avoidance", &RAVOID);

        ImGui::TreePop();
    }
}

int main(int argc, char** argv) {

    // Configure the argument parser
    args::ArgumentParser parser("Poisson disk-sample a mesh.");
    args::Positional<std::string> inputFilename(parser, "mesh", "A mesh file.");

    // Parse args
    try {
        parser.ParseCLI(argc, argv);
    } catch (args::Help&) {
        std::cout << parser;
        return 0;
    } catch (args::ParseError& e) {
        std::cerr << e.what() << std::endl;
        std::cerr << parser;
        return 1;
    }

    // Make sure a mesh name was given
    if (!inputFilename) {
        std::cerr << "Please specify a mesh file as argument" << std::endl;
        return EXIT_FAILURE;
    }

    // Initialize polyscope
    polyscope::init();
    polyscope::state::userCallback = myCallback;

    // Load mesh
    std::string loadedFilename = args::get(inputFilename);
    std::tie(mesh, geometry) = readManifoldSurfaceMesh(loadedFilename);

    // Set distance unit.
    Vector3 bboxMin, bboxMax;
    std::tie(bboxMin, bboxMax) = boundingBox(*geometry);
    UNIT = (bboxMin - bboxMax).norm() / 100.;

    // Initialize sampler object.
    poissonSampler = std::unique_ptr<PoissonDiskSampler>(new PoissonDiskSampler(*mesh, *geometry));

    // Register the mesh with polyscope
    MESHNAME = polyscope::guessNiceNameFromPath(loadedFilename);
    psMesh = polyscope::registerSurfaceMesh(MESHNAME, geometry->inputVertexPositions, mesh->getFaceVertexList(),
                                            polyscopePermutations(*mesh));

    polyscope::show();

    return EXIT_SUCCESS;
}