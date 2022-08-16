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

using namespace geometrycentral;
using namespace geometrycentral::surface;

// == Geometry-central data
std::unique_ptr<ManifoldSurfaceMesh> mesh;
std::unique_ptr<VertexPositionGeometry> geometry;

// Polyscope visualization handle, to quickly add data to the surface
polyscope::SurfaceMesh* psMesh;

// The sampler object
std::unique_ptr<PoissonDiskSampler> poissonSampler;

// UI parameters
std::string MESHNAME = "input mesh";
double RCOEF = 1.0;
int KCANDIDATES = 30;
int RAVOID = -1;
std::vector<SurfacePoint> POINTS_TO_AVOID = {};
std::vector<SurfacePoint> SAMPLES;


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

void myCallback() {

    if (ImGui::Button("Sample")) {
        SAMPLES = poissonSampler->sample(RCOEF, KCANDIDATES, POINTS_TO_AVOID, RAVOID);
        visualizeSamples();
    }

    // Options
    ImGui::InputDouble("Scale distance", &RCOEF, 0.0, 0.0, "%.6f");
    // ImGui::SameLine();
    // if (ImGui::SliderFloat("", &poissonSampler->rCoef, 0.0, 0.0)) {
    //     SAMPLES = poissonSampler->sample(RCOEF, KCANDIDATES, POINTS_TO_AVOID);
    //     visualizeSamples();
    // }

    ImGui::InputInt("kCandidates", &KCANDIDATES);


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

    // Initialize sampler object.
    poissonSampler = std::unique_ptr<PoissonDiskSampler>(new PoissonDiskSampler(*mesh, *geometry));

    // Register the mesh with polyscope
    MESHNAME = polyscope::guessNiceNameFromPath(loadedFilename);
    psMesh = polyscope::registerSurfaceMesh(MESHNAME, geometry->inputVertexPositions, mesh->getFaceVertexList(),
                                            polyscopePermutations(*mesh));

    polyscope::show();

    return EXIT_SUCCESS;
}