#pragma once
#include <Eigen/Dense>
#include <vector>

struct DynamicArgs
{
    std::string input;       // input json file [Required]
    std::string output = ""; // if not empty, the folder will contains all the intermediate results

    int numSteps; // total number of steps taken in the dynamic solver
    double timeStep; // time step size in the dynamic solver
};

struct NewtonSolverParams
{
    int iterations = 1000; // number of iterations for dynamic solver
    double gradTol = 1e-6; // gradient norm tolerance
    double fDelta = 0;	// energy update tolerance
    double xDelta = 0; // variables update tolerance
};

struct MaterialParams
{
    double thickness;   // thickness of cloth
    double poisson;     // poisson ratio
    double youngs;      // youngs modulus
    double density;     // density

    Eigen::Vector3d gravity;    // gravity

    int matid;      // which material to use: StVK, Neo-Hookean, or tension field
    int sffid;      // the way to discretize the second fundamental form
};

struct ChosenVertex
{
    int vid;    // vertex id
    Eigen::Vector3d vel; // velocity
};
