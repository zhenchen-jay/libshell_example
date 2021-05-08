#include "CLI11.hpp"
#include "DynamicSolver.h"
#include "ShellIO.h"
#include "common.h"

#include "../include/MeshConnectivity.h"
#include "../include/ElasticShell.h"
#include "../include/MidedgeAngleTanFormulation.h"
#include "../include/MidedgeAngleSinFormulation.h"
#include "../include/MidedgeAverageFormulation.h"
#include "../include/StVKMaterial.h"
#include "../include/TensionFieldStVKMaterial.h"
#include "../include/NeoHookeanMaterial.h"

void lameParameters(const double& poisson, const double& youngs, double& alpha, double& beta)
{
    alpha = youngs * poisson / (1.0 - poisson * poisson);
    beta = youngs / 2.0 / (1.0 + poisson);
}

template <class SFF>
void runSimulation(
    const MeshConnectivity &restMesh,
    const MeshConnectivity &mesh,
    const Eigen::MatrixXd &restPos,
    const Eigen::MatrixXd& extForce,
    const std::map<int, Eigen::Vector3d>& clampedDOFs,
    Eigen::MatrixXd &curVel,
    Eigen::MatrixXd &curPos, 
    DynamicArgs dynamicArgs,
    MaterialParams matParams,
    NewtonSolverParams newtonParams
    )
{
    // initialize default edge DOFs (edge director angles)
    Eigen::VectorXd edgeDOFs;
    SFF::initializeExtraDOFs(edgeDOFs, mesh, curPos);
    // initialize first fundamental forms to those of input mesh
    std::vector<Eigen::Matrix2d> abar;
    ElasticShell<SFF>::firstFundamentalForms(restMesh, restPos, abar);

    // initialize second fundamental forms to rest flat
    std::vector<Eigen::Matrix2d> bbar;
    bbar.resize(restMesh.nFaces());
    for (int i = 0; i < restMesh.nFaces(); i++)
        bbar[i].setZero();

    double lameAlpha, lameBeta;
    lameParameters(matParams.poisson, matParams.youngs, lameAlpha, lameBeta);
    MaterialModel<SFF> *mat;
    switch (matParams.matid)
    {
    case 0:
        mat = new NeoHookeanMaterial<SFF>(lameAlpha, lameBeta);
        break;
    case 1:
        mat = new StVKMaterial<SFF>(lameAlpha, lameBeta);
        break;
    case 2:
        mat = new TensionFieldStVKMaterial<SFF>(lameAlpha, lameBeta);
        break;
    default:
        assert(false);
    }
    Eigen::VectorXd thicknesses(mesh.nFaces());
    thicknesses.setConstant(matParams.thickness);

    int nverts = curPos.rows();
    std::vector<double> massVec(nverts, 0);

    for (int i = 0; i < restMesh.nFaces(); i++)
    {
        double area = 0.5 * sqrt(abar[i].determinant()) / 3.0;
        for (int j = 0; j < 3; j++)
            massVec[mesh.faceVertex(i, j)] += area * thicknesses(i) * matParams.density;
    }

    std::map<int, Eigen::Vector3d> initPos = clampedDOFs, targetPos = clampedDOFs;
    for (auto& it : initPos)
        it.second = curPos.row(it.first).transpose();

    for (int j = 0; j < dynamicArgs.numSteps; j++)
    {
        std::cout << std::endl << "dynamic step: " << j << std::endl;
        std::map<int, Eigen::Vector3d> intermeidatePos = initPos;
        double rate = 1.0 / dynamicArgs.numSteps * (j + 1);

        for (auto& it : intermeidatePos)
        {
            it.second = (1 - rate) * initPos[it.first] + rate * targetPos[it.first];
        }

        Eigen::MatrixXd prePos = curPos;
        int flag = takeOneStep(mesh, curVel, curPos, extForce, clampedDOFs, edgeDOFs, *mat, thicknesses, abar, bbar, massVec, dynamicArgs.timeStep, matParams, newtonParams);
        curVel = (curPos - prePos) / dynamicArgs.timeStep;
    }

    std::string outPutFolder = dynamicArgs.output;
    std::string meshFile = outPutFolder + "/mesh_simulated.obj";
    igl::writeOBJ(meshFile, curPos, mesh.faces());

    // velocity
    std::string velFileName = outPutFolder + "/vel_simulated.txt";
    std::ofstream vfs(velFileName);
    if (vfs)
    {
        for (int i = 0; i < curVel.rows(); i++)
        {
            vfs << std::setprecision(std::numeric_limits<long double>::digits10 + 1) << curVel.row(i) << std::endl;
        }
    }

    // edge dofs
    std::string edgeFileName = outPutFolder + "/edgedofs_simulated.txt";
    std::ofstream efs(edgeFileName);
    if (efs)
    {
        for (int i = 0; i < edgeDOFs.rows(); i++)
        {
            efs << std::setprecision(std::numeric_limits<long double>::digits10 + 1) << edgeDOFs(i) << std::endl;
        }
    }

    delete mat;
}

int main(int argc, char* argv[])
{
    DynamicArgs dynamicArgs;
    NewtonSolverParams newtonParams;
    MaterialParams materialParams;

    // Parse arguments
    CLI::App app("ShellSolver");
    app.add_option("input,-i,--input", dynamicArgs.input, "Input json file.");
    app.add_option("output,-o,--output", dynamicArgs.output, "Output folder.");
    app.add_option("numSteps,-n,--numSteps", dynamicArgs.numSteps, "Number of dynamic steps");
    app.add_option("timeStep,-t,--timeStep", dynamicArgs.timeStep, "Time step size");


    try {
        app.parse(argc, argv);
    }
    catch (const CLI::ParseError& e) {
        return app.exit(e);
    }


    Eigen::MatrixXd restPos, curPos, curVel, extForce;
    Eigen::MatrixXi restF, curF;

    std::map<int, Eigen::Vector3d> clampedDOFs;

    bool ok = loadShell(dynamicArgs.input.c_str(), restPos, restF, curPos, curF, curVel, extForce, clampedDOFs, materialParams, newtonParams);
    if (!ok)
        return 0;
    MeshConnectivity restMesh(restF), mesh(curF);

    switch (materialParams.sffid)
    {
    case 0:
        runSimulation<MidedgeAngleTanFormulation>(restMesh, mesh, restPos, extForce, clampedDOFs, curVel, curPos, dynamicArgs, materialParams, newtonParams);
        break;
    case 1:
        runSimulation<MidedgeAngleSinFormulation>(restMesh, mesh, restPos, extForce, clampedDOFs, curVel, curPos, dynamicArgs, materialParams, newtonParams);
        break;
    case 2:
        runSimulation<MidedgeAverageFormulation>(restMesh, mesh, restPos, extForce, clampedDOFs, curVel, curPos, dynamicArgs, materialParams, newtonParams);
        break;
    default:
        assert(false);
    }
    return 0;
}
