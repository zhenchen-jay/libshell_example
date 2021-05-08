#pragma once
#include <json/json.h>
#include <igl/readOBJ.h>
#include <igl/writeOBJ.h>
#include <fstream>

#include "common.h"


bool parseTXT(const std::string& path, const int& n, Eigen::VectorXd& v)
{
	std::ifstream vfile(path);
	if (!vfile)
		return false;
	v.resize(n);
	v.setZero();
	for (int i = 0; i < n; i++)
		vfile >> v(i);
	return true;
}

bool parse(bool& b, const Json::Value& json) {
	if (!json.isBool())
		return false;
	b = json.asBool();
	return true;
}
bool parse(int& n, const Json::Value& json) {
	if (!json.isIntegral())
		return false;
	n = json.asInt();
	return true;
}
bool parse(double& x, const Json::Value& json) {
	if (!json.isNumeric())
		return false;
	x = json.asDouble();
	return true;
}
bool parse(std::string& s, const Json::Value& json)
{
	if (!json.isString())
		return false;
	s = json.asString();
	return true;
}


bool loadShell(const std::string& path, Eigen::MatrixXd& restPos, Eigen::MatrixXi& restF, Eigen::MatrixXd& curPos, Eigen::MatrixXi& curF, Eigen::MatrixXd& curVel, Eigen::MatrixXd &extForce, std::map<int, Eigen::Vector3d>& clampedDOFs, MaterialParams &matParams, NewtonSolverParams &solverParams)
{
	Json::Value json;
	Json::Reader reader;

	std::string filePath = path;
	std::replace(filePath.begin(), filePath.end(), '\\', '/'); // handle the backslash issue for windows

	int index = filePath.rfind("/");
	std::string filePathPrefix = filePath.substr(0, index + 1);

	std::ifstream file(path);
	bool parsingSuccessful = reader.parse(file, json);
	if (!parsingSuccessful) {
		std::cout << "Error reading file: " << path << std::endl;
		return false;
	}
	file.close();
	if (!parse(matParams.poisson, json["poisson_ratio"]))
	{
		std::cout << "missing poisson ratio." << std::endl;
		return false;
	}
	if (!parse(matParams.thickness, json["thickness"]))
	{
		std::cout << "missing thickness." << std::endl;
		return false;
	}
	if (!parse(matParams.youngs, json["youngs_modulus"]))
	{
		std::cout << "missing youngs modulus." << std::endl;
		return false;
	}
	if (!parse(matParams.density, json["density"]))
	{
		std::cout << "missing density." << std::endl;
		return false;
	}
	if (!parse(matParams.sffid, json["sff_type"]))
	{
		std::cout << "missing sff type, set default to MidedgeAverageFormulation" << std::endl;
		matParams.sffid = 2;
	}
	if (!parse(matParams.matid, json["model_type"]))
	{
		std::cout << "missing model type, set default to stvk." << std::endl;
		matParams.matid = 1;
	}
	if (json["gravity"].isNull())
	{
		matParams.gravity.setZero();
	}
	else
	{
		for (int i = 0; i < 3; i++)
			parse(matParams.gravity(i), json["gravity"][i]);
	}

	if (!parse(solverParams.iterations, json["newton_iter"]))
	{
		std::cout << "missing newton iteration setup, set default to 1000." << std::endl;
		solverParams.iterations = 1000;
	}
	if (!parse(solverParams.gradTol, json["grad_tol"]))
	{
		std::cout << "missing tolerance for gradient, set default to 1e-6." << std::endl;
		solverParams.gradTol = 1e-6;;
	}
	if (!parse(solverParams.fDelta, json["energy_tol"]))
	{
		std::cout << "missing tolerance for energy change, set default to 0." << std::endl;
		solverParams.fDelta = 1e-6;;
	}
	if (!parse(solverParams.xDelta, json["variable_tol"]))
	{
		std::cout << "missing tolerance for variable change, set default to 0." << std::endl;
		solverParams.xDelta = 1e-6;;
	}

	// load meshes
	std::string meshPath;
	if (!parse(meshPath, json["rest_mesh"]))
	{
		std::cout << "missing the rest mesh path." << std::endl;
		return false;
	}
	if (!igl::readOBJ(filePathPrefix + meshPath, restPos, restF))
		return false;

	Eigen::MatrixXi F;
	if (!parse(meshPath, json["cur_mesh"]))
	{
		std::cout << "missing the current mesh path, set rest state as the current state" << std::endl;
		curPos = restPos;
		curF = restF;
	}
	if (!igl::readOBJ(filePathPrefix + meshPath, curPos, curF))
	{
		std::cout << "failed to read the current mesh path, set rest state as the current state" << std::endl;
		curPos = restPos;
		curF = restF;
	}
	
	int nverts = curPos.rows();

	//external forces
	extForce.resize(nverts, 3);
	std::string extFPath;
	if (!parse(extFPath, json["external_force"]))
	{
		std::cout << "missing external force path." << std::endl;
		extForce.setZero();
	}

	std::ifstream ifs(filePathPrefix + extFPath);
	if (!ifs)
	{
		std::cout << "Missing " << extFPath << std::endl;
		extForce.setZero();
	}
	else
	{
		for (int i = 0; i < nverts; i++)
		{
			std::string line;
			std::getline(ifs, line);
			std::stringstream ss(line);

			std::string fx; // f_x, f_y, f_z
			ss >> fx;
			if (!ss)
			{
				std::cout << "-error in the force file: " << std::endl;
				return false;
			}
			else
			{
				for (int j = 0; j < 3; j++)
				{
					extForce(i, j) = std::stod(fx);
					ss >> fx;
				}
			}
		}
	}

	// current clamped dofs
	std::string clampedName;
	if (!parse(clampedName, json["clamped_DOFs"]))
	{
		std::cout << "Missing " << clampedName << std::endl;
		return false;
	}

	std::ifstream cfs(filePathPrefix + clampedName);
	if (!cfs)
	{
		std::cout << "Missing " << clampedName << std::endl;
	}
	else
	{
		int nclamped = 0;
		cfs >> nclamped;
		for (int i = 0; i < nclamped; i++)
		{
			int vid;
			cfs >> vid;
			
			Eigen::Vector3d pos = restPos.row(vid).transpose();
			cfs >> pos(0) >> pos(1) >> pos(2);
			clampedDOFs[vid] = pos;
		}

		std::cout << "number of clamped dofs: " << clampedDOFs.size() << std::endl;
		for (auto& it : clampedDOFs)
		{
			std::cout << "vid: " << it.first << ", " << it.second.transpose() << std::endl;
		}

	}


	//currrent velocity
	curVel.resize(nverts, 3);
	std::string velPath;
	if (!parse(velPath, json["current_velocity"]))
	{
		std::cout << "missing current velocity path." << std::endl;
		curVel.setZero();
	}

	std::ifstream vfs(filePathPrefix + velPath);
	if (!vfs)
	{
		std::cout << "Missing " << velPath << std::endl;
		curVel.setZero();
	}
	else
	{
		for (int i = 0; i < nverts; i++)
		{
			std::string line;
			std::getline(vfs, line);
			std::stringstream ss(line);

			std::string vx; // vel_x, vel_y, vel_z
			ss >> vx;
			if (!ss)
			{
				std::cout << "-error in the velocity file: " << std::endl;
				return false;
			}
			else
			{
				for (int j = 0; j < 3; j++)
				{
					curVel(i, j) = std::stod(vx);
					ss >> vx;
				}
			}
		}
	}

	return true;

}