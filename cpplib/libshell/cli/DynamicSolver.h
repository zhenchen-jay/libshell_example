#pragma once

#include <Eigen/Core>
#include <vector>
#include "Projection.h"
#include "../include/MaterialModel.h"
#include "../include/MeshConnectivity.h"
#include "../include/ElasticShell.h"


double innertialEnergy(const std::vector<double>& massVec, const Eigen::MatrixXd& curPos, const Eigen::MatrixXd& prevPos, const Eigen::MatrixXd& vel, double dt, Eigen::VectorXd* deriv, std::vector<Eigen::Triplet<double>>* hess)
{
	assert(massVec.size() == curPos.rows());
	double energy = 0;
	int nverts = massVec.size();

	if (deriv)
	{
		deriv->resize(3 * nverts);
		deriv->setZero();
	}

	if (hess)
		hess->clear();

	for (int i = 0; i < nverts; i++)
	{
		Eigen::VectorXd diff = curPos.row(i) - prevPos.row(i) - dt * vel.row(i);
		energy += 0.5 * massVec[i] * diff.dot(diff);

		if (deriv)
		{
			deriv->segment<3>(3 * i) = massVec[i] * diff;
		}

		if (hess)
		{
			for (int j = 0; j < 3; j++)
				hess->push_back({ 3 * i + j, 3 * i + j, massVec[i] });
		}
	}

	return energy;
}

bool isVertexChosen(int vid, std::vector<ChosenVertex> chosenList)
{
	bool in = false;
	for (auto& v : chosenList)
	{
		if (v.vid == vid)
			in = true;
	}
	return in;
}

template <class SFF>
double dynamicEnergy(const MeshConnectivity& mesh,
	const Eigen::MatrixXd& curVel,
	const Eigen::MatrixXd& prePos,
	const Eigen::MatrixXd& curPos,
	const Eigen::MatrixXd& extForce,
	const Eigen::VectorXd& curEdgeDOFs,
	const MaterialModel<SFF>& mat,
	const Eigen::VectorXd& thicknesses,
	const std::vector<Eigen::Matrix2d>& abars,
	const std::vector<Eigen::Matrix2d>& bbars,
	const std::vector<double> massVec,
	double stepSize,
	MaterialParams matParams,
	Eigen::VectorXd *deriv,
	std::vector<Eigen::Triplet<double>> *hess)
{
	int nverts = (int)curPos.rows();
	int nedges = mesh.nEdges();
	int nedgedofs = SFF::numExtraDOFs;
	int nDOFs = 3 * nverts + nedgedofs * nedges;

	Eigen::VectorXd derivative, innertialDeriv;
	std::vector<Eigen::Triplet<double> > hessian, innertialHess, stepHessian;

	// compute value, gradient and hessian
	double energy = 0;
	energy = ElasticShell<SFF>::elasticEnergy(mesh, curPos, curEdgeDOFs, mat, thicknesses, abars, bbars, (deriv || hess) ? &derivative : NULL, hess ? &hessian : NULL);

	double gravityPotential = 0;
	for (int i = 0; i < nverts; i++)
	{
		Eigen::Vector3d pos = curPos.row(i).transpose();
		Eigen::Vector3d mg = massVec[i] * matParams.gravity;
		gravityPotential += -mg.dot(pos);

		if(deriv)
			derivative.segment<3>(3 * i) += -mg;
	}
	energy += gravityPotential;


	// external force
	double extPotential = 0;
	for (int i = 0; i < nverts; i++)
	{
		Eigen::Vector3d pos = curPos.row(i).transpose();
		extPotential += -pos.dot(extForce.row(i));

		if (deriv)
			derivative.segment<3>(3 * i) += -extForce.row(i).transpose();
	}
	energy += extPotential;

	energy = stepSize * stepSize * energy;

	energy += innertialEnergy(massVec, curPos, prePos, curVel, stepSize, (deriv || hess) ? &innertialDeriv : NULL, hess ? &innertialHess : NULL);
	//std::cout << "energy: " << energy << std::endl;
	if (deriv)
	{
		deriv->resize(nDOFs);
		deriv->setZero();
		(*deriv) = stepSize * stepSize * derivative;
		deriv->segment(0, 3 * nverts) += innertialDeriv;
		//std::cout << "||g||: " << deriv->norm() << std::endl;
	}

	if (hess)
	{
		hess->clear();

		for (auto& it : hessian)
		{
			int rid = it.row();
			int cid = it.col();
			hess->push_back(Eigen::Triplet<double>(rid, cid, stepSize * stepSize * it.value()));
		}

		for (auto& it : innertialHess)
		{
			hess->push_back(it);
		}
	}
	return energy;
}

template <class SFF>
void testDynamicEnergy(const MeshConnectivity& mesh,
	const Eigen::MatrixXd& curVel,
	const Eigen::MatrixXd& prePos,
	const Eigen::MatrixXd& curPos,
	const Eigen::VectorXd& curEdgeDOFs,
	const MaterialModel<SFF>& mat,
	const Eigen::VectorXd& thicknesses,
	const std::vector<Eigen::Matrix2d>& abars,
	const std::vector<Eigen::Matrix2d>& bbars,
	const std::vector<double> massVec,
	double stepSize,
	MaterialParams matParams)
{
	int nverts = (int)curPos.rows();
	int nedges = mesh.nEdges();
	int nedgedofs = SFF::numExtraDOFs;

	Eigen::VectorXd deriv;
	Eigen::SparseMatrix<double> hess;
	std::vector<Eigen::Triplet<double>> hessT;

	Eigen::MatrixXd extF = curPos;
	extF.setRandom();

	double E0 = dynamicEnergy<SFF>(mesh, curVel, prePos, curPos, extF, curEdgeDOFs, mat, thicknesses, abars, bbars, massVec, stepSize, matParams, &deriv, &hessT);

	int nDOFs = 3 * nverts + nedgedofs * nedges;
	hess.resize(nDOFs, nDOFs);
	hess.setFromTriplets(hessT.begin(), hessT.end());

	//std::cout << hess.toDense() << std::endl;

	Eigen::VectorXd dir = deriv;
	dir.setRandom();

	// test value and gradient
	std::cout << "value gradient test" << std::endl;
	for (int i = 3; i < 10; i++)
	{
		double eps = std::pow(0.1, i);
		Eigen::MatrixXd newPos = curPos;
		Eigen::VectorXd newEdgeDOFs = curEdgeDOFs;

		for (int j = 0; j < nverts; j++)
		{
			newPos.row(j) += eps * dir.segment<3>(3 * j).transpose();
		}

		newEdgeDOFs += eps * dir.segment(3 * nverts, nedgedofs * nedges);

		double E1 = dynamicEnergy<SFF>(mesh, curVel, prePos, newPos, extF, newEdgeDOFs, mat, thicknesses, abars, bbars, massVec, stepSize, matParams, NULL, NULL);
		std::cout << "eps: " << eps << ", E0: " << E0 << ", E1: " << E1 << ", finite difference: " << (E1 - E0) / eps << ", directional derivative: " << dir.dot(deriv) << ", error: " << std::abs((E1 - E0) / eps - dir.dot(deriv)) << std::endl;
	}

	std::cout << "gradient hessian test" << std::endl;
	for (int i = 3; i < 10; i++)
	{
		double eps = std::pow(0.1, i);
		Eigen::MatrixXd newPos = curPos;
		Eigen::VectorXd newEdgeDOFs = curEdgeDOFs;

		for (int j = 0; j < nverts; j++)
		{
			newPos.row(j) += eps * dir.segment<3>(3 * j).transpose();
		}

		newEdgeDOFs += eps * dir.segment(3 * nverts, nedgedofs * nedges);
		Eigen::VectorXd deriv1;
		double E1 = dynamicEnergy<SFF>(mesh, curVel, prePos, newPos, extF, newEdgeDOFs, mat, thicknesses, abars, bbars, massVec, stepSize, matParams, &deriv1, NULL);
		std::cout << "eps: " << eps << ", ||g0||: " << deriv.norm() << ", ||g1||: " << deriv1.norm() << ", finite difference: " << (deriv1 - deriv).norm() / eps << ", directional derivative: " << (hess * dir).norm() << ", error: " << ((deriv1 - deriv)/ eps - hess * dir).norm() << std::endl;
	}
}

template <class SFF>
int takeOneStep(const MeshConnectivity& mesh,
	const Eigen::MatrixXd& curVel,
	Eigen::MatrixXd& curPos,
	const Eigen::MatrixXd& extForce,
	const std::map<int, Eigen::Vector3d>& clampedDOFs,
	Eigen::VectorXd& curEdgeDOFs,
	const MaterialModel<SFF>& mat,
	const Eigen::VectorXd& thicknesses,
	const std::vector<Eigen::Matrix2d>& abars,
	const std::vector<Eigen::Matrix2d>& bbars,
	const std::vector<double> massVec,
	double stepSize,
	MaterialParams matParams,
	NewtonSolverParams newtonParams)

	/*
	* return value:
	* -1: line search failed
	* 0: converged, gradient norm is small
	* 1: variable update is small
	* 2: energy update is small
	* 3: reach the maximum iteration.
	*/

{

	int nverts = (int)curPos.rows();
	int nedges = mesh.nEdges();
	int nedgedofs = SFF::numExtraDOFs;

	Projection proj;

	std::vector<bool> keepDOFs(3 * nverts + nedgedofs * nedges);

	int row = 0;


	for (int i = 0; i < nverts; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			keepDOFs[row] = (clampedDOFs.find(i) == clampedDOFs.end());
			row++;
		}
	}

	for (int i = 0; i < nedges; i++)
	{
		for (int j = 0; j < nedgedofs; j++)
		{
			keepDOFs[row] = true;
			row++;
		}
	}
	proj = Projection(keepDOFs);

	int nDOFs = 3 * nverts + nedgedofs * nedges;
	double reg = 1e-6;

	Eigen::MatrixXd prePos = curPos;
	int iter = 0;

	for (auto& it : clampedDOFs)
	{
		int vid = it.first;
		curPos.row(vid) = it.second.transpose();
	}

	while (true)
	{
		std::cout << "iter: " << iter << std::endl;
		Eigen::VectorXd derivative;
		std::vector<Eigen::Triplet<double> > hessian;

		// compute value, gradient and hessian
		double energy = dynamicEnergy<SFF>(mesh, curVel, prePos, curPos, extForce, curEdgeDOFs, mat, thicknesses, abars, bbars, massVec, stepSize, matParams, &derivative, &hessian);
		Eigen::VectorXd force = -derivative;

		// remove the chosen vertices 
		Eigen::VectorXd reducedF;
		proj.projectVector(force, reducedF);

		Eigen::SparseMatrix<double> reducedH, tmpH;
		proj.projectMatrix(hessian);
		reducedH.resize(proj.projDOFs(), proj.projDOFs());
		reducedH.setFromTriplets(hessian.begin(), hessian.end());

		Eigen::SparseMatrix<double> I = reducedH;
		I.setIdentity();

		tmpH = reducedH;
		std::cout << "make sure hessian is PSD." << std::endl;
		// make sure the hessian is PSD
		Eigen::SimplicialLLT<Eigen::SparseMatrix<double> > solver(tmpH);
		while (solver.info() != Eigen::Success)
		{
			tmpH = reducedH + reg * I;
			reg *= 2;
			std::cout << "matrix is not PSD, current reg = " << reg << std::endl;
			solver.compute(tmpH);
		}

		Eigen::VectorXd reducedDir = solver.solve(reducedF);
		Eigen::VectorXd fullDir;
		proj.unprojectVector(reducedDir, fullDir);

		// do the line search, only apply the armijo condition
		double f0 = energy;
		double rate = 1.0;
		Eigen::MatrixXd newPos = curPos;
		Eigen::VectorXd newEdgeDOFs = curEdgeDOFs;

		//std::cout << "reduced dir: " << std::endl << reducedDir << std::endl << std::endl;
		for (int i = 0; i < nverts; i++)
		{
			newPos.row(i) = curPos.row(i) + rate * fullDir.segment<3>(3 * i).transpose();
		}
		for (auto& it : clampedDOFs)
		{
			int vid = it.first;
			newPos.row(vid) = it.second.transpose();
		}

		newEdgeDOFs = curEdgeDOFs + rate * fullDir.segment(3 * nverts, nedgedofs * nedges);
		double f1 = dynamicEnergy<SFF>(mesh, curVel, prePos, newPos, extForce, curEdgeDOFs, mat, thicknesses, abars, bbars, massVec, stepSize, matParams, NULL, NULL);

		double cache = -1e-4 * reducedF.dot(reducedDir);

		while (f1 > f0 + rate * cache)
		{
			rate *= 0.5;
			for (int i = 0; i < nverts; i++)
			{
				newPos.row(i) = curPos.row(i) + rate * fullDir.segment<3>(3 * i).transpose();
			}
			for (auto& it : clampedDOFs)
			{
				int vid = it.first;
				newPos.row(vid) = it.second.transpose();
			}
			newEdgeDOFs = curEdgeDOFs + rate * fullDir.segment(3 * nverts, nedgedofs * nedges);

			f1 = dynamicEnergy<SFF>(mesh, curVel, prePos, newPos, extForce, curEdgeDOFs, mat, thicknesses, abars, bbars, massVec, stepSize, matParams, NULL, NULL);

			if (rate < 1e-10)
			{
				std::cout << "failed to find a descent step. " << std::endl;
				std::cout << "f0: " << f0 << ", f1: " << f1 << ", cache: " << cache << std::endl;
				return -1;
			}
		}
		curPos = newPos;
		curEdgeDOFs = newEdgeDOFs;

		iter++;
		double deltaX = rate * fullDir.norm();
		double deltaF = f0 - f1;

		Eigen::VectorXd deriv1, reducedDeriv;
		energy = dynamicEnergy<SFF>(mesh, curVel, prePos, curPos, extForce, curEdgeDOFs, mat, thicknesses, abars, bbars, massVec, stepSize, matParams, &deriv1, NULL);
		proj.projectVector(deriv1, reducedDeriv);

		double gnorm = reducedDeriv.norm();
		std::cout << "current reg for hessian: " << reg << ", line search rate: " << rate << std::endl;
		std::cout << "previous energy: " << f0 << ", new energy: " << f1 << ", descent in energy: " << deltaF << ", current gradient norm: " << gnorm << ", variable update: " << deltaX << std::endl;

		reg *= 0.5;

		// convergence check
		if (gnorm < newtonParams.gradTol)
		{
			std::cout << "converged with gradient norm: " << gnorm << std::endl;
			return 0;
		}

		if (deltaX < newtonParams.xDelta)
		{
			std::cout << "terminated with update in varibles smaller than " << newtonParams.xDelta << std::endl;
			return 1;
		}

		if (deltaF < newtonParams.fDelta)
		{
			std::cout << "terminated with update in energy smaller than " << newtonParams.fDelta << std::endl;
			return 2;
		}

		if (iter > newtonParams.iterations)
		{
			std::cout << "reach the maximum iteration: " << newtonParams.iterations << std::endl;
			return 3;
		}
	}
	return 0;
}