#include "Projection.h"

Projection::Projection(const std::vector<bool>& keepDOFs)
{
	int fulldofs = keepDOFs.size();
	dofmap.resize(fulldofs);
	int idx = 0;
	for (int i = 0; i < fulldofs; i++)
	{
		if (keepDOFs[i])
		{
			dofmap[i] = idx;
			invdofmap.push_back(i);
			idx++;
		}
		else
			dofmap[i] = -1;
	}
}

void Projection::projectVector(const Eigen::VectorXd& fullVec, Eigen::VectorXd& projVec) const
{
	int projdofs = invdofmap.size();
	projVec.resize(projdofs);

	for (int i = 0; i < projdofs; i++)
	{
		projVec[i] = fullVec[invdofmap[i]];
	}
}

void Projection::unprojectVector(const Eigen::VectorXd& projVec, Eigen::VectorXd& fullVec) const
{
	int fulldofs = dofmap.size();
	fullVec.resize(fulldofs);
	for (int i = 0; i < fulldofs; i++)
	{
		fullVec[i] = (dofmap[i] == -1 ? 0.0 : projVec[dofmap[i]]);
	}
}

void Projection::projectMatrix(std::vector<Eigen::Triplet<double> >& mat) const
{
	int dim = mat.size();
	for (int i = 0; i < dim; i++)
	{
		int r = mat[i].row();
		int c = mat[i].col();
		int pr = dofmap[r];
		int pc = dofmap[c];
		if (pr != -1 && pc != -1)
		{
			mat[i] = { pr,pc,mat[i].value() };
		}
		else
		{
			mat[i] = { 0,0,0.0 };
		}
	}
}