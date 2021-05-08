#pragma once
#include "common.h"
#include <vector>
#include <Eigen/Core>
#include <Eigen/Sparse>

class Projection
{
public:
	Projection() : isPorjNeeded(true) {}
	Projection(const std::vector<bool>& keepDOFs);

	int projDOFs() const { return invdofmap.size(); }
	void projectVector(const Eigen::VectorXd& fullVec, Eigen::VectorXd& projVec) const;
	void unprojectVector(const Eigen::VectorXd& projVec, Eigen::VectorXd& fullVec) const;
	void projectMatrix(std::vector<Eigen::Triplet<double> >& mat) const;

private:
	std::vector<int> dofmap;
	std::vector<int> invdofmap;
	bool isPorjNeeded;
};