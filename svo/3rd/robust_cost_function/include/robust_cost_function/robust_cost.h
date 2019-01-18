// Copyright (C) LamyZee
// part of svo_inspire
// https://github.com/LamyZee/svo_inspire.git

#ifndef ROBUST_COST_H_
#define ROBUST_COST_H_

#include <memory>
#include <stdlib.h>
#include <math.h>

namespace robust_cost {

class RobustCost {
 public:
  virtual ~RobustCost() {};
  virtual void Init(const double& params) = 0;
  virtual double CalcWeight(const double& errorsqr) = 0;
};

typedef std::shared_ptr<RobustCost> RobustCostPtr;

// Suggest form "Switchable constraints for robust pose graph slam"
static const double kDefaultPhi = 1.f;

// Dynamic Covariance Scaling
// Robus Map Optimization using Dynamic Covariance Scaling
// Authors: Pratik Agarwal, Gian Diego Tipaldi, etc.
// At All Costs: A Comparison of Robust Cost Functions
// for Camera Correspondence Outliers
// Authors: Kirk MacTavish and Timothy D. Barfoot
// Switchable constraints for robust pose graph slam
class DynamicCovScaling : public RobustCost {
 public:
  DynamicCovScaling();

  void Init(const double& params);

  double CalcWeight(const double& errorsqr);

 private:
  double phi_;
  double doub_phi_;
};

typedef std::shared_ptr<DynamicCovScaling> DynamicCovScalingPtr;

} // robust_cost

#endif