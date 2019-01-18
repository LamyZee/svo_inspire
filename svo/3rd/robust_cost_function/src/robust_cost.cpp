// Copyright (C) LamyZee
// part of svo_inspire
// https://github.com/LamyZee/svo_inspire.git

#include "robust_cost_function/robust_cost.h"

namespace robust_cost {

DynamicCovScaling::DynamicCovScaling():
  phi_(kDefaultPhi),
  doub_phi_(2.f*kDefaultPhi)
{}

void DynamicCovScaling::Init(const double& params) {
  phi_ = params;
  doub_phi_ = 2.f * params;
}

// error should be L2 norm, or real error^2
double DynamicCovScaling::CalcWeight(const double& errorsqr) {
  const double temp_scaling = doub_phi_ / (errorsqr + phi_);
  return 1.f <  temp_scaling ? 1.f : temp_scaling;
}

} // namespace robust_cost