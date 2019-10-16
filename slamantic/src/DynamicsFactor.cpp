//
// copyright Matthias Schoerghuber (AIT)
//
#include <slamantic/DynamicsFactor.hpp>


double slamantic::computeDynamicsFactor(const size_t& numObservations,
                                        const double& labelDynamicFactor,
                                        const double& labelConsistency,
                                        const double& k)
{
  DCHECK(k > 0.0);

  if(numObservations < 1)
  {
    return DYNAMICS_FACTOR_DYNAMIC;
  }

  double       lc       = labelConsistency;
  double       df_label = lc > 0.5 ? labelDynamicFactor * lc : labelDynamicFactor;
  double const d        = 0.5;
  double const s        = 1.0;

  double df_obs = -static_cast<double>((numObservations * (numObservations - 1))) / k + d;

  double dynamicFactor = std::max(std::max(df_obs + d * s * df_label, df_label), 0.0);
  return dynamicFactor;
}
bool slamantic::isDfDynamic(double const& dynamicsFactor)
{
  return dynamicsFactor > DYNAMICS_FACTOR_STATIC_DYNAMIC;
}
bool slamantic::isDfStatic(double const& dynamicsFactor)
{
  return dynamicsFactor <= DYNAMICS_FACTOR_STATIC;
}
bool slamantic::isDfPotentialDynamic(double const& dynamicsFactor)
{
  return !isDfDynamic(dynamicsFactor) && !isDfStatic(dynamicsFactor);
}
