//
// copyright Matthias Schoerghuber (AIT)
//
#ifndef SLAMANTIC_DYNAMICSFACTOR_HPP
#define SLAMANTIC_DYNAMICSFACTOR_HPP

#include <slamantic/common.hpp>

namespace slamantic
{
/**
 *
 * @param numObservations number of observations
 * @param labelDynamicFactor [-1 to 1] static to dynamic of label
 * @param labelConsistency probability of being label
 * @param k declanation factor
 * @return dynamic factor
 */
  double computeDynamicsFactor(size_t const& numObservations,
                               double const& labelDynamicFactor = 0.0,
                               double const& labelConsistency = 0.0,
                               double const& k = 20);



  const double DYNAMICS_FACTOR_DYNAMIC = 1.0;
  const double DYNAMICS_FACTOR_STATIC_DYNAMIC = 0.5;
  const double DYNAMICS_FACTOR_STATIC  = 0.25;


  /**
   * return true if dynamicsFactor is considered as dynamic
   * @param dynamicsFactor
   * @return
   */
  bool isDfDynamic(double const& dynamicsFactor);

  /**
 * return true if dynamicsFactor is considered as static
 * @param dynamicsFactor
 * @return
 */
  bool isDfStatic(double const& dynamicsFactor);

  /**
 * return true if dynamicsFactor is considered as potential dynamic
 * @param dynamicsFactor
 * @return
 */
  bool isDfPotentialDynamic(double const& dynamicsFactor);

}

#endif