/*
    Automatic Emergency Braking Project (AEB-Project).
    Copyright (c) 2023:
*/

#ifndef VEHICLE_PREPROCESSOR_HPP
#define VEHICLE_PREPROCESSOR_HPP

#include <iostream>
#include "common_types.h"
#include "vehicle_self_data.h"

namespace ns_preprocessor {

class preprocessor {

 public:

  preprocessor();

  void run_preprocessor();
  void run_vehicle_preprocessor(VehSelf_t* vsd);
  ////void vehicle_preprocessor_output();

private:

  VehSelf_old_t oldcycle_data;

  //vehicle_preprocessor
  ////VehSelfSpdCompensation
  void VehSelfSpdCompensation(VehSelf_t* vsd);
  ////CalculateCurvature
  void CalculateCurvature(VehSelf_t* vsd, f32_t* PinionSteerAg, f32_t* PinionSteerAgSpd, DynCalPrm_t* DynCalPrm);
  f32_t CalculateByStaticBicycleModelC0(VehSelf_t* vsd, f32_t* PinionSteerAg, f32_t* PinionSteerAgSpd, DynCalPrm_t* DynCalPrm);
  f32_t CalculateByStaticBicycleModelC1(VehSelf_t* vsd, f32_t* PinionSteerAg, f32_t* PinionSteerAgSpd, DynCalPrm_t* DynCalPrm);
  f32_t LookupTable1D(f32_t* InputValue);
  ////LongitudinalAccelerationWithoutCMbB
  void LongitudinalAccelerationWithoutCMbB();
  ////CheckCMbBActive
  void CheckCMbBActive();
  ////others
  void Init_vehicle_preprocessor();
  
  //ModifyObjBusAndDetectDrivingSide
  ////

};
}

#endif //VEHICLE_PREPROCESSOR_HPP
