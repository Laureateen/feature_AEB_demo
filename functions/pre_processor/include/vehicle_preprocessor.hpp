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

  void run_Function();
  //subfunction level 0-1
  void run_vehicle_preprocessor(VehSelf_t* vsd, f32_t* PinionSteerAg, f32_t* PinionSteerAgSpd, DynCalPrm_t* DynCalPrm,
      f32_t DrvrDecelReq, ActtnDataFromCMbB_t* ActtnDataFromCllsnRednByBrkgCtrl);
  //subfunction level 0-2
  void run_ModifyObjBusAndDetectDrivingSide();
  //subfunction level 0-3
  void run_ExtractAccTargetIdIfAccActive();
  //subfunction level 0-4
  void run_FindIndicesOfValidFrontObjects();
  //subfunction level 0-5
  void run_ObjectPreProcessor();

private:

  // static vriables zone
  VehSelf_old_t oldcycle_data;

  /**************** vehicle_preprocessor subfucntion******************/
  ////VehSelfSpdCompensation
  void VehSelfSpdCompensation(VehSelf_t* vsd);

  ////CalculateCurvature
  void CalculateCurvature(VehSelf_t* vsd, f32_t* PinionSteerAg, f32_t* PinionSteerAgSpd, DynCalPrm_t* DynCalPrm);
  f32_t CalculateByStaticBicycleModelC0(VehSelf_t* vsd, f32_t* PinionSteerAgorSpd, DynCalPrm_t* DynCalPrm);
  f32_t CalculateByStaticBicycleModelC1(VehSelf_t* vsd, f32_t* PinionSteerAgorSpd, DynCalPrm_t* DynCalPrm);
  f32_t LookupTable1D(f32_t* InputValue, f32_t fx0, f32_t fy0, f32_t fx1, f32_t fy1);
  f32_t LowPassFilter(f32_t PinionAngleRate_in);
  ////LongitudinalAccelerationWithoutCMbB
  f32_t LongitudinalAccelerationWithoutCMbB(VehSelf_t* vsd,f32_t DrvrDecelReq, ActtnDataFromCMbB_t* ActtnDataFromCllsnRednByBrkgCtrl);
  ////CheckCMbBActive
  b8_t CheckCMbBActive(VehSelf_t* vsd, f32_t DrvrDecelReq, ActtnDataFromCMbB_t* ActtnDataFromCllsnRednByBrkgCtrl);
  ////others

  /**************** ModifyObjBusAndDetectDrivingSide subfucntion******************/
  void LoopOverObjects();
  /**************** ExtractAccTargetIdIfAccActive subfucntion******************/


  /**************** FindIndicesOfValidFrontObjects subfucntion******************/
  void ObjUtilityAndInPath();
  void AddInPathObjects();
  void SortObjectsByUtility();
  void AddAdjacentObjects();
  /**************** ObjectPreProcessor subfucntion******************/
  void FindObjectsToPreprocess();
  void ObjectPreProcessorLoop();

  void Init_vehicle_preprocessor();
  

};
}

#endif //VEHICLE_PREPROCESSOR_HPP
