#include "vehicle_preprocessor.hpp"

namespace ns_preprocessor {

  preprocessor::preprocessor(){
    Init_vehicle_preprocessor();

  };

  void preprocessor::run_vehicle_preprocessor(VehSelf_t* vsd)
  {
    VehSelfSpdCompensation(vsd);
    CalculateCurvature(vsd,1,1,);
    LongitudinalAccelerationWithoutCMbB();
    CheckCMbBActive();
  };

  //修正了纵向加速度和速度，记录需要的数据
  void preprocessor::VehSelfSpdCompensation(VehSelf_t* vsd )
  {
      f32_t ALgtFiltered;
      f32_t VLgtCmp;
      //
      VLgtCmp = vsd->Velocity_longitudinal_mps * vsd->Velocity_longitudinal_compact_factor;
      ALgtFiltered  = vsd->Accaceration_longitudinal_mpss* ALgtFiltParam1InAsyEvlrCritEve
                    + oldcycle_data.Accaceration_longitudinal_lateonecycle_mpss * ALgtFiltParam2InAsyEvlrCritEve
                    + oldcycle_data.Accaceration_longitudinal_latetwocycle_mpss * ALgtFiltParam3InAsyEvlrCritEve 
                    + oldcycle_data.Accaceration_longitudinal_filtered_lateonecycle_mpss * ALgtFiltParam4InAsyEvlrCritEve
                    + oldcycle_data.Accaceration_longitudinal_filtered_latetwocycle_mpss * ALgtFiltParam4InAsyEvlrCritEve;
      //存储前一二周期的数据
      oldcycle_data.Accaceration_longitudinal_filtered_lateonecycle_mpss = oldcycle_data.Accaceration_longitudinal_filtered_latetwocycle_mpss;
      oldcycle_data.Accaceration_longitudinal_filtered_latetwocycle_mpss = ALgtFiltered;
      oldcycle_data.Accaceration_longitudinal_latetwocycle_mpss = oldcycle_data.Accaceration_longitudinal_lateonecycle_mpss;
      oldcycle_data.Accaceration_longitudinal_lateonecycle_mpss = vsd->Accaceration_longitudinal_mpss;
      //这样写方便整理思想，后期再优化掉
      vsd->Velocity_longitudinal_mps = VLgtCmp;
      vsd->Accaceration_longitudinal_mpss = ALgtFiltered;
  };

  void preprocessor::CalculateCurvature(VehSelf_t* vsd,f32_t* PinionSteerAg, f32_t* PinionSteerAgSpd, DynCalPrm_t* DynCalPrm)
  {
      //lookup table

     
  };
  f32_t preprocessor::CalculateByStaticBicycleModelC1(VehSelf_t* vsd, f32_t* PinionSteerAg, f32_t* PinionSteerAgSpd, DynCalPrm_t* DynCalPrm) 
  {
      f32_t WheelAngle;
      f32_t DistCoGToRearAxle;
      f32_t DistCoGToFrontAxle;
      f32_t MassSpeedSquaredDivWheelBase;
      f32_t CurvatureCoGTem;
      f32_t CurvatureCoG;
      f32_t RadiusCoG;
      f32_t SlipAngle;
      f32_t RadiusRear;
      f32_t CurvatureRear;
      //CalculateWheelAngle
      if (DynCalPrm->DynCalPrmForVehSteerWhlAgRat != 0) { WheelAngle = *PinionSteerAg / DynCalPrm->DynCalPrmForVehSteerWhlAgRat; }
      else { WheelAngle = F32_MAX; };

      //CalculateDistanceToAxles
      DistCoGToRearAxle = DynCalPrm->DynCalPrmForVehWhlBas - DynCalPrm->DynCalPrmForBicycleMdlAxleDistFrnt;
      DistCoGToFrontAxle = DynCalPrm->DynCalPrmForBicycleMdlAxleDistFrnt;

      //IntermediateCalculation
      if (DynCalPrm->DynCalPrmForVehWhlBas != 0)
      {
          MassSpeedSquaredDivWheelBase = (vsd->Velocity_longitudinal_mps * vsd->Velocity_longitudinal_mps * DynCalPrm->DynCalPrmForVehM) / DynCalPrm->DynCalPrmForVehWhlBas;
      }
      else { MassSpeedSquaredDivWheelBase = F32_MAX; }

      //CalculateCurvatureCoG
      CurvatureCoGTem = MassSpeedSquaredDivWheelBase * (SafeDivideFloat(DistCoGToRearAxle, DynCalPrm->DynCalPrmForBicycleMdlCornrgStfnFrnt)
          - SafeDivideFloat(DistCoGToFrontAxle, DynCalPrm->DynCalPrmForBicycleMdlCornrgStfnRe)) + DynCalPrm->DynCalPrmForVehWhlBas;
      CurvatureCoG = SafeDivideFloat(WheelAngle, CurvatureCoGTem);
      RadiusCoG = SafeDivideFloat(1.0f, CurvatureCoG);

      //CalculateSlip
      SlipAngle = CurvatureCoG * (DistCoGToRearAxle - MassSpeedSquaredDivWheelBase * SafeDivideFloat(DistCoGToFrontAxle, DynCalPrm->DynCalPrmForBicycleMdlCornrgStfnRe));

      //CalculateCurvatureRear
      RadiusRear = SqrtFloat((DistCoGToRearAxle - sinf(SlipAngle) * RadiusCoG) * (DistCoGToRearAxle - sinf(SlipAngle) * RadiusCoG) + (cosf(SlipAngle) * RadiusCoG) * (cosf(SlipAngle) * RadiusCoG))
          * SignumFloat(RadiusCoG);

      CurvatureRear = SafeDivideFloat(1, RadiusRear);
      return CurvatureRear;
  };


  f32_t preprocessor::CalculateByStaticBicycleModelC0(VehSelf_t* vsd, f32_t* PinionSteerAg, f32_t* PinionSteerAgSpd, DynCalPrm_t* DynCalPrm)
  {

  };
  f32_t preprocessor::LookupTable1D(f32_t* InputValue)
  {
      

  };

  void preprocessor::run_preprocessor() {};

  void preprocessor::LongitudinalAccelerationWithoutCMbB(){};

  void preprocessor::CheckCMbBActive(){};

  //void preprocessor::vehicle_preprocessor_output(){};

  void preprocessor::Init_vehicle_preprocessor(){};
}
