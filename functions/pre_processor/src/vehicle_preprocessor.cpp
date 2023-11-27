#include "vehicle_preprocessor.hpp"

namespace ns_preprocessor {

  preprocessor::preprocessor(){
    Init_vehicle_preprocessor();
  };

  void preprocessor::run_vehicle_preprocessor(VehSelf_t* vsd, f32_t* PinionSteerAg, f32_t* PinionSteerAgSpd, DynCalPrm_t* DynCalPrm,
      f32_t DrvrDecelReq, ActtnDataFromCMbB_t* ActtnDataFromCllsnRednByBrkgCtrl)
  {
    VehSelfSpdCompensation(vsd);
    CalculateCurvature(vsd,PinionSteerAg,PinionSteerAgSpd,DynCalPrm);
    LongitudinalAccelerationWithoutCMbB(vsd, DrvrDecelReq, ActtnDataFromCllsnRednByBrkgCtrl);
    CheckCMbBActive(vsd, DrvrDecelReq,ActtnDataFromCllsnRednByBrkgCtrl);
    //output arguments:
    //VehSelf ,VehSelfData[Curvature, CurvatureRate, CurvatureRadius ,DrvrALgt, CMbBActive]

    std::cout << "Accaceration_lateral_raw_mpss :" << vsd->Accaceration_lateral_raw_mpss << std::endl;
    std::cout << "Accaceration_longitudinal_mpss :" << vsd->Accaceration_longitudinal_mpss << std::endl;
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
      f32_t VLgtWeightFactor;
      f32_t PinionAngleBasedCurvature;
      f32_t PinionAngleBasedCurvatureRate;
      f32_t YawRateBasedCurvature;
      f32_t Curvature;
      f32_t CurvatureRate;
      f32_t CurvatureRadius;
      //YawRateBasedCurvature
      YawRateBasedCurvature = SafeDivideFloat(vsd->Yawrate_radps,vsd->Velocity_longitudinal_mps);
      //lookup table
      VLgtWeightFactor = LookupTable1D(&(vsd->Velocity_longitudinal_mps), 8, 10, 0, 1);
      //PinionAngleBasedCurvature
      PinionAngleBasedCurvature = CalculateByStaticBicycleModelC1(vsd, PinionSteerAg, DynCalPrm);
      //PinionAngleBasedCurvatureRate
      *PinionSteerAgSpd = LowPassFilter(*PinionSteerAgSpd);
      PinionAngleBasedCurvatureRate = CalculateByStaticBicycleModelC1(vsd,  PinionSteerAgSpd, DynCalPrm);

      // Curvature = YawRateBasedCurvature * VLgtWeightFactor + PinionAngleBasedCurvature * (1 - VLgtWeightFactor);
      if (VLgtWeightFactor>=1)
      {
          Curvature = YawRateBasedCurvature;
          CurvatureRate = 0.0f;
      }
      else if (VLgtWeightFactor <= 0)
      {
          Curvature = PinionAngleBasedCurvature;
          CurvatureRate = PinionAngleBasedCurvatureRate;
      }
      else
      {
          Curvature = LimitsFloat((YawRateBasedCurvature * VLgtWeightFactor + PinionAngleBasedCurvature * (1 - VLgtWeightFactor)), -0.1f, 0.1f);
          CurvatureRate = LimitsFloat((0.0f * VLgtWeightFactor + PinionAngleBasedCurvatureRate * (1 - VLgtWeightFactor)), -0.1f, 0.1f);
      }
      CurvatureRadius = SafeDivideFloat(1.0f, Curvature);
      std::cout << "Curvature :" << Curvature << std::endl;
      std::cout << "CurvatureRate :" << CurvatureRate << std::endl;
      std::cout << "CurvatureRadius :" << CurvatureRadius << std::endl;
  };
  f32_t preprocessor::CalculateByStaticBicycleModelC1(VehSelf_t* vsd, f32_t* PinionSteerAgorSpd, DynCalPrm_t* DynCalPrm) 
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
      if (DynCalPrm->DynCalPrmForVehSteerWhlAgRat != 0) { WheelAngle = *PinionSteerAgorSpd / DynCalPrm->DynCalPrmForVehSteerWhlAgRat; }
      else { WheelAngle = F32_MAX; };
      //WheelAngle = SafeDivideFloat(*PinionSteerAg, DynCalPrm->DynCalPrmForVehSteerWhlAgRat);


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


  f32_t preprocessor::CalculateByStaticBicycleModelC0(VehSelf_t* vsd, f32_t* PinionSteerAgorSpd, DynCalPrm_t* DynCalPrm)
  {
      //this function is defined by simulink model equal c1 function
      return 0;
  };
 
  f32_t preprocessor::LookupTable1D(f32_t* InputValueX,f32_t fx0, f32_t fy0, f32_t fx1, f32_t fy1)
  {
      f32_t OutputValueY;
      f32_t slope, intercept;
      if (fx0>fx1)
      {
          f32_t tem;
          tem = fx0;
          fx0 = fx1;
          fx1 = tem;
          tem = fy0;
          fy0 = fy1;
          fy1 = tem;
      };
      if (*InputValueX < fx0) { OutputValueY = fx0; }
      else if (*InputValueX > fx1) { OutputValueY = fx1; }
      else
      {
          slope = (fx1 - fx0) / (fy1 - fy0);
          intercept = fy0 - slope * fx0;
          OutputValueY = slope * (*InputValueX) + intercept;
      };
      return OutputValueY;
  };

  //LowPassFilter
  f32_t preprocessor::LowPassFilter(f32_t PinionAngleRate_in)
  {
      f32_t filtered_output;
      filtered_output = (PinionAngleRate_in * PinionAgRateFilConInAsyEvlrCritEve) + oldcycle_data.Filtered_PinionAngleRate_lateonecycle_radps
          * (1 - PinionAgRateFilConInAsyEvlrCritEve);
      oldcycle_data.Filtered_PinionAngleRate_lateonecycle_radps = filtered_output;
      return filtered_output;
  };


  void preprocessor::run_preprocessor() {};

  f32_t preprocessor::LongitudinalAccelerationWithoutCMbB(VehSelf_t* vsd, f32_t DrvrDecelReq, ActtnDataFromCMbB_t* ActtnDataFromCllsnRednByBrkgCtrl)
  {
      f32_t DrvrALgt;
      if (ActtnDataFromCllsnRednByBrkgCtrl->DecelEna == on)
      {
          DrvrALgt = DrvrDecelReq;
      }
      else
      {
          DrvrALgt = vsd->Accaceration_longitudinal_mpss;
      }
      std::cout << "DrvrALgt :" << DrvrALgt << std::endl;
      return DrvrALgt;
  };

  b8_t preprocessor::CheckCMbBActive(VehSelf_t* vsd, f32_t DrvrDecelReq, ActtnDataFromCMbB_t* ActtnDataFromCllsnRednByBrkgCtrl)
  {
      b8_t flagtem = false;
      b8_t flag = false;
      if (ActtnDataFromCllsnRednByBrkgCtrl->CllsnThreat == ThreatMed && vsd->Velocity_longitudinal_mps > SpdLimLoForChkPreBrkActvInAsyEvlrCritEve) { flagtem = true;}
      if ((flagtem || ActtnDataFromCllsnRednByBrkgCtrl->CllsnThreat == ThreatHi) && ActtnDataFromCllsnRednByBrkgCtrl->DecelEna == on)
      {
          flag = true;
      }
      return flag;
      std::cout << "CMbBActive :" << flag << std::endl;
  };

  //void preprocessor::vehicle_preprocessor_output(){};

  void preprocessor::Init_vehicle_preprocessor()
  {
      oldcycle_data = { 0,0,0,0,0 };
      std::cout << "Init_vehicle_preprocessor" << std::endl;
  };
}
