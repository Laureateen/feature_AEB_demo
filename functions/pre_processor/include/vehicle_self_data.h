#pragma once

#include "common_types.h"
#include "common_tools.h"
#include <iostream>


#define ALgtFiltParam1InAsyEvlrCritEve 0.097624f
#define ALgtFiltParam2InAsyEvlrCritEve 0.19525f
#define ALgtFiltParam3InAsyEvlrCritEve 0.097624f
#define ALgtFiltParam4InAsyEvlrCritEve 0.94284f
#define ALgtFiltParam5InAsyEvlrCritEve -0.33333f
#define PinionAgRateFilConInAsyEvlrCritEve 0.200f
#define SpdLimLoForChkPreBrkActvInAsyEvlrCritEve 5.56f



typedef enum
{
	off,
	on
}OnOff1Vcc;

typedef enum
{
	Ukwn,
	Resd1,
	ThreatMed,
	ThreatHi
}ReqSts1Vcc;

typedef struct
{
	f32_t Velocity_longitudinal_mps;
	f32_t Velocity_longitudinal_compact_factor;
	f32_t Accaceration_longitudinal_mpss;
	f32_t Accaceration_longitudinal_raw_mpss;
	f32_t Accaceration_lateral_raw_mpss;
	f32_t Yawrate_radps;
	f32_t Yawrate_raw_radps;
	f32_t Posn_longitudinal_delta_rad;
	f32_t Posn_lateral_delta_rad;//?
	f32_t AgDir_delta_rad;//?

}VehSelf_t;


typedef struct
{
	f32_t Accaceration_longitudinal_filtered_lateonecycle_mpss;
	f32_t Accaceration_longitudinal_filtered_latetwocycle_mpss;
	f32_t Accaceration_longitudinal_lateonecycle_mpss;
	f32_t Accaceration_longitudinal_latetwocycle_mpss;
	f32_t Filtered_PinionAngleRate_lateonecycle_radps;

}VehSelf_old_t;

typedef struct
{
	f32_t CmnCalPrmForMinThdForALatAvl;
	f32_t DynCalPrmForAxleDstReToVehFrnt;
	f32_t DynCalPrmForBicycleMdlAxleDistFrnt;
	f32_t DynCalPrmForBicycleMdlCornrgStfnFrnt;
	f32_t DynCalPrmForBicycleMdlCornrgStfnRe;
	f32_t DynCalPrmForVehLen;
	f32_t DynCalPrmForVehM;
	f32_t DynCalPrmForVehSteerWhlAgRat;
	f32_t DynCalPrmForVehWghtDistbn;
	f32_t DynCalPrmForVehWhlBas;
	f32_t DynCalPrmForVehWidth;
	f32_t HalfVehWidth;
	f32_t DynCalPrmForBicycleMdlCornrgStfnFrntByVehSpd;
	f32_t DynCalPrmForBicycleMdlCornrgStfnReByVehSpd;
	f32_t DynCalPrmForFacToScaForVehALat;
	f32_t DynCalPrmForVehicleSpdForBicycleMdlCornrgStfn;

}DynCalPrm_t;

typedef struct
{
	f32_t DecelReq;
	f32_t DamprReq;
	f32_t DecelEna;
	f32_t SteerAsEna;
	f32_t CrsCnclReq;
	f32_t StandStillReq;
	f32_t BltLvl;
	f32_t CllsnThreat;
	f32_t SteerTqSgnReq;
	f32_t SteerGainEna;

}ActtnDataFromCMbB_t;

