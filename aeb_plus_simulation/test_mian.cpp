#include "vehicle_preprocessor.hpp"
#include<iostream>

using namespace ns_preprocessor;
//typedef ns_preprocessor::preprocessor preprocessor;
VehSelf_old_t oldcycle_data_m;
int main(int argc, char** argv)
{
    std::cout << "-------------------AEB TEST BEGIN-----------------" << std::endl;
    VehSelf_t vsd = { 5, 0.2,1,1,0.1,0.2,0.2,0,0,0 };
    f32_t PinionSteerAg = { 0.2 };
    f32_t PinionSteerAgSpd = { 0.5 };
    DynCalPrm_t DynCalPrm = { 0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
    f32_t DrvrDecelReq = { 1 };
    ActtnDataFromCMbB_t ActtnDataFromCllsnRednByBrkgCtrl = { 1,1,1,1,1,1,1,1,1,1 } ;
    //oldcycle_data_m
    preprocessor a;
    a.run_vehicle_preprocessor(&vsd, &PinionSteerAg, &PinionSteerAgSpd, &DynCalPrm, DrvrDecelReq, &ActtnDataFromCllsnRednByBrkgCtrl);
    return 0;
};