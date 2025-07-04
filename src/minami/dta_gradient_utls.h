#ifndef DTA_GRADIENT_UTLS_H
#define DTA_GRADIENT_UTLS_H

#include "Snap.h"
#include "dlink.h"
#include "path.h"
#include "factory.h"

#include <unordered_map>
#include <set>

#include <Eigen/Sparse>

struct dar_record { 
  TInt path_ID; 
  TInt assign_int; 
  TInt link_ID;
  TFlt link_start_int;
  TFlt flow;
};

struct ltg_record { 
  TInt path_ID; 
  int assign_int; 
  TInt link_ID;
  int link_start_int;
  TFlt gradient;
};


namespace MNM_DTA_GRADIENT
{
TFlt get_link_inflow(MNM_Dlink* link, 
                    TFlt start_time, TFlt end_time);
TFlt get_link_inflow(MNM_Dlink* link, 
                    TInt start_time, TInt end_time);

TFlt get_last_valid_time(MNM_Cumulative_Curve *N_in, MNM_Cumulative_Curve *N_out, TInt end_loading_timestamp, const std::string& s="");

TFlt get_travel_time_from_cc(TFlt start_time, MNM_Cumulative_Curve *N_in, MNM_Cumulative_Curve *N_out, TFlt last_valid_time, TFlt fftt, bool rounding_up=false);

TFlt get_travel_time_from_FD(MNM_Dlink* link, TFlt start_time, TFlt unit_interval);
// TFlt get_travel_time(MNM_Dlink* link, TFlt start_time, TFlt unit_interval);

TFlt get_travel_time(MNM_Dlink* link, TFlt start_time, TFlt unit_interval, TInt end_loading_timestamp);

TFlt get_path_travel_time(MNM_Path* path, TFlt start_time, MNM_Link_Factory* link_factory, TFlt unit_interval, TInt end_loading_timestamp);

int add_dar_records(std::vector<dar_record*> &record, MNM_Dlink* link, 
                    std::unordered_map<MNM_Path*, int> path_map, TFlt start_time, TFlt end_time);
int add_dar_records_eigen(std::vector<Eigen::Triplet<double>> &record, MNM_Dlink* link, 
                    std::unordered_map<MNM_Path*, int> path_map, TFlt start_time, TFlt end_time,
                    int link_ind, int interval_ind, int num_e_link, int num_e_path,
                    const double *f_ptr);
};
#endif