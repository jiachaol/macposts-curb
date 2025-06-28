#ifndef DTA_API_H
#define DTA_API_H

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>

#include "Snap.h"
#include "dta.h"
// #include "multiclass.h"
#include "multiclass_curb.h"
#include "multimodal.h"

#include <set>

namespace py = pybind11;

using SparseMatrixR = Eigen::SparseMatrix<double, Eigen::RowMajor>;

int run_dta(std::string folder);

class Dta_Api
{
public:
  Dta_Api();
  ~Dta_Api();
  int initialize(std::string folder);
  int install_cc();
  int install_cc_tree();
  int run_once();
  int run_whole();
  int register_links(py::array_t<int> links);
  int register_paths(py::array_t<int> paths);
  int get_cur_loading_interval();
  py::array_t<double> get_link_inflow(py::array_t<int>start_intervals, 
                                        py::array_t<int>end_intervals);
  py::array_t<double> get_link_tt(py::array_t<int>start_intervals);
  py::array_t<double> get_path_tt(py::array_t<int>start_intervals);
  py::array_t<double> get_link_in_cc(int link_ID);
  py::array_t<double> get_link_out_cc(int link_ID);
  py::array_t<double> get_dar_matrix(py::array_t<int>link_start_intervals, py::array_t<int>link_end_intervals);
  SparseMatrixR get_complete_dar_matrix(py::array_t<int>start_intervals, py::array_t<int>end_intervals,
                                                int num_intervals, py::array_t<double> f);
  MNM_Dta *m_dta;
  std::vector<MNM_Dlink*> m_link_vec;
  std::vector<MNM_Path*> m_path_vec;
  std::unordered_map<MNM_Path*, int> m_path_map; 
  // std::unordered_map<MNM_Dlink*, int> m_link_map; 
  std::unordered_map<TInt, MNM_Path*> m_ID_path_mapping;
};

class Mcdta_Api_Biclass
{
public: 
  Mcdta_Api_Biclass();  
  ~Mcdta_Api_Biclass();

  MNM_Dta_Multiclass *m_mcdta;
  std::vector<MNM_Dlink_Multiclass*> m_link_vec_count; // for get_count_data and count DAR
  std::vector<MNM_Dlink_Multiclass*> m_link_vec_tt; // for get_travel_time
  std::vector<MNM_Dlink_Multiclass*> m_link_vec_density; // for get_density

  std::vector<MNM_Path*> m_path_vec_car;
  std::set<MNM_Path*> m_path_set_car; 
  std::unordered_map<TInt, MNM_Path*> m_ID_path_mapping_car;

  std::vector<MNM_Path*> m_path_vec_truck; // truck
  std::set<MNM_Path*> m_path_set_truck; // truck
  std::unordered_map<TInt, MNM_Path*> m_ID_path_mapping_truck; // truck

  int initialize_biclass_sep(std::string folder);
  int preloading();
  int run_once_control(int load_int, int assign_int);
  int run_whole_control();
  int run_whole_control_false();

  int register_paths_car(py::array_t<int> paths_car);
  int register_paths_truck(py::array_t<int> paths_truck);
  int register_links_count(py::array_t<int> links);
  int register_links_tt(py::array_t<int> links);
  int register_links_density(py::array_t<int> links);
  int install_cc_separate_with_trees_density();
  int get_cur_loading_interval();

  // condition functions
  // count
  py::array_t<double> get_link_inflow_car(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
  py::array_t<double> get_link_inflow_truck(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
  py::array_t<double> get_link_outflow_car(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
  py::array_t<double> get_link_outflow_truck(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
  
  // TT
  py::array_t<double> get_link_tt_car(py::array_t<double>start_intervals);
  py::array_t<double> get_link_tt_truck(py::array_t<double>start_intervals);

  py::array_t<double> get_link_fftt_car(py::array_t<int>link_IDs);
  py::array_t<double> get_link_fftt_truck(py::array_t<int>link_IDs);

  // density
  py::array_t<double> get_link_density_car(py::array_t<int>timestamps);
  py::array_t<double> get_link_density_truck(py::array_t<int>timestamps);

  py::array_t<double> get_link_density_car_robust(py::array_t<int>timestamps);
  py::array_t<double> get_link_density_truck_robust(py::array_t<int>timestamps);

  // count DAR
  py::array_t<double> get_car_dar_matrix_count(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
  py::array_t<double> get_truck_dar_matrix_count(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
  
  // TT DAR
  py::array_t<double> get_car_dar_matrix_tt(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
  py::array_t<double> get_truck_dar_matrix_tt(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
  
  // density DAR
  py::array_t<double> get_car_dar_matrix_density_in(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
  py::array_t<double> get_truck_dar_matrix_density_in(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
  
  py::array_t<double> get_car_dar_matrix_density_out(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
  py::array_t<double> get_truck_dar_matrix_density_out(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
  
};

class Mcdta_Api
{
public:
  Mcdta_Api();
  ~Mcdta_Api();
  int initialize(std::string folder);
  int initialize_curb(std::string folder);
  int install_cc();
  int install_cc_tree();
  int run_whole();

// jiachao added in May 04
  int run_once_control(int load_int, int assign_int);
  int run_whole_control();
  int run_whole_control_false();

  int preloading();
// jiachao added
  int run_whole_curb();

  int run_whole_curb_false();
  // bool check_movement(std::string check_movement);

  // double get_control_rate_single(std::string movement_id);

  // py::array_t<std::string> get_control_movement_all();

  // py::array_t<double> get_control_rate_all();
  
  // int change_control_rate_single(std::string movement_id, double control_rate);

  // int change_control_rate_all(py::array_t<std::string>movement_ids, py::array_t<double>control_rates);


// added end

  int register_links(py::array_t<int> links);
  int get_cur_loading_interval();
  py::array_t<double> get_travel_stats();
  int print_emission_stats();
  int print_simulation_results(std::string folder, int cong_frequency = 180);

  // Jiachao added in Feb 2023
  int build_link_cost_map(bool with_congestion_indicator=false); // checked
  int get_link_queue_dissipated_time();// checked

  py::array_t<double> get_car_link_fftt(py::array_t<int>link_IDs); // checked
  py::array_t<double> get_truck_link_fftt(py::array_t<int>link_IDs); // checked
  // added end
  
  py::array_t<double> get_car_link_tt(py::array_t<double>start_intervals);
  py::array_t<double> get_car_link_tt_robust(py::array_t<double>start_intervals, py::array_t<double>end_intervals);
  py::array_t<double> get_truck_link_tt(py::array_t<double>start_intervals);
  
  py::array_t<double> get_car_link_speed(py::array_t<double>start_intervals);
  py::array_t<double> get_truck_link_speed(py::array_t<double>start_intervals);
  
  py::array_t<double> get_link_car_inflow(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
  py::array_t<double> get_link_truck_inflow(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
  py::array_t<double> get_link_rh_inflow(py::array_t<int>start_intervals, py::array_t<int>end_intervals);

  py::array_t<double> get_link_car_outflow(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
  py::array_t<double> get_link_truck_outflow(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
  
  // TODO 10/04
  py::array_t<double> get_link_rh_outflow(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
  
  int register_paths(py::array_t<int> paths);
  // Jiachao
  int register_paths_cc(py::array_t<int> paths_cc); // cc means curb choice

  py::array_t<double> get_car_link_out_cc(int link_ID); 
  py::array_t<double> get_car_link_in_cc(int link_ID); 
  py::array_t<double> get_truck_link_out_cc(int link_ID); 
  py::array_t<double> get_truck_link_in_cc(int link_ID); 

  py::array_t<double> get_enroute_and_queue_veh_stats_agg();
  py::array_t<double> get_queue_veh_each_link(py::array_t<int>useful_links, py::array_t<int>intervals);
  
  double get_car_link_out_num(int link_ID, double time);
  double get_truck_link_out_num(int link_ID, double time);

  // py::array_t<double> get_car_dar_matrix(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
  // py::array_t<double> get_truck_dar_matrix(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
  // py::array_t<double> get_rh_dar_matrix(py::array_t<int>start_intervals, py::array_t<int>end_intervals);

  // new added curb arrival and departure DAR
  // py::array_t<double> get_car_dar_arrival_matrix(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
  // py::array_t<double> get_car_dar_departure_matrix(py::array_t<int>start_intervals, py::array_t<int>end_intervals);

  // py::array_t<double> get_truck_dar_arrival_matrix(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
  // py::array_t<double> get_truck_dar_departure_matrix(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
  
  // py::array_t<double> get_rh_dar_arrival_matrix(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
  // py::array_t<double> get_rh_dar_departure_matrix(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
  // end added curb arrival and departure DAR
  
  py::array_t<double> get_waiting_time_at_intersections();
  py::array_t<int> get_link_spillback();
  py::array_t<double> get_path_tt_car(py::array_t<int>link_IDs, py::array_t<double>start_intervals);
  py::array_t<double> get_path_tt_truck(py::array_t<int>link_IDs, py::array_t<double>start_intervals);

  // Jiachao added for different link set for count, TT and curb observations - 0604
  // 0614 jiachao
  int initialize_curb_sep(std::string folder);
  int initialize_biclass_sep(std::string folder);

  int register_paths_car(py::array_t<int> paths_car);
  int register_paths_truck(py::array_t<int> paths_truck);
  int register_paths_rh(py::array_t<int> paths_rh);

  int register_links_count(py::array_t<int> links);
  int register_links_tt(py::array_t<int> links);
  int register_links_curb(py::array_t<int> links);
  int register_links_density(py::array_t<int> links);
  
  int install_cc_separate();
  int install_cc_tree_separate();
  int install_cc_separate_with_trees_curb();
  int install_cc_separate_with_trees_density();

  int delete_mcdta();

  std::vector<MNM_Dlink_Multiclass*> m_link_vec_count; // for get_count_data and count DAR
  std::vector<MNM_Dlink_Multiclass*> m_link_vec_tt; // for get_travel_time
  std::vector<MNM_Dlink_Multiclass*> m_link_vec_curb; // for get_curb_in/out_flow and DAR
  std::vector<MNM_Dlink_Multiclass*> m_link_vec_density; // for get_density

  MNM_Dta_Multiclass_Curb *m_mcdta;

  // TT
  py::array_t<double> get_car_link_tt_sep(py::array_t<double>start_intervals);
  py::array_t<double> get_truck_link_tt_sep(py::array_t<double>start_intervals);

  // count
  py::array_t<double> get_link_car_inflow_sep(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
  py::array_t<double> get_link_truck_inflow_sep(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
  py::array_t<double> get_link_rh_inflow_sep(py::array_t<int>start_intervals, py::array_t<int>end_intervals);

  // density
  py::array_t<double> get_link_k_car_total_robust(py::array_t<int>timestamps);
  py::array_t<double> get_link_k_truck_total_robust(py::array_t<int>timestamps);

  py::array_t<double> get_link_k_car_moving_robust(py::array_t<int>timestamps);
  py::array_t<double> get_link_k_truck_moving_robust(py::array_t<int>timestamps);

  py::array_t<double> get_link_k_car_parking_robust(py::array_t<int>timestamps);
  py::array_t<double> get_link_k_truck_parking_robust(py::array_t<int>timestamps);

  // curb
  py::array_t<double> get_link_truck_curb_inflow_sep(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
  py::array_t<double> get_link_rh_curb_inflow_sep(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
  py::array_t<double> get_link_truck_curb_outflow_sep(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
  py::array_t<double> get_link_rh_curb_outflow_sep(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
  
  // count DAR
  py::array_t<double> get_car_dar_matrix_sep(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
  py::array_t<double> get_truck_dar_matrix_sep(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
  py::array_t<double> get_rh_dar_matrix_sep(py::array_t<int>start_intervals, py::array_t<int>end_intervals);

  // density DAR
  py::array_t<double> get_car_dar_k_in(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
  py::array_t<double> get_truck_dar_k_in(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
  py::array_t<double> get_rh_dar_k_in(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
  py::array_t<double> get_car_dar_k_out(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
  py::array_t<double> get_truck_dar_k_out(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
  py::array_t<double> get_rh_dar_k_out(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
  
  // density for parking DAR
  py::array_t<double> get_rh_dar_k_in_parking(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
  py::array_t<double> get_truck_dar_k_in_parking(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
  py::array_t<double> get_rh_dar_k_out_parking(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
  py::array_t<double> get_truck_dar_k_out_parking(py::array_t<int>start_intervals, py::array_t<int>end_intervals);

  // TT DAR
  py::array_t<double> get_car_dar_matrix_tt(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
  py::array_t<double> get_truck_dar_matrix_tt(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
  py::array_t<double> get_rh_dar_matrix_tt(py::array_t<int>start_intervals, py::array_t<int>end_intervals); 

  // curb DAR
  py::array_t<double> get_truck_dar_arrival_matrix_sep(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
  py::array_t<double> get_truck_dar_departure_matrix_sep(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
  py::array_t<double> get_rh_dar_arrival_matrix_sep(py::array_t<int>start_intervals, py::array_t<int>end_intervals);
  py::array_t<double> get_rh_dar_departure_matrix_sep(py::array_t<int>start_intervals, py::array_t<int>end_intervals);

  // Jiachao added
  // py::array_t<double> get_car_ltg_matrix(py::array_t<int>start_intervals, int threshold_timestamp); // dim = 5 * _record.size()
  // py::array_t<double> get_truck_ltg_matrix(py::array_t<int>start_intervals, int threshold_timestamp);
  // py::array_t<double> get_rh_ltg_matrix(py::array_t<int>start_intervals, int threshold_timestamp);

  // 0626 added by Jiachao
  py::array_t<double> get_car_ltg_matrix_sep(py::array_t<int>start_intervals, int threshold_timestamp); // dim = 5 * _record.size()
  py::array_t<double> get_truck_ltg_matrix_sep(py::array_t<int>start_intervals, int threshold_timestamp);
  py::array_t<double> get_rh_ltg_matrix_sep(py::array_t<int>start_intervals, int threshold_timestamp);
  // added end

  MNM_Dta_Multiclass *m_mcdta_bi;

  std::vector<MNM_Dlink_Multiclass*> m_link_vec;
  std::vector<MNM_Path*> m_path_vec;
  std::set<MNM_Path*> m_path_set; 
  std::unordered_map<TInt, MNM_Path*> m_ID_path_mapping;
  // jiachao added
  std::vector<MNM_Path*> m_path_vec_truck; // truck
  std::set<MNM_Path*> m_path_set_truck; // truck
  std::unordered_map<TInt, MNM_Path*> m_ID_path_mapping_truck; // truck
  
  std::unordered_map<TInt, MNM_Path*> m_ID_path_mapping_rh; //RH
  std::vector<MNM_Path*> m_path_vec_rh; // RH
  std::set<MNM_Path*> m_path_set_rh; // RH

  // Jiachao added
  // time-varying link tt
  std::unordered_map<TInt, TFlt *> m_link_tt_map;
  std::unordered_map<TInt, TFlt *> m_link_tt_map_truck;

  // time-varying link cost
  std::unordered_map<TInt, TFlt *> m_link_cost_map;
  std::unordered_map<TInt, TFlt *> m_link_cost_map_truck;

  // time-varying indicator
  std::unordered_map<TInt, bool *> m_link_congested_car;
  std::unordered_map<TInt, bool *> m_link_congested_truck;

  // time-varying queue dissipated time
  std::unordered_map<TInt, int *> m_queue_dissipated_time_car;
  std::unordered_map<TInt, int *> m_queue_dissipated_time_truck;

  std::unordered_map<TInt, MNM_TDSP_Tree*> m_tdsp_tree_map;
  // end adding
};

#endif