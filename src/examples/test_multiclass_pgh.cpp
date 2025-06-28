#include "io.h"
#include "multiclass.h"
#include "Snap.h"
#include <iostream>
#include <vector>

int main()
{
    // print cwd
    char buffer[256];
    char *val = getcwd(buffer, sizeof(buffer));
    if (val) {
        std::cout << buffer << std::endl;
    }

	std::string folder = "/srv/data/jiachao/MAC-POSTS/data/input_files_estimate_path_flow_v6_final999";

    printf("BEGIN multiclass test!\n");

    MNM_ConfReader *config = new MNM_ConfReader(folder + "/config.conf", "STAT");
    std::string rec_folder = config -> get_string("rec_folder");

    MNM_Dta_Multiclass *test_dta = new MNM_Dta_Multiclass(folder);
    printf("================================ DTA set! =================================\n");

    test_dta -> build_from_files_control();

    printf("========================= Finished initialization! new ========================\n");

    test_dta -> hook_up_node_and_link();
    printf("====================== Finished node and link hook-up! new ====================\n");

    test_dta -> is_ok();
    printf("============================ DTA is OK to run! ============================\n");

    MNM_Dlink *_link;
	MNM_Dlink_Multiclass *_link_multiclass;
    // MNM_Dlink_Ctm_Multiclass *_link_multiclass_ctm;

	for (auto _link_it = test_dta->m_link_factory->m_link_map.begin();
        _link_it != test_dta->m_link_factory->m_link_map.end(); _link_it++) {

		_link = _link_it -> second;

		_link_multiclass = dynamic_cast<MNM_Dlink_Multiclass *>(_link);

		_link_multiclass -> install_cumulative_curve_multiclass();

        _link_multiclass -> install_cumulative_curve_tree_multiclass();

        // _link_multiclass -> install_cumulative_curve_tree_multiclass_curb();
    }

	printf("=========================== Finished install cc and trees ===========================\n");

    test_dta -> pre_loading();
    printf("========================== Finished pre_loading! ==========================\n");

    TInt _current_inter = 0;
	TInt _assign_inter = test_dta -> m_start_assign_interval;
    std::string _str_parking = "curb_ID timestep parking_car parking_truck parking_rh dp_car dp_truck dp_rh\n";
    std::string _str_arr_dep = "curb_ID timestep arr_car arr_truck dep_car dep_truck\n";
    std::string _str_flow_tt = "link_ID timestep car_inflow truck_inflow rh_inflow car_tt truck_tt\n";
	std::ofstream _vis_file1;
    std::ofstream _vis_file2;
    std::ofstream _vis_file3;
    // std::ofstream _vis_emission;
    std::string _str_emission;
    // TInt save_parking_frequency = 1;
    // TInt save_arr_dep_frequency = 180;
    TInt save_flow_tt_frequency = 60;

    _vis_file1.open(folder + "/" + rec_folder + "/link_parking_num.txt", std::ofstream::out);
    _vis_file2.open(folder + "/" + rec_folder + "/parking_arr_dep.txt", std::ofstream::out);
    _vis_file3.open(folder + "/" + rec_folder + "/flow_tt.txt", std::ofstream::out);

    if (! _vis_file1.is_open()){
        printf("Error happens when open _vis_file1\n");
        exit(-1);
    }
    _vis_file1 << _str_parking;

    if (! _vis_file2.is_open()){
        printf("Error happens when open _vis_file2\n");
        exit(-1);
    }
    _vis_file2 << _str_arr_dep;

    if (! _vis_file3.is_open()){
        printf("Error happens when open _vis_file2\n");
        exit(-1);
    }
    _vis_file3 << _str_flow_tt;

    // _vis_emission.open(folder + "/" + rec_folder + "/link_emission_4.txt", std::ofstream::out);
    // if (! _vis_emission.is_open()){
    //     printf("Error happens when open _vis_emission\n");
    //     exit(-1);
    // }

    printf("========================== Loading starts ! ==========================\n");

	bool _verbose = true; // print process or not

    while (!test_dta -> finished_loading(_current_inter) || _assign_inter < test_dta -> m_total_assign_inter){
		printf("\nCurrent loading interval: %d, Current assignment interval: %d\n", _current_inter(), _assign_inter());

        test_dta -> load_once_control(_verbose, _current_inter, _assign_inter);

        test_dta -> m_emission -> update(test_dta->m_veh_factory);

        // if (_current_inter % save_parking_frequency == 0){

        //     for (auto _link_it = test_dta -> m_link_factory -> m_link_map.begin(); _link_it != test_dta -> m_link_factory -> m_link_map.end(); _link_it++){
        //         _link = _link_it -> second;
        //         _link_multiclass = dynamic_cast<MNM_Dlink_Multiclass*>(_link);

        //         if (std::find(curbs.begin(), curbs.end(), _link -> m_link_ID) != curbs.end()){
        //             _link_multiclass_ctm = dynamic_cast<MNM_Dlink_Ctm_Multiclass*>(_link_multiclass);
        //             // link_ID timestep parking_car truck rh dp_car truck rh
        //             _str_parking = std::to_string(_link -> m_link_ID) + " " + std::to_string(int(_current_inter)) + " ";
        //             _str_parking += std::to_string(MNM_DTA_GRADIENT::get_link_parking_num_car(_link_multiclass_ctm)) + " " + std::to_string(MNM_DTA_GRADIENT::get_link_parking_num_truck(_link_multiclass_ctm)) + " " + std::to_string(MNM_DTA_GRADIENT::get_link_parking_num_rh(_link_multiclass_ctm)) + " "; 
        //             _str_parking += std::to_string(MNM_DTA_GRADIENT::get_link_dp_num_car(_link_multiclass_ctm)) + " " + std::to_string(MNM_DTA_GRADIENT::get_link_dp_num_truck(_link_multiclass_ctm)) + " " + std::to_string(MNM_DTA_GRADIENT::get_link_dp_num_rh(_link_multiclass_ctm)) + "\n";       
        //             _vis_file1 << _str_parking;
        //         }
        //         // _str_emission = "\ntimestamp (intervals): " + std::to_string(int(_current_inter)) + " ";
        //         // _str_emission += "link_ID: " + std::to_string(_link -> m_link_ID()) + " ";
        //         // _str_emission += "fuel " + std::to_string(MNM_DTA_GRADIENT::get_link_fuel_rate_biclass(_link_multiclass, test_dta -> m_unit_time)) + " ";
        //         // _str_emission += "CO2 " + std::to_string(MNM_DTA_GRADIENT::get_link_CO2_rate_biclass(_link_multiclass, test_dta -> m_unit_time)) + " ";
        //         // _str_emission += "HC " + std::to_string(MNM_DTA_GRADIENT::get_link_HC_rate_biclass(_link_multiclass, test_dta -> m_unit_time)) + " ";
        //         // _str_emission += "CO " + std::to_string(MNM_DTA_GRADIENT::get_link_CO_rate_biclass(_link_multiclass, test_dta -> m_unit_time)) + " ";
        //         // _str_emission += "NOX " + std::to_string(MNM_DTA_GRADIENT::get_link_NOX_rate_biclass(_link_multiclass, test_dta -> m_unit_time)) + " ";
        //         // _str_emission += "VHT " + std::to_string(MNM_DTA_GRADIENT::get_link_VHT_biclass(_link_multiclass, test_dta -> m_unit_time)) + " ";
        //         // _str_emission += "VMT " + std::to_string(MNM_DTA_GRADIENT::get_link_VMT_biclass(_link_multiclass, test_dta -> m_unit_time)) + " ";
        //         // _str_emission += "car_link_flow: " + std::to_string(_link_multiclass -> get_link_flow_car()) + " ";
		// 		// _str_emission += "truck_link_flow: " + std::to_string(_link_multiclass -> get_link_flow_truck()) + "\n";
        //         // _vis_emission << _str_emission;
        //     }
        // }

        if (_current_inter % test_dta -> m_assign_freq == 0 || _current_inter == 0){
			_assign_inter += 1;
		}
        _current_inter += 1;

        // cannot stop when testing Oct. 19
        // interval 1823: Released vehicle 10974, Enroute vehicle 5979, Finished vehicle 4995

    }

    TInt _iter = 0;
    while (_iter < _current_inter){
        // if (_iter % save_arr_dep_frequency == 0 || _iter == _current_inter - 1){
        //     printf("Saving curb arrival and departure, current loading interval: %d\n", int(_iter));
        //     for (auto _link_it = test_dta -> m_link_factory -> m_link_map.begin(); _link_it != test_dta -> m_link_factory -> m_link_map.end(); _link_it++){
        //         _link = _link_it -> second;
        //         _link_multiclass = dynamic_cast<MNM_Dlink_Multiclass*>(_link);
        //         // if (std::find(curbs.begin(), curbs.end(), _link -> m_link_ID) != curbs.end()){
        //         _str_arr_dep = std::to_string(_link -> m_link_ID) + " " + std::to_string(int(_iter)) + " ";
        //         _str_arr_dep += std::to_string(MNM_DTA_GRADIENT::get_curb_inflow_rh(_link_multiclass, TFlt(_iter), TFlt(_iter + save_arr_dep_frequency))) + " ";
        //         _str_arr_dep += std::to_string(MNM_DTA_GRADIENT::get_curb_inflow_truck(_link_multiclass, TFlt(_iter), TFlt(_iter + save_arr_dep_frequency))) + " ";
        //         _str_arr_dep += std::to_string(MNM_DTA_GRADIENT::get_curb_outflow_rh(_link_multiclass, TFlt(_iter), TFlt(_iter + save_arr_dep_frequency))) + " ";
        //         _str_arr_dep += std::to_string(MNM_DTA_GRADIENT::get_curb_outflow_truck(_link_multiclass, TFlt(_iter), TFlt(_iter + save_arr_dep_frequency))) + "\n";
        //         _vis_file2 << _str_arr_dep;
        //         // }
        //     }
        // }

        if (_iter % save_flow_tt_frequency == 0 || _iter == _current_inter - 1){
            // printf("Saving link flow and tt, current loading interval: %d\n", int(_iter));
            for (auto _link_it = test_dta -> m_link_factory -> m_link_map.begin(); _link_it != test_dta -> m_link_factory -> m_link_map.end(); _link_it++){
                _link = _link_it -> second;
                _link_multiclass = dynamic_cast<MNM_Dlink_Multiclass*>(_link);
                // if (std::find(curbs.begin(), curbs.end(), _link -> m_link_ID) != curbs.end()){
                // link_ID timestep car_inflow truck_inflow car_tt truck_tt
                _str_flow_tt = std::to_string(_link -> m_link_ID) + " " + std::to_string(int(_iter)) + " ";
                _str_flow_tt += std::to_string(MNM_DTA_GRADIENT::get_link_inflow_car(_link_multiclass, TFlt(_iter), TFlt(_iter + save_flow_tt_frequency))) + " ";
                _str_flow_tt += std::to_string(MNM_DTA_GRADIENT::get_link_inflow_truck(_link_multiclass, TFlt(_iter), TFlt(_iter + save_flow_tt_frequency))) + " ";
                _str_flow_tt += std::to_string(MNM_DTA_GRADIENT::get_link_inflow_rh(_link_multiclass, TFlt(_iter), TFlt(_iter + save_flow_tt_frequency))) + " ";
                _str_flow_tt += std::to_string(MNM_DTA_GRADIENT::get_travel_time_car(_link_multiclass, TFlt(_iter), TFlt(test_dta -> m_unit_time), TInt(_current_inter))) + " ";
                _str_flow_tt += std::to_string(MNM_DTA_GRADIENT::get_travel_time_truck(_link_multiclass, TFlt(_iter), TFlt(test_dta -> m_unit_time), TInt(_current_inter))) + " ";
                _str_flow_tt += std::to_string(MNM_DTA_GRADIENT::get_link_density_car(_link_multiclass, TFlt(_iter), TInt(_current_inter))) + " ";
                _str_flow_tt += std::to_string(MNM_DTA_GRADIENT::get_link_density_truck(_link_multiclass, TFlt(_iter), TInt(_current_inter))) + " ";
                _str_flow_tt += std::to_string(MNM_DTA_GRADIENT::get_link_density_stop_car(_link_multiclass, TFlt(_iter), TInt(_current_inter))) + " ";
                _str_flow_tt += std::to_string(MNM_DTA_GRADIENT::get_link_density_stop_truck(_link_multiclass, TFlt(_iter), TInt(_current_inter))) + "\n";
                _vis_file3 << _str_flow_tt;
                // }
            }
        }

        _iter += 1;
    }

    if (_vis_file1.is_open()) _vis_file1.close();

    if (_vis_file2.is_open()) _vis_file2.close();

    if (_vis_file3.is_open()) _vis_file3.close();

    // // if (_vis_emission.is_open()) _vis_emission.close();

    //  save emission
    std::ofstream _vis_emission_total;
    _vis_emission_total.open(folder + "/" + rec_folder + "/metrics_emission_total.txt", std::ofstream::out);
    if (! _vis_emission_total.is_open()){
        printf("Error happens when open _vis_emission_total\n");
        exit(-1);
    }

    // std::string _str_emission_total;
    // _str_emission_total = test_dta->m_emission->output_save();
    // _vis_emission_total << _str_emission_total;
    // if (_vis_emission_total.is_open()) _vis_emission_total.close();

    // output CC of some special links
    // MNM_Dlink_Multiclass *_link_m;
    // std::string _link_ID;
	// for (auto _link_it = test_dta -> m_link_factory -> m_link_map.begin(); _link_it != test_dta -> m_link_factory -> m_link_map.end(); _link_it++){
		
    //     _link = _link_it -> second;
	// 	// if (_link -> m_link_ID() == 7){
    //     _link_m = dynamic_cast<MNM_Dlink_Multiclass*>(_link);

    //     _link_ID = std::to_string(_link -> m_link_ID);
    //     printf("\nlink_ID\n");
    //     std::cout << _link_ID << std::endl;

    //     if (_link_m -> m_N_in_car_all -> m_recorder.back().second != _link_m -> m_N_out_car_all -> m_recorder.back().second){
    //         printf("\nm_N_in_car_all: \n");
    //         std::cout << int(_link_m -> m_N_in_car_all -> m_recorder.back().second) << std::endl;
    //         printf("\nm_N_out_car_all: \n");
    //         std::cout << int(_link_m -> m_N_out_car_all -> m_recorder.back().second) << std::endl;
    //     }

    //     if (_link_m -> m_N_in_car_cc -> m_recorder.back().second != _link_m -> m_N_out_car_cc -> m_recorder.back().second){
    //         printf("\nm_N_in_car_cc: \n");
    //         std::cout << int(_link_m -> m_N_in_car_cc -> m_recorder.back().second) << std::endl;
    //         printf("\nm_N_out_car_cc: \n");
    //         std::cout << int(_link_m -> m_N_out_car_cc -> m_recorder.back().second) << std::endl;
    //     }

    //     if (_link_m -> m_N_in_truck -> m_recorder.back().second != _link_m -> m_N_out_truck -> m_recorder.back().second){
    //         printf("\nm_N_in_truck: \n");
    //         std::cout << int(_link_m -> m_N_in_truck -> m_recorder.back().second) << std::endl;
    //         printf("\nm_N_out_truck: \n");
    //         std::cout << int(_link_m -> m_N_out_truck -> m_recorder.back().second) << std::endl;
    //     }

    //     if (_link_m -> m_N_in_truck_cc -> m_recorder.back().second != _link_m -> m_N_out_truck_cc -> m_recorder.back().second){
    //         printf("\nm_N_in_truck_cc: \n");
    //         std::cout << int(_link_m -> m_N_in_truck_cc -> m_recorder.back().second) << std::endl;
    //         printf("\nm_N_out_truck_cc: \n");
    //         std::cout << int(_link_m -> m_N_out_truck_cc -> m_recorder.back().second) << std::endl;
    //     }

    //     if (_link_m -> m_N_in_rh -> m_recorder.back().second != _link_m -> m_N_out_rh -> m_recorder.back().second){
    //         printf("\nm_N_in_rh: \n");
    //         std::cout << int(_link_m -> m_N_in_rh -> m_recorder.back().second) << std::endl;
    //         printf("\nm_N_out_rh: \n");
    //         std::cout << int(_link_m -> m_N_out_rh -> m_recorder.back().second) << std::endl;
    //     }

    //     if (_link_m -> m_N_in_rh_cc -> m_recorder.back().second != _link_m -> m_N_out_rh_cc -> m_recorder.back().second){
    //         printf("\nm_N_in_rh_cc: \n");
    //         std::cout << int(_link_m -> m_N_in_rh_cc -> m_recorder.back().second) << std::endl;
    //         printf("\nm_N_out_rh_cc: \n");
    //         std::cout << int(_link_m -> m_N_out_rh_cc -> m_recorder.back().second) << std::endl;
    //     }
	// }
    
    // // save link metrics
    // MNM_Dlink_Multiclass *_link_m;
    // std::string _str;
    // bool output_link_cong = true;
    // TInt cong_frequency = 180;

	// std::ofstream _vis_file2;
	// if (output_link_cong){
	// 	_vis_file2.open(folder + "/" + rec_folder + "/link_cong_raw_frq_180int_3.txt", std::ofstream::out);
	// 	if (! _vis_file2.is_open()){
    //     	printf("Error happens when open _vis_file2\n");
    //     	exit(-1);
    //     }
        
	// 	TInt _iter = 0;
    //     while (_iter < _current_inter){
    //         if (_iter % cong_frequency == 0 || _iter == _current_inter - 1){
    //             printf("Current loading interval: %d\n", int(_iter));
    //             for (auto _link_it = test_dta -> m_link_factory -> m_link_map.begin(); _link_it != test_dta -> m_link_factory -> m_link_map.end(); _link_it++){
    //                 _link = _link_it -> second;
    //                 // if (std::find(monitor.begin(), monitor.end(), _link ->m_link_ID) != monitor.end()){
    //                 _link_m = dynamic_cast<MNM_Dlink_Multiclass*>(_link);
    //                 _str = "\ntimestamp (intervals): " + std::to_string(int(_iter)) + " ";
    //                 _str += "link_ID: " + std::to_string(_link -> m_link_ID()) + " ";
    //                 _str += "car_inflow: " + std::to_string(MNM_DTA_GRADIENT::get_link_inflow_car(_link_m, _iter, _iter + 1)) + " ";
    //                 _str += "truck_inflow: " + std::to_string(MNM_DTA_GRADIENT::get_link_inflow_truck(_link_m, _iter, _iter + 1)) + " ";
    //                 _str += "car_tt_FD (s): " + std::to_string(MNM_DTA_GRADIENT::get_travel_time_from_FD_car(_link_m, TFlt(_iter + 1), test_dta -> m_unit_time) * test_dta -> m_unit_time) + " ";
    //                 _str += "truck_tt_FD (s): " + std::to_string(MNM_DTA_GRADIENT::get_travel_time_from_FD_truck(_link_m, TFlt(_iter + 1), test_dta -> m_unit_time) * test_dta -> m_unit_time) + " ";
    //                 _str += "car_tt (s): " + std::to_string(MNM_DTA_GRADIENT::get_travel_time_car(_link_m, TFlt(_iter + 1), test_dta -> m_unit_time) * test_dta -> m_unit_time) + " ";
    //                 _str += "truck_tt (s): " + std::to_string(MNM_DTA_GRADIENT::get_travel_time_truck(_link_m, TFlt(_iter + 1), test_dta -> m_unit_time) * test_dta -> m_unit_time) + " ";
    //                 _str += "car_fftt (s): " + std::to_string(_link_m -> get_link_freeflow_tt_car()) + " ";
    //                 _str += "truck_fftt (s): " + std::to_string(_link_m -> get_link_freeflow_tt_truck()) + "\n";
                    
    //                 _vis_file2 << _str;
    //                     // printf("link id = %d\n", int(_link->m_link_ID));
    //             }
    //         }
    //         _iter += 1;
    //     }
	// 	if (_vis_file2.is_open()) _vis_file2.close();
	// }

    delete config;
    delete test_dta;
    printf("\n\nFinished delete test_dta!\n");

    return 0;
}
