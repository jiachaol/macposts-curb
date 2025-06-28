#include "io.h"
#include "multiclass.h"
#include "Snap.h"
#include <iostream>
#include <vector>

// void print_vector(std::vector<float> const &a){
// 	std::cout << "The vector elements are :";

// 	for (size_t i = 0; i < a.size(); i++){
// 		std::cout << a.at(i) << " ";
// 	}
// 	std::cout << "\n";
// }

// void print_vector_string(std::vector<std::string> const &a){
// 	std::cout << "The vector elements are :";

// 	for (size_t i = 0; i < a.size(); i++){
// 		std::cout << a.at(i) << " ";
// 	}
// 	std::cout << "\n";
// }


int main()
{
	// check list
	std::vector<int> monitor;
	monitor.push_back(489);
	monitor.push_back(480);
	monitor.push_back(607);
	monitor.push_back(331);
	monitor.push_back(563);
	monitor.push_back(461);
	monitor.push_back(564);
	monitor.push_back(467);
	monitor.push_back(466);
	monitor.push_back(395);
	monitor.push_back(397);
	monitor.push_back(469);	
	monitor.push_back(2001);

	// std::vector<float> alinea_ki_vector = {0.0022, 0.0044, 0.0067};	
	// std::vector<float> lsc_qm_vector = {6.0, 7.0, 8.0};
	// std::vector<float> set_point_vector = {10.0, 14.0, 18.0};

	std::vector<float> alinea_ki_vector = {0.0044};	
	std::vector<float> lsc_qm_vector = {7.0};
	std::vector<float> set_point_vector = {14.0};

	// print cwd
    char buffer[256];
    char *val = getcwd(buffer, sizeof(buffer));
    if (val) {
        std::cout << buffer << std::endl;
    }

	std::string control_plan = "lsc";
	// std::string folder = "/srv/data/jiachao/MAC-POSTS/data/TSMO_ALINEA";
	std::string folder = "/srv/data/jiachao/MAC-POSTS/data/TSMO_LSC";

	for (int k = 0; k < (int)alinea_ki_vector.size(); ++k){
		for (int q = 0; q < (int)lsc_qm_vector.size(); ++q){
			for (int s = 0; s < (int)set_point_vector.size(); ++s){
				printf("BEGIN multiclass test!\n");

				MNM_ConfReader *config = new MNM_ConfReader(folder + "/config.conf", "STAT");
				std::string rec_folder = config -> get_string("rec_folder");

				MNM_Dta_Multiclass *test_dta = new MNM_Dta_Multiclass(folder);
				printf("================================ DTA set! =================================\n");

				test_dta -> build_from_files_control();
				printf("========================= Finished initialization! new ========================\n");

				// test_dta ->build_curbs();

				test_dta->m_control_factory->m_ramp_metering_map[480][5] = alinea_ki_vector[k];

				test_dta->m_control_factory->m_ramp_metering_map[480][6] = lsc_qm_vector[q];

				test_dta->m_control_factory->m_ramp_metering_map[480][7] = set_point_vector[s];

				printf("========================= current param setting: k = %f, q = %f, s = %f ================\n", test_dta->m_control_factory->m_ramp_metering_map[480][5],test_dta->m_control_factory->m_ramp_metering_map[480][6],test_dta->m_control_factory->m_ramp_metering_map[480][7]);

				test_dta -> hook_up_node_and_link();
				printf("====================== Finished node and link hook-up! new ====================\n");

				test_dta -> is_ok();
				printf("============================ DTA is OK to run! ============================\n");

				test_dta -> pre_loading();
				printf("========================== Finished pre_loading! ==========================\n");

				printf("========== emission link registered = %d ==============\n", (int)test_dta->m_emission->m_link_vector.size());

				TInt _current_inter = 0;
				TInt _assign_inter = test_dta -> m_start_assign_interval;
				
				bool _verbose = true; // print process or not
				
				std::ofstream _vis_file;
				std::string _str;
				std::string _str_link_flow;

				_vis_file.open(folder + "/" + rec_folder + "/link_flow_" + control_plan + "_control_k" + std::to_string(alinea_ki_vector[k]) + "q" + std::to_string(lsc_qm_vector[q]) + "s" + std::to_string(set_point_vector[s]) + ".txt", std::ofstream::out);
				if (! _vis_file.is_open()){
					printf("Error happens when open _vis_file\n");
					exit(-1);
				}
				_vis_file << "timestamp,link_ID,car_link_flow,truck_link_flow\n";
				MNM_Dlink *_link_temp; 
				MNM_Dlink_Multiclass *_link_m_temp;

				while (!test_dta -> finished_loading(_current_inter) || _assign_inter < test_dta -> m_total_assign_inter){
					printf("\nCurrent loading interval: %d, Current assignment interval: %d\n", _current_inter(), _assign_inter());

					test_dta -> load_once_control(_verbose, _current_inter, _assign_inter);
					test_dta -> m_emission -> update(test_dta->m_veh_factory);

					for (auto _link_it = test_dta -> m_link_factory -> m_link_map.begin(); _link_it != test_dta -> m_link_factory -> m_link_map.end(); _link_it++){
						_link_temp = _link_it -> second;
						if (std::find(monitor.begin(), monitor.end(), _link_temp -> m_link_ID) != monitor.end()){
							_link_m_temp = dynamic_cast<MNM_Dlink_Multiclass*>(_link_temp);
							_str_link_flow = std::to_string(int(_current_inter)) + ",";
							_str_link_flow += std::to_string(_link_temp -> m_link_ID()) + ",";
							_str_link_flow += std::to_string(_link_m_temp -> get_link_flow_car()) + ",";
							_str_link_flow += std::to_string(_link_m_temp -> get_link_flow_truck()) + "\n";
							_vis_file << _str_link_flow;
						}
					}

					if (_current_inter % test_dta -> m_assign_freq == 0 || _current_inter == 0){
						_assign_inter += 1;
					}
					_current_inter += 1;
					// if (_current_inter > 1800) break;
				}
				if (_vis_file.is_open()) _vis_file.close();

				std::ofstream _vis_emission;
				_vis_emission.open(folder + "/" + rec_folder + "/metrics_" + control_plan + "_control_k" + std::to_string(alinea_ki_vector[k]) + "q" + std::to_string(lsc_qm_vector[q]) + "s" + std::to_string(set_point_vector[s]) + ".txt", std::ofstream::out);
				if (! _vis_emission.is_open()){
				    printf("Error happens when open _vis_emission\n");
				    exit(-1);
				}

				std::string _str_emission;
				_str_emission = test_dta->m_emission->output_save();
				_vis_emission << _str_emission;
				if (_vis_emission.is_open()) _vis_emission.close();

				delete test_dta;
				printf("\n\nFinished delete test_dta!\n");
			}

		}
	}
	// printf("BEGIN multiclass test!\n");

	// // On ubuntu (PC)
	// // std::string folder = "/home/alanpi/Desktop/MAC-POSTS/data/input_files_SPC_separate_Routing";
	// // std::string folder = "/home/lemma/Documents/MAC-POSTS/src/examples/mcDODE/a6e7b31067d2ead8d3725fc0ed587d06c958f63c";
	// // std::string folder = "/srv/data/jiachao/MAC-POSTS/data/input_files_7link_multiclass";
	// // std::string folder = "/srv/data/jiachao/MAC-POSTS/data/input_files_MckeesRocks_SPC_xidong";
    // // std::string folder = "/srv/data/jiachao/MAC-POSTS/data/input_files_SR41_multiclass";
	
	// std::string folder = "/srv/data/jiachao/MAC-POSTS/data/input_files_TSMO_magnified_demands_inserted_link";

	// // on macOS (Mac air)
	// // std::string folder = "/Users/alan-air/Dropbox/MAC-POSTS/data/input_files_MckeesRocks_SPC";
	// // std::string folder = "/media/lemma/WD/nhd/experiments/src/temp_input";


    // MNM_ConfReader *config = new MNM_ConfReader(folder + "/config.conf", "STAT");
    // std::string rec_folder = config -> get_string("rec_folder");


	// MNM_Dta_Multiclass *test_dta = new MNM_Dta_Multiclass(folder);
	// printf("================================ DTA set! =================================\n");
	
	// // test_dta ->build_from_files_no_control();

	// // test_dta -> hook_up_node_and_link();

	// // test_dta ->build_control_file(folder);

	// test_dta -> build_from_files();
	// printf("========================= Finished initialization! new ========================\n");

	// test_dta->m_control_factory->m_ramp_metering_map[480][5];

	// test_dta->m_control_factory->m_ramp_metering_map[480][6];

	// test_dta->m_control_factory->m_ramp_metering_map[480][7];

	// printf("========================= current param setting: ");

	// // std::vector<float> control_rate_list = test_dta -> m_control_factory -> m_control_map_list["4_3_5"];

	// // std::cout << "list[0] contains " << control_rate_list[0] << " elements.\n";
	// // printf("\nnumber of movement: %d\n", int(test_dta -> m_control_factory -> m_control_map.size()));
	// // float _control_rate;
	// // std::string _movement_ID;
	// // for (auto _control_it = test_dta -> m_control_factory -> m_control_map.begin(); _control_it != test_dta -> m_control_factory -> m_control_map.end(); _control_it++){
	// // 	_movement_ID = _control_it -> first;
	// // 	_control_rate = _control_it -> second;
	// // 	printf("\nmovement id = %s, control_rate = %f\n", _movement_ID.c_str(), _control_rate);
	// // }

	// test_dta -> hook_up_node_and_link();
	// printf("====================== Finished node and link hook-up! new ====================\n");

	
	// // bool check_temp;
	// // check_temp = MNM_DTA_GRADIENT::check_movement_exist(test_dta, "4_3_5");
	// // if (check_temp){
	// // 	printf("True: test passed\n");
	// // }
	// // else{
	// // 	printf("False: error\n");
	// // }

	// // TFlt check_rate;

	// // check_rate = MNM_DTA_GRADIENT::get_control_rate_one(test_dta, "4_3_5");

	// // printf("check rate = %f\n", double(check_rate));

	// // MNM_DTA_GRADIENT::change_control_rate_one(test_dta, "4_3_5", 0.95);
	
	// // printf("check new rate = %f\n", double(test_dta -> m_control_factory -> m_control_map["4_3_5"]));

	// // auto control_rate_all = MNM_DTA_GRADIENT::get_control_rate_all(test_dta);

	// // print_vector(control_rate_all.second);

	// // print_vector_string(control_rate_all.first);

	// // // check in out link list
	// // // save in-out link relation file


	// test_dta -> is_ok();
	// printf("============================ DTA is OK to run! ============================\n");

	// test_dta -> pre_loading();
	// printf("========================== Finished pre_loading! ==========================\n");

	// printf("========== emission link registered = %d ==============\n", (int)test_dta->m_emission->m_link_vector.size());

	// // test_dta ->m_emission ->m_link_vector;
	// // test_dta ->loading_control(true);
	
	// // printf("\n\n\n====================================== End loading! =======================================\n");

	// // Output total travels and travel time, before divided by flow_scalar
	
	// // MNM_Veh_Multiclass* _veh;
	// // TInt _count_car = 0, _count_truck = 0;
	// // TFlt _tot_tt = 0.0;
	// // for (auto _map_it : test_dta -> m_veh_factory -> m_veh_map){
	// // 	if (_map_it.second -> m_finish_time > 0) {
	// // 		_veh = dynamic_cast<MNM_Veh_Multiclass *>(_map_it.second);
	// // 		if (_veh -> m_class == 0){
	// // 			_count_car += 1;
	// // 		}
	// // 		else {
	// // 			_count_truck += 1;
	// // 		}
	// // 		_tot_tt += (_veh -> m_finish_time - _veh -> m_start_time) * test_dta -> m_unit_time / 3600.0;
	// // 	}
	// // }
	// // printf("\n\n\nTotal car: %d, Total truck: %d, Total tt: %.2f hours\n\n\n\n", int(_count_car), int(_count_truck), float(_tot_tt));


	// // _current_inter is loading interval 
	// // _assign_inter is assinment interval

	// TInt _current_inter = 0;
	// TInt _assign_inter = test_dta -> m_start_assign_interval;
	
	// bool _verbose = true; // print process or not
	
	// // bool output_link_cong = true; // if true output link congestion level every cong_frequency
	// // TInt cong_frequency = 180; // 15 minutes
	// // bool output_veh_locs = true; // if true output veh location every vis_frequency
	// // TInt vis_frequency = 60; // 5 minutes
	// // MNM_Veh_Multiclass* _veh;
	// std::ofstream _vis_file;
	// std::string _str;
	// std::string _str_link_flow;

	// // if (output_veh_locs){
	// _vis_file.open(folder + "/" + rec_folder + "/link_flow_lsc_control_v2.txt", std::ofstream::out);
	// if (! _vis_file.is_open()){
    //     printf("Error happens when open _vis_file\n");
    //     exit(-1);
    // }

	// MNM_Dlink *_link_temp; 
	// MNM_Dlink_Multiclass *_link_m_temp;

    // while (!test_dta -> finished_loading(_current_inter) || _assign_inter < test_dta -> m_total_assign_inter){
    //     printf("\nCurrent loading interval: %d, Current assignment interval: %d\n", _current_inter(), _assign_inter());

	// 	test_dta -> load_once_control(_verbose, _current_inter, _assign_inter);
	// 	test_dta -> m_emission -> update(test_dta->m_veh_factory);

	// 	for (auto _link_it = test_dta -> m_link_factory -> m_link_map.begin(); _link_it != test_dta -> m_link_factory -> m_link_map.end(); _link_it++){
    //         _link_temp = _link_it -> second;
	// 		if (std::find(monitor.begin(), monitor.end(), _link_temp -> m_link_ID) != monitor.end()){
	// 			_link_m_temp = dynamic_cast<MNM_Dlink_Multiclass*>(_link_temp);
	// 			_str_link_flow = "\ntimestamp (intervals): " + std::to_string(int(_current_inter)) + " ";
	// 			_str_link_flow += "link_ID: " + std::to_string(_link_temp -> m_link_ID()) + " ";
	// 			_str_link_flow += "car_link_flow: " + std::to_string(_link_m_temp -> get_link_flow_car()) + " ";
	// 			_str_link_flow += "truck_link_flow: " + std::to_string(_link_m_temp -> get_link_flow_truck()) + "\n";
	// 			_vis_file << _str_link_flow;
	// 		}
	// 	}

	// 	if (_current_inter % test_dta -> m_assign_freq == 0 || _current_inter == 0){
	// 		_assign_inter += 1;
	// 	}
	// 	_current_inter += 1;
	// 	// if (_current_inter > 1800) break;
	// }
	// if (_vis_file.is_open()) _vis_file.close();


	// MNM_Dlink *_link; 
	// MNM_Dlink_Multiclass *_link_m;

	// std::ofstream _vis_file2;
	// if (output_link_cong){
	// 	_vis_file2.open(folder + "/" + rec_folder + "/link_metrics_lsc_v3.txt", std::ofstream::out);
	// 	if (! _vis_file2.is_open()){
    //     	printf("Error happens when open _vis_file2\n");
    //     	exit(-1);
    //     }
	// 	TInt _iter = 0;
    //     while (_iter < _current_inter){
    //         // if (_iter % cong_frequency == 0 || _iter == _current_inter - 1){
    //         printf("Current loading interval: %d\n", int(_iter));
    //         for (auto _link_it = test_dta -> m_link_factory -> m_link_map.begin(); _link_it != test_dta -> m_link_factory -> m_link_map.end(); _link_it++){
    //             _link = _link_it -> second;
	// 			if (std::find(monitor.begin(), monitor.end(), _link ->m_link_ID) != monitor.end()){
	// 				_link_m = dynamic_cast<MNM_Dlink_Multiclass*>(_link);
    //             	_str = "\ntimestamp (intervals): " + std::to_string(int(_iter)) + " ";
    //             	_str += "link_ID: " + std::to_string(_link -> m_link_ID()) + " ";
    //             	_str += "car_inflow: " + std::to_string(MNM_DTA_GRADIENT::get_link_inflow_car(_link_m, _iter, _iter + 1)) + " ";
    //             	_str += "truck_inflow: " + std::to_string(MNM_DTA_GRADIENT::get_link_inflow_truck(_link_m, _iter, _iter + 1)) + " ";
    //             	_str += "car_tt (s): " + std::to_string(MNM_DTA_GRADIENT::get_travel_time_from_FD_car(_link_m, TFlt(_iter + 1), test_dta -> m_unit_time) * test_dta -> m_unit_time) + " ";
    //             	_str += "truck_tt (s): " + std::to_string(MNM_DTA_GRADIENT::get_travel_time_from_FD_truck(_link_m, TFlt(_iter + 1), test_dta -> m_unit_time) * test_dta -> m_unit_time) + " ";
	// 				// _str += "car_tt (s): " + std::to_string(MNM_DTA_GRADIENT::get_travel_time_car(_link_m, TFlt(_iter + 1), test_dta -> m_unit_time) * test_dta -> m_unit_time) + " ";
	// 				// _str += "truck_tt (s): " + std::to_string(MNM_DTA_GRADIENT::get_travel_time_truck(_link_m, TFlt(_iter + 1), test_dta -> m_unit_time) * test_dta -> m_unit_time) + " ";
	// 				_str += "car_fftt (s): " + std::to_string(_link_m -> get_link_freeflow_tt_car()) + " ";
    //             	_str += "truck_fftt (s): " + std::to_string(_link_m -> get_link_freeflow_tt_truck()) + "\n";
    //             	_vis_file2 << _str;
	// 				// printf("link id = %d\n", int(_link->m_link_ID));
	// 			} 

    //         }
    //         // }
    //         _iter += 1;
    //     }
	// 	if (_vis_file2.is_open()) _vis_file2.close();
	// }
	
	// test_dta->m_emission->output();

	// std::ofstream _vis_emission;
	// _vis_emission.open(folder + "/" + rec_folder + "/metrics_emission.txt", std::ofstream::out);
	// if (! _vis_emission.is_open()){
    //     printf("Error happens when open _vis_emission\n");
    //     exit(-1);
    // }

	// std::string _str_emission;
	// _str_emission = std::to_string(test_dta->m_emission->output());
	// _vis_emission << _str_emission;
	// if (_vis_emission.is_open()) _vis_emission.close();
	// // // output tt of some special links
	// // for (auto _link_it = test_dta -> m_link_factory -> m_link_map.begin(); _link_it != test_dta -> m_link_factory -> m_link_map.end(); _link_it++){
	// // 		_link = _link_it -> second;
	// // 		if (_link -> m_link_ID() == 7186) {
	// // 			TInt _iter = 0;
	// // 			while (_iter < _current_inter){
	// // 				// if (_iter == 984){
	// // 					_link_m = dynamic_cast<MNM_Dlink_Multiclass*>(_link);
	// // 					printf("%d,%.2f,%.2f\n", int(_iter),
	// // 						double(MNM_DTA_GRADIENT::get_travel_time_car(_link_m, TFlt(_iter + 1), test_dta -> m_unit_time)),
	// // 						double(MNM_DTA_GRADIENT::get_travel_time_truck(_link_m, TFlt(_iter + 1), test_dta -> m_unit_time)));
	// // 				// }
	// // 				_iter += 1;
	// // 			}
	// // 		}
	// // }

	// // // output CC of some special links
	// // for (auto _link_it = test_dta -> m_link_factory -> m_link_map.begin(); 
	// // 			  _link_it != test_dta -> m_link_factory -> m_link_map.end(); _link_it++){
	// // 		_link = _link_it -> second;
	// // 	if (_link -> m_link_ID() == 7186){
	// // 		_link_m = dynamic_cast<MNM_Dlink_Multiclass*>(_link);
	// // 		printf("\n\nm_N_in_car: \n");
	// // 		std::cout <<_link_m -> m_N_in_car -> to_string() << std::endl;
	// // 		printf("\n\nm_N_out_car: \n");
	// // 		std::cout <<_link_m -> m_N_out_car -> to_string() << std::endl;
	// // 		printf("\n\nm_N_in_truck: \n");
	// // 		std::cout <<_link_m -> m_N_in_truck -> to_string() << std::endl;
	// // 		printf("\n\nm_N_out_truck: \n");
	// // 		std::cout <<_link_m -> m_N_out_truck -> to_string() << std::endl;
	// // 	}
	// // }

	// delete test_dta;
	// printf("\n\nFinished delete test_dta!\n");

	return 0;
}
