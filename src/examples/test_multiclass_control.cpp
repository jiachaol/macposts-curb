#include "io.h"
#include "multiclass.h"
#include "Snap.h"
#include <iostream>
#include <vector>

int main()
{
	// check list
	std::vector<int> monitor = {489, 480, 607, 331, 467, 469, 395, 397, 564, 461, 539, 563};
	// std::vector<int> monitor = {175, 35, 328, 329, 330, 331, 332, 333, 
	// 							334, 335, 340, 343, 344, 345, 346, 347, 
	// 							348, 349, 350, 352, 353, 356, 357, 371, 
	// 							372, 374, 375, 377, 378, 379, 380, 381, 
	// 							382, 383, 384, 385, 386, 387, 388, 389, 
	// 							390, 391, 392, 394, 395, 396, 397, 398, 
	// 							399, 400, 401, 409, 411, 412, 415, 416, 
	// 							417, 418, 419, 420, 421, 422, 423, 425, 
	// 							426, 427, 428, 429, 430, 444, 446, 455, 
	// 							459, 460, 461, 462, 463, 464, 465, 466, 
	// 							467, 468, 469, 470, 471, 472, 479, 480, 
	// 							481, 482, 483, 484, 485, 486, 487, 488, 489, 490, 491, 492, 493, 494, 495, 523, 525, 
	// 							538, 539, 552, 555, 560, 563, 564, 565, 566, 569, 570, 571, 572, 575, 578, 579, 582, 
	// 							583, 585, 598, 607, 608, 665, 668, 671, 672, 677};
	
	// location A
	// std::vector<int> monitor = {35, 328, 329, 330, 331, 332, 333, 334, 335, 340, 343, 344, 345, 444, 446, 479, 480, 481, 482, 483, 484, 485, 486, 487, 488, 489, 490, 491, 492, 
	// 							493, 523, 525, 578, 579, 582, 583, 585, 598, 607, 608, 665, 668};
	// location A smaller set
	// std::vector<int> monitor = {330,331,444,446,479,480,481,482,483,484,485,486,487,488,489,493,523,525,598,607,608};

	// location B
	// std::vector<int> monitor = {175, 349, 350, 352, 353, 356, 357, 371, 372, 374, 375, 377, 378, 379, 380, 381, 382, 383, 384, 385, 386, 387, 388, 389, 390, 391, 392, 394, 395, 
	// 							396, 397, 398, 399, 400, 401, 409, 411, 412, 415, 416, 417, 418, 419, 420, 421, 422, 423, 425, 426, 427, 428, 429, 430, 455, 459, 460, 461, 462, 
	// 							463, 464, 465, 466, 467, 468, 469, 470, 471, 472, 479, 538, 539, 552, 555, 560, 563, 564, 565, 566, 569, 570, 571, 572, 607, 671, 672, 677};
	// print cwd
	// location B smaller set
	// std::vector<int> monitor = {391,392,394,395,396,397,398,399,400,401,455,459,460,461,462,463,464,465,466,467,468,469,470,471,472,538,539,563,564,565,566};
	
    char buffer[256];
    char *val = getcwd(buffer, sizeof(buffer));
    if (val) {
        std::cout << buffer << std::endl;
    }

	std::vector<std::string> folder_names = {"input_files_MACPOSTS_loc_A_LSC_demand_level=0.9", 
											"input_files_MACPOSTS_loc_A_LSC_demand_level=1.1",
											"input_files_MACPOSTS_loc_A_LSC_demand_level=1.2",
											"input_files_MACPOSTS_loc_A_with_incident_LSC_demand_level=1",
											"input_files_MACPOSTS_loc_B_LSC_demand_level=0.9",
											"input_files_MACPOSTS_loc_B_LSC_demand_level=1.1",
											"input_files_MACPOSTS_loc_B_LSC_demand_level=1.2",
											"input_files_MACPOSTS_loc_B_with_incident_LSC_demand_level=1"};
											

											// "input_files_MACPOSTS_loc_B_no_control_demand_level=1.0",
											// "input_files_MACPOSTS_loc_B_ALINEA_demand_level=1.0",
											// "input_files_MACPOSTS_loc_B_LSC_demand_level=1.0",
											// "input_files_MACPOSTS_loc_A_no_control_demand_level=1.0",
											// "input_files_MACPOSTS_loc_A_ALINEA_demand_level=1.0",
											// "input_files_MACPOSTS_loc_A_LSC_demand_level=1.0"
	
	// std::vector<std::string> folder_names = {"input_files_MACPOSTS_loc_A_ALINEA_demand_level=0.9", 
	// 										"input_files_MACPOSTS_loc_A_ALINEA_demand_level=1.0",
	// 										"input_files_MACPOSTS_loc_A_ALINEA_demand_level=1.1",
	// 										"input_files_MACPOSTS_loc_A_ALINEA_demand_level=1.2",
	// 										"input_files_MACPOSTS_loc_A_with_incident_ALINEA_demand_level=1",
	// 										"input_files_MACPOSTS_loc_B_ALINEA_demand_level=0.9",
	// 										"input_files_MACPOSTS_loc_B_ALINEA_demand_level=1.0",
	// 										"input_files_MACPOSTS_loc_B_ALINEA_demand_level=1.1",
	// 										"input_files_MACPOSTS_loc_B_ALINEA_demand_level=1.2",
	// 										"input_files_MACPOSTS_loc_B_with_incident_ALINEA_demand_level=1",
	// 										"input_files_MACPOSTS_loc_A_LSC_demand_level=0.9", 
	// 										"input_files_MACPOSTS_loc_A_LSC_demand_level=1.0",
	// 										"input_files_MACPOSTS_loc_A_LSC_demand_level=1.1",
	// 										"input_files_MACPOSTS_loc_A_LSC_demand_level=1.2",
	// 										"input_files_MACPOSTS_loc_A_with_incident_LSC_demand_level=1",
	// 										"input_files_MACPOSTS_loc_B_LSC_demand_level=0.9",
	// 										"input_files_MACPOSTS_loc_B_LSC_demand_level=1.0",
	// 										"input_files_MACPOSTS_loc_B_LSC_demand_level=1.1",
	// 										"input_files_MACPOSTS_loc_B_LSC_demand_level=1.2",
	// 										"input_files_MACPOSTS_loc_B_with_incident_LSC_demand_level=1"};	

	for (int k = 0; k < (int)folder_names.size(); ++k){

		printf("%d\n", k);

		std::string folder = "/srv/data/jiachao/MAC-POSTS/data/TSMO/" + folder_names[k];

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

		test_dta -> pre_loading();
		printf("========================== Finished pre_loading! ==========================\n");

		printf("========== emission link registered = %d ==============\n", (int)test_dta->m_emission->m_link_vector.size());

		TInt _current_inter = 0;
		TInt _assign_inter = test_dta -> m_start_assign_interval;
		
		bool _verbose = true; // print process or not
		
		// std::ofstream _vis_file;
		// std::string _str;
		// std::string _str_link_flow;

		// _vis_file.open(folder + "/" + rec_folder + "/link_flow_v2.txt", std::ofstream::out);
		// if (! _vis_file.is_open()){
		// 	printf("Error happens when open _vis_file\n");
		// 	exit(-1);
		// }
		// _vis_file << "timestamp,link_ID,car_link_flow,truck_link_flow,car_link_tt,truck_link_tt,car_link_ffs,truck_link_ffs\n";

		MNM_Dlink *_link_temp; 
		MNM_Dlink_Multiclass *_link_m_temp;

		while (!test_dta -> finished_loading(_current_inter) || _assign_inter < test_dta -> m_total_assign_inter){
			printf("\nCurrent loading interval: %d, Current assignment interval: %d\n", _current_inter(), _assign_inter());

			test_dta -> load_once_control(_verbose, _current_inter, _assign_inter);
			test_dta -> m_emission -> update(test_dta->m_veh_factory);

			// for (auto _link_it = test_dta -> m_link_factory -> m_link_map.begin(); _link_it != test_dta -> m_link_factory -> m_link_map.end(); _link_it++){
			// 	_link_temp = _link_it -> second;
			// 	_link_m_temp = dynamic_cast<MNM_Dlink_Multiclass*>(_link_temp);
			// 	_str_link_flow = std::to_string(int(_current_inter)) + ",";
			// 	_str_link_flow += std::to_string(_link_temp -> m_link_ID()) + ",";
			// 	_str_link_flow += std::to_string(_link_m_temp -> get_link_flow_car()) + ",";
			// 	_str_link_flow += std::to_string(_link_m_temp -> get_link_flow_truck()) + ",";
			// 	_str_link_flow += std::to_string(_link_m_temp -> get_link_tt_from_flow_car(_link_m_temp -> get_link_flow_car())) + ",";
			// 	// _str_link_flow += std::to_string(MNM_DTA_GRADIENT::get_travel_time_car(_link_m_temp, TFlt(_current_inter), TFlt(5), TInt(5040))) + ",";
			// 	_str_link_flow += std::to_string(_link_m_temp -> get_link_tt_from_flow_truck(_link_m_temp -> get_link_flow_truck())) + ",";
			// 	// _str_link_flow += std::to_string(MNM_DTA_GRADIENT::get_travel_time_truck(_link_m_temp, TFlt(_current_inter), TFlt(5), TInt(5040))) + ",";
			// 	_str_link_flow += std::to_string(_link_m_temp -> get_link_freeflow_tt_car()) + ",";
			// 	_str_link_flow += std::to_string(_link_m_temp -> get_link_freeflow_tt_truck()) + "\n";
			// 	_vis_file << _str_link_flow;
			// }

			if (_current_inter % test_dta -> m_assign_freq == 0 || _current_inter == 0){
				_assign_inter += 1;
			}
			_current_inter += 1;
			// if (_current_inter > 1800) break;
		}
		// if (_vis_file.is_open()) _vis_file.close();

		std::ofstream _vis_emission;
		_vis_emission.open(folder + "/" + rec_folder + "/emission_metrics_v2.txt", std::ofstream::out);
		if (! _vis_emission.is_open()){
			printf("Error happens when open _vis_emission\n");
			exit(-1);
		}

		std::string _str_emission;
		_str_emission = test_dta->m_emission->output_save();
		_vis_emission << _str_emission;
		if (_vis_emission.is_open()) _vis_emission.close();

		std::ofstream _vis_delay;
		std::string _str_delay;

		_vis_delay.open(folder + "/" + rec_folder + "/link_delay_v2.txt", std::ofstream::out);
		_vis_delay << "link_ID, tot_car_delay, tot_truck_delay, tot_car, tot_truck\n";

		for (auto _link_it = test_dta -> m_link_factory -> m_link_map.begin(); _link_it != test_dta -> m_link_factory -> m_link_map.end(); _link_it++){
			_link_temp = _link_it -> second;
			if (std::find(monitor.begin(), monitor.end(), _link_temp -> m_link_ID) != monitor.end()){
				_link_m_temp = dynamic_cast<MNM_Dlink_Multiclass*>(_link_temp);
				_str_delay = "";
				_str_delay += std::to_string(_link_temp -> m_link_ID()) + ",";
				_str_delay += std::to_string(_link_m_temp -> m_tot_wait_time_at_intersection_car()) + ",";
				_str_delay += std::to_string(_link_m_temp -> m_tot_wait_time_at_intersection_truck()) + ","; 
				_str_delay += std::to_string(_link_m_temp -> m_N_in_car -> m_recorder.back().second) + ",";
				_str_delay += std::to_string(_link_m_temp -> m_N_in_truck -> m_recorder.back().second) + "\n";
				_vis_delay << _str_delay;
			}
		}
		if (_vis_delay.is_open()) _vis_delay.close();

		delete test_dta;
		printf("\n\nFinished delete test_dta!\n");
	}
	return 0;
}
