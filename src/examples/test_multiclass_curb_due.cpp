
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

	std::string folder = "/srv/data/jiachao/MAC-POSTS/src/examples/MNMAPI_test/MNM_7link_new_2024/input_files_7link_multiclass_curb_dode_v999";

    printf("BEGIN multiclass DUE test!\n");

    MNM_ConfReader *config = new MNM_ConfReader(folder + "/config.conf", "STAT");
    std::string rec_folder = config -> get_string("rec_folder");

    MNM_Dta_Multiclass *test_dta;
    MNM_Due_Curb *test_due = new MNM_Due_Curb(folder);

    printf("================================ DUE set! =================================\n");

    test_due -> initialize();

    printf("========================= Finished initialization! ========================\n");

    test_due -> load_fixed_pathset(test_due -> m_base_dta);

    printf("======================== Finished init path loading =======================\n");

    std::string gap_file_name = folder + "/" + rec_folder + "/gap_iteration";
    std::ofstream gap_file;
    gap_file.open(gap_file_name, std::ofstream::out);
    if (!gap_file.is_open()){
        printf("Error happens when open gap_file\n");
        exit(-1);
    }

    TFlt gap;

    for (int i = 0; i < test_due -> m_max_iter; ++i){
        printf("---------- Iteration %d ----------\n", i);

        test_dta = test_due -> run_dta_curb(false);

        test_due -> build_cost_map(test_dta);

        gap = test_due -> compute_merit_function_fixed_departure_time_choice(test_dta);

        printf("----------  Gap = %lf  ----------\n", (float) gap);

        gap_file << std::to_string(gap) + "\n";

        test_due -> update_path_table_gp_fixed_departure_time_choice_fixed_pathset(test_dta);

        // delete test_dta;
    }

    gap_file.close();

    MNM_Origin *_orig;
	MNM_Origin_Multiclass *_orig_multiclass;

    MNM_Destination *_dest;
	MNM_Destination_Multiclass *_dest_multiclass;

    TInt _orig_node_ID, _dest_node_ID;

    std::pair<MNM_Path *, TInt> _path_result_car, _path_result_truck, _path_result_rh;
    // MNM_Path *_path_car, *_path_truck, *_path_rh;
    MNM_Pathset *_path_set_car, *_path_set_rh, *_path_set_truck;

    std::string _str;

    std::ofstream _path_buffer_file;

    std::ofstream _path_buffer_file_car;
    std::ofstream _path_buffer_file_truck;
    std::ofstream _path_buffer_file_rh;

    // _path_buffer_file.open(folder + "/" + rec_folder + "/path_buffer_final.txt", std::ofstream::out);
    // if (! _path_buffer_file.is_open()){
    //     printf("Error happens when open path_buffer_final\n");
    //     exit(-1);
    // }

    _path_buffer_file_car.open(folder + "/" + rec_folder + "/path_table_buffer", std::ofstream::out);
    if (! _path_buffer_file_car.is_open()){
        printf("Error happens when open path_table_buffer\n");
        exit(-1);
    }

    _path_buffer_file_truck.open(folder + "/" + rec_folder + "/path_table_curb_buffer", std::ofstream::out);
    if (! _path_buffer_file_truck.is_open()){
        printf("Error happens when open path_table_curb_buffer\n");
        exit(-1);
    }

    _path_buffer_file_rh.open(folder + "/" + rec_folder + "/path_table_curb_rh_buffer", std::ofstream::out);
    if (! _path_buffer_file_rh.is_open()){
        printf("Error happens when open path_table_curb_rh_buffer\n");
        exit(-1);
    }

    for (auto _it : test_dta -> m_od_factory -> m_destination_map){
		_dest = _it.second;
		_dest_multiclass = dynamic_cast<MNM_Destination_Multiclass *>(_dest);
		_dest_node_ID = _dest -> m_dest_node -> m_node_ID;

        for (auto _map_it : test_dta -> m_od_factory -> m_origin_map) {
			_orig = _map_it.second;
			_orig_multiclass = dynamic_cast<MNM_Origin_Multiclass *>(_orig);
			_orig_node_ID = _orig -> m_origin_node -> m_node_ID;

			if (_orig_multiclass -> m_demand_car.find(_dest_multiclass) == _orig_multiclass -> m_demand_car.end()){
				continue;
			}

            _path_set_car = MNM::get_pathset(test_due -> m_path_table_car, _orig_node_ID, _dest_node_ID);

			_path_set_rh = MNM::get_pathset(test_due -> m_path_table_rh, _orig_node_ID, _dest_node_ID);

			_path_set_truck = MNM::get_pathset(test_due -> m_path_table_truck, _orig_node_ID, _dest_node_ID);

            for (auto _tmp_path : _path_set_car -> m_path_vec) {
                for (int _col = 0; _col < test_due -> m_total_assign_inter; _col++) {
                    TFlt _buffer = _tmp_path -> m_buffer[_col];
                    _path_buffer_file_car << std::to_string(_buffer) + " ";
                }
                _path_buffer_file_car << "0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0\n";                
            }

            for (auto _tmp_path : _path_set_truck -> m_path_vec) {
                _path_buffer_file_truck << "0 0 0 0 0 0 0 0 0 0";
                for (int _col = 0; _col < test_due -> m_total_assign_inter; _col++) {
                    TFlt _buffer = _tmp_path -> m_buffer[_col];
                    _path_buffer_file_truck << " " + std::to_string(_buffer);
                }
                _path_buffer_file_truck << " 0 0 0 0 0 0 0 0 0 0\n";                
            }

            for (auto _tmp_path : _path_set_rh -> m_path_vec) {
                _path_buffer_file_rh << "0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0";
                for (int _col = 0; _col < test_due -> m_total_assign_inter; _col++) {
                    TFlt _buffer = _tmp_path -> m_buffer[_col];
                    _path_buffer_file_rh << " " + std::to_string(_buffer);
                }
                _path_buffer_file_rh << "\n";                
            }

            for (int _col = 0; _col < test_due -> m_total_assign_inter; _col++) {
                for (auto _tmp_path : _path_set_car -> m_path_vec) {
                    TFlt _buffer = _tmp_path -> m_buffer[_col];
                    TInt _ID = _tmp_path -> m_path_ID;
					_str = std::string("car") + " " + std::to_string(_col) + " " + std::to_string(_ID) + " " + std::to_string(_buffer) + "\n";
                    _path_buffer_file << _str;
                }

                for (auto _tmp_path : _path_set_rh -> m_path_vec) {
                    TFlt _buffer = _tmp_path -> m_buffer[_col];
                    TInt _ID = _tmp_path -> m_path_ID;
					_str = std::string("rh") + " " + std::to_string(_col) + " " + std::to_string(_ID) + " " + std::to_string(_buffer) + "\n";
                    _path_buffer_file << _str;
                }

                for (auto _tmp_path : _path_set_truck -> m_path_vec) {
                    TFlt _buffer = _tmp_path -> m_buffer[_col];
                    TInt _ID = _tmp_path -> m_path_ID;
					_str = std::string("truck") + " " + std::to_string(_col) + " " + std::to_string(_ID) + " " + std::to_string(_buffer) + "\n";
                    _path_buffer_file << _str;
                }
            }

        }
    }

    if (_path_buffer_file.is_open()) _path_buffer_file.close();

    delete config;

    delete test_due;

    printf("========================= Finished delete test_due! ======================\n");

    return 0;
}
