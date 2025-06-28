#include "io.h"
#include "multiclass.h"
#include "Snap.h"
#include <iostream>
#include <vector>

int main()
{
	std::string folder = "/srv/data/jiachao/MAC-POSTS/data/input_files_MckeesRocks_SPC_AM_Fujitsu";

	printf("BEGIN multiclass test!\n");

    MNM_ConfReader *config = new MNM_ConfReader(folder + "/config.conf", "STAT");
    std::string rec_folder = config -> get_string("rec_folder");

    MNM_Dta_Multiclass *test_dta = new MNM_Dta_Multiclass(folder);

	printf("================================ DTA set! =================================\n");
	
	test_dta -> build_from_files_control();
	printf("========================= Finished initialization! ========================\n");

	test_dta -> hook_up_node_and_link();
	printf("====================== Finished node and link hook-up! ====================\n");

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

    }

	printf("=========================== Finished install cc and trees ===========================\n");

    test_dta -> pre_loading();
    printf("========================== Finished pre_loading! ==========================\n");

	TInt _current_inter = 0;
	TInt _assign_inter = test_dta -> m_start_assign_interval;

	printf("\n\n\n====================================== Start loading! =======================================\n");
	
	bool _verbose = true;
	
    while (!test_dta -> finished_loading(_current_inter) || _assign_inter < test_dta -> m_total_assign_inter){
		printf("\nCurrent loading interval: %d, Current assignment interval: %d\n", _current_inter(), _assign_inter());

        test_dta -> load_once_control(_verbose, _current_inter, _assign_inter);

        if (_current_inter % test_dta -> m_assign_freq == 0 || _current_inter == 0){
			_assign_inter += 1;
		}
        _current_inter += 1;
    }

	std::ofstream _vis_file1;
	_vis_file1.open(folder + "/" + rec_folder + "/density.txt", std::ofstream::out);

	if (! _vis_file1.is_open()){
        printf("Error happens when open _vis_file1\n");
        exit(-1);
    }

	std::string _str_density = "link_ID timestep car_k truck_k\n";

	TInt _iter = 0;
    while (_iter < _current_inter){
        
		printf("Saving link density, current loading interval: %d\n", int(_iter));
		for (auto _link_it = test_dta -> m_link_factory -> m_link_map.begin(); _link_it != test_dta -> m_link_factory -> m_link_map.end(); _link_it++){
			_link = _link_it -> second;
			_link_multiclass = dynamic_cast<MNM_Dlink_Multiclass*>(_link);
			
			_str_density = std::to_string(_link -> m_link_ID) + " " + std::to_string(int(_iter)) + " ";
			_str_density += std::to_string(MNM_DTA_GRADIENT::get_link_density_car(_link_multiclass, TFlt(_iter), TInt(_current_inter))) + " ";
			_str_density += std::to_string(MNM_DTA_GRADIENT::get_link_density_truck(_link_multiclass, TFlt(_iter), TInt(_current_inter))) + " ";
			_str_density += std::to_string(MNM_DTA_GRADIENT::get_link_density_car_robust(_link_multiclass, TFlt(_iter), TInt(_current_inter), TInt(5))) + " ";
			_str_density += std::to_string(MNM_DTA_GRADIENT::get_link_density_truck_robust(_link_multiclass, TFlt(_iter), TInt(_current_inter), TInt(5))) + "\n";
			_vis_file1 << _str_density;

		}

        _iter += 1;
    }

    if (_vis_file1.is_open()) _vis_file1.close();

	return 0;
}