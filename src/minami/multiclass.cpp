#include "limits.h"
#include "multiclass.h"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <time.h>
#include <random>

/******************************************************************************************************************
*******************************************************************************************************************
												Link Models
*******************************************************************************************************************
******************************************************************************************************************/

MNM_Dlink_Multiclass::MNM_Dlink_Multiclass(TInt ID,
										TInt number_of_lane,
										TFlt length, // meters
										TFlt ffs_car, // Free-flow speed (m/s)
										TFlt ffs_truck)
	: MNM_Dlink::MNM_Dlink(ID, number_of_lane, length, ffs_car) // Note: although m_ffs is not used in child class, let it be ffs_car
{
	m_ffs_car = ffs_car;
	m_ffs_truck = ffs_truck;

	m_N_in_car = nullptr;
	m_N_out_car = nullptr;

	m_N_in_truck = nullptr;
	m_N_out_truck = nullptr;

	m_N_in_rh = nullptr;
	m_N_out_rh = nullptr;

	// cumulative curve for stopping vehicles
	m_N_in_car_cc = nullptr;
	m_N_out_car_cc = nullptr;

  	m_N_in_truck_cc = nullptr;
  	m_N_out_truck_cc = nullptr;

	m_N_in_rh_cc = nullptr;
	m_N_out_rh_cc = nullptr;

	// cumulative curve for all vehicles
	m_N_in_car_all = nullptr;
	m_N_out_car_all = nullptr;

	// cumulative trees for non-stopping vehicles
	m_N_in_tree_car = nullptr;
	m_N_out_tree_car = nullptr;

	m_N_in_tree_truck = nullptr;
	m_N_out_tree_truck = nullptr;

	m_N_in_tree_rh = nullptr;
	m_N_out_tree_rh = nullptr;

	// cumulative trees for stopping vehicles
	m_N_in_tree_curb_car = nullptr;
	m_N_out_tree_curb_car = nullptr;

	m_N_in_tree_curb_truck = nullptr;
	m_N_out_tree_curb_truck = nullptr;

	m_N_in_tree_curb_rh = nullptr;
	m_N_out_tree_curb_rh = nullptr;

	// average waiting time per vehicle = tot_wait_time/(tot_num_car + tot_num_truck)
	m_tot_wait_time_at_intersection = 0; // seconds
	m_tot_wait_time_at_intersection_car = 0;
	m_tot_wait_time_at_intersection_truck = 0;

	// flag of spill back on this link
	m_spill_back = false; // if spill back happens during simulation, then set to true

	install_cumulative_curve_multiclass();
}

MNM_Dlink_Multiclass::~MNM_Dlink_Multiclass()
{
	if (m_N_out_car != nullptr) delete m_N_out_car;
  	if (m_N_in_car != nullptr) delete m_N_in_car;
  	if (m_N_out_truck != nullptr) delete m_N_out_truck;
  	if (m_N_in_truck != nullptr) delete m_N_in_truck;
	if (m_N_in_rh != nullptr) delete m_N_in_rh;
	if (m_N_out_rh != nullptr) delete m_N_out_rh;

	if (m_N_out_car_cc != nullptr) delete m_N_out_car_cc;
  	if (m_N_in_car_cc != nullptr) delete m_N_in_car_cc;
  	if (m_N_out_truck_cc != nullptr) delete m_N_out_truck_cc;
  	if (m_N_in_truck_cc != nullptr) delete m_N_in_truck_cc;
	if (m_N_out_rh_cc != nullptr) delete m_N_out_rh_cc;
	if (m_N_in_rh_cc != nullptr) delete m_N_in_rh_cc;

	if (m_N_out_car_all != nullptr) delete m_N_out_car_all;
  	if (m_N_in_car_all != nullptr) delete m_N_in_car_all;

  	if (m_N_out_tree_car != nullptr) delete m_N_out_tree_car;
  	if (m_N_in_tree_car != nullptr) delete m_N_in_tree_car;
  	if (m_N_out_tree_truck != nullptr) delete m_N_out_tree_truck;
  	if (m_N_in_tree_truck != nullptr) delete m_N_in_tree_truck;
	if (m_N_in_tree_rh != nullptr) delete m_N_in_tree_rh;
	if (m_N_out_tree_rh != nullptr) delete m_N_out_tree_rh;

	// new added
	if (m_N_in_tree_curb_truck != nullptr) delete m_N_in_tree_curb_truck;
	if (m_N_out_tree_curb_truck != nullptr) delete m_N_out_tree_curb_truck;
	if (m_N_in_tree_curb_rh != nullptr) delete m_N_in_tree_curb_rh;
	if (m_N_out_tree_curb_rh != nullptr) delete m_N_out_tree_curb_rh;	

	if (m_N_in_tree_curb_car != nullptr) delete m_N_in_tree_curb_car;
	if (m_N_out_tree_curb_car != nullptr) delete m_N_out_tree_curb_car;
}

int MNM_Dlink_Multiclass::install_cumulative_curve_multiclass()
{
	if (m_N_out_car != nullptr) delete m_N_out_car;
  	if (m_N_in_car != nullptr) delete m_N_in_car;
  	if (m_N_out_truck != nullptr) delete m_N_out_truck;
  	if (m_N_in_truck != nullptr) delete m_N_in_truck;
	if (m_N_in_rh != nullptr) delete m_N_in_rh;
	if (m_N_out_rh != nullptr) delete m_N_out_rh;

	if (m_N_out_car_cc != nullptr) delete m_N_out_car_cc;
  	if (m_N_in_car_cc != nullptr) delete m_N_in_car_cc;
  	if (m_N_out_truck_cc != nullptr) delete m_N_out_truck_cc;
  	if (m_N_in_truck_cc != nullptr) delete m_N_in_truck_cc;
	if (m_N_out_rh_cc != nullptr) delete m_N_out_rh_cc;
	if (m_N_in_rh_cc != nullptr) delete m_N_in_rh_cc;

	if (m_N_out_car_all != nullptr) delete m_N_out_car_all;
  	if (m_N_in_car_all != nullptr) delete m_N_in_car_all;

	m_N_in_car = new MNM_Cumulative_Curve();
  	m_N_out_car = new MNM_Cumulative_Curve();
  	m_N_in_truck = new MNM_Cumulative_Curve();
  	m_N_out_truck = new MNM_Cumulative_Curve();
	m_N_in_rh = new MNM_Cumulative_Curve();
	m_N_out_rh = new MNM_Cumulative_Curve();

	m_N_in_car_cc = new MNM_Cumulative_Curve();
  	m_N_out_car_cc = new MNM_Cumulative_Curve();
  	m_N_in_truck_cc = new MNM_Cumulative_Curve();
  	m_N_out_truck_cc = new MNM_Cumulative_Curve();
	m_N_in_rh_cc = new MNM_Cumulative_Curve();
	m_N_out_rh_cc = new MNM_Cumulative_Curve();

	m_N_in_car_all = new MNM_Cumulative_Curve();
  	m_N_out_car_all = new MNM_Cumulative_Curve();	

  	m_N_in_car -> add_record(std::pair<TFlt, TFlt>(TFlt(0), TFlt(0)));
  	m_N_out_car -> add_record(std::pair<TFlt, TFlt>(TFlt(0), TFlt(0)));
  	m_N_in_truck -> add_record(std::pair<TFlt, TFlt>(TFlt(0), TFlt(0)));
  	m_N_out_truck -> add_record(std::pair<TFlt, TFlt>(TFlt(0), TFlt(0)));
	m_N_in_rh -> add_record(std::pair<TFlt, TFlt>(TFlt(0), TFlt(0)));
	m_N_out_rh -> add_record(std::pair<TFlt, TFlt>(TFlt(0), TFlt(0)));

	m_N_in_car_cc -> add_record(std::pair<TFlt, TFlt>(TFlt(0), TFlt(0)));
  	m_N_out_car_cc -> add_record(std::pair<TFlt, TFlt>(TFlt(0), TFlt(0)));
  	m_N_in_truck_cc -> add_record(std::pair<TFlt, TFlt>(TFlt(0), TFlt(0)));
  	m_N_out_truck_cc -> add_record(std::pair<TFlt, TFlt>(TFlt(0), TFlt(0)));
	m_N_in_rh_cc -> add_record(std::pair<TFlt, TFlt>(TFlt(0), TFlt(0)));
	m_N_out_rh_cc -> add_record(std::pair<TFlt, TFlt>(TFlt(0), TFlt(0)));

	m_N_in_car_all -> add_record(std::pair<TFlt, TFlt>(TFlt(0), TFlt(0)));
  	m_N_out_car_all -> add_record(std::pair<TFlt, TFlt>(TFlt(0), TFlt(0)));

  	return 0;
}

int MNM_Dlink_Multiclass::install_cumulative_curve_tree_multiclass()
{
	if (m_N_out_tree_car != nullptr) delete m_N_out_tree_car;
  	if (m_N_in_tree_car != nullptr) delete m_N_in_tree_car;
  	if (m_N_out_tree_truck != nullptr) delete m_N_out_tree_truck;
  	if (m_N_in_tree_truck != nullptr) delete m_N_in_tree_truck;
	if (m_N_in_tree_rh != nullptr) delete m_N_in_tree_rh;
	if (m_N_out_tree_rh != nullptr) delete m_N_out_tree_rh;

  	// !!! Close all cc_tree if only doing loading to save a lot of memory !!!
	m_N_in_tree_car = new MNM_Tree_Cumulative_Curve();
  	m_N_out_tree_car = new MNM_Tree_Cumulative_Curve();
	m_N_in_tree_truck = new MNM_Tree_Cumulative_Curve();
  	m_N_out_tree_truck = new MNM_Tree_Cumulative_Curve();
	m_N_in_tree_rh = new MNM_Tree_Cumulative_Curve();
	m_N_out_tree_rh = new MNM_Tree_Cumulative_Curve();

  	return 0;
}

// jiachao 0605
int MNM_Dlink_Multiclass::install_cumulative_curve_tree_multiclass_curb()
{
	// new added
	if (m_N_in_tree_curb_truck != nullptr) delete m_N_in_tree_curb_truck;
	if (m_N_out_tree_curb_truck != nullptr) delete m_N_out_tree_curb_truck;

	if (m_N_in_tree_curb_rh != nullptr) delete m_N_in_tree_curb_rh;
	if (m_N_out_tree_curb_rh != nullptr) delete m_N_out_tree_curb_rh;

	if (m_N_in_tree_curb_car != nullptr) delete m_N_in_tree_curb_car;
	if (m_N_out_tree_curb_car != nullptr) delete m_N_out_tree_curb_car;

	m_N_in_tree_curb_truck = new MNM_Tree_Cumulative_Curve();
	m_N_out_tree_curb_truck = new MNM_Tree_Cumulative_Curve();

	m_N_in_tree_curb_rh = new MNM_Tree_Cumulative_Curve();
	m_N_out_tree_curb_rh = new MNM_Tree_Cumulative_Curve();

	m_N_in_tree_curb_car = new MNM_Tree_Cumulative_Curve();
	m_N_out_tree_curb_car = new MNM_Tree_Cumulative_Curve();

	return 0;
}

TFlt MNM_Dlink_Multiclass::get_link_freeflow_tt_car()
{
	return m_length/m_ffs_car;  // seconds, absolute tt
}

TFlt MNM_Dlink_Multiclass::get_link_freeflow_tt_truck()
{
	return m_length/m_ffs_truck;  // seconds, absolute tt
}


/*************************************************************************					
						Multiclass CTM Functions
			(currently only for car & truck two classes)
	(see: Z. (Sean) Qian et al./Trans. Res. Part B 99 (2017) 183-204)			
**************************************************************************/
MNM_Dlink_Ctm_Multiclass::MNM_Dlink_Ctm_Multiclass(TInt ID,
												   TInt number_of_lane,
												   TFlt length, // (m)
												   TFlt lane_hold_cap_car, // Jam density (veh/m/lane)
												   TFlt lane_hold_cap_truck,
												   TFlt lane_flow_cap_car, // Max flux (veh/s/lane)
												   TFlt lane_flow_cap_truck,
												   TFlt ffs_car, // Free-flow speed (m/s)
												   TFlt ffs_truck, 
												   TFlt unit_time, // (s)
												   TFlt veh_convert_factor, // 1 * truck = c * private cars // when compute node demand
												   TFlt flow_scalar,// flow_scalar can be 2.0, 5.0, 10.0, etc.
												   TInt curb_spaces = TInt(0),
												   TInt curb_dest = TInt(-1))
	: MNM_Dlink_Multiclass::MNM_Dlink_Multiclass(ID, number_of_lane, length, ffs_car, ffs_truck)
{
	
	m_link_type = MNM_TYPE_CTM_MULTICLASS;
	// Jam density for private cars and trucks cannot be negative
	if ((lane_hold_cap_car < 0) || (lane_hold_cap_truck < 0)){
		printf("lane_hold_cap can't be negative, current link ID is %d\n", m_link_ID());
		exit(-1);
	}
	// Jam density for private cars cannot be too large
	if (lane_hold_cap_car > TFlt(400) / TFlt(1600)){
		// "lane_hold_cap is too large, set to 300 veh/mile
		lane_hold_cap_car = TFlt(400) / TFlt(1600);
	}
	// Jam density for trucks cannot be too large
	if (lane_hold_cap_truck > TFlt(400) / TFlt(1600)){
		// "lane_hold_cap is too large, set to 300 veh/mile
		lane_hold_cap_truck = TFlt(400) / TFlt(1600);
	}

	// Maximum flux for private cars and trucks cannot be negative
	if ((lane_flow_cap_car < 0) || (lane_flow_cap_truck < 0)){
		printf("lane_flow_cap can't be less than zero, current link ID is %d\n", m_link_ID());
		exit(-1);
	}
	// Maximum flux for private cars cannot be too large
	if (lane_flow_cap_car > TFlt(3500) / TFlt(3600)){
		// lane_flow_cap is too large, set to 3500 veh/hour
		lane_flow_cap_car = TFlt(3500) / TFlt(3600);
	}
	// Maximum flux for trucks cannot be too large
	if (lane_flow_cap_truck > TFlt(3500) / TFlt(3600)){
		// lane_flow_cap is too large, set to 3500 veh/hour
		lane_flow_cap_truck = TFlt(3500) / TFlt(3600);
	}

	if ((ffs_car < 0) || (ffs_truck < 0)){
		printf("free-flow speed can't be less than zero, current link ID is %d\n", m_link_ID());
		exit(-1);
	}

	if (veh_convert_factor < 1){
		printf("veh_convert_factor can't be less than 1, current link ID is %d\n", m_link_ID());
		exit(-1);
	}

	if (flow_scalar < 1){
		printf("flow_scalar can't be less than 1, current link ID is %d\n", m_link_ID());
		exit(-1);
	}

	if (unit_time <= 0){
		printf("unit_time should be positive, current link ID is %d\n", m_link_ID());
		exit(-1);
	}
	m_unit_time = unit_time;
	m_lane_flow_cap_car = lane_flow_cap_car;
	m_lane_flow_cap_truck = lane_flow_cap_truck;	
	m_lane_hold_cap_car = lane_hold_cap_car;
	m_lane_hold_cap_truck = lane_hold_cap_truck;
	m_veh_convert_factor = veh_convert_factor;
	m_flow_scalar = flow_scalar;
	m_curb_spaces = curb_spaces;
	m_curb_dest = curb_dest;


	m_cell_array = std::vector<Ctm_Cell_Multiclass*>();

	// Note m_ffs_car > m_ffs_truck, use ffs_car to define the standard cell length
	TFlt _std_cell_length = m_ffs_car * unit_time;
	m_num_cells = TInt(floor(m_length / _std_cell_length));
	if (m_num_cells == 0){
		m_num_cells = 1;
		m_length = _std_cell_length;
	}
	TFlt _last_cell_length = m_length - TFlt(m_num_cells - 1) * _std_cell_length;

	m_lane_critical_density_car = m_lane_flow_cap_car / m_ffs_car;
	m_lane_critical_density_truck = m_lane_flow_cap_truck / m_ffs_truck;

	if (m_lane_hold_cap_car <= m_lane_critical_density_car){
		printf("Wrong private car parameters, current link ID is %d\n", m_link_ID());
		exit(-1);
	}
	m_wave_speed_car = m_lane_flow_cap_car / (m_lane_hold_cap_car - m_lane_critical_density_car);

	if (m_lane_hold_cap_truck <= m_lane_critical_density_truck){
		printf("Wrong truck parameters, current link ID is %d\n", m_link_ID());
		exit(-1);
	}
	m_wave_speed_truck = m_lane_flow_cap_truck / (m_lane_hold_cap_truck - m_lane_critical_density_truck);

	// see the reference paper for definition
	// m_lane_rho_1_N > m_lane_critical_density_car and m_lane_critical_density_truck
	m_lane_rho_1_N = m_lane_hold_cap_car * (m_wave_speed_car / (m_ffs_truck + m_wave_speed_car));

	init_cell_array(unit_time, _std_cell_length, _last_cell_length);

	m_curb_cell_array = std::vector<Cell_Curb_Multiclass*>();

	install_curb();

	// PMC
	m_congested_car = int(0);
	m_congested_truck = int(0);

	m_diff_car = true;
	m_diff_truck = true;

}


MNM_Dlink_Ctm_Multiclass::~MNM_Dlink_Ctm_Multiclass()
{
	for (Ctm_Cell_Multiclass* _cell : m_cell_array){
		delete _cell;
	}
	m_cell_array.clear();
	// Jiachao added in Sep
	for (Cell_Curb_Multiclass* _cell_curb: m_curb_cell_array){
		delete _cell_curb;
	}
	m_curb_cell_array.clear();
}


// Curb-Jiachao added ---- Curb cells
int MNM_Dlink_Ctm_Multiclass::install_curb()
{
	// calculate ave cap for each previous cell
	TInt ave_cap = TInt(m_curb_spaces)/TInt(m_num_cells);

	// calculate how many is left for the last curb cell
	TInt last_cap = m_curb_spaces - ave_cap * (m_num_cells - 1);

	TInt dest_ID = this -> m_curb_dest;

	TInt link_ID = this -> m_link_ID;

	// TODO needs a input file to indicate the curb space belongs to which destination
	// All previous cells
	Cell_Curb_Multiclass *curb_cell = NULL;
	for (int i = 0; i < m_num_cells; ++i){
		// TInt cell_ID, TInt link_ID,TInt dest_ID,TInt cell_capacity,TFlt unit_time
		if (i != (m_num_cells - 1)){
			curb_cell = new Cell_Curb_Multiclass(TInt(i), // cell_ID, from 0 to (m_num_cells - 1)
                                       		 TInt(link_ID), // link_ID: current link ID
											 TInt(dest_ID), // TODO dest_ID, from input file
											 TInt(ave_cap), // cell_capacity
									         TFlt(m_unit_time));
			if (curb_cell == NULL) {
				printf("Fail to install curb cell before last.\n");
				exit(-1);
			}
		}
		if (i == (m_num_cells - 1)){
			curb_cell = new Cell_Curb_Multiclass(TInt(i), // cell_ID, from 0 to (m_num_cells - 1)
                                       		 TInt(link_ID), // link_ID: current link ID
											 TInt(dest_ID), // dest_ID, from input file
											 TInt(last_cap), // cell_capacity
									         TFlt(m_unit_time));
			if (curb_cell == NULL) {
				printf("Fail to install the last curb cell.\n");
				exit(-1);
			}
		}
		
		
		m_curb_cell_array.push_back(curb_cell);
	}
	// compress the cell_array to reduce space
	m_curb_cell_array.shrink_to_fit();
	return 0;
}

// Jiachao added in Sep
int MNM_Dlink_Ctm_Multiclass::move_veh_queue_curb_biclass(std::deque<MNM_Veh*> *from_queue_car, // m_cell_array[i] -> m_veh_queue_car
															std::deque<MNM_Veh*> *from_queue_truck, // m_cell_array[i] -> m_veh_queue_truck
															std::deque<MNM_Veh*> *from_queue_curb,// m_curb_cell_array[i] -> m_veh_departing
															std::deque<MNM_Veh*> *to_queue_car, // m_cell_array[i+1] -> m_veh_queue_car
															std::deque<MNM_Veh*> *to_queue_truck, // m_cell_array[i+1] -> m_veh_queue_truck
															std::deque<MNM_Veh*> *to_queue_curb, // m_curb_cell_array[i] -> m_veh_arriving
															std::deque<MNM_Veh*> *to_queue_car_dp, // m_curb_cell_array[i] -> m_veh_doubleparking_car
															std::deque<MNM_Veh*> *to_queue_truck_dp, // m_curb_cell_array[i] -> m_veh_doubleparking_truck
															std::deque<MNM_Veh*> *to_queue_rh_dp, // m_curb_cell_array[i] -> m_veh_doubleparking_rh
															TInt number_tomove_car,
															TInt number_tomove_truck,
															TInt cell_ID,
															TInt timestamp)
{
	MNM_Veh* _veh;
	MNM_Veh_Multiclass* _veh_multiclass;
	
	for (int i = 0; i < number_tomove_car; ++i) {
		_veh = from_queue_car -> front();
		from_queue_car -> pop_front();

		_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass *>(_veh);

		// check if _veh_multiclass needs parking, if yes added into curb_arriving_queue
		/* TODO check if veh's interdest front item is this link's end node */
		if (_veh_multiclass -> m_curb_destination_list.front() == this -> m_link_ID){
			to_queue_curb -> push_back(_veh); // now the arriving is not real arriving, it is a buffer
		}
		// else if (_veh_multiclass -> m_destination_list.front() == this -> m_to_node -> m_node_ID){
		// 	to_queue_curb -> push_back(_veh); // now the arriving is not real arriving, it is a buffer
		// 	_veh_multiclass -> m_curb_destination_list.push_back(this -> m_link_ID);
		// }
		else{
			// if no, add vehs in next array to_queue and update the vehicle position on current link. 0: at the beginning, 1: at the end.
			_veh_multiclass -> m_visual_position_on_link += float(1)/float(m_num_cells);

			if (_veh_multiclass -> m_visual_position_on_link > 0.99){
				_veh_multiclass -> m_visual_position_on_link = 0.99;
			} 
			// add it to next link cell
			to_queue_car -> push_back(_veh);
		}
	}

	for (int j = 0; j < number_tomove_truck; ++j) {
		_veh = from_queue_truck -> front();
		from_queue_truck -> pop_front();

		_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass *>(_veh);

		// check if _veh_multiclass needs parking, if yes added into curb_arriving_queue
		if (_veh_multiclass -> m_curb_destination_list.front() == this -> m_link_ID){
			to_queue_curb -> push_back(_veh); // now the arriving is not real arriving, it is a buffer
		}
		// else if (_veh_multiclass -> m_destination_list.front() == this -> m_to_node -> m_node_ID){
		// 	to_queue_curb -> push_back(_veh); // now the arriving is not real arriving, it is a buffer
		// 	_veh_multiclass -> m_curb_destination_list.push_back(this -> m_link_ID);
		// }
		else{
			// if no, add vehs in next array to_queue and update the vehicle position on current link. 0: at the beginning, 1: at the end.
			_veh_multiclass -> m_visual_position_on_link += float(1)/float(m_num_cells);

			if (_veh_multiclass -> m_visual_position_on_link > 0.99){
				_veh_multiclass -> m_visual_position_on_link = 0.99;
			} 
			// add it to next link cell
			to_queue_truck -> push_back(_veh);
		}
	}

	// check curb cell capacity
	// one truck account for 2 cars
	int _num_available = int(m_curb_cell_array[cell_ID] -> m_cell_capacity) - (int(m_curb_cell_array[cell_ID] -> m_veh_parking_car.size()) \
																				+ int(m_curb_cell_array[cell_ID] -> m_veh_parking_truck.size()) * 2 \
																				+ int(m_curb_cell_array[cell_ID] -> m_veh_parking_rh.size()));

	int _num_available_dp = int(m_curb_cell_array[cell_ID] -> m_cell_capacity) - (int(m_curb_cell_array[cell_ID] -> m_veh_doubleparking_truck.size()) * 2 \
																				+ int(m_curb_cell_array[cell_ID] -> m_veh_doubleparking_rh.size()));

	int _num_available_next = int(0);

	for (int a = cell_ID +1; a < m_num_cells - 1; ++a){
		_num_available_next += (m_curb_cell_array[a] -> m_cell_capacity - (int(m_curb_cell_array[a] -> m_veh_parking_car.size()) \
																		 + int(m_curb_cell_array[a] -> m_veh_parking_truck.size()) * 2 \
																		 + int(m_curb_cell_array[a] -> m_veh_parking_rh.size())));
	}

	// shuffle to make sure fairness for car and truck to park
	random_shuffle(to_queue_curb -> begin(), to_queue_curb -> end());

	// now check how many vehs can really arrive
	int check_num = int(to_queue_curb -> size());

	for (int k = 0; k < check_num; ++k){
		_veh = to_queue_curb -> front();
		to_queue_curb -> pop_front();

		_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass *>(_veh);

		// for car and rh 
		if ((_veh_multiclass -> m_class == 0) || (_veh_multiclass -> m_class == 2)){
			if (_num_available >= 1){
				to_queue_curb -> push_back(_veh);
				_num_available -= 1;
			}
			else if ((_num_available_dp >= 1) && (_veh_multiclass -> m_parking_location * m_num_cells < (cell_ID +1)) && (_num_available_next < 1)){ //put into double parking
				
				// no double parking for driving
				if (_veh_multiclass -> m_class == 0){
					// _veh_multiclass -> m_arrival_time_list.push_back(timestamp);
					// to_queue_car_dp->push_back(_veh);
					_veh_multiclass -> m_visual_position_on_link += float(1)/float(m_num_cells);

					if (_veh_multiclass -> m_visual_position_on_link > 0.99){
						_veh_multiclass -> m_visual_position_on_link = 0.99;
					} 
					to_queue_car->push_back(_veh);
					// printf("Double parking occurs\n");
				}
				else {
					_veh_multiclass -> m_arrival_time_list.push_back(timestamp);
					to_queue_rh_dp->push_back(_veh);
					_num_available_dp -= 1;
					// printf("Double parking occurs\n");
				}
			}
			else { // back to next cell queue
				_veh_multiclass -> m_visual_position_on_link += float(1)/float(m_num_cells);

				if (_veh_multiclass -> m_visual_position_on_link > 0.99){
					_veh_multiclass -> m_visual_position_on_link = 0.99;
				} 
				to_queue_car->push_back(_veh);
			}
		}

		// for truck
		if (_veh_multiclass -> m_class == 1){
			if (_num_available >= 2){
				to_queue_curb -> push_back(_veh);
				_num_available -= 2;
			}
			else if ((_num_available_dp >= 2) && (_veh_multiclass -> m_parking_location * m_num_cells < (cell_ID+1)) && (_num_available_next < 1)){
				_veh_multiclass -> m_arrival_time_list.push_back(timestamp);
				to_queue_truck_dp -> push_back(_veh);
				// printf("Double parking occurs\n");
				_num_available_dp -= 2;
			}
			else{
				_veh_multiclass -> m_visual_position_on_link += float(1)/float(m_num_cells);

				if (_veh_multiclass -> m_visual_position_on_link > 0.99){
					_veh_multiclass -> m_visual_position_on_link = 0.99;
				}
				to_queue_truck->push_back(_veh);
			}
		}
	}
	// now the to_queue_curb are all real arrivals

	//  add departure vehs into next queue array
	int _from_queue_curb_size = int(from_queue_curb -> size());
	for (int h = 0; h < _from_queue_curb_size; ++h){
		_veh = from_queue_curb -> front();
		from_queue_curb -> pop_front();

		_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass *>(_veh);
		_veh_multiclass -> m_visual_position_on_link += float(1)/float(m_num_cells);
		_veh_multiclass -> m_curb_destination_list.pop_front();

		if ((_veh_multiclass -> m_class == 0) || (_veh_multiclass -> m_class == 2)){
			to_queue_car -> push_back(_veh);
		}

		if (_veh_multiclass -> m_class == 1){
			to_queue_truck -> push_back(_veh);
		}
	}
	return 0;
}

// int MNM_Dlink_Ctm_Multiclass::move_veh_queue_curb(std::deque<MNM_Veh*> *from_queue, // current cell: m_cell_array[i] -> m_veh_queue_car/truck
// 										std::deque<MNM_Veh*> *from_queue_curb,// departing vehs: m_curb_cell_array[i] -> m_veh_departing_car/truck
//                                 		std::deque<MNM_Veh*> *to_queue, // next_cell: m_cell_array[i+1]
// 										std::deque<MNM_Veh*> *to_queue_curb, // arriving vehs: m_curb_cell_array[i] -> m_veh_arriving_car/truck
//                                 		TInt number_tomove, // number to move from cell i to cell i+1
// 										TInt link_ID)
// {
// 	// MNM_Veh* _veh;
// 	// MNM_Veh_Multiclass* _veh_multiclass;

// 	// // check curb cell capacity
// 	// int _num_available = (m_curb_cell_array[link_ID] -> m_cell_capacity) - (m_curb_cell_array[link_ID] -> m_veh_parking_car.size()
// 	// 																		+ m_curb_cell_array[link_ID] -> m_veh_parking_truck.size()
// 	// 																		+ m_curb_cell_array[link_ID] -> m_veh_parking_rh.size());

// 	// for (int i = 0; i < number_tomove; ++i) {
// 	// 	_veh = from_queue -> front();
// 	// 	from_queue -> pop_front();
// 	// 	_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass *>(_veh);

// 	// 	// check if _veh_multiclass needs parking, if yes added into curb_arrving_queue
// 	// 	if (_veh_multiclass -> m_curb_destination_list.front() == link_ID)
// 	// 	{
// 	// 		// TODO: check if this curb cell is full
// 	// 		// if not full
// 	// 		if (_num_available > 0){
// 	// 			to_queue_curb -> push_back(_veh);
// 	// 			_num_available -= 1;
// 	// 		}
// 	// 		else{ // if full
// 	// 			to_queue -> push_back(_veh);
// 	// 		}
// 	// 	}
// 	// 	else{
// 	// 		// if no, add vehs in next array to_queue and update the vehicle position on current link. 0: at the beginning, 1: at the end.
// 	// 		_veh_multiclass -> m_visual_position_on_link += float(1)/float(m_num_cells);
// 	// 		if (_veh_multiclass -> m_visual_position_on_link > 0.99){
// 	// 			_veh_multiclass -> m_visual_position_on_link = 0.99;
// 	// 		} 
// 	// 		to_queue -> push_back(_veh);
// 	// 	}
// 	// }
	
// 	// //  add vehs in next queue array
// 	// for (int j = 0; j < int(from_queue_curb -> size()); ++j){
// 	// 	_veh = from_queue_curb -> front();
// 	// 	from_queue_curb -> pop_front();

// 	// 	_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass *>(_veh);
// 	// 	_veh_multiclass -> m_visual_position_on_link += float(1)/float(m_num_cells);

// 	// 	_veh_multiclass -> m_curb_destination_list.pop_front();

// 	// 	to_queue -> push_back(_veh);
// 	// }
// 	return 0;
// }

int MNM_Dlink_Ctm_Multiclass::move_veh_queue(std::deque<MNM_Veh*> *from_queue,
                                		std::deque<MNM_Veh*> *to_queue, 
                                		TInt number_tomove)
{
	MNM_Veh* _veh;
	MNM_Veh_Multiclass* _veh_multiclass;
	for (int i = 0; i < number_tomove; ++i) {
		_veh = from_queue -> front();
		from_queue -> pop_front();
		_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass *>(_veh);
		// update the vehicle position on current link. 0: at the beginning, 1: at the end.
		_veh_multiclass -> m_visual_position_on_link += float(1)/float(m_num_cells);
		if (_veh_multiclass -> m_visual_position_on_link > 0.99) 
			_veh_multiclass -> m_visual_position_on_link = 0.99;
		to_queue -> push_back(_veh);
	}
	return 0;
}

int MNM_Dlink_Ctm_Multiclass::init_cell_array(TFlt unit_time, 
											  TFlt std_cell_length, 
											  TFlt last_cell_length)
{
	// All previous cells
	Ctm_Cell_Multiclass *cell = NULL;
	for (int i = 0; i < m_num_cells - 1; ++i){
		cell = new Ctm_Cell_Multiclass(TInt(i),
                                       std_cell_length,
									   unit_time,
									   // Convert lane parameters to cell (link) parameters by multiplying # of lanes
									   TFlt(m_number_of_lane) * m_lane_hold_cap_car,
									   TFlt(m_number_of_lane) * m_lane_hold_cap_truck,
									   TFlt(m_number_of_lane) * m_lane_critical_density_car,
									   TFlt(m_number_of_lane) * m_lane_critical_density_truck,
									   TFlt(m_number_of_lane) * m_lane_rho_1_N,
									   TFlt(m_number_of_lane) * m_lane_flow_cap_car,
									   TFlt(m_number_of_lane) * m_lane_flow_cap_truck,
									   m_ffs_car,
									   m_ffs_truck,
									   m_wave_speed_car,
									   m_wave_speed_truck,
									   m_flow_scalar);
		if (cell == NULL) {
			printf("Fail to initialize some standard cell.\n");
			exit(-1);
		}
		m_cell_array.push_back(cell);
	}

	// The last cell
	// last cell must exist as long as link length > 0, see definition above
	if (m_length > 0.0) {
		cell = new Ctm_Cell_Multiclass(m_num_cells - 1,
		                               last_cell_length, // Note last cell length is longer but < 2X
									   unit_time,
									   TFlt(m_number_of_lane) * m_lane_hold_cap_car,
									   TFlt(m_number_of_lane) * m_lane_hold_cap_truck,
									   TFlt(m_number_of_lane) * m_lane_critical_density_car,
									   TFlt(m_number_of_lane) * m_lane_critical_density_truck,
									   TFlt(m_number_of_lane) * m_lane_rho_1_N,
									   TFlt(m_number_of_lane) * m_lane_flow_cap_car,
									   TFlt(m_number_of_lane) * m_lane_flow_cap_truck,
									   m_ffs_car,
									   m_ffs_truck,
									   m_wave_speed_car,
									   m_wave_speed_truck,
									   m_flow_scalar);
		if (cell == NULL) {
			printf("Fail to initialize the last cell.\n");
			exit(-1);
		}
		m_cell_array.push_back(cell);
	}

	// compress the cell_array to reduce space
	m_cell_array.shrink_to_fit();

	return 0;
}

void MNM_Dlink_Ctm_Multiclass::print_info()
{
	printf("Total number of cell: \t%d\n Flow scalar: \t%.4f\n", int(m_num_cells), double(m_flow_scalar));

	printf("Car volume for each cell is:\n");
	for (int i = 0; i < m_num_cells - 1; ++i){
		printf("%d, ", int(m_cell_array[i] -> m_volume_car));
	} 
	printf("%d\n", int(m_cell_array[m_num_cells - 1] -> m_volume_car));

	printf("Truck volume for each cell is:\n");
	for (int i = 0; i < m_num_cells - 1; ++i){
		printf("%d, ", int(m_cell_array[i] -> m_volume_truck));
	}
	printf("%d\n", int(m_cell_array[m_num_cells - 1] -> m_volume_truck));

	printf("ffs car for each cell is:\n");
	for (int i = 0; i < m_num_cells - 1; ++i){
		printf("%d, ", int(m_cell_array[i] -> m_ffs_car));
	}
	printf("%d\n", int(m_cell_array[m_num_cells - 1] -> m_ffs_car));

	printf("ffs truck for each cell is:\n");
	for (int i = 0; i < m_num_cells - 1; ++i){
		printf("%d, ", int(m_cell_array[i] -> m_ffs_truck));
	}
	printf("%d\n", int(m_cell_array[m_num_cells - 1] -> m_ffs_truck));	

}

/* TODO change perceived demand and supply by Jiachao */
/* this should be a function of timestamp and change by lane_closure setting */
/* TODO with ramp metering */
int MNM_Dlink_Ctm_Multiclass::update_out_veh_control(TFlt _ratio_lane_closure, TInt _control_cell_ID, TFlt _control_cell_rate)
{
	TFlt _temp_out_flux_car, _supply_car, _demand_car, _controled_cap_car;
	TFlt _temp_out_flux_truck, _supply_truck, _demand_truck, _controled_cap_truck;

	if (m_num_cells > 1){
		for (int i = 0; i < m_num_cells - 1; ++i){
			// car, veh_type = TInt(0)
			_demand_car = m_cell_array[i] -> get_perceived_demand(TInt(0), _ratio_lane_closure);
			_supply_car = m_cell_array[i + 1] -> get_perceived_supply(TInt(0), _ratio_lane_closure);

			_demand_truck = m_cell_array[i] -> get_perceived_demand(TInt(1), _ratio_lane_closure);
			_supply_truck = m_cell_array[i + 1] -> get_perceived_supply(TInt(1), _ratio_lane_closure);

			if (i == (int)_control_cell_ID){
				_controled_cap_car = (m_cell_array[i] -> m_flow_cap_car * m_unit_time) * _control_cell_rate;
				_controled_cap_truck = (m_cell_array[i] -> m_flow_cap_truck * m_unit_time) * _control_cell_rate;

				_temp_out_flux_car = m_cell_array[i] -> m_space_fraction_car * MNM_Ults::min(MNM_Ults::min(_demand_car, _supply_car), _controled_cap_car);

				// TODO control method -- alinea
				// next link ID -- first cell -- vehicle number
				// current link vehicle number
				// storing flow rates for previous intervals

				_temp_out_flux_truck = m_cell_array[i] -> m_space_fraction_truck * MNM_Ults::min(MNM_Ults::min(_demand_truck, _supply_truck), _controled_cap_truck);

			}
			else{
				_temp_out_flux_car = m_cell_array[i] -> m_space_fraction_car * MNM_Ults::min(_demand_car, _supply_car);
				_temp_out_flux_truck = m_cell_array[i] -> m_space_fraction_truck * MNM_Ults::min(_demand_truck, _supply_truck);
			}
			
			m_cell_array[i] -> m_out_veh_car = MNM_Ults::round(_temp_out_flux_car * m_flow_scalar);
			m_cell_array[i] -> m_out_veh_truck = MNM_Ults::round(_temp_out_flux_truck * m_flow_scalar);

		}
	}
	m_cell_array[m_num_cells - 1] -> m_out_veh_car = m_cell_array[m_num_cells - 1] -> m_veh_queue_car.size(); // what is the m_veh_queue_car??? why use all m_veh_queue_car??
	m_cell_array[m_num_cells - 1] -> m_out_veh_truck = m_cell_array[m_num_cells - 1] -> m_veh_queue_truck.size();
	return 0;
}

// Curb-Jiachao added in Aug 29
int MNM_Dlink_Ctm_Multiclass::update_out_veh_curb(std::vector<TFlt> _ratio_lane_closure_list)
{
	TFlt _temp_out_flux_car, _supply_car, _demand_car;
	TFlt _temp_out_flux_truck, _supply_truck, _demand_truck;

	if ((m_num_cells > 1) && (int(_ratio_lane_closure_list.size()) == m_num_cells)){
		for (int i = 0; i < m_num_cells - 1; ++i){
			// car, veh_type = TInt(0)
			TFlt _ratio_lane_closure = _ratio_lane_closure_list[i];
			TFlt _ratio_lane_closure_next = _ratio_lane_closure_list[i + 1];

			_demand_car = m_cell_array[i] -> get_perceived_demand(TInt(0), _ratio_lane_closure);
			_supply_car = m_cell_array[i + 1] -> get_perceived_supply(TInt(0), _ratio_lane_closure_next);
			_temp_out_flux_car = m_cell_array[i] -> m_space_fraction_car * MNM_Ults::min(_demand_car, _supply_car);
			m_cell_array[i] -> m_out_veh_car = MNM_Ults::round(_temp_out_flux_car * m_flow_scalar);

			// truck, veh_type = TInt(1)
			_demand_truck = m_cell_array[i] -> get_perceived_demand(TInt(1), _ratio_lane_closure);
			_supply_truck = m_cell_array[i + 1] -> get_perceived_supply(TInt(1), _ratio_lane_closure_next);
			_temp_out_flux_truck = m_cell_array[i] -> m_space_fraction_truck * MNM_Ults::min(_demand_truck, _supply_truck);
			m_cell_array[i] -> m_out_veh_truck = MNM_Ults::round(_temp_out_flux_truck * m_flow_scalar);

		}
	}
	m_cell_array[m_num_cells - 1] -> m_out_veh_car = m_cell_array[m_num_cells - 1] -> m_veh_queue_car.size();
	m_cell_array[m_num_cells - 1] -> m_out_veh_truck = m_cell_array[m_num_cells - 1] -> m_veh_queue_truck.size();

	return 0;
}

int MNM_Dlink_Ctm_Multiclass::update_out_veh(TFlt _ratio_lane_closure)
{
	TFlt _temp_out_flux_car, _supply_car, _demand_car;
	TFlt _temp_out_flux_truck, _supply_truck, _demand_truck;

	// TInt _output_link = 16815;
	// if (m_link_ID == _output_link)
	// 	printf("Link %d to be moved: ", int(m_link_ID));
	// no update is needed if only one cell
	if (m_num_cells > 1){
		for (int i = 0; i < m_num_cells - 1; ++i){
			// car, veh_type = TInt(0)
			_demand_car = m_cell_array[i] -> get_perceived_demand(TInt(0), _ratio_lane_closure);
			_supply_car = m_cell_array[i + 1] -> get_perceived_supply(TInt(0), _ratio_lane_closure);

			// formula: q_m = alpha_m * min(D_m^l, S_m^r)
			_temp_out_flux_car = m_cell_array[i] -> m_space_fraction_car * MNM_Ults::min(_demand_car, _supply_car);
			m_cell_array[i] -> m_out_veh_car = MNM_Ults::round(_temp_out_flux_car * m_flow_scalar);

			// m_cell_array[i]-> m_flow_cap_car
			// m_cell_array[i] -> m_unit_time
			// m_unit_time

			// truck, veh_type = TInt(1)
			_demand_truck = m_cell_array[i] -> get_perceived_demand(TInt(1), _ratio_lane_closure);
			_supply_truck = m_cell_array[i + 1] -> get_perceived_supply(TInt(1), _ratio_lane_closure);
			_temp_out_flux_truck = m_cell_array[i] -> m_space_fraction_truck * MNM_Ults::min(_demand_truck, _supply_truck);
			m_cell_array[i] -> m_out_veh_truck = MNM_Ults::round(_temp_out_flux_truck * m_flow_scalar);

		}
	}
	m_cell_array[m_num_cells - 1] -> m_out_veh_car = m_cell_array[m_num_cells - 1] -> m_veh_queue_car.size(); // what is the m_veh_queue_car??? why use all m_veh_queue_car??
	m_cell_array[m_num_cells - 1] -> m_out_veh_truck = m_cell_array[m_num_cells - 1] -> m_veh_queue_truck.size();
	// if (m_link_ID == _output_link) printf("(%d, %d)\n", int(m_cell_array[m_num_cells - 1] -> m_out_veh_car), int(m_cell_array[m_num_cells - 1] -> m_out_veh_truck));
	return 0;
}

// Jiachao added 
int MNM_Dlink_Ctm_Multiclass::evolve_curb(TInt timestamp)
{
	std::deque<MNM_Veh*>::iterator _veh_it;
	TInt _count_car = 0; // car number
	TInt _count_truck = 0; // truck number 
	TInt _count_tot_vehs = 0; // total number

	/* update volume */
	/* TODO with timestamp */
	// if (_ratio_lane_closure == 0.5){
	// 	printf("=== %d ===\n", timestamp);
	// }

	// update_out_veh(_ratio_lane_closure);
	// Curb-Jiachao added in Aug 29
	std::vector<TFlt> _ratio_lane_closure_list;

	// change the ratio
	// two params: 0.2 for normal parking and 0.5 for double parking
	for (int i = 0; i < m_num_cells; ++i)
	{
		TInt _num_lane = this -> m_number_of_lane;
		TFlt _normal_parking_factor, _double_parking_factor, _total_factor;

		_normal_parking_factor = 0.0 * TFlt(m_curb_cell_array[i] -> m_veh_parking_num) / TFlt(m_curb_cell_array[i] -> m_cell_capacity); // 0.2 * 

		_double_parking_factor = 0.0 * TFlt(m_curb_cell_array[i] -> m_veh_doubleparking_num) / TFlt(m_curb_cell_array[i] -> m_cell_capacity); // 0.5 * 

		_total_factor = _normal_parking_factor + _double_parking_factor;

		if (_total_factor > 1.0){
			_total_factor = 1.0;
		}

		TFlt _num_lane_left = TFlt(this -> m_number_of_lane) - _total_factor;

		if (_num_lane_left < 0){
			_num_lane_left = 0;
		}

		_ratio_lane_closure_list.push_back(_num_lane_left/_num_lane);
	}

	update_out_veh_curb(_ratio_lane_closure_list);

	// printf("=== update_out_veh_curb done ===\n");

	TInt _num_veh_tomove_car, _num_veh_tomove_truck;
	/* previous cells */
	if (m_num_cells > 1){
		for (int i = 0; i < m_num_cells - 1; ++i)
		{

			// step 1: check how many vehicles are departing from curb cell, update departing queue
			m_curb_cell_array[i] -> move_veh_out_curb(timestamp);
													// &m_curb_cell_array[i]->m_veh_parking_car,
													// &m_curb_cell_array[i]->m_veh_parking_truck,
													// &m_curb_cell_array[i]->m_veh_parking_rh,
													// &m_curb_cell_array[i]->m_veh_departing);
			// step 2: moving vehs from link cell i to link cell i+1
			// check how many vehs are arriving, if space available, add them to arriving queue
			// moving departing vehs to next link cell i+1

			// printf("=== move_veh_out_curb done cell id %d===\n", i);
			// Car
			_num_veh_tomove_car = m_cell_array[i] -> m_out_veh_car;
			_num_veh_tomove_truck = m_cell_array[i] -> m_out_veh_truck;

			move_veh_queue_curb_biclass(&(m_cell_array[i]->m_veh_queue_car),
										&(m_cell_array[i] -> m_veh_queue_truck),
										&(m_curb_cell_array[i] -> m_veh_departing),
										&(m_cell_array[i+1] -> m_veh_queue_car),
										&(m_cell_array[i+1] -> m_veh_queue_truck),
										&(m_curb_cell_array[i] -> m_veh_arriving),
										&(m_curb_cell_array[i] -> m_veh_doubleparking_car),
										&(m_curb_cell_array[i] -> m_veh_doubleparking_truck),
										&(m_curb_cell_array[i] -> m_veh_doubleparking_rh),
										_num_veh_tomove_car,
										_num_veh_tomove_truck,
										i,
										timestamp);

			// step 3: moving vehs from arriving to real curb parking queues
			// printf("=== move_veh_queue_curb_biclass done ===\n");

			m_curb_cell_array[i] -> move_veh_in_curb(timestamp);
													// &m_curb_cell_array[i]->m_veh_arriving,
													// &m_curb_cell_array[i]->m_veh_parking_car,
													// &m_curb_cell_array[i]->m_veh_parking_truck,
													// &m_curb_cell_array[i]->m_veh_parking_rh);
			
			// printf("=== move_veh_in_curb done ===\n");

			// after 3 steps: arriving and departing queue should be empty, only veh_parking represents parking vehs
			m_curb_cell_array[i] -> m_veh_parking_num = int(m_curb_cell_array[i] -> m_veh_parking_car.size()) + 
														int(m_curb_cell_array[i] -> m_veh_parking_rh.size()) +
														2 * int(m_curb_cell_array[i] -> m_veh_parking_truck.size());

			// TODO add double parking
			m_curb_cell_array[i] -> m_veh_doubleparking_num = int(m_curb_cell_array[i] -> m_veh_doubleparking_rh.size()) +
														2 * int(m_curb_cell_array[i] -> m_veh_doubleparking_truck.size());

		}
	}

	// printf("=== move previous cell done ===\n");
	/* last cell */

	move_last_cell_curb(timestamp);

	// printf("=== move last cell done ===\n");

	m_tot_wait_time_at_intersection += TFlt(m_finished_array.size())/m_flow_scalar * m_unit_time;
	// separate car and truck
	for (auto _veh_it = m_finished_array.begin(); _veh_it != m_finished_array.end(); _veh_it++){
		MNM_Veh_Multiclass *_veh = dynamic_cast<MNM_Veh_Multiclass *>(*_veh_it);
		if (_veh -> m_class == 0) m_tot_wait_time_at_intersection_car += TFlt(1)/m_flow_scalar * m_unit_time;
		if (_veh -> m_class == 1) m_tot_wait_time_at_intersection_truck += TFlt(1)/m_flow_scalar * m_unit_time;
	}

	// (TODO) update _ratio_lane_closure_list
	for (int i = 0; i < m_num_cells; ++i)
	{
		TInt _num_lane = this -> m_number_of_lane;
		TFlt _normal_parking_factor, _double_parking_factor, _total_factor;

		_normal_parking_factor = 0.0 * TFlt(m_curb_cell_array[i] -> m_veh_parking_num) / TFlt(m_curb_cell_array[i] -> m_cell_capacity); // 0.2

		_double_parking_factor = 0.0 * TFlt(m_curb_cell_array[i] -> m_veh_doubleparking_num) / TFlt(m_curb_cell_array[i] -> m_cell_capacity); // 0.5 * 

		_total_factor = _normal_parking_factor + _double_parking_factor;

		if (_total_factor > 1.0){
			_total_factor = 1.0;
		}

		TFlt _num_lane_left = TFlt(this -> m_number_of_lane) - _total_factor;

		if (_num_lane_left < 0){
			_num_lane_left = 0;
		}

		_ratio_lane_closure_list[i] = _num_lane_left/_num_lane;

		// _ratio_lane_closure_list[i] = (TFlt(this -> m_number_of_lane) - 
		// 0.2 * TFlt(m_curb_cell_array[i] -> m_veh_parking_num) / TFlt(m_curb_cell_array[i] -> m_cell_capacity) -                                        // 0.2
		// 0.5 * TFlt(m_curb_cell_array[i] -> m_veh_doubleparking_num) / TFlt(m_curb_cell_array[i] -> m_cell_capacity))/(TFlt(this -> m_number_of_lane)); // 0.5
	}

	// printf("=== _ratio_lane_closure_list done ===\n");

	/* update volume */
	if (m_num_cells > 1){
		for (int i = 0; i < m_num_cells - 1; ++i)
		{
			m_cell_array[i] -> m_volume_car = m_cell_array[i] -> m_veh_queue_car.size();
			m_cell_array[i] -> m_volume_truck = m_cell_array[i] -> m_veh_queue_truck.size();
			// Update perceived density of the i-th cell
			m_cell_array[i] -> update_perceived_density(_ratio_lane_closure_list[i]);
			// if (m_link_ID == _output_link)
			// 	printf("(%d, %d) ", int(m_cell_array[i] -> m_volume_car), int(m_cell_array[i] -> m_volume_truck));
		}
	}

	_count_car = 0;
	_count_truck = 0;
	// m_class: 0 - private car, 1 - truck
	for (_veh_it = m_finished_array.begin(); _veh_it != m_finished_array.end(); _veh_it++){
		MNM_Veh_Multiclass *_veh = dynamic_cast<MNM_Veh_Multiclass *>(*_veh_it);
		if (_veh -> m_class == 0) _count_car += 1;
		if (_veh -> m_class == 1) _count_truck += 1;
	}
	m_cell_array[m_num_cells - 1] -> m_volume_car = 
		m_cell_array[m_num_cells - 1] -> m_veh_queue_car.size() + _count_car; // why adding two terms?
	m_cell_array[m_num_cells - 1] -> m_volume_truck = 
		m_cell_array[m_num_cells - 1] -> m_veh_queue_truck.size() + _count_truck;	
	m_cell_array[m_num_cells - 1] -> update_perceived_density(_ratio_lane_closure_list[m_num_cells - 1]);

	/* compute total volume of link, check if spill back */
	_count_tot_vehs = 0;
	for (int i = 0; i <= m_num_cells - 1; ++i)
	{
		_count_tot_vehs += m_cell_array[i] -> m_volume_car;
		_count_tot_vehs += m_cell_array[i] -> m_volume_truck * m_veh_convert_factor;
	}
	if (TFlt(_count_tot_vehs)/m_flow_scalar/m_length > m_lane_hold_cap_car * m_number_of_lane){
		m_spill_back = true;
	}

	//  count parking number
	TInt m_parking_num_car = 0;
	TInt m_parking_num_truck = 0;
	TInt m_parking_num_rh = 0;

	for (int m = 0; m < (m_num_cells - 1); ++m)
	{
		m_parking_num_car += TInt(m_curb_cell_array[m] -> m_veh_parking_car.size()) + TInt(m_curb_cell_array[m] -> m_veh_doubleparking_car.size());
		m_parking_num_truck += TInt(m_curb_cell_array[m] -> m_veh_parking_truck.size()) + TInt(m_curb_cell_array[m] -> m_veh_doubleparking_truck.size());
		m_parking_num_rh += TInt(m_curb_cell_array[m] -> m_veh_parking_rh.size()) + TInt(m_curb_cell_array[m] -> m_veh_doubleparking_rh.size());
	}

	// printf("======= link ID = %d, parking number = %d ==========\n", int(this -> m_link_ID) , int(_link_parking_num));
	// printf("======= link ID = %d, doubleparking number = %d ==========\n", int(this -> m_link_ID) , int(_link_doubleparking_num));
	// // printf("-------  m_lane_hold_cap_car * m_number_of_lane = %f  -----------\n", double(m_lane_hold_cap_car * m_number_of_lane));
	// printf("-------  m_lane_hold_cap_car * m_number_of_lane * _ratio = %f  -----------\n", double(m_lane_hold_cap_car * m_number_of_lane * _ratio_lane_closure));

	return 0;
}

/* TODO with lane closure */
/* TODO CTM link */
int MNM_Dlink_Ctm_Multiclass::evolve_control(TInt timestamp, TFlt _ratio_lane_closure, TFlt cell_position, TFlt cell_control_rate)
{
	
	std::deque<MNM_Veh*>::iterator _veh_it;
	TInt _count_car = 0; // car number
	TInt _count_truck = 0; // truck number 
	TInt _count_tot_vehs = 0; // total number

	// upper bound m_num_cells * cell_position
	if (cell_position == -1.0){
		update_out_veh(_ratio_lane_closure);
	}
	else{
		TInt control_cell_ID = (int)std::floor(m_num_cells * cell_position);
		update_out_veh_control(_ratio_lane_closure, control_cell_ID, cell_control_rate);
	}
	
	TInt _num_veh_tomove_car, _num_veh_tomove_truck;
	/* previous cells */
	if (m_num_cells > 1){
		for (int i = 0; i < m_num_cells - 1; ++i)
		{
			// Car
			_num_veh_tomove_car = m_cell_array[i] -> m_out_veh_car;
			move_veh_queue( &(m_cell_array[i] -> m_veh_queue_car),
							&(m_cell_array[i+1] -> m_veh_queue_car),
							_num_veh_tomove_car);
			// Truck
			_num_veh_tomove_truck = m_cell_array[i] -> m_out_veh_truck;
			move_veh_queue( &(m_cell_array[i] -> m_veh_queue_truck),
							&(m_cell_array[i+1] -> m_veh_queue_truck),
							_num_veh_tomove_truck);
		}
	}

	/* last cell */
	move_last_cell();
	m_tot_wait_time_at_intersection += TFlt(m_finished_array.size())/m_flow_scalar * m_unit_time; 

	/* update volume */
	if (m_num_cells > 1){
		for (int i = 0; i < m_num_cells - 1; ++i)
		{
			m_cell_array[i] -> m_volume_car = m_cell_array[i] -> m_veh_queue_car.size();
			m_cell_array[i] -> m_volume_truck = m_cell_array[i] -> m_veh_queue_truck.size();
			// Update perceived density of the i-th cell
			m_cell_array[i] -> update_perceived_density(_ratio_lane_closure);
		}
	}

	_count_car = 0;
	_count_truck = 0;
	// m_class: 0 - private car, 1 - truck
	for (_veh_it = m_finished_array.begin(); _veh_it != m_finished_array.end(); _veh_it++){
		MNM_Veh_Multiclass *_veh = dynamic_cast<MNM_Veh_Multiclass *>(*_veh_it);
		if (_veh -> m_class == 0) _count_car += 1;
		if (_veh -> m_class == 1) _count_truck += 1;
	}
	m_cell_array[m_num_cells - 1] -> m_volume_car = 
		m_cell_array[m_num_cells - 1] -> m_veh_queue_car.size() + _count_car;
	m_cell_array[m_num_cells - 1] -> m_volume_truck = 
		m_cell_array[m_num_cells - 1] -> m_veh_queue_truck.size() + _count_truck;	
	m_cell_array[m_num_cells - 1] -> update_perceived_density(_ratio_lane_closure);

	m_tot_wait_time_at_intersection_car += TFlt (_count_car) / m_flow_scalar * m_unit_time;
  	m_tot_wait_time_at_intersection_truck += TFlt (_count_truck) / m_flow_scalar * m_unit_time;

	/* compute total volume of link, check if spill back */
	_count_tot_vehs = 0;
	for (int i = 0; i <= m_num_cells - 1; ++i)
	{
		_count_tot_vehs += m_cell_array[i] -> m_volume_car;
		_count_tot_vehs += m_cell_array[i] -> m_volume_truck * m_veh_convert_factor;
	}
	if (TFlt(_count_tot_vehs)/m_flow_scalar/m_length > m_lane_hold_cap_car * m_number_of_lane * _ratio_lane_closure){
		m_spill_back = true;
	}

	// PMC indicators, us the first cell
	m_congested_car = m_cell_array[0] -> m_cell_congested_car;
	m_congested_truck = m_cell_array[0] -> m_cell_congested_truck;
	m_diff_car = m_cell_array[0] -> m_cell_diff_car;
	m_diff_truck = m_cell_array[0] -> m_cell_diff_truck;
	m_space_fraction_car = m_cell_array[0] -> m_space_fraction_car;
	m_space_fraction_truck = m_cell_array[0] -> m_space_fraction_truck;

	return 0; // load_int, ratio_lane_closure, cell_position, cell_control_rate
}

int MNM_Dlink_Ctm_Multiclass::evolve(TInt timestamp, TFlt _ratio_lane_closure)
{
	std::deque<MNM_Veh*>::iterator _veh_it;
	TInt _count_car = 0; // car number
	TInt _count_truck = 0; // truck number 
	TInt _count_tot_vehs = 0; // total number

	update_out_veh(_ratio_lane_closure);
	
	TInt _num_veh_tomove_car, _num_veh_tomove_truck;
	/* previous cells */
	if (m_num_cells > 1){
		for (int i = 0; i < m_num_cells - 1; ++i)
		{
			// Car
			_num_veh_tomove_car = m_cell_array[i] -> m_out_veh_car;
			move_veh_queue( &(m_cell_array[i] -> m_veh_queue_car),
							&(m_cell_array[i+1] -> m_veh_queue_car),
							_num_veh_tomove_car);
			// Truck
			_num_veh_tomove_truck = m_cell_array[i] -> m_out_veh_truck;
			move_veh_queue( &(m_cell_array[i] -> m_veh_queue_truck),
							&(m_cell_array[i+1] -> m_veh_queue_truck),
							_num_veh_tomove_truck);
		}
	}

	/* last cell */

	move_last_cell();
	m_tot_wait_time_at_intersection += TFlt(m_finished_array.size())/m_flow_scalar * m_unit_time;

	// if (m_link_ID == _output_link)
	// 	printf("Link %d volume after: ", int(m_link_ID));

	/* update volume */
	if (m_num_cells > 1){
		for (int i = 0; i < m_num_cells - 1; ++i)
		{
			m_cell_array[i] -> m_volume_car = m_cell_array[i] -> m_veh_queue_car.size();
			m_cell_array[i] -> m_volume_truck = m_cell_array[i] -> m_veh_queue_truck.size();
			// Update perceived density of the i-th cell
			m_cell_array[i] -> update_perceived_density(_ratio_lane_closure);
			// if (m_link_ID == _output_link)
			// 	printf("(%d, %d) ", int(m_cell_array[i] -> m_volume_car), int(m_cell_array[i] -> m_volume_truck));
		}
	}

	_count_car = 0;
	_count_truck = 0;
	// m_class: 0 - private car, 1 - truck
	for (_veh_it = m_finished_array.begin(); _veh_it != m_finished_array.end(); _veh_it++){
		MNM_Veh_Multiclass *_veh = dynamic_cast<MNM_Veh_Multiclass *>(*_veh_it);
		if (_veh -> m_class == 0) _count_car += 1;
		if (_veh -> m_class == 1) _count_truck += 1;
	}
	m_cell_array[m_num_cells - 1] -> m_volume_car = 
		m_cell_array[m_num_cells - 1] -> m_veh_queue_car.size() + _count_car; // why adding two terms?
	m_cell_array[m_num_cells - 1] -> m_volume_truck = 
		m_cell_array[m_num_cells - 1] -> m_veh_queue_truck.size() + _count_truck;	
	m_cell_array[m_num_cells - 1] -> update_perceived_density(_ratio_lane_closure);

	// if (m_link_ID == _output_link){
	// 	printf("(%d, %d; %d, %d)\n", int(m_cell_array[m_num_cells - 1] -> m_veh_queue_car.size()), 
	// 		int(m_cell_array[m_num_cells - 1] -> m_veh_queue_truck.size()), int(_count_car), int(_count_truck));
	// 	for (_veh_it = m_finished_array.begin(); _veh_it != m_finished_array.end(); _veh_it++){
	// 		MNM_Veh_Multiclass *_veh = dynamic_cast<MNM_Veh_Multiclass *>(*_veh_it);
	// 		printf("(%d-%d: %d->%d) ", int(_veh -> m_veh_ID), int(_veh -> m_class), int(_veh -> get_current_link() -> m_link_ID), 
	// 			int(_veh -> get_next_link() -> m_link_ID));
	// 	}
	// 	printf("\n");
	// }

	/* compute total volume of link, check if spill back */
	_count_tot_vehs = 0;
	for (int i = 0; i <= m_num_cells - 1; ++i)
	{
		_count_tot_vehs += m_cell_array[i] -> m_volume_car;
		_count_tot_vehs += m_cell_array[i] -> m_volume_truck * m_veh_convert_factor;
	}
	if (TFlt(_count_tot_vehs)/m_flow_scalar/m_length > m_lane_hold_cap_car * m_number_of_lane * _ratio_lane_closure){
		m_spill_back = true;
	}

	// printf("=======  ratio = %f ==========\n", double(_ratio_lane_closure));
	// printf("-------  m_lane_hold_cap_car * m_number_of_lane = %f  -----------\n", double(m_lane_hold_cap_car * m_number_of_lane));
	// printf("-------  m_lane_hold_cap_car * m_number_of_lane * _ratio = %f  -----------\n", double(m_lane_hold_cap_car * m_number_of_lane * _ratio_lane_closure));

	return 0;
}

// TODO Sep. 29 
int MNM_Dlink_Ctm_Multiclass::move_last_cell_curb(TInt timestamp)
{
	// step 1: check how many vehicles are departing from curb cell, update departing queue
	m_curb_cell_array[m_num_cells - 1] -> move_veh_out_curb(timestamp);
															// &m_curb_cell_array[m_num_cells - 1]->m_veh_parking_car,
															// &m_curb_cell_array[m_num_cells - 1]->m_veh_parking_truck,
															// &m_curb_cell_array[m_num_cells - 1]->m_veh_parking_rh,
															// &m_curb_cell_array[m_num_cells - 1]->m_veh_departing);
	// move last cell
	TInt _num_veh_tomove_car = m_cell_array[m_num_cells - 1] -> m_out_veh_car; // m_out_veh_car = m_cell_array[m_num_cells - 1] -> m_veh_queue_car.size()
	TInt _num_veh_tomove_truck = m_cell_array[m_num_cells - 1] -> m_out_veh_truck;
	TFlt _pstar = TFlt(_num_veh_tomove_car)/TFlt(_num_veh_tomove_car + _num_veh_tomove_truck);
	MNM_Veh* _veh;
	MNM_Veh_Multiclass* _veh_multiclass;
	TFlt _r;

	while ((_num_veh_tomove_car > 0) || (_num_veh_tomove_truck > 0)){ // both car and truck have flow to move
		_r = MNM_Ults::rand_flt(); // randomly get a number to decide to move a car or truck.
		// probability = _pstar to move a car
		if (_r < _pstar){ // if _r is less than random number
			// and still has car to move
			if (_num_veh_tomove_car > 0){
				_veh = m_cell_array[m_num_cells - 1] -> m_veh_queue_car.front();
				m_cell_array[m_num_cells - 1] -> m_veh_queue_car.pop_front(); // delete the first element	
				_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass *>(_veh);

				// veh has predefined curb locations from reading inout file
				if (_veh_multiclass -> m_curb_destination_list.front() == this -> m_link_ID){
					m_curb_cell_array[m_num_cells - 1] -> m_veh_arriving.push_back(_veh); // now the arriving is not real arriving, it is a buffer
				}
				// veh has predefined inter-destination = the end node of current link
				// else if (_veh_multiclass -> m_destination_list.front() == this -> m_to_node -> m_node_ID){
				// 	m_curb_cell_array[m_num_cells - 1] -> m_veh_arriving.push_back(_veh);
				// }	
				else{
					// if not parking, move to finish array
					if (_veh -> has_next_link()){
						m_finished_array.push_back(_veh);
					}
					else {
						printf("Dlink_CTM_Multiclass::Some thing wrong!\n");
						exit(-1);
					}
				}
				_num_veh_tomove_car--;
			}
			// no car to move, move a truck
			else {
				_veh = m_cell_array[m_num_cells - 1] -> m_veh_queue_truck.front();
				m_cell_array[m_num_cells - 1] -> m_veh_queue_truck.pop_front();
				_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass *>(_veh);

				if (_veh_multiclass -> m_curb_destination_list.front() == this -> m_link_ID){
					m_curb_cell_array[m_num_cells - 1] -> m_veh_arriving.push_back(_veh); // now the arriving is not real arriving, it is a buffer
				}	
				else{
					// if not parking, move to finish array
					if (_veh -> has_next_link()){
						m_finished_array.push_back(_veh);
					}
					else {
						printf("Dlink_CTM_Multiclass::Some thing wrong!\n");
						exit(-1);
					}
				}
				_num_veh_tomove_truck--;
			}
		}
		// probability = 1 - _pstar to move a truck
		else {
			// still has truck to move
			if (_num_veh_tomove_truck > 0){
				_veh = m_cell_array[m_num_cells - 1] -> m_veh_queue_truck.front();
				m_cell_array[m_num_cells - 1] -> m_veh_queue_truck.pop_front();
				_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass *>(_veh);

				if (_veh_multiclass -> m_curb_destination_list.front() == this -> m_link_ID){
					m_curb_cell_array[m_num_cells - 1] -> m_veh_arriving.push_back(_veh); // now the arriving is not real arriving, it is a buffer
				}	
				else{
					// if not parking, move to finish array
					if (_veh -> has_next_link()){
						m_finished_array.push_back(_veh);
					}
					else {
						printf("Dlink_CTM_Multiclass::Some thing wrong!\n");
						exit(-1);
					}
				}
				_num_veh_tomove_truck--;
			}
			
			// no truck to move, move a car
			else {
				_veh = m_cell_array[m_num_cells - 1] -> m_veh_queue_car.front();
				m_cell_array[m_num_cells - 1] -> m_veh_queue_car.pop_front();
				_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass *>(_veh);

				if (_veh_multiclass -> m_curb_destination_list.front() == this -> m_link_ID){
					m_curb_cell_array[m_num_cells - 1] -> m_veh_arriving.push_back(_veh); // now the arriving is not real arriving, it is a buffer
				}	
				else{
					// if not parking, move to finish array
					if (_veh -> has_next_link()){
						m_finished_array.push_back(_veh);
					}
					else {
						printf("Dlink_CTM_Multiclass::Some thing wrong!\n");
						exit(-1);
					}
				}
				_num_veh_tomove_car--;
			}
		}
	}

	// check arriving vehs are real arriving

	int _num_available = int(m_curb_cell_array[m_num_cells - 1] -> m_cell_capacity) - (int(m_curb_cell_array[m_num_cells - 1] -> m_veh_parking_car.size()) \
																				+ int(m_curb_cell_array[m_num_cells - 1] -> m_veh_parking_truck.size()) * 2 \
																				+ int(m_curb_cell_array[m_num_cells - 1] -> m_veh_parking_rh.size()));

	// shuffle to make sure fairness for car and truck to park
	random_shuffle(m_curb_cell_array[m_num_cells - 1] -> m_veh_arriving.begin(), m_curb_cell_array[m_num_cells - 1] -> m_veh_arriving.end());

	// now check how many vehs can really arrive

	int check_num = int(m_curb_cell_array[m_num_cells - 1] -> m_veh_arriving.size());

	for (int k = 0; k < check_num; ++k){
		_veh = m_curb_cell_array[m_num_cells - 1] -> m_veh_arriving.front();
		m_curb_cell_array[m_num_cells - 1] -> m_veh_arriving.pop_front();

		_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass *>(_veh);

		// for car and rh 
		if ((_veh_multiclass -> m_class == 0) || (_veh_multiclass -> m_class == 2)){
			if (_num_available >= 1){
				m_curb_cell_array[m_num_cells - 1] -> m_veh_arriving.push_back(_veh);
				_num_available -= 1;
			}
			else { //put into double parking
				
				if (_veh_multiclass -> m_class == 0){
					// _veh_multiclass -> m_arrival_time_list.push_back(timestamp);
					// m_curb_cell_array[m_num_cells - 1] -> m_veh_doubleparking_car.push_back(_veh);
					m_curb_cell_array[m_num_cells - 1] -> m_veh_arriving.push_back(_veh);

				}
				else {
					_veh_multiclass -> m_arrival_time_list.push_back(timestamp);
					m_curb_cell_array[m_num_cells - 1] -> m_veh_doubleparking_rh.push_back(_veh);
				}
				// _num_available_dp -= 1;
			}
			// else { // back to next cell queue
			// 	_veh_multiclass -> m_visual_position_on_link += float(1)/float(m_num_cells);

			// 	if (_veh_multiclass -> m_visual_position_on_link > 0.99){
			// 		_veh_multiclass -> m_visual_position_on_link = 0.99;
			// 	} 
			// 	m_finished_array.push_back(_veh);
			// }
		}

		// for truck
		if (_veh_multiclass -> m_class == 1){
			if (_num_available >= 2){
				m_curb_cell_array[m_num_cells - 1] -> m_veh_arriving.push_back(_veh);
				_num_available -= 2;
			}
			else {
				_veh_multiclass -> m_arrival_time_list.push_back(timestamp);
				m_curb_cell_array[m_num_cells - 1] -> m_veh_doubleparking_truck.push_back(_veh);
				// _num_available_dp -= 2;
			}
			// else{
			// 	_veh_multiclass -> m_visual_position_on_link += float(1)/float(m_num_cells);

			// 	if (_veh_multiclass -> m_visual_position_on_link > 0.99){
			// 		_veh_multiclass -> m_visual_position_on_link = 0.99;
			// 	}
			// 	m_finished_array.push_back(_veh);
			// }
		}
	}

	// move departing to finish_array
	// seperate car and truck
	TInt _num_departing = m_curb_cell_array[m_num_cells - 1] -> m_veh_departing.size();

	TInt _num_departing_car = TInt(0);

	TInt _num_departing_truck = TInt(0);

	for (int i = 0; i < _num_departing; ++i){
		_veh = m_curb_cell_array[m_num_cells - 1] -> m_veh_departing.front();
		m_curb_cell_array[m_num_cells - 1] -> m_veh_departing.pop_front();

		_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass *>(_veh);

		if ((_veh_multiclass -> m_class == 0) || (_veh_multiclass -> m_class == 2)){
			m_curb_cell_array[m_num_cells - 1] -> m_veh_departing_car.push_back(_veh);

			_num_departing_car++;
		}

		else if (_veh_multiclass -> m_class == 1){
			m_curb_cell_array[m_num_cells - 1] -> m_veh_departing_truck.push_back(_veh);
			_num_departing_truck++;
		}

		else{
			printf("Curb last cell departing veh class error\n");
			exit(-1);
		}
	}

	// moving departing to finish array
	TFlt _pstar_depart = TFlt(_num_departing_car)/TFlt(_num_departing_car + _num_departing_truck);
	TFlt _r_depart;

	while ((_num_departing_car > 0) || (_num_departing_truck > 0)){ // both car and truck have flow to move
		_r_depart = MNM_Ults::rand_flt(); // randomly get a number to decide to move a car or truck.
		// probability = _pstar to move a car
		if (_r_depart < _pstar_depart){ // if _r is less than random number
			// and still has car to move
			if (_num_departing_car > 0){
				_veh = m_curb_cell_array[m_num_cells - 1] -> m_veh_departing_car.front();

				_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass *>(_veh);

				m_curb_cell_array[m_num_cells - 1] -> m_veh_departing_car.pop_front(); // delete the first element	

				// move to finish array
				if (_veh -> has_next_link()){
					m_finished_array.push_back(_veh);
					_veh_multiclass -> m_curb_destination_list.pop_front();
					_veh_multiclass -> m_complete_stop_current_link = TInt(1);
				}
				else {
					printf("Dlink_CTM_Multiclass::Some thing wrong!\n");
					exit(-1);
				}
				_num_departing_car--;
			}
			// no car to move, move a truck
			else {
				_veh = m_curb_cell_array[m_num_cells - 1] -> m_veh_departing_truck.front();

				_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass *>(_veh);

				m_curb_cell_array[m_num_cells - 1] -> m_veh_departing_truck.pop_front();

				// if not parking, move to finish array
				if (_veh -> has_next_link()){
					m_finished_array.push_back(_veh);
					_veh_multiclass -> m_curb_destination_list.pop_front();
					_veh_multiclass -> m_complete_stop_current_link = TInt(1);
				}
				else {
					printf("Dlink_CTM_Multiclass::Some thing wrong!\n");
					exit(-1);
				}
				_num_departing_truck--;
			}
		}
		// probability = 1 - _pstar to move a truck
		else {
			// still has truck to move
			if (_num_departing_truck > 0){
				_veh = m_curb_cell_array[m_num_cells - 1] -> m_veh_departing_truck.front();

				_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass *>(_veh);

				m_curb_cell_array[m_num_cells - 1] -> m_veh_departing_truck.pop_front();

				// if not parking, move to finish array
				if (_veh -> has_next_link()){
					m_finished_array.push_back(_veh);
					_veh_multiclass -> m_curb_destination_list.pop_front();
					_veh_multiclass -> m_complete_stop_current_link = TInt(1);
				}
				else {
					printf("Dlink_CTM_Multiclass::Some thing wrong!\n");
					exit(-1);
				}
				_num_departing_truck--;
			}
			
			// no truck to move, move a car
			else {
				_veh = m_curb_cell_array[m_num_cells - 1] -> m_veh_departing_car.front();

				_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass *>(_veh);

				m_curb_cell_array[m_num_cells - 1] -> m_veh_departing_car.pop_front(); // delete the first element	

				// move to finish array
				if (_veh -> has_next_link()){
					m_finished_array.push_back(_veh);
					_veh_multiclass -> m_curb_destination_list.pop_front();
					_veh_multiclass -> m_complete_stop_current_link = TInt(1);
				}
				else {
					printf("Dlink_CTM_Multiclass::Some thing wrong!\n");
					exit(-1);
				}
				_num_departing_car--;
			}
		}
	}

	// step 3: moving vehs from arriving to curb parking queue
	m_curb_cell_array[m_num_cells - 1] -> move_veh_in_curb(timestamp);
													// &m_curb_cell_array[m_num_cells - 1]->m_veh_arriving,
													// &m_curb_cell_array[m_num_cells - 1]->m_veh_parking_car,
													// &m_curb_cell_array[m_num_cells - 1]->m_veh_parking_truck,
													// &m_curb_cell_array[m_num_cells - 1]->m_veh_parking_rh);

	// update parking num
	m_curb_cell_array[m_num_cells - 1] -> m_veh_parking_num = int(m_curb_cell_array[m_num_cells - 1] -> m_veh_parking_car.size()) + 
														int(m_curb_cell_array[m_num_cells - 1] -> m_veh_parking_rh.size()) +
														2 * int(m_curb_cell_array[m_num_cells - 1] -> m_veh_parking_truck.size());

	m_curb_cell_array[m_num_cells - 1] -> m_veh_doubleparking_num = int(m_curb_cell_array[m_num_cells - 1] -> m_veh_doubleparking_rh.size()) +
														2 * int(m_curb_cell_array[m_num_cells - 1] -> m_veh_doubleparking_truck.size());

	return 0;
}

// this is how to get m_finished_array??
// m_finished_array means vehicles are already 
// in the last cell and ready to leave the link

// jiachao added in Apr. 19
int MNM_Dlink_Ctm_Multiclass::move_last_cell()
{
	TInt _num_veh_tomove_car = m_cell_array[m_num_cells - 1] -> m_out_veh_car; // m_out_veh_car = m_cell_array[m_num_cells - 1] -> m_veh_queue_car.size()
	TInt _num_veh_tomove_truck = m_cell_array[m_num_cells - 1] -> m_out_veh_truck;
	TFlt _pstar = TFlt(_num_veh_tomove_car)/TFlt(_num_veh_tomove_car + _num_veh_tomove_truck);
	MNM_Veh* _veh;
	TFlt _r;
	while ((_num_veh_tomove_car > 0) || (_num_veh_tomove_truck > 0)){ // both car and truck have flow to move
		_r = MNM_Ults::rand_flt(); // randomly get a number to decide to move a car or truck.
		// probability = _pstar to move a car
		if (_r < _pstar){ // if _r is less than random number
			// and still has car to move
			if (_num_veh_tomove_car > 0){
				_veh = m_cell_array[m_num_cells - 1] -> m_veh_queue_car.front();
				m_cell_array[m_num_cells - 1] -> m_veh_queue_car.pop_front(); // delete the first element
				if (_veh -> has_next_link()){
					m_finished_array.push_back(_veh);
				}
				else {
					printf("Dlink_CTM_Multiclass::Some thing wrong!\n");
					exit(-1);
				}
				_num_veh_tomove_car--;
			}
			// no car to move, move a truck
			else {
				_veh = m_cell_array[m_num_cells - 1] -> m_veh_queue_truck.front();
				m_cell_array[m_num_cells - 1] -> m_veh_queue_truck.pop_front();
				if (_veh -> has_next_link()){
					m_finished_array.push_back(_veh);
				}
				else {
					printf("Dlink_CTM_Multiclass::Some thing wrong!\n");
					exit(-1);
				}
				_num_veh_tomove_truck--;
			}
		}
		// probability = 1 - _pstar to move a truck
		else {
			// still has truck to move
			if (_num_veh_tomove_truck > 0){
				_veh = m_cell_array[m_num_cells - 1] -> m_veh_queue_truck.front();
				m_cell_array[m_num_cells - 1] -> m_veh_queue_truck.pop_front();
				if (_veh -> has_next_link()){
					m_finished_array.push_back(_veh);
				}
				else {
					printf("Dlink_CTM_Multiclass::Some thing wrong!\n");
					exit(-1);
				}
				_num_veh_tomove_truck--;
			}
			
			// no truck to move, move a car
			else {
				_veh = m_cell_array[m_num_cells - 1] -> m_veh_queue_car.front();
				m_cell_array[m_num_cells - 1] -> m_veh_queue_car.pop_front();
				if (_veh -> has_next_link()){
					m_finished_array.push_back(_veh);
				}
				else {
					printf("Dlink_CTM_Multiclass::Some thing wrong!\n");
					exit(-1);
				}
				_num_veh_tomove_car--;
			}
		}
	}
	return 0;
}

/* get_link_capacity function added by Jiachao */
TFlt MNM_Dlink_Ctm_Multiclass::get_link_capacity()
{
	return m_lane_flow_cap_car * TFlt(m_number_of_lane) * m_unit_time;
}

/* TODO by Jiachao add ratio*/
TFlt MNM_Dlink_Ctm_Multiclass::get_link_supply()
{
	TFlt _real_volume_both = ( TFlt(m_cell_array[0] -> m_volume_truck) * m_veh_convert_factor + 
							   TFlt(m_cell_array[0] -> m_volume_car) ) / m_flow_scalar;
	// TFlt _real_volume_both = ( TFlt(m_cell_array[0] -> m_volume_truck) * 1 + 
	// 						   TFlt(m_cell_array[0] -> m_volume_car) ) / m_flow_scalar;

	// m_cell_length can't be 0 according to implementation above
	TFlt _density = _real_volume_both / (m_cell_array[0] -> m_cell_length);
	double _tmp = std::min(double(m_cell_array[0] -> m_flow_cap_car), m_wave_speed_car * (m_cell_array[0] -> m_hold_cap_car - _density));

	// only use when network is too large and complex and no other ways solving gridlock.
	// _tmp = std::max(_tmp, m_wave_speed_car * 0.25 * (m_cell_array[0] -> m_hold_cap_car - _density));

	return std::max(0.0, _tmp) * (m_cell_array[0] -> m_unit_time);
}
	
int MNM_Dlink_Ctm_Multiclass::clear_incoming_array(TInt timestamp, TFlt _ratio_lane_closure)
{
	MNM_Veh_Multiclass* _veh;
	size_t _cur_size = m_incoming_array.size();
	for (size_t i = 0; i < _cur_size; ++i) {
		_veh = dynamic_cast<MNM_Veh_Multiclass *>(m_incoming_array.front());
		m_incoming_array.pop_front();
		if (_veh -> m_class == TInt(0)) {
			// printf("car\n");
			m_cell_array[0] -> m_veh_queue_car.push_back(_veh);
		}
		else {
			// printf("truck\n");
			m_cell_array[0] -> m_veh_queue_truck.push_back(_veh);
		}
		_veh -> m_visual_position_on_link = float(1)/float(m_num_cells)/float(2); // initial position at first cell
	}
	m_cell_array[0] -> m_volume_car = m_cell_array[0] -> m_veh_queue_car.size();
	m_cell_array[0] -> m_volume_truck = m_cell_array[0] -> m_veh_queue_truck.size();
	m_cell_array[0] -> update_perceived_density(_ratio_lane_closure);
	
	return 0;
}

TFlt MNM_Dlink_Ctm_Multiclass::get_link_flow_car()
{
	TInt _total_volume_car = 0;
	for (int i = 0; i < m_num_cells; ++i){
		_total_volume_car += m_cell_array[i] -> m_volume_car;
	}
	std::deque<MNM_Veh*>::iterator _veh_it;
	for (_veh_it = m_finished_array.begin(); _veh_it != m_finished_array.end(); _veh_it++){
		MNM_Veh_Multiclass *_veh = dynamic_cast<MNM_Veh_Multiclass *>(*_veh_it);
		if (_veh -> m_class == 0) _total_volume_car += 1;
	}
	return TFlt(_total_volume_car) / m_flow_scalar;
}

TFlt MNM_Dlink_Ctm_Multiclass::get_link_flow_truck()
{
	TInt _total_volume_truck = 0;
	for (int i = 0; i < m_num_cells; ++i){
		_total_volume_truck += m_cell_array[i] -> m_volume_truck;
	}
	std::deque<MNM_Veh*>::iterator _veh_it;
	for (_veh_it = m_finished_array.begin(); _veh_it != m_finished_array.end(); _veh_it++){
		MNM_Veh_Multiclass *_veh = dynamic_cast<MNM_Veh_Multiclass *>(*_veh_it);
		if (_veh -> m_class == 1) _total_volume_truck += 1;
	}
	return TFlt(_total_volume_truck) / m_flow_scalar;
}

TFlt MNM_Dlink_Ctm_Multiclass::get_link_flow()
{
	// For get_link_tt in adaptive routing
	TInt _total_volume_car = 0;
	TInt _total_volume_truck = 0;
	for (int i = 0; i < m_num_cells; ++i){
		_total_volume_car += m_cell_array[i] -> m_volume_car;
		_total_volume_truck += m_cell_array[i] -> m_volume_truck;
	}
	return TFlt(_total_volume_car + _total_volume_truck + m_finished_array.size()) / m_flow_scalar;
}

TFlt MNM_Dlink_Ctm_Multiclass::get_link_tt()
{
	// For adaptive routing and emissions, need modification for multiclass cases
	TFlt _cost, _spd;
	// get the density in veh/mile/lane
	TFlt _rho = get_link_flow()/m_number_of_lane/m_length;
	// get the jam density
	TFlt _rhoj = m_lane_hold_cap_car;
	// get the critical density
	TFlt _rhok = m_lane_flow_cap_car/m_ffs_car;

	if (_rho >= _rhoj){
		_cost = MNM_Ults::max_link_cost();
	}
	else {
		if (_rho <= _rhok){
			_spd = m_ffs_car;
		}
		else {
			_spd = MNM_Ults::max(0.001 * m_ffs_car, 
					m_lane_flow_cap_car * (_rhoj - _rho) / (_rhoj - _rhok) / _rho);
		}
		_cost = m_length / _spd;
	}
	return _cost;
}

TFlt MNM_Dlink_Ctm_Multiclass::get_link_tt_from_flow_car(TFlt flow)
{
    TFlt _cost, _spd;
    // get the density in veh/mile/lane
    TFlt _rho = flow/m_number_of_lane/m_length;
    // get the jam density
    TFlt _rhoj = m_lane_hold_cap_car;
    // get the critical density
    TFlt _rhok = m_lane_flow_cap_car/m_ffs_car;

    if (_rho >= _rhoj){
        _cost = MNM_Ults::max_link_cost();
    }
    else {
        if (_rho <= _rhok){
            _spd = m_ffs_car;
        }
        else {
            _spd = MNM_Ults::max(0.001 * m_ffs_car,
                                 m_lane_flow_cap_car * (_rhoj - _rho) / (_rhoj - _rhok) / _rho);
        }
        _cost = m_length / _spd;
    }
    return _cost;
}

TFlt MNM_Dlink_Ctm_Multiclass::get_link_tt_from_flow_truck(TFlt flow)
{
    TFlt _cost, _spd;
    // get the density in veh/mile/lane
    TFlt _rho = flow/m_number_of_lane/m_length;
    // get the jam density
    TFlt _rhoj = m_lane_hold_cap_truck;
    // get the critical density
    TFlt _rhok = m_lane_flow_cap_truck/m_ffs_truck;

    if (_rho >= _rhoj){
        _cost = MNM_Ults::max_link_cost();
    }
    else {
        if (_rho <= _rhok){
            _spd = m_ffs_truck;
        }
        else {
            _spd = MNM_Ults::max(0.001 * m_ffs_truck,
                                 m_lane_flow_cap_truck * (_rhoj - _rho) / (_rhoj - _rhok) / _rho);
        }
        _cost = m_length / _spd;
    }
    return _cost;
}

TInt MNM_Dlink_Ctm_Multiclass::get_link_freeflow_tt_loading_car()
{
	return m_num_cells;
}

TInt MNM_Dlink_Ctm_Multiclass::get_link_freeflow_tt_loading_truck()
{
	// due the random rounding, this is a random number
	return m_num_cells * m_ffs_car / m_ffs_truck;
}

/**************************************************************************
 						Multiclass Curb cell for CTM link
**************************************************************************/

MNM_Dlink_Ctm_Multiclass::Cell_Curb_Multiclass::Cell_Curb_Multiclass(TInt cell_ID,
						 											 TInt link_ID,
						 											 TInt dest_ID,
						 											 TInt cell_capacity,
						 											 TFlt unit_time)
{
	m_veh_parking_car = std::deque<MNM_Veh*>();
	m_veh_parking_truck = std::deque<MNM_Veh*>();
	m_veh_parking_rh = std::deque<MNM_Veh*>();

	m_veh_doubleparking_car = std::deque<MNM_Veh*>();
	m_veh_doubleparking_truck = std::deque<MNM_Veh*>();
	m_veh_doubleparking_rh = std::deque<MNM_Veh*>();

	m_veh_arriving = std::deque<MNM_Veh*>();
	m_veh_departing = std::deque<MNM_Veh*>();
	m_veh_departing_car = std::deque<MNM_Veh*>();
	m_veh_departing_truck = std::deque<MNM_Veh*>();

	// m_veh_arriving_car = std::deque<MNM_Veh*>();
	// m_veh_arriving_truck = std::deque<MNM_Veh*>();

	m_cell_ID = cell_ID;
	m_link_ID = link_ID;
	m_dest_ID = dest_ID;
	m_cell_capacity = cell_capacity;
	m_unit_time = unit_time;
	m_current_time = TFlt(0);
	m_veh_parking_num = TInt(0);
	m_veh_doubleparking_num = TInt(0);
}

MNM_Dlink_Ctm_Multiclass::Cell_Curb_Multiclass::~Cell_Curb_Multiclass()
{
	m_veh_parking_car.clear();
	m_veh_parking_truck.clear();
	m_veh_parking_rh.clear();

	m_veh_doubleparking_car.clear();
	m_veh_doubleparking_truck.clear();
	m_veh_doubleparking_rh.clear();

	m_veh_arriving.clear();
	m_veh_departing.clear();

	// m_veh_departing_car.clear();
	// m_veh_departing_truck.clear();
	// m_veh_arriving_car.clear();
	// m_veh_arriving_truck.clear();
}

// Curb-Jiachao added moving veh from m_veh_arriving to m_veh_parking in curb cell
// from_queue is m_veh_arriving (after checking if vehs can park at the curb within the capacity)
// arriving vehs must arrive !!!
int MNM_Dlink_Ctm_Multiclass::Cell_Curb_Multiclass::move_veh_in_curb(TInt timestamp)
																	//  std::deque<MNM_Veh*> *from_queue, // arriving_queue m_veh_arriving
                                									//  std::deque<MNM_Veh*> *to_queue_car, // parking_queue for driving m_veh_parking_car
																	//  std::deque<MNM_Veh*> *to_queue_truck, // parking queue for truck m_veh_parking_truck
																	//  std::deque<MNM_Veh*> *to_queue_rh)// parking queue for RH m_veh_parking_rh
{
	MNM_Veh* _veh;
	MNM_Veh_Multiclass* _veh_multiclass;

	int _arriving_num = int(m_veh_arriving.size());

	for (int i = 0; i < _arriving_num; ++i){
		_veh = m_veh_arriving.front();
		m_veh_arriving.pop_front();
		_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass *>(_veh);
		
		if (_veh_multiclass->m_class == 0){
			// record the arrival time
			_veh_multiclass -> m_arrival_time_list.push_back(timestamp);
			m_veh_parking_car.push_back(_veh);
		}

		else if (_veh_multiclass->m_class == 1){

			// record the arrival time
			_veh_multiclass -> m_arrival_time_list.push_back(timestamp);
			m_veh_parking_truck.push_back(_veh);
		}

		else if (_veh_multiclass->m_class == 2){
			// record the arrival time
			_veh_multiclass -> m_arrival_time_list.push_back(timestamp);
			m_veh_parking_rh.push_back(_veh);
		}
		else{
			printf("error: from_queue has veh which is not car/rh/truck, unknown m_class\n");
		}
		
	} 
	return 0;
}

// Curb-Jiachao added moving veh from m_veh_parking in curb cell to m_veh_departing
// first checking parking duration then deciding whether to move
// from_queue_car/truck/rh are m_veh_parking_car/truck/rh
// to_queue is m_veh_departing
int MNM_Dlink_Ctm_Multiclass::Cell_Curb_Multiclass::move_veh_out_curb(TInt timestamp)
																	//   std::deque<MNM_Veh*> *from_queue_car,m_veh_parking_car 
																	//   std::deque<MNM_Veh*> *from_queue_truck,m_veh_parking_truck
																	//   std::deque<MNM_Veh*> *from_queue_rh,m_veh_parking_rh
                                									//   std::deque<MNM_Veh*> *to_queue) // m_veh_departing
{
	MNM_Veh* _veh;
	MNM_Veh_Multiclass* _veh_multiclass;

	int _num_parking_car = int(m_veh_parking_car.size());

	for (int i = 0; i < _num_parking_car; ++i){
		_veh = m_veh_parking_car.front();
		m_veh_parking_car.pop_front();
		_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass *>(_veh);

		// count stopping time, return current number
		int stop_id = _veh_multiclass -> m_arrival_time_list.size(); // m_arrival_time_list: std::vector<TInt> vector to record arrival time, the last one is the current arrival time

		if ((timestamp - _veh_multiclass -> m_arrival_time_list.back()) >= (_veh_multiclass -> m_parking_duration_list[stop_id - 1])){
			// record departure time
			_veh_multiclass -> m_departure_time_list.push_back(timestamp);
			_veh_multiclass -> m_complete_stop_current_link = TInt(1);
			
			m_veh_departing.push_back(_veh);
		}
		else{
			m_veh_parking_car.push_back(_veh);
		}
	}

	int _num_parking_truck = int(m_veh_parking_truck.size());

	for (int j = 0; j < _num_parking_truck; ++j){
		_veh = m_veh_parking_truck.front();
		m_veh_parking_truck.pop_front();
		_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass *>(_veh);

		int stop_id = _veh_multiclass -> m_arrival_time_list.size();

		if ((timestamp - _veh_multiclass -> m_arrival_time_list.back()) >= _veh_multiclass->m_parking_duration_list[stop_id - 1]){
			// record departure time
			_veh_multiclass -> m_departure_time_list.push_back(timestamp);
			_veh_multiclass -> m_complete_stop_current_link = TInt(1);
			// _veh_multiclass -> m_curb_destination_list.pop_front(); // delete this curb destination
			m_veh_departing.push_back(_veh);
		}
		else{
			m_veh_parking_truck.push_back(_veh);
		}
	}

	int _num_parking_rh = int(m_veh_parking_rh.size());
	for (int k = 0; k < _num_parking_rh; ++k){
		_veh = m_veh_parking_rh.front();
		m_veh_parking_rh.pop_front();
		_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass *>(_veh);

		int stop_id = _veh_multiclass -> m_arrival_time_list.size();

		if ((timestamp - _veh_multiclass -> m_arrival_time_list.back()) >= _veh_multiclass->m_parking_duration_list[stop_id - 1]){
			// record departure time
			_veh_multiclass -> m_departure_time_list.push_back(timestamp);
			_veh_multiclass -> m_complete_stop_current_link = TInt(1);
			// _veh_multiclass -> m_curb_destination_list.pop_front(); // delete this curb destination
			m_veh_departing.push_back(_veh);
		}
		else{
			m_veh_parking_rh.push_back(_veh);
		}
	}

	// Double parking
	int _num_dp_car = int(m_veh_doubleparking_car.size());

	for (int i = 0; i < _num_dp_car; ++i){
		_veh = m_veh_doubleparking_car.front();
		m_veh_doubleparking_car.pop_front();
		_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass *>(_veh);

		// count stopping time, return current number
		int stop_id = _veh_multiclass -> m_arrival_time_list.size(); // m_arrival_time_list: std::vector<TInt> vector to record arrival time, the last one is the current arrival time

		if ((timestamp - _veh_multiclass -> m_arrival_time_list.back()) >= (_veh_multiclass -> m_parking_duration_list[stop_id - 1])){
			// record departure time
			_veh_multiclass -> m_departure_time_list.push_back(timestamp);
			_veh_multiclass -> m_complete_stop_current_link = TInt(1);
			// _veh_multiclass -> m_curb_destination_list.pop_front(); // delete this curb destination
			m_veh_departing.push_back(_veh);
		}
		else{
			m_veh_doubleparking_car.push_back(_veh);
		}
	}

	int _num_dp_truck = int(m_veh_doubleparking_truck.size());

	for (int j = 0; j < _num_dp_truck; ++j){
		_veh = m_veh_doubleparking_truck.front();
		m_veh_doubleparking_truck.pop_front();
		_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass *>(_veh);

		int stop_id = _veh_multiclass -> m_arrival_time_list.size();

		if ((timestamp - _veh_multiclass -> m_arrival_time_list.back()) >= _veh_multiclass->m_parking_duration_list[stop_id - 1]){
			// record departure time
			_veh_multiclass -> m_departure_time_list.push_back(timestamp);
			_veh_multiclass -> m_complete_stop_current_link = TInt(1);
			// _veh_multiclass -> m_curb_destination_list.pop_front(); // delete this curb destination
			m_veh_departing.push_back(_veh);
		}
		else{
			m_veh_doubleparking_truck.push_back(_veh);
		}
	}

	int _num_dp_rh = int(m_veh_doubleparking_rh.size());

	for (int k = 0; k < _num_dp_rh; ++k){
		_veh = m_veh_doubleparking_rh.front();
		m_veh_doubleparking_rh.pop_front();
		_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass *>(_veh);

		int stop_id = _veh_multiclass -> m_arrival_time_list.size();

		if ((timestamp - _veh_multiclass -> m_arrival_time_list.back()) >= _veh_multiclass->m_parking_duration_list[stop_id - 1]){
			// record departure time
			_veh_multiclass -> m_departure_time_list.push_back(timestamp);
			_veh_multiclass -> m_complete_stop_current_link = TInt(1);
			// _veh_multiclass -> m_curb_destination_list.pop_front(); // delete this curb destination
			m_veh_departing.push_back(_veh);
		}
		else{
			m_veh_doubleparking_rh.push_back(_veh);
		}
	}
	return 0;
}

/**************************************************************************
						Multiclass CTM Cells
**************************************************************************/
MNM_Dlink_Ctm_Multiclass::Ctm_Cell_Multiclass::Ctm_Cell_Multiclass(TInt cell_ID, TFlt cell_length,
														TFlt unit_time,
														TFlt hold_cap_car,
														TFlt hold_cap_truck,
														TFlt critical_density_car,
														TFlt critical_density_truck, 
														TFlt rho_1_N,
														TFlt flow_cap_car,
														TFlt flow_cap_truck,
														TFlt ffs_car,
														TFlt ffs_truck,
														TFlt wave_speed_car,
														TFlt wave_speed_truck,
														TFlt flow_scalar)
{
	
	m_cell_ID = cell_ID;
	m_cell_length = cell_length;
	m_unit_time = unit_time;
	m_flow_scalar = flow_scalar;

	m_hold_cap_car = hold_cap_car; // Veh/m
	m_hold_cap_truck = hold_cap_truck; // Veh/m
	m_critical_density_car = critical_density_car; // Veh/m
	m_critical_density_truck = critical_density_truck; // Veh/m
	m_rho_1_N = rho_1_N; // Veh/m
	m_flow_cap_car = flow_cap_car; // Veh/s
	m_flow_cap_truck = flow_cap_truck; // Veh/s
	m_ffs_car = ffs_car;
	m_ffs_truck = ffs_truck;
	m_wave_speed_car = wave_speed_car;
	m_wave_speed_truck = wave_speed_truck;

	// initialized as car=1, truck=0
	m_space_fraction_car = TFlt(1);
	m_space_fraction_truck = TFlt(0);

	m_volume_car = TInt(0);
	m_volume_truck = TInt(0);
	m_out_veh_car = TInt(0);
	m_out_veh_truck = TInt(0);
	m_veh_queue_car = std::deque<MNM_Veh*>();
	m_veh_queue_truck = std::deque<MNM_Veh*>();

	// PMC
	m_cell_diff_car = true;
	m_cell_diff_truck = true;

	m_cell_congested_car = int(0);
	m_cell_congested_truck = int(0);
}

MNM_Dlink_Ctm_Multiclass::Ctm_Cell_Multiclass::~Ctm_Cell_Multiclass()
{
	m_veh_queue_car.clear();
	m_veh_queue_truck.clear();
}


int MNM_Dlink_Ctm_Multiclass::Ctm_Cell_Multiclass::update_perceived_density(TFlt _ratio_lane_closure)
{
	TFlt _real_volume_car = TFlt(m_volume_car) / m_flow_scalar;
	TFlt _real_volume_truck = TFlt(m_volume_truck) / m_flow_scalar;

	TFlt _density_car = _real_volume_car / m_cell_length;
	TFlt _density_truck = _real_volume_truck / m_cell_length;

	TFlt _space_fraction_car, _space_fraction_truck;
	// printf("0");
	// Free-flow traffic (free-flow for both car and truck classes)
	if (_density_car/(m_critical_density_car * _ratio_lane_closure) + _density_truck/(m_critical_density_truck * _ratio_lane_closure) <= 1) {
		_space_fraction_car = _density_car/(m_critical_density_car * _ratio_lane_closure);
		_space_fraction_truck = _density_truck/(m_critical_density_truck * _ratio_lane_closure);
		m_perceived_density_car = _density_car + (m_critical_density_car * _ratio_lane_closure) * _space_fraction_truck;
		m_perceived_density_truck = _density_truck + (m_critical_density_truck * _ratio_lane_closure) * _space_fraction_car;
		if (_space_fraction_car + _space_fraction_truck == 0){
			// same to initial values car=1, truck=0
			m_space_fraction_car = 1;
			m_space_fraction_truck = 0;
		}
		else {
			m_space_fraction_car = _space_fraction_car / (_space_fraction_car + _space_fraction_truck);
			m_space_fraction_truck = _space_fraction_truck / (_space_fraction_car + _space_fraction_truck);
		}
		// printf("-1, %.4f, %.4f", m_space_fraction_car, m_space_fraction_truck);
		
		// PMC 
		// cell_congested: 1 - free flow ; 2 - semi-congested ; 3 - fully-congested.
		m_cell_congested_car = int(1);
		m_cell_congested_truck = int(1); 

		// if perceived density approximates critical density, label with non-differentiable (false)
		if (std::abs(m_perceived_density_car - m_critical_density_car * _ratio_lane_closure) <= 0.05 * m_critical_density_car * _ratio_lane_closure){
			m_cell_diff_car = false;
		}
		if (std::abs(m_perceived_density_truck - m_critical_density_truck * _ratio_lane_closure) <= 0.05 * m_perceived_density_truck * _ratio_lane_closure){
			m_cell_diff_truck = false;
		}
	}

	// Semi-congested traffic (truck free-flow but car not)
	else if ((_density_truck / (m_critical_density_truck * _ratio_lane_closure) < 1) && 
			 (_density_car / (1 - _density_truck/ (m_critical_density_truck * _ratio_lane_closure)) <= (m_rho_1_N * _ratio_lane_closure))) {
		_space_fraction_truck = _density_truck/(m_critical_density_truck * _ratio_lane_closure);
		_space_fraction_car = 1 - _space_fraction_truck;
		m_perceived_density_car = _density_car / _space_fraction_car;
		m_perceived_density_truck = (m_critical_density_truck * _ratio_lane_closure);
		m_space_fraction_car = _space_fraction_car;
		m_space_fraction_truck = _space_fraction_truck;
		// printf("-2, %.4f, %.4f", m_space_fraction_car, m_space_fraction_truck);
		m_cell_congested_car = int(2);
		m_cell_congested_truck = int(2);

		// if perceived density approximates critical density, label with non-differentiable (false)
		if (std::abs(m_perceived_density_car - m_critical_density_car * _ratio_lane_closure) <= 0.05 * m_critical_density_car * _ratio_lane_closure){
			m_cell_diff_car = false;
		}
		if (std::abs(m_perceived_density_truck - m_critical_density_truck * _ratio_lane_closure) <= 0.05 * m_perceived_density_truck * _ratio_lane_closure){
			m_cell_diff_truck = false;
		}
	}
	// Fully congested traffic (both car and truck not free-flow)
	// this case should satisfy: 1. m_perceived_density_car > m_rho_1_N
	// 							 2. m_perceived_density_truck > m_critical_density_truck
	else {
		// _density_truck (m_volume_truck) could still be 0
		if (m_volume_truck == 0) {
			m_perceived_density_car = _density_car;
			_space_fraction_car = 1;
			_space_fraction_truck = 0;
			// this case same speed (u) for both private cars and trucks
			TFlt _u = (m_hold_cap_car * _ratio_lane_closure - _density_car) * m_wave_speed_car / _density_car;
			m_perceived_density_truck = (m_hold_cap_truck * _ratio_lane_closure * m_wave_speed_truck) / (_u + m_wave_speed_truck);

			// if perceived density approximates critical density, label with non-differentiable (false)
			if (std::abs(m_perceived_density_car - m_critical_density_car * _ratio_lane_closure) <= 0.05 * m_critical_density_car * _ratio_lane_closure){
				m_cell_diff_car = false;
			}
			if (std::abs(m_perceived_density_truck - m_critical_density_truck * _ratio_lane_closure) <= 0.05 * m_perceived_density_truck * _ratio_lane_closure){
				m_cell_diff_truck = false;
			}
		}
		// _density_car (m_volume_car) could still be 0 in some extreme case
		else if (m_volume_car == 0) {
			m_perceived_density_truck = _density_truck;
			_space_fraction_car = 0;
			_space_fraction_truck = 1;
			// this case same speed (u) for both private cars and trucks
			TFlt _u = (m_hold_cap_truck * _ratio_lane_closure - _density_truck) * m_wave_speed_truck / _density_truck;
			m_perceived_density_car = (m_hold_cap_car * _ratio_lane_closure * m_wave_speed_car) / (_u + m_wave_speed_car);

			// if perceived density approximates critical density, label with non-differentiable (false)
			if (std::abs(m_perceived_density_car - m_critical_density_car * _ratio_lane_closure) <= 0.05 * m_critical_density_car * _ratio_lane_closure){
				m_cell_diff_car = false;
			}
			if (std::abs(m_perceived_density_truck - m_critical_density_truck * _ratio_lane_closure) <= 0.05 * m_critical_density_truck * _ratio_lane_closure){
				m_cell_diff_truck = false;
			}
			
		}
		else {
			TFlt _tmp_1 = m_hold_cap_car * _ratio_lane_closure * m_wave_speed_car * _density_truck;
			TFlt _tmp_2 = m_hold_cap_truck * _ratio_lane_closure * m_wave_speed_truck * _density_car;
			_space_fraction_car = ( _density_car * _density_truck * (m_wave_speed_car - m_wave_speed_truck) 
									 + _tmp_2 ) / ( _tmp_2 + _tmp_1 );
			_space_fraction_truck = ( _density_car * _density_truck * (m_wave_speed_truck - m_wave_speed_car)
									   + _tmp_1 ) / ( _tmp_2 + _tmp_1 );
			m_perceived_density_car = _density_car / _space_fraction_car;
			m_perceived_density_truck = _density_truck / _space_fraction_truck;

			// if perceived density approximates critical density, label with non-differentiable (false)
			if (std::abs(m_perceived_density_car - m_critical_density_car * _ratio_lane_closure) <= 0.05 * m_critical_density_car * _ratio_lane_closure){
				m_cell_diff_car = false;
			}
			if (std::abs(m_perceived_density_truck - m_critical_density_truck * _ratio_lane_closure) <= 0.05 * m_perceived_density_truck * _ratio_lane_closure){
				m_cell_diff_truck = false;
			}
		}
		m_space_fraction_car = _space_fraction_car;
		m_space_fraction_truck = _space_fraction_truck;
		// printf("-3, %.4f, %.4f", m_space_fraction_car, m_space_fraction_truck);
		m_cell_congested_car = int(3);
		m_cell_congested_truck = int(3);
	}
	// printf("\n");
	return 0;
}

TFlt MNM_Dlink_Ctm_Multiclass::Ctm_Cell_Multiclass::get_perceived_demand(TInt veh_type, TFlt _ratio_lane_closure)
{	
	// car
	if (veh_type == TInt(0)) {
		// formula: min(q_m^{c,l}, u_m^F rho) Sean's Part B
		return std::min(TFlt(m_flow_cap_car * _ratio_lane_closure), TFlt(m_ffs_car * m_perceived_density_car)) * m_unit_time;
	}
	// truck
	else {
		return std::min(TFlt(m_flow_cap_truck * _ratio_lane_closure), TFlt(m_ffs_truck * m_perceived_density_truck)) * m_unit_time;
	}
}

TFlt MNM_Dlink_Ctm_Multiclass::Ctm_Cell_Multiclass::get_perceived_supply(TInt veh_type, TFlt _ratio_lane_closure)
{
	TFlt _tmp;
	// car
	if (veh_type == TInt(0)) {
		// formula: min(q_m^{c,r}, w_m * (rho_m^{J,r} - rho)) Sean's Part B
		_tmp = std::min(TFlt(m_flow_cap_car * _ratio_lane_closure), TFlt(m_wave_speed_car * (m_hold_cap_car * _ratio_lane_closure - m_perceived_density_car)));
	}
	// truck
	else {
		_tmp = std::min(TFlt(m_flow_cap_truck * _ratio_lane_closure), TFlt(m_wave_speed_truck * (m_hold_cap_truck * _ratio_lane_closure - m_perceived_density_truck)));
	}
	return std::max(TFlt(0.0), _tmp) * m_unit_time;
}

int MNM_Dlink_Ctm_Multiclass::Ctm_Cell_Multiclass::update_perceived_density_lane_closure(TFlt _ratio_lane_closure)
{
	TFlt _real_volume_car = TFlt(m_volume_car) / m_flow_scalar;
	TFlt _real_volume_truck = TFlt(m_volume_truck) / m_flow_scalar;

	TFlt _density_car = _real_volume_car / m_cell_length;
	TFlt _density_truck = _real_volume_truck / m_cell_length;

	TFlt _space_fraction_car, _space_fraction_truck;
	// printf("0");
	// Free-flow traffic (free-flow for both car and truck classes)
	if (_density_car/(m_critical_density_car * _ratio_lane_closure) + _density_truck/(m_critical_density_truck * _ratio_lane_closure) <= 1) {
		_space_fraction_car = _density_car/(m_critical_density_car * _ratio_lane_closure);
		_space_fraction_truck = _density_truck/(m_critical_density_truck * _ratio_lane_closure);
		m_perceived_density_car = _density_car + (m_critical_density_car * _ratio_lane_closure) * _space_fraction_truck;
		m_perceived_density_truck = _density_truck + (m_critical_density_truck * _ratio_lane_closure) * _space_fraction_car;
		if (_space_fraction_car + _space_fraction_truck == 0){
			// same to initial values car=1, truck=0
			m_space_fraction_car = 1;
			m_space_fraction_truck = 0;
		}
		else {
			m_space_fraction_car = _space_fraction_car / (_space_fraction_car + _space_fraction_truck);
			m_space_fraction_truck = _space_fraction_truck / (_space_fraction_car + _space_fraction_truck);
		}
		// printf("-1, %.4f, %.4f", m_space_fraction_car, m_space_fraction_truck);
		
		// PMC 
		// cell_congested: 1 - free flow ; 2 - semi-congested ; 3 - fully-congested.
		m_cell_congested_car = int(1);
		m_cell_congested_truck = int(1); 

		// if perceived density approximates critical density, label with non-differentiable (false)
		if (std::abs(m_perceived_density_car - m_critical_density_car * _ratio_lane_closure) <= 0.05 * m_critical_density_car * _ratio_lane_closure){
			m_cell_diff_car = false;
		}
		if (std::abs(m_perceived_density_truck - m_critical_density_truck * _ratio_lane_closure) <= 0.05 * m_perceived_density_truck * _ratio_lane_closure){
			m_cell_diff_truck = false;
		}
	}

	// Semi-congested traffic (truck free-flow but car not)
	else if ((_density_truck / (m_critical_density_truck * _ratio_lane_closure) < 1) && 
			 (_density_car / (1 - _density_truck/ (m_critical_density_truck * _ratio_lane_closure)) <= (m_rho_1_N * _ratio_lane_closure))) {
		_space_fraction_truck = _density_truck/(m_critical_density_truck * _ratio_lane_closure);
		_space_fraction_car = 1 - _space_fraction_truck;
		m_perceived_density_car = _density_car / _space_fraction_car;
		m_perceived_density_truck = (m_critical_density_truck * _ratio_lane_closure);
		m_space_fraction_car = _space_fraction_car;
		m_space_fraction_truck = _space_fraction_truck;
		// printf("-2, %.4f, %.4f", m_space_fraction_car, m_space_fraction_truck);
		m_cell_congested_car = int(2);
		m_cell_congested_truck = int(2);

		// if perceived density approximates critical density, label with non-differentiable (false)
		if (std::abs(m_perceived_density_car - m_critical_density_car * _ratio_lane_closure) <= 0.05 * m_critical_density_car * _ratio_lane_closure){
			m_cell_diff_car = false;
		}
		if (std::abs(m_perceived_density_truck - m_critical_density_truck * _ratio_lane_closure) <= 0.05 * m_perceived_density_truck * _ratio_lane_closure){
			m_cell_diff_truck = false;
		}
	}
	// Fully congested traffic (both car and truck not free-flow)
	// this case should satisfy: 1. m_perceived_density_car > m_rho_1_N
	// 							 2. m_perceived_density_truck > m_critical_density_truck
	else {
		// _density_truck (m_volume_truck) could still be 0
		if (m_volume_truck == 0) {
			m_perceived_density_car = _density_car;
			_space_fraction_car = 1;
			_space_fraction_truck = 0;
			// this case same speed (u) for both private cars and trucks
			TFlt _u = (m_hold_cap_car * _ratio_lane_closure - _density_car) * m_wave_speed_car / _density_car;
			m_perceived_density_truck = (m_hold_cap_truck * _ratio_lane_closure * m_wave_speed_truck) / (_u + m_wave_speed_truck);

			// if perceived density approximates critical density, label with non-differentiable (false)
			if (std::abs(m_perceived_density_car - m_critical_density_car * _ratio_lane_closure) <= 0.05 * m_critical_density_car * _ratio_lane_closure){
				m_cell_diff_car = false;
			}
			if (std::abs(m_perceived_density_truck - m_critical_density_truck * _ratio_lane_closure) <= 0.05 * m_perceived_density_truck * _ratio_lane_closure){
				m_cell_diff_truck = false;
			}
		}
		// _density_car (m_volume_car) could still be 0 in some extreme case
		else if (m_volume_car == 0) {
			m_perceived_density_truck = _density_truck;
			_space_fraction_car = 0;
			_space_fraction_truck = 1;
			// this case same speed (u) for both private cars and trucks
			TFlt _u = (m_hold_cap_truck * _ratio_lane_closure - _density_truck) * m_wave_speed_truck / _density_truck;
			m_perceived_density_car = (m_hold_cap_car * _ratio_lane_closure * m_wave_speed_car) / (_u + m_wave_speed_car);

			// if perceived density approximates critical density, label with non-differentiable (false)
			if (std::abs(m_perceived_density_car - m_critical_density_car * _ratio_lane_closure) <= 0.05 * m_critical_density_car * _ratio_lane_closure){
				m_cell_diff_car = false;
			}
			if (std::abs(m_perceived_density_truck - m_critical_density_truck * _ratio_lane_closure) <= 0.05 * m_critical_density_truck * _ratio_lane_closure){
				m_cell_diff_truck = false;
			}
			
		}
		else {
			TFlt _tmp_1 = m_hold_cap_car * _ratio_lane_closure * m_wave_speed_car * _density_truck;
			TFlt _tmp_2 = m_hold_cap_truck * _ratio_lane_closure * m_wave_speed_truck * _density_car;
			_space_fraction_car = ( _density_car * _density_truck * (m_wave_speed_car - m_wave_speed_truck) 
									 + _tmp_2 ) / ( _tmp_2 + _tmp_1 );
			_space_fraction_truck = ( _density_car * _density_truck * (m_wave_speed_truck - m_wave_speed_car)
									   + _tmp_1 ) / ( _tmp_2 + _tmp_1 );
			m_perceived_density_car = _density_car / _space_fraction_car;
			m_perceived_density_truck = _density_truck / _space_fraction_truck;

			// if perceived density approximates critical density, label with non-differentiable (false)
			if (std::abs(m_perceived_density_car - m_critical_density_car * _ratio_lane_closure) <= 0.05 * m_critical_density_car * _ratio_lane_closure){
				m_cell_diff_car = false;
			}
			if (std::abs(m_perceived_density_truck - m_critical_density_truck * _ratio_lane_closure) <= 0.05 * m_perceived_density_truck * _ratio_lane_closure){
				m_cell_diff_truck = false;
			}
		}
		m_space_fraction_car = _space_fraction_car;
		m_space_fraction_truck = _space_fraction_truck;
		// printf("-3, %.4f, %.4f", m_space_fraction_car, m_space_fraction_truck);
		m_cell_congested_car = int(3);
		m_cell_congested_truck = int(3);
	}
	// printf("\n");
	return 0;
}

TFlt MNM_Dlink_Ctm_Multiclass::Ctm_Cell_Multiclass::get_perceived_demand_lane_closure(TInt veh_type, TFlt _ratio_lane_closure)
{	
	// car
	if (veh_type == TInt(0)) {
		// formula: min(q_m^{c,l}, u_m^F rho) Sean's Part B
		return std::min(TFlt(m_flow_cap_car * _ratio_lane_closure), TFlt(m_ffs_car * m_perceived_density_car)) * m_unit_time;
	}
	// truck
	else {
		return std::min(TFlt(m_flow_cap_truck * _ratio_lane_closure), TFlt(m_ffs_truck * m_perceived_density_truck)) * m_unit_time;
	}
}

TFlt MNM_Dlink_Ctm_Multiclass::Ctm_Cell_Multiclass::get_perceived_supply_lane_closure(TInt veh_type, TFlt _ratio_lane_closure)
{
	TFlt _tmp;
	// car
	if (veh_type == TInt(0)) {
		// formula: min(q_m^{c,r}, w_m * (rho_m^{J,r} - rho)) Sean's Part B
		_tmp = std::min(TFlt(m_flow_cap_car * _ratio_lane_closure), TFlt(m_wave_speed_car * (m_hold_cap_car * _ratio_lane_closure - m_perceived_density_car)));
	}
	// truck
	else {
		_tmp = std::min(TFlt(m_flow_cap_truck * _ratio_lane_closure), TFlt(m_wave_speed_truck * (m_hold_cap_truck * _ratio_lane_closure - m_perceived_density_truck)));
	}
	return std::max(TFlt(0.0), _tmp) * m_unit_time;
}


/**************************************************************************
							Multiclass Link-Queue Model
**************************************************************************/
MNM_Dlink_Lq_Multiclass::MNM_Dlink_Lq_Multiclass(TInt ID,
												TInt number_of_lane,
												TFlt length,
												TFlt lane_hold_cap_car,
												TFlt lane_hold_cap_truck,
												TFlt lane_flow_cap_car,
												TFlt lane_flow_cap_truck,
												TFlt ffs_car,
												TFlt ffs_truck,
												TFlt unit_time,
												TFlt veh_convert_factor,
												TFlt flow_scalar)
  : MNM_Dlink_Multiclass::MNM_Dlink_Multiclass(ID, number_of_lane, length, ffs_car, ffs_truck)
{
    m_link_type = MNM_TYPE_LQ_MULTICLASS;
	m_k_j_car = lane_hold_cap_car * number_of_lane;
	m_k_j_truck = lane_hold_cap_truck * number_of_lane;
	m_C_car = lane_flow_cap_car * number_of_lane;
	m_C_truck = lane_flow_cap_truck * number_of_lane;
	m_k_C_car = m_C_car / ffs_car;
	m_k_C_truck = m_C_truck / ffs_truck;
	m_w_car = m_C_car / (m_k_j_car - m_k_C_car);
	m_w_truck = m_C_truck / (m_k_j_truck - m_k_C_truck);
	m_rho_1_N = m_k_j_car * (m_w_car / (m_ffs_truck + m_w_car));
	
	m_veh_queue_car = std::deque<MNM_Veh*>();
	m_veh_queue_truck = std::deque<MNM_Veh*>();
	m_veh_out_buffer_car = std::deque<MNM_Veh*>();
	m_veh_out_buffer_truck = std::deque<MNM_Veh*>();

	// Jiachao added in 03/12/2024 to model parking vehicle for density estimation
	m_veh_parking_car = std::deque<MNM_Veh*>();
	m_veh_parking_truck = std::deque<MNM_Veh*>();
	m_veh_parking_rh = std::deque<MNM_Veh*>();

	m_volume_car = TInt(0);
	m_volume_truck = TInt(0);

	// initialized as car=1, truck=0
	m_space_fraction_car = TFlt(1);
	m_space_fraction_truck = TFlt(0);

	m_flow_scalar = flow_scalar;
	m_unit_time = unit_time;
	m_veh_convert_factor = veh_convert_factor;

	// PMC
	m_congested_car = int(0);
	m_congested_truck = int(0);

	m_diff_car = true;
	m_diff_truck = true;
}

MNM_Dlink_Lq_Multiclass::~MNM_Dlink_Lq_Multiclass()
{
	m_veh_queue_car.clear();
	m_veh_queue_truck.clear();
	m_veh_out_buffer_car.clear();
	m_veh_out_buffer_truck.clear();
	m_veh_parking_car.clear();
	m_veh_parking_truck.clear();
	m_veh_parking_rh.clear();
}

// Jiachao added
int MNM_Dlink_Lq_Multiclass::evolve_curb(TInt timestamp)
{
	// Jiachao added in 03/12/2024 to model parking vehicles 
	MNM_Veh* _veh_p;
	MNM_Veh_Multiclass* _veh_multiclass;

	// printf("Dlink_Lq_Multiclass parking car number = %d\n", int(m_veh_parking_car.size()));

	int _num_parking_car = int(m_veh_parking_car.size());
	for (int i = 0; i < _num_parking_car; ++i){
		_veh_p = m_veh_parking_car.front();
		m_veh_parking_car.pop_front();
		_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass *>(_veh_p);
		int stop_id = _veh_multiclass -> m_arrival_time_list.size();
		// parking complete
		if ((timestamp - _veh_multiclass -> m_arrival_time_list.back()) >= _veh_multiclass->m_parking_duration_list[stop_id - 1]){
			// record departure time
			_veh_multiclass -> m_departure_time_list.push_back(timestamp);
			_veh_multiclass -> m_complete_stop_current_link = TInt(1);
			m_veh_queue_car.push_back(_veh_p);
		}
		// not complete
		else{
			m_veh_parking_car.push_back(_veh_p);
		}
	}
	m_parking_num_car = m_veh_parking_car.size();
	// printf("Dlink_Lq_Multiclass parking car completed\n");

	int _num_parking_truck = int(m_veh_parking_truck.size());
	for (int i = 0; i < _num_parking_truck; ++i){
		_veh_p = m_veh_parking_truck.front();
		m_veh_parking_truck.pop_front();
		_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass *>(_veh_p);
		int stop_id = _veh_multiclass -> m_arrival_time_list.size();
		
		if (stop_id == 0){
			throw std::runtime_error("truck stop_id == 0");
		}
		assert (stop_id == 1);

		// parking complete
		if (_veh_multiclass -> m_parking_duration_list.size() == 0){
			// throw std::runtime_error("truck parking_duration_list.size() == 0");
			_veh_multiclass -> m_parking_duration_list.push_back(TInt(240));
		}

		if ((timestamp - _veh_multiclass -> m_arrival_time_list.back()) >= _veh_multiclass->m_parking_duration_list[stop_id - 1]){
			// record departure time
			_veh_multiclass -> m_departure_time_list.push_back(timestamp);
			_veh_multiclass -> m_complete_stop_current_link = TInt(1);
			m_veh_queue_truck.push_back(_veh_p);
		}
		// not complete
		else{
			m_veh_parking_truck.push_back(_veh_p);
		}
	}
	m_parking_num_truck = m_veh_parking_truck.size();
	// printf("Dlink_Lq_Multiclass parking truck completed\n");

	int _num_parking_rh = int(m_veh_parking_rh.size());
	for (int i = 0; i < _num_parking_rh; ++i){
		_veh_p = m_veh_parking_rh.front();
		m_veh_parking_rh.pop_front();
		_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass *>(_veh_p);
		int stop_id = _veh_multiclass -> m_arrival_time_list.size();
		assert (stop_id == 1);

		// parking complete
		if (_veh_multiclass -> m_parking_duration_list.size() == 0){
			// throw std::runtime_error("truck parking_duration_list.size() == 0");
			_veh_multiclass -> m_parking_duration_list.push_back(TInt(240));
		}

		// parking complete
		if ((timestamp - _veh_multiclass -> m_arrival_time_list.back()) >= _veh_multiclass->m_parking_duration_list[stop_id - 1]){
			// record departure time
			_veh_multiclass -> m_departure_time_list.push_back(timestamp);
			_veh_multiclass -> m_complete_stop_current_link = TInt(1);
			m_veh_queue_car.push_back(_veh_p);
		}
		// not complete
		else{
			m_veh_parking_rh.push_back(_veh_p);
		}
	}
	m_parking_num_rh = m_veh_parking_rh.size();
	// printf("Dlink_Lq_Multiclass parking rh completed\n");

	/* Update volume, perceived density, space fraction, and demand/supply */
	std::deque<MNM_Veh*>::iterator _veh_it;
	TInt _count_car = 0;
	TInt _count_truck = 0;
	TInt _count_tot_vehs = 0;
	for (_veh_it = m_finished_array.begin(); _veh_it != m_finished_array.end(); _veh_it++){
		MNM_Veh_Multiclass *_veh = dynamic_cast<MNM_Veh_Multiclass *>(*_veh_it);
		if (_veh -> m_class == 0) _count_car += 1;
		if (_veh -> m_class == 1) _count_truck += 1;
		if (_veh -> m_class == 2) _count_car += 1;
	}
	m_volume_car = m_veh_queue_car.size() + _count_car;
	m_volume_truck = m_veh_queue_truck.size() + _count_truck;

	/* compute total volume of link, check if spill back */
	_count_tot_vehs = m_volume_car + m_volume_truck * m_veh_convert_factor;
	if (TFlt(_count_tot_vehs)/m_flow_scalar/m_length > m_k_j_car){
		m_spill_back = true;
	}

	update_perceived_density();

	// printf("Dlink_Lq_Multiclass update perceived density completed\n");

	TFlt _demand_car = m_space_fraction_car * std::min(m_C_car, TFlt(m_ffs_car * m_perceived_density_car)) * m_unit_time;
	TFlt _demand_truck = m_space_fraction_truck * std::min(m_C_truck, TFlt(m_ffs_truck * m_perceived_density_truck)) * m_unit_time;
	TFlt _demand = _demand_car + m_veh_convert_factor * _demand_truck;
	TFlt _veh_to_move = _demand * m_flow_scalar - TInt(m_finished_array.size());

	// Move vehicle from queue to buffer
	MNM_Veh *_v;
	TInt _veh_to_move_car = MNM_Ults::round(_veh_to_move * (_demand_car / _demand));
	_veh_to_move_car = std::min(_veh_to_move_car, TInt(m_veh_queue_car.size()));

	TInt _veh_to_move_truck = MNM_Ults::round(_veh_to_move * (m_veh_convert_factor * _demand_truck / _demand) / m_veh_convert_factor);

	_veh_to_move_truck = std::min(_veh_to_move_truck, TInt(m_veh_queue_truck.size()));
	// printf("demand %f, Veh queue size %d, finished size %d, to move %d \n", (float) _demand(), (int) m_veh_queue.size(), (int)m_finished_array.size(), _veh_to_move());
	for (int i = 0; i < _veh_to_move_car; ++i){
		_v = m_veh_queue_car.front();
		m_veh_out_buffer_car.push_back(_v);
		m_veh_queue_car.pop_front();
	}
	for (int i = 0; i < _veh_to_move_truck; ++i){
		_v = m_veh_queue_truck.front();
		m_veh_out_buffer_truck.push_back(_v);
		m_veh_queue_truck.pop_front();
	}

	// Empty buffers, nothing to move to finished array
	if ((m_veh_out_buffer_car.size() == 0) && (m_veh_out_buffer_car.size() == 0)){
		m_tot_wait_time_at_intersection += m_finished_array.size()/m_flow_scalar * m_unit_time;
		return 0;
	}

	// Move vehicles from buffer to finished array
	TInt _num_veh_tomove_car = m_veh_out_buffer_car.size();
	TInt _num_veh_tomove_truck = m_veh_out_buffer_truck.size();
	TFlt _pstar = TFlt(_num_veh_tomove_car)/TFlt(_num_veh_tomove_car + _num_veh_tomove_truck);
	MNM_Veh* _veh;
	TFlt _r;
	while ((_num_veh_tomove_car > 0) || (_num_veh_tomove_truck > 0)){
		_r = MNM_Ults::rand_flt();
		// probability = _pstar to move a car
		if (_r < _pstar){
			// still has car to move
			if (_num_veh_tomove_car > 0){
				_veh = m_veh_out_buffer_car.front();
				m_veh_out_buffer_car.pop_front();
				if (_veh -> has_next_link()){
					m_finished_array.push_back(_veh);
				}
				else {
					printf("Dlink_Lq_Multiclass::Some thing wrong!\n");
					exit(-1);
				}
				_num_veh_tomove_car--;
			}
			// no car to move, move a truck
			else {
				_veh = m_veh_out_buffer_truck.front();
				m_veh_out_buffer_truck.pop_front();
				if (_veh -> has_next_link()){
					m_finished_array.push_back(_veh);
				}
				else {
					printf("Dlink_Lq_Multiclass::Some thing wrong!\n");
					exit(-1);
				}
				_num_veh_tomove_truck--;
			}
		}
		// probability = 1 - _pstar to move a truck
		else {
			// still has truck to move
			if (_num_veh_tomove_truck > 0){
				_veh = m_veh_out_buffer_truck.front();
				m_veh_out_buffer_truck.pop_front();
				if (_veh -> has_next_link()){
					m_finished_array.push_back(_veh);
				}
				else {
					printf("Dlink_Lq_Multiclass::Some thing wrong!\n");
					exit(-1);
				}
				_num_veh_tomove_truck--;
			}
			// no truck to move, move a car
			else {
				_veh = m_veh_out_buffer_car.front();
				m_veh_out_buffer_car.pop_front();
				if (_veh -> has_next_link()){
					m_finished_array.push_back(_veh);
				}
				else {
					printf("Dlink_Lq_Multiclass::Some thing wrong!\n");
					exit(-1);
				}
				_num_veh_tomove_car--;
			}
		}
	}

	// printf("Dlink_Lq_Multiclass move completed\n");

	if ((m_veh_out_buffer_car.size() != 0) || (m_veh_out_buffer_car.size() != 0)){
		printf("Something wrong with our buffer, not empty!\n");
		exit(-1);
	}
	m_tot_wait_time_at_intersection += m_finished_array.size()/m_flow_scalar * m_unit_time;
	return 0;
}

int MNM_Dlink_Lq_Multiclass::update_perceived_density()
{
	TFlt _real_volume_car = TFlt(m_volume_car) / m_flow_scalar;
	TFlt _real_volume_truck = TFlt(m_volume_truck) / m_flow_scalar;

	TFlt _density_car = _real_volume_car / m_length;
	TFlt _density_truck = _real_volume_truck / m_length;

	TFlt _space_fraction_car, _space_fraction_truck;
	// printf("0");
	// Free-flow traffic (free-flow for both car and truck classes)
	if (_density_car/m_k_C_car + _density_truck/m_k_C_truck <= 1) {
		_space_fraction_car = _density_car/m_k_C_car;
		_space_fraction_truck = _density_truck/m_k_C_truck;
		m_perceived_density_car = _density_car + m_k_C_car * _space_fraction_truck;
		m_perceived_density_truck = _density_truck + m_k_C_truck * _space_fraction_car;
		if (_space_fraction_car + _space_fraction_truck == 0){
			// same to initial values: car=1, truck=0
			m_space_fraction_car = 1;
			m_space_fraction_truck = 1;
		}
		else {
			m_space_fraction_car = _space_fraction_car / (_space_fraction_car + _space_fraction_truck);
			m_space_fraction_truck = _space_fraction_truck / (_space_fraction_car + _space_fraction_truck);
		}
		
		// PMC 
		// 1 - free flow ; 2 - semi-congested ; 3 - fully-congested.
		m_congested_car = int(1);
		m_congested_truck = int(1); 

		// if perceived density approximates critical density, label with non-differentiable (false)
		if (std::abs(m_perceived_density_car - m_k_C_car) <= 0.1 * m_k_C_car){
			m_diff_car = false;
		}
		if (std::abs(m_perceived_density_truck - m_k_C_truck) <= 0.1 * m_k_C_truck){
			m_diff_truck = false;
		}
	}
	// Semi-congested traffic (truck free-flow but car not)
	else if ((_density_truck / m_k_C_truck < 1) && 
			 (_density_car / (1 - _density_truck/m_k_C_truck) <= m_rho_1_N)) {
		_space_fraction_truck = _density_truck/m_k_C_truck;
		_space_fraction_car = 1 - _space_fraction_truck;
		m_perceived_density_car = _density_car / _space_fraction_car;
		m_perceived_density_truck = m_k_C_truck;
		m_space_fraction_car = _space_fraction_car;
		m_space_fraction_truck = _space_fraction_truck;
		
		// PMC 
		// cell_congested: 1 - free flow ; 2 - semi-congested ; 3 - fully-congested.
		m_congested_car = int(2);
		m_congested_truck = int(2); 

		// if perceived density approximates critical density, label with non-differentiable (false)
		if (std::abs(m_perceived_density_car - m_k_C_car) <= 0.1 * m_k_C_car){
			m_diff_car = false;
		}
		if (std::abs(m_perceived_density_truck - m_k_C_truck) <= 0.1 * m_k_C_truck){
			m_diff_truck = false;
		}
	}
	// Fully congested traffic (both car and truck not free-flow)
	// this case should satisfy: 1. m_perceived_density_car > m_rho_1_N
	// 							 2. m_perceived_density_truck > m_k_C_truck
	else {
		// _density_truck (m_volume_truck) could still be 0
		if (m_volume_truck == 0) {
			m_perceived_density_car = _density_car;
			_space_fraction_car = 1;
			_space_fraction_truck = 0;
			// this case same speed (u) for both private cars and trucks
			TFlt _u = (m_k_j_car - _density_car) * m_w_car / _density_car;
			m_perceived_density_truck = (m_k_j_truck * m_w_truck) / (_u + m_w_truck);

			// PMC 
			// cell_congested: 1 - free flow ; 2 - semi-congested ; 3 - fully-congested.
			m_congested_car = int(3);
			m_congested_truck = int(3); 

			// if perceived density approximates critical density, label with non-differentiable (false)
			if (std::abs(m_perceived_density_car - m_k_C_car) <= 0.1 * m_k_C_car){
				m_diff_car = false;
			}
			if (std::abs(m_perceived_density_truck - m_k_C_truck) <= 0.1 * m_k_C_truck){
				m_diff_truck = false;
			}
		}
		// _density_car (m_volume_car) could still be 0 in some extreme case
		else if (m_volume_car == 0) {
			m_perceived_density_truck = _density_truck;
			_space_fraction_car = 0;
			_space_fraction_truck = 1;
			// this case same speed (u) for both private cars and trucks
			TFlt _u = (m_k_j_truck - _density_truck) * m_w_truck / _density_truck;
			m_perceived_density_car = (m_k_j_car * m_w_car) / (_u + m_w_car);

			// PMC 
			// cell_congested: 1 - free flow ; 2 - semi-congested ; 3 - fully-congested.
			m_congested_car = int(3);
			m_congested_truck = int(3); 

			// if perceived density approximates critical density, label with non-differentiable (false)
			if (std::abs(m_perceived_density_car - m_k_C_car) <= 0.1 * m_k_C_car){
				m_diff_car = false;
			}
			if (std::abs(m_perceived_density_truck - m_k_C_truck) <= 0.1 * m_k_C_truck){
				m_diff_truck = false;
			}
		}
		else {
			TFlt _tmp_1 = m_k_j_car * m_w_car * _density_truck;
			TFlt _tmp_2 = m_k_j_truck * m_w_truck * _density_car;
			_space_fraction_car = ( _density_car * _density_truck * (m_w_car - m_w_truck) + _tmp_2 ) / ( _tmp_2 + _tmp_1 );
			_space_fraction_truck = ( _density_car * _density_truck * (m_w_truck - m_w_car) + _tmp_1 ) / ( _tmp_2 + _tmp_1 );
			m_perceived_density_car = _density_car / _space_fraction_car;
			m_perceived_density_truck = _density_truck / _space_fraction_truck;

			// PMC 
			// cell_congested: 1 - free flow ; 2 - semi-congested ; 3 - fully-congested.
			m_congested_car = int(3);
			m_congested_truck = int(3); 

			// if perceived density approximates critical density, label with non-differentiable (false)
			if (std::abs(m_perceived_density_car - m_k_C_car) <= 0.1 * m_k_C_car){
				m_diff_car = false;
			}
			if (std::abs(m_perceived_density_truck - m_k_C_truck) <= 0.1 * m_k_C_truck){
				m_diff_truck = false;
			}
		}
		m_space_fraction_car = _space_fraction_car;
		m_space_fraction_truck = _space_fraction_truck;
		// printf("-3, %.4f, %.4f", m_space_fraction_car, m_space_fraction_truck);
	}
	// printf("\n");
	return 0;
}

/* TODO with lane closure */
/* TODO LQ link */
int MNM_Dlink_Lq_Multiclass::evolve_control(TInt timestamp, TFlt _ratio_lane_closure, TFlt cell_position, TFlt cell_control_rate)
{
	std::deque<MNM_Veh*>::iterator _veh_it;
	TInt _count_car = 0;
	TInt _count_truck = 0;
	TInt _count_tot_vehs = 0;
	for (_veh_it = m_finished_array.begin(); _veh_it != m_finished_array.end(); _veh_it++){
		MNM_Veh_Multiclass *_veh = dynamic_cast<MNM_Veh_Multiclass *>(*_veh_it);
		if (_veh -> m_class == 0) _count_car += 1;
		if (_veh -> m_class == 1) _count_truck += 1;
	}
	m_volume_car = m_veh_queue_car.size() + _count_car;
	m_volume_truck = m_veh_queue_truck.size() + _count_truck;

	/* compute total volume of link, check if spill back */
	_count_tot_vehs = m_volume_car + m_volume_truck * m_veh_convert_factor;
	if (TFlt(_count_tot_vehs)/m_flow_scalar/m_length > m_k_j_car){
		m_spill_back = true;
	}

	update_perceived_density();


	TFlt _demand_car = m_space_fraction_car * std::min(m_C_car, TFlt(m_ffs_car * m_perceived_density_car)) * m_unit_time;
	TFlt _demand_truck = m_space_fraction_truck * std::min(m_C_truck, TFlt(m_ffs_truck * m_perceived_density_truck)) * m_unit_time;
	TFlt _demand = _demand_car + m_veh_convert_factor * _demand_truck;
	TFlt _veh_to_move = _demand * m_flow_scalar - TInt(m_finished_array.size());

	// Move vehicle from queue to buffer
	MNM_Veh *_v;
	TInt _veh_to_move_car = MNM_Ults::round(_veh_to_move * (_demand_car / _demand));
	_veh_to_move_car = std::min(_veh_to_move_car, TInt(m_veh_queue_car.size()));

	TInt _veh_to_move_truck = MNM_Ults::round(_veh_to_move * (m_veh_convert_factor * _demand_truck / _demand) / m_veh_convert_factor);

	_veh_to_move_truck = std::min(_veh_to_move_truck, TInt(m_veh_queue_truck.size()));
	// printf("demand %f, Veh queue size %d, finished size %d, to move %d \n", (float) _demand(), (int) m_veh_queue.size(), (int)m_finished_array.size(), _veh_to_move());
	for (int i = 0; i < _veh_to_move_car; ++i){
		_v = m_veh_queue_car.front();
		m_veh_out_buffer_car.push_back(_v);
		m_veh_queue_car.pop_front();
	}
	for (int i = 0; i < _veh_to_move_truck; ++i){
		_v = m_veh_queue_truck.front();
		m_veh_out_buffer_truck.push_back(_v);
		m_veh_queue_truck.pop_front();
	}

	// Empty buffers, nothing to move to finished array
	if ((m_veh_out_buffer_car.size() == 0) && (m_veh_out_buffer_car.size() == 0)){
		m_tot_wait_time_at_intersection += m_finished_array.size()/m_flow_scalar * m_unit_time;
		// separate car and truck
		for (auto _veh_it = m_finished_array.begin(); _veh_it != m_finished_array.end(); _veh_it++){
			MNM_Veh_Multiclass *_veh = dynamic_cast<MNM_Veh_Multiclass *>(*_veh_it);
			if (_veh -> m_class == 0) m_tot_wait_time_at_intersection_car += TFlt(1)/m_flow_scalar * m_unit_time;
			if (_veh -> m_class == 1) m_tot_wait_time_at_intersection_truck += TFlt(1)/m_flow_scalar * m_unit_time;
		}
		return 0;
	}

	// Move vehicles from buffer to finished array
	TInt _num_veh_tomove_car = m_veh_out_buffer_car.size();
	TInt _num_veh_tomove_truck = m_veh_out_buffer_truck.size();
	TFlt _pstar = TFlt(_num_veh_tomove_car)/TFlt(_num_veh_tomove_car + _num_veh_tomove_truck);
	MNM_Veh* _veh;
	TFlt _r;
	while ((_num_veh_tomove_car > 0) || (_num_veh_tomove_truck > 0)){
		_r = MNM_Ults::rand_flt();
		// probability = _pstar to move a car
		if (_r < _pstar){
			// still has car to move
			if (_num_veh_tomove_car > 0){
				_veh = m_veh_out_buffer_car.front();
				m_veh_out_buffer_car.pop_front();
				if (_veh -> has_next_link()){
					m_finished_array.push_back(_veh);
				}
				else {
					printf("Dlink_Lq_Multiclass::Some thing wrong!\n");
					exit(-1);
				}
				_num_veh_tomove_car--;
			}
			// no car to move, move a truck
			else {
				_veh = m_veh_out_buffer_truck.front();
				m_veh_out_buffer_truck.pop_front();
				if (_veh -> has_next_link()){
					m_finished_array.push_back(_veh);
				}
				else {
					printf("Dlink_Lq_Multiclass::Some thing wrong!\n");
					exit(-1);
				}
				_num_veh_tomove_truck--;
			}
		}
		// probability = 1 - _pstar to move a truck
		else {
			// still has truck to move
			if (_num_veh_tomove_truck > 0){
				_veh = m_veh_out_buffer_truck.front();
				m_veh_out_buffer_truck.pop_front();
				if (_veh -> has_next_link()){
					m_finished_array.push_back(_veh);
				}
				else {
					printf("Dlink_Lq_Multiclass::Some thing wrong!\n");
					exit(-1);
				}
				_num_veh_tomove_truck--;
			}
			// no truck to move, move a car
			else {
				_veh = m_veh_out_buffer_car.front();
				m_veh_out_buffer_car.pop_front();
				if (_veh -> has_next_link()){
					m_finished_array.push_back(_veh);
				}
				else {
					printf("Dlink_Lq_Multiclass::Some thing wrong!\n");
					exit(-1);
				}
				_num_veh_tomove_car--;
			}
		}
	}

	if ((m_veh_out_buffer_car.size() != 0) || (m_veh_out_buffer_car.size() != 0)){
		printf("Something wrong with our buffer, not empty!\n");
		exit(-1);
	}
	m_tot_wait_time_at_intersection += m_finished_array.size()/m_flow_scalar * m_unit_time;
	// separate car and truck
	for (auto _veh_it = m_finished_array.begin(); _veh_it != m_finished_array.end(); _veh_it++){
		MNM_Veh_Multiclass *_veh = dynamic_cast<MNM_Veh_Multiclass *>(*_veh_it);
		if (_veh -> m_class == 0) m_tot_wait_time_at_intersection_car += TFlt(1)/m_flow_scalar * m_unit_time;
		if (_veh -> m_class == 1) m_tot_wait_time_at_intersection_truck += TFlt(1)/m_flow_scalar * m_unit_time;
	}
	return 0;
}

int MNM_Dlink_Lq_Multiclass::evolve(TInt timestamp, TFlt _ratio_lane_closure)
{
	// Update volume, perceived density, space fraction, and demand/supply
	// printf("1\n");
	std::deque<MNM_Veh*>::iterator _veh_it;
	TInt _count_car = 0;
	TInt _count_truck = 0;
	TInt _count_tot_vehs = 0;
	for (_veh_it = m_finished_array.begin(); _veh_it != m_finished_array.end(); _veh_it++){
		MNM_Veh_Multiclass *_veh = dynamic_cast<MNM_Veh_Multiclass *>(*_veh_it);
		if (_veh -> m_class == 0) _count_car += 1;
		if (_veh -> m_class == 1) _count_truck += 1;
	}
	m_volume_car = m_veh_queue_car.size() + _count_car;
	m_volume_truck = m_veh_queue_truck.size() + _count_truck;

	/* compute total volume of link, check if spill back */
	_count_tot_vehs = m_volume_car + m_volume_truck * m_veh_convert_factor;
	if (TFlt(_count_tot_vehs)/m_flow_scalar/m_length > m_k_j_car){
		m_spill_back = true;
	}

	update_perceived_density();


	TFlt _demand_car = m_space_fraction_car * std::min(m_C_car, TFlt(m_ffs_car * m_perceived_density_car)) * m_unit_time;
	TFlt _demand_truck = m_space_fraction_truck * std::min(m_C_truck, TFlt(m_ffs_truck * m_perceived_density_truck)) * m_unit_time;
	TFlt _demand = _demand_car + m_veh_convert_factor * _demand_truck;
	TFlt _veh_to_move = _demand * m_flow_scalar - TInt(m_finished_array.size());

	// Move vehicle from queue to buffer
	MNM_Veh *_v;
	TInt _veh_to_move_car = MNM_Ults::round(_veh_to_move * (_demand_car / _demand));
	_veh_to_move_car = std::min(_veh_to_move_car, TInt(m_veh_queue_car.size()));

	// TFlt _raw = _veh_to_move * (m_veh_convert_factor * _demand_truck / _demand) / m_veh_convert_factor;
	// TInt _tmp = std::floor(_raw);
	// _raw = _raw - TFlt(_tmp);
	// TInt _tmp2 = 0;
	// if (_raw < 0.2){
	// 	_tmp2 = 0;
	// }
	// else if (_raw > 0.8){
	// 	_tmp2 = 1;
	// }
	// else {
	// 	_tmp2 = MNM_Ults::round(_raw);
	// }
	// TInt _veh_to_move_truck = _tmp + _tmp2; 
	TInt _veh_to_move_truck = MNM_Ults::round(_veh_to_move * (m_veh_convert_factor * _demand_truck / _demand) / m_veh_convert_factor);

	_veh_to_move_truck = std::min(_veh_to_move_truck, TInt(m_veh_queue_truck.size()));
	// printf("demand %f, Veh queue size %d, finished size %d, to move %d \n", (float) _demand(), (int) m_veh_queue.size(), (int)m_finished_array.size(), _veh_to_move());
	for (int i = 0; i < _veh_to_move_car; ++i){
		_v = m_veh_queue_car.front();
		m_veh_out_buffer_car.push_back(_v);
		m_veh_queue_car.pop_front();
	}
	for (int i = 0; i < _veh_to_move_truck; ++i){
		_v = m_veh_queue_truck.front();
		m_veh_out_buffer_truck.push_back(_v);
		m_veh_queue_truck.pop_front();
	}

	// Empty buffers, nothing to move to finished array
	if ((m_veh_out_buffer_car.size() == 0) && (m_veh_out_buffer_car.size() == 0)){
		m_tot_wait_time_at_intersection += m_finished_array.size()/m_flow_scalar * m_unit_time;
		return 0;
	}

	// Move vehicles from buffer to finished array
	TInt _num_veh_tomove_car = m_veh_out_buffer_car.size();
	TInt _num_veh_tomove_truck = m_veh_out_buffer_truck.size();
	TFlt _pstar = TFlt(_num_veh_tomove_car)/TFlt(_num_veh_tomove_car + _num_veh_tomove_truck);
	MNM_Veh* _veh;
	TFlt _r;
	while ((_num_veh_tomove_car > 0) || (_num_veh_tomove_truck > 0)){
		_r = MNM_Ults::rand_flt();
		// probability = _pstar to move a car
		if (_r < _pstar){
			// still has car to move
			if (_num_veh_tomove_car > 0){
				_veh = m_veh_out_buffer_car.front();
				m_veh_out_buffer_car.pop_front();
				if (_veh -> has_next_link()){
					m_finished_array.push_back(_veh);
				}
				else {
					printf("Dlink_Lq_Multiclass::Some thing wrong!\n");
					exit(-1);
				}
				_num_veh_tomove_car--;
			}
			// no car to move, move a truck
			else {
				_veh = m_veh_out_buffer_truck.front();
				m_veh_out_buffer_truck.pop_front();
				if (_veh -> has_next_link()){
					m_finished_array.push_back(_veh);
				}
				else {
					printf("Dlink_Lq_Multiclass::Some thing wrong!\n");
					exit(-1);
				}
				_num_veh_tomove_truck--;
			}
		}
		// probability = 1 - _pstar to move a truck
		else {
			// still has truck to move
			if (_num_veh_tomove_truck > 0){
				_veh = m_veh_out_buffer_truck.front();
				m_veh_out_buffer_truck.pop_front();
				if (_veh -> has_next_link()){
					m_finished_array.push_back(_veh);
				}
				else {
					printf("Dlink_Lq_Multiclass::Some thing wrong!\n");
					exit(-1);
				}
				_num_veh_tomove_truck--;
			}
			// no truck to move, move a car
			else {
				_veh = m_veh_out_buffer_car.front();
				m_veh_out_buffer_car.pop_front();
				if (_veh -> has_next_link()){
					m_finished_array.push_back(_veh);
				}
				else {
					printf("Dlink_Lq_Multiclass::Some thing wrong!\n");
					exit(-1);
				}
				_num_veh_tomove_car--;
			}
		}
	}

	if ((m_veh_out_buffer_car.size() != 0) || (m_veh_out_buffer_car.size() != 0)){
		printf("Something wrong with our buffer, not empty!\n");
		exit(-1);
	}
	m_tot_wait_time_at_intersection += m_finished_array.size()/m_flow_scalar * m_unit_time;
	return 0;
}

// Jiachao added

TFlt MNM_Dlink_Lq_Multiclass::get_link_capacity()
{
	return m_C_car * m_unit_time;
}

TFlt MNM_Dlink_Lq_Multiclass::get_link_supply()
{
	TFlt _supply_car = m_space_fraction_car * std::min(m_C_car, TFlt(m_w_car * (m_k_j_car - m_perceived_density_car))) * m_unit_time;
	TFlt _supply_truck = m_space_fraction_truck * std::min(m_C_truck, TFlt(m_w_truck * (m_k_j_truck - m_perceived_density_truck))) * m_unit_time;

	// Only for short links, change the FD shape around rhoj:
    _supply_car = std::max(_supply_car, TFlt(m_space_fraction_car * m_w_car * 0.30 * (m_k_j_car - m_k_C_car) * m_unit_time));
    _supply_truck = std::max(_supply_truck, TFlt(m_space_fraction_truck * m_w_truck * 0.30 * (m_k_j_truck - m_k_C_truck) * m_unit_time));

	TFlt _supply = std::max(TFlt(0.0), _supply_car) + m_veh_convert_factor * std::max(TFlt(0.0), _supply_truck);
	return _supply;
}

int MNM_Dlink_Lq_Multiclass::clear_incoming_array(TInt timestamp, TFlt _ratio_lane_closure) {
  	MNM_Veh_Multiclass* _veh;
	size_t _cur_size = m_incoming_array.size();
	// loop for each veh in the incoming array, check if it needs to park at this LQ link
	for (size_t i = 0; i < _cur_size; ++i) {
		_veh = dynamic_cast<MNM_Veh_Multiclass *>(m_incoming_array.front());
		// m_incoming_array.pop_front();
		if (_veh -> m_class != TInt(1)) {
			// if parking at this link, add it to the parking array
			if (_veh -> m_curb_destination_list.front() == this -> m_link_ID){
				if (_veh -> m_class == TInt(0)){
					m_veh_parking_car.push_back(m_incoming_array.front());
					m_incoming_array.pop_front();
					_veh -> m_arrival_time_list.push_back(timestamp);
					// printf("Car parked at link %d\n", int(this -> m_link_ID));
					// this is to avoid weird bug of segmentation fault (some cars park at this link)
					// let this car park for 0 interval
					if (_veh -> m_parking_duration_list.size() == 0){
						_veh -> m_parking_duration_list.push_back(0);
					}
					// printf("car parking destination: %d\n", int(_veh -> m_curb_destination_list.front()));
					// printf("car parking duration size: %d\n", int(_veh -> m_parking_duration_list.size()));
					// printf("car arrival time list length: %d\n", int(_veh -> m_arrival_time_list.size()));
					// printf("car parking duration: %d\n", int(_veh -> m_parking_duration_list[0]));
					// throw std::runtime_error("Car parked at link");
				}
				if (_veh -> m_class == TInt(2)){
					m_veh_parking_rh.push_back(m_incoming_array.front());
					m_incoming_array.pop_front();
					_veh -> m_arrival_time_list.push_back(timestamp);
				}
			}
			// if not parking, add it to queue array
			else{ 
				m_veh_queue_car.push_back(_veh);
				m_incoming_array.pop_front();
			}
		}
		else {
			assert(_veh -> m_class == TInt(1));
			// if parking at this link, add it to the parking array
			if (_veh -> m_curb_destination_list.front() == this -> m_link_ID){
				m_veh_parking_truck.push_back(m_incoming_array.front());
				m_incoming_array.pop_front();
				_veh -> m_arrival_time_list.push_back(timestamp);
			}
			// if not parking, add it to queue array
			else{
				m_veh_queue_truck.push_back(_veh);
				m_incoming_array.pop_front();
			}
		}
		_veh -> m_visual_position_on_link = 0.5;
	}
	return 0;
}

void MNM_Dlink_Lq_Multiclass::print_info()
{
	printf("Link Dynamic model: Multiclass Link Queue\n");
	printf("Total car volume in the link: %.4f\n", (float)(m_volume_car/m_flow_scalar));
	printf("Total truck volume in the link: %.4f\n", (float)(m_volume_truck/m_flow_scalar));
}

TFlt MNM_Dlink_Lq_Multiclass::get_link_flow_car()
{
	return TFlt(m_volume_car) / m_flow_scalar;
}

TFlt MNM_Dlink_Lq_Multiclass::get_link_flow_truck()
{
	return TFlt(m_volume_truck) / m_flow_scalar;
}

TFlt MNM_Dlink_Lq_Multiclass::get_link_flow()
{
	// For get_link_tt in adaptive routing
	return TFlt(m_volume_car + m_volume_truck) / m_flow_scalar;
}

TFlt MNM_Dlink_Lq_Multiclass::get_link_tt()
{
	// For adaptive routing and emissions, need modification for multiclass cases
	TFlt _cost, _spd;
	TFlt _rho  = get_link_flow() / m_number_of_lane / m_length; // get the density in veh/mile
	TFlt _rhoj = m_k_j_car; //get the jam density
	TFlt _rhok = m_k_C_car; //get the critical density
	//  if (abs(rho - rhok) <= 0.0001) cost = POS_INF_INT;
	if (_rho >= _rhoj) {
		_cost = MNM_Ults::max_link_cost(); //sean: i think we should use rhoj, not rhok
	} 
	else {
		if (_rho <= _rhok) {
			_spd = m_ffs_car;
		}
		else {
			_spd = MNM_Ults::max(DBL_EPSILON * m_ffs_car, 
					m_C_car * (_rhoj - _rho) / ((_rhoj - _rhok) * _rho));
		}
		_cost = m_length / _spd;
	} 
	return _cost;
}

TFlt MNM_Dlink_Lq_Multiclass::get_link_tt_from_flow_car(TFlt flow)
{
    TFlt _cost, _spd;
    TFlt _rho  = flow / m_number_of_lane / m_length; // get the density in veh/mile
    TFlt _rhoj = m_k_j_car; //get the jam density
    TFlt _rhok = m_k_C_car; //get the critical density
    //  if (abs(rho - rhok) <= 0.0001) cost = POS_INF_INT;
    if (_rho >= _rhoj) {
        _cost = MNM_Ults::max_link_cost(); //sean: i think we should use rhoj, not rhok
    }
    else {
        if (_rho <= _rhok) {
            _spd = m_ffs_car;
        }
        else {
            _spd = MNM_Ults::max(DBL_EPSILON * m_ffs_car,
                                 m_C_car * (_rhoj - _rho) / ((_rhoj - _rhok) * _rho));
        }
        _cost = m_length / _spd;
    }
    return _cost;
}

TFlt MNM_Dlink_Lq_Multiclass::get_link_tt_from_flow_truck(TFlt flow)
{
    TFlt _cost, _spd;
    TFlt _rho  = flow / m_number_of_lane / m_length; // get the density in veh/mile
    TFlt _rhoj = m_k_j_truck; //get the jam density
    TFlt _rhok = m_k_C_truck; //get the critical density
    //  if (abs(rho - rhok) <= 0.0001) cost = POS_INF_INT;
    if (_rho >= _rhoj) {
        _cost = MNM_Ults::max_link_cost(); //sean: i think we should use rhoj, not rhok
    }
    else {
        if (_rho <= _rhok) {
            _spd = m_ffs_truck;
        }
        else {
            _spd = MNM_Ults::max(DBL_EPSILON * m_ffs_truck,
                                 m_C_truck * (_rhoj - _rho) / ((_rhoj - _rhok) * _rho));
        }
        _cost = m_length / _spd;
    }
    return _cost;
}

TInt MNM_Dlink_Lq_Multiclass::get_link_freeflow_tt_loading_car() {
    // throw std::runtime_error("Error, MNM_Dlink_Lq_Multiclass::get_link_freeflow_tt_loading_car NOT implemented");
	// Jiachao fixed in 0627
    return MNM_Ults::round_up_time(m_length / (m_ffs_car * m_unit_time));
}

TInt MNM_Dlink_Lq_Multiclass::get_link_freeflow_tt_loading_truck() {
    // throw std::runtime_error("Error, MNM_Dlink_Lq_Multiclass::get_link_freeflow_tt_loading_truck NOT implemented");
	// jiachao fixed in 0627
	return MNM_Ults::round_up_time(m_length / (m_ffs_truck * m_unit_time));
}

/**************************************************************************
							Multiclass Point-Queue Model
**************************************************************************/
MNM_Dlink_Pq_Multiclass::MNM_Dlink_Pq_Multiclass(TInt ID,
												TInt number_of_lane,
												TFlt length,
												TFlt lane_hold_cap_car,
												TFlt lane_hold_cap_truck,
												TFlt lane_flow_cap_car,
												TFlt lane_flow_cap_truck,
												TFlt ffs_car,
												TFlt ffs_truck,
												TFlt unit_time,
												TFlt veh_convert_factor,
												TFlt flow_scalar)
  : MNM_Dlink_Multiclass::MNM_Dlink_Multiclass(ID, number_of_lane, length, ffs_car, ffs_truck)
{
    m_link_type = MNM_TYPE_PQ_MULTICLASS;
	// PQ only used for OD connectors, cap/rhoj are all 99999 
	// so no need to use truck parameters
	m_lane_hold_cap = lane_hold_cap_car;
	m_lane_flow_cap = lane_flow_cap_car;
	m_flow_scalar = flow_scalar;
	m_hold_cap = m_lane_hold_cap * TFlt(number_of_lane) * m_length;
	m_max_stamp = MNM_Ults::round(m_length/(ffs_car * unit_time));
	// printf("m_max_stamp = %d\n", m_max_stamp);
	m_veh_pool = std::unordered_map<MNM_Veh*, TInt>();
	m_volume_car = TInt(0);
	m_volume_truck = TInt(0);
	m_unit_time = unit_time;
	m_veh_convert_factor = veh_convert_factor;

	// PMC
	m_congested_car = int(0);
	m_congested_truck = int(0);

	m_diff_car = true;
	m_diff_truck = true;

	m_space_fraction_car = TFlt(1.0);
	m_space_fraction_truck = TFlt(1.0);
}

MNM_Dlink_Pq_Multiclass::~MNM_Dlink_Pq_Multiclass()
{
	m_veh_pool.clear();
}

TFlt MNM_Dlink_Pq_Multiclass::get_link_supply()
{
	return m_lane_flow_cap * TFlt(m_number_of_lane) * m_unit_time;
	// return 9999999;
}

TFlt MNM_Dlink_Pq_Multiclass::get_link_capacity()
{
	return m_lane_flow_cap * TFlt(m_number_of_lane) * m_unit_time;
}

int MNM_Dlink_Pq_Multiclass::clear_incoming_array(TInt timestamp, TFlt _ratio_lane_closure) {
	MNM_Veh_Multiclass *_veh;
	TFlt _to_be_moved = get_link_supply() * m_flow_scalar;
	while (!m_incoming_array.empty()) {
		if ( _to_be_moved > 0){
			_veh = dynamic_cast<MNM_Veh_Multiclass *>(m_incoming_array.front());
			m_incoming_array.pop_front();
			m_veh_pool.insert({_veh, TInt(0)});
			if (_veh -> m_class == 0) {
				//printf("car\n");
				// m_volume_car += 1;
				_to_be_moved -= 1;
			}
			else {
				//printf("truck\n");
				// m_volume_truck += 1;
				// _to_be_moved -= m_veh_convert_factor;
				_to_be_moved -= 1;
			}
		}
		else {
			break;
		}
	}

    m_volume_car = 0;
    m_volume_truck = 0;
    for (auto _veh_it : m_veh_pool){
        auto *_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass *>(_veh_it.first);
        if (_veh_multiclass -> m_class == 0) m_volume_car += 1;
        if (_veh_multiclass -> m_class == 1) m_volume_truck += 1;
    }
    for (auto _veh_it : m_finished_array){
        auto *_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass *>(_veh_it);
        if (_veh_multiclass -> m_class == 0) m_volume_car += 1;
        if (_veh_multiclass -> m_class == 1) m_volume_truck += 1;
    }
	// printf("car: %d, truck: %d\n", m_volume_car, m_volume_truck);
	return 0;
}

void MNM_Dlink_Pq_Multiclass::print_info()
{
	printf("Link Dynamic model: Multiclass Point Queue\n");
	printf("Total car volume in the link: %.4f\n", (float)(m_volume_car/m_flow_scalar));
	printf("Total truck volume in the link: %.4f\n", (float)(m_volume_truck/m_flow_scalar));
}

// Jiachao added
int MNM_Dlink_Pq_Multiclass::evolve_curb(TInt timestamp)
{
	std::unordered_map<MNM_Veh*, TInt>::iterator _que_it = m_veh_pool.begin();
	MNM_Veh_Multiclass* _veh;
	TInt _num_car = 0, _num_truck = 0;
	while (_que_it != m_veh_pool.end()) {
		if (_que_it -> second >= m_max_stamp) {
			m_finished_array.push_back(_que_it -> first);
			_veh = dynamic_cast<MNM_Veh_Multiclass *>(m_finished_array.back());
			if (_veh -> m_class == 0) {
				_num_car += 1;
			}
			else {
				_num_truck += 1;
			}
			_que_it = m_veh_pool.erase(_que_it); //c++ 11
		}
		else {
			_que_it -> second += 1;
			_que_it ++;
		}
	}
	// printf("car: %d, truck: %d\n", _num_car, _num_truck);
	m_tot_wait_time_at_intersection += m_finished_array.size()/m_flow_scalar * m_unit_time;
	return 0;
}

/* TODO with lane closure */
/* TODO PQ link */
int MNM_Dlink_Pq_Multiclass::evolve_control(TInt timestamp, TFlt _ratio_lane_closure, TFlt cell_position, TFlt cell_control_rate)
{
	std::unordered_map<MNM_Veh*, TInt>::iterator _que_it = m_veh_pool.begin();
	MNM_Veh_Multiclass* _veh;
	TInt _num_car = 0, _num_truck = 0;
	while (_que_it != m_veh_pool.end()) {
		if (_que_it -> second >= m_max_stamp) {
			m_finished_array.push_back(_que_it -> first);
			_veh = dynamic_cast<MNM_Veh_Multiclass *>(m_finished_array.back());
			if (_veh -> m_class == 0) {
				_num_car += 1;
			}
			else {
				_num_truck += 1;
			}
			_que_it = m_veh_pool.erase(_que_it); //c++ 11
		}
		else {
			_que_it -> second += 1;
			_que_it ++;
		}
	}
	// printf("car: %d, truck: %d\n", _num_car, _num_truck);
	m_tot_wait_time_at_intersection += m_finished_array.size()/m_flow_scalar * m_unit_time;
	// separate car and truck
	for (auto _veh_it = m_finished_array.begin(); _veh_it != m_finished_array.end(); _veh_it++){
		MNM_Veh_Multiclass *_veh = dynamic_cast<MNM_Veh_Multiclass *>(*_veh_it);
		if (_veh -> m_class == 0) m_tot_wait_time_at_intersection_car += TFlt(1)/m_flow_scalar * m_unit_time;
		if (_veh -> m_class == 1) m_tot_wait_time_at_intersection_truck += TFlt(1)/m_flow_scalar * m_unit_time;
	}

	return 0;
}


int MNM_Dlink_Pq_Multiclass::evolve(TInt timestamp, TFlt _ratio_lane_closure)
{
	std::unordered_map<MNM_Veh*, TInt>::iterator _que_it = m_veh_pool.begin();
	MNM_Veh_Multiclass* _veh;
	TInt _num_car = 0, _num_truck = 0;
	while (_que_it != m_veh_pool.end()) {
		if (_que_it -> second >= m_max_stamp) {
			m_finished_array.push_back(_que_it -> first);
			_veh = dynamic_cast<MNM_Veh_Multiclass *>(m_finished_array.back());
			if (_veh -> m_class == 0) {
				_num_car += 1;
			}
			else {
				_num_truck += 1;
			}
			_que_it = m_veh_pool.erase(_que_it); //c++ 11
		}
		else {
			_que_it -> second += 1;
			_que_it ++;
		}
	}
	// printf("car: %d, truck: %d\n", _num_car, _num_truck);
	m_tot_wait_time_at_intersection += m_finished_array.size()/m_flow_scalar * m_unit_time;
	return 0;
}

TFlt MNM_Dlink_Pq_Multiclass::get_link_flow_car()
{
	return TFlt(m_volume_car) / m_flow_scalar;
}

TFlt MNM_Dlink_Pq_Multiclass::get_link_flow_truck()
{
	return TFlt(m_volume_truck) / m_flow_scalar;
}

TFlt MNM_Dlink_Pq_Multiclass::get_link_flow()
{
	// For adaptive routing, need modification for multiclass case
	return TFlt(m_volume_car + m_volume_truck) / m_flow_scalar;
	// return 0;
}

TFlt MNM_Dlink_Pq_Multiclass::get_link_tt()
{
	return m_length/m_ffs_car;
}

TFlt MNM_Dlink_Pq_Multiclass::get_link_tt_from_flow_car(TFlt flow)
{
    return m_length/m_ffs_car;
}

TFlt MNM_Dlink_Pq_Multiclass::get_link_tt_from_flow_truck(TFlt flow)
{
    return m_length/m_ffs_car;
}

TInt MNM_Dlink_Pq_Multiclass::get_link_freeflow_tt_loading_car()
{
	return m_max_stamp;  // ensure this >= 1 in constructor
}

TInt MNM_Dlink_Pq_Multiclass::get_link_freeflow_tt_loading_truck()
{
	return m_max_stamp;  // ensure this >= 1 in constructor
}

/******************************************************************************************************************
*******************************************************************************************************************
												Node Models
*******************************************************************************************************************
******************************************************************************************************************/

/**************************************************************************
                              Origin node
**************************************************************************/
MNM_DMOND_Multiclass::MNM_DMOND_Multiclass(TInt ID, TFlt flow_scalar, TFlt veh_convert_factor)
	: MNM_DMOND::MNM_DMOND(ID, flow_scalar)
{
	m_veh_convert_factor = veh_convert_factor;
}

MNM_DMOND_Multiclass::~MNM_DMOND_Multiclass()
{
	;
}

// Jiachao added
int MNM_DMOND_Multiclass::evolve_curb(TInt timestamp)
{
	MNM_Dlink *_link;
  	MNM_Veh_Multiclass *_veh;
  	MNM_Dlink_Pq_Multiclass *_next_link;

  	for (unsigned i = 0; i < m_out_link_array.size(); ++i){
    	_link = m_out_link_array[i];
   		m_out_volume[_link] = 0;
  	}

  	/* compute out flow */
  	std::deque<MNM_Veh*>::iterator _que_it = m_in_veh_queue.begin();
  	while (_que_it != m_in_veh_queue.end()) {
  		_veh = dynamic_cast<MNM_Veh_Multiclass *>(*_que_it);
    	_link = _veh -> get_next_link();
    	if (_veh -> m_class == 0){
    		m_out_volume[_link] += 1;
    	}
		if (_veh -> m_class == 1) {
    		// _next_link = dynamic_cast<MNM_Dlink_Pq_Multiclass *>(_link);
    		// m_out_volume[_link] += _next_link -> m_veh_convert_factor;
    		m_out_volume[_link] += 1;
    	}
		if (_veh -> m_class == 2) {
			m_out_volume[_link] += 1;
		}
    	_que_it++;
  	}
  	for (unsigned i = 0; i < m_out_link_array.size(); ++i){
	    _link = m_out_link_array[i];
	    if ((_link -> get_link_supply() * m_flow_scalar) < TFlt(m_out_volume[_link])){
	      	m_out_volume[_link] = TInt(MNM_Ults::round(_link -> get_link_supply() * m_flow_scalar));
	    }
  	}

  	/* move vehicle */
  	TInt _moved_car, _moved_truck, _moved_rh;
  	for (unsigned i = 0; i < m_out_link_array.size(); ++i){
	    _link = m_out_link_array[i];
	    _moved_car = 0;
	    _moved_truck = 0;	
		_moved_rh = 0;    
	    _que_it = m_in_veh_queue.begin();
	    while (_que_it != m_in_veh_queue.end()) {
	      	if (m_out_volume[_link] > 0){
		        _veh = dynamic_cast<MNM_Veh_Multiclass *>(*_que_it);
		        if (_veh -> get_next_link() == _link){
					_link -> m_incoming_array.push_back(_veh);
					_veh -> set_current_link(_link);
					if (_veh -> m_class == 0){
						m_out_volume[_link] -= 1;
						_moved_car += 1;
					}
					// else {
					// 	_next_link = dynamic_cast<MNM_Dlink_Pq_Multiclass *>(_link);
					// 	// m_out_volume[_link] -= _next_link -> m_veh_convert_factor;
					// 	m_out_volume[_link] -= 1;
					// 	_moved_truck += 1;
					// }
					if (_veh -> m_class == 1) {
						// _next_link = dynamic_cast<MNM_Dlink_Pq_Multiclass *>(_link);
						// m_out_volume[_link] -= _next_link -> m_veh_convert_factor;
						m_out_volume[_link] -= 1;
						_moved_truck += 1;
					}
					if (_veh -> m_class == 2) {
						m_out_volume[_link] -= 1;
						_moved_rh += 1;
					}
					_que_it = m_in_veh_queue.erase(_que_it); //c++ 11
		        }
		        else{
		        	_que_it++;
		        }
	      	}
	      	else{
	        	break; //break while loop
	      	}
	    }
	    // record cc for both classes
	    _next_link = dynamic_cast<MNM_Dlink_Pq_Multiclass *>(_link);
		// next link is the Pq link
	    if (_next_link -> m_N_in_car != nullptr) {
	      	_next_link -> m_N_in_car -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp + 1), TFlt(_moved_car)/m_flow_scalar)); 
			// this should be all released cars and rhs
	    }
	    if (_next_link -> m_N_in_truck != nullptr) {
	      	_next_link -> m_N_in_truck -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp + 1), TFlt(_moved_truck)/m_flow_scalar)); 
			// this should be all released trucks
	    }
		if (_next_link -> m_N_in_rh != nullptr) {
			_next_link -> m_N_in_rh -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp + 1), TFlt(_moved_rh)/m_flow_scalar));
		}

		if (_next_link -> m_N_in_car_all != nullptr) {
			_next_link -> m_N_in_car_all -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp + 1), TFlt(_moved_car + _moved_rh)/m_flow_scalar));
		}
	    // printf("car: %d, truck: %d\n", _moved_car, _moved_truck);
  	}
	return 0;
}

int MNM_DMOND_Multiclass::evolve_control(TInt timestamp, 
										 std::unordered_map<std::string, float> *control_map, 
										 std::unordered_map<std::string, std::vector<float>> *control_map_list, 
										 std::unordered_map<TInt, std::vector<TInt>> *lane_closure_map)
{
	MNM_Dlink *_link;
  	MNM_Veh_Multiclass *_veh;
  	MNM_Dlink_Pq_Multiclass *_next_link;

  	for (unsigned i = 0; i < m_out_link_array.size(); ++i){
    	_link = m_out_link_array[i];
   		m_out_volume[_link] = 0;
  	}

  	/* compute out flow */
  	std::deque<MNM_Veh*>::iterator _que_it = m_in_veh_queue.begin();
  	while (_que_it != m_in_veh_queue.end()) {
  		_veh = dynamic_cast<MNM_Veh_Multiclass *>(*_que_it);
    	_link = _veh -> get_next_link();
    	if (_veh -> m_class == 0){
    		m_out_volume[_link] += 1;
    	}
    	else {
    		_next_link = dynamic_cast<MNM_Dlink_Pq_Multiclass *>(_link);
    		// m_out_volume[_link] += _next_link -> m_veh_convert_factor;
    		m_out_volume[_link] += 1;
    	}
    	_que_it++;
  	}
  	for (unsigned i = 0; i < m_out_link_array.size(); ++i){
	    _link = m_out_link_array[i];
	    if ((_link -> get_link_supply() * m_flow_scalar) < TFlt(m_out_volume[_link])){
	      	m_out_volume[_link] = TInt(MNM_Ults::round(_link -> get_link_supply() * m_flow_scalar));
	    }
  	}

  	/* move vehicle */
  	TInt _moved_car, _moved_truck;
  	for (unsigned i = 0; i < m_out_link_array.size(); ++i){
	    _link = m_out_link_array[i];
	    _moved_car = 0;
	    _moved_truck = 0;	    
	    _que_it = m_in_veh_queue.begin();
	    while (_que_it != m_in_veh_queue.end()) {
	      	if (m_out_volume[_link] > 0){
		        _veh = dynamic_cast<MNM_Veh_Multiclass *>(*_que_it);
		        if (_veh -> get_next_link() == _link){
					_link -> m_incoming_array.push_back(_veh);
					_veh -> set_current_link(_link);
					if (_veh -> m_class == 0){
						m_out_volume[_link] -= 1;
						_moved_car += 1;
					}
					else {
						_next_link = dynamic_cast<MNM_Dlink_Pq_Multiclass *>(_link);
						// m_out_volume[_link] -= _next_link -> m_veh_convert_factor;
						m_out_volume[_link] -= 1;
						_moved_truck += 1;
					}
					_que_it = m_in_veh_queue.erase(_que_it); //c++ 11
		        }
		        else{
		        	_que_it++;
		        }
	      	}
	      	else{
	        	break; //break while loop
	      	}
	    }
	    // record cc for both classes
	    _next_link = dynamic_cast<MNM_Dlink_Pq_Multiclass *>(_link);
	    if (_next_link -> m_N_in_car != nullptr) {
	      	_next_link -> m_N_in_car -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp + 1), TFlt(_moved_car)/m_flow_scalar));
	    }
		if (_next_link -> m_N_in_car_all != nullptr) {
	      	_next_link -> m_N_in_car_all -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp + 1), TFlt(_moved_car)/m_flow_scalar));
	    }
	    if (_next_link -> m_N_in_truck != nullptr) {
	      	_next_link -> m_N_in_truck -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp + 1), TFlt(_moved_truck)/m_flow_scalar));
	    }
	    // printf("car: %d, truck: %d\n", _moved_car, _moved_truck);
  	}
	return 0;
}

int MNM_DMOND_Multiclass::evolve(TInt timestamp)
{
  	MNM_Dlink *_link;
  	MNM_Veh_Multiclass *_veh;
  	MNM_Dlink_Pq_Multiclass *_next_link;

  	for (unsigned i = 0; i < m_out_link_array.size(); ++i){
    	_link = m_out_link_array[i];
   		m_out_volume[_link] = 0;
  	}

  	/* compute out flow */
  	std::deque<MNM_Veh*>::iterator _que_it = m_in_veh_queue.begin();
  	while (_que_it != m_in_veh_queue.end()) {
  		_veh = dynamic_cast<MNM_Veh_Multiclass *>(*_que_it);
    	_link = _veh -> get_next_link();
    	if (_veh -> m_class == 0){
    		m_out_volume[_link] += 1;
    	}
    	else {
    		_next_link = dynamic_cast<MNM_Dlink_Pq_Multiclass *>(_link);
    		// m_out_volume[_link] += _next_link -> m_veh_convert_factor;
    		m_out_volume[_link] += 1;
    	}
    	_que_it++;
  	}
  	for (unsigned i = 0; i < m_out_link_array.size(); ++i){
	    _link = m_out_link_array[i];
	    if ((_link -> get_link_supply() * m_flow_scalar) < TFlt(m_out_volume[_link])){
	      	m_out_volume[_link] = TInt(MNM_Ults::round(_link -> get_link_supply() * m_flow_scalar));
	    }
  	}

  	/* move vehicle */
  	TInt _moved_car, _moved_truck;
  	for (unsigned i = 0; i < m_out_link_array.size(); ++i){
	    _link = m_out_link_array[i];
	    _moved_car = 0;
	    _moved_truck = 0;	    
	    _que_it = m_in_veh_queue.begin();
	    while (_que_it != m_in_veh_queue.end()) {
	      	if (m_out_volume[_link] > 0){
		        _veh = dynamic_cast<MNM_Veh_Multiclass *>(*_que_it);
		        if (_veh -> get_next_link() == _link){
					_link -> m_incoming_array.push_back(_veh);
					_veh -> set_current_link(_link);
					if (_veh -> m_class == 0){
						m_out_volume[_link] -= 1;
						_moved_car += 1;
					}
					else {
						_next_link = dynamic_cast<MNM_Dlink_Pq_Multiclass *>(_link);
						// m_out_volume[_link] -= _next_link -> m_veh_convert_factor;
						m_out_volume[_link] -= 1;
						_moved_truck += 1;
					}
					_que_it = m_in_veh_queue.erase(_que_it); //c++ 11
		        }
		        else{
		        	_que_it++;
		        }
	      	}
	      	else{
	        	break; //break while loop
	      	}
	    }
	    // record cc for both classes
	    _next_link = dynamic_cast<MNM_Dlink_Pq_Multiclass *>(_link);
	    if (_next_link -> m_N_in_car != nullptr) {
	      	_next_link -> m_N_in_car -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp + 1), TFlt(_moved_car)/m_flow_scalar));
	    }
	    if (_next_link -> m_N_in_truck != nullptr) {
	      	_next_link -> m_N_in_truck -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp + 1), TFlt(_moved_truck)/m_flow_scalar));
	    }
	    // printf("car: %d, truck: %d\n", _moved_car, _moved_truck);
  	}
  	return 0;
}


/**************************************************************************
                              Destination node
**************************************************************************/
MNM_DMDND_Multiclass::MNM_DMDND_Multiclass(TInt ID, TFlt flow_scalar, TFlt veh_convert_factor)
	: MNM_DMDND::MNM_DMDND(ID, flow_scalar)
{
	m_veh_convert_factor = veh_convert_factor;
}

MNM_DMDND_Multiclass::~MNM_DMDND_Multiclass()
{
	;
}

// Jiachao added
int MNM_DMDND_Multiclass::evolve_curb(TInt timestamp)
{
	MNM_Dlink *_link;
  	MNM_Dlink_Pq_Multiclass *_from_link;
  	MNM_Veh_Multiclass *_veh;
  	size_t _size;
  	TInt _moved_car, _moved_truck, _moved_rh;
  	for (size_t i = 0; i < m_in_link_array.size(); ++i){
  		_moved_car = 0;
	    _moved_truck = 0;
		_moved_rh = 0;
	    _link = m_in_link_array[i];
	    _size = _link -> m_finished_array.size();
	    for (size_t j = 0; j < _size; ++j){
			_veh = dynamic_cast<MNM_Veh_Multiclass *>(_link -> m_finished_array.front());
			if (_veh -> get_next_link() != nullptr){
				printf("Something wrong in DMDND evolve\n");
				exit(-1);
			}
			m_out_veh_queue.push_back(_veh);
			_veh -> set_current_link(nullptr);
			if (_veh -> m_class == 0){
				_moved_car += 1;
			}
			if (_veh -> m_class == 1) {
				_moved_truck += 1;
			}
			if (_veh -> m_class == 2) {
				_moved_rh += 1;
			}
			_link -> m_finished_array.pop_front();
	    }
	    // record cc for both classes evolve_curb
	    _from_link = dynamic_cast<MNM_Dlink_Pq_Multiclass *>(_link);
	    if (_from_link -> m_N_out_car != nullptr) {
	      	_from_link -> m_N_out_car -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp + 1), TFlt(_moved_car)/m_flow_scalar));
	    }
	    if (_from_link -> m_N_out_truck != nullptr) {
	      	_from_link -> m_N_out_truck -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp + 1), TFlt(_moved_truck)/m_flow_scalar));
	    }
		if (_from_link -> m_N_out_rh != nullptr) {
			_from_link -> m_N_out_rh -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp + 1), TFlt(_moved_rh)/m_flow_scalar));
		}
		if (_from_link -> m_N_out_car_all != nullptr) {
			_from_link -> m_N_out_car_all -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp + 1), TFlt(_moved_car + _moved_rh)/m_flow_scalar));
		}
  	}
  	return 0;
}

int MNM_DMDND_Multiclass::evolve_control(TInt timestamp, std::unordered_map<std::string, float> *control_map, std::unordered_map<std::string, std::vector<float>> *control_map_list, std::unordered_map<TInt, std::vector<TInt>> *lane_closure_map)
{
	MNM_Dlink *_link;
  	MNM_Dlink_Pq_Multiclass *_from_link;
  	MNM_Veh_Multiclass *_veh;
  	size_t _size;
  	TInt _moved_car, _moved_truck;
  	for (size_t i = 0; i < m_in_link_array.size(); ++i){
  		_moved_car = 0;
	    _moved_truck = 0;
	    _link = m_in_link_array[i];
	    _size = _link -> m_finished_array.size();
	    for (size_t j = 0; j < _size; ++j){
			_veh = dynamic_cast<MNM_Veh_Multiclass *>(_link -> m_finished_array.front());
			if (_veh -> get_next_link() != nullptr){
				printf("Something wrong in DMDND evolve\n");
				exit(-1);
			}
			m_out_veh_queue.push_back(_veh);
			_veh -> set_current_link(nullptr);
			if (_veh -> m_class == 0){
				_moved_car += 1;
			}
			else {
				_moved_truck += 1;
			}
			_link -> m_finished_array.pop_front();
	    }
	    // record cc for both classes
	    _from_link = dynamic_cast<MNM_Dlink_Pq_Multiclass *>(_link);
	    if (_from_link -> m_N_out_car != nullptr) {
	      	_from_link -> m_N_out_car -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp + 1), TFlt(_moved_car)/m_flow_scalar));
	    }
		if (_from_link -> m_N_out_car_all != nullptr) {
	      	_from_link -> m_N_out_car_all -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp + 1), TFlt(_moved_car)/m_flow_scalar));
	    }
	    if (_from_link -> m_N_out_truck != nullptr) {
	      	_from_link -> m_N_out_truck -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp + 1), TFlt(_moved_truck)/m_flow_scalar));
	    }
  	}
  	return 0;
}

int MNM_DMDND_Multiclass::evolve(TInt timestamp)
{
  	MNM_Dlink *_link;
  	MNM_Dlink_Pq_Multiclass *_from_link;
  	MNM_Veh_Multiclass *_veh;
  	size_t _size;
  	TInt _moved_car, _moved_truck;
  	for (size_t i = 0; i < m_in_link_array.size(); ++i){
  		_moved_car = 0;
	    _moved_truck = 0;
	    _link = m_in_link_array[i];
	    _size = _link -> m_finished_array.size();
	    for (size_t j = 0; j < _size; ++j){
			_veh = dynamic_cast<MNM_Veh_Multiclass *>(_link -> m_finished_array.front());
			if (_veh -> get_next_link() != nullptr){
				printf("Something wrong in DMDND evolve\n");
				exit(-1);
			}
			m_out_veh_queue.push_back(_veh);
			_veh -> set_current_link(nullptr);
			if (_veh -> m_class == 0){
				_moved_car += 1;
			}
			else {
				_moved_truck += 1;
			}
			_link -> m_finished_array.pop_front();
	    }
	    // record cc for both classes
	    _from_link = dynamic_cast<MNM_Dlink_Pq_Multiclass *>(_link);
	    if (_from_link -> m_N_out_car != nullptr) {
	      	_from_link -> m_N_out_car -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp + 1), TFlt(_moved_car)/m_flow_scalar));
	    }
	    if (_from_link -> m_N_out_truck != nullptr) {
	      	_from_link -> m_N_out_truck -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp + 1), TFlt(_moved_truck)/m_flow_scalar));
	    }
  	}
  	return 0;
}


/**************************************************************************
                   				In-out node
**************************************************************************/

/**************************************************************************
what if adding signal control variable

demand of incoming links should be changed

for ramp metering 

D = min(rC, Q) where r is metering rate

for general intersection with signals 

D = min(bC, Q) where b is binary variable indicating link is assigned the right of way or not.

b should vary with time interval 

**************************************************************************/

MNM_Dnode_Inout_Multiclass::MNM_Dnode_Inout_Multiclass(TInt ID, TFlt flow_scalar, TFlt veh_convert_factor)
	: MNM_Dnode::MNM_Dnode(ID, flow_scalar)
{
	m_demand = NULL;
	m_supply = NULL;
	m_veh_flow = NULL;
	m_veh_moved_car = NULL;
	m_veh_moved_truck = NULL;
	m_veh_moved_car_cc = NULL;
	m_veh_moved_truck_cc = NULL;

	m_veh_moved_car_ilink = NULL;
	m_veh_moved_car_ilink_cc = NULL;
	m_veh_moved_truck_ilink = NULL;
	m_veh_moved_truck_ilink_cc = NULL;

	m_veh_convert_factor = veh_convert_factor;
}

MNM_Dnode_Inout_Multiclass::~MNM_Dnode_Inout_Multiclass()
{
  	if (m_demand != NULL) free(m_demand);
  	if (m_supply != NULL) free(m_supply);
  	if (m_veh_flow != NULL) free(m_veh_flow);
  	if (m_veh_moved_car != NULL) free(m_veh_moved_car);
  	if (m_veh_moved_truck != NULL) free(m_veh_moved_truck);
	if (m_veh_moved_car_cc != NULL) free(m_veh_moved_car_cc);
	if (m_veh_moved_truck_cc != NULL) free(m_veh_moved_truck_cc);

	if (m_veh_moved_car_ilink != NULL) free(m_veh_moved_car_ilink);
	if (m_veh_moved_truck_ilink != NULL) free(m_veh_moved_truck_ilink);
	if (m_veh_moved_car_ilink_cc != NULL) free(m_veh_moved_car_ilink_cc);
	if (m_veh_moved_truck_ilink_cc != NULL) free(m_veh_moved_truck_ilink_cc);
}

int MNM_Dnode_Inout_Multiclass::prepare_loading()
{
	TInt _num_in = m_in_link_array.size(); // incoming link number
	TInt _num_out = m_out_link_array.size(); // outgoing link number 
	m_demand = (TFlt*) malloc(sizeof(TFlt) * _num_in * _num_out); // real-world vehicles
	memset(m_demand, 0x0, sizeof(TFlt) * _num_in * _num_out); // matrix??? demand should be 2-D each incoming to each outgoing
	m_supply = (TFlt*) malloc(sizeof(TFlt) * _num_out); // real-world vehicles
	memset(m_supply, 0x0, sizeof(TFlt) * _num_out); // supply is 1-D each outgoing has a supply
	m_veh_flow = (TFlt*) malloc(sizeof(TFlt) * _num_in * _num_out); // real-world vehicles
	memset(m_veh_flow, 0x0, sizeof(TFlt) * _num_in * _num_out);

	m_veh_moved_car = (TFlt*) malloc(sizeof(TFlt) * _num_in * _num_out); // simulation vehicles = real-world vehicles * flow scalar
	memset(m_veh_moved_car, 0x0, sizeof(TFlt) * _num_in * _num_out);

	m_veh_moved_truck = (TFlt*) malloc(sizeof(TFlt) * _num_in * _num_out); // simulation vehicles = real-world vehicles * flow scalar
	memset(m_veh_moved_truck, 0x0, sizeof(TFlt) * _num_in * _num_out);

	m_veh_moved_car_cc = (TFlt*) malloc(sizeof(TFlt) * _num_in * _num_out); // simulation vehicles = real-world vehicles * flow scalar
	memset(m_veh_moved_car_cc, 0x0, sizeof(TFlt) * _num_in * _num_out);

	m_veh_moved_truck_cc = (TFlt*) malloc(sizeof(TFlt) * _num_in * _num_out); // simulation vehicles = real-world vehicles * flow scalar
	memset(m_veh_moved_truck_cc, 0x0, sizeof(TFlt) * _num_in * _num_out);

	m_veh_moved_car_ilink = (TFlt*) malloc(sizeof(TFlt) * _num_in * _num_out);
	memset(m_veh_moved_car_ilink, 0x0, sizeof(TFlt) * _num_in * _num_out);

	m_veh_moved_car_ilink_cc = (TFlt*) malloc(sizeof(TFlt) * _num_in * _num_out);
	memset(m_veh_moved_car_ilink_cc, 0x0, sizeof(TFlt) * _num_in * _num_out);

	m_veh_moved_truck_ilink = (TFlt*) malloc(sizeof(TFlt) * _num_in * _num_out);
	memset(m_veh_moved_truck_ilink, 0x0, sizeof(TFlt) * _num_in * _num_out);
	
	m_veh_moved_truck_ilink_cc = (TFlt*) malloc(sizeof(TFlt) * _num_in * _num_out);
	memset(m_veh_moved_truck_ilink_cc, 0x0, sizeof(TFlt) * _num_in * _num_out);

	return 0;
}

int MNM_Dnode_Inout_Multiclass::prepare_supplyANDdemand_control(TInt timestamp, std::unordered_map<std::string, float> *control_map, std::unordered_map<std::string, std::vector<float>> *control_map_list, std::unordered_map<TInt, std::vector<TInt>> *lane_closure_map)
{
	size_t _num_in = m_in_link_array.size();// incoming link number of the node
	size_t _num_out = m_out_link_array.size();// outgoing link number of the node
	size_t _offset = m_out_link_array.size(); // the offset used to store element in 1-D vector
	TFlt _equiv_count;
	std::deque<MNM_Veh*>::iterator _veh_it;// veh iterator
	MNM_Dlink *_in_link, *_out_link; // initialize in_link and out_link for loop
	std::string _movement_pair_ID;

	/* zerolize num of vehicle moved */
	memset(m_veh_moved_car, 0x0, sizeof(TFlt) * _num_in * _num_out);
	memset(m_veh_moved_truck, 0x0, sizeof(TFlt) * _num_in * _num_out);

	// clock_t start_demand, end_demand;
	// start_demand = clock();
	/* calculate demand */
	// the demand for each incoming link??

	// std::unordered_map<TInt, std::vector<TInt>> _lane_closure_info_all = *lane_closure_map;
	// std::unordered_map<std::string, float> _control_map = *control_map;
	// std::unordered_map<std::string, std::vector<float>> _control_map_list = *control_map_list;

	for (size_t i = 0; i < _num_in; ++i){ // loop for each incoming link
		_in_link = m_in_link_array[i];

		TInt _in_link_ID = _in_link -> m_link_ID;
		TFlt _in_link_capacity;

		auto _check = (*lane_closure_map).find(_in_link_ID);
		if (_check != (*lane_closure_map).end()){

			std::vector<TInt> _lane_closure_info_temp = (*lane_closure_map)[_in_link_ID];

			TInt _lane_num = _lane_closure_info_temp[0];
			TInt _lane_num_change = _lane_closure_info_temp[1];
			TInt _start_int = _lane_closure_info_temp[2];
			TInt _end_int = _lane_closure_info_temp[3];

			if ((timestamp >= _start_int) && (timestamp < _end_int)){

				TFlt _ratio_lane_closure = TFlt(_lane_num_change) / TFlt(_lane_num);
				_in_link_capacity = (_in_link -> get_link_capacity()) * _ratio_lane_closure;
				printf("===== closure for link id = %d =====\n", (int) _in_link_ID);
			}
			else{
				_in_link_capacity = _in_link -> get_link_capacity();
			}
		}
		else{
			_in_link_capacity = _in_link -> get_link_capacity();
		}
		
		// printf("====== link id = %d, capacity = %f ======\n", (int)_in_link ->m_link_ID, (double)_in_link_capacity);
		for (_veh_it = _in_link -> m_finished_array.begin(); _veh_it != _in_link -> m_finished_array.end(); _veh_it++){ // what is this loop for?
			if (std::find(m_out_link_array.begin(), m_out_link_array.end(), (*_veh_it) -> get_next_link()) == m_out_link_array.end()){ 
				printf("Vehicle in the wrong node, no exit!\n");
        		printf("Vehicle is on link %d, node %d, next link ID is: %d\n", _in_link -> m_link_ID(), m_node_ID(), 
        			   (*_veh_it) -> get_next_link() -> m_link_ID());
        		exit(-1);
			}
		}
		for (size_t j = 0; j < _num_out; ++j){
			_out_link = m_out_link_array[j];
			_equiv_count = 0;
			for (_veh_it = _in_link -> m_finished_array.begin(); _veh_it != _in_link -> m_finished_array.end(); _veh_it++){
        		MNM_Veh_Multiclass *_veh = dynamic_cast<MNM_Veh_Multiclass *>(*_veh_it);
        		if (_veh -> get_next_link() == _out_link) {
        			if (_veh -> m_class == 0) {
        				// private car
        				_equiv_count += 1;
        			}
        			else { 
        				// truck
        				_equiv_count += m_veh_convert_factor;
        			}
        		}
      		}
			
			// jiachao added in Apr. 11
			// printf("---- in link ID = %d ----\n" ,int(_in_link -> m_link_ID));

			// printf("---- out link ID = %d ----\n" ,int(_out_link -> m_link_ID));

			// printf("---- current node ID = %d ----\n", int(m_node_ID));
			// control_map

			
			_movement_pair_ID = std::to_string(m_node_ID) + "_" + std::to_string(_in_link -> m_link_ID) + "_" + std::to_string(_out_link -> m_link_ID);

			
			TFlt _control_rate;
			std::vector<float> _control_rates;
			

			auto _check = (*control_map).find(_movement_pair_ID);
			auto _check_list = (*control_map_list).find(_movement_pair_ID);

			
			if (_check != (*control_map).end()){
				_control_rate = (*control_map)[_movement_pair_ID]; 
			}
			if (_check_list != (*control_map_list).end()){
				_control_rates = (*control_map_list)[_movement_pair_ID];
				_control_rate = _control_rates[timestamp];
			}
			else{
				_control_rate = 1.0;
				// printf("====== default 1.0 ======\n");
			}

			// printf("---- movement ID = %s ----\n", _movement_pair_ID.c_str());
			// printf("check rate = %f\n", double(_control_rate));

			// _in_link->m_
			// jiachao fixed at June 3
      		// m_demand[_offset * i + j] = (_equiv_count / m_flow_scalar) * _control_rate;// this is the mixed total flow for car and truck
			
			
			// m_demand[_offset * i + j] = std::min(TFlt(_equiv_count / m_flow_scalar), TFlt(_control_rate * _in_link_capacity / m_flow_scalar));

			// if (TFlt(_equiv_count / m_flow_scalar) > TFlt(_control_rate * _in_link_capacity / m_flow_scalar)){
			// 	printf("===== in_link id = %d has blowed demand of %f but capacity is %f =====\n", (int)_in_link -> m_link_ID, (float)_equiv_count / m_flow_scalar, (float)_control_rate * _in_link_capacity/m_flow_scalar);
			// 	printf("===== using %f =====\n", (float)m_demand[_offset * i + j]);
			// }
			m_demand[_offset * i + j] = TFlt(_equiv_count / m_flow_scalar * _control_rate);
			// printf("---- demand = %f ----\n", m_demand[_offset * i + j]);
			// printf("==== old demand = %f ====\n", _equiv_count / m_flow_scalar);
		}
	}

	// end_demand = clock();

	// std::cout << "demand time = " << double(end_demand - start_demand)/CLOCKS_PER_SEC << "s" << std::endl;

	/* calculating supply */
	// supply for each outgoing link??
	// clock_t start_supply, end_supply;

	// start_supply = clock();

  	for (size_t j = 0; j < _num_out; ++j){
  		// printf("%d, %d\n", j, m_out_link_array[j] -> m_link_ID);
		_out_link = m_out_link_array[j];

		TInt _out_link_ID = _out_link -> m_link_ID;

		auto _check = (*lane_closure_map).find(_out_link_ID);

		if (_check != (*lane_closure_map).end()){

			std::vector<TInt> _lane_closure_info_temp = (*lane_closure_map)[_out_link_ID];

			TInt _lane_num = _lane_closure_info_temp[0];
			TInt _lane_num_change = _lane_closure_info_temp[1];
			TInt _start_int = _lane_closure_info_temp[2];
			TInt _end_int = _lane_closure_info_temp[3];

			if ((timestamp >= _start_int) && (timestamp < _end_int)){

				TFlt _ratio_lane_closure = TFlt(_lane_num_change) / TFlt(_lane_num);
				m_supply[j] = (_out_link -> get_link_supply()) * _ratio_lane_closure;
				printf("===== closure for link id = %d =====\n", (int)_out_link_ID);
			}

			else{
				m_supply[j] = _out_link -> get_link_supply();
			}
		}
		else{
			m_supply[j] = _out_link -> get_link_supply(); // supply should not be changed
		}
  	}
	// end_supply = clock();

	// std::cout<<"supply time = "<<double(end_supply-start_supply)/CLOCKS_PER_SEC<<"s"<<std::endl; 

  	return 0;
}

int MNM_Dnode_Inout_Multiclass::prepare_supplyANDdemand()
{
	size_t _num_in = m_in_link_array.size();// incoming link number of the node
	size_t _num_out = m_out_link_array.size();// outgoing link number of the node
	size_t _offset = m_out_link_array.size(); // the offset used to store element in 1-D vector
	TFlt _equiv_count;
	std::deque<MNM_Veh*>::iterator _veh_it;// veh iterator
	MNM_Dlink *_in_link, *_out_link; // initialize in_link and out_link for loop

	/* zerolize num of vehicle moved */
	memset(m_veh_moved_car, 0x0, sizeof(TFlt) * _num_in * _num_out);
	memset(m_veh_moved_truck, 0x0, sizeof(TFlt) * _num_in * _num_out);

	memset(m_veh_moved_car_cc, 0x0, sizeof(TFlt) * _num_in * _num_out);
	memset(m_veh_moved_truck_cc, 0x0, sizeof(TFlt) * _num_in * _num_out);

	memset(m_veh_moved_car_ilink, 0x0, sizeof(TFlt) * _num_in * _num_out);
	memset(m_veh_moved_car_ilink_cc, 0x0, sizeof(TFlt) * _num_in * _num_out);

	memset(m_veh_moved_truck_ilink, 0x0, sizeof(TFlt) * _num_in * _num_out);
	memset(m_veh_moved_truck_ilink_cc, 0x0, sizeof(TFlt) * _num_in * _num_out);

	/* calculate demand */
	// the demand for each incoming link??
	for (size_t i = 0; i < _num_in; ++i){ // loop for each incoming link
		_in_link = m_in_link_array[i];
		for (_veh_it = _in_link -> m_finished_array.begin(); _veh_it != _in_link -> m_finished_array.end(); _veh_it++){ // what is this loop for?
			if (std::find(m_out_link_array.begin(), m_out_link_array.end(), (*_veh_it) -> get_next_link()) == m_out_link_array.end()){ 
				printf("Vehicle in the wrong node, no exit!\n");
        		printf("Vehicle is on link %d, node %d, next link ID is: %d\n", _in_link -> m_link_ID(), m_node_ID(), 
        			   (*_veh_it) -> get_next_link() -> m_link_ID());
				// MNM_Veh_Multiclass *_veh = dynamic_cast<MNM_Veh_Multiclass *>(*_veh_it);

				// if (_veh -> m_veh_ID == TInt(46)){
				// 	printf("Vehicle ID = %d, which is class %d, and current link is %d\n", (int)_veh -> m_veh_ID, (int)_veh -> m_class, (int)_veh -> m_current_link -> m_link_ID);
				// }
					// printf("last arrival time is %f and last departure time is %f\n", (double)_veh -> m_arrival_time_list.back(), (double)_veh -> m_departure_time_list.back());
        		exit(-1);
			}
		}
		for (size_t j = 0; j < _num_out; ++j){
			_out_link = m_out_link_array[j];
			_equiv_count = 0;
			for (_veh_it = _in_link -> m_finished_array.begin(); _veh_it != _in_link -> m_finished_array.end(); _veh_it++){
        		MNM_Veh_Multiclass *_veh = dynamic_cast<MNM_Veh_Multiclass *>(*_veh_it);
        		if (_veh -> get_next_link() == _out_link) {

					// Jiachao added for curb
        			if ((_veh -> m_class == TInt(0)) || (_veh -> m_class == TInt(2))) {
        				// private car
        				_equiv_count += 1;
        			}
        			else if (_veh -> m_class == TInt(1)){ 
        				// truck
        				_equiv_count += m_veh_convert_factor;
        			}
        		}
      		}
			
      		m_demand[_offset * i + j] = _equiv_count / m_flow_scalar;// this is the mixed total flow for car and truck
			
		}
	}

	/* calculated supply */
  	for (size_t j = 0; j < _num_out; ++j){
  		// printf("%d, %d\n", j, m_out_link_array[j] -> m_link_ID);
	    m_supply[j] = m_out_link_array[j] -> get_link_supply(); // supply should not be changed
  	}

  	return 0;
}

int MNM_Dnode_Inout_Multiclass::move_vehicle_curb(TInt timestamp)
{
	MNM_Dlink *_in_link, *_out_link;
	MNM_Dlink_Multiclass *_ilink, *_olink;
	size_t _offset = m_out_link_array.size();
	TFlt _to_move;
	TFlt _equiv_num;
	TFlt _r;

	// in link array index [0,1,2,...]
	std::vector<size_t> _in_link_ind_array = std::vector<size_t>();
    for (size_t i=0; i<m_in_link_array.size(); ++i){
        _in_link_ind_array.push_back(i);
    }

	// loop for each out link
	for (size_t j = 0; j < m_out_link_array.size(); ++j){
		_out_link = m_out_link_array[j];

        // shuffle the in links, reserve the FIFO (why shuffle? Jiachao)
        std::random_device rng; // random sequence
        std::shuffle(_in_link_ind_array.begin(), _in_link_ind_array.end(), rng);

		// loop for each in link
        for (size_t i : _in_link_ind_array) {
			_in_link = m_in_link_array[i];
			_to_move = m_veh_flow[i * _offset + j] * m_flow_scalar;
			auto _veh_it = _in_link -> m_finished_array.begin();

			while (_veh_it != _in_link -> m_finished_array.end()){
				if (_to_move > 0){
					MNM_Veh_Multiclass *_veh = dynamic_cast<MNM_Veh_Multiclass *>(*_veh_it);
					if (_veh -> get_next_link() == _out_link){
						// printf("%d ", _veh -> m_class);
						if ((_veh -> m_class == TInt(0)) || (_veh -> m_class == TInt(2))) {
							// private car & RH
							_equiv_num = 1;
						}
						if (_veh -> m_class == TInt(1)) { 
							// truck
							_equiv_num = m_veh_convert_factor;
						}
						if (_to_move < _equiv_num) {
							// Randomly decide to move or not in this case base on the probability = _to_move/_equiv_num < 1
							// Will result in WRONG INCOMING ARRAY SIZE if the beginning check in function
							// MNM_Dlink_Ctm_Multiclass::clear_incoming_array() was not commented out (@_@)!
							// _r = MNM_Ults::rand_flt();
							
							// Always move 1 more vehicle _r = 0? not random ??? Jiachao
							_r = 0;
							if (_r <= _to_move/_equiv_num){
								_out_link -> m_incoming_array.push_back(_veh);
								_veh -> set_current_link(_out_link);

								/* for adaptive curb users, delete the first inter-dest if out link is curb choice */
								if ((_veh -> m_type == MNM_TYPE_ADAPTIVE_CURB) && 
								   (_veh -> m_curb_destination_list.front() == _out_link -> m_link_ID)){
									if (_veh -> m_destination_list.front() == _out_link -> m_to_node -> m_node_ID){
										_veh -> m_destination_list.pop_front();
									}
								}

								// driving car
								if (_veh -> m_class == TInt(0)) {
									// m_veh_moved_car[i * _offset + j] += 1;
									_ilink = dynamic_cast<MNM_Dlink_Multiclass *>(_in_link);
									_olink = dynamic_cast<MNM_Dlink_Multiclass *>(_out_link);

									// count curb departure
									// for in_link
									if (_veh -> m_complete_stop_current_link == TInt(1)){
										_veh -> m_complete_stop_current_link = TInt(0);

										_ilink -> m_N_out_car_cc -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(1)/m_flow_scalar));

										if (_ilink -> m_N_out_tree_curb_car != nullptr){
											_ilink -> m_N_out_tree_curb_car -> add_flow(TFlt(timestamp + 1), 1/m_flow_scalar, _veh -> m_path, _veh -> m_assign_interval);
										}

										m_veh_moved_car_ilink_cc[i * _offset + j] += 1;
									}
									else {
										m_veh_moved_car_ilink[i * _offset + j] += 1;
										_ilink -> m_N_out_car -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(1)/m_flow_scalar));
										_ilink -> m_N_out_car_all -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(1)/m_flow_scalar));
									}

									// all cars are inclued in the in_link out cc
									// m_veh_moved_car_ilink[i * _offset + j] += 1;
									// veh not park at out link
									if (_veh -> m_curb_destination_list.front() != _out_link -> m_link_ID){
										if (_olink -> m_N_in_tree_car != nullptr) {
									 		_olink -> m_N_in_tree_car -> add_flow(TFlt(timestamp + 1), 1/m_flow_scalar, _veh -> m_path, _veh -> m_assign_interval);
										}
										m_veh_moved_car[i * _offset + j] += 1;
										_olink -> m_N_in_car -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(1)/m_flow_scalar));
										_olink -> m_N_in_car_all -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(1)/m_flow_scalar));
									}
									else{ // veh will park at out link
										m_veh_moved_car_cc[i * _offset + j] += 1;
										_olink -> m_N_in_car_cc -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(1)/m_flow_scalar));

										if (_olink -> m_N_in_tree_curb_car != nullptr){
											_olink -> m_N_in_tree_curb_car -> add_flow(TFlt(timestamp + 1), 1/m_flow_scalar, _veh -> m_path, _veh -> m_assign_interval);
										}
									}
								}

								// truck mode
								if (_veh -> m_class == TInt(1)) {
									// m_veh_moved_truck[i * _offset + j] += 1;
									_ilink = dynamic_cast<MNM_Dlink_Multiclass *>(_in_link);
									_olink = dynamic_cast<MNM_Dlink_Multiclass *>(_out_link);

									// count curb departure at in link
									if (_veh -> m_complete_stop_current_link == TInt(1)){
										if (_ilink -> m_N_out_tree_curb_truck != nullptr){
											_ilink -> m_N_out_tree_curb_truck -> add_flow(TFlt(timestamp + 1), 1/m_flow_scalar, _veh -> m_path, _veh -> m_assign_interval);
										}
										_veh -> m_complete_stop_current_link = TInt(0);
										m_veh_moved_truck_ilink_cc[i * _offset + j] += 1;
										_ilink -> m_N_out_truck_cc -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(1)/m_flow_scalar));
									}
									else {
										m_veh_moved_truck_ilink[i * _offset + j] += 1;
										_ilink -> m_N_out_truck -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(1)/m_flow_scalar));
									}

									// next link - in flow
									if (_veh -> m_curb_destination_list.front() != _out_link -> m_link_ID){
										if (_olink -> m_N_in_tree_truck != nullptr) {
											_olink -> m_N_in_tree_truck -> add_flow(TFlt(timestamp + 1), 1/m_flow_scalar, _veh -> m_path, _veh -> m_assign_interval);
										}
										m_veh_moved_truck[i * _offset + j] += 1;
										_olink -> m_N_in_truck -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(1)/m_flow_scalar));
									}
									else{
										if (_olink -> m_N_in_tree_curb_truck != nullptr){
											_olink -> m_N_in_tree_curb_truck -> add_flow(TFlt(timestamp + 1), 1/m_flow_scalar, _veh -> m_path, _veh -> m_assign_interval);
										}
										m_veh_moved_truck_cc[i * _offset + j] += 1;
										_olink -> m_N_in_truck_cc -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(1)/m_flow_scalar));
									}
								}

								// ride-hailing mode
								if (_veh -> m_class == TInt(2)) {
									// m_veh_moved_car[i * _offset + j] += 1;
									_ilink = dynamic_cast<MNM_Dlink_Multiclass *>(_in_link);
									_olink = dynamic_cast<MNM_Dlink_Multiclass *>(_out_link);

									// count curb departure
									// for in_link
									if (_veh -> m_complete_stop_current_link == TInt(1)){
										if (_ilink -> m_N_out_tree_curb_rh != nullptr){
											_ilink -> m_N_out_tree_curb_rh -> add_flow(TFlt(timestamp + 1), 1/m_flow_scalar, _veh -> m_path, _veh -> m_assign_interval);
										}
										_veh -> m_complete_stop_current_link = TInt(0);
										m_veh_moved_car_ilink_cc[i * _offset + j] += 1;
										_ilink -> m_N_out_rh_cc -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(1)/m_flow_scalar));
									}
									else {
										m_veh_moved_car_ilink[i * _offset + j] += 1;
										_ilink -> m_N_out_rh -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(1)/m_flow_scalar));
										_ilink -> m_N_out_car_all -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(1)/m_flow_scalar));
									}

									// for out link
									if (_veh -> m_curb_destination_list.front() != _out_link -> m_link_ID){
										if (_olink -> m_N_in_tree_rh != nullptr) {
											_olink -> m_N_in_tree_rh -> add_flow(TFlt(timestamp + 1), 1/m_flow_scalar, _veh -> m_path, _veh -> m_assign_interval);
										}
										m_veh_moved_car[i * _offset + j] += 1;
										_olink -> m_N_in_rh -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(1)/m_flow_scalar));
										_olink -> m_N_in_car_all -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(1)/m_flow_scalar));
									}
									else{
										if (_olink -> m_N_in_tree_curb_rh != nullptr){
											_olink -> m_N_in_tree_curb_rh -> add_flow(TFlt(timestamp + 1), 1/m_flow_scalar, _veh -> m_path, _veh -> m_assign_interval);
										}
										// _veh -> m_curb_destination_list.push_front(_out_link -> m_link_ID);	
										m_veh_moved_car_cc[i * _offset + j] += 1;
										_olink -> m_N_in_rh_cc -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(1)/m_flow_scalar));
									}
								}

								_veh_it = _in_link -> m_finished_array.erase(_veh_it);
							}
						}
						else {
							_out_link -> m_incoming_array.push_back(_veh);
							_veh -> set_current_link(_out_link);

							/* for adaptive curb users, delete the first inter-dest if out link is curb choice */
							if ((_veh -> m_type == MNM_TYPE_ADAPTIVE_CURB) && 
								(_veh -> m_curb_destination_list.front() == _out_link -> m_link_ID)){
								if (_veh -> m_destination_list.front() == _out_link -> m_to_node -> m_node_ID){
									_veh -> m_destination_list.pop_front();
								}
							}
							
							// car
							if (_veh -> m_class == TInt(0)){
								// m_veh_moved_car[i * _offset + j] += 1;
								_ilink = dynamic_cast<MNM_Dlink_Multiclass *>(_in_link);
								_olink = dynamic_cast<MNM_Dlink_Multiclass *>(_out_link);

								// count curb departure
								// for in_link
								if (_veh -> m_complete_stop_current_link == TInt(1)){
									_veh -> m_complete_stop_current_link = TInt(0);
									m_veh_moved_car_ilink_cc[i * _offset + j] += 1;
									_ilink -> m_N_out_car_cc -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(1)/m_flow_scalar));

									if (_ilink -> m_N_out_tree_curb_car != nullptr){
										_ilink -> m_N_out_tree_curb_car -> add_flow(TFlt(timestamp + 1), 1/m_flow_scalar, _veh -> m_path, _veh -> m_assign_interval);
									}
								}
								else {
									m_veh_moved_car_ilink[i * _offset + j] += 1;
									_ilink -> m_N_out_car -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(1)/m_flow_scalar));
									_ilink -> m_N_out_car_all -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(1)/m_flow_scalar));
								}
								// all cars are included in the in_link out cc
								// m_veh_moved_car_ilink[i * _offset + j] += 1;
								
								if (_veh -> m_curb_destination_list.front() != _out_link -> m_link_ID){
									if (_olink -> m_N_in_tree_car != nullptr) {
										_olink -> m_N_in_tree_car -> add_flow(TFlt(timestamp + 1), 1/m_flow_scalar, _veh -> m_path, _veh -> m_assign_interval);
									}
									m_veh_moved_car[i * _offset + j] += 1;
									_olink -> m_N_in_car -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(1)/m_flow_scalar));
									_olink -> m_N_in_car_all -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(1)/m_flow_scalar));
								}
								else{
									m_veh_moved_car_cc[i * _offset + j] += 1;
									_olink -> m_N_in_car_cc -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(1)/m_flow_scalar));
									if (_olink -> m_N_in_tree_curb_car != nullptr){
										_olink -> m_N_in_tree_curb_car -> add_flow(TFlt(timestamp + 1), 1/m_flow_scalar, _veh -> m_path, _veh -> m_assign_interval);
									}
								}
							}
							// truck
							if (_veh -> m_class == TInt(1)) {
								// m_veh_moved_truck[i * _offset + j] += 1;
								_olink = dynamic_cast<MNM_Dlink_Multiclass *>(_out_link);
								_ilink = dynamic_cast<MNM_Dlink_Multiclass *>(_in_link);

								// count curb departure
								if (_veh -> m_complete_stop_current_link == TInt(1)){
									if (_ilink -> m_N_out_tree_curb_truck != nullptr){
										_ilink -> m_N_out_tree_curb_truck -> add_flow(TFlt(timestamp + 1), 1/m_flow_scalar, _veh -> m_path, _veh -> m_assign_interval);
									}
									_veh -> m_complete_stop_current_link = TInt(0);
									
									m_veh_moved_truck_ilink_cc[i * _offset + j] += 1;
									_ilink -> m_N_out_truck_cc -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(1)/m_flow_scalar));
								}
								else {
									m_veh_moved_truck_ilink[i * _offset + j] += 1;
									_ilink -> m_N_out_truck -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(1)/m_flow_scalar));
								}

								if (_veh -> m_curb_destination_list.front() != _out_link -> m_link_ID){
									if (_olink -> m_N_in_tree_truck != nullptr) {
										_olink -> m_N_in_tree_truck -> add_flow(TFlt(timestamp + 1), 1/m_flow_scalar, _veh -> m_path, _veh -> m_assign_interval);
									}
									m_veh_moved_truck[i * _offset + j] += 1;
									_olink -> m_N_in_truck -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(1)/m_flow_scalar));
								}
								else{
									// will stop at out link, count curb arrival
									if (_olink -> m_N_in_tree_curb_truck != nullptr){
										_olink -> m_N_in_tree_curb_truck -> add_flow(TFlt(timestamp + 1), 1/m_flow_scalar, _veh -> m_path, _veh -> m_assign_interval);
									}
									m_veh_moved_truck_cc[i * _offset + j] += 1;
									_olink -> m_N_in_truck_cc -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(1)/m_flow_scalar));
								}
							}
							// ride hailing
							if (_veh -> m_class == TInt(2)) {
								// m_veh_moved_car[i * _offset + j] += 1;
								_olink = dynamic_cast<MNM_Dlink_Multiclass *>(_out_link);
								_ilink = dynamic_cast<MNM_Dlink_Multiclass *>(_in_link);

								// count curb departure
								if (_veh -> m_complete_stop_current_link == TInt(1)){
									if (_ilink -> m_N_out_tree_curb_rh != nullptr){
										_ilink -> m_N_out_tree_curb_rh -> add_flow(TFlt(timestamp + 1), 1/m_flow_scalar, _veh -> m_path, _veh -> m_assign_interval);
									}
									_veh -> m_complete_stop_current_link = TInt(0);
									m_veh_moved_car_ilink_cc[i * _offset + j] += 1;
									_ilink -> m_N_out_rh_cc -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(1)/m_flow_scalar));
								}
								else {
									m_veh_moved_car_ilink[i * _offset + j] += 1;
									_ilink -> m_N_out_rh -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(1)/m_flow_scalar));
									_ilink -> m_N_out_car_all -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(1)/m_flow_scalar));
								}

								if (_veh -> m_curb_destination_list.front() != _out_link -> m_link_ID){
									if (_olink -> m_N_in_tree_rh != nullptr) {
										_olink -> m_N_in_tree_rh -> add_flow(TFlt(timestamp + 1), 1/m_flow_scalar, _veh -> m_path, _veh -> m_assign_interval);
									}
									m_veh_moved_car[i * _offset + j] += 1;
									_olink -> m_N_in_rh -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(1)/m_flow_scalar));
									_olink -> m_N_in_car_all -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(1)/m_flow_scalar));
								}
								else{
									if (_olink -> m_N_in_tree_curb_rh != nullptr){
										_olink -> m_N_in_tree_curb_rh -> add_flow(TFlt(timestamp + 1), 1/m_flow_scalar, _veh -> m_path, _veh -> m_assign_interval);
									}
									m_veh_moved_car_cc[i * _offset + j] += 1;
									_olink -> m_N_in_rh_cc -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(1)/m_flow_scalar));
								}
							}
							_veh_it = _in_link -> m_finished_array.erase(_veh_it);
						}
						_to_move -= _equiv_num;
					}
					else {
						_veh_it++;
					}
				}
				else {
					break;
				}
			}
			// printf("\n");
			if (_to_move > 0.001){
				printf("Something wrong during the vehicle moving, remaining to move %.16f\n", (float)_to_move);
				// printf("The finished veh queue is now size %d\n", (int)_in_link->m_finished_array.size());
				// printf("But it is heading to %d\n", (int)_in_link->m_finished_array.front() -> get_next_link() -> m_link_ID);
				exit(-1);
			}
		}
        // make the queue randomly perturbed, may not be true in signal controlled intersection, violate FIFO
        // random_shuffle(_out_link -> m_incoming_array.begin(), _out_link -> m_incoming_array.end());
	}
    _in_link_ind_array.clear();
	return 0;
}

int MNM_Dnode_Inout_Multiclass::move_vehicle(TInt timestamp)
{
	MNM_Dlink *_in_link, *_out_link;
	// MNM_Dlink_Multiclass *_ilink, *_olink;
	MNM_Dlink_Multiclass *_olink, *_ilink;
	size_t _offset = m_out_link_array.size();
	TFlt _to_move;
	TFlt _equiv_num;
	TFlt _r;

    std::vector<size_t> _in_link_ind_array = std::vector<size_t>();
    for (size_t i=0; i<m_in_link_array.size(); ++i){
        _in_link_ind_array.push_back(i);
    }

	for (size_t j = 0; j < m_out_link_array.size(); ++j){
		_out_link = m_out_link_array[j];

        // shuffle the in links, reserve the FIFO
        std::random_device rng; // random sequence
        std::shuffle(_in_link_ind_array.begin(), _in_link_ind_array.end(), rng);
        for (size_t i : _in_link_ind_array) {
		// for (size_t i = 0; i < m_in_link_array.size(); ++i){
			_in_link = m_in_link_array[i];
			_to_move = m_veh_flow[i * _offset + j] * m_flow_scalar;
			// printf("from %d to %d: %.4f\n", int(_in_link -> m_link_ID), int(_out_link -> m_link_ID), double(_to_move));
			auto _veh_it = _in_link -> m_finished_array.begin();

			while (_veh_it != _in_link -> m_finished_array.end()){
				if (_to_move > 0){
					MNM_Veh_Multiclass *_veh = dynamic_cast<MNM_Veh_Multiclass *>(*_veh_it);
					if (_veh -> get_next_link() == _out_link){
						// printf("%d ", _veh -> m_class);
						if (_veh -> m_class == 0) {
							// private car
							_equiv_num = 1;
						}
						else { 
							// truck
							_equiv_num = m_veh_convert_factor;
							// _equiv_num = 1;
						}
						if (_to_move < _equiv_num) {
							// Randomly decide to move or not in this case base on the probability = _to_move/_equiv_num < 1
							// Will result in WRONG INCOMING ARRAY SIZE if the beginning check in function
							// MNM_Dlink_Ctm_Multiclass::clear_incoming_array() was not commented out (@_@)!
							//_r = MNM_Ults::rand_flt();
							
							// Always move 1 more vehicle
							_r = 0;
							if (_r <= _to_move/_equiv_num){
								_out_link -> m_incoming_array.push_back(_veh);
								_veh -> set_current_link(_out_link);
								_olink = dynamic_cast<MNM_Dlink_Multiclass *>(_out_link);
								_ilink = dynamic_cast<MNM_Dlink_Multiclass *>(_in_link);
								if (_veh -> m_class == 0){
									m_veh_moved_car[i * _offset + j] += 1;
									if (_olink -> m_N_in_tree_car != nullptr) {
									 	_olink -> m_N_in_tree_car -> add_flow(TFlt(timestamp + 1), 1/m_flow_scalar, _veh -> m_path, _veh -> m_assign_interval);
									}
									if (_ilink -> m_N_out_tree_car != nullptr) {
										_ilink -> m_N_out_tree_car -> add_flow(TFlt(timestamp + 1), 1/m_flow_scalar, _veh -> m_path, _veh -> m_assign_interval);
									}
								}

								else {
									IAssert(_veh -> m_class == 1);
									m_veh_moved_truck[i * _offset + j] += 1;
									if (_olink -> m_N_in_tree_truck != nullptr) {
									 	_olink -> m_N_in_tree_truck -> add_flow(TFlt(timestamp + 1), 1/m_flow_scalar, _veh -> m_path, _veh -> m_assign_interval);
									}
									if (_ilink -> m_N_out_tree_truck != nullptr) {
										_ilink -> m_N_out_tree_truck -> add_flow(TFlt(timestamp + 1), 1/m_flow_scalar, _veh -> m_path, _veh -> m_assign_interval);
									}
								}
								_veh_it = _in_link -> m_finished_array.erase(_veh_it);
							}
						}
						else {
							_out_link -> m_incoming_array.push_back(_veh);
							_veh -> set_current_link(_out_link);
							_olink = dynamic_cast<MNM_Dlink_Multiclass *>(_out_link);
							_ilink = dynamic_cast<MNM_Dlink_Multiclass *>(_in_link);
							if (_veh -> m_class == 0){
								m_veh_moved_car[i * _offset + j] += 1;
								
								if (_olink -> m_N_in_tree_car != nullptr) {
								 	_olink -> m_N_in_tree_car -> add_flow(TFlt(timestamp + 1), 1/m_flow_scalar, _veh -> m_path, _veh -> m_assign_interval);
								}
								if (_ilink -> m_N_out_tree_car != nullptr) {
									_ilink -> m_N_out_tree_car -> add_flow(TFlt(timestamp + 1), 1/m_flow_scalar, _veh -> m_path, _veh -> m_assign_interval);
								}
							}
							else {
								IAssert(_veh -> m_class == 1);
								m_veh_moved_truck[i * _offset + j] += 1;
								if (_olink -> m_N_in_tree_truck != nullptr) {
								 	_olink -> m_N_in_tree_truck -> add_flow(TFlt(timestamp + 1), 1/m_flow_scalar, _veh -> m_path, _veh -> m_assign_interval);
								}
								if (_ilink -> m_N_out_tree_truck != nullptr) {
									_ilink -> m_N_out_tree_truck -> add_flow(TFlt(timestamp + 1), 1/m_flow_scalar, _veh -> m_path, _veh -> m_assign_interval);
								}
							}
							_veh_it = _in_link -> m_finished_array.erase(_veh_it);
						}
						_to_move -= _equiv_num;
					}
					else {
						_veh_it++;
					}
				}
				else {
					break;
				}
			}
			// printf("\n");
			if (_to_move > 0.001){
				printf("Something wrong during the vehicle moving, remaining to move %.16f\n", (float)_to_move);
				// printf("The finished veh queue is now size %d\n", (int)_in_link->m_finished_array.size());
				// printf("But it is heading to %d\n", (int)_in_link->m_finished_array.front() -> get_next_link() -> m_link_ID);
				exit(-1);
			}
		}
        // make the queue randomly perturbed, may not be true in signal controlled intersection, violate FIFO
        // random_shuffle(_out_link -> m_incoming_array.begin(), _out_link -> m_incoming_array.end());
	}
    _in_link_ind_array.clear();
	return 0;
}

int MNM_Dnode_Inout_Multiclass::record_cumulative_curve(TInt timestamp)
{
  	TInt _temp_sum_car, _temp_sum_truck;
  	MNM_Dlink_Multiclass *_in_link, *_out_link;
  	size_t _offset = m_out_link_array.size();

  	for (size_t j = 0; j < m_out_link_array.size(); ++j){
    	_temp_sum_car = 0;
    	_temp_sum_truck = 0;
    	_out_link = dynamic_cast<MNM_Dlink_Multiclass *>(m_out_link_array[j]);
    	for (size_t i = 0; i < m_in_link_array.size(); ++i) {
    		// _in_link = dynamic_cast<MNM_Dlink_Multiclass *>(m_in_link_array[i]);
       		_temp_sum_car += m_veh_moved_car[i * _offset + j];
      		_temp_sum_truck += m_veh_moved_truck[i * _offset + j];
    	}
    	if (_out_link -> m_N_in_car != nullptr) {
      		_out_link -> m_N_in_car -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(_temp_sum_car)/m_flow_scalar));
    	}
		if (_out_link -> m_N_in_car_all != nullptr){
			_out_link -> m_N_in_car_all -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(_temp_sum_car)/m_flow_scalar));
		}
    	if (_out_link -> m_N_in_truck != nullptr) {
      		_out_link -> m_N_in_truck -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(_temp_sum_truck)/m_flow_scalar));
    	}
  	}

  	for (size_t i = 0; i < m_in_link_array.size(); ++i){
    	_temp_sum_car = 0;
    	_temp_sum_truck = 0;
    	_in_link = dynamic_cast<MNM_Dlink_Multiclass *>(m_in_link_array[i]);
    	for (size_t j = 0; j < m_out_link_array.size(); ++j) {
      		// _out_link = dynamic_cast<MNM_Dlink_Multiclass *>(m_out_link_array[j]);
      		_temp_sum_car += m_veh_moved_car[i * _offset + j];
      		_temp_sum_truck += m_veh_moved_truck[i * _offset + j];
    	}
    	if (_in_link -> m_N_out_car != nullptr) {
      		_in_link -> m_N_out_car -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(_temp_sum_car)/m_flow_scalar));
    	}
		if (_in_link -> m_N_out_car_all != nullptr) {
      		_in_link -> m_N_out_car_all -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(_temp_sum_car)/m_flow_scalar));
    	}
    	if (_in_link -> m_N_out_truck != nullptr) {
      		_in_link -> m_N_out_truck -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(_temp_sum_truck)/m_flow_scalar));
    	}
  	}
  	return 0;
}

int MNM_Dnode_Inout_Multiclass::record_cumulative_curve_curb(TInt timestamp)
{
	TInt _temp_sum_car, _temp_sum_truck;

	TInt _temp_sum_car_cc, _temp_sum_truck_cc;

  	MNM_Dlink_Multiclass *_in_link, *_out_link;
  	size_t _offset = m_out_link_array.size();

	// record out links m_N_in_car/truck
  	for (size_t j = 0; j < m_out_link_array.size(); ++j){
    	_temp_sum_car = 0;
    	_temp_sum_truck = 0;

		_temp_sum_car_cc = 0;
    	_temp_sum_truck_cc = 0;

    	_out_link = dynamic_cast<MNM_Dlink_Multiclass *>(m_out_link_array[j]);
    	for (size_t i = 0; i < m_in_link_array.size(); ++i) {
    		// _in_link = dynamic_cast<MNM_Dlink_Multiclass *>(m_in_link_array[i]);
       		_temp_sum_car += m_veh_moved_car[i * _offset + j];
      		_temp_sum_truck += m_veh_moved_truck[i * _offset + j];

			_temp_sum_car_cc += m_veh_moved_car_cc[i * _offset + j];
      		_temp_sum_truck_cc += m_veh_moved_truck_cc[i * _offset + j];
    	}
    	if (_out_link -> m_N_in_car != nullptr) {
      		_out_link -> m_N_in_car -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(_temp_sum_car)/m_flow_scalar)); // curb
    	}
    	if (_out_link -> m_N_in_truck != nullptr) {
      		_out_link -> m_N_in_truck -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(_temp_sum_truck)/m_flow_scalar)); // curb
    	}
		if (_out_link -> m_N_in_car_cc != nullptr) {
      		_out_link -> m_N_in_car_cc -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(_temp_sum_car_cc)/m_flow_scalar)); // curb
    	}
    	if (_out_link -> m_N_in_truck_cc != nullptr) {
      		_out_link -> m_N_in_truck_cc -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(_temp_sum_truck_cc)/m_flow_scalar)); // curb
    	}
  	}

	// // record in links m_N_out_car/truck
  	for (size_t i = 0; i < m_in_link_array.size(); ++i){
    	_temp_sum_car = 0;
    	_temp_sum_truck = 0;

		_temp_sum_car_cc = 0;
    	_temp_sum_truck_cc = 0;

    	_in_link = dynamic_cast<MNM_Dlink_Multiclass *>(m_in_link_array[i]);
    	for (size_t j = 0; j < m_out_link_array.size(); ++j) {
      		// _out_link = dynamic_cast<MNM_Dlink_Multiclass *>(m_out_link_array[j]);
      		_temp_sum_car += m_veh_moved_car_ilink[i * _offset + j];
      		_temp_sum_truck += m_veh_moved_truck_ilink[i * _offset + j];

			_temp_sum_car_cc += m_veh_moved_car_ilink_cc[i * _offset + j];
      		_temp_sum_truck_cc += m_veh_moved_truck_ilink_cc[i * _offset + j];
    	}
    	if (_in_link -> m_N_out_car != nullptr) {
      		_in_link -> m_N_out_car -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(_temp_sum_car)/m_flow_scalar));
    	}
    	if (_in_link -> m_N_out_truck != nullptr) {
      		_in_link -> m_N_out_truck -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(_temp_sum_truck)/m_flow_scalar));
    	}
		if (_in_link -> m_N_out_car_cc != nullptr) {
      		_in_link -> m_N_out_car_cc -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(_temp_sum_car_cc)/m_flow_scalar));
    	}
    	if (_in_link -> m_N_out_truck_cc != nullptr) {
      		_in_link -> m_N_out_truck_cc -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(_temp_sum_truck_cc)/m_flow_scalar));
    	}
  	}
	return 0;
}

int MNM_Dnode_Inout_Multiclass::add_out_link(MNM_Dlink* out_link)
{
  	m_out_link_array.push_back(out_link);
  	return 0;
}

int MNM_Dnode_Inout_Multiclass::add_in_link(MNM_Dlink *in_link)
{
  	m_in_link_array.push_back(in_link);
  	return 0;
}

// Jiachao added
int MNM_Dnode_Inout_Multiclass::evolve_curb(TInt timestamp)
{
	prepare_supplyANDdemand();
	// prepare_supplyANDdemand_control(timestamp, control_map, control_map_list, lane_closure_map);
	
	// end = clock();
	// std::cout << "prepare supply and demand time = " << double(end-start)/CLOCKS_PER_SEC << "s" << std::endl;
	
	// printf("control works\n"); 

	// start = clock();

	compute_flow();

	// end = clock();
	// std::cout << "compute flow time = " << double(end-start)/CLOCKS_PER_SEC << "s" << std::endl;

	// // printf("3\n");
	// start = clock();

	move_vehicle_curb(timestamp);

	// end = clock();
	// std::cout << "move vehicle time = " << double(end-start)/CLOCKS_PER_SEC << "s" << std::endl;

	// // printf("4\n");
	// start = clock();

	// record_cumulative_curve_curb(timestamp);

	return 0;
}


int MNM_Dnode_Inout_Multiclass::evolve_control(TInt timestamp, std::unordered_map<std::string, float> *control_map, std::unordered_map<std::string, std::vector<float>> *control_map_list, std::unordered_map<TInt, std::vector<TInt>> *lane_closure_map)
{
	// printf("1\n");
	// clock_t start, end;
	// start = clock();
	
	prepare_supplyANDdemand_control(timestamp, control_map, control_map_list, lane_closure_map);
	
	// end = clock();
	// std::cout << "prepare supply and demand time = " << double(end-start)/CLOCKS_PER_SEC << "s" << std::endl;
	
	// printf("control works\n"); 

	// start = clock();

	compute_flow();

	// end = clock();
	// std::cout << "compute flow time = " << double(end-start)/CLOCKS_PER_SEC << "s" << std::endl;

	// // printf("3\n");
	// start = clock();

	move_vehicle(timestamp);

	// end = clock();
	// std::cout << "move vehicle time = " << double(end-start)/CLOCKS_PER_SEC << "s" << std::endl;

	// // printf("4\n");
	// start = clock();

	record_cumulative_curve(timestamp);

	// end = clock();
	// std::cout << "record time = " << double(end-start)/CLOCKS_PER_SEC << "s" << std::endl;
	return 0;
}

int MNM_Dnode_Inout_Multiclass::evolve(TInt timestamp)
{
	// printf("Inout node evolve\n");
	// printf("1\n");
	prepare_supplyANDdemand();
	// printf("2\n"); 
	compute_flow();
	// printf("3\n");
	move_vehicle(timestamp);
	// printf("4\n");
	record_cumulative_curve(timestamp);
	// printf("5\n");
	return 0;
}

/*                          FWJ node
**************************************************************************/
MNM_Dnode_FWJ_Multiclass::MNM_Dnode_FWJ_Multiclass(TInt ID, TFlt flow_scalar, TFlt veh_convert_factor)
  : MNM_Dnode_Inout_Multiclass::MNM_Dnode_Inout_Multiclass(ID, flow_scalar, veh_convert_factor)
{
}

MNM_Dnode_FWJ_Multiclass::~MNM_Dnode_FWJ_Multiclass()
{
	;
}

int MNM_Dnode_FWJ_Multiclass::compute_flow()
{
	size_t _offset = m_out_link_array.size();
	TFlt _sum_in_flow, _portion;
	for (size_t j = 0; j < m_out_link_array.size(); ++j){
		_sum_in_flow = TFlt(0);
		for (size_t i = 0; i < m_in_link_array.size(); ++i){
	  		_sum_in_flow += m_demand[i * _offset + j];
		}
		for (size_t i = 0; i < m_in_link_array.size(); ++i){
	  		_portion = MNM_Ults::divide(m_demand[i * _offset + j], _sum_in_flow);
		  	m_veh_flow[i * _offset + j] = MNM_Ults::min(m_demand[i * _offset + j], _portion * m_supply[j]);
		}
	}

	return 0;
}

/*               General Road Junction node
**************************************************************************/
MNM_Dnode_GRJ_Multiclass::MNM_Dnode_GRJ_Multiclass(TInt ID, TFlt flow_scalar, TFlt veh_convert_factor)
  : MNM_Dnode_Inout_Multiclass::MNM_Dnode_Inout_Multiclass(ID, flow_scalar, veh_convert_factor)
{
	m_d_a = nullptr;
	m_C_a = nullptr;
}

MNM_Dnode_GRJ_Multiclass::~MNM_Dnode_GRJ_Multiclass()
{
	if (m_d_a != nullptr) free(m_d_a);
	if (m_C_a != nullptr) free(m_C_a);
}

int MNM_Dnode_GRJ_Multiclass::prepare_loading()
{
	MNM_Dnode_Inout_Multiclass::prepare_loading();
	TInt _num_in = m_in_link_array.size();
	m_d_a = (TFlt*) malloc(sizeof(TFlt) * _num_in);
	memset(m_d_a, 0x0, sizeof(TFlt) * _num_in);
	m_C_a = (TFlt*) malloc(sizeof(TFlt) * _num_in);
	memset(m_C_a, 0x0, sizeof(TFlt) * _num_in);
	return 0;
}

int MNM_Dnode_GRJ_Multiclass::compute_flow()
{
	// to be implemented...
	return 0;
}




/******************************************************************************************************************
*******************************************************************************************************************
												Multiclass OD
*******************************************************************************************************************
******************************************************************************************************************/

/**************************************************************************
                          		Destination
**************************************************************************/
MNM_Destination_Multiclass::MNM_Destination_Multiclass(TInt ID)
	: MNM_Destination::MNM_Destination(ID)
{
	;
}


MNM_Destination_Multiclass::~MNM_Destination_Multiclass()
{
	;
}

/**************************************************************************
                          		Origin
**************************************************************************/
MNM_Origin_Multiclass::MNM_Origin_Multiclass(TInt ID, 
											 TInt max_interval,
											 TFlt flow_scalar,
											 TInt frequency)
	: MNM_Origin::MNM_Origin(ID, max_interval, flow_scalar, frequency)
{
	m_demand_car = std::unordered_map<MNM_Destination_Multiclass*, TFlt*>(); // destination node, time-varying demand list
	m_demand_truck = std::unordered_map<MNM_Destination_Multiclass*, TFlt*>(); // destination node, time-varying demand list
	// jiachao added in Sep
	m_demand_tnc = std::unordered_map<MNM_Destination_Multiclass*, TFlt*>(); // destination node, time-varying demand list
}

MNM_Origin_Multiclass::~MNM_Origin_Multiclass()
{
	for (auto _demand_it : m_demand_car) {
		free(_demand_it.second);
	}
	m_demand_car.clear();

	for (auto _demand_it : m_demand_truck) {
		free(_demand_it.second);
	}
	m_demand_truck.clear();

	// jiachao added in Sep
	for (auto _demand_it : m_demand_tnc) {
		free(_demand_it.second);
	}
	m_demand_tnc.clear();

}

int MNM_Origin_Multiclass::add_dest_demand_multiclass(MNM_Destination_Multiclass *dest, 
													TFlt* demand_car, 
													TFlt* demand_truck)
{
	// split (15-mins demand) to (15 * 1-minute demand)
  	TFlt* _demand_car = (TFlt*) malloc(sizeof(TFlt) * m_max_assign_interval * 15);
  	for (int i = 0; i < m_max_assign_interval * 15; ++i) {
  		_demand_car[i] =  TFlt(demand_car[i]);
  	}
  	m_demand_car.insert({dest, _demand_car});

  	TFlt* _demand_truck = (TFlt*) malloc(sizeof(TFlt) * m_max_assign_interval * 15);
  	for (int i = 0; i < m_max_assign_interval * 15; ++i) {
  		_demand_truck[i] =  TFlt(demand_truck[i]);
  	}
  	m_demand_truck.insert({dest, _demand_truck});
  	
  	return 0;
}

int MNM_Origin_Multiclass::release(MNM_Veh_Factory* veh_factory, TInt current_interval)
{
  	return 0;
}

int MNM_Origin_Multiclass::release_one_interval(TInt current_interval, 
												MNM_Veh_Factory* veh_factory, 
												TInt assign_interval, 
												TFlt adaptive_ratio)
{
	if (assign_interval < 0) return 0;
	TInt _veh_to_release;
	MNM_Veh_Multiclass *_veh;
	MNM_Veh_Factory_Multiclass *_vfactory = dynamic_cast<MNM_Veh_Factory_Multiclass *>(veh_factory);
	// release all car
	for (auto _demand_it = m_demand_car.begin(); _demand_it != m_demand_car.end(); _demand_it++) {
		_veh_to_release = TInt(MNM_Ults::round((_demand_it -> second)[assign_interval] * m_flow_scalar));
		for (int i = 0; i < _veh_to_release; ++i) {
			if (adaptive_ratio == TFlt(0)){
				_veh = _vfactory -> make_veh_multiclass(current_interval, MNM_TYPE_STATIC, TInt(0));
			}
			else if (adaptive_ratio == TFlt(1)){
				_veh = _vfactory -> make_veh_multiclass(current_interval, MNM_TYPE_ADAPTIVE, TInt(0));
			}
			else{
				TFlt _r = MNM_Ults::rand_flt();
				if (_r <= adaptive_ratio){
					_veh = _vfactory -> make_veh_multiclass(current_interval, MNM_TYPE_ADAPTIVE, TInt(0));
				}
				else{
					_veh = _vfactory -> make_veh_multiclass(current_interval, MNM_TYPE_STATIC, TInt(0));
				}
			}
			_veh -> set_destination(_demand_it -> first);
			_veh -> set_origin(this);
			_veh -> m_assign_interval = assign_interval;
			m_origin_node -> m_in_veh_queue.push_back(_veh);
		}
	}
	// release all truck
	for (auto _demand_it = m_demand_truck.begin(); _demand_it != m_demand_truck.end(); _demand_it++) {
		_veh_to_release = TInt(MNM_Ults::round((_demand_it -> second)[assign_interval] * m_flow_scalar));
		for (int i = 0; i < _veh_to_release; ++i) {
			if (adaptive_ratio == TFlt(0)){
				_veh = _vfactory -> make_veh_multiclass(current_interval, MNM_TYPE_STATIC, TInt(1));
			}
			else if (adaptive_ratio == TFlt(1)){
				_veh = _vfactory -> make_veh_multiclass(current_interval, MNM_TYPE_ADAPTIVE, TInt(1));
			}
			else{
				TFlt _r = MNM_Ults::rand_flt();
				if (_r <= adaptive_ratio){
					_veh = _vfactory -> make_veh_multiclass(current_interval, MNM_TYPE_ADAPTIVE, TInt(1));
				}
				else{
					_veh = _vfactory -> make_veh_multiclass(current_interval, MNM_TYPE_STATIC, TInt(1));
				}
			}
			_veh -> set_destination(_demand_it -> first);
			_veh -> set_origin(this);
			_veh -> m_assign_interval = assign_interval;
			m_origin_node -> m_in_veh_queue.push_back(_veh);
		}
	}
	random_shuffle(m_origin_node -> m_in_veh_queue.begin(), m_origin_node -> m_in_veh_queue.end());
 	return 0;
}

int MNM_Origin_Multiclass::release_one_interval_biclass(TInt current_interval, 
														MNM_Veh_Factory* veh_factory, 
														TInt assign_interval, 
														TFlt adaptive_ratio_car,
														TFlt adaptive_ratio_truck)
{
	if (assign_interval < 0) return 0;
	TInt _veh_to_release;
	MNM_Veh_Multiclass *_veh;
	MNM_Veh_Factory_Multiclass *_vfactory = dynamic_cast<MNM_Veh_Factory_Multiclass *>(veh_factory);
	// release all car
	for (auto _demand_it = m_demand_car.begin(); _demand_it != m_demand_car.end(); _demand_it++) {
		_veh_to_release = TInt(MNM_Ults::round((_demand_it -> second)[assign_interval] * m_flow_scalar));
		for (int i = 0; i < _veh_to_release; ++i) {
			if (adaptive_ratio_car == TFlt(0)){
				_veh = _vfactory -> make_veh_multiclass(current_interval, MNM_TYPE_STATIC, TInt(0));
			}
			else if (adaptive_ratio_car == TFlt(1)){
				_veh = _vfactory -> make_veh_multiclass(current_interval, MNM_TYPE_ADAPTIVE, TInt(0));
			}
			else{
				TFlt _r = MNM_Ults::rand_flt();
				if (_r <= adaptive_ratio_car){
					_veh = _vfactory -> make_veh_multiclass(current_interval, MNM_TYPE_ADAPTIVE, TInt(0));
				}
				else{
					_veh = _vfactory -> make_veh_multiclass(current_interval, MNM_TYPE_STATIC, TInt(0));
				}
			}
			_veh -> set_destination(_demand_it -> first);
			_veh -> set_origin(this);
			_veh -> m_assign_interval = assign_interval;
			m_origin_node -> m_in_veh_queue.push_back(_veh);
		}
	}
	// release all truck
	for (auto _demand_it = m_demand_truck.begin(); _demand_it != m_demand_truck.end(); _demand_it++) {
		_veh_to_release = TInt(MNM_Ults::round((_demand_it -> second)[assign_interval] * m_flow_scalar));
		for (int i = 0; i < _veh_to_release; ++i) {
			if (adaptive_ratio_truck == TFlt(0)){
				_veh = _vfactory -> make_veh_multiclass(current_interval, MNM_TYPE_STATIC, TInt(1));
			}
			else if (adaptive_ratio_truck == TFlt(1)){
				_veh = _vfactory -> make_veh_multiclass(current_interval, MNM_TYPE_ADAPTIVE, TInt(1));
			}
			else{
				TFlt _r = MNM_Ults::rand_flt();
				if (_r <= adaptive_ratio_truck){
					_veh = _vfactory -> make_veh_multiclass(current_interval, MNM_TYPE_ADAPTIVE, TInt(1));
				}
				else{
					_veh = _vfactory -> make_veh_multiclass(current_interval, MNM_TYPE_STATIC, TInt(1));
				}
			}
			_veh -> set_destination(_demand_it -> first);
			_veh -> set_origin(this);
			_veh -> m_assign_interval = assign_interval;
			m_origin_node -> m_in_veh_queue.push_back(_veh);
		}
	}
	// jiachao why shuffle?
	random_shuffle(m_origin_node -> m_in_veh_queue.begin(), m_origin_node -> m_in_veh_queue.end());
 	return 0;
}


/******************************************************************************************************************
*******************************************************************************************************************
												Multiclass Vehicle
*******************************************************************************************************************
******************************************************************************************************************/
MNM_Veh_Multiclass::MNM_Veh_Multiclass(TInt ID, TInt vehicle_class, TInt start_time)
	: MNM_Veh::MNM_Veh(ID, start_time)
{
	m_class = vehicle_class;  // 0: driving, 1: truck, 2: RH

	m_visual_position_on_link = 0.5; // default: visualize veh as at the middle point of link

	TFlt _r = MNM_Ults::rand_flt();

	if (_r > 1) {_r = 1;}

	m_parking_location = _r;

	m_complete_stop_current_link = TInt(0);

}

MNM_Veh_Multiclass::~MNM_Veh_Multiclass()
{
	;
}


/******************************************************************************************************************
*******************************************************************************************************************
												Multiclass Factory
*******************************************************************************************************************
******************************************************************************************************************/

// jiachao added in Sep.
// MNM_Curb_Factory_Multiclass::MNM_Curb_Factory_Multiclass()
// {
// 	m_curb_destination_map = std::unordered_map<TInt, TInt>();
// 	m_curb_capacity_map = std::unordered_map<TInt, TInt>();
// 	m_path_inter_dest_map = std::unordered_map<TInt, std::vector<TInt>>();
// 	m_path_inter_dest_map_rh = std::unordered_map<TInt, std::vector<TInt>>(); 
// 	m_od_inter_dest_map = std::unordered_map<TInt, std::unordered_map<TInt, std::vector<TInt>>>();
// 	m_interdest_curb_map = std::unordered_map<TInt, std::vector<TInt>>();
// 	m_interdest_list = std::vector<TInt>();
// 	m_curb_price_list = std::unordered_map<TInt, std::vector<TFlt>>();
// 	m_interdest_map = std::unordered_map<TInt, MNM_Destination*>();
// }

// jiachao added in Apr. 06
/**************************************************************************
                          Control Factory
**************************************************************************/
MNM_Control_Factory_Multiclass::MNM_Control_Factory_Multiclass()
{
	m_control_map = std::unordered_map<std::string, float>();
	m_control_map_list = std::unordered_map<std::string, std::vector<float>>();
	TInt m_lane_closure_bin;
	TInt m_fix_signal_plan;
	m_lane_closure_map = std::unordered_map<TInt, std::vector<TInt>>();
}

// MNM_Control_Factory_Multiclass::~MNM_Control_Factory_Multiclass()
// {
// 	;
// }


/**************************************************************************
                          Vehicle Factory
**************************************************************************/
MNM_Veh_Factory_Multiclass::MNM_Veh_Factory_Multiclass()
	: MNM_Veh_Factory::MNM_Veh_Factory()
{
	;
}

MNM_Veh_Factory_Multiclass::~MNM_Veh_Factory_Multiclass()
{
	;
}

MNM_Veh_Multiclass* MNM_Veh_Factory_Multiclass::make_veh_multiclass(TInt timestamp, 
														 			Vehicle_type veh_type,
														 			TInt vehicle_cls)
{
	// printf("A vehicle is produce at time %d, ID is %d\n", (int)timestamp, (int)m_num_veh + 1);
	MNM_Veh_Multiclass *_veh = new MNM_Veh_Multiclass(m_num_veh + 1, vehicle_cls, timestamp);
	_veh -> m_type = veh_type;
	m_veh_map.insert({m_num_veh + 1, _veh});
	m_num_veh += 1;

	return _veh;
}

// MNM_Veh_Multiclass* MNM_Veh_Factory_Multiclass::make_veh_multiclass_curb(TInt timestamp, 
// 														 			Vehicle_type veh_type,
// 														 			TInt vehicle_cls,
// 																	MNM_Curb_Factory_Multiclass* curb_factory,
// 																	TInt _o_node_ID,
// 																	TInt _d_node_ID)
// {
// 	MNM_Veh_Multiclass *_veh = new MNM_Veh_Multiclass(m_num_veh + 1, vehicle_cls, timestamp);
// 	_veh -> m_type = veh_type;
// 	m_veh_map.insert({m_num_veh + 1, _veh});
// 	m_num_veh += 1;

// 	std::vector<TInt> _inter_dest_list;
// 	_inter_dest_list = curb_factory -> m_od_inter_dest_map[_o_node_ID][_d_node_ID];
// 	TInt _int_dest_num = TInt(_inter_dest_list.size());
// 	if (_int_dest_num == 0){
// 		_veh -> m_type = MNM_TYPE_STATIC;
// 	}
// 	else{
// 		for (int j = 0; j < _int_dest_num; ++j){
// 			_veh -> m_destination_list.push_back(TInt(_inter_dest_list[j]));
// 			if (vehicle_cls == TInt(0)){
// 				_veh -> m_parking_duration_list.push_back(TInt(240)); // parking duration change 90
// 			}
// 			else if (vehicle_cls == TInt(1)){
// 				_veh -> m_parking_duration_list.push_back(TInt(240)); // 60
// 			}
// 			else{
// 				_veh -> m_parking_duration_list.push_back(TInt(240)); // 20
// 			}
// 		}
// 	}

// 	return _veh;
// }

/**************************************************************************
                          Node factory
**************************************************************************/
MNM_Node_Factory_Multiclass::MNM_Node_Factory_Multiclass()
	: MNM_Node_Factory::MNM_Node_Factory()
{
	;
}

MNM_Node_Factory_Multiclass::~MNM_Node_Factory_Multiclass()
{
	;
}

MNM_Dnode *MNM_Node_Factory_Multiclass::make_node_multiclass(TInt ID, 
												  			DNode_type_multiclass node_type, 
												  			TFlt flow_scalar,
												  			TFlt veh_convert_factor)
{
	MNM_Dnode *_node;
	switch (node_type){
    	case MNM_TYPE_FWJ_MULTICLASS:
			_node = new MNM_Dnode_FWJ_Multiclass(ID, flow_scalar, veh_convert_factor);
			break;
    	case MNM_TYPE_ORIGIN_MULTICLASS:
			_node = new MNM_DMOND_Multiclass(ID, flow_scalar, veh_convert_factor);
			break;
    	case MNM_TYPE_DEST_MULTICLASS:
			_node = new MNM_DMDND_Multiclass(ID, flow_scalar, veh_convert_factor);
			break;
    	default:
			printf("Wrong node type.\n");
			exit(-1);
	}
	m_node_map.insert({ID, _node});
	return _node;
}

/**************************************************************************
                          Link factory
**************************************************************************/
MNM_Link_Factory_Multiclass::MNM_Link_Factory_Multiclass()
	: MNM_Link_Factory::MNM_Link_Factory()
{
	;
}

MNM_Link_Factory_Multiclass::~MNM_Link_Factory_Multiclass()
{
	;
}

MNM_Dlink *MNM_Link_Factory_Multiclass::make_link_multiclass_curb(TInt ID,
									                        DLink_type_multiclass link_type,
															TInt number_of_lane,
															TFlt length,
															TFlt lane_hold_cap_car,
															TFlt lane_hold_cap_truck,
															TFlt lane_flow_cap_car,
															TFlt lane_flow_cap_truck,
															TFlt ffs_car,
															TFlt ffs_truck,
															TFlt unit_time,
															TFlt veh_convert_factor,
															TFlt flow_scalar,
															TInt curb_spaces,
															TInt curb_dest)
{
	MNM_Dlink *_link;
	_link = new MNM_Dlink_Ctm_Multiclass(ID,
										number_of_lane,
										length,
										lane_hold_cap_car,
										lane_hold_cap_truck,
										lane_flow_cap_car,
										lane_flow_cap_truck,
										ffs_car,
										ffs_truck,
										unit_time,
										veh_convert_factor,
										flow_scalar,
										curb_spaces,
										curb_dest);

	m_link_map.insert({ID, _link});
	return _link;
}


MNM_Dlink *MNM_Link_Factory_Multiclass::make_link_multiclass(TInt ID,
									                        DLink_type_multiclass link_type,
															TInt number_of_lane,
															TFlt length,
															TFlt lane_hold_cap_car,
															TFlt lane_hold_cap_truck,
															TFlt lane_flow_cap_car,
															TFlt lane_flow_cap_truck,
															TFlt ffs_car,
															TFlt ffs_truck,
															TFlt unit_time,
															TFlt veh_convert_factor,
															TFlt flow_scalar)
{
	MNM_Dlink *_link;
	switch (link_type){
    	case MNM_TYPE_CTM_MULTICLASS:
			_link = new MNM_Dlink_Ctm_Multiclass(ID,
												number_of_lane,
												length,
												lane_hold_cap_car,
												lane_hold_cap_truck,
												lane_flow_cap_car,
												lane_flow_cap_truck,
												ffs_car,
												ffs_truck,
												unit_time,
												veh_convert_factor,
												flow_scalar);
			break;
		case MNM_TYPE_LQ_MULTICLASS:
			_link = new MNM_Dlink_Lq_Multiclass(ID,
												number_of_lane,
												length,
												lane_hold_cap_car,
												lane_hold_cap_truck,
												lane_flow_cap_car,
												lane_flow_cap_truck,
												ffs_car,
												ffs_truck,
												unit_time,
												veh_convert_factor,
												flow_scalar);
			break;
    	case MNM_TYPE_PQ_MULTICLASS:
			_link = new MNM_Dlink_Pq_Multiclass(ID,
												number_of_lane,
												length,
												lane_hold_cap_car,
												lane_hold_cap_truck,
												lane_flow_cap_car,
												lane_flow_cap_truck,
												ffs_car,
												ffs_truck,
												unit_time,
												veh_convert_factor,
												flow_scalar);
			break;
    	default:
			printf("Wrong link type.\n");
			exit(-1);
	}
	m_link_map.insert({ID, _link});
	return _link;
}

/**************************************************************************
                          OD factory
**************************************************************************/
MNM_OD_Factory_Multiclass::MNM_OD_Factory_Multiclass()
	: MNM_OD_Factory::MNM_OD_Factory()
{
	;
}

MNM_OD_Factory_Multiclass::~MNM_OD_Factory_Multiclass()
{
	;
}

MNM_Destination_Multiclass *MNM_OD_Factory_Multiclass::make_destination(TInt ID)
{
	MNM_Destination_Multiclass *_dest;
	_dest = new MNM_Destination_Multiclass(ID);
	m_destination_map.insert({ID, _dest});
	return _dest;
}

MNM_Origin_Multiclass *MNM_OD_Factory_Multiclass::make_origin(TInt ID, 
												TInt max_interval, 
												TFlt flow_scalar, 
												TInt frequency)
{
	MNM_Origin_Multiclass *_origin;
	_origin = new MNM_Origin_Multiclass(ID, max_interval, flow_scalar, frequency);
	m_origin_map.insert({ID, _origin});
	return _origin;
}

/******************************************************************************************************************
*******************************************************************************************************************
												Multiclass IO Functions
*******************************************************************************************************************
******************************************************************************************************************/
// // Jiachao 0604
// int MNM_IO_Multiclass::load_inter_dest_curb_separate(const std::string& file_folder,
//  									 		MNM_ConfReader *conf_reader,
// 											MNM_Curb_Factory_Multiclass *curb_factory)
// {
// 	std::unordered_map<TInt, std::vector<TInt>> _path_inter_dest_curb;

// 	/* find file */
// 	std::string _file_name = file_folder + "/path_inter_dest_curb";
// 	std::ifstream _file;
// 	_file.open(_file_name, std::ios::in);

// 	TInt _num_path = conf_reader -> get_int("num_of_path_curb");

// 	TInt _num_inter_dest, _inter_dest_ID, _inter_curb_ID;

// 	std::string _line;
// 	std::vector<std::string> _words;
// 	std::vector<TInt> _num_vector;
// 	if (_file.is_open())
// 	{
// 		std::getline(_file,_line); //skip the first line

// 		for (int i = 0; i < _num_path; ++i){
// 			std::getline(_file,_line);
// 			_words = split(_line, ' ');

// 			// 0604
// 			_num_inter_dest = TInt(std::stoi(_words[0]));
// 			if (_num_inter_dest != TInt(0)){
// 				IAssert(_num_inter_dest * 2 + 1 == TInt(_words.size()));
// 				for (int j = 1; j < int(_words.size()); ++j){
// 					_num_vector.push_back(TInt(std::stoi(_words[j])));
// 				}
// 				_path_inter_dest_curb.insert({TInt(i), _num_vector});
// 				_num_vector.clear();
// 			}
// 		}
// 		_file.close();
// 	}

// 	curb_factory -> m_path_inter_dest_map = _path_inter_dest_curb;

// 	// RH
// 	std::unordered_map<TInt, std::vector<TInt>> _path_inter_dest_curb_rh;

// 	/* find file */
// 	std::string _file_name_rh = file_folder + "/path_inter_dest_curb_rh";
// 	std::ifstream _file_rh;
// 	_file_rh.open(_file_name_rh, std::ios::in);

// 	TInt _num_path_rh = conf_reader -> get_int("num_of_path_curb_rh");

// 	if (_file_rh.is_open())
// 	{
// 		std::getline(_file_rh,_line); //skip the first line

// 		for (int i = 0; i < _num_path_rh; ++i){
// 			std::getline(_file_rh,_line);
// 			_words = split(_line, ' ');

// 			_num_inter_dest = TInt(std::stoi(_words[0]));

// 			IAssert(_num_inter_dest * 2 + 1 == TInt(_words.size()));

// 			for (int j = 1; j < int(_words.size()); ++j){
// 				_num_vector.push_back(TInt(std::stoi(_words[j])));
// 			}

// 			_path_inter_dest_curb_rh.insert({TInt(i), _num_vector});
// 			_num_vector.clear();

// 		}
// 		_file_rh.close();
// 	}

// 	curb_factory -> m_path_inter_dest_map_rh = _path_inter_dest_curb_rh;

// 	// car (only for DUE)
// 	TInt _load_car = conf_reader -> get_int("load_car_curb_choice");

// 	if (_load_car == 1){
// 		std::unordered_map<TInt, std::vector<TInt>> _path_inter_dest_curb_car;

// 		/* find file */
// 		std::string _file_name_car = file_folder + "/path_inter_dest_curb_car";
// 		std::ifstream _file_car;
// 		_file_car.open(_file_name_car, std::ios::in);

// 		TInt _num_path_car = conf_reader -> get_int("num_of_path_curb_car");

// 		if (_file_car.is_open()){
// 			std::getline(_file_car,_line); //skip the first line
// 			for (int i = 0; i < _num_path_car; ++i){
// 				std::getline(_file_car,_line);
// 				_words = split(_line, ' ');
// 				_num_inter_dest = TInt(std::stoi(_words[0]));
// 				if (_num_inter_dest != TInt(0)){
// 					IAssert(_num_inter_dest * 2 + 1 == TInt(_words.size()));
// 					for (int j = 1; j < int(_words.size()); ++j){
// 						_num_vector.push_back(TInt(std::stoi(_words[j])));
// 					}
// 					_path_inter_dest_curb_car.insert({TInt(i), _num_vector});
// 					_num_vector.clear();
// 				}
// 			}
// 			_file_car.close();
// 		}

// 		curb_factory -> m_path_inter_dest_map_car = _path_inter_dest_curb_car;
// 	}

// 	return 0;
// }

// // Jiachao added in Sep.
// int MNM_IO_Multiclass::load_inter_dest_curb(const std::string& file_folder,
//  									 		MNM_ConfReader *conf_reader,
// 											MNM_Curb_Factory_Multiclass *curb_factory,
//  									 		const std::string& file_name)
// {
// 	std::unordered_map<TInt, std::vector<TInt>> _path_inter_dest_curb;

// 	/* find file */
// 	std::string _file_name = file_folder + "/" + file_name;
// 	std::ifstream _file;
// 	_file.open(_file_name, std::ios::in);

// 	TInt _num_path = conf_reader -> get_int("num_of_path_curb");

// 	TInt _num_inter_dest, _inter_dest_ID, _inter_curb_ID;

// 	std::string _line;
// 	std::vector<std::string> _words;
// 	std::vector<TInt> _num_vector;
// 	if (_file.is_open())
// 	{
// 		std::getline(_file,_line); //skip the first line

// 		for (int i = 0; i < _num_path; ++i){
// 			std::getline(_file,_line);
// 			_words = split(_line, ' ');

// 			_num_inter_dest = TInt(std::stoi(_words[0]));

// 			IAssert(_num_inter_dest * 2 + 1 == TInt(_words.size()));

// 			for (int j = 1; j < int(_words.size()); ++j){
// 				_num_vector.push_back(TInt(std::stoi(_words[j])));
// 			}

// 			_path_inter_dest_curb.insert({TInt(i), _num_vector});

// 			_num_vector.clear();

// 		}
// 		_file.close();
// 	}

// 	curb_factory -> m_path_inter_dest_map = _path_inter_dest_curb;

// 	return 0;
// }

// // jiachao fixed curb pricing
// int MNM_IO_Multiclass::build_curb_factory_multiclass(const std::string& file_folder,
//  									 					MNM_ConfReader *conf_reader,
//  									 					MNM_Curb_Factory_Multiclass *curb_factory)
// {
// 	/* read config */
// 	int _curb_install = conf_reader -> get_int("curb_install");
// 	int _num_of_curb = conf_reader -> get_int("num_of_curb");
// 	TFlt _flow_scalar = conf_reader -> get_float("flow_scalar");

// 	// each OD pair has an exogenous inter-destination sequence (for now, only one interdestination for each Od pair for simplicity)
// 	int _num_od_interdest = conf_reader -> get_int("od_interdest"); 

// 	/* read multi-class curb prices */
// 	int _curb_price_multiclass = conf_reader -> get_int("curb_price_multiclass");

// 	if (_curb_install == 1){

// 		/* find curb dest file */
// 		std::string _curb_file_name = file_folder + "/MNM_input_curb_dest";
// 		std::ifstream _curb_file;
// 		_curb_file.open(_curb_file_name, std::ios::in);

// 		std::string _line;
// 		std::vector<std::string> _words;
// 		TInt _curb_ID;
// 		TInt _dest_node_ID;
// 		TInt _curb_cap;
// 		std::vector<TInt> _new_vector;

// 		MNM_Destination *_dest;

// 		if (_curb_file.is_open())
// 		{
// 			std::getline(_curb_file,_line); //skip the first line
// 			for (int i = 0; i < _num_of_curb; ++i){
// 				std::getline(_curb_file,_line);
// 				// printf("%d\n", i);
// 				_words = split(_line, ' ');
// 				if (_words.size() == 3) {
// 					_curb_ID = TInt(std::stoi(_words[0]));
// 					_dest_node_ID = TInt(std::stoi(_words[1]));
// 					_curb_cap = TInt(std::stoi(_words[2]));

// 					curb_factory -> m_curb_destination_map.insert({_curb_ID, _dest_node_ID});
// 					curb_factory -> m_curb_capacity_map.insert({_curb_ID, _curb_cap * _flow_scalar}); // Jiachao fixed this 11/07

// 					// if _dest_node_ID is not in m_interdest_list, add it into the list
// 					if (std::find(curb_factory -> m_interdest_list.begin(), curb_factory -> m_interdest_list.end(), _dest_node_ID) == curb_factory -> m_interdest_list.end()){
// 						if (_dest_node_ID != TInt(-1)){
// 							curb_factory -> m_interdest_list.push_back(_dest_node_ID);
// 							_dest = new MNM_Destination(_dest_node_ID);
//   							curb_factory -> m_interdest_map.insert(std::pair<TInt, MNM_Destination*>(_dest_node_ID, _dest));
// 						}
// 					}

// 					if (_dest_node_ID != TInt(-1)){
// 						if (curb_factory -> m_interdest_curb_map.find(_dest_node_ID) == curb_factory -> m_interdest_curb_map.end()){					
// 							curb_factory -> m_interdest_curb_map.insert({_dest_node_ID, std::vector<TInt>()});
// 						}
// 					}

// 					curb_factory -> m_interdest_curb_map.find(_dest_node_ID) -> second.push_back(_curb_ID);
// 				}
// 				else {
// 					printf("MNM_IO_Multiclass::build_curb_factory_multiclass: Wrong length of line.\n");
// 					exit(-1);
// 				}
// 			}
// 			_curb_file.close();
// 		}

// 		// read in OD interdest map from file
// 		std::string _od_interdest_file_name = file_folder + "/od_inter_dest";
// 		std::ifstream _od_interdest_file;
// 		TInt _o_node;
// 		TInt _d_node;
// 		TInt _inter_node_ID;
// 		std::vector<TInt> _interdest_vec;

// 		_od_interdest_file.open(_od_interdest_file_name, std::ios::in);
// 		if (_od_interdest_file.is_open()){
// 			for (int i = 0; i < _num_od_interdest; ++i){
// 				std::getline(_od_interdest_file,_line);
// 				_words = split(_line, ' ');
// 				if (_words.size() > 2) {
// 					_o_node = TInt(std::stoi(_words[0]));
// 					_d_node = TInt(std::stoi(_words[1]));

// 					for (int j = 2; j < int(_words.size()); ++j){
// 						_inter_node_ID = TInt(std::stoi(_words[j]));
// 						_interdest_vec.push_back(_inter_node_ID);
// 						if (std::find(curb_factory -> m_interdest_list.begin(), curb_factory -> m_interdest_list.end(), _inter_node_ID) == curb_factory -> m_interdest_list.end()){
// 							curb_factory -> m_interdest_list.push_back(_inter_node_ID);
// 						}
// 					}

// 					if (curb_factory -> m_od_inter_dest_map.find(_o_node) == curb_factory -> m_od_inter_dest_map.end()){
// 						curb_factory -> m_od_inter_dest_map.insert({_o_node, std::unordered_map<TInt, std::vector<TInt>>()});
// 					}

// 					curb_factory -> m_od_inter_dest_map.find(_o_node) -> second.insert({_d_node, _interdest_vec});
// 					_interdest_vec.clear();
// 				}
// 				else {
// 					printf("MNM_IO_Multiclass::build_curb_factory_multiclass:read_od_interdest wrong length of line.\n");
// 					exit(-1);
// 				}
// 			}
// 			_od_interdest_file.close();
// 		}

// 		/* (TODO) read curb price */
// 		std::string _curb_price_line;
// 		std::vector<std::string> _curb_price_words;
		
// 		if (_curb_price_multiclass == 0){
// 			std::string _curb_price_name = file_folder + "/curb_price";
// 			std::ifstream _curb_price_file;

// 			_curb_price_file.open(_curb_price_name, std::ios::in);

// 			if (_curb_price_file.is_open())
// 			{
// 				for (int i = 0; i < _num_of_curb; ++i){
// 					std::getline(_curb_price_file, _curb_price_line);
// 					_curb_price_words = split(_curb_price_line, ' ');

// 					if (_curb_price_words.size() >= 2){
						
// 						_curb_ID = TInt(std::stoi(_curb_price_words[0]));

// 						if (curb_factory -> m_curb_price_list.find(_curb_ID) == curb_factory -> m_curb_price_list.end()){
// 							curb_factory -> m_curb_price_list.insert({_curb_ID, std::vector<TFlt>()});
// 						}

// 						for (int j = 1; j < int(_curb_price_words.size()); ++j){
// 							curb_factory -> m_curb_price_list.find(_curb_ID) -> second.push_back(TFlt(std::stof(_curb_price_words[j])));
// 						}	
// 					}
// 					else{
// 						printf("MNM_IO_Multiclass::build_curb_factory_multiclass:curb_price wrong length of line input (smaller than 2).\n");
// 						exit(-1);
// 					}
// 				}
// 			}
// 		}
// 		else{
// 			assert(_curb_price_multiclass == 1);
// 			std::string _curb_price_name_car = file_folder + "/curb_price_car";
// 			std::string _curb_price_name_truck = file_folder + "/curb_price_truck";
// 			std::string _curb_price_name_rh = file_folder + "/curb_price_rh";
// 			std::ifstream _curb_price_file_car, _curb_price_file_truck, _curb_price_file_rh;

// 			_curb_price_file_car.open(_curb_price_name_car, std::ios::in);

// 			if (_curb_price_file_car.is_open())
// 			{
// 				for (int i = 0; i < _num_of_curb; ++i){
// 					std::getline(_curb_price_file_car, _curb_price_line);
// 					_curb_price_words = split(_curb_price_line, ' ');

// 					if (_curb_price_words.size() >= 2){
						
// 						_curb_ID = TInt(std::stoi(_curb_price_words[0]));

// 						if (curb_factory -> m_curb_price_list_car.find(_curb_ID) == curb_factory -> m_curb_price_list_car.end()){
// 							curb_factory -> m_curb_price_list_car.insert({_curb_ID, std::vector<TFlt>()});
// 						}

// 						for (int j = 1; j < int(_curb_price_words.size()); ++j){
// 							curb_factory -> m_curb_price_list_car.find(_curb_ID) -> second.push_back(TFlt(std::stof(_curb_price_words[j])));
// 						}	
// 					}
// 					else{
// 						printf("MNM_IO_Multiclass::build_curb_factory_multiclass:curb_price_car wrong length of line input (smaller than 2).\n");
// 						exit(-1);
// 					}
// 				}
// 			}

// 			_curb_price_file_truck.open(_curb_price_name_truck, std::ios::in);

// 			if (_curb_price_file_truck.is_open())
// 			{
// 				for (int i = 0; i < _num_of_curb; ++i){
// 					std::getline(_curb_price_file_truck, _curb_price_line);
// 					_curb_price_words = split(_curb_price_line, ' ');

// 					if (_curb_price_words.size() >= 2){
						
// 						_curb_ID = TInt(std::stoi(_curb_price_words[0]));

// 						if (curb_factory -> m_curb_price_list_truck.find(_curb_ID) == curb_factory -> m_curb_price_list_truck.end()){
// 							curb_factory -> m_curb_price_list_truck.insert({_curb_ID, std::vector<TFlt>()});
// 						}

// 						for (int j = 1; j < int(_curb_price_words.size()); ++j){
// 							curb_factory -> m_curb_price_list_truck.find(_curb_ID) -> second.push_back(TFlt(std::stof(_curb_price_words[j])));
// 						}	
// 					}
// 					else{
// 						printf("MNM_IO_Multiclass::build_curb_factory_multiclass:curb_price_truck wrong length of line input (smaller than 2).\n");
// 						exit(-1);
// 					}
// 				}
// 			}

// 			_curb_price_file_rh.open(_curb_price_name_rh, std::ios::in);

// 			if (_curb_price_file_rh.is_open())
// 			{
// 				for (int i = 0; i < _num_of_curb; ++i){
// 					std::getline(_curb_price_file_rh, _curb_price_line);
// 					_curb_price_words = split(_curb_price_line, ' ');

// 					if (_curb_price_words.size() >= 2){
						
// 						_curb_ID = TInt(std::stoi(_curb_price_words[0]));

// 						if (curb_factory -> m_curb_price_list_rh.find(_curb_ID) == curb_factory -> m_curb_price_list_rh.end()){
// 							curb_factory -> m_curb_price_list_rh.insert({_curb_ID, std::vector<TFlt>()});
// 						}

// 						for (int j = 1; j < int(_curb_price_words.size()); ++j){
// 							curb_factory -> m_curb_price_list_rh.find(_curb_ID) -> second.push_back(TFlt(std::stof(_curb_price_words[j])));
// 						}	
// 					}
// 					else{
// 						printf("MNM_IO_Multiclass::build_curb_factory_multiclass:curb_price_rh wrong length of line input (smaller than 2).\n");
// 						exit(-1);
// 					}
// 				}
// 			}
// 		}
// 	}
// 	return 0;
// }

// jiachao added in Apr.
int MNM_IO_Multiclass::build_control_factory_multiclass(const std::string& file_folder,
														MNM_ConfReader *conf_reader,		
 											 			MNM_Control_Factory_Multiclass *control_factory,
 											 			const std::string& file_name)
{
	/* read config */
	TInt _num_of_movement = conf_reader -> get_int("num_of_movement");
	
	TInt fix_signal_plan = conf_reader -> get_int("fix_signal_plan");

	TInt lane_closure = conf_reader -> get_int("lane_closure");

	TInt _total_interval = conf_reader -> get_int("total_interval");

	TInt _num_of_lane_closure = conf_reader -> get_int("num_of_lane_closure");

	TInt fix_ramp_metering = conf_reader -> get_int("fix_ramp_metering"); // 1 if fixed 0 otherwise

	TInt _num_of_ramp_metering = conf_reader -> get_int("num_of_ramp_metering");

	TInt _alinea_control = conf_reader ->get_int("alinea_control");

	TInt _lsc_control = conf_reader->get_int("lsc_control");

	/* find ramp file */
	std::string _ramp_file_name = file_folder + "/MNM_rm";
	std::ifstream _ramp_file;
	_ramp_file.open(_ramp_file_name, std::ios::in);

	/* find control file */
	std::string _control_file_name = file_folder + "/" + file_name;
	std::ifstream _control_file;
	_control_file.open(_control_file_name, std::ios::in);

	/* find lane file */
	std::string _lane_file_name = file_folder + "/" + "MNM_laneclosure";
	std::ifstream _lane_file;
	_lane_file.open(_lane_file_name, std::ios::in);

	/* read ramp file preparation */
	std::string _line_ramp;
	std::vector<std::string> _words_ramp;
	TInt _ramp_link_ID;
	std::vector<float> _ramp_metering_info;
	std::vector<float> _ramp_control_rate_historical;

	/* read control file preparation */
	std::string _line;
	std::vector<std::string> _words;
	std::string _node_ID;
	std::string _in_link_ID;
	std::string _out_link_ID;
	std::string _movement_pair_ID;
	float _control_rate;
	std::vector<float> _control_rates;

	/* read lane file preparation */
	std::string _line_lane;
	std::vector<std::string> _words_lane;
	TInt _link_ID;
	std::vector<TInt> _lane_closure_info;


	// MNM_Control_Factory_Multiclass control_factory;
	MNM_Control_Factory_Multiclass* _control_factory = dynamic_cast<MNM_Control_Factory_Multiclass *>(control_factory);

	/* two binary indicator */
	_control_factory -> m_lane_closure_bin = lane_closure;
	_control_factory -> m_fix_signal_plan = fix_signal_plan;
	_control_factory -> m_ramp_metering_bin = fix_ramp_metering;
	_control_factory -> m_alinea_control = _alinea_control;
	_control_factory -> m_lsc_control = _lsc_control;
	
	/* read ramp metering file */
	if (_control_factory -> m_ramp_metering_bin == 1)
	{
		if (_ramp_file.is_open()){
			std::getline(_ramp_file,_line_ramp);
			for (int i = 0; i < _num_of_ramp_metering; ++i){
				std::getline(_ramp_file,_line_ramp);
				
				_words_ramp = split(trim(_line_ramp), ' ');

				if (_words_ramp.size() == 9){
					_ramp_link_ID = TInt(std::stoi(_words_ramp[0]));

					for (int n = 1; n < 9; ++n){
						_ramp_metering_info.push_back(std::stof(_words_ramp[n]));
					}	
					
					_control_factory -> m_ramp_metering_map.insert({_ramp_link_ID, _ramp_metering_info}); 
					
					for (int k = 0; k < _total_interval; ++k){
						_ramp_control_rate_historical.push_back(std::stof(_words_ramp[1]));
					}
					
					_control_factory ->m_rm_control_historical.insert({_ramp_link_ID, _ramp_control_rate_historical});

					_ramp_metering_info.clear();
					_ramp_control_rate_historical.clear();
				
				}
				else{
					printf("Error: ramp metering info has missing columns for %d line input\n", i);
				}
			}
			_ramp_file.close();
		}
	}

	/* read control file */
	if (_control_file.is_open())
	{
		std::getline(_control_file,_line); //skip the first line
		for (int i = 0; i < _num_of_movement; ++i){

			std::getline(_control_file,_line);
			// printf("%s\n", _line.c_str());

			_words = split(trim(_line), ' ');
			// printf("this line words num = %d\n", int(_words.size()));

			if (_control_factory -> m_fix_signal_plan == 0){
				if (_words.size() == 4){
					_node_ID = trim(_words[0]);
					_in_link_ID = trim(_words[1]);
					_out_link_ID = trim(_words[2]);
					_movement_pair_ID = _node_ID + "_" + _in_link_ID + "_" + _out_link_ID;
					_control_rate = std::stof(_words[3]);
					_control_factory -> m_control_map.insert({_movement_pair_ID, _control_rate});
				}
				else{
					printf("Error: not fix control rate for movement %d\n", i);
				}
			}
			if (_control_factory -> m_fix_signal_plan == 1){
				if (int(_words.size()) == int(_total_interval + 3)){
					_node_ID = trim(_words[0]);
					_in_link_ID = trim(_words[1]);
					_out_link_ID = trim(_words[2]);
					_movement_pair_ID = _node_ID + "_" + _in_link_ID + "_" + _out_link_ID;

					for (int j = 3; j < _total_interval + 3; ++j){
						_control_rates.push_back(std::stof(_words[j]));
					}

					_control_factory -> m_control_map_list.insert({_movement_pair_ID, _control_rates});

					_control_rates.clear();	
				}
				else{
					printf("Error: fix control rate list for movement %d\n", i);
				}

			}	
				// printf("get control information for movement %d\n", i);
		}
		_control_file.close();
	}

	/* read lane file */
	if (lane_closure == 1){
		if (_lane_file.is_open())
		{
			std::getline(_lane_file,_line_lane); //skip the first line
			
			for (int m = 0; m < _num_of_lane_closure; ++m){
				std::getline(_lane_file, _line_lane);
				_words_lane = split(trim(_line_lane), ' ');

				if (_words_lane.size() == 5){
					_link_ID = TInt(std::stoi(_words_lane[0]));


					for (int n = 1; n < 5; ++n){
						_lane_closure_info.push_back(TInt(std::stoi(_words_lane[n])));
					}	

					_control_factory -> m_lane_closure_map.insert({_link_ID, _lane_closure_info});

					_lane_closure_info.clear();
				}
				else{
					printf("Error: lane closure info has missing columns for %d line input\n", m);
				}
			}

			_lane_file.close();
		}
	}
	
	return 0;
}
// jiachao added end
int MNM_IO_Multiclass::build_node_factory_multiclass(const std::string& file_folder,
											         MNM_ConfReader *conf_reader,
											         MNM_Node_Factory *node_factory,
                                                     const std::string& file_name)
{
	/* find file */
	std::string _node_file_name = file_folder + "/" + file_name;
	std::ifstream _node_file;
	_node_file.open(_node_file_name, std::ios::in);

	/* read config */
	TInt _num_of_node = conf_reader -> get_int("num_of_node");
	TFlt _flow_scalar = conf_reader -> get_float("flow_scalar");

	/* read file */
	std::string _line;
	std::vector<std::string> _words;
	TInt _node_ID;
	std::string _type;
	TFlt _veh_convert_factor;

	MNM_Node_Factory_Multiclass* _node_factory = dynamic_cast<MNM_Node_Factory_Multiclass *>(node_factory);

	if (_node_file.is_open())
	{
		std::getline(_node_file,_line); //skip the first line
		for (int i = 0; i < _num_of_node; ++i){
			std::getline(_node_file,_line);
			// printf("%d\n", i);
			_words = split(_line, ' ');
			if (_words.size() == 3) {
				_node_ID = TInt(std::stoi(_words[0]));
				_type = trim(_words[1]);
				_veh_convert_factor = TFlt(std::stod(_words[2]));
				if (_type == "FWJ"){
					_node_factory -> make_node_multiclass(_node_ID, 
														MNM_TYPE_FWJ_MULTICLASS, 
														_flow_scalar,
														_veh_convert_factor);
					continue;
				}				
				if (_type =="DMOND"){
					_node_factory -> make_node_multiclass(_node_ID, 
														MNM_TYPE_ORIGIN_MULTICLASS, 
														_flow_scalar,
														_veh_convert_factor);
					continue;
				}
				if (_type =="DMDND"){
					_node_factory -> make_node_multiclass(_node_ID, 
														MNM_TYPE_DEST_MULTICLASS, 
														_flow_scalar,
														_veh_convert_factor);
					continue;
				}
				printf("Wrong node type, %s\n", _type.c_str());
				exit(-1);
			}
			else {
				printf("MNM_IO_Multiclass::build_node_factory_Multiclass: Wrong length of line.\n");
				exit(-1);
			}
		}
		_node_file.close();
	}
	return 0;
}

int MNM_IO_Multiclass::build_link_factory_multiclass(const std::string& file_folder,
                                                     MNM_ConfReader *conf_reader,
                                                     MNM_Link_Factory *link_factory,
                                                     const std::string& file_name)
{
	/* find file */
	std::string _link_file_name = file_folder + "/" + file_name;
	std::ifstream _link_file;
	_link_file.open(_link_file_name, std::ios::in);

	/* read config */
	TInt _num_of_link = conf_reader -> get_int("num_of_link");
	TFlt _flow_scalar = conf_reader -> get_float("flow_scalar");
	TFlt _unit_time = conf_reader -> get_float("unit_time");
	// jiachao added for curb
	TInt _curb_install = conf_reader -> get_int("curb_install");

	/* read file */
	std::string _line;
	std::vector<std::string> _words;
	TInt _link_ID;
	TFlt _lane_hold_cap_car;
	TFlt _lane_flow_cap_car;
	TInt _number_of_lane;
	TFlt _length;
	TFlt _ffs_car;
	std::string _type;
	// new in multiclass vehicle case
	TFlt _lane_hold_cap_truck;
	TFlt _lane_flow_cap_truck;
	TFlt _ffs_truck;
	TFlt _veh_convert_factor;
	TInt _curb_spaces;
	TInt _curb_dest;

	MNM_Link_Factory_Multiclass* _link_factory = dynamic_cast<MNM_Link_Factory_Multiclass *>(link_factory);

	if (_link_file.is_open())
	{
		// printf("Start build link factory.\n");
		std::getline(_link_file,_line); //skip the first line
		for (int i = 0; i < _num_of_link; ++i){
			std::getline(_link_file,_line);
			_words = split(_line, ' ');

			if ((_words.size() == 11) && (_curb_install == TInt(0))) {
				_link_ID = TInt(std::stoi(_words[0]));
				_type = trim(_words[1]);
				_length = TFlt(std::stod(_words[2]));
				_ffs_car = TFlt(std::stod(_words[3]));
				_lane_flow_cap_car = TFlt(std::stod(_words[4]));  // flow capacity (vehicles/hour/lane)
				_lane_hold_cap_car = TFlt(std::stod(_words[5]));  // jam density (vehicles/mile/lane)
				_number_of_lane = TInt(std::stoi(_words[6]));
				// new in multiclass vehicle case
				_ffs_truck = TFlt(std::stod(_words[7]));
				_lane_flow_cap_truck = TFlt(std::stod(_words[8]));
				_lane_hold_cap_truck = TFlt(std::stod(_words[9]));
				_veh_convert_factor = TFlt(std::stod(_words[10]));

				/* unit conversion */
				// mile -> meter, hour -> second
				_length = _length * TFlt(1600); // m
				_ffs_car = _ffs_car * TFlt(1600) / TFlt(3600); // m/s
				_lane_flow_cap_car = _lane_flow_cap_car / TFlt(3600);  // vehicles/s/lane
				_lane_hold_cap_car = _lane_hold_cap_car / TFlt(1600);  // vehicles/m/lane
				_ffs_truck = _ffs_truck * TFlt(1600) / TFlt(3600);  // m/s
				_lane_flow_cap_truck = _lane_flow_cap_truck / TFlt(3600);  // vehicles/s/lane
				_lane_hold_cap_truck = _lane_hold_cap_truck / TFlt(1600);  // vehicles/m/lane

				/* build */
				if (_type == "PQ"){
					_link_factory -> make_link_multiclass(_link_ID,
														MNM_TYPE_PQ_MULTICLASS,
														_number_of_lane,
														_length,
														_lane_hold_cap_car,
														_lane_hold_cap_truck,
														_lane_flow_cap_car,
														_lane_flow_cap_truck,
														_ffs_car,
														_ffs_truck,
														_unit_time,
														_veh_convert_factor,
														_flow_scalar);
					continue;
				}
				if (_type == "LQ"){
					_link_factory -> make_link_multiclass(_link_ID,
														MNM_TYPE_LQ_MULTICLASS,
														_number_of_lane,
														_length,
														_lane_hold_cap_car,
														_lane_hold_cap_truck,
														_lane_flow_cap_car,
														_lane_flow_cap_truck,
														_ffs_car,
														_ffs_truck,
														_unit_time,
														_veh_convert_factor,
														_flow_scalar);
					continue;
				}
				if (_type =="CTM"){
					_link_factory -> make_link_multiclass(_link_ID,
														MNM_TYPE_CTM_MULTICLASS,
														_number_of_lane,
														_length,
														_lane_hold_cap_car,
														_lane_hold_cap_truck,
														_lane_flow_cap_car,
														_lane_flow_cap_truck,
														_ffs_car,
														_ffs_truck,
														_unit_time,
														_veh_convert_factor,
														_flow_scalar);
					continue;
				}
				printf("Wrong link type, %s\n", _type.c_str());
				exit(-1);
			}
			else{
				printf("MNM_IO::build_link_factory::Wrong length of line.\n");
				exit(-1);
			}
		}
		_link_file.close();
	}
	return 0;
}								

int MNM_IO_Multiclass::build_demand_multiclass(const std::string& file_folder,
 											   MNM_ConfReader *conf_reader,
 											   MNM_OD_Factory *od_factory,
                                               const std::string& file_name)
{
	/* find file */
	std::string _demand_file_name = file_folder + "/" + file_name;
	std::ifstream _demand_file;
	_demand_file.open(_demand_file_name, std::ios::in);

	/* read config */
	TInt _unit_time = conf_reader -> get_int("unit_time");
	TInt _num_of_minute =  int(conf_reader -> get_int("assign_frq")) / (60 / _unit_time);  // the releasing strategy is assigning vehicles per 1 minute
	TInt _max_interval = conf_reader -> get_int("max_interval"); 
	TInt _num_OD = conf_reader -> get_int("OD_pair");

	/* build */
	TInt _O_ID, _D_ID;
	MNM_Origin_Multiclass *_origin;
	MNM_Destination_Multiclass *_dest;
	std::string _line;
	std::vector<std::string> _words;
	if (_demand_file.is_open())
	{
		// printf("Start build demand profile.\n");
		TFlt *_demand_vector_car = (TFlt*) malloc(sizeof(TFlt) * _max_interval * _num_of_minute);
		TFlt *_demand_vector_truck = (TFlt*) malloc(sizeof(TFlt) * _max_interval * _num_of_minute);
		memset(_demand_vector_car, 0x0, sizeof(TFlt) * _max_interval * _num_of_minute);
		memset(_demand_vector_truck, 0x0, sizeof(TFlt) * _max_interval * _num_of_minute);
		TFlt _demand_car;
		TFlt _demand_truck;

		std::getline(_demand_file,_line); //skip the first line
		for (int i = 0; i < _num_OD; ++i){
			std::getline(_demand_file,_line);
			_words = split(_line, ' ');
			if (TInt(_words.size()) == (_max_interval * 2 + 2)) {
				_O_ID = TInt(std::stoi(_words[0]));
				_D_ID = TInt(std::stoi(_words[1]));
				// the releasing strategy is assigning vehicles per 1 minute, so disaggregate 15-min demand into 1-min demand
				for (int j = 0; j < _max_interval; ++j) {
					_demand_car = TFlt(std::stod(_words[j + 2])) / TFlt(_num_of_minute);  
					_demand_truck = TFlt(std::stod(_words[j + _max_interval + 2])) / TFlt(_num_of_minute);
					for (int k = 0; k < _num_of_minute; ++k){
						_demand_vector_car[j * _num_of_minute + k] = _demand_car;
						_demand_vector_truck[j * _num_of_minute + k] = _demand_truck;
					}
				}
				_origin = dynamic_cast<MNM_Origin_Multiclass *>(od_factory -> get_origin(_O_ID));
				_dest = dynamic_cast<MNM_Destination_Multiclass *>(od_factory -> get_destination(_D_ID));
				_origin -> add_dest_demand_multiclass(_dest, _demand_vector_car, _demand_vector_truck);
			}
			else{
				printf("Something wrong in build_demand! line = %d\n", i+1);
				free(_demand_vector_car);
				free(_demand_vector_truck);
				exit(-1);
			}
		}
		free(_demand_vector_car);
		free(_demand_vector_truck);
		_demand_file.close();
	}
	return 0;
}



/******************************************************************************************************************
*******************************************************************************************************************
												Multiclass DTA
*******************************************************************************************************************
******************************************************************************************************************/
MNM_Dta_Multiclass::MNM_Dta_Multiclass(const std::string& file_folder)
	: MNM_Dta::MNM_Dta(file_folder)
{
	// Re-run the multiclass version of initialize();
	initialize();
}

MNM_Dta_Multiclass::~MNM_Dta_Multiclass()
{
	;
}

int MNM_Dta_Multiclass::initialize()
{
	if (m_veh_factory != nullptr) delete m_veh_factory;
  	if (m_node_factory != nullptr) delete m_node_factory;
  	if (m_link_factory != nullptr) delete m_link_factory;
  	if (m_od_factory != nullptr) delete m_od_factory;
  	if (m_config != nullptr) delete m_config;
	m_veh_factory = new MNM_Veh_Factory_Multiclass();
	// printf("1\n");
	m_node_factory = new MNM_Node_Factory_Multiclass();
	// printf("2\n");
	m_link_factory = new MNM_Link_Factory_Multiclass();
	// printf("3\n");
	m_od_factory = new MNM_OD_Factory_Multiclass();
	// printf("4\n");

	// jiachao added in Apr.
	m_control_factory = new MNM_Control_Factory_Multiclass();
	// jiachao commented out in 11/07/2024
	// m_curb_factory = new MNM_Curb_Factory_Multiclass();

	m_config = new MNM_ConfReader(m_file_folder + "/config.conf", "DTA");
	m_unit_time = m_config -> get_int("unit_time");
	m_flow_scalar = m_config -> get_int("flow_scalar");
	// printf("5\n");
	m_emission = new MNM_Cumulative_Emission_Multiclass(TFlt(m_unit_time), 0);
    
	// the releasing strategy is assigning vehicles per 1 minute, so disaggregate 15-min demand into 1-min demand
	// change assign_freq to 12 (1 minute = 12 x 5 second / 60) and total_assign_interval to max_interval*_num_of_minute
	m_assign_freq = 60 / int(m_unit_time);  // # of unit intervals in 1 min = # of assign freq
	TInt _num_of_minute =  int(m_config -> get_int("assign_frq")) / m_assign_freq;  // 15 min, # of minutes in original assign interval
	m_total_assign_inter = m_config ->  get_int("max_interval") * _num_of_minute;  // how many 1-min intervals
	m_start_assign_interval = m_config -> get_int("start_assign_interval");

	return 0;
}

// Jiachao
int MNM_Dta_Multiclass::build_from_files_control()
{
	// jiachao added in Apr.
	MNM_IO_Multiclass::build_control_factory_multiclass(m_file_folder, m_config, m_control_factory);

	MNM_IO_Multiclass::build_node_factory_multiclass(m_file_folder, m_config, m_node_factory);

	MNM_IO_Multiclass::build_link_factory_multiclass(m_file_folder, m_config, m_link_factory);

	MNM_IO_Multiclass::build_od_factory(m_file_folder, m_config, m_od_factory, m_node_factory);

	m_graph = MNM_IO_Multiclass::build_graph(m_file_folder, m_config);

	MNM_IO_Multiclass::build_demand_multiclass(m_file_folder, m_config, m_od_factory);

	m_workzone = nullptr;
	set_statistics();
	set_routing();

	return 0;
}

// jiachao
int MNM_Dta_Multiclass::build_from_files_no_control()
{
	MNM_IO_Multiclass::build_node_factory_multiclass(m_file_folder, m_config, m_node_factory);

	MNM_IO_Multiclass::build_link_factory_multiclass(m_file_folder, m_config, m_link_factory);

	MNM_IO_Multiclass::build_od_factory(m_file_folder, m_config, m_od_factory, m_node_factory);

	m_graph = MNM_IO_Multiclass::build_graph(m_file_folder, m_config);

	MNM_IO_Multiclass::build_demand_multiclass(m_file_folder, m_config, m_od_factory);

	return 0;
}
// jiachao
int MNM_Dta_Multiclass::build_control_file(std::string folder)
{
	MNM_Dnode *_node;
	MNM_Dlink *_in_link, *_out_link;
	bool save_in_out_file = true;
	std::ofstream _link_in_out_file;
	std::string _str;
	TInt m_in_link_size = 0;
	TInt m_out_link_size = 0;

	if (save_in_out_file){
		_link_in_out_file.open(folder + "/MNM_control", std::ofstream::out);
		if (! _link_in_out_file.is_open()){
        	printf("Error happens when open _link_in_out_file\n");
        	exit(-1);
        }
	}

	_link_in_out_file << "#node_id in_link_id out_link_id control_rate\n";

	for (auto _node_it = m_node_factory -> m_node_map.begin(); _node_it != m_node_factory -> m_node_map.end(); _node_it++){
    	_node = _node_it -> second;

		m_in_link_size = _node -> m_in_link_array.size();

		m_out_link_size = _node -> m_out_link_array.size();

		if ((m_in_link_size != 0)&&(m_out_link_size != 0)){
			for (size_t i=0; i < _node -> m_in_link_array.size(); i++){
				_in_link = _node -> m_in_link_array[i];
				for (size_t j=0; j < _node -> m_out_link_array.size(); j++){
					_out_link = _node -> m_out_link_array[j];
					_str = std::to_string(_node -> m_node_ID) + " ";
					_str += std::to_string(_in_link -> m_link_ID) + " " + std::to_string(_out_link -> m_link_ID) + " 1.0";
					_str += "\n";
					_link_in_out_file << _str;
				}
			}
		}

	}

	if (_link_in_out_file.is_open()) _link_in_out_file.close();

	return 0;
}

// int MNM_Dta_Multiclass::build_from_files()
// {
// 	// jiachao added in Apr.
// 	MNM_IO_Multiclass::build_control_factory_multiclass(m_file_folder, m_config, m_control_factory);

// 	// jiachao added
// 	MNM_IO_Multiclass::build_curb_factory_multiclass(m_file_folder, m_config, m_curb_factory);

// 	MNM_IO_Multiclass::build_node_factory_multiclass(m_file_folder, m_config, m_node_factory);

// 	MNM_IO_Multiclass::build_link_factory_multiclass(m_file_folder, m_config, m_link_factory, m_curb_factory);

// 	MNM_IO_Multiclass::build_od_factory(m_file_folder, m_config, m_od_factory, m_node_factory);

// 	m_graph = MNM_IO_Multiclass::build_graph(m_file_folder, m_config);

// 	MNM_IO_Multiclass::build_demand_multiclass_curb(m_file_folder, m_config, m_od_factory);

// 	MNM_IO_Multiclass::load_inter_dest_curb(m_file_folder, m_config, m_curb_factory);

// 	// build_workzone();
// 	m_workzone = nullptr;
// 	set_statistics();
// 	set_statistics_curb();
// 	// set routing curb --- build m_routing which is MNM_Routing_Biclass_Hybrid_Curb
// 	set_routing_curb();
// 	return 0;
// }

// // Jiachao 0604
// int MNM_Dta_Multiclass::build_from_files_separate()
// {
// 	// jiachao added in Apr.
// 	MNM_IO_Multiclass::build_control_factory_multiclass(m_file_folder, m_config, m_control_factory);

// 	// jiachao added
// 	MNM_IO_Multiclass::build_curb_factory_multiclass(m_file_folder, m_config, m_curb_factory);

// 	MNM_IO_Multiclass::build_node_factory_multiclass(m_file_folder, m_config, m_node_factory);

// 	MNM_IO_Multiclass::build_link_factory_multiclass(m_file_folder, m_config, m_link_factory, m_curb_factory);

// 	MNM_IO_Multiclass::build_od_factory(m_file_folder, m_config, m_od_factory, m_node_factory);

// 	m_graph = MNM_IO_Multiclass::build_graph(m_file_folder, m_config);

// 	MNM_IO_Multiclass::build_demand_multiclass_curb(m_file_folder, m_config, m_od_factory);

// 	MNM_IO_Multiclass::load_inter_dest_curb_separate(m_file_folder, m_config, m_curb_factory);

// 	// build_workzone();
// 	m_workzone = nullptr;
// 	set_statistics();
// 	set_statistics_curb();
// 	// set routing curb --- build m_routing which is MNM_Routing_Biclass_Hybrid_Curb
// 	set_routing_curb_separate();
// 	return 0;
// }

// // Jiachao added
// int MNM_Dta_Multiclass::build_from_file_no_routing()
// {
// 	// jiachao added in Apr.
// 	MNM_IO_Multiclass::build_control_factory_multiclass(m_file_folder, m_config, m_control_factory);

// 	// jiachao added
// 	MNM_IO_Multiclass::build_curb_factory_multiclass(m_file_folder, m_config, m_curb_factory);

// 	MNM_IO_Multiclass::build_node_factory_multiclass(m_file_folder, m_config, m_node_factory);

// 	MNM_IO_Multiclass::build_link_factory_multiclass(m_file_folder, m_config, m_link_factory, m_curb_factory);

// 	MNM_IO_Multiclass::build_od_factory(m_file_folder, m_config, m_od_factory, m_node_factory);

// 	m_graph = MNM_IO_Multiclass::build_graph(m_file_folder, m_config);

// 	MNM_IO_Multiclass::build_demand_multiclass_curb(m_file_folder, m_config, m_od_factory);

// 	MNM_IO_Multiclass::load_inter_dest_curb(m_file_folder, m_config, m_curb_factory);

// 	// build_workzone();
// 	m_workzone = nullptr;
// 	set_statistics();
// 	// set routing curb --- build m_routing which is MNM_Routing_Biclass_Hybrid_Curb
// 	// set_routing_curb();
// 	return 0;
// }

// add a new function
// load_path_table_biclass()
Path_Table *MNM_Dta_Multiclass::load_path_table_biclass(const std::string& file_name, const PNEGraph& graph,
                                    TInt num_path, bool w_buffer, TInt m_class, bool w_ID)
{
	if (w_ID){
		throw std::runtime_error("Error, MNM_IO::load_path_table, with ID loading not implemented");
	}

	TInt Num_Path = num_path;

	std::ifstream _path_table_file, _buffer_file;
	std::string _buffer_file_name;

	if (w_buffer){
		_buffer_file_name = file_name + "_buffer";
		_buffer_file.open(_buffer_file_name, std::ios::in);
	}

	_path_table_file.open(file_name, std::ios::in);

	Path_Table *_path_table = new Path_Table();

	/* read file */
	std::string _line, _buffer_line;
	std::vector<std::string> _words, _buffer_words;
	TInt _origin_node_ID, _dest_node_ID, _node_ID;
	std::unordered_map<TInt, MNM_Pathset*> *_new_map;
	MNM_Pathset *_pathset;
	MNM_Path *_path;
	TInt _from_ID, _to_ID, _link_ID;
	TInt _path_ID_counter = 0;
	if (_path_table_file.is_open()){
		for (int i = 0; i < Num_Path; ++i){
			std::getline(_path_table_file,_line);
			if (w_buffer){
				std::getline(_buffer_file,_buffer_line);
				_buffer_words = MNM_IO::split(_buffer_line, ' ');
			}
			// std::cout << "Processing: " << _line << "\n";
			_words = MNM_IO::split(_line, ' ');

			// the path length >= 2 (O,D,...)
			if (_words.size() >= 2){
				_origin_node_ID = TInt(std::stoi(_words[0]));
				_dest_node_ID = TInt(std::stoi(_words.back()));

				// there is no current Origin
				if (_path_table -> find(_origin_node_ID) == _path_table -> end()){
					// create a new map for this origin
					_new_map = new std::unordered_map<TInt, MNM_Pathset*>();
					_path_table -> insert(std::pair<TInt, std::unordered_map<TInt, MNM_Pathset*>*>(_origin_node_ID, _new_map));
				}

				// there is no current Dest
				if (_path_table -> find(_origin_node_ID) -> second -> find(_dest_node_ID) == _path_table -> find(_origin_node_ID) -> second -> end()){
					// create a new pathset
					// this object has a m_path_vec to store path object
					_pathset = new MNM_Pathset();
					_path_table -> find(_origin_node_ID) -> second -> insert(std::pair<TInt, MNM_Pathset*>(_dest_node_ID, _pathset));
				}

				// store this path
				_path = new MNM_Path();

				// store path ID
				_path -> m_path_ID = _path_ID_counter;
				_path_ID_counter += 1;

				// m_node_vec is the node collection of path
				for (std::string _s_node_ID : _words){
					_node_ID = TInt(std::stoi(_s_node_ID));
					_path -> m_node_vec.push_back(_node_ID);
				}

			// m_link_vec is the link collection of path
			// this is done based on node collection and graph
				for (size_t j = 0; j < _path -> m_node_vec.size() - 1; ++j){
					_from_ID = _path -> m_node_vec[j];
					_to_ID = _path -> m_node_vec[j+1];
					_link_ID = graph -> GetEI(_from_ID, _to_ID).GetId();  // assume this is not a MultiGraph
					_path -> m_link_vec.push_back(_link_ID);
				}

				if (w_buffer && (_buffer_words.size() > 0)){
					
					TInt _buffer_len = TInt(_buffer_words.size());

					_path -> allocate_buffer(TInt(_buffer_len/2));

					int m = 0;
					for (int j = m_class * (_buffer_len/2); j < (m_class + 1) * (_buffer_len/2); ++j){
						_path -> m_buffer[m] = TFlt(std::stof(MNM_IO::trim(_buffer_words[j])));
						m++;
					}
				}
				
				_path_table -> find(_origin_node_ID) -> second -> find(_dest_node_ID) -> second -> m_path_vec.push_back(_path);
			}
		}
	
		_path_table_file.close();

		if (w_buffer){
			_buffer_file.close();
		}
	}
	else{
		printf("Can't open path table file!\n");
		exit(-1);
	}
	return _path_table;
}

Path_Table *MNM_Dta_Multiclass::load_path_table_curb(const std::string& file_name, const PNEGraph& graph,
                                    TInt num_path, bool w_buffer, TInt m_class, bool w_ID)
{
	if (w_ID){
		throw std::runtime_error("Error, MNM_IO::load_path_table, with ID loading not implemented");
	}
	// printf("Loading Path Table!\n");

	TInt Num_Path = num_path;

	// printf("Number of path %d\n", Num_Path());

	std::ifstream _path_table_file, _buffer_file;
	std::string _buffer_file_name;

	if (w_buffer){
		_buffer_file_name = file_name + "_buffer";
		_buffer_file.open(_buffer_file_name, std::ios::in);
	}

	_path_table_file.open(file_name, std::ios::in);

	Path_Table *_path_table = new Path_Table();

	/* read file */
	std::string _line, _buffer_line;
	std::vector<std::string> _words, _buffer_words;
	TInt _origin_node_ID, _dest_node_ID, _node_ID;
	std::unordered_map<TInt, MNM_Pathset*> *_new_map;
	MNM_Pathset *_pathset;
	MNM_Path *_path;
	TInt _from_ID, _to_ID, _link_ID;
	TInt _path_ID_counter = 0;
	if (_path_table_file.is_open()){
		for (int i = 0; i < Num_Path; ++i){
			std::getline(_path_table_file,_line);
			if (w_buffer){
				std::getline(_buffer_file,_buffer_line);
				_buffer_words = MNM_IO::split(_buffer_line, ' ');
			}
			// std::cout << "Processing: " << _line << "\n";
			_words = MNM_IO::split(_line, ' ');

			// the path length >= 2 (O,D,...)
			if (_words.size() >= 2){
				_origin_node_ID = TInt(std::stoi(_words[0]));
				_dest_node_ID = TInt(std::stoi(_words.back()));

				// there is no current Origin
				if (_path_table -> find(_origin_node_ID) == _path_table -> end()){
					// create a new map for this origin
					_new_map = new std::unordered_map<TInt, MNM_Pathset*>();
					_path_table -> insert(std::pair<TInt, std::unordered_map<TInt, MNM_Pathset*>*>(_origin_node_ID, _new_map));
				}

				// there is no current Dest
				if (_path_table -> find(_origin_node_ID) -> second -> find(_dest_node_ID) == _path_table -> find(_origin_node_ID) -> second -> end()){
					// create a new pathset
					// this object has a m_path_vec to store path object
					_pathset = new MNM_Pathset();
					_path_table -> find(_origin_node_ID) -> second -> insert(std::pair<TInt, MNM_Pathset*>(_dest_node_ID, _pathset));
				}

				// store this path
				_path = new MNM_Path();

				// store path ID
				_path -> m_path_ID = _path_ID_counter;
				_path_ID_counter += 1;

				// m_node_vec is the node collection of path
				for (std::string _s_node_ID : _words){
					_node_ID = TInt(std::stoi(_s_node_ID));
					_path -> m_node_vec.push_back(_node_ID);
				}

			// m_link_vec is the link collection of path
			// this is done based on node collection and graph
				for (size_t j = 0; j < _path -> m_node_vec.size() - 1; ++j){
					_from_ID = _path -> m_node_vec[j];
					_to_ID = _path -> m_node_vec[j+1];
					_link_ID = graph -> GetEI(_from_ID, _to_ID).GetId();  // assume this is not a MultiGraph
					_path -> m_link_vec.push_back(_link_ID);
				}

				if (w_buffer && (_buffer_words.size() > 0)){
					
					TInt _buffer_len = TInt(_buffer_words.size());

				//   IAssert(_buffer_len == 3 * m_total_assign_inter);
					
					// printf("Buffer len %d\n", _buffer_len());

					_path -> allocate_buffer(TInt(_buffer_len/3));

					int m = 0;
					for (int j = m_class * (_buffer_len/3); j < (m_class + 1) * (_buffer_len/3); ++j){
						_path -> m_buffer[m] = TFlt(std::stof(MNM_IO::trim(_buffer_words[j])));
						m++;
					}
				}
				// printf("%f\n", (float)_path -> m_buffer[0]);
				// printf("%f\n", (float)_path -> m_buffer[5]);
				_path_table -> find(_origin_node_ID) -> second -> find(_dest_node_ID) -> second -> m_path_vec.push_back(_path);
			}
		}
	
		_path_table_file.close();

		if (w_buffer){
			_buffer_file.close();
		}
	}
	else{
		printf("Can't open path table file!\n");
		exit(-1);
	}
	// printf("Finish Loading Path Table!\n");
	// printf("path table %p\n", _path_table);
	// printf("path table %s\n", _path_table -> find(100283) -> second -> find(150153) -> second 
	//                           -> m_path_vec.front() -> node_vec_to_string());
	return _path_table;
}

int MNM_Dta_Multiclass::pre_loading()
{
	MNM_Dnode *_node;
	// printf("MNM: Prepare loading!\n");
	m_statistics -> init_record();
	for (auto _node_it = m_node_factory -> m_node_map.begin(); _node_it != m_node_factory -> m_node_map.end(); _node_it++){
		_node = _node_it -> second;
		_node -> prepare_loading();
	}

    // https://stackoverflow.com/questions/7443787/using-c-ifstream-extraction-operator-to-read-formatted-data-from-a-file
	std::ifstream _emission_file(m_file_folder + "/MNM_input_emission_linkID");
	int _link_ID;
	std::unordered_map<int, int> _emission_links = {};
	while (_emission_file >> _link_ID)
	{
	    _emission_links.insert({_link_ID, 0});
	}
	_emission_file.close();

	std::deque<TInt> *_rec;
  	for (auto _map_it : m_link_factory -> m_link_map)
  	{
    	_rec = new std::deque<TInt>();
    	m_queue_veh_map.insert({_map_it.second -> m_link_ID, _rec});
    	if (_emission_links.find(int(_map_it.second -> m_link_ID)) != _emission_links.end()) 
    		m_emission -> register_link(_map_it.second);
  	}
  	
	// printf("Exiting MNM: Prepare loading!\n");
	return 0;
}

// ******* load once with control rate by Jiachao ******* //
int MNM_Dta_Multiclass::load_once_control(bool verbose, TInt load_int, TInt assign_int)
{
  // m_control_factory;
  
  std::unordered_map<std::string, float> *_control_map_single;

  _control_map_single = &m_control_factory -> m_control_map;

  std::unordered_map<std::string, std::vector<float>> *_control_map_all;
  
  _control_map_all = &m_control_factory -> m_control_map_list;

  std::unordered_map<TInt, std::vector<TInt>> *_lane_closure_map;

  // std::unordered_map<TInt, std::vector<TInt>>
  _lane_closure_map = &m_control_factory -> m_lane_closure_map;

  MNM_Origin *_origin;
  MNM_Dnode *_node;
  MNM_Dlink *_link;
  MNM_Destination *_dest;
  if (verbose) printf("-------------------------------    Interval %d   ------------------------------ \n", (int)load_int);

  // step 1: Origin release vehicle
  if (verbose) printf("Releasing!\n");

  if (load_int % m_assign_freq == 0 || load_int==0){
    for (auto _origin_it = m_od_factory -> m_origin_map.begin(); _origin_it != m_od_factory -> m_origin_map.end(); _origin_it++){
      _origin = _origin_it -> second;
      if (assign_int >= m_total_assign_inter) {
        _origin -> release_one_interval(load_int, m_veh_factory, -1, TFlt(-1));
      }
      else{
        if (m_config -> get_string("routing_type") == "Fixed"){
          //printf("Fixed Releasing.\n");
          _origin -> release_one_interval(load_int, m_veh_factory, assign_int, TFlt(0));
        }
        else if((m_config -> get_string("routing_type") == "Adaptive")){
          _origin -> release_one_interval(load_int, m_veh_factory, assign_int, TFlt(1));
        }
        else if((m_config -> get_string("routing_type") == "Hybrid")){
          TFlt _ad_ratio = m_config -> get_float("adaptive_ratio");
          if (_ad_ratio > 1) _ad_ratio = 1;
          if (_ad_ratio < 0) _ad_ratio = 0;
          _origin -> release_one_interval(load_int, m_veh_factory, assign_int, _ad_ratio);
        }
        else if((m_config -> get_string("routing_type") == "Biclass_Hybrid")){
          TFlt _ad_ratio_car = m_config -> get_float("adaptive_ratio_car");
          if (_ad_ratio_car > 1) _ad_ratio_car = 1;
          if (_ad_ratio_car < 0) _ad_ratio_car = 0;

          TFlt _ad_ratio_truck = m_config -> get_float("adaptive_ratio_truck");
          if (_ad_ratio_truck > 1) _ad_ratio_truck = 1;
          if (_ad_ratio_truck < 0) _ad_ratio_truck = 0;
          // NOTE: in this case the release function is different
          _origin -> release_one_interval_biclass(load_int, m_veh_factory, assign_int, _ad_ratio_car, _ad_ratio_truck);
        }
        else{
          printf("WARNING:No assignment!\n");
        }
      }
    } 
  }

  if (verbose) printf("Routing!\n");
  // step 2: route the vehicle
  m_routing -> update_routing(load_int);

  if (verbose) printf("Moving through node!\n");
  // step 3: move vehicles through node
  // TInt count = 1;
  for (auto _node_it = m_node_factory -> m_node_map.begin(); _node_it != m_node_factory -> m_node_map.end(); _node_it++)
  {
	// clock_t _start_node, _end_node;
	// _start_node = clock();
    _node = _node_it -> second;

    // printf("node ID is %d\n", _node -> m_node_ID());
	// _node -> evolve(load_int);
	/* load with control by Jiachao*/
	
    _node -> evolve_control(load_int, _control_map_single, _control_map_all, _lane_closure_map); // how to confirm we use the function for node_multiclass if we declare _node as MNM_Dnode
	// _end_node = clock();
	// std::cout<<"++++++" << " node " << count << " with time = " << double(_end_node - _start_node)/CLOCKS_PER_SEC<< "s" << std::endl; 
	// count += 1;
  }


  // record queuing vehicles after node evolve, which is num of vehicles in finished array
  record_queue_vehicles();

  if (verbose) printf("Moving through link!\n");
  // step 4: move vehicles through link
  // for each link loop
  for (auto _link_it = m_link_factory -> m_link_map.begin(); _link_it != m_link_factory -> m_link_map.end(); _link_it++){
    _link = _link_it -> second;

	// printf("++++++ link id = %d ++++++\n", int(_link->m_link_ID));
	// check configuration lane_closure_bin ?= 1
	TFlt ratio_lane_closure = 1.0; // 1.0 by default meaning there is 100 % lane in use
	TFlt cell_position = -1.0; // -1.0 by default
	TFlt cell_control_rate = 1.0; // 100% outflow

	if (m_control_factory -> m_lane_closure_bin == 1){
		
		// check if this link needs lane closure
		auto _check_link = m_control_factory->m_lane_closure_map.find(_link -> m_link_ID);
		
		if (_check_link != m_control_factory->m_lane_closure_map.end()){
			
			// current link closure info
			std::vector<TInt> _lane_closure_info_temp = m_control_factory->m_lane_closure_map[_link -> m_link_ID];
			TInt _lane_num = _lane_closure_info_temp[0];
			TInt _lane_num_change = _lane_closure_info_temp[1];
			TInt _start_int = _lane_closure_info_temp[2];
			TInt _end_int = _lane_closure_info_temp[3];

			if ((load_int >= _start_int) && (load_int < _end_int)){
				
				ratio_lane_closure = TFlt(_lane_num_change) / TFlt(_lane_num);
					
				// _link -> clear_incoming_array(load_int, ratio_lane_closure);
    			// _link -> evolve(load_int, ratio_lane_closure);
				// printf("======= link id = %d has ratio = %f after lane closure =======\n", (int)_link->m_link_ID, float(ratio_lane_closure));
			}	
		}
	}

	if (m_control_factory -> m_ramp_metering_bin == 1){
		auto _check_ramp = m_control_factory->m_ramp_metering_map.find(_link->m_link_ID);
		if (_check_ramp != m_control_factory->m_ramp_metering_map.end()){
			
			if ((m_control_factory -> m_alinea_control == 0) && (m_control_factory -> m_lsc_control == 0)){
				std::vector<float> _ramp_metering_info_temp = m_control_factory->m_ramp_metering_map[_link->m_link_ID];
			
				TInt _start_ramp = (TInt)_ramp_metering_info_temp[2];
				TInt _end_ramp = (TInt)_ramp_metering_info_temp[3];

				if ((load_int >= _start_ramp) && (load_int < _end_ramp)){
					cell_position = TFlt(_ramp_metering_info_temp[0]);
					cell_control_rate = TFlt(_ramp_metering_info_temp[1]);
				}
				printf("### no control fixed rate = %f ##\n", (double)cell_control_rate);
			}

			/* ALINEA control
			current_flow_rate = previous_flow_rate + ki * (set_point - next_link_flow)
			*/
			if ((m_control_factory -> m_alinea_control == 1) && (m_control_factory -> m_lsc_control == 0)){

				// cell_control_rate = TFlt(0.5);
				
				std::vector<float> _ramp_metering_info_temp = m_control_factory->m_ramp_metering_map[_link->m_link_ID];
			
				TInt _start_ramp = (TInt)_ramp_metering_info_temp[2];
				TInt _end_ramp = (TInt)_ramp_metering_info_temp[3];
				TInt _next_link_ID = (TInt)_ramp_metering_info_temp[4];
				TFlt _alinea_ki = (TFlt)_ramp_metering_info_temp[5];
				TFlt _set_point = (TFlt)_ramp_metering_info_temp[7];
				// _alinea_ki = TFlt(0.0044);
				// _set_point = TFlt(50.0);

				// TInt _lsc_qm = (TInt)_ramp_metering_info_temp[6];

				if ((load_int >= _start_ramp) && (load_int < _end_ramp)){
					
					cell_position = TFlt(_ramp_metering_info_temp[0]);

					TFlt _current_link_flow_car;
					TFlt _current_link_flow_truck;
					TFlt _previous_flow_rate;

					// TODO store previous flow rate
					// std::unordered_map<TInt,std::vector<float>>
					if (load_int > 0){
						_previous_flow_rate = m_control_factory -> m_rm_control_historical[_link->m_link_ID][load_int - 1];
					}
					else{
						_previous_flow_rate = TFlt(_ramp_metering_info_temp[1]);
					}

					// printf("%s\n", m_link_factory -> m_link_map[_next_link_ID] -> m_link_type);
					// MNM_Dlink *_next_link_dlink = m_link_factory -> m_link_map[_next_link_ID];
					MNM_Dlink_Ctm_Multiclass *_next_link = dynamic_cast<MNM_Dlink_Ctm_Multiclass *>(m_link_factory -> m_link_map[_next_link_ID]);

					_current_link_flow_car = _next_link -> get_link_flow_car();
					_current_link_flow_truck = _next_link -> get_link_flow_truck();

					cell_control_rate = _previous_flow_rate + _alinea_ki * (_set_point - (_current_link_flow_car + _current_link_flow_truck * (_next_link -> m_veh_convert_factor)));

					if (cell_control_rate > 1.0){
						printf("## %f ##\n", (double)cell_control_rate);
						printf("## current flow = %f ##\n", (double)(_current_link_flow_car + _current_link_flow_truck * (_next_link -> m_veh_convert_factor)));
						cell_control_rate = 1.0;
					}

					if (cell_control_rate < 0.0){
						printf("## %f ##\n", (double)cell_control_rate);
						cell_control_rate = 0.0;
					}

					m_control_factory -> m_rm_control_historical[_link->m_link_ID][load_int] = cell_control_rate;

				}
				printf("############### alinea control rate is %f #################\n", (double)cell_control_rate);
			}

			/* LSC 
			if ramp link flow >= Q_m
			*/
			if ((m_control_factory -> m_alinea_control == 0) && (m_control_factory -> m_lsc_control == 1)){

				// if (load_int < 2520){
				// 	cell_control_rate = 0.5;
				// }

				// else{
				// 	cell_control_rate = 1.0;
				// }

				std::vector<float> _ramp_metering_info_temp = m_control_factory -> m_ramp_metering_map[_link->m_link_ID];
			
				TInt _start_ramp = (TInt)_ramp_metering_info_temp[2];
				TInt _end_ramp = (TInt)_ramp_metering_info_temp[3];
				TInt _next_link_ID = (TInt)_ramp_metering_info_temp[4];
				TFlt _alinea_ki = (TFlt)_ramp_metering_info_temp[5];
				TFlt _lsc_qm = (TFlt)_ramp_metering_info_temp[6];
				TFlt _set_point = (TFlt)_ramp_metering_info_temp[7];
				// _alinea_ki = TFlt(0.0044);
				// _set_point = TFlt(50.0);

				if ((load_int >= _start_ramp) && (load_int < _end_ramp)){
					
					cell_position = TFlt(_ramp_metering_info_temp[0]);

					TFlt _veh_num_on_ramp = 0.0;

					MNM_Dlink_Ctm_Multiclass *_ramp_link = dynamic_cast<MNM_Dlink_Ctm_Multiclass *>(_link);

					_veh_num_on_ramp += _ramp_link -> get_link_flow_car() + (_ramp_link -> get_link_flow_truck()) * (_ramp_link->m_veh_convert_factor);

					printf("### veh on ramp = %f ### Qm = %f\n", (double)_veh_num_on_ramp, (double)_lsc_qm);

					if (_veh_num_on_ramp <= (TFlt)_lsc_qm){
						// ALINEA method
						
						printf("#### ALINEA ####\n");
						
						TFlt _current_link_flow_car;
						TFlt _current_link_flow_truck;
						TFlt _previous_flow_rate;

						if (load_int > 0){
							_previous_flow_rate = m_control_factory -> m_rm_control_historical[_link->m_link_ID][load_int - 1];
						}
						else{
							_previous_flow_rate = TFlt(_ramp_metering_info_temp[1]);
						}

						MNM_Dlink_Ctm_Multiclass *_next_link = dynamic_cast<MNM_Dlink_Ctm_Multiclass *>(m_link_factory -> m_link_map[_next_link_ID]);

						_current_link_flow_car = _next_link->get_link_flow_car();
						_current_link_flow_truck = _next_link->get_link_flow_truck();

						cell_control_rate = _previous_flow_rate + _alinea_ki * (_set_point - (_current_link_flow_car + _current_link_flow_truck * (_next_link->m_veh_convert_factor)));

						if (cell_control_rate > 1.0){
							cell_control_rate = 1.0;
						}
						if (cell_control_rate < 0.0){
							cell_control_rate = 0.0;
						}
						m_control_factory ->m_rm_control_historical[_link->m_link_ID][load_int] = cell_control_rate;
					}
					else{
						cell_control_rate = 1.0;
					}
				}
				printf("############### lsc control rate is %f #################\n", (double)cell_control_rate);
			}
		}
	}

	_link -> clear_incoming_array(load_int, ratio_lane_closure);
 	
	/* TODO ramp metering */ 
	// _link -> evolve(load_int, ratio_lane_closure, cell_position, cell_control_rate);  
	// _link -> evolve(load_int, ratio_lane_closure);
	_link -> evolve_control(load_int, ratio_lane_closure, cell_position, cell_control_rate);

    // if (_link -> get_link_flow() > 0){
    //   printf("Current Link %d:, traffic flow %.4f, incomming %d, finished %d\n", 
    //       _link -> m_link_ID(), _link -> get_link_flow()(), (int)_link -> m_incoming_array.size(),  (int)_link -> m_finished_array.size());
    //   _link -> print_info();
    // }
    // printf("link ID is %d\n", _link-> m_link_ID());
    // _link -> print_info();
    
  }

  if (verbose) printf("Receiving!\n");
  // step 5: Destination receive vehicle  
  for (auto _dest_it = m_od_factory -> m_destination_map.begin(); _dest_it != m_od_factory -> m_destination_map.end(); _dest_it++){
    _dest = _dest_it -> second;
    _dest -> receive(load_int);
  }

  if (verbose) printf("Update record!\n");
  // step 5: update record
  m_statistics -> update_record(load_int);

  record_enroute_vehicles();

  if (verbose){
	MNM::print_vehicle_statistics(m_veh_factory);
  }
  
  // test();  
  return 0;
}

int MNM_Dta_Multiclass::loading_control(bool verbose)
{
	if (verbose){
		printf("\n\n\n====================================== Start loading! =======================================\n");
	}
	
	TInt _current_inter = 0;
	TInt _assign_inter = m_start_assign_interval;

	while (!finished_loading(_current_inter) || _assign_inter < m_total_assign_inter){
        
		if (verbose){
			printf("\nCurrent loading interval: %d, Current assignment interval: %d\n", _current_inter(), _assign_inter());
		}
		
		load_once_control(verbose, _current_inter, _assign_inter);

		if (_current_inter % m_assign_freq == 0 || _current_inter == 0){
			_assign_inter += 1;
		}
		
		_current_inter += 1;
	}
	
	m_current_loading_interval = _current_inter;
	
	return 0;
}

/******************************************************************************************************************
*******************************************************************************************************************
										Multiclass DTA Gradient Utils
*******************************************************************************************************************
******************************************************************************************************************/

// All functions/API to python should be coded under this namespace
namespace MNM_DTA_GRADIENT
{
// jiachao curb added
TInt get_link_parking_num(MNM_Dlink_Ctm_Multiclass* link)
{
	TInt _link_parking_num = 0;

	for (int m = 0; m < link -> m_num_cells; ++m)
	{
		_link_parking_num += TInt(link -> m_curb_cell_array[m] -> m_veh_parking_num);
		// _link_doubleparking_num += TInt(m_curb_cell_array[m] -> m_veh_doubleparking_num);
	}

	return _link_parking_num;
}

TInt get_link_dp_num(MNM_Dlink_Ctm_Multiclass* link)
{
	TInt _link_dp_num = 0;

	for (int m = 0; m < link -> m_num_cells; ++m)
	{
		_link_dp_num += TInt(link -> m_curb_cell_array[m] -> m_veh_doubleparking_num);
	}

	return _link_dp_num;
}

// jiachao added
TInt get_link_parking_num_car(MNM_Dlink_Ctm_Multiclass* link)
{
	TInt _num_car;

	for (int m = 0; m < link -> m_num_cells; ++m){
		_num_car += TInt((link -> m_curb_cell_array[m] -> m_veh_parking_car).size());
	}

	return _num_car;
}

TInt get_link_parking_num_truck(MNM_Dlink_Ctm_Multiclass* link)
{
	TInt _num_truck;

	for (int m = 0; m < link -> m_num_cells; ++m){
		_num_truck += TInt((link -> m_curb_cell_array[m] -> m_veh_parking_truck).size());
	}

	return _num_truck;
}

TInt get_link_parking_num_rh(MNM_Dlink_Ctm_Multiclass* link)
{
	TInt _num_rh;

	for (int m = 0; m < link ->m_num_cells; ++m){
		_num_rh += TInt((link -> m_curb_cell_array[m] -> m_veh_parking_rh).size());
	}

	return _num_rh;
}

TInt get_link_dp_num_car(MNM_Dlink_Ctm_Multiclass* link)
{
	TInt _num_dp_car;

	for (int m = 0; m < link ->m_num_cells; ++m){
		_num_dp_car += TInt((link -> m_curb_cell_array[m] -> m_veh_doubleparking_car).size());
	}

	return _num_dp_car;
}
TInt get_link_dp_num_truck(MNM_Dlink_Ctm_Multiclass* link)
{
	TInt _num_dp_truck;

	for (int m = 0; m < link ->m_num_cells; ++m){
		_num_dp_truck += TInt((link -> m_curb_cell_array[m] -> m_veh_doubleparking_truck).size());
	}

	return _num_dp_truck;
}
TInt get_link_dp_num_rh(MNM_Dlink_Ctm_Multiclass* link)
{
	TInt _num_dp_rh;

	for (int m = 0; m < link ->m_num_cells; ++m){
		_num_dp_rh += TInt((link -> m_curb_cell_array[m] -> m_veh_doubleparking_rh).size());
	}

	return _num_dp_rh;
}

// jiachao added in Apr.
// input  : MNM_Dta_Multiclass, movement id to check
// output : true or false
bool check_movement_exist(MNM_Dta_Multiclass* test_dta,
						std::string check_movement)
{
	TFlt control_r_temp;
	// item to check
	// std::string check_movement = "4_3_5";
	printf("new checking movement id = %s\n", check_movement.c_str());
	auto check = test_dta -> m_control_factory -> m_control_map.find(check_movement);
	if (check != test_dta -> m_control_factory -> m_control_map.end()){
		control_r_temp = test_dta -> m_control_factory -> m_control_map[check_movement];
		return true;
	}
	else{
		printf("Error: no movement found with id = %s\n", check_movement.c_str());
		return false;
	}
}

// jiachao added in Apr.
TFlt get_control_rate_one(MNM_Dta_Multiclass* test_dta, 
						std::string movement_id)
{
	TFlt _control_rate;
	auto _check = test_dta -> m_control_factory -> m_control_map.find(movement_id);
	if (_check != test_dta -> m_control_factory -> m_control_map.end()){
		_control_rate = test_dta -> m_control_factory -> m_control_map[movement_id];
		return _control_rate;
	}
	else{
		printf("Error: no control rate for id = %s\n", movement_id.c_str());
		return -1;
	}
}

// jiachao added in Apr.
// input  MNM_dta_Multiclass
// output (1) movement_id array: nodeid_inlinkid_outlinkid; (2) control_rate for each movement_id  format : pair()
std::pair<std::vector<std::string>, std::vector<float>> get_control_rate_all(MNM_Dta_Multiclass* test_dta)
{
	std::unordered_map<std::string, float> _control_map_temp;
	std::vector<std::string> _control_movement_list;
	std::vector<float> _control_rate_list;
	_control_map_temp = test_dta -> m_control_factory -> m_control_map;

	for (auto _movement_it = _control_map_temp.begin(); _movement_it != _control_map_temp.end(); _movement_it++){
		_control_movement_list.push_back(_movement_it -> first);
		_control_rate_list.push_back(_movement_it -> second);
	}
	return std::make_pair(_control_movement_list, _control_rate_list);
}

int change_control_rate_one(MNM_Dta_Multiclass* test_dta,
						std::string movement_id,
						TFlt control_rate)
{
	auto _check = test_dta -> m_control_factory -> m_control_map.find(movement_id);
	if (_check != test_dta -> m_control_factory -> m_control_map.end()){
		test_dta -> m_control_factory -> m_control_map[movement_id] = control_rate;
	}
	else{
		printf("Error: no movement found with id = %s\n", movement_id.c_str());
	}	
	return 0;
}

int change_control_rate_all(MNM_Dta_Multiclass* test_dta,
						std::unordered_map<std::string, TFlt> change_movement_map)
{	
	std::string _movement_id;
	TFlt _control_rate;
	for (auto _move_it = change_movement_map.begin(); _move_it != change_movement_map.end(); _move_it++){
		_movement_id = _move_it -> first;
		_control_rate = _move_it -> second;
	
		auto _check = test_dta -> m_control_factory -> m_control_map.find(_movement_id);
		if (_check != test_dta -> m_control_factory -> m_control_map.end()){
			test_dta -> m_control_factory -> m_control_map[_movement_id] = _control_rate;
		}
		else{
			printf("Error: no movement found with id = %s\n", _movement_id.c_str());
		}
	}
	return 0;
}

// added end

TFlt get_link_inflow_car(MNM_Dlink_Multiclass* link, 
                    	TFlt start_time, TFlt end_time)
{
	if (link == nullptr){
		throw std::runtime_error("Error, get_link_inflow_car link is null");
	}
	if (link -> m_N_in_car == nullptr){
		throw std::runtime_error("Error, get_link_inflow_car link cumulative curve is not installed");
	}
	return link -> m_N_in_car -> get_result(end_time) - link -> m_N_in_car -> get_result(start_time);
}

// jiachao added m_N_in_car_cc
TFlt get_link_inflow_car(MNM_Dlink_Multiclass* link, 
                    	TInt start_time, TInt end_time)
{
	if (link == nullptr){
		throw std::runtime_error("Error, get_link_inflow_car link is null");
	}
	if (link -> m_N_in_car == nullptr){
		throw std::runtime_error("Error, get_link_inflow_car link cumulative curve is not installed");
	}

	if (link -> m_N_in_car_cc == nullptr){
		throw std::runtime_error("Error, get_link_inflow_car link cumulative curve cc (curb) is not installed");
	}

	return (link -> m_N_in_car -> get_result(TFlt(end_time)) - link -> m_N_in_car -> get_result(TFlt(start_time))) + (link -> m_N_in_car_cc -> get_result(TFlt(end_time)) - link -> m_N_in_car_cc -> get_result(TFlt(start_time)));
}

TFlt get_link_inflow_truck(MNM_Dlink_Multiclass* link, 
                    	TFlt start_time, TFlt end_time)
{
	if (link == nullptr){
		throw std::runtime_error("Error, get_link_inflow_truck link is null");
	}
	if (link -> m_N_in_truck == nullptr){
		throw std::runtime_error("Error, get_link_inflow_truck link cumulative curve is not installed");
	}
	return link -> m_N_in_truck -> get_result(end_time) - link -> m_N_in_truck -> get_result(start_time);
}

// jiachao added m_N_in_truck_cc
TFlt get_link_inflow_truck(MNM_Dlink_Multiclass* link, 
                           TInt start_time, TInt end_time)
{
	if (link == nullptr){
		throw std::runtime_error("Error, get_link_inflow_truck link is null");
	}
	if (link -> m_N_in_truck == nullptr){
		throw std::runtime_error("Error, get_link_inflow_truck link cumulative curve is not installed");
	}

	if (link -> m_N_in_truck_cc == nullptr){
		throw std::runtime_error("Error, get_link_inflow_truck link cumulative curve cc (curb) is not installed");
	}

	return (link -> m_N_in_truck -> get_result(TFlt(end_time)) - link -> m_N_in_truck -> get_result(TFlt(start_time))) + (link -> m_N_in_truck_cc -> get_result(TFlt(end_time)) - link -> m_N_in_truck_cc -> get_result(TFlt(start_time)));
}

TFlt get_link_inflow_rh(MNM_Dlink_Multiclass* link,
						TFlt start_time, TFlt end_time)
{
	if (link == nullptr){
		throw std::runtime_error("Error, get_link_inflow_rh link is null");
	}
	if (link -> m_N_in_rh == nullptr){
		throw std::runtime_error("Error, get_link_inflow_rh link cumulative curve is not installed");
	}
	return link -> m_N_in_rh -> get_result(end_time) - link -> m_N_in_rh -> get_result(start_time);
}

TFlt get_link_outflow_car(MNM_Dlink_Multiclass* link, 
                    	TFlt start_time, TFlt end_time)
{
	if (link == nullptr){
		throw std::runtime_error("Error, get_link_outflow_car link is null");
	}
	if (link -> m_N_out_car == nullptr){
		throw std::runtime_error("Error, get_link_outflow_car link cumulative curve is not installed");
	}
	return link -> m_N_out_car -> get_result(end_time) - link -> m_N_out_car -> get_result(start_time);
}

TFlt get_link_outflow_truck(MNM_Dlink_Multiclass* link, 
                    	TFlt start_time, TFlt end_time)
{
	if (link == nullptr){
		throw std::runtime_error("Error, get_link_outflow_truck link is null");
	}
	if (link -> m_N_out_truck == nullptr){
		throw std::runtime_error("Error, get_link_outflow_truck link cumulative curve is not installed");
	}
	return link -> m_N_out_truck -> get_result(end_time) - link -> m_N_out_truck -> get_result(start_time);
}

// new added Jiachao
TFlt get_curb_inflow_car(MNM_Dlink_Multiclass* link, 
                    	TFlt start_time, TFlt end_time)
{
	if (link == nullptr){
		throw std::runtime_error("Error, get_curb_inflow_car link is null");
	}

	if (link -> m_N_in_car_cc == nullptr){
		throw std::runtime_error("Error, get_curb_inflow_car link cumulative curve cc (curb) is not installed");
	}

	return link -> m_N_in_car_cc -> get_result(TFlt(end_time)) - link -> m_N_in_car_cc -> get_result(TFlt(start_time));
}

TFlt get_curb_outflow_car(MNM_Dlink_Multiclass* link, 
                    	TFlt start_time, TFlt end_time)
{
	if (link == nullptr){
		throw std::runtime_error("Error, get_curb_outflow_car link is null");
	}

	if (link -> m_N_out_car_cc == nullptr){
		throw std::runtime_error("Error, get_curb_outflow_car link cumulative curve cc (curb) is not installed");
	}

	return link -> m_N_out_car_cc -> get_result(TFlt(end_time)) - link -> m_N_out_car_cc -> get_result(TFlt(start_time));
}

TFlt get_curb_inflow_truck(MNM_Dlink_Multiclass* link, 
                    	TFlt start_time, TFlt end_time)
{
	if (link == nullptr){
		throw std::runtime_error("Error, get_curb_inflow_truck link is null");
	}

	if (link -> m_N_in_truck_cc == nullptr){
		throw std::runtime_error("Error, get_curb_inflow_truck link cumulative curve cc (curb) is not installed");
	}

	return link -> m_N_in_truck_cc -> get_result(TFlt(end_time)) - link -> m_N_in_truck_cc -> get_result(TFlt(start_time));
}

TFlt get_curb_outflow_truck(MNM_Dlink_Multiclass* link, 
                    	TFlt start_time, TFlt end_time)
{
	if (link == nullptr){
		throw std::runtime_error("Error, get_curb_outflow_truck link is null");
	}

	if (link -> m_N_out_truck_cc == nullptr){
		throw std::runtime_error("Error, get_curb_outflow_truck link cumulative curve cc (curb) is not installed");
	}

	return link -> m_N_out_truck_cc -> get_result(TFlt(end_time)) - link -> m_N_out_truck_cc -> get_result(TFlt(start_time));
}

TFlt get_curb_inflow_rh(MNM_Dlink_Multiclass* link, 
                    	TFlt start_time, TFlt end_time)
{
	if (link == nullptr){
		throw std::runtime_error("Error, get_curb_inflow_truck link is null");
	}

	if (link -> m_N_in_truck_cc == nullptr){
		throw std::runtime_error("Error, get_curb_inflow_truck link cumulative curve cc (curb) is not installed");
	}

	return link -> m_N_in_rh_cc -> get_result(TFlt(end_time)) - link -> m_N_in_rh_cc -> get_result(TFlt(start_time));
}

TFlt get_curb_outflow_rh(MNM_Dlink_Multiclass* link, 
                    	TFlt start_time, TFlt end_time)
{
	if (link == nullptr){
		throw std::runtime_error("Error, get_curb_outflow_truck link is null");
	}

	if (link -> m_N_out_car_cc == nullptr){
		throw std::runtime_error("Error, get_curb_outflow_truck link cumulative curve cc (curb) is not installed");
	}

	return link -> m_N_out_rh_cc -> get_result(TFlt(end_time)) - link -> m_N_out_rh_cc -> get_result(TFlt(start_time));	
}

// new added end

TFlt get_average_waiting_time_at_intersection(MNM_Dlink_Multiclass* link)
{
	if (link == nullptr){
		throw std::runtime_error("Error, get_average_waiting_time_at_intersection link is null");
	}
	if (link -> m_N_in_car == nullptr){
		throw std::runtime_error("Error, get_average_waiting_time_at_intersection link car in cumulative curve is not installed");
	}
	if (link -> m_N_in_truck == nullptr){
		throw std::runtime_error("Error, get_average_waiting_time_at_intersection link truck in cumulative curve is not installed");
	}
	TFlt _tot_vehs = 0;
	_tot_vehs = link -> m_N_in_car -> m_recorder.back().second + link -> m_N_in_truck -> m_recorder.back().second;

	return link -> m_tot_wait_time_at_intersection / _tot_vehs;
}

TInt get_is_spillback(MNM_Dlink_Multiclass* link) // 0 - no spillback, 1 - spillback
{
	if (link == nullptr){
		throw std::runtime_error("Error, get_is_spillback link is null");
	}
	if (link -> m_spill_back){
		return 1;
	}
	else {
		return 0;
	}
}

TFlt get_travel_time_from_FD_car(MNM_Dlink_Multiclass *link, TFlt start_time, TFlt unit_interval)
{
    TFlt _flow = link -> m_N_in_car -> get_result(start_time) - link -> m_N_out_car -> get_result(start_time);
    if (_flow < 0.) _flow = 0.;
    TFlt _tt = link ->get_link_tt_from_flow_car(_flow);
    return _tt / unit_interval;
}

TFlt get_travel_time_from_FD_truck(MNM_Dlink_Multiclass *link, TFlt start_time, TFlt unit_interval)
{
    TFlt _flow = link -> m_N_in_truck -> get_result(start_time) - link -> m_N_out_truck -> get_result(start_time);
    if (_flow < 0.) _flow = 0.;
    TFlt _tt = link ->get_link_tt_from_flow_truck(_flow);
    return _tt / unit_interval;
}

/* Speed Problem from Qiling */
/* changed to m_N_in/out_car 2024-03-09 */
TFlt get_travel_time_car(MNM_Dlink_Multiclass* link, TFlt start_time, TFlt unit_interval, TInt end_loading_timestamp)
{
	if (link == nullptr){
		throw std::runtime_error("Error, get_travel_time_car link is null");
	}
	if (link -> m_N_in_car == nullptr){
		throw std::runtime_error("Error, get_travel_time_car link cumulative curve is not installed");
	}

	TFlt fftt = TFlt(int(link -> get_link_freeflow_tt_loading_car())); // actual intervals in loading

	if (link -> m_last_valid_time < 0){
		link -> m_last_valid_time = get_last_valid_time(link -> m_N_in_car_all, link -> m_N_out_car_all, end_loading_timestamp); // end_loading_timestamp
	}

	IAssert(link -> m_last_valid_time >= 0);

	return get_travel_time_from_cc(start_time, link -> m_N_in_car_all, link -> m_N_out_car_all, link -> m_last_valid_time, fftt);
}


TFlt get_travel_time_car_robust(MNM_Dlink_Multiclass* link, TFlt start_time, TFlt end_time, TFlt unit_interval, TInt end_loading_timestamp, TInt num_trials)
{
	IAssert(end_time > start_time);
	num_trials = num_trials > TInt(end_time - start_time) ? TInt(end_time - start_time) : num_trials;
	TFlt _delta = (end_time - start_time) / TFlt(num_trials);
	TFlt _ave_tt = TFlt(0);
	for (int i=0; i < num_trials(); ++i){
		_ave_tt += get_travel_time_car(link, start_time + TFlt(i) * _delta, unit_interval, end_loading_timestamp);
	}
	return _ave_tt / TFlt(num_trials);
}


TFlt get_travel_time_truck(MNM_Dlink_Multiclass* link, TFlt start_time, TFlt unit_interval, TInt end_loading_timestamp)
{
	if (link == nullptr){
		throw std::runtime_error("Error, get_travel_time_truck link is null");
	}
	if (link -> m_N_in_truck == nullptr){
		throw std::runtime_error("Error, get_travel_time_truck link cumulative curve is not installed");
	}

	TFlt fftt = TFlt(int(link -> get_link_freeflow_tt_loading_truck()));  // actual intervals in loading

	if (link -> m_last_valid_time_truck < 0) {
		link -> m_last_valid_time_truck = get_last_valid_time(link -> m_N_in_truck, link -> m_N_out_truck, end_loading_timestamp);
	}
	IAssert(link -> m_last_valid_time_truck >= 0);

	return get_travel_time_from_cc(start_time, link -> m_N_in_truck, link -> m_N_out_truck, link -> m_last_valid_time_truck, fftt);
}

TFlt get_travel_time_truck_robust(MNM_Dlink_Multiclass* link, TFlt start_time, TFlt end_time, TFlt unit_interval, TInt end_loading_timestamp, TInt num_trials)
{
	IAssert(end_time > start_time);
	num_trials = num_trials > TInt(end_time - start_time) ? TInt(end_time - start_time) : num_trials;
	TFlt _delta = (end_time - start_time) / TFlt(num_trials);
	TFlt _ave_tt = TFlt(0);
	for (int i=0; i < num_trials(); ++i){
		_ave_tt += get_travel_time_truck(link, start_time + TFlt(i) * _delta, unit_interval, end_loading_timestamp);
	}
	return _ave_tt / TFlt(num_trials);
}

// here the pathset is m_path_set not m_path_vec in dta_api
int add_dar_records_car(std::vector<dar_record*> &record, MNM_Dlink_Multiclass* link, 
                        std::set<MNM_Path*> pathset, TFlt start_time, TFlt end_time)
{
	if (link == nullptr){
		throw std::runtime_error("Error, add dar records car link is null");
	}
	if (link -> m_N_in_tree_car == nullptr){
		throw std::runtime_error("Error, add dar records car link cumulative curve tree is not installed");
	}
	MNM_Path* _path;
	for (auto path_it : link -> m_N_in_tree_car -> m_record){
		_path = path_it.first;
		if (pathset.find(_path) != pathset.end()) {
		for (auto depart_it : path_it.second){
			TFlt tmp_flow = depart_it.second -> get_result(end_time) - depart_it.second -> get_result(start_time);
			if (tmp_flow > DBL_EPSILON){
				auto new_record = new dar_record();
				new_record -> path_ID = path_it.first -> m_path_ID;
				// the count of 1 min intervals, the vehicles record this assign_int
				new_record -> assign_int = depart_it.first;
				new_record -> link_ID = link -> m_link_ID;
				// the count of unit time interval (5s)
				new_record -> link_start_int = start_time;
				new_record -> flow = tmp_flow;
				// printf("Adding record, %d, %d, %d, %f, %f\n", new_record -> path_ID(), new_record -> assign_int(), 
				//     new_record -> link_ID(), (float)new_record -> link_start_int(), (float) new_record -> flow());
				record.push_back(new_record);
			}
		}
		}
	}
	return 0;
}

int add_dar_records_truck(std::vector<dar_record*> &record, MNM_Dlink_Multiclass* link, 
                          std::set<MNM_Path*> pathset, TFlt start_time, TFlt end_time)
{
	if (link == nullptr){
		throw std::runtime_error("Error, add_dar_records_truck link is null");
	}
	if (link -> m_N_in_tree_truck == nullptr){
		throw std::runtime_error("Error, add_dar_records_truck link cumulative curve tree is not installed");
	}
	MNM_Path* _path;
	for (auto path_it : link -> m_N_in_tree_truck -> m_record){
		_path = path_it.first;
		if (pathset.find(_path) != pathset.end()) {
			for (auto depart_it : path_it.second){
				TFlt tmp_flow = depart_it.second -> get_result(end_time) - depart_it.second -> get_result(start_time);
				if (tmp_flow > DBL_EPSILON){
					auto new_record = new dar_record();
					new_record -> path_ID = path_it.first -> m_path_ID;
					// the count of 1 min intervals, the vehicles record this assign_int
					new_record -> assign_int = depart_it.first;
					new_record -> link_ID = link -> m_link_ID;
					// the count of unit time interval (5s)
					new_record -> link_start_int = start_time;
					new_record -> flow = tmp_flow;
					//   printf("Adding record, %d, %d, %d, %f, %f\n", new_record -> path_ID(), new_record -> assign_int(), 
					//       new_record -> link_ID(), (float)new_record -> link_start_int(), (float) new_record -> flow());
					record.push_back(new_record);
				}
			}
		}
	}
	return 0;
}

int add_dar_records_car_out(std::vector<dar_record*> &record, MNM_Dlink_Multiclass* link, 
                    std::set<MNM_Path*> pathset, TFlt start_time, TFlt end_time)
{
	if (link == nullptr){
		throw std::runtime_error("Error, add dar records car link is null");
	}
	if (link -> m_N_out_tree_car == nullptr){
		throw std::runtime_error("Error, add dar records car link cumulative curve tree (out) is not installed");
	}
	MNM_Path* _path;
	for (auto path_it : link -> m_N_out_tree_car -> m_record){
		_path = path_it.first;
		if (pathset.find(_path) != pathset.end()) {
			for (auto depart_it : path_it.second){
				TFlt tmp_flow = depart_it.second -> get_result(end_time) - depart_it.second -> get_result(start_time);
				if (tmp_flow > DBL_EPSILON){
					auto new_record = new dar_record();
					new_record -> path_ID = path_it.first -> m_path_ID;
					// the count of 1 min intervals, the vehicles record this assign_int
					new_record -> assign_int = depart_it.first;
					new_record -> link_ID = link -> m_link_ID;
					// the count of unit time interval (5s)
					new_record -> link_start_int = start_time;
					new_record -> flow = tmp_flow;
					// printf("Adding record, %d, %d, %d, %f, %f\n", new_record -> path_ID(), new_record -> assign_int(), 
					//     new_record -> link_ID(), (float)new_record -> link_start_int(), (float) new_record -> flow());
					record.push_back(new_record);
				}
			}
		}
	}
	return 0;
}

int add_dar_records_truck_out(std::vector<dar_record*> &record, MNM_Dlink_Multiclass* link, 
                    std::set<MNM_Path*> pathset, TFlt start_time, TFlt end_time)
{
	if (link == nullptr){
		throw std::runtime_error("Error, add dar records car link is null");
	}
	if (link -> m_N_out_tree_truck == nullptr){
		throw std::runtime_error("Error, add dar records car link cumulative curve tree (out) is not installed");
	}
	MNM_Path* _path;
	for (auto path_it : link -> m_N_out_tree_truck -> m_record){
		_path = path_it.first;
		if (pathset.find(_path) != pathset.end()) {
			for (auto depart_it : path_it.second){
				TFlt tmp_flow = depart_it.second -> get_result(end_time) - depart_it.second -> get_result(start_time);
				if (tmp_flow > DBL_EPSILON){
					auto new_record = new dar_record();
					new_record -> path_ID = path_it.first -> m_path_ID;
					// the count of 1 min intervals, the vehicles record this assign_int
					new_record -> assign_int = depart_it.first;
					new_record -> link_ID = link -> m_link_ID;
					// the count of unit time interval (5s)
					new_record -> link_start_int = start_time;
					new_record -> flow = tmp_flow;
					// printf("Adding record, %d, %d, %d, %f, %f\n", new_record -> path_ID(), new_record -> assign_int(), 
					//     new_record -> link_ID(), (float)new_record -> link_start_int(), (float) new_record -> flow());
					record.push_back(new_record);
				}
			}
		}
	}
	return 0;
}

int add_dar_records_rh_out(std::vector<dar_record*> &record, MNM_Dlink_Multiclass* link, 
                    std::set<MNM_Path*> pathset, TFlt start_time, TFlt end_time)
{
	if (link == nullptr){
		throw std::runtime_error("Error, add dar records rh link is null");
	}
	if (link -> m_N_out_tree_rh == nullptr){
		throw std::runtime_error("Error, add dar records rh link cumulative curve tree (out) is not installed");
	}
	MNM_Path* _path;
	for (auto path_it : link -> m_N_out_tree_rh -> m_record){
		_path = path_it.first;
		if (pathset.find(_path) != pathset.end()) {
			for (auto depart_it : path_it.second){
				TFlt tmp_flow = depart_it.second -> get_result(end_time) - depart_it.second -> get_result(start_time);
				if (tmp_flow > DBL_EPSILON){
					auto new_record = new dar_record();
					new_record -> path_ID = path_it.first -> m_path_ID;
					// the count of 1 min intervals, the vehicles record this assign_int
					new_record -> assign_int = depart_it.first;
					new_record -> link_ID = link -> m_link_ID;
					// the count of unit time interval (5s)
					new_record -> link_start_int = start_time;
					new_record -> flow = tmp_flow;
					// printf("Adding record, %d, %d, %d, %f, %f\n", new_record -> path_ID(), new_record -> assign_int(), 
					//     new_record -> link_ID(), (float)new_record -> link_start_int(), (float) new_record -> flow());
					record.push_back(new_record);
				}
			}
		}
	}
	return 0;
}

int add_dar_records_rh(std::vector<dar_record*> &record, MNM_Dlink_Multiclass* link, 
                        std::set<MNM_Path*> pathset, TFlt start_time, TFlt end_time)
{
  if (link == nullptr){
    throw std::runtime_error("Error, add_dar_records_rh link is null");
  }
  if (link -> m_N_in_tree_rh == nullptr){
    throw std::runtime_error("Error, add_dar_records_rh link cumulative curve tree is not installed");
  }
  MNM_Path* _path;
  for (auto path_it : link -> m_N_in_tree_rh -> m_record){
    _path = path_it.first;
	if (pathset.find(_path) != pathset.end()) {
      for (auto depart_it : path_it.second){
        TFlt tmp_flow = depart_it.second -> get_result(end_time) - depart_it.second -> get_result(start_time);
        if (tmp_flow > DBL_EPSILON){
          auto new_record = new dar_record();
          new_record -> path_ID = path_it.first -> m_path_ID;
          // the count of 1 min intervals, the vehicles record this assign_int
          new_record -> assign_int = depart_it.first;
          new_record -> link_ID = link -> m_link_ID;
          // the count of unit time interval (5s)
          new_record -> link_start_int = start_time;
          new_record -> flow = tmp_flow;
          // printf("Adding record, %d, %d, %d, %f, %f\n", new_record -> path_ID(), new_record -> assign_int(), 
          //     new_record -> link_ID(), (float)new_record -> link_start_int(), (float) new_record -> flow());
          record.push_back(new_record);
        }
      }
    }
  }
  return 0;
}

int add_dar_records_car(std::vector<dar_record*> &record, MNM_Dlink_Multiclass* link, 
                        std::set<TInt> pathID_set, TFlt start_time, TFlt end_time)
{
  if (link == nullptr){
    throw std::runtime_error("Error, add dar records car link is null");
  }
  if (link -> m_N_in_tree_car == nullptr){
    throw std::runtime_error("Error, add dar records car link cumulative curve tree is not installed");
  }

  MNM_Path* _path;
  for (auto path_it : link -> m_N_in_tree_car -> m_record){
    _path = path_it.first;
	if (pathID_set.find(_path -> m_path_ID) != pathID_set.end()) {
      for (auto depart_it : path_it.second){
        TFlt tmp_flow = depart_it.second -> get_result(end_time) - depart_it.second -> get_result(start_time);
        if (tmp_flow > DBL_EPSILON){
          auto new_record = new dar_record();
          new_record -> path_ID = path_it.first -> m_path_ID;
          // the count of 1 min intervals, the vehicles record this assign_int
          new_record -> assign_int = depart_it.first;
          new_record -> link_ID = link -> m_link_ID;
          // the count of unit time interval (5s)
          new_record -> link_start_int = start_time;
          new_record -> flow = tmp_flow;
          // printf("Adding record, %d, %d, %d, %f, %f\n", new_record -> path_ID(), new_record -> assign_int(), 
          //     new_record -> link_ID(), (float)new_record -> link_start_int(), (float) new_record -> flow());
          record.push_back(new_record);
        }
      }
    }
  }
  return 0;
}

int add_dar_records_truck(std::vector<dar_record*> &record, MNM_Dlink_Multiclass* link, 
                          std::set<TInt> pathID_set, TFlt start_time, TFlt end_time)
{
  if (link == nullptr){
    throw std::runtime_error("Error, add dar records truck link is null");
  }
  if (link -> m_N_in_tree_truck == nullptr){
    throw std::runtime_error("Error, add dar records truck link cumulative curve tree is not installed");
  }

  MNM_Path* _path;
  for (auto path_it : link -> m_N_in_tree_truck -> m_record){
    _path = path_it.first;
	if (pathID_set.find(_path -> m_path_ID) != pathID_set.end()) {
      for (auto depart_it : path_it.second){
        TFlt tmp_flow = depart_it.second -> get_result(end_time) - depart_it.second -> get_result(start_time);
        if (tmp_flow > DBL_EPSILON){
          auto new_record = new dar_record();
          new_record -> path_ID = path_it.first -> m_path_ID;
          // the count of 1 min intervals, the vehicles record this assign_int
          new_record -> assign_int = depart_it.first;
          new_record -> link_ID = link -> m_link_ID;
          // the count of unit time interval (5s)
          new_record -> link_start_int = start_time;
          new_record -> flow = tmp_flow;
          // printf("Adding record, %d, %d, %d, %f, %f\n", new_record -> path_ID(), new_record -> assign_int(), 
          //     new_record -> link_ID(), (float)new_record -> link_start_int(), (float) new_record -> flow());
          record.push_back(new_record);
        }
		else {
		  throw std::runtime_error("Error, no adding record");
		}
      }
    }
  }
  return 0;
}

int add_dar_records_rh(std::vector<dar_record*> &record, MNM_Dlink_Multiclass* link, 
                        std::set<TInt> pathID_set, TFlt start_time, TFlt end_time)
{
  if (link == nullptr){
    throw std::runtime_error("Error, add dar records rh link is null");
  }
  if (link -> m_N_in_tree_rh == nullptr){
    throw std::runtime_error("Error, add dar records rh link cumulative curve tree is not installed");
  }

  MNM_Path* _path;
  for (auto path_it : link -> m_N_in_tree_rh -> m_record){
    _path = path_it.first;
	if (pathID_set.find(_path -> m_path_ID) != pathID_set.end()) {
      for (auto depart_it : path_it.second){
        TFlt tmp_flow = depart_it.second -> get_result(end_time) - depart_it.second -> get_result(start_time);
        if (tmp_flow > DBL_EPSILON){
          auto new_record = new dar_record();
          new_record -> path_ID = path_it.first -> m_path_ID;
          // the count of 1 min intervals, the vehicles record this assign_int
          new_record -> assign_int = depart_it.first;
          new_record -> link_ID = link -> m_link_ID;
          // the count of unit time interval (5s)
          new_record -> link_start_int = start_time;
          new_record -> flow = tmp_flow;
          // printf("Adding record, %d, %d, %d, %f, %f\n", new_record -> path_ID(), new_record -> assign_int(), 
          //     new_record -> link_ID(), (float)new_record -> link_start_int(), (float) new_record -> flow());
          record.push_back(new_record);
        }
      }
    }
  }
  return 0;
}

// new added
int add_dar_records_curb_arrival_car(std::vector<dar_record*> &record, MNM_Dlink_Multiclass* link, 
									std::set<MNM_Path*> pathset, TFlt start_time, TFlt end_time)
{
	if (link == nullptr){
		throw std::runtime_error("Error, add dar records curb arrival car link is null");
	}
	if (link -> m_N_in_tree_curb_car == nullptr){
		throw std::runtime_error("Error, arrival curb CC tree for car on this link is not installed");
	}
	MNM_Path* _path;
	for (auto path_it : link -> m_N_in_tree_curb_car -> m_record){
		_path = path_it.first;
		if (pathset.find(_path) != pathset.end()) {
			for (auto depart_it : path_it.second){
				TFlt tmp_flow = depart_it.second -> get_result(end_time) - depart_it.second -> get_result(start_time);
				if (tmp_flow > DBL_EPSILON){
					auto new_record = new dar_record();
					new_record -> path_ID = path_it.first -> m_path_ID;
					// the count of 1 min intervals, the vehicles record this assign_int
					new_record -> assign_int = depart_it.first;
					new_record -> link_ID = link -> m_link_ID;
					// the count of unit time interval (5s)
					new_record -> link_start_int = start_time;
					new_record -> flow = tmp_flow;
					// printf("Adding record, %d, %d, %d, %f, %f\n", new_record -> path_ID(), new_record -> assign_int(), 
					//     new_record -> link_ID(), (float)new_record -> link_start_int(), (float) new_record -> flow());
					record.push_back(new_record);
				}
			}
		}
	}
	return 0;
}

int add_dar_records_curb_departure_car(std::vector<dar_record*> &record, MNM_Dlink_Multiclass* link, 
									std::set<MNM_Path*> pathset, TFlt start_time, TFlt end_time)
{
	if (link == nullptr){
		throw std::runtime_error("Error, add dar records curb departure car link is null");
	}

	if (link -> m_N_out_tree_curb_car == nullptr){
		throw std::runtime_error("Error, arrival curb CC tree for car on this link is not installed");
	}
	MNM_Path* _path;
	for (auto path_it : link -> m_N_out_tree_curb_car -> m_record){
		_path = path_it.first;
		if (pathset.find(_path) != pathset.end()) {
			for (auto depart_it : path_it.second){
				TFlt tmp_flow = depart_it.second -> get_result(end_time) - depart_it.second -> get_result(start_time);
				if (tmp_flow > DBL_EPSILON){
					auto new_record = new dar_record();
					new_record -> path_ID = path_it.first -> m_path_ID;
					// the count of 1 min intervals, the vehicles record this assign_int
					new_record -> assign_int = depart_it.first;
					new_record -> link_ID = link -> m_link_ID;
					// the count of unit time interval (5s)
					new_record -> link_start_int = start_time;
					new_record -> flow = tmp_flow;
					// printf("Adding record, %d, %d, %d, %f, %f\n", new_record -> path_ID(), new_record -> assign_int(), 
					//     new_record -> link_ID(), (float)new_record -> link_start_int(), (float) new_record -> flow());
					record.push_back(new_record);
				}
			}
		}
	}
	return 0;
}

int add_dar_records_curb_arrival_truck(std::vector<dar_record*> &record, MNM_Dlink_Multiclass* link, 
									std::set<MNM_Path*> pathset, TFlt start_time, TFlt end_time)
{
	if (link == nullptr){
		throw std::runtime_error("Error, add dar records curb arrival truck link is null");
	}

	// TODO add m_N_in_tree_curb_truck in link class
	// in tree is related to arrival and out tree is departure
	if (link -> m_N_in_tree_curb_truck == nullptr){
		throw std::runtime_error("Error, arrival curb CC tree for truck on this link is not installed");
	}
	MNM_Path* _path;
	for (auto path_it : link -> m_N_in_tree_curb_truck -> m_record){
		_path = path_it.first;
		if (pathset.find(_path) != pathset.end()) {
			for (auto depart_it : path_it.second){
				TFlt tmp_flow = depart_it.second -> get_result(end_time) - depart_it.second -> get_result(start_time);
				if (tmp_flow > DBL_EPSILON){
					auto new_record = new dar_record();
					new_record -> path_ID = path_it.first -> m_path_ID;
					// the count of 1 min intervals, the vehicles record this assign_int
					new_record -> assign_int = depart_it.first;
					new_record -> link_ID = link -> m_link_ID;
					// the count of unit time interval (5s)
					new_record -> link_start_int = start_time;
					new_record -> flow = tmp_flow;
					// printf("Adding record, %d, %d, %d, %f, %f\n", new_record -> path_ID(), new_record -> assign_int(), 
					//     new_record -> link_ID(), (float)new_record -> link_start_int(), (float) new_record -> flow());
					record.push_back(new_record);
				}
			}
		}
	}
	return 0;
}

int add_dar_records_curb_departure_truck(std::vector<dar_record*> &record, MNM_Dlink_Multiclass* link, 
									std::set<MNM_Path*> pathset, TFlt start_time, TFlt end_time)
{
	if (link == nullptr){
		throw std::runtime_error("Error, add dar records curb departure truck link is null");
	}

	if (link -> m_N_out_tree_curb_truck == nullptr){
		throw std::runtime_error("Error, arrival curb CC tree for truck on this link is not installed");
	}
	MNM_Path* _path;
	for (auto path_it : link -> m_N_out_tree_curb_truck -> m_record){
		_path = path_it.first;
		if (pathset.find(_path) != pathset.end()) {
			for (auto depart_it : path_it.second){
				TFlt tmp_flow = depart_it.second -> get_result(end_time) - depart_it.second -> get_result(start_time);
				if (tmp_flow > DBL_EPSILON){
					auto new_record = new dar_record();
					new_record -> path_ID = path_it.first -> m_path_ID;
					// the count of 1 min intervals, the vehicles record this assign_int
					new_record -> assign_int = depart_it.first;
					new_record -> link_ID = link -> m_link_ID;
					// the count of unit time interval (5s)
					new_record -> link_start_int = start_time;
					new_record -> flow = tmp_flow;
					// printf("Adding record, %d, %d, %d, %f, %f\n", new_record -> path_ID(), new_record -> assign_int(), 
					//     new_record -> link_ID(), (float)new_record -> link_start_int(), (float) new_record -> flow());
					record.push_back(new_record);
				}
			}
		}
	}
	return 0;
}

int add_dar_records_curb_arrival_rh(std::vector<dar_record*> &record, MNM_Dlink_Multiclass* link, 
									std::set<MNM_Path*> pathset, TFlt start_time, TFlt end_time)
{
	if (link == nullptr){
		throw std::runtime_error("Error, add dar records curb arrival RH link is null");
	}

	if (link -> m_N_in_tree_curb_rh == nullptr){
		throw std::runtime_error("Error, arrival curb CC tree for RH on this link is not installed");
	}

	MNM_Path* _path;
	for (auto path_it : link -> m_N_in_tree_curb_rh -> m_record){
		_path = path_it.first;
		if (pathset.find(_path) != pathset.end()) {
			for (auto depart_it : path_it.second){
				TFlt tmp_flow = depart_it.second -> get_result(end_time) - depart_it.second -> get_result(start_time);
				if (tmp_flow > DBL_EPSILON){
					auto new_record = new dar_record();
					new_record -> path_ID = path_it.first -> m_path_ID;
					// the count of 1 min intervals, the vehicles record this assign_int
					new_record -> assign_int = depart_it.first;
					new_record -> link_ID = link -> m_link_ID;
					// the count of unit time interval (5s)
					new_record -> link_start_int = start_time;
					new_record -> flow = tmp_flow;
					// printf("Adding record, %d, %d, %d, %f, %f\n", new_record -> path_ID(), new_record -> assign_int(), 
					//     new_record -> link_ID(), (float)new_record -> link_start_int(), (float) new_record -> flow());
					record.push_back(new_record);
				}
			}
		}
	}
	return 0;
}

int add_dar_records_curb_departure_rh(std::vector<dar_record*> &record, MNM_Dlink_Multiclass* link, 
									std::set<MNM_Path*> pathset, TFlt start_time, TFlt end_time)
{
	if (link == nullptr){
		throw std::runtime_error("Error, add dar records curb departure RH link is null");
	}

	if (link -> m_N_out_tree_curb_rh == nullptr){
		throw std::runtime_error("Error, departure curb CC tree for RH on this link is not installed");
	}
	MNM_Path* _path;
	for (auto path_it : link -> m_N_out_tree_curb_rh -> m_record){
		_path = path_it.first;
		if (pathset.find(_path) != pathset.end()) {
			for (auto depart_it : path_it.second){
				TFlt tmp_flow = depart_it.second -> get_result(end_time) - depart_it.second -> get_result(start_time);
				if (tmp_flow > DBL_EPSILON){
					auto new_record = new dar_record();
					new_record -> path_ID = path_it.first -> m_path_ID;
					// the count of 1 min intervals, the vehicles record this assign_int
					new_record -> assign_int = depart_it.first;
					new_record -> link_ID = link -> m_link_ID;
					// the count of unit time interval (5s)
					new_record -> link_start_int = start_time;
					new_record -> flow = tmp_flow;
					// printf("Adding record, %d, %d, %d, %f, %f\n", new_record -> path_ID(), new_record -> assign_int(), 
					//     new_record -> link_ID(), (float)new_record -> link_start_int(), (float) new_record -> flow());
					record.push_back(new_record);
				}
			}
		}
	}	
	return 0;
}
// new added end

TFlt get_link_fuel_rate_biclass(MNM_Dlink_Multiclass* link, TFlt m_unit_time)
{
	TFlt _fuel_rate_car, _fuel_rate_truck;

	TFlt _v, _v_converted;
	
	_v = link -> m_length / link -> get_link_tt(); // m/s
	_v_converted = _v * TFlt(3600) / TFlt(1600); // mile / hour
	_v_converted = MNM_Ults::max(_v_converted, TFlt(5));
	_v_converted = MNM_Ults::min(_v_converted, TFlt(65));

	TFlt _convert_factor = 1.0;
	if (_v_converted < 25){
		_convert_factor = 1.53;
	}
	else if (_v_converted < 55){
		_convert_factor = 1.50;
	}
	else {
		_convert_factor = 1.55;
	}

	TFlt _fule_eco =  1.19102380 * 1e-07 * pow(_v_converted, 5) 
                  - 2.67383161 * 1e-05 * pow(_v_converted, 4)
                  + 2.35409750 * 1e-03 * pow(_v_converted, 3) 
                  - 1.11752399 * 1e-01 * pow(_v_converted, 2)
                  + 2.96137050 * pow(_v_converted, 1) -1.51623933;
	
	_fuel_rate_car = MNM_Ults::max(TFlt(1)/_fule_eco, TFlt(0));
	_fuel_rate_truck = _fuel_rate_car * _convert_factor;

	return _fuel_rate_car * (_v * m_unit_time / TFlt(1600)) * link -> get_link_flow_car() + _fuel_rate_truck * (_v * m_unit_time / TFlt(1600)) * link -> get_link_flow_truck();
}

TFlt get_link_CO2_rate_biclass(MNM_Dlink_Multiclass* link, TFlt m_unit_time)
{
	TFlt _CO2_rate_car, _CO2_rate_truck;

	TFlt _v, _v_converted;
	
	_v = link -> m_length / link -> get_link_tt(); // m/s
	_v_converted = _v * TFlt(3600) / TFlt(1600); // mile / hour
	_v_converted = MNM_Ults::max(_v_converted, TFlt(5));
	_v_converted = MNM_Ults::min(_v_converted, TFlt(65));

	TFlt _convert_factor = 1.0;
	if (_v_converted < 25){
		_convert_factor = 1.53;
	}
	else if (_v_converted < 55){
		_convert_factor = 1.50;
	}
	else {
		_convert_factor = 1.55;
	}

	TFlt _fule_eco =  1.19102380 * 1e-07 * pow(_v_converted, 5) 
                  - 2.67383161 * 1e-05 * pow(_v_converted, 4)
                  + 2.35409750 * 1e-03 * pow(_v_converted, 3) 
                  - 1.11752399 * 1e-01 * pow(_v_converted, 2)
                  + 2.96137050 * pow(_v_converted, 1) -1.51623933;
  
 	TFlt _fuel_rate = MNM_Ults::max(TFlt(1)/_fule_eco, TFlt(0));

	_CO2_rate_car = MNM_Ults::max(_fuel_rate * TFlt(8887), TFlt(0));

	_CO2_rate_truck = _CO2_rate_car * _convert_factor;
	
	return _CO2_rate_car * (_v * m_unit_time / TFlt(1600)) * link -> get_link_flow_car() + _CO2_rate_truck * (_v * m_unit_time / TFlt(1600)) * link -> get_link_flow_truck();
}

TFlt get_link_HC_rate_biclass(MNM_Dlink_Multiclass* link, TFlt m_unit_time)
{
	TFlt _HC_rate_car, _HC_rate_truck;
	
	TFlt _v, _v_converted;
	
	_v = link -> m_length / link -> get_link_tt(); // m/s
	_v_converted = _v * TFlt(3600) / TFlt(1600); // mile / hour
	_v_converted = MNM_Ults::max(_v_converted, TFlt(5));
	_v_converted = MNM_Ults::min(_v_converted, TFlt(65));

	TFlt _convert_factor = 1.0;
	if (_v_converted < 25){
		_convert_factor = 1.87;
	}
	else if (_v_converted < 55){
		_convert_factor = 2.41;
	}
	else {
		_convert_factor = 2.01;
	}

	TFlt _HC_rate = 1.61479076909784e-13 * pow(_v_converted,8.0) - 1.27884474982285e-10 * pow(_v_converted,7.0)
                + 2.92924270300974e-8 * pow(_v_converted,6.0) - 3.23670086149171e-6 * pow(_v_converted,5.0)
                + 0.000201135990745703 * pow(_v_converted,4.0) - 0.00737871178398462 * pow(_v_converted,3.0)
                + 0.15792241257931 * pow(_v_converted,2.0) - 1.82687242201925 * _v_converted + 9.84559996919605;

	_HC_rate_car = MNM_Ults::max(_HC_rate, TFlt(0));
	_HC_rate_truck = _HC_rate_car * _convert_factor;

	return _HC_rate_car * (_v * m_unit_time / TFlt(1600)) * link -> get_link_flow_car() + _HC_rate_truck * (_v * m_unit_time / TFlt(1600)) * link -> get_link_flow_truck();
}

TFlt get_link_CO_rate_biclass(MNM_Dlink_Multiclass* link, TFlt m_unit_time)
{

	TFlt _CO_rate_car, _CO_rate_truck;

	TFlt _v, _v_converted;
	
	_v = link -> m_length / link -> get_link_tt(); // m/s
	_v_converted = _v * TFlt(3600) / TFlt(1600); // mile / hour
	_v_converted = MNM_Ults::max(_v_converted, TFlt(5));
	_v_converted = MNM_Ults::min(_v_converted, TFlt(65));

	TFlt _convert_factor = 1.0;
	if (_v_converted < 25){
		_convert_factor = 3.97;
	}
	else if (_v_converted < 55){
		_convert_factor = 2.67;
	}
	else {
		_convert_factor = 5.01;
	}

	TFlt _CO_rate = -1.08317411174986 * 1e-12 * pow(_v_converted,8) + 2.53340626614398 * 1e-10 * pow(_v_converted,7)
            -2.12944112670644 * 1e-8* pow(_v_converted,6) + 5.97070024385679 * 1e-7 * pow(_v_converted,5)
            +1.79281854904105 * 1e-5 * pow(_v_converted,4) - 0.00170366500109581 * pow(_v_converted,3)
            +0.047711166912908 * pow(_v_converted,2) - 0.615061016205463 * _v_converted + 4.12900319568868;

	_CO_rate_car = MNM_Ults::max(_CO_rate, TFlt(0));
	_CO_rate_truck = _CO_rate_car * _convert_factor;

	return _CO_rate_car * (_v * m_unit_time / TFlt(1600)) * link -> get_link_flow_car() + _CO_rate_truck * (_v * m_unit_time / TFlt(1600)) * link -> get_link_flow_truck();
}

TFlt get_link_NOX_rate_biclass(MNM_Dlink_Multiclass* link, TFlt m_unit_time)
{
	TFlt _NOX_rate_car, _NOX_rate_truck;

	TFlt _v, _v_converted;
	
	_v = link -> m_length / link -> get_link_tt(); // m/s
	_v_converted = _v * TFlt(3600) / TFlt(1600); // mile / hour
	_v_converted = MNM_Ults::max(_v_converted, TFlt(5));
	_v_converted = MNM_Ults::min(_v_converted, TFlt(65));

	TFlt _convert_factor = 1.0;
	if (_v_converted < 25){
		_convert_factor = 7.32;
	}
	else if (_v_converted < 55){
		_convert_factor = 6.03;
	}
	else {
		_convert_factor = 5.75;
	}

	TFlt _NOX_rate = -6.52009367269462 * 1e-13 * pow(_v_converted,8) + 1.25335312366681 * 1e-10 * pow(_v_converted,7)
        -4.67202313364846 * 1e-9 * pow(_v_converted,6) - 6.63892272105462 * 1e-7 * pow(_v_converted,5)
        +8.01942113220463 * 1e-5 * pow(_v_converted,4) - 0.00374632777368871 * pow(_v_converted,3)
        +0.0895029037098895 * pow(_v_converted,2) - 1.07265851515536 * _v_converted + 6.06514023873933;
	
	_NOX_rate_car = MNM_Ults::max(_NOX_rate, TFlt(0));
	_NOX_rate_truck = _NOX_rate_car * _convert_factor;

	return _NOX_rate_car * (_v * m_unit_time / TFlt(1600)) * link -> get_link_flow_car() + _NOX_rate_truck * (_v * m_unit_time / TFlt(1600)) * link -> get_link_flow_truck();
}

TFlt get_link_VMT_biclass(MNM_Dlink_Multiclass* link, TFlt m_unit_time)
{
	TFlt _v, _VMT_car, _VMT_truck;

	_v = link -> m_length / link -> get_link_tt(); // m/s

	_VMT_car = (_v * m_unit_time / TFlt(1600)) * link -> get_link_flow_car();

	_VMT_truck = (_v * m_unit_time / TFlt(1600)) * link -> get_link_flow_truck();

	return _VMT_car + _VMT_truck;
}

TFlt get_link_VHT_biclass(MNM_Dlink_Multiclass* link, TFlt m_unit_time)
{
	TFlt _v, _VHT_car, _VHT_truck;

	_v = link -> m_length / link -> get_link_tt(); // m/s

	_VHT_car += m_unit_time * link -> get_link_flow_car() / 3600;
	_VHT_truck += m_unit_time * link -> get_link_flow_truck() / 3600;

	return _VHT_car + _VHT_truck;
}

TFlt get_departure_cc_slope_car(MNM_Dlink_Multiclass* link, TFlt start_time, TFlt end_time)
{
	if (link == nullptr){
		throw std::runtime_error("Error, get_departure_cc_slope_car link is null");
	}
	if (link -> m_N_out_car == nullptr){
		throw std::runtime_error("Error, get_departure_cc_slope_car link cumulative curve is not installed");
	}
	if (start_time > link -> m_N_out_car->m_recorder.back().first) {
		return 0;
	}
	int _delta = int(end_time) - int(start_time);
	IAssert(_delta > 0);
	TFlt _cc1, _cc2, _slope = 0.;
	for (int i = 0; i < _delta; i++) {
		_cc1 = link -> m_N_out_car -> get_result(TFlt(start_time + i));
		_cc2 = link -> m_N_out_car -> get_result(TFlt(start_time + i + 1));
		_slope += (_cc2 -_cc1);
	}
	// if (_slope <= 0) {
	// 	std::cout << "car in" << "\n";
	// 	std::cout << link -> m_N_in_car -> to_string() << "\n";
	// 	std::cout << "car out" << "\n";
	// 	std::cout << link -> m_N_out_car -> to_string() << "\n";
	// 	printf("debug");
	// }
	return _slope / _delta;  // flow per unit interval (notes: average c)
}

TFlt get_departure_cc_slope_truck(MNM_Dlink_Multiclass* link, TFlt start_time, TFlt end_time)
{
	if (link == nullptr){
		throw std::runtime_error("Error, get_departure_cc_slope_truck link is null");
	}
	if (link -> m_N_out_truck == nullptr){
		throw std::runtime_error("Error, get_departure_cc_slope_truck link cumulative curve is not installed");
	}
	if (start_time > link -> m_N_out_truck->m_recorder.back().first) {
		return 0;
	}
	int _delta = int(end_time) - int(start_time);
	IAssert(_delta > 0);
	TFlt _cc1, _cc2, _slope = 0.;
	for (int i = 0; i < _delta; i++) {
		_cc1 = link -> m_N_out_truck -> get_result(TFlt(start_time + i));
		_cc2 = link -> m_N_out_truck -> get_result(TFlt(start_time + i + 1));
		_slope += (_cc2 -_cc1);
	}
	return _slope / _delta;  // flow per unit interval
}

int add_ltg_records_veh(std::vector<ltg_record*> &record, MNM_Dlink_Multiclass *link,
						MNM_Path* path, int depart_time, int start_time, TFlt gradient)
{
	if (link == nullptr){
		throw std::runtime_error("Error, add_ltg_records_veh link is null");
	}
	if (path == nullptr){
		throw std::runtime_error("Error, add_ltg_records_veh path is null");
	}
	if (!path -> is_link_in(link -> m_link_ID)){
		throw std::runtime_error("Error, add_ltg_records_veh link is not in path");
	}

	auto new_record = new ltg_record();
	new_record -> path_ID = path -> m_path_ID;
	// the count of 1 min intervals in terms of 5s intervals, the vehicles record this assign_int
	new_record -> assign_int = depart_time;
	new_record -> link_ID = link -> m_link_ID;
	// the count of unit time interval (5s)
	new_record -> link_start_int = start_time;
	new_record -> gradient = gradient;
	// printf("Adding record, %d, %d, %d, %d, %f\n", new_record -> path_ID(), new_record -> assign_int, 
	//         new_record -> link_ID(), new_record -> link_start_int, (float) new_record -> gradient());
	record.push_back(new_record);
	return 0;
}

// Fujitsu paper - density from satellite images
TFlt get_link_density_from_cc(TFlt time, MNM_Cumulative_Curve *N_in, MNM_Cumulative_Curve *N_out, TFlt last_valid_time)
{
	TFlt _density = 0.0;

	if (last_valid_time <= 0){
		_density = 0.0;
	}
	if (time > last_valid_time){
		time = last_valid_time;
	}

	TFlt _flow_in = N_in -> get_result(time);

	if (_flow_in <= DBL_EPSILON){
		_density = 0.0;
		// throw std::runtime_error("Error-2, _density = 0\n");
	}
	else{
		TFlt _flow_out = N_out -> get_result(time);

		_density = _flow_in - _flow_out;

		if (_density < 0.0){
			_density = 0.0;
		}
	}
	return _density;
}

TFlt get_link_density_car(MNM_Dlink_Multiclass* link, TFlt time, TInt end_loading_timestamp)
{
	if (link == nullptr){
    	throw std::runtime_error ("Error, get_link_density_car link is null");
    }
  	if ((link -> m_N_in_car == nullptr) || (link -> m_N_out_car == nullptr)){
    	throw std::runtime_error ("Error, get_link_density_car link cumulative curve is not installed");
    }

	if (link -> m_last_valid_time < 0){
    	link -> m_last_valid_time = get_last_valid_time(link -> m_N_in_car, link -> m_N_out_car, end_loading_timestamp);
    }

  	IAssert (link -> m_last_valid_time >= 0);

	return get_link_density_from_cc(time, link -> m_N_in_car, link -> m_N_out_car, link -> m_last_valid_time);
}

TFlt get_link_density_truck(MNM_Dlink_Multiclass* link, TFlt time, TInt end_loading_timestamp)
{
	if (link == nullptr){
    	throw std::runtime_error ("Error, get_link_density_truck link is null");
    }
  	if ((link -> m_N_in_truck == nullptr) || (link -> m_N_out_truck == nullptr)){
    	throw std::runtime_error ("Error, get_link_density_truck link cumulative curve is not installed");
    }

	if (link -> m_last_valid_time < 0){
    	link -> m_last_valid_time = get_last_valid_time(link -> m_N_in_truck, link -> m_N_out_truck, end_loading_timestamp);
    }

  	IAssert (link -> m_last_valid_time >= 0);

	return get_link_density_from_cc(time, link -> m_N_in_truck, link -> m_N_out_truck, link -> m_last_valid_time);
}

TFlt get_link_density_stop_car(MNM_Dlink_Multiclass* link, TFlt time, TInt end_loading_timestamp)
{
	if (link == nullptr){
    	throw std::runtime_error ("Error, get_link_density_stop_car link is null");
    }
  	if ((link -> m_N_in_car_cc == nullptr) || (link -> m_N_out_car_cc == nullptr)){
    	throw std::runtime_error ("Error, get_link_density_stop_car link cumulative curve is not installed");
    }

	if (link -> m_last_valid_time < 0){
    	link -> m_last_valid_time = get_last_valid_time(link -> m_N_in_car_cc, link -> m_N_out_car_cc, end_loading_timestamp);
    }

  	IAssert (link -> m_last_valid_time >= 0);

	return get_link_density_from_cc(time, link -> m_N_in_car_cc, link -> m_N_out_car_cc, link -> m_last_valid_time);
}

TFlt get_link_density_stop_truck(MNM_Dlink_Multiclass* link, TFlt time, TInt end_loading_timestamp)
{
	if (link == nullptr){
    	throw std::runtime_error ("Error, get_link_density_stop_truck link is null");
    }
  	if ((link -> m_N_in_truck_cc == nullptr) || (link -> m_N_out_truck_cc == nullptr)){
    	throw std::runtime_error ("Error, get_link_density_stop_truck link cumulative curve is not installed");
    }

	if (link -> m_last_valid_time < 0){
    	link -> m_last_valid_time = get_last_valid_time(link -> m_N_in_truck_cc, link -> m_N_out_truck_cc, end_loading_timestamp);
    }

  	IAssert (link -> m_last_valid_time >= 0);

	return get_link_density_from_cc(time, link -> m_N_in_truck_cc, link -> m_N_out_truck_cc, link -> m_last_valid_time);
}

TFlt get_link_density_car_robust(MNM_Dlink_Multiclass* link, TFlt time, TInt end_loading_timestamp, TInt num_trials)
{
	if (link == nullptr){
    	throw std::runtime_error ("Error, get_link_density_car link is null");
    }
  	if ((link -> m_N_in_car == nullptr) || (link -> m_N_out_car == nullptr)){
    	throw std::runtime_error ("Error, get_link_density_car link cumulative curve is not installed");
    }

	if (link -> m_last_valid_time < 0){
    	link -> m_last_valid_time = get_last_valid_time(link -> m_N_in_car, link -> m_N_out_car, end_loading_timestamp);
    }

  	IAssert (link -> m_last_valid_time >= 0);

	TFlt _total_density = 0.0;
	TFlt _time_tmp;

	for (int i = 0; i < num_trials; ++i){
		_time_tmp = TFlt(i - 2 + time) > TFlt(0) ? TFlt(i - 2 + time) : TFlt(0);
		_total_density += get_link_density_from_cc(_time_tmp, link -> m_N_in_car, link -> m_N_out_car, link -> m_last_valid_time);
	}

	return _total_density/num_trials;
}

TFlt get_link_density_truck_robust(MNM_Dlink_Multiclass* link, TFlt time, TInt end_loading_timestamp, TInt num_trials)
{
	if (link == nullptr){
    	throw std::runtime_error ("Error, get_link_density_truck link is null");
    }
  	if ((link -> m_N_in_truck == nullptr) || (link -> m_N_out_truck == nullptr)){
    	throw std::runtime_error ("Error, get_link_density_truck link cumulative curve is not installed");
    }

	if (link -> m_last_valid_time < 0){
    	link -> m_last_valid_time = get_last_valid_time(link -> m_N_in_truck, link -> m_N_out_truck, end_loading_timestamp);
    }

  	IAssert (link -> m_last_valid_time >= 0);

	TFlt _total_density = 0.0;
	TFlt _time_tmp;

	for (int i = 0; i < num_trials; ++i){
		_time_tmp = TFlt(i - 2 + time) > TFlt(0) ? TFlt(i - 2 + time) : TFlt(0);
		_total_density += get_link_density_from_cc(_time_tmp, link -> m_N_in_truck, link -> m_N_out_truck, link -> m_last_valid_time);
	}

	return _total_density/num_trials;
}

TFlt get_link_density_rh_robust(MNM_Dlink_Multiclass* link, TFlt time, TInt end_loading_timestamp, TInt num_trials)
{
	if (link == nullptr){
		throw std::runtime_error ("Error, get_link_density_rh_robust link is null");
	}
  	if ((link -> m_N_in_rh == nullptr) || (link -> m_N_out_rh == nullptr)){
		throw std::runtime_error ("Error, get_link_density_rh_robust link cumulative curve is not installed");
	}

	if (link -> m_last_valid_time < 0){
		link -> m_last_valid_time = get_last_valid_time(link -> m_N_in_rh, link -> m_N_out_rh, end_loading_timestamp);
	}

  	IAssert (link -> m_last_valid_time >= 0);

	TFlt _total_density = 0.0;
	TFlt _time_tmp;

	for (int i = 0; i < num_trials; ++i){
		_time_tmp = TFlt(i - 2 + time) > TFlt(0) ? TFlt(i - 2 + time) : TFlt(0);
		_total_density += get_link_density_from_cc(_time_tmp, link -> m_N_in_rh, link -> m_N_out_rh, link -> m_last_valid_time);
	}

	return _total_density/num_trials;
}

TFlt get_link_density_stop_rh_robust(MNM_Dlink_Multiclass* link, TFlt time, TInt end_loading_timestamp, TInt num_trials)
{
	if (link == nullptr){
    	throw std::runtime_error ("Error, get_link_density_stop_rh_robust link is null");
    }
	
  	if (link -> m_N_in_rh_cc == nullptr){
    	throw std::runtime_error ("Error, get_link_density_stop_rh_robust link cumulative curve is not installed");
    }

	if (link -> m_last_valid_time < 0){
    	link -> m_last_valid_time = get_last_valid_time(link -> m_N_in_rh_cc, link -> m_N_out_rh_cc, end_loading_timestamp);
    }

  	IAssert (link -> m_last_valid_time >= 0);

	TFlt _total_density = 0.0;
	TFlt _time_tmp;

	for (int i = 0; i < num_trials; ++i){
		_time_tmp = TFlt(i - 2 + time) > TFlt(0) ? TFlt(i - 2 + time) : TFlt(0);
		_total_density += get_link_density_from_cc(_time_tmp, link -> m_N_in_rh_cc, link -> m_N_out_rh_cc, link -> m_last_valid_time);
	}

	return _total_density/num_trials;
}

TFlt get_link_density_stop_truck_robust(MNM_Dlink_Multiclass* link, TFlt time, TInt end_loading_timestamp, TInt num_trials)
{
	if (link == nullptr){
    	throw std::runtime_error ("Error, get_link_density_stop_truck_robust link is null");
    }

  	if (link -> m_N_in_truck_cc == nullptr){
    	throw std::runtime_error ("Error, get_link_density_stop_truck_robust link cumulative curve is not installed");
    }

	if (link -> m_last_valid_time < 0){
    	link -> m_last_valid_time = get_last_valid_time(link -> m_N_in_truck_cc, link -> m_N_out_truck_cc, end_loading_timestamp);
    }

  	IAssert (link -> m_last_valid_time >= 0);

	TFlt _total_density = 0.0;
	TFlt _time_tmp;

	for (int i = 0; i < num_trials; ++i){
		_time_tmp = TFlt(i - 2 + time) > TFlt(0) ? TFlt(i - 2 + time) : TFlt(0);
		_total_density += get_link_density_from_cc(_time_tmp, link -> m_N_in_truck_cc, link -> m_N_out_truck_cc, link -> m_last_valid_time);
	}

	return _total_density/num_trials;
}

}//end namespace MNM_DTA_GRADIENT


/******************************************************************************************************************
*******************************************************************************************************************
											Multiclass Emissions
*******************************************************************************************************************
******************************************************************************************************************/
MNM_Cumulative_Emission_Multiclass::MNM_Cumulative_Emission_Multiclass(TFlt unit_time, TInt freq)
	: MNM_Cumulative_Emission::MNM_Cumulative_Emission(unit_time, freq)
{
	m_fuel_truck = TFlt(0);
	m_CO2_truck = TFlt(0);
	m_HC_truck = TFlt(0);
	m_CO_truck = TFlt(0);
	m_NOX_truck = TFlt(0);
	m_VMT_truck = TFlt(0);

	m_VHT_car = TFlt(0);
	m_VHT_truck = TFlt(0);

	m_car_set = std::unordered_set<MNM_Veh*>();
	m_truck_set = std::unordered_set<MNM_Veh*>();
}

MNM_Cumulative_Emission_Multiclass::~MNM_Cumulative_Emission_Multiclass()
{
	;
}

// All convert_factors from MOVES
// Reference: MOVES default database - class 2b trucks with 4 tires
TFlt MNM_Cumulative_Emission_Multiclass::calculate_fuel_rate_truck(TFlt v)
{
	TFlt _convert_factor = 1.0;
	if (v < 25){
		_convert_factor = 1.53;
	}
	else if (v < 55){
		_convert_factor = 1.50;
	}
	else {
		_convert_factor = 1.55;
	}
	TFlt _fuel_rate_car = calculate_fuel_rate(v);
	TFlt _fuel_rate_truck = _fuel_rate_car * _convert_factor;
	return _fuel_rate_truck;
}

TFlt MNM_Cumulative_Emission_Multiclass::calculate_CO2_rate_truck(TFlt v)
{
	TFlt _convert_factor = 1.0;
	if (v < 25){
		_convert_factor = 1.53;
	}
	else if (v < 55){
		_convert_factor = 1.50;
	}
	else {
		_convert_factor = 1.55;
	}
	TFlt _CO2_rate_car = calculate_CO2_rate(v);
	TFlt _CO2_rate_truck = _CO2_rate_car * _convert_factor;
	return _CO2_rate_truck;
}

TFlt MNM_Cumulative_Emission_Multiclass::calculate_HC_rate_truck(TFlt v)
{
	TFlt _convert_factor = 1.0;
	if (v < 25){
		_convert_factor = 1.87;
	}
	else if (v < 55){
		_convert_factor = 2.41;
	}
	else {
		_convert_factor = 2.01;
	}
	TFlt _HC_rate_car = calculate_HC_rate(v);
	TFlt _HC_rate_truck = _HC_rate_car * _convert_factor;
	return _HC_rate_truck;
}

TFlt MNM_Cumulative_Emission_Multiclass::calculate_CO_rate_truck(TFlt v)
{
	TFlt _convert_factor = 1.0;
	if (v < 25){
		_convert_factor = 3.97;
	}
	else if (v < 55){
		_convert_factor = 2.67;
	}
	else {
		_convert_factor = 5.01;
	}
	TFlt _CO_rate_car = calculate_CO_rate(v);
	TFlt _CO_rate_truck = _CO_rate_car * _convert_factor;
	return _CO_rate_truck;
}

TFlt MNM_Cumulative_Emission_Multiclass::calculate_NOX_rate_truck(TFlt v)
{
	TFlt _convert_factor = 1.0;
	if (v < 25){
		_convert_factor = 7.32;
	}
	else if (v < 55){
		_convert_factor = 6.03;
	}
	else {
		_convert_factor = 5.75;
	}
	TFlt _NOX_rate_car = calculate_NOX_rate(v);
	TFlt _NOX_rate_truck = _NOX_rate_car * _convert_factor;
	return _NOX_rate_truck;
}

int MNM_Cumulative_Emission_Multiclass::update(MNM_Veh_Factory* veh_factory)
{
	// assume car truck the same speed on the same link when computing the emissions
	// possible to change to more accurate speeds for cars and trucks
	TFlt _v;
	TFlt _v_converted;
	for (MNM_Dlink *link : m_link_vector){
		MNM_Dlink_Multiclass *_mlink = dynamic_cast<MNM_Dlink_Multiclass *>(link);
		IAssert(_mlink != nullptr);
		_v = _mlink -> m_length / _mlink -> get_link_tt(); // m/s
		_v_converted = _v * TFlt(3600) / TFlt(1600); // mile / hour
		_v_converted = MNM_Ults::max(_v_converted, TFlt(5));
		_v_converted = MNM_Ults::min(_v_converted, TFlt(65));

		// cars
		m_fuel += calculate_fuel_rate(_v_converted) * (_v * m_unit_time / TFlt(1600)) * _mlink -> get_link_flow_car();
		m_CO2 += calculate_CO2_rate(_v_converted) * (_v * m_unit_time / TFlt(1600)) * _mlink -> get_link_flow_car();
		m_HC += calculate_HC_rate(_v_converted) * (_v * m_unit_time / TFlt(1600)) * _mlink -> get_link_flow_car();
		m_CO += calculate_CO_rate(_v_converted) * (_v * m_unit_time / TFlt(1600)) * _mlink -> get_link_flow_car();
		m_NOX += calculate_NOX_rate(_v_converted) * (_v * m_unit_time / TFlt(1600)) * _mlink -> get_link_flow_car();
		m_VMT += (_v * m_unit_time / TFlt(1600)) * _mlink -> get_link_flow_car();

		// trucks
		m_fuel_truck += calculate_fuel_rate_truck(_v_converted) * (_v * m_unit_time / TFlt(1600)) * _mlink -> get_link_flow_truck();
		m_CO2_truck += calculate_CO2_rate_truck(_v_converted) * (_v * m_unit_time / TFlt(1600)) * _mlink -> get_link_flow_truck();
		m_HC_truck += calculate_HC_rate_truck(_v_converted) * (_v * m_unit_time / TFlt(1600)) * _mlink -> get_link_flow_truck();
		m_CO_truck += calculate_CO_rate_truck(_v_converted) * (_v * m_unit_time / TFlt(1600)) * _mlink -> get_link_flow_truck();
		m_NOX_truck += calculate_NOX_rate_truck(_v_converted) * (_v * m_unit_time / TFlt(1600)) * _mlink -> get_link_flow_truck();
		m_VMT_truck += (_v * m_unit_time / TFlt(1600)) * _mlink -> get_link_flow_truck();

		// VHT (hours)
		m_VHT_car += m_unit_time * _mlink -> get_link_flow_car() / 3600;
		m_VHT_truck += m_unit_time * _mlink -> get_link_flow_truck() / 3600;

	}
	// trips
	MNM_Veh * _veh;
	MNM_Veh_Multiclass * _veh_multiclass;
	for (auto pair_it : veh_factory -> m_veh_map){
		_veh =  pair_it.second;
		_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass *>(_veh);
		if (m_link_set.find(_veh_multiclass -> m_current_link) != m_link_set.end()){
			if (_veh_multiclass -> m_class == 0){
				m_car_set.insert(_veh_multiclass);
			}
			if (_veh_multiclass -> m_class == 1){
				m_truck_set.insert(_veh_multiclass);
			}		
		}
	}
	
	return 0;
}

int MNM_Cumulative_Emission_Multiclass::output()
{
	// std::string _str;

	// _str = "\nThe emission stats for cars are: ";
	// _str += "fuel: " + std::to_string(m_fuel()) + " gallons, CO2: " + std::to_string(m_CO2()) + " g, HC: " + std::to_string(m_HC()) + " g, CO: " + std::to_string(m_CO()) + " g, NOX: " + std::to_string(m_NOX()) + " g, VMT: " + std::to_string(m_VMT()) + " miles, VHT: " + std::to_string(m_VHT_car) + " hours, " + std::to_string(int(m_car_set.size())) + "trips\n";
	// _str += "fuel: " + std::to_string(m_fuel_truck()) + " gallons, CO2: " + std::to_string(m_CO2_truck()) + " g, HC: " + std::to_string(m_HC_truck()) + " g, CO: " + std::to_string(m_CO_truck()) + " g, NOX: " + std::to_string(m_NOX_truck()) + " g, VMT: " + std::to_string(m_VMT_truck()) + " miles, VHT: " + std::to_string(m_VHT_truck) + " hours, " + std::to_string(int(m_truck_set.size())) + "trips\n";
	
	printf("The emission stats for cars are: ");
	printf("fuel: %lf gallons, CO2: %lf g, HC: %lf g, CO: %lf g, NOX: %lf g, VMT: %lf miles, VHT: %lf hours, %d trips\n", 
		   m_fuel(), m_CO2(), m_HC(), m_CO(), m_NOX(), m_VMT(), m_VHT_car(), int(m_car_set.size()));

	printf("The emission stats for trucks are: ");
	printf("fuel: %lf gallons, CO2: %lf g, HC: %lf g, CO: %lf g, NOX: %lf g, VMT: %lf miles, VHT: %lf hours, %d trips\n", 
		   m_fuel_truck(), m_CO2_truck(), m_HC_truck(), m_CO_truck(), m_NOX_truck(), m_VMT_truck(), m_VHT_truck(), int(m_truck_set.size()));
	
	return 0;
}

std::string MNM_Cumulative_Emission_Multiclass::output_save()
{
	std::string _str;

	_str = "class fuel(gallons) CO2(g) HC(g) CO(g) NOX(g) VMT(miles) VHT(hours) trips\n";
	_str += "car " + std::to_string(m_fuel()) + " " + std::to_string(m_CO2()) + " " + std::to_string(m_HC()) + " " + std::to_string(m_CO()) + " " + std::to_string(m_NOX()) + " " + std::to_string(m_VMT()) + " " + std::to_string(m_VHT_car()) + " " + std::to_string(int(m_car_set.size())) + "\n";
	_str += "truck " + std::to_string(m_fuel_truck()) + " " + std::to_string(m_CO2_truck()) + " " + std::to_string(m_HC_truck()) + " " + std::to_string(m_CO_truck()) + " " + std::to_string(m_NOX_truck()) + " " + std::to_string(m_VMT_truck()) + " " + std::to_string(m_VHT_truck()) + " " + std::to_string(int(m_truck_set.size())) + "\n";
	
	printf("The emission stats for cars are: ");
	printf("fuel: %lf gallons, CO2: %lf g, HC: %lf g, CO: %lf g, NOX: %lf g, VMT: %lf miles, VHT: %lf hours, %d trips\n", 
		   m_fuel(), m_CO2(), m_HC(), m_CO(), m_NOX(), m_VMT(), m_VHT_car(), int(m_car_set.size()));

	printf("The emission stats for trucks are: ");
	printf("fuel: %lf gallons, CO2: %lf g, HC: %lf g, CO: %lf g, NOX: %lf g, VMT: %lf miles, VHT: %lf hours, %d trips\n", 
		   m_fuel_truck(), m_CO2_truck(), m_HC_truck(), m_CO_truck(), m_NOX_truck(), m_VMT_truck(), m_VHT_truck(), int(m_truck_set.size()));
	
	return _str;
}

// /******************************************************************************************************************
// *******************************************************************************************************************
// 										Multiclass DUE with Curb
// *******************************************************************************************************************
// ******************************************************************************************************************/
// MNM_Due_Curb::MNM_Due_Curb(std::string file_folder){
// 	m_file_folder = file_folder;
//     m_dta_config = new MNM_ConfReader(m_file_folder + "/config.conf", "DTA");
// 	// IAssert(m_dta_config->get_string("routing_type") == "Due");
//     // IAssert(m_dta_config->get_int("total_interval") > 0);
//     // IAssert(m_dta_config->get_int("total_interval") >=
//     //         m_dta_config->get_int("assign_frq") * m_dta_config->get_int("max_interval"));
    
// 	m_unit_time = m_dta_config->get_float("unit_time");
//     m_total_loading_inter = m_dta_config->get_int("total_interval");

// 	m_assign_freq_due = m_dta_config->get_int("assign_frq");
// 	m_total_assign_inter = m_dta_config->get_int("max_interval");

// 	m_due_config = new MNM_ConfReader(m_file_folder + "/config.conf", "DUE");

// 	m_unit_time_cost_car = m_due_config -> get_float("unit_time_cost_car");
// 	m_unit_time_cost_truck = m_due_config -> get_float("unit_time_cost_truck");
// 	m_unit_dist_cost_truck = m_due_config -> get_float("unit_dist_cost_truck");

// 	m_unit_time_cost_rh = m_due_config -> get_float("unit_time_cost_rh");
// 	m_unit_dist_cost_rh = m_due_config -> get_float("unit_dist_cost_rh");
// 	m_fixed_fee_rh = m_due_config -> get_float("fixed_fee_rh");
// 	m_general_cost_car = m_due_config -> get_float("general_cost_car");
// 	m_parking_fee_car = m_due_config -> get_float("parking_fee_car");
// 	m_step_size = m_due_config -> get_float("step_size");
// 	m_max_iter = m_due_config -> get_int("max_iter");
	
//     m_path_table_car = nullptr;
// 	m_path_table_truck = nullptr;
// 	m_path_table_rh = nullptr;

// 	m_car_cost_map = std::unordered_map<TInt, TFlt *>(); 
// 	m_truck_cost_map = std::unordered_map<TInt, TFlt *>(); 
// }

// MNM_Due_Curb::~MNM_Due_Curb() {
//     if (m_dta_config != nullptr) delete m_dta_config;
//     if (m_due_config != nullptr) delete m_due_config;
//     // if (m_od_factory != nullptr) delete m_od_factory;
// }

// MNM_Dta_Multiclass *MNM_Due_Curb::run_dta_curb(bool verbose)
// {
// 	MNM_Dta_Multiclass_Curb *test_dta = new MNM_Dta_Multiclass_Curb(m_file_folder);
//     if (verbose){
// 		printf("================================ DTA set! =================================\n");
// 	}

//     test_dta -> build_from_files_separate();
	
// 	// test_dta -> m_routing -> init_routing_curb

// 	MNM_Routing_Biclass_Hybrid_Curb *_routing_biclass;

// 	_routing_biclass = dynamic_cast<MNM_Routing_Biclass_Hybrid_Curb *>(test_dta -> m_routing);

// 	_routing_biclass -> init_routing_curb(m_path_table_car, m_path_table_truck, m_path_table_rh);

// 	if (verbose){
//     	printf("========================= Finished initialization! ========================\n");
// 	}

//     test_dta -> hook_up_node_and_link();

//     if (verbose){
// 		printf("====================== Finished node and link hook-up! ====================\n");
// 	}

//     test_dta -> is_ok();

// 	if (verbose){
//     	printf("============================ DTA is OK to run! ============================\n");
// 	}

// 	MNM_Dlink *_link;

// 	MNM_Dlink_Multiclass *_link_multiclass;

// 	for (auto _link_it = test_dta->m_link_factory->m_link_map.begin();
//         _link_it != test_dta->m_link_factory->m_link_map.end(); _link_it++) {

// 		_link = _link_it -> second;

// 		_link_multiclass = dynamic_cast<MNM_Dlink_Multiclass *>(_link);

// 		_link_multiclass -> install_cumulative_curve_multiclass();
//     }

// 	if (verbose){
// 		printf("=========================== Finished install cc ===========================\n");
// 	}

//     test_dta -> pre_loading();

// 	if (verbose){
//  	   printf("========================== Finished pre_loading! ==========================\n");
// 	}

// 	TInt _current_inter = 0;
// 	TInt _assign_inter = test_dta -> m_start_assign_interval;

//     while (!test_dta -> finished_loading(_current_inter) || _assign_inter < test_dta -> m_total_assign_inter){

// 		if (verbose){
// 			printf("\nCurrent loading interval: %d, Current assignment interval: %d\n", _current_inter(), _assign_inter());
// 		}
		
//         test_dta -> load_once_curb(verbose, _current_inter, _assign_inter);

//         if (_current_inter % test_dta -> m_assign_freq == 0 || _current_inter == 0){
// 			_assign_inter += 1;
// 		}
//         _current_inter += 1;
//     }

// 	return test_dta;
// }

// int MNM_Due_Curb::initialize()
// {
// 	m_base_dta = new MNM_Dta_Multiclass_Curb(m_file_folder);
//     m_base_dta -> build_from_files_separate();
//     // m_path_table_car = MNM::build_shortest_pathset(m_base_dta->m_graph,
//     //                                            m_base_dta->m_od_factory, m_base_dta->m_link_factory);
//     // MNM::allocate_path_table_buffer(m_path_table_car, m_total_assign_inter);  // create zero buffer
//     // MNM::save_path_table(m_path_table, m_base_dta -> m_od_factory, true);

//     for (auto _link_it : m_base_dta->m_link_factory->m_link_map) {
//         m_car_cost_map[_link_it.first] = new TFlt[m_total_loading_inter];
// 		m_truck_cost_map[_link_it.first] = new TFlt[m_total_loading_inter];
//     }

// 	m_path_inter_dest_map = m_base_dta -> m_curb_factory -> m_path_inter_dest_map;
// 	m_path_inter_dest_map_rh = m_base_dta -> m_curb_factory -> m_path_inter_dest_map_rh;
// 	m_path_inter_dest_map_car = m_base_dta -> m_curb_factory -> m_path_inter_dest_map_car;

// 	/* find file */
// 	std::string _demand_file_name = m_file_folder + "/MNM_input_demand";
// 	std::ifstream _demand_file;
// 	_demand_file.open(_demand_file_name, std::ios::in);

// 	std::string _line;
// 	std::vector<std::string> _words;

// 	if (_demand_file.is_open())
// 	{
// 		TFlt _demand_driving;
// 		TFlt _demand_tnc;
// 		TFlt _demand_truck;

// 		std::getline(_demand_file,_line); //skip the first line
// 		std::getline(_demand_file,_line);
// 		_words = MNM_IO::split(_line, ' ');

// 		IAssert(TInt(_words.size()) == (10 * 3 + 2));

// 		for (int j = 0; j < 10; ++j) {
// 			// [Driving_demand, Truck_demand, RH_demand]
// 			_demand_driving = TFlt(std::stod(_words[j + 2]));  
// 			_demand_truck = TFlt(std::stod(_words[j + 10 + 2]));
// 			_demand_tnc = TFlt(std::stod(_words[j + 10 * 2 + 2]));

// 			m_demand_car.push_back(_demand_driving);
// 			m_demand_truck.push_back(_demand_truck);
// 			m_demand_rh.push_back(_demand_tnc);
// 		}

// 	}
// 	printf("finish initialization\n");
// 	return 0;
// }

// TFlt MNM_Due_Curb::compute_merit_function(MNM_Dta_Multiclass *mc_dta)
// {
// 	return 0.0;
// }

// TFlt MNM_Due_Curb::compute_merit_function_fixed_departure_time_choice(MNM_Dta_Multiclass *mc_dta)
// {
// 	TFlt _tt, _depart_time, _dis_utl, _lowest_dis_utl;
// 	TFlt _total_gap_car = 0.0;
// 	TFlt _total_gap_truck = 0.0;
// 	TFlt _total_gap_rh = 0.0;
// 	TFlt _min_flow_cost_car = 0.0;
// 	TFlt _min_flow_cost_truck = 0.0;
// 	TFlt _min_flow_cost_rh = 0.0;

// 	// car
// 	for (auto _it : *m_path_table_car){
// 		for (auto _it_it : *(_it.second)){
// 			for (int _col = 0; _col < m_total_assign_inter; _col++){
// 				_lowest_dis_utl = DBL_MAX;
// 				_depart_time = TFlt(_col * m_dta_config -> get_int("assign_frq"));
// 				for (MNM_Path *_path : _it_it.second -> m_path_vec){
// 					if (_path -> m_buffer[_col] >= 0){
// 						_tt = get_tt_car(_depart_time, _path);
// 						_dis_utl = get_disutility_car(_col, _tt, _path);
// 						if (_dis_utl < _lowest_dis_utl) _lowest_dis_utl = _dis_utl;
// 					}
// 					// printf("----------  car path %d cost = %lf  ----------\n",int(_path -> m_path_ID), float(_dis_utl));
// 				}

// 				for (MNM_Path *_path : _it_it.second -> m_path_vec){
// 					if (_path -> m_buffer[_col] >= 0){
// 						_tt = get_tt_car(_depart_time, _path);
// 						_dis_utl = get_disutility_car(_col, _tt, _path);
// 						_total_gap_car += (_dis_utl - _lowest_dis_utl) * _path -> m_buffer[_col] * m_demand_car[_col] * m_base_dta -> m_flow_scalar;
// 						_min_flow_cost_car += _lowest_dis_utl * _path -> m_buffer[_col] * m_demand_car[_col] * m_base_dta -> m_flow_scalar;
// 					}
// 				}
// 			}
// 		}
// 	}
	
// 	// truck
// 	for (auto _it : *m_path_table_truck){
// 		for (auto _it_it : *(_it.second)){
// 			for (int _col = 0; _col < m_total_assign_inter; _col++){
// 				_lowest_dis_utl = DBL_MAX;
// 				_depart_time = TFlt(_col * m_dta_config -> get_int("assign_frq"));
// 				for (MNM_Path *_path : _it_it.second -> m_path_vec){
// 					if (_path -> m_buffer[_col] >= 0){
// 						_tt = get_tt_truck(_depart_time, _path);
// 						_dis_utl = get_disutility_truck(_col, _tt, _path);
// 						if (_dis_utl < _lowest_dis_utl) _lowest_dis_utl = _dis_utl;
// 					}
// 					// printf("---------- truck path %d cost = %lf  ----------\n",int(_path -> m_path_ID), float(_dis_utl));
// 				}

// 				for (MNM_Path *_path : _it_it.second -> m_path_vec){
// 					if (_path -> m_buffer[_col] >= 0){
// 						_tt = get_tt_truck(_depart_time, _path);
// 						_dis_utl = get_disutility_truck(_col, _tt, _path);
// 						_total_gap_truck += (_dis_utl - _lowest_dis_utl) * _path -> m_buffer[_col] * m_demand_truck[_col] * m_base_dta -> m_flow_scalar;
// 						_min_flow_cost_truck += _lowest_dis_utl * _path -> m_buffer[_col] * m_demand_truck[_col] * m_base_dta -> m_flow_scalar;
// 					}
// 				}
// 			}
// 		}
// 	}

// 	// rh
// 	for (auto _it : *m_path_table_rh){
// 		for (auto _it_it : *(_it.second)){
// 			for (int _col = 0; _col < m_total_assign_inter; _col++){
// 				_lowest_dis_utl = DBL_MAX;
// 				_depart_time = TFlt(_col * m_dta_config -> get_int("assign_frq"));
// 				for (MNM_Path *_path : _it_it.second -> m_path_vec){
// 					if (_path -> m_buffer[_col] >= 0){
// 						_tt = get_tt_rh(_depart_time, _path);
// 						_dis_utl = get_disutility_rh(_col, _tt, _path);
// 						if (_dis_utl < _lowest_dis_utl) _lowest_dis_utl = _dis_utl;
// 					}
// 					// printf("-------- RH path %d cost = %lf  ; buffer = %lf--------\n",int(_path -> m_path_ID), float(_dis_utl), float(_path->m_buffer[_col]));
// 				}

// 				for (MNM_Path *_path : _it_it.second -> m_path_vec){
// 					if (_path -> m_buffer[_col] >= 0){
// 						_tt = get_tt_rh(_depart_time, _path);
// 						_dis_utl = get_disutility_rh(_col, _tt, _path);
// 						_total_gap_rh += (_dis_utl - _lowest_dis_utl) * _path -> m_buffer[_col] * m_demand_rh[_col] * m_base_dta -> m_flow_scalar;
// 						_min_flow_cost_rh += _lowest_dis_utl * _path -> m_buffer[_col] * m_demand_rh[_col] * m_base_dta -> m_flow_scalar;
// 					}
// 				}
// 			}
// 		}
// 	}

// 	// return (_total_gap_car + _total_gap_truck + _total_gap_rh) / (_min_flow_cost_car + _min_flow_cost_rh + _min_flow_cost_truck);
// 	return _total_gap_car/_min_flow_cost_car + _total_gap_truck/_min_flow_cost_rh + _total_gap_rh/_min_flow_cost_truck;
// 	// return _total_gap_car + _total_gap_truck + _total_gap_rh;
// }

// int MNM_Due_Curb::update_path_table(MNM_Dta_Multiclass *mc_dta, int iter)
// {
// 	return 0;
// }

// int MNM_Due_Curb::update_path_table_gp_fixed_departure_time_choice_fixed_pathset(MNM_Dta_Multiclass *mc_dta)
// {
// 	MNM_Origin *_orig;
// 	MNM_Origin_Multiclass *_orig_multiclass;

//     MNM_Destination *_dest;
// 	MNM_Destination_Multiclass *_dest_multiclass;

//     TInt _orig_node_ID, _dest_node_ID, _tot_nonzero_path;
//     std::pair<MNM_Path *, TInt> _path_result_car, _path_result_truck, _path_result_rh;
//     // MNM_Path *_path_car, *_path_truck, *_path_rh;
//     MNM_Pathset *_path_set_car, *_path_set_rh, *_path_set_truck;
//     TFlt _tot_path_cost, _tmp_change, _tau, _tmp_tt, _tmp_cost, _min_flow;

//     // build_cost_map(mc_dta);
// 	// check path table
	
// 	for (auto _it : mc_dta -> m_od_factory -> m_destination_map){
// 		_dest = _it.second;
// 		_dest_multiclass = dynamic_cast<MNM_Destination_Multiclass *>(_dest);
// 		_dest_node_ID = _dest -> m_dest_node -> m_node_ID;

// 		// MNM_TDSP_Tree *_tdsp_tree_car = new MNM_TDSP_Tree(_dest_node_ID, mc_dta->m_graph, m_total_loading_inter);

//         // _tdsp_tree_car -> initialize();
//         // _tdsp_tree_car -> update_tree(m_car_cost_map);

// 		// MNM_TDSP_Tree *_tdsp_tree_truck = new MNM_TDSP_Tree(_dest_node_ID, mc_dta->m_graph, m_total_loading_inter);

//         // _tdsp_tree_truck -> initialize();
//         // _tdsp_tree_truck -> update_tree(m_truck_cost_map);

// 		for (auto _map_it : mc_dta -> m_od_factory -> m_origin_map) {
// 			_orig = _map_it.second;
// 			_orig_multiclass = dynamic_cast<MNM_Origin_Multiclass *>(_orig);
// 			_orig_node_ID = _orig -> m_origin_node -> m_node_ID;

// 			if (_orig_multiclass -> m_demand_car.find(_dest_multiclass) == _orig_multiclass -> m_demand_car.end()){
// 				continue;
// 			}

// 			_path_set_car = MNM::get_pathset(m_path_table_car, _orig_node_ID, _dest_node_ID);

// 			_path_set_rh = MNM::get_pathset(m_path_table_rh, _orig_node_ID, _dest_node_ID);

// 			_path_set_truck = MNM::get_pathset(m_path_table_truck, _orig_node_ID, _dest_node_ID);

// 			// for car
// 			for (int _col = 0; _col < m_total_assign_inter; _col++) {
//                 _tot_path_cost = 0.0;
//                 _tot_nonzero_path = 0;
//                 _tau = TFlt(std::numeric_limits<double>::max());
//                 _min_flow = TFlt(std::numeric_limits<double>::max());
//                 _tmp_change = 0.0;
                
// 				// average path cost for car
// 				// loop for path table of car mode
//                 for (auto _tmp_path : _path_set_car->m_path_vec) {
// 					// if this path has flow
//                     if (_tmp_path->m_buffer[_col] > 0.001) {

// 						// get path travel time
//                         _tmp_tt = get_tt_car(_col * m_assign_freq_due, _tmp_path);
//                         // printf("path in pathset, tt %lf\n", (float)_tmp_tt);

// 						// get path utility function
//                         _tmp_cost = get_disutility_car(TInt(_col), _tmp_tt, _tmp_path);

// 						// update total sum
//                         _tot_path_cost += _tmp_cost;
//                         _tot_nonzero_path += 1;

// 						// update minimum path flow (buffer)
//                         if ((_tmp_path->m_buffer[_col] > 0.001) && (_min_flow > _tmp_path->m_buffer[_col])) {
//                             _min_flow = _tmp_path -> m_buffer[_col];
//                         }
//                     }
//                 }

// 				// minimum tau for car
//                 for (auto _tmp_path : _path_set_car -> m_path_vec) {
//                     if (_tmp_path->m_buffer[_col] > 0.001) {
//                         _tmp_tt = get_tt_car(_col * m_assign_freq_due, _tmp_path);
//                         _tmp_cost = get_disutility_car(TInt(_col), _tmp_tt, _tmp_path);
//                         _tmp_change = _tmp_cost - _tot_path_cost / _tot_nonzero_path;
//                         if ((_tmp_change > 0) && (_tau > m_step_size * _min_flow / _tmp_change)) {
//                             _tau = m_step_size * _min_flow / _tmp_change;
//                         }
//                     }
//                 }

// 				// flow adjustment for car
//                 for (auto _tmp_path : _path_set_car -> m_path_vec) {
//                     if (_tmp_path->m_buffer[_col] > 0.001) {
//                         _tmp_tt = get_tt_car(_col * m_assign_freq_due, _tmp_path);
//                         _tmp_cost = get_disutility_car(TInt(_col), _tmp_tt, _tmp_path);
//                         _tmp_change = _tmp_cost - _tot_path_cost / _tot_nonzero_path;
//                         _tmp_path -> m_buffer[_col] -= MNM_Ults::min(_tmp_path -> m_buffer[_col], _tau * _tmp_change);
//                     }
//                 }
// 			} // end for car

// 			// begin for truck
// 			for (int _col = 0; _col < m_total_assign_inter; _col++) {
//                 _tot_path_cost = 0.0;
//                 _tot_nonzero_path = 0;
//                 _tau = TFlt(std::numeric_limits<double>::max());
//                 _min_flow = TFlt(std::numeric_limits<double>::max());
//                 _tmp_change = 0.0;
                
// 				// average path cost for truck
// 				// loop for path table of truck mode
//                 for (auto _tmp_path : _path_set_truck->m_path_vec) {
// 					// if this path has flow
//                     if (_tmp_path->m_buffer[_col] > 0.001) {

// 						// get path travel time
//                         _tmp_tt = get_tt_truck(_col * m_assign_freq_due, _tmp_path);
//                         // printf("path in pathset, tt %lf\n", (float)_tmp_tt);

// 						// get path utility function
//                         _tmp_cost = get_disutility_truck(TInt(_col), _tmp_tt, _tmp_path);

// 						// update total sum
//                         _tot_path_cost += _tmp_cost;
//                         _tot_nonzero_path += 1;

// 						// update minimum path flow (buffer)
//                         if ((_tmp_path->m_buffer[_col] > 0.001) && (_min_flow > _tmp_path->m_buffer[_col])) {
//                             _min_flow = _tmp_path -> m_buffer[_col];
//                         }
//                     }
//                 }

// 				// minimum tau for truck
//                 for (auto _tmp_path : _path_set_truck -> m_path_vec) {
//                     if (_tmp_path->m_buffer[_col] > 0.001) {
//                         _tmp_tt = get_tt_truck(_col * m_assign_freq_due, _tmp_path);
//                         _tmp_cost = get_disutility_truck(TInt(_col), _tmp_tt, _tmp_path);
//                         _tmp_change = _tmp_cost - _tot_path_cost / _tot_nonzero_path;
//                         if ((_tmp_change > 0) && (_tau > m_step_size * _min_flow / _tmp_change)) {
//                             _tau = m_step_size * _min_flow / _tmp_change;
//                         }
//                     }
//                 }

// 				// flow adjustment for truck
//                 for (auto _tmp_path : _path_set_truck -> m_path_vec) {
//                     if (_tmp_path->m_buffer[_col] > 0.001) {
//                         _tmp_tt = get_tt_truck(_col * m_assign_freq_due, _tmp_path);
//                         _tmp_cost = get_disutility_truck(TInt(_col), _tmp_tt, _tmp_path);
//                         _tmp_change = _tmp_cost - _tot_path_cost / _tot_nonzero_path;
//                         _tmp_path -> m_buffer[_col] -= MNM_Ults::min(_tmp_path -> m_buffer[_col], _tau * _tmp_change);
//                     }
//                 }
// 			}// end for truck

// 			// begin for RH
// 			for (int _col = 0; _col < m_total_assign_inter; _col++) {
//                 _tot_path_cost = 0.0;
//                 _tot_nonzero_path = 0;
//                 _tau = TFlt(std::numeric_limits<double>::max());
//                 _min_flow = TFlt(std::numeric_limits<double>::max());
//                 _tmp_change = 0.0;
                
// 				// average path cost for car
// 				// loop for path table of car mode
//                 for (auto _tmp_path : _path_set_rh -> m_path_vec) {
// 					// if this path has flow
//                     if (_tmp_path -> m_buffer[_col] > 0.001) {

// 						// get path travel time
//                         _tmp_tt = get_tt_rh(_col * m_assign_freq_due, _tmp_path);
                        
// 						// get path utility function
//                         _tmp_cost = get_disutility_rh(TInt(_col), _tmp_tt, _tmp_path);

// 						// update total sum
//                         _tot_path_cost += _tmp_cost;
//                         _tot_nonzero_path += 1;

// 						// update minimum path flow (buffer)
//                         if ((_tmp_path -> m_buffer[_col] > 0.001) && (_min_flow > _tmp_path -> m_buffer[_col])) {
//                             _min_flow = _tmp_path -> m_buffer[_col];
//                         }
//                     }
//                 }

// 				// minimum tau for car
//                 for (auto _tmp_path : _path_set_rh -> m_path_vec) {
//                     if (_tmp_path -> m_buffer[_col] > 0.001) {

//                         _tmp_tt = get_tt_rh(_col * m_assign_freq_due, _tmp_path);

//                         _tmp_cost = get_disutility_rh(TInt(_col), _tmp_tt, _tmp_path);

//                         _tmp_change = _tmp_cost - _tot_path_cost / _tot_nonzero_path;

//                         if ((_tmp_change > 0) && (_tau > m_step_size * _min_flow / _tmp_change)) {
//                             _tau = m_step_size * _min_flow / _tmp_change;
//                         }
//                     }
//                 }

// 				// flow adjustment for car
//                 for (auto _tmp_path : _path_set_rh -> m_path_vec) {
//                     if (_tmp_path -> m_buffer[_col] > 0.001) {

//                         _tmp_tt = get_tt_rh(_col * m_assign_freq_due, _tmp_path);

//                         _tmp_cost = get_disutility_rh(TInt(_col), _tmp_tt, _tmp_path);

//                         _tmp_change = _tmp_cost - _tot_path_cost / _tot_nonzero_path;

//                         _tmp_path -> m_buffer[_col] -= MNM_Ults::min(_tmp_path -> m_buffer[_col], _tau * _tmp_change);
//                     }
// 					printf("-------- RH path %d ; buffer = %lf--------\n",int(_tmp_path -> m_path_ID), float(_tmp_path->m_buffer[_col]));
//                 }
// 			} // end for RH
// 		}

// 		// delete _tdsp_tree_car;
// 		// delete _tdsp_tree_truck;
// 	}
// 	// MNM::print_path_table(m_path_table, mc_dta -> m_od_factory, true);
//     // MNM::save_path_table(mc_dta -> m_file_folder + "/" + mc_dta -> m_statistics -> m_self_config -> get_string("rec_folder"), m_path_table, mc_dta -> m_od_factory, true);
// 	return 0;
// }

// int MNM_Due_Curb::update_path_table_gp_fixed_departure_time_choice_column_generation(MNM_Dta_Multiclass *mc_dta, int iter)
// {
// 	return 0;
// }

// int MNM_Due_Curb::update_demand_from_path_table(MNM_Dta_Multiclass *mc_dta)
// {
// 	// TFlt _tot_dmd;

// 	// MNM_Origin_Multiclass *_origin_multiclass;

// 	// MNM_Destination_Multiclass *_dest_multiclass;

// 	// MNM_Path *_one_path;

// 	// // for car demand, use m_path_table_car
// 	// for (auto _it : *m_path_table_car) { // origin loop
// 	// 	for (auto _it_it : *(_it.second)) { // destination loop
// 	// 		for (int _col = 0; _col < m_total_assign_inter; _col++){
// 	// 			_tot_dmd = 0.0;

// 	// 		}

// 	// 	}
// 	// }
// 	return 0;
// }

// int MNM_Due_Curb::build_cost_map(MNM_Dta_Multiclass *mc_dta)
// {
// 	MNM_Dlink *_link;
// 	MNM_Dlink_Multiclass *_link_multiclass;

//     for (int i = 0; i < m_total_loading_inter; i++) {
//         // std::cout << "********************** interval " << i << " **********************\n";
//         for (auto _link_it : mc_dta -> m_link_factory -> m_link_map) {
//             _link = _link_it.second;

// 			_link_multiclass = dynamic_cast<MNM_Dlink_Multiclass *>(_link);

//             m_car_cost_map[_link_it.first][i] = MNM_DTA_GRADIENT::get_travel_time_car(_link_multiclass, TFlt(i), m_unit_time, m_total_loading_inter);
// 			m_truck_cost_map[_link_it.first][i] = MNM_DTA_GRADIENT::get_travel_time_truck(_link_multiclass, TFlt(i), m_unit_time, m_total_loading_inter);
//             // std::cout << "interval: " << i << ", link: " << _link_it.first << ", tt: " << m_cost_map[_link_it.first][i] << "\n";
//         }
//     }
// 	return 0;
// }

// /* TODO include curb stopping time */
// /* total travel time: arrival time at destination - departing time at origin */
// TFlt MNM_Due_Curb::get_tt_car(TFlt depart_time, MNM_Path *path) {
//     // true path tt
//     TFlt _cur_time = depart_time;
//     TInt _query_time;
//     for (TInt _link_ID : path->m_link_vec) {
//         _query_time = MNM_Ults::min(TInt(_cur_time), TInt(m_total_loading_inter - 1));
//         _cur_time += m_car_cost_map[_link_ID][_query_time];
//     }
//     return _cur_time - depart_time;
// }

// TFlt MNM_Due_Curb::get_tt_truck(TFlt depart_time, MNM_Path *path) {
//     TFlt _cur_time = depart_time;
// 	TInt _path_ID = path -> m_path_ID;

// 	std::vector<TInt> _curb_list;
// 	std::vector<TInt> _inter_dest_info;
	
// 	auto _check = m_path_inter_dest_map.find(_path_ID);

// 	if (_check != m_path_inter_dest_map.end()){

// 		_inter_dest_info = m_path_inter_dest_map[_path_ID];

// 		TInt _num_stops = TInt(_inter_dest_info.size()/2);

// 		if (_num_stops > 0){
// 			for (int j = _num_stops; j < int(_inter_dest_info.size()); ++j){
// 				_curb_list.push_back(_inter_dest_info[j]);
// 			}
// 		}
// 	}

//     TInt _query_time;
//     for (TInt _link_ID : path->m_link_vec) {
//         _query_time = MNM_Ults::min(TInt(_cur_time), TInt(m_total_loading_inter - 1));
// 		/* leave time from this link (_link_ID) */
//         _cur_time += m_truck_cost_map[_link_ID][_query_time];

// 		auto it = std::find(_curb_list.begin(), _curb_list.end(), _link_ID);

// 		if (it != _curb_list.end()){
// 			_cur_time += TInt(120); /* add curb average stop time for RH = 10 mins */
// 		}
//     }

//     return _cur_time - depart_time;
// }

// TFlt MNM_Due_Curb::get_tt_rh(TFlt depart_time, MNM_Path *path){
	
// 	TFlt _cur_time = depart_time;
// 	TInt _path_ID = path -> m_path_ID;
	
// 	std::vector<TInt> _curb_list;
// 	std::vector<TInt> _inter_dest_info;
	
// 	auto _check = m_path_inter_dest_map.find(_path_ID);

// 	if (_check != m_path_inter_dest_map.end()){

// 		_inter_dest_info = m_path_inter_dest_map_rh[_path_ID];

// 		TInt _num_stops = TInt(_inter_dest_info.size()/2);

// 		if (_num_stops > 0){
// 			for (int j = _num_stops; j < int(_inter_dest_info.size()); ++j){
// 				_curb_list.push_back(_inter_dest_info[j]);
// 			}
// 		}
// 	}

//     TInt _query_time;
//     for (TInt _link_ID : path->m_link_vec) {
//         _query_time = MNM_Ults::min(TInt(_cur_time), TInt(m_total_loading_inter - 1));
// 		/* leave time from this link (_link_ID) */
//         _cur_time += m_car_cost_map[_link_ID][_query_time];

// 		/* _link_ID is in _curb_list */
// 		auto it = std::find(_curb_list.begin(), _curb_list.end(), _link_ID);

// 		if (it != _curb_list.end()){
// 			_cur_time += TInt(60); /* add curb average stop time for RH = 5 mins */
// 		}
//     }

//     return _cur_time - depart_time;
// }

// TFlt MNM_Due_Curb::get_disutility_car(TInt depart_int, TFlt tt, MNM_Path *path) {

// 	TFlt _disutility_car = 0.0;

// 	_disutility_car += tt * m_unit_time_cost_car;

// 	TInt _path_ID = path -> m_path_ID;
	
// 	std::vector<TInt> _curb_list;
// 	std::vector<TInt> _inter_dest_info;
// 	TInt _curb_ID;

// 	auto _check = m_path_inter_dest_map_car.find(_path_ID);

// 	if (_check != m_path_inter_dest_map_car.end()){

// 		_inter_dest_info = m_path_inter_dest_map_car[_path_ID];
// 		TInt _num_stops = TInt(_inter_dest_info.size()/2);
// 		IAssert(_num_stops == 1);
// 		_curb_ID = _inter_dest_info[1];
// 		TFlt _curb_price = m_base_dta -> m_curb_factory -> m_curb_price_list.find(_curb_ID) -> second[depart_int];
// 		_disutility_car += _curb_price;

// 	}
// 	else{
// 		_disutility_car += m_parking_fee_car;
// 	}

//     return _disutility_car;
// }

// TFlt MNM_Due_Curb::get_disutility_truck(TInt depart_int, TFlt tt, MNM_Path *path){
	
// 	TFlt _disutility_truck = 0.0;

// 	_disutility_truck += tt * m_unit_time_cost_truck;

// 	TInt _path_ID = path -> m_path_ID;
	
// 	std::vector<TInt> _curb_list;
// 	std::vector<TInt> _inter_dest_info;
	
// 	auto _check = m_path_inter_dest_map.find(_path_ID);

// 	if (_check != m_path_inter_dest_map.end()){

// 		_inter_dest_info = m_path_inter_dest_map[_path_ID];

// 		TInt _num_stops = TInt(_inter_dest_info.size()/2);

// 		if (_num_stops > 0){
// 			for (int j = _num_stops; j < int(_inter_dest_info.size()); ++j){
// 				_curb_list.push_back(_inter_dest_info[j]);
// 			}
// 		}
// 	}

// 	/* for loop: each link in the path */

// 	for (TInt _link_ID : path -> m_link_vec){
// 		_disutility_truck += m_unit_dist_cost_truck * m_base_dta -> m_link_factory -> m_link_map.find(_link_ID) -> second -> m_length;
		
// 		/* _link_ID is in _curb_list */
// 		auto it = std::find(_curb_list.begin(), _curb_list.end(), _link_ID);
// 		if (it != _curb_list.end()){
// 			TFlt _curb_price = m_base_dta -> m_curb_factory -> m_curb_price_list.find(_link_ID) -> second[depart_int];
// 			_disutility_truck += _curb_price; /* add curb price for this link */
// 		}
// 	}

// 	return _disutility_truck;
// }

// TFlt MNM_Due_Curb::get_disutility_rh(TInt depart_int, TFlt tt, MNM_Path *path)
// {
// 	TFlt _path_length = 0.0; 
// 	TFlt _disutility_rh = 0.0;

// 	/* extract current path: curb usage list */
// 	std::vector<TInt> _curb_list;
// 	std::vector<TInt> _inter_dest_info;
	
// 	auto _check = m_path_inter_dest_map_rh.find(TInt(path -> m_path_ID));

// 	if (_check != m_path_inter_dest_map_rh.end()){

// 		_inter_dest_info = m_path_inter_dest_map_rh[TInt(path -> m_path_ID)];

// 		TInt _num_stops = TInt(_inter_dest_info.size()/2);

// 		if (_num_stops > 0){
// 			for (int j = _num_stops; j < int(_inter_dest_info.size()); ++j){
// 				_curb_list.push_back(_inter_dest_info[j]);
// 			}
// 		}
// 	}

// 	/* for loop: each link in the path */
// 	MNM_Dlink *_link;
// 	for (TInt _link_ID : path -> m_link_vec)
// 	{
// 		_link = m_base_dta -> m_link_factory -> m_link_map[_link_ID];

// 		_path_length += TFlt(_link -> m_length);

// 		/* _link_ID is in _curb_list, add curb price */
// 		auto it = std::find(_curb_list.begin(), _curb_list.end(), _link_ID);

// 		if (it != _curb_list.end()){
// 			TFlt _curb_price = m_base_dta -> m_curb_factory -> m_curb_price_list.find(_link_ID) -> second[depart_int];
// 			_disutility_rh += _curb_price; /* add curb price for this link */
// 		}
// 	}

// 	_disutility_rh += (m_fixed_fee_rh + tt * m_unit_time_cost_rh + _path_length * m_unit_dist_cost_rh);

// 	return _disutility_rh;
// }

// int MNM_Due_Curb::load_fixed_pathset(MNM_Dta_Multiclass *mc_dta)
// {
// 	MNM_ConfReader* _tmp_conf = new MNM_ConfReader(m_file_folder + "/config.conf", "FIXED");
// 	Path_Table *_path_table_car;
// 	Path_Table *_path_table_truck;
// 	Path_Table *_path_table_rh;

// 	// printf("loading driving path table\n");
// 	_path_table_car = mc_dta -> load_path_table_curb(m_file_folder + "/" + "path_table", 
// 							mc_dta -> m_graph, _tmp_conf -> get_int("num_path"), true, TInt(0));

// 	// printf("loading truck path table\n");
// 	_path_table_truck = mc_dta -> load_path_table_curb(m_file_folder + "/" + "path_table_curb", 
// 							mc_dta -> m_graph, _tmp_conf -> get_int("num_path_curb"), true, TInt(1));

// 	// printf("loading ride-hailing path table\n");
// 	_path_table_rh = mc_dta -> load_path_table_curb(m_file_folder + "/" + "path_table_curb_rh", 
// 							mc_dta -> m_graph, _tmp_conf -> get_int("num_path_curb_rh"), true, TInt(2));

// 	delete _tmp_conf;

// 	m_path_table_car = _path_table_car;
// 	m_path_table_truck = _path_table_truck;
// 	m_path_table_rh = _path_table_rh;

// 	return 0;
// }

// int MNM_Due_Curb::set_routing_from_path_table(MNM_Dta_Multiclass *mc_dta)
// {
// 	// mc_dta -> m_routing->init_routing_curb();
// 	mc_dta -> m_routing -> init_routing_curb(m_path_table_car, m_path_table_truck, m_path_table_rh);
// 	return 0;
// }


// int MNM_Due_Curb::run_due_curb(bool verbose)
// {
	
// 	// initialize();
// 	// load_fixed_pathset();
// 	return 0;
// }

/******************************************************************************************************************
*******************************************************************************************************************
									Path Marginal Cost (PMC) Project 
	1. Network Input: input_files_nie

	2. Main functions:
		
		2.1 initialize: build dta and dso config

			* Ignore all configurations for curb and control

		2.2 dnl_once: load one interval and record link congestion state

			Run DNL once

			link loop:
			
				record space split ratio regimes: 1 - free flow; 2 - semi-congestion; 3 - fully-congestion
				
				recorded in vectors of "m_link_congested_car" and "m_link_congested_truck"

				if semi: (non-differetiable issue only happens in semi)

					check perceived density ?= critical density (within a threshold):

						true - m_link_nondiff_car/truck = true
					
						false - m_link_nondiff_car/truck = false

		2.3 build_link_cost_map: get link tt for cars and trucks

		2.4 get_link_marginal_cost: compute link marginal cost for each link, including upper and lower bound

*******************************************************************************************************************
******************************************************************************************************************/

MNM_Dso_Multiclass::MNM_Dso_Multiclass(std::string file_folder){
	m_file_folder = file_folder;
    m_dta_config = new MNM_ConfReader(m_file_folder + "/config.conf", "DTA");
    
	m_unit_time = m_dta_config->get_float("unit_time");
    m_total_loading_inter = m_dta_config->get_int("total_interval");

	m_assign_freq = m_dta_config->get_int("assign_frq");
	m_total_assign_inter = m_dta_config->get_int("max_interval");

	m_dso_config = new MNM_ConfReader(m_file_folder + "/config.conf", "DSO");

	m_step_size = m_dso_config -> get_float("step_size");
	m_max_iter = m_dso_config -> get_int("max_iter");
	
    m_path_table_car = nullptr;
	m_path_table_truck = nullptr;

	m_gap_function_value = 0.0;
	m_total_sdc_car = 0.0;
	m_total_sdc_truck = 0.0;
	m_total_ttc_car = 0.0;
	m_total_ttc_truck = 0.0;

	// initialize maps:
	m_link_congested_car = std::unordered_map<TInt, int *>(); 
	m_link_congested_truck = std::unordered_map<TInt, int *>(); 

	m_link_congested_car_tt = std::unordered_map<TInt, bool *>(); 
	m_link_congested_truck_tt = std::unordered_map<TInt, bool *>(); 

	m_link_diff_car = std::unordered_map<TInt, bool *>(); 
	m_link_diff_truck = std::unordered_map<TInt, bool *>(); 

	m_link_space_fraction_map_car = std::unordered_map<TInt, TFlt *>();
	m_link_space_fraction_map_truck = std::unordered_map<TInt, TFlt *>();

	m_queue_dissipated_time_car = std::unordered_map<TInt, int *>(); 
	m_queue_dissipated_time_truck = std::unordered_map<TInt, int *>(); 

	m_link_tt_map_car = std::unordered_map<TInt, TFlt *>();
	m_link_tt_map_truck = std::unordered_map<TInt, TFlt *>();

	m_lmc_car_lower = std::unordered_map<TInt, TFlt *>();
	m_lmc_car_upper = std::unordered_map<TInt, TFlt *>();

	m_lmc_truck_lower = std::unordered_map<TInt, TFlt *>();
	m_lmc_truck_upper = std::unordered_map<TInt, TFlt *>();

	m_lmc_car_tt = std::unordered_map<TInt, TFlt *>();
	m_lmc_truck_tt = std::unordered_map<TInt, TFlt *>();

}

MNM_Dso_Multiclass::~MNM_Dso_Multiclass() {
    if (m_dta_config != nullptr) delete m_dta_config;
    if (m_dso_config != nullptr) delete m_dso_config;
	if (m_base_dta != nullptr) delete m_base_dta;
    
	for (auto _it: m_link_congested_car) {
        delete _it.second;
    }
    m_link_congested_car.clear();

	for (auto _it: m_link_congested_truck) {
        delete _it.second;
    }
    m_link_congested_truck.clear();

	for (auto _it: m_link_congested_car_tt){
		delete _it.second;
	}
	m_link_congested_car_tt.clear();

	for (auto _it: m_link_congested_truck_tt) {
		delete _it.second;
	}
	m_link_congested_truck_tt.clear();

	for (auto _it: m_link_diff_car) {
        delete _it.second;
    }
    m_link_diff_car.clear();

	for (auto _it: m_link_diff_truck) {
        delete _it.second;
    }
    m_link_diff_truck.clear();

	for (auto _it: m_link_space_fraction_map_car) {
		delete _it.second;
	}
	m_link_space_fraction_map_car.clear();

	for (auto _it: m_link_space_fraction_map_truck) {
		delete _it.second;
	}
	m_link_space_fraction_map_truck.clear();

    for (auto _it: m_queue_dissipated_time_car) {
        delete _it.second;
    }
    m_queue_dissipated_time_car.clear();

	for (auto _it: m_queue_dissipated_time_truck) {
        delete _it.second;
    }
    m_queue_dissipated_time_truck.clear();

	for (auto _it: m_link_tt_map_car) {
        delete _it.second;
    }
    m_link_tt_map_car.clear();

	for (auto _it: m_link_tt_map_truck) {
        delete _it.second;
    }
    m_link_tt_map_truck.clear();

	for (auto _it: m_lmc_car_lower) {
        delete _it.second;
    }
    m_lmc_car_lower.clear();

	for (auto _it: m_lmc_car_upper) {
        delete _it.second;
    }
    m_lmc_car_upper.clear();

	for (auto _it: m_lmc_truck_lower) {
        delete _it.second;
    }
    m_lmc_truck_lower.clear();

	for (auto _it: m_lmc_truck_upper) {
        delete _it.second;
    }
    m_lmc_truck_upper.clear();

	for (auto _it: m_lmc_car_tt) {
        delete _it.second;
    }
    m_lmc_car_tt.clear();

	for (auto _it: m_lmc_truck_tt) {
        delete _it.second;
    }
    m_lmc_truck_tt.clear();
}

int MNM_Dso_Multiclass::initialize(bool verbose)
{
	m_base_dta = new MNM_Dta_Multiclass(m_file_folder);

	m_base_dta -> build_from_files_control();

	if (verbose){
		printf("========================= Finished building! ========================\n");
	}
	
	// m_base_dta -> hook_up_node_and_link();
	// if (verbose){
	// 	printf("====================== Finished node and link hook-up! ====================\n");
	// }
	
	MNM_Dlink *_link;

	// MNM_Dlink_Multiclass *_link_multiclass;

	for (auto _link_it = m_base_dta->m_link_factory->m_link_map.begin();
        _link_it != m_base_dta->m_link_factory->m_link_map.end(); _link_it++) {

		_link = _link_it -> second;

		if (m_link_congested_car.find(_link -> m_link_ID) == m_link_congested_car.end()) {
            m_link_congested_car[_link -> m_link_ID] = new int[m_total_loading_inter];
        }

		if (m_link_congested_truck.find(_link -> m_link_ID) == m_link_congested_truck.end()) {
            m_link_congested_truck[_link -> m_link_ID] = new int[m_total_loading_inter];
        }

		if (m_link_congested_car_tt.find(_link -> m_link_ID) == m_link_congested_car_tt.end()) {
            m_link_congested_car_tt[_link -> m_link_ID] = new bool[m_total_loading_inter];
        }

		if (m_link_congested_truck_tt.find(_link -> m_link_ID) == m_link_congested_truck_tt.end()) {
            m_link_congested_truck_tt[_link -> m_link_ID] = new bool[m_total_loading_inter];
        }

		if (m_link_diff_car.find(_link -> m_link_ID) == m_link_diff_car.end()) {
            m_link_diff_car[_link -> m_link_ID] = new bool[m_total_loading_inter];
        }

		if (m_link_diff_truck.find(_link -> m_link_ID) == m_link_diff_truck.end()) {
            m_link_diff_truck[_link -> m_link_ID] = new bool[m_total_loading_inter];
        }

		if (m_link_space_fraction_map_car.find(_link -> m_link_ID) == m_link_space_fraction_map_car.end()) {
			m_link_space_fraction_map_car[_link -> m_link_ID] = new TFlt[m_total_loading_inter];
		}

		if (m_link_space_fraction_map_truck.find(_link -> m_link_ID) == m_link_space_fraction_map_truck.end()) {
			m_link_space_fraction_map_truck[_link -> m_link_ID] = new TFlt[m_total_loading_inter];
		}

		if (m_queue_dissipated_time_car.find(_link -> m_link_ID) == m_queue_dissipated_time_car.end()) {
            m_queue_dissipated_time_car[_link -> m_link_ID] = new int[m_total_loading_inter];
        }

		if (m_queue_dissipated_time_truck.find(_link -> m_link_ID) == m_queue_dissipated_time_truck.end()) {
            m_queue_dissipated_time_truck[_link -> m_link_ID] = new int[m_total_loading_inter];
        }

		if (m_lmc_car_upper.find(_link -> m_link_ID) == m_lmc_car_upper.end()) {
            m_lmc_car_upper[_link -> m_link_ID] = new TFlt[m_total_loading_inter];
        }

		if (m_lmc_car_lower.find(_link -> m_link_ID) == m_lmc_car_lower.end()) {
            m_lmc_car_lower[_link -> m_link_ID] = new TFlt[m_total_loading_inter];
        }

		if (m_lmc_truck_upper.find(_link -> m_link_ID) == m_lmc_truck_upper.end()) {
            m_lmc_truck_upper[_link -> m_link_ID] = new TFlt[m_total_loading_inter];
        }

		if (m_lmc_truck_lower.find(_link -> m_link_ID) == m_lmc_truck_lower.end()) {
            m_lmc_truck_lower[_link -> m_link_ID] = new TFlt[m_total_loading_inter];
        }

		if (m_lmc_car_tt.find(_link -> m_link_ID) == m_lmc_car_tt.end()) {
            m_lmc_car_tt[_link -> m_link_ID] = new TFlt[m_total_loading_inter];
        }

		if (m_lmc_truck_tt.find(_link -> m_link_ID) == m_lmc_truck_tt.end()) {
            m_lmc_truck_tt[_link -> m_link_ID] = new TFlt[m_total_loading_inter];
        }
    }

	if (verbose){
		printf("========================== Finished Dso initialization! ==========================\n");
	}	
	return 0;
}

MNM_Dta_Multiclass *MNM_Dso_Multiclass::dnl(bool verbose)
{

	MNM_Dta_Multiclass *test_dta = new MNM_Dta_Multiclass(m_file_folder);
    if (verbose){
		printf("================================ DTA set! =================================\n");
	}

    test_dta -> build_from_files_control();

	MNM_Routing_Biclass_Hybrid *_routing_biclass = dynamic_cast<MNM_Routing_Biclass_Hybrid *>(test_dta -> m_routing);

	_routing_biclass -> m_routing_fixed_car -> m_path_table = m_path_table_car;
	_routing_biclass -> m_routing_fixed_truck -> m_path_table = m_path_table_truck;

	if (verbose){
    	printf("========================= Finished set routing! ========================\n");
	}

    test_dta -> hook_up_node_and_link();

    if (verbose){
		printf("====================== Finished node and link hook-up! ====================\n");
	}

    test_dta -> is_ok();

	if (verbose){
    	printf("============================ DTA is OK to run! ============================\n");
	}

	MNM_Dlink *_link;

	MNM_Dlink_Multiclass *_link_multiclass;

	for (auto _link_it = test_dta->m_link_factory->m_link_map.begin(); _link_it != test_dta->m_link_factory->m_link_map.end(); _link_it++) {

		_link = _link_it -> second;

		_link_multiclass = dynamic_cast<MNM_Dlink_Multiclass *>(_link);

		_link_multiclass -> install_cumulative_curve_multiclass();
    }

	if (verbose){
		printf("=========================== Finished install cc ===========================\n");
	}

    test_dta -> pre_loading();

	if (verbose){
 	   printf("========================== Finished pre_loading! ==========================\n");
	}
	
	TInt _current_inter = 0;
	TInt _assign_inter = test_dta -> m_start_assign_interval;
	MNM_Dlink_Multiclass *_link_m;

	while (!test_dta -> finished_loading(_current_inter) || _assign_inter < test_dta -> m_total_assign_inter){

		if (verbose){
			printf("\nCurrent loading interval: %d, Current assignment interval: %d\n", _current_inter(), _assign_inter());
		}

		test_dta -> load_once_control(verbose, _current_inter, _assign_inter);

		// update the link state
		for (auto _link_it: test_dta -> m_link_factory -> m_link_map){
			_link_m = dynamic_cast<MNM_Dlink_Multiclass *>(_link_it.second);

			// record congestion regime 0(default)/1/2/3
			m_link_congested_car[_link_m -> m_link_ID][_current_inter] = _link_m -> m_congested_car;
			m_link_congested_truck[_link_m -> m_link_ID][_current_inter] = _link_m -> m_congested_truck;
			
			m_link_diff_car[_link_m -> m_link_ID][_current_inter] = _link_m -> m_diff_car;
			m_link_diff_truck[_link_m -> m_link_ID][_current_inter] = _link_m -> m_diff_truck;

			m_link_space_fraction_map_car[_link_m -> m_link_ID][_current_inter] = _link_m -> m_space_fraction_car;
			m_link_space_fraction_map_truck[_link_m -> m_link_ID][_current_inter] = _link_m -> m_space_fraction_truck;

		}

		// update assign inter and current loading inter
		if (_current_inter % test_dta -> m_assign_freq == 0 || _current_inter == 0){
			_assign_inter += 1;
		}
		_current_inter += 1;
	}
	return test_dta;
}

int MNM_Dso_Multiclass::build_link_cost_map(MNM_Dta_Multiclass *test_dta, bool verbose)
{
	MNM_Dlink *_link;
	MNM_Dlink_Multiclass *_link_multiclass;
	for (auto _link_it : test_dta -> m_link_factory -> m_link_map) {

		_link = _link_it.second;

		if (m_link_tt_map_car.find(_link -> m_link_ID) == m_link_tt_map_car.end()) {
            m_link_tt_map_car[_link -> m_link_ID] = new TFlt[m_total_loading_inter];
        }

		if (m_link_tt_map_truck.find(_link -> m_link_ID) == m_link_tt_map_truck.end()) {
            m_link_tt_map_truck[_link -> m_link_ID] = new TFlt[m_total_loading_inter];
        }

        for (int i = 0; i < m_total_loading_inter; i++) {
            
			_link_multiclass = dynamic_cast<MNM_Dlink_Multiclass *>(_link);
            // use i+1 as start_time in cc to compute link travel time for vehicles arriving at the beginning of interval i, i+1 is the end of the interval i, the beginning of interval i + 1
            m_link_tt_map_car[_link_it.first][i] = MNM_DTA_GRADIENT::get_travel_time_car(_link_multiclass, TFlt(i+1), m_unit_time, m_total_loading_inter);  // intervals
            m_link_tt_map_truck[_link_it.first][i] = MNM_DTA_GRADIENT::get_travel_time_truck(_link_multiclass, TFlt(i+1), m_unit_time, m_total_loading_inter);

			m_link_congested_car_tt[_link_it.first][i] = m_link_tt_map_car[_link_it.first][i] > _link_multiclass -> get_link_freeflow_tt_loading_car();
			m_link_congested_truck_tt[_link_it.first][i] = m_link_tt_map_truck[_link_it.first][i] > _link_multiclass -> get_link_freeflow_tt_loading_truck();
        }
		if (verbose) {
			std::cout << "********************** link " << _link_it.first() << " build tt maps **********************\n";
		}
    } 
	return 0;
}

TFlt MNM_Dso_Multiclass::dso_get_pmc_car(TInt depart_time, MNM_Path *path, TFlt weight_lower, TFlt weight_upper){
	TFlt _total_cost = 0.0;
    TInt _query_time = depart_time;
    for (TInt _link_ID : path->m_link_vec) {
        _query_time = MNM_Ults::min(TInt(_query_time), TInt(m_total_loading_inter - 1));
        _total_cost += m_lmc_car_upper[_link_ID][_query_time] * weight_upper + m_lmc_car_lower[_link_ID][_query_time] * weight_lower;
		_query_time += m_link_tt_map_car[_link_ID][_query_time];
    }
    return _total_cost;
}

TFlt MNM_Dso_Multiclass::dso_get_pmc_truck(TInt depart_time, MNM_Path *path, TFlt weight_lower, TFlt weight_upper)
{
	TFlt _total_cost = 0.0;
    TInt _query_time = depart_time;
    for (TInt _link_ID : path->m_link_vec) {
        _query_time = MNM_Ults::min(TInt(_query_time), TInt(m_total_loading_inter - 1));
        _total_cost += m_lmc_truck_upper[_link_ID][_query_time] * weight_upper + m_lmc_truck_lower[_link_ID][_query_time] * weight_lower;
		_query_time += m_link_tt_map_truck[_link_ID][_query_time];
    }
    return _total_cost;
}

// New function to handle interactive PMC for cars and trucks
TFlt MNM_Dso_Multiclass::dso_get_pmc_car_interactive(TInt depart_time, MNM_Path *path, TFlt weight_lower, TFlt weight_upper)
{
	TFlt _total_cost = 0.0;
	TInt _query_time = depart_time;
	for (TInt _link_ID : path->m_link_vec) {
		_query_time = MNM_Ults::min(TInt(_query_time), TInt(m_total_loading_inter - 1));
		_total_cost += m_lmc_car_upper[_link_ID][_query_time] * weight_upper + m_lmc_car_lower[_link_ID][_query_time] * weight_lower;

		// check m_link_congested_car state
		if (m_link_congested_car[_link_ID][_query_time] == int(1)) /* free flow state at this time, no interaction should be considered */
		{
			_total_cost += 0.0;
		}
		else if (m_link_congested_car[_link_ID][_query_time] == int(2)) /* semi-congestion state at this time, adding a car has no impact on truck but adding a truck has impact on car */
		{
			_total_cost += 0.0;
		}
		else if (m_link_congested_car[_link_ID][_query_time] == int(3)) /* fully-congestion state at this time, adding a car or a truck has impact on the other class */
		{
			if (m_link_space_fraction_map_truck[_link_ID][_query_time] == TFlt(0.0))
			{
				_total_cost += 0.0;
			}
			else
			{
				_total_cost += (m_lmc_truck_upper[_link_ID][_query_time] * weight_upper + m_lmc_truck_lower[_link_ID][_query_time] * weight_lower) * m_link_space_fraction_map_car[_link_ID][_query_time] / m_link_space_fraction_map_truck[_link_ID][_query_time];
			}
		}
		else /* default state, no interaction should be considered */
		{
			_total_cost += 0.0;
		}
		
		_query_time += m_link_tt_map_car[_link_ID][_query_time];
	}
	return _total_cost;
}

TFlt MNM_Dso_Multiclass::dso_get_pmc_truck_interactive(TInt depart_time, MNM_Path *path, TFlt weight_lower, TFlt weight_upper)
{
	TFlt _total_cost = 0.0;
	TInt _query_time = depart_time;
	for (TInt _link_ID : path->m_link_vec) {
		_query_time = MNM_Ults::min(TInt(_query_time), TInt(m_total_loading_inter - 1));
		_total_cost += m_lmc_truck_upper[_link_ID][_query_time] * weight_upper + m_lmc_truck_lower[_link_ID][_query_time] * weight_lower;

		// check m_link_congested_truck state
		if (m_link_congested_truck[_link_ID][_query_time] == int(1)) {/* free flow state at this time, no interaction should be considered */
			_total_cost += 0.0;
		}
		else if (m_link_congested_truck[_link_ID][_query_time] == int(2) || m_link_congested_truck[_link_ID][_query_time] == int(3)) {
			/* semi-congestion state at this time, adding a truck has impact on car */
			/* fully-congestion state at this time, adding a truck has impact on car */
			if (m_link_space_fraction_map_car[_link_ID][_query_time] == TFlt(0.0)) { // no car at all
				_total_cost += 0.0;
			}
			else {
				_total_cost += (m_lmc_car_upper[_link_ID][_query_time] * weight_upper + m_lmc_car_lower[_link_ID][_query_time] * weight_lower) * m_link_space_fraction_map_truck[_link_ID][_query_time] / m_link_space_fraction_map_car[_link_ID][_query_time];
			}
		}
		else { /* default state, no interaction should be considered */
			_total_cost += 0.0;
		}
		
		_query_time += m_link_tt_map_truck[_link_ID][_query_time];
	}
	return _total_cost;
}

TFlt MNM_Dso_Multiclass::get_arrival_cc_slope(MNM_Dlink_Multiclass* link, TInt veh_class, TFlt start_time, TFlt end_time)
{
	TFlt _cc1, _cc2, _slope = 0.;
	int _delta = int(end_time) - int(start_time);
	IAssert(_delta > 0);

	if (link == nullptr){
        throw std::runtime_error("Error, get_arrival_cc_slope link is null");
    }
	if (veh_class == TInt(0)){
		if (link -> m_N_in_car == nullptr){
			throw std::runtime_error("Error, get_arrival_cc_slope link cumulative curve is not installed");
		}

		if (start_time > link -> m_N_in_car -> m_recorder.back().first) {
        	return 0;
    	}

		IAssert(_delta > 0);
		
		for (int i = 0; i < _delta; i++) {
			_cc1 = link -> m_N_in_car -> get_result(TFlt(start_time + i));
			_cc2 = link -> m_N_in_car -> get_result(TFlt(start_time + i + 1));
			_slope += (_cc2 -_cc1);
		}
	}

	if (veh_class == TInt(1)){
		if (link -> m_N_in_truck == nullptr){
			throw std::runtime_error("Error, get_arrival_cc_slope link cumulative curve is not installed");
		}

		if (start_time > link -> m_N_in_truck -> m_recorder.back().first) {
        	return 0;
    	}
		
		for (int i = 0; i < _delta; i++) {
			_cc1 = link -> m_N_in_truck -> get_result(TFlt(start_time + i));
			_cc2 = link -> m_N_in_truck -> get_result(TFlt(start_time + i + 1));
			_slope += (_cc2 -_cc1);
		}
	}

	return _slope / _delta;
}

TFlt MNM_Dso_Multiclass::get_departure_cc_slope(MNM_Dlink_Multiclass* link, TInt veh_class, TFlt start_time, TFlt end_time)
{
	TFlt _cc1, _cc2, _slope = 0.;
	int _delta = int(end_time) - int(start_time);
	IAssert(_delta > 0);

	if (link == nullptr){
        throw std::runtime_error("Error, get_arrival_cc_slope link is null");
    }
	if (veh_class == TInt(0)){
		if (link -> m_N_out_car == nullptr){
			throw std::runtime_error("Error, get_arrival_cc_slope link cumulative curve is not installed");
		}

		if (start_time > link -> m_N_out_car -> m_recorder.back().first) {
        	return 0;
    	}
		
		for (int i = 0; i < _delta; i++) {
			_cc1 = link -> m_N_out_car -> get_result(TFlt(start_time + i));
			_cc2 = link -> m_N_out_car -> get_result(TFlt(start_time + i + 1));
			_slope += (_cc2 -_cc1);
		}
	}

	if (veh_class == TInt(1)){
		if (link -> m_N_out_truck == nullptr){
			throw std::runtime_error("Error, get_arrival_cc_slope link cumulative curve is not installed");
		}

		if (start_time > link -> m_N_out_truck -> m_recorder.back().first) {
        	return 0;
    	}
		
		for (int i = 0; i < _delta; i++) {
			_cc1 = link -> m_N_out_truck -> get_result(TFlt(start_time + i));
			_cc2 = link -> m_N_out_truck -> get_result(TFlt(start_time + i + 1));
			_slope += (_cc2 -_cc1);
		}
	}

	return _slope / _delta;
}

int MNM_Dso_Multiclass::get_link_marginal_cost_tt(MNM_Dta_Multiclass *test_dta, bool verbose)
{
	MNM_Dlink_Multiclass *_link_multiclass;

	int _total_loading_inter = m_total_loading_inter;

	IAssert(_total_loading_inter > 0);

	int _actual_lift_up_time_car;

	int _actual_lift_up_time_truck;

	bool _flg_car; // indicator for finding the queue dissipate time

	bool _flg_truck;

	TInt _link_fftt_car;

	TInt _link_fftt_truck;

	if (verbose) {
		std::cout << "\n********************** Begin MNM_Dso_Multiclass::get_link_marginal_cost_tt **********************\n";
	}

	for (auto _link_it : test_dta -> m_link_factory -> m_link_map){
		
		_link_multiclass = dynamic_cast<MNM_Dlink_Multiclass *>(_link_it.second);

		_link_fftt_car = _link_multiclass -> get_link_freeflow_tt_loading_car();

		_link_fftt_truck = _link_multiclass -> get_link_freeflow_tt_loading_truck();

		for (int i = 0; i < _total_loading_inter; i++){

			_actual_lift_up_time_car = _total_loading_inter;
			_actual_lift_up_time_truck = _total_loading_inter;

			// car lift up time
			for (int j = i; j < _total_loading_inter; j++){
				if (dynamic_cast<MNM_Dlink_Ctm_Multiclass *>(_link_multiclass) != nullptr){
					// check after i, which is the free flow regime
					TFlt _inflow_rate_car = get_arrival_cc_slope(_link_multiclass, 0, TFlt(j), TFlt(j+1));
					TFlt _cap_car = dynamic_cast<MNM_Dlink_Ctm_Multiclass*>(_link_multiclass) -> m_cell_array.front() -> m_flow_cap_car * test_dta -> m_unit_time;
					
					if (MNM_Ults::approximate_less_than(_inflow_rate_car * test_dta -> m_flow_scalar, floor(_cap_car * test_dta -> m_flow_scalar))){
						_actual_lift_up_time_car = j;
						break;
					}
				}
				else if (dynamic_cast<MNM_Dlink_Pq_Multiclass *>(_link_multiclass) != nullptr) {
					_actual_lift_up_time_car = j;
					break;
				}
				else {
					throw std::runtime_error("MNM_Dso_Multiclass::get_link_marginal_cost_tt, Link type not implemented for actual lift up time car\n");
				}
			}

			// truck lift up time
			for (int j = i; j < _total_loading_inter; j++){
				if (dynamic_cast<MNM_Dlink_Ctm_Multiclass *>(_link_multiclass) != nullptr) {
					// check after i, which is the free flow regime
					TFlt _inflow_rate_truck = get_arrival_cc_slope(_link_multiclass, 1, TFlt(j), TFlt(j+1));
					TFlt _cap_truck = dynamic_cast<MNM_Dlink_Ctm_Multiclass*>(_link_multiclass) -> m_cell_array.front() -> m_flow_cap_truck * test_dta -> m_unit_time;

					if (MNM_Ults::approximate_less_than(_inflow_rate_truck * test_dta -> m_flow_scalar, floor(_cap_truck * test_dta -> m_flow_scalar))){
						_actual_lift_up_time_truck = j;
						break;
					}
				}
				else if (dynamic_cast<MNM_Dlink_Pq_Multiclass *>(_link_multiclass) != nullptr) {
					_actual_lift_up_time_car = j;
					break;
				}
				else {
					throw std::runtime_error("MNM_Dso_Multiclass::get_link_marginal_cost_tt, Link type not implemented for actual lift up time car\n");
				}
			}

			// queue dissipate car
			if (_actual_lift_up_time_car == _total_loading_inter) {
				m_queue_dissipated_time_car[_link_it.first][i] = 2 * _total_loading_inter;
			}
			else {
				if (m_link_congested_car_tt[_link_it.first][_actual_lift_up_time_car]){
					if (_actual_lift_up_time_car == _total_loading_inter - 1){
						m_queue_dissipated_time_car[_link_it.first][i] = 2 * _total_loading_inter;
					}
					else{
						_flg_car = false;
						for (int k = _actual_lift_up_time_car + 1; k < _total_loading_inter; k++){
							if (m_link_congested_car_tt[_link_it.first][k-1] && !m_link_congested_car_tt[_link_it.first][k]){
								m_queue_dissipated_time_car[_link_it.first][i] = k;
								_flg_car = true;
								break;
							}
						}
						if (!_flg_car){
							m_queue_dissipated_time_car[_link_it.first][i] = 2 * _total_loading_inter;
						}
					}
				}
				else{
					if (MNM_Ults::approximate_equal(m_link_tt_map_car[_link_it.first][_actual_lift_up_time_car], (float)_link_fftt_car)){
						if (dynamic_cast<MNM_Dlink_Ctm_Multiclass*>(_link_multiclass) != nullptr) {
							TFlt _outflow_rate_car = get_departure_cc_slope(_link_multiclass, 0, TFlt(_actual_lift_up_time_car + (int)_link_fftt_car),
																							   TFlt(_actual_lift_up_time_car + (int)_link_fftt_car + 1));
							TFlt _out_cap_car = dynamic_cast<MNM_Dlink_Ctm_Multiclass*>(_link_multiclass) -> m_cell_array.back() -> m_flow_cap_car * test_dta -> m_unit_time;
							if (MNM_Ults::approximate_equal(_outflow_rate_car * test_dta -> m_flow_scalar, floor(_out_cap_car * test_dta -> m_flow_scalar))){
								if (_actual_lift_up_time_car == _total_loading_inter - 1) {
                                    m_queue_dissipated_time_car[_link_it.first][i] = 1.2 * _total_loading_inter;
                                }
                                else {
                                    _flg_car = false;
                                    for (int k = _actual_lift_up_time_car + 1; k < _total_loading_inter; k++) {
                                        if (m_link_congested_car_tt[_link_it.first][k-1] && !m_link_congested_car_tt[_link_it.first][k]) {
                                            m_queue_dissipated_time_car[_link_it.first][i] = k;
                                            _flg_car = true;
                                            break;
                                        }
                                    }
                                    if (!_flg_car) {
                                        m_queue_dissipated_time_car[_link_it.first][i] = 1.2 * _total_loading_inter;
                                    }
                                }
							}
							else{
								// TODO: boundary condition ?? jiachao
                                m_queue_dissipated_time_car[_link_it.first][i] = _actual_lift_up_time_car;
							}
						}
						else if (dynamic_cast<MNM_Dlink_Pq_Multiclass *>(_link_multiclass) != nullptr) {
                            // PQ link as OD connectors always has sufficient capacity
                            m_queue_dissipated_time_car[_link_it.first][i] = _actual_lift_up_time_car;
                        }
                        else {
                            throw std::runtime_error("MNM_Dso_Multiclass::get_link_marginal_cost_tt, Link type not implemented queue dissipate time car\n");
                        }
					}
					else {
                        throw std::runtime_error("MNM_Dso_Multiclass::get_link_marginal_cost_tt, Link travel time less than fftt\n");
                    }
				}
			}
			IAssert(m_queue_dissipated_time_car[_link_it.first][i] >= _actual_lift_up_time_car);

			// queue dissipate truck
			if (_actual_lift_up_time_truck == _total_loading_inter) {
				m_queue_dissipated_time_truck[_link_it.first][i] = 2 * _total_loading_inter;
			}
			else {
				if (m_link_congested_truck_tt[_link_it.first][_actual_lift_up_time_truck]){
					if (_actual_lift_up_time_truck == _total_loading_inter - 1){
						m_queue_dissipated_time_truck[_link_it.first][i] = 2 * _total_loading_inter;
					}
					else{
						_flg_truck = false;
						for (int k = _actual_lift_up_time_truck + 1; k < _total_loading_inter; k++){
							if (m_link_congested_truck_tt[_link_it.first][k-1] && !m_link_congested_truck_tt[_link_it.first][k]){
								m_queue_dissipated_time_truck[_link_it.first][i] = k;
								_flg_truck = true;
								break;
							}
						}
						if (!_flg_truck){
							m_queue_dissipated_time_truck[_link_it.first][i] = 2 * _total_loading_inter;
						}
					}
				}
				else{
					if (MNM_Ults::approximate_equal(m_link_tt_map_truck[_link_it.first][_actual_lift_up_time_truck], (float)_link_fftt_truck)){
						if (dynamic_cast<MNM_Dlink_Ctm_Multiclass*>(_link_multiclass) != nullptr) {

							TFlt _outflow_rate_truck = get_departure_cc_slope(_link_multiclass, 1, TFlt(_actual_lift_up_time_truck + (int)_link_fftt_truck),
																							   	   TFlt(_actual_lift_up_time_truck + (int)_link_fftt_truck + 1));

							TFlt _out_cap_truck = dynamic_cast<MNM_Dlink_Ctm_Multiclass*>(_link_multiclass) -> m_cell_array.back() -> m_flow_cap_truck * test_dta -> m_unit_time;
							
							if (MNM_Ults::approximate_equal(_outflow_rate_truck * test_dta -> m_flow_scalar, floor(_out_cap_truck * test_dta -> m_flow_scalar))){
								if (_actual_lift_up_time_truck == _total_loading_inter - 1) {
                                    m_queue_dissipated_time_truck[_link_it.first][i] = 1.2 * _total_loading_inter;
                                }
                                else {
                                    _flg_truck = false;
                                    for (int k = _actual_lift_up_time_truck + 1; k < _total_loading_inter; k++) {
                                        if (m_link_congested_truck_tt[_link_it.first][k-1] && !m_link_congested_truck_tt[_link_it.first][k]) {
                                            m_queue_dissipated_time_truck[_link_it.first][i] = k;
                                            _flg_truck = true;
                                            break;
                                        }
                                    }
                                    if (!_flg_truck) {
                                        m_queue_dissipated_time_truck[_link_it.first][i] = 1.2 * _total_loading_inter;
                                    }
                                }
							}
							else{
								// TODO: boundary condition ?? jiachao
                                m_queue_dissipated_time_truck[_link_it.first][i] = _actual_lift_up_time_truck;
							}
						}
						else if (dynamic_cast<MNM_Dlink_Pq_Multiclass *>(_link_multiclass) != nullptr) {
                            // PQ link as OD connectors always has sufficient capacity
                            m_queue_dissipated_time_truck[_link_it.first][i] = _actual_lift_up_time_truck;
                        }
                        else {
                            throw std::runtime_error("MNM_Dso_Multiclass::get_link_marginal_cost_tt, Link type not implemented queue dissipate time truck\n");
                        }
					}
					else {
                        throw std::runtime_error("MNM_Dso_Multiclass::get_link_marginal_cost_tt, Link travel time less than fftt\n");
                    }
				}
			}
			IAssert(m_queue_dissipated_time_truck[_link_it.first][i] >= _actual_lift_up_time_truck);

			m_lmc_car_upper[_link_it.first][i] = m_queue_dissipated_time_car[_link_it.first][i] - _actual_lift_up_time_car + _link_fftt_car;
			m_lmc_truck_upper[_link_it.first][i] = m_queue_dissipated_time_truck[_link_it.first][i] - _actual_lift_up_time_truck + _link_fftt_truck;

			// lower bounds depend on diff indicator
            if (m_link_diff_car[_link_it.first][i] == true) {
				m_lmc_car_lower[_link_it.first][i] = m_lmc_car_upper[_link_it.first][i];
			}
			else {
				IAssert(m_link_diff_car[_link_it.first][i] == false);
				m_lmc_car_lower[_link_it.first][i] = _link_fftt_car;
			}

			if (m_link_diff_truck[_link_it.first][i] == true) {
				m_lmc_truck_lower[_link_it.first][i] = m_lmc_truck_upper[_link_it.first][i];
			}
			else {
				IAssert(m_link_diff_truck[_link_it.first][i] == false);
				m_lmc_truck_lower[_link_it.first][i] = _link_fftt_truck;
			}
		}
		if (verbose) {
			std::cout << "********************** link " << _link_it.first() << " got gradient **********************\n";
		}
	}
	return 0;
}

int MNM_Dso_Multiclass::get_link_marginal_cost_v1(MNM_Dta_Multiclass *test_dta, bool verbose)
{
	MNM_Dlink_Multiclass *_link_multiclass;
	int _total_loading_inter = m_total_loading_inter;
	IAssert(_total_loading_inter > 0);
	int _actual_lift_up_time_car;
	int _actual_lift_up_time_truck;
	bool _flg_car; // indicator for finding the queue dissipate time
	bool _flg_truck;
	TInt _link_fftt_car;
	TInt _link_fftt_truck;

	if (verbose) {
		std::cout << "\n********************** Begin MNM_Dso_Multiclass::get_link_marginal_cost **********************\n";
	}

	for (auto _link_it : test_dta -> m_link_factory -> m_link_map){
		
		_link_multiclass = dynamic_cast<MNM_Dlink_Multiclass *>(_link_it.second);
		_link_fftt_car = _link_multiclass -> get_link_freeflow_tt_loading_car();
		_link_fftt_truck = _link_multiclass -> get_link_freeflow_tt_loading_truck();

		for (int i = 0; i < _total_loading_inter; i++){

			_actual_lift_up_time_car = _total_loading_inter;
			_actual_lift_up_time_truck = _total_loading_inter;

			// car lift up time
			for (int j = i; j < _total_loading_inter; j++){
				if (dynamic_cast<MNM_Dlink_Ctm_Multiclass *>(_link_multiclass) != nullptr || dynamic_cast<MNM_Dlink_Lq_Multiclass *>(_link_multiclass) != nullptr){
					// check after time i, when is the free flow regime, that is the actual lift up time
					if (m_link_congested_car[_link_it.first][j] == int(1) || m_link_congested_car[_link_it.first][j] == int(0))
					{
						_actual_lift_up_time_car = j;
						break;
					}
				}
				else if (dynamic_cast<MNM_Dlink_Pq_Multiclass *>(_link_multiclass) != nullptr){
					_actual_lift_up_time_car = j;
					break;
				}
				else {
					throw std::runtime_error("MNM_Dso_Multiclass::get_link_marginal_cost, Link type not implemented for actual lift up time car\n");
				}
			}

			// truck lift up time
			for (int j = i; j < _total_loading_inter; j++){
				if (dynamic_cast<MNM_Dlink_Ctm_Multiclass *>(_link_multiclass) != nullptr || dynamic_cast<MNM_Dlink_Lq_Multiclass *>(_link_multiclass) != nullptr){
					if (m_link_congested_truck[_link_it.first][j] == int(1) || m_link_congested_truck[_link_it.first][j] == int(0))
					{
						_actual_lift_up_time_truck = j;
						break;
					}
				}
				else if (dynamic_cast<MNM_Dlink_Pq_Multiclass *>(_link_multiclass) != nullptr){
					_actual_lift_up_time_truck = j;
					break;
				}
				else {
					throw std::runtime_error("MNM_Dso_Multiclass::get_link_marginal_cost, Link type not implemented for actual lift up time truck\n");
				}
			}

			// queue dissipate time - car
			if (_actual_lift_up_time_car == _total_loading_inter) {
				m_queue_dissipated_time_car[_link_it.first][i] = 1.2 * _total_loading_inter; // just for making code run
			}
			else {
                if (m_link_congested_car[_link_it.first][_actual_lift_up_time_car] == int(2) || m_link_congested_car[_link_it.first][_actual_lift_up_time_car] == int(3)) {
                    if (_actual_lift_up_time_car == _total_loading_inter - 1) {
                        // assign a large dissipate time 2 * _total_loading_inter
						m_queue_dissipated_time_car[_link_it.first][i] = 1.2 * _total_loading_inter;
                    }
                    else {
                        _flg_car = false;
                        for (int k = _actual_lift_up_time_car + 1; k < _total_loading_inter; k++) {
							// check the breakpoint
							// k-1 time inter is congested (regime 2/3) for car but k time inter is free flow (regime 1/0 - default)
                            if ((m_link_congested_car[_link_it.first][k - 1] == int(2) || m_link_congested_car[_link_it.first][k - 1] == int(3)) && 
								(m_link_congested_car[_link_it.first][k] == int(1) || m_link_congested_car[_link_it.first][k] == int(0))) 
							{
								// set k inter as the queue dissipate time
                                m_queue_dissipated_time_car[_link_it.first][i] = k;
								// set flg = true, find the queue dissipate time
                                _flg_car = true;
                                break;
                            }
                        }
						// loop finish but still no queue dissipate time is found
                        if (!_flg_car) {
							// assign a large dissipate time 2 * _total_loading_inter
                            m_queue_dissipated_time_car[_link_it.first][i] = 1.2 * _total_loading_inter;
                        }
                    }
                }
				else { // current time inter is not congested
					if (m_link_congested_car[_link_it.first][_actual_lift_up_time_car] == int(1) || 
						m_link_congested_car[_link_it.first][_actual_lift_up_time_car] == int(0)) {
                        
						// based on subgradient paper, when out flow = capacity and link tt = fftt, this is critical state where the subgradient applies
						// check if diff = false, nondifferentiable
						if (dynamic_cast<MNM_Dlink_Ctm_Multiclass*>(_link_multiclass) != nullptr || dynamic_cast<MNM_Dlink_Lq_Multiclass*>(_link_multiclass) != nullptr) {
                            if (m_link_diff_car[_link_it.first][_actual_lift_up_time_car] == false) {
                                if (_actual_lift_up_time_car == _total_loading_inter - 1) {
                                    m_queue_dissipated_time_car[_link_it.first][i] = 1.2 * _total_loading_inter;
                                }
                                else {
                                    _flg_car = false;
                                    for (int k = _actual_lift_up_time_car + 1; k < _total_loading_inter; k++) {
                                        if ((m_link_congested_car[_link_it.first][k - 1] == int(2) || m_link_congested_car[_link_it.first][k - 1] == int(3)) && 
											(m_link_congested_car[_link_it.first][k] == int(1) || m_link_congested_car[_link_it.first][k] == int(0))) 
										{
                                            m_queue_dissipated_time_car[_link_it.first][i] = k;
                                            _flg_car = true;
                                            break;
                                        }
                                    }
                                    if (!_flg_car) {
                                        m_queue_dissipated_time_car[_link_it.first][i] = 1.2 * _total_loading_inter;
                                    }
                                }
                            } 
                            else {
                                // TODO: boundary condition ?? jiachao
                                m_queue_dissipated_time_car[_link_it.first][i] = _actual_lift_up_time_car;
                            }
                        }
                        else if (dynamic_cast<MNM_Dlink_Pq_Multiclass *>(_link_multiclass) != nullptr) {
                            // PQ link as OD connectors always has sufficient capacity
                            m_queue_dissipated_time_car[_link_it.first][i] = _actual_lift_up_time_car;
                        }
                        else {
                            throw std::runtime_error("MNM_Dso_Multiclass::get_link_marginal_cost, Link type not implemented queue dissipate time car\n");
                        }
                    }
                    else {
                        throw std::runtime_error("MNM_Dso_Multiclass::get_link_marginal_cost, link congestion indicator has some abnormal value queue dissipate time car\n");
                    }
				}
			}

			// queue dissipate truck
			if (_actual_lift_up_time_truck == _total_loading_inter) {
				m_queue_dissipated_time_truck[_link_it.first][i] = 1.2 * _total_loading_inter;
			}
			else {
                if (m_link_congested_truck[_link_it.first][_actual_lift_up_time_truck] == int(2) || m_link_congested_truck[_link_it.first][_actual_lift_up_time_truck] == int(3)) {
                    if (_actual_lift_up_time_truck == _total_loading_inter - 1) {
                        // assign a large dissipate time 2 * _total_loading_inter
						m_queue_dissipated_time_truck[_link_it.first][i] = 1.2 * _total_loading_inter;
                    }
                    else {
                        _flg_truck = false;
                        for (int k = _actual_lift_up_time_truck + 1; k < _total_loading_inter; k++) {
							// check the breakpoint
							// k-1 time inter is congested (regime 2/3) for truck but k time inter is free flow (regime 1/0 - default)
                            if ((m_link_congested_truck[_link_it.first][k - 1] == int(2) || m_link_congested_truck[_link_it.first][k - 1] == int(3)) && 
								(m_link_congested_truck[_link_it.first][k] == int(1) || m_link_congested_truck[_link_it.first][k] == int(0))) 
							{
								// set k inter as the queue dissipate time
                                m_queue_dissipated_time_truck[_link_it.first][i] = k;
								// set flg = true, find the queue dissipate time
                                _flg_truck = true;
                                break;
                            }
                        }
						// loop finish but still no queue dissipate time is found
                        if (!_flg_truck) {
							// assign a large dissipate time 2 * _total_loading_inter
                            m_queue_dissipated_time_truck[_link_it.first][i] = 1.2 * _total_loading_inter;
                        }
                    }
                }
				else { // current time inter is not congested
					if (m_link_congested_truck[_link_it.first][_actual_lift_up_time_truck] == int(1) || 
						m_link_congested_truck[_link_it.first][_actual_lift_up_time_truck] == int(0)) {
                        
						// check if diff = false, nondifferentiable
						if (dynamic_cast<MNM_Dlink_Ctm_Multiclass*>(_link_multiclass) != nullptr || dynamic_cast<MNM_Dlink_Lq_Multiclass*>(_link_multiclass) != nullptr) {
                            if (m_link_diff_truck[_link_it.first][_actual_lift_up_time_truck] == false) {
                                if (_actual_lift_up_time_truck == _total_loading_inter - 1) {
                                    m_queue_dissipated_time_truck[_link_it.first][i] = 1.2 * _total_loading_inter;
                                }
                                else {
                                    _flg_truck = false;
                                    for (int k = _actual_lift_up_time_truck + 1; k < _total_loading_inter; k++) {
                                        if ((m_link_congested_truck[_link_it.first][k - 1] == int(2) || m_link_congested_truck[_link_it.first][k - 1] == int(3)) && 
											(m_link_congested_truck[_link_it.first][k] == int(1) || m_link_congested_truck[_link_it.first][k] == int(0))) 
										{
                                            m_queue_dissipated_time_truck[_link_it.first][i] = k;
                                            _flg_truck = true;
                                            break;
                                        }
                                    }
                                    if (!_flg_truck) {
                                        m_queue_dissipated_time_truck[_link_it.first][i] = 1.2 * _total_loading_inter;
                                    }
                                }
                            } 
                            else {
                                // TODO: boundary condition ?? jiachao
                                m_queue_dissipated_time_truck[_link_it.first][i] = _actual_lift_up_time_truck;
                            }
                        }
                        else if (dynamic_cast<MNM_Dlink_Pq_Multiclass *>(_link_multiclass) != nullptr) {
                            // PQ link as OD connectors always has sufficient capacity
                            m_queue_dissipated_time_truck[_link_it.first][i] = _actual_lift_up_time_truck;
                        }
                        else {
                            throw std::runtime_error("MNM_Dso_Multiclass::get_link_marginal_cost, Link type not implemented queue dissipate time truck\n");
                        }
                    }
                    else {
                        throw std::runtime_error("MNM_Dso_Multiclass::get_link_marginal_cost, link congestion indicator has some abnormal value queue dissipate time truck\n");
                    }
				}
			}

			IAssert(m_queue_dissipated_time_car[_link_it.first][i] >= _actual_lift_up_time_car);
			IAssert(m_queue_dissipated_time_truck[_link_it.first][i] >= _actual_lift_up_time_truck);

			// calculate gradient upper and lower bound
			// upper bounds are same = queue_dissipate_time - _actual_lift_up_time + freeflow
			m_lmc_car_upper[_link_it.first][i] = m_queue_dissipated_time_car[_link_it.first][i] - _actual_lift_up_time_car + _link_fftt_car;
			m_lmc_truck_upper[_link_it.first][i] = m_queue_dissipated_time_truck[_link_it.first][i] - _actual_lift_up_time_truck + _link_fftt_truck;

			// lower bounds depend on diff indicator
            if (m_link_diff_car[_link_it.first][i] == true) {
				m_lmc_car_lower[_link_it.first][i] = m_lmc_car_upper[_link_it.first][i];
			}
			else {
				IAssert(m_link_diff_car[_link_it.first][i] == false);
				m_lmc_car_lower[_link_it.first][i] = _link_fftt_car;
			}

			if (m_link_diff_truck[_link_it.first][i] == true) {
				m_lmc_truck_lower[_link_it.first][i] = m_lmc_truck_upper[_link_it.first][i];
			}
			else {
				IAssert(m_link_diff_truck[_link_it.first][i] == false);
				m_lmc_truck_lower[_link_it.first][i] = _link_fftt_truck;
			}
		}	// end time interval loop
		if (verbose) {
			std::cout << "********************** link " << _link_it.first() << " got gradient **********************\n";
		}
	}	// end link loop
	return 0;
}

int MNM_Dso_Multiclass::get_link_marginal_cost_v2(MNM_Dta_Multiclass *test_dta, bool verbose)
{
	MNM_Dlink_Multiclass *_link_multiclass;
	int _total_loading_inter = m_total_loading_inter;
	IAssert(_total_loading_inter > 0);
	int _actual_lift_up_time_car;
	int _actual_lift_up_time_truck;
	bool _flg_car;
	bool _flg_truck;
	TInt _link_fftt_car;
	TInt _link_fftt_truck;

	if (verbose) {
		std::cout << "\n********************** Begin MNM_Dso_Multiclass::get_link_marginal_cost_v2 **********************\n";
	}

	// loop through all links
	for (auto _link_it : test_dta -> m_link_factory -> m_link_map)
	{
		_link_multiclass = dynamic_cast<MNM_Dlink_Multiclass *>(_link_it.second);
		_link_fftt_car = _link_multiclass -> get_link_freeflow_tt_loading_car();
		_link_fftt_truck = _link_multiclass -> get_link_freeflow_tt_loading_truck();

		for (int i = 0; i < _total_loading_inter; i++)
		{
			// use exact time i as the lift up time, do not consider congestion
			_actual_lift_up_time_car = i;
			_actual_lift_up_time_truck = i;

			// queue dissipation time - car
			if (_actual_lift_up_time_car == _total_loading_inter) {
				m_queue_dissipated_time_car[_link_it.first][i] = 1.2 * _total_loading_inter; // just for making code run
			}
			else {
                if (m_link_congested_car[_link_it.first][_actual_lift_up_time_car] == int(2) || m_link_congested_car[_link_it.first][_actual_lift_up_time_car] == int(3)) {
                    if (_actual_lift_up_time_car == _total_loading_inter - 1) {
                        // assign a large dissipate time 2 * _total_loading_inter
						m_queue_dissipated_time_car[_link_it.first][i] = 1.2 * _total_loading_inter;
                    }
                    else {
                        _flg_car = false;
                        for (int k = _actual_lift_up_time_car + 1; k < _total_loading_inter; k++) {
							// check the breakpoint
							// k-1 time inter is congested (regime 2/3) for car but k time inter is free flow (regime 1/0 - default)
                            if ((m_link_congested_car[_link_it.first][k - 1] == int(2) || m_link_congested_car[_link_it.first][k - 1] == int(3)) && 
								(m_link_congested_car[_link_it.first][k] == int(1) || m_link_congested_car[_link_it.first][k] == int(0))) 
							{
								// set k inter as the queue dissipate time
                                m_queue_dissipated_time_car[_link_it.first][i] = k;
								// set flg = true, find the queue dissipate time
                                _flg_car = true;
                                break;
                            }
                        }
						// loop finish but still no queue dissipate time is found
                        if (!_flg_car) {
							// assign a large dissipate time 2 * _total_loading_inter
                            m_queue_dissipated_time_car[_link_it.first][i] = 1.2 * _total_loading_inter;
                        }
                    }
                }
				else { // current time inter is not congested
					if (m_link_congested_car[_link_it.first][_actual_lift_up_time_car] == int(1) || 
						m_link_congested_car[_link_it.first][_actual_lift_up_time_car] == int(0)) 
					{
						// based on subgradient paper, when out flow = capacity and link tt = fftt, this is critical state where the subgradient applies
						// check if diff = false, nondifferentiable

						if (dynamic_cast<MNM_Dlink_Ctm_Multiclass*>(_link_multiclass) != nullptr || dynamic_cast<MNM_Dlink_Lq_Multiclass*>(_link_multiclass) != nullptr) {
                            if (m_link_diff_car[_link_it.first][_actual_lift_up_time_car] == false) 
							{
                                if (_actual_lift_up_time_car == _total_loading_inter - 1) {
                                    m_queue_dissipated_time_car[_link_it.first][i] = 1.2 * _total_loading_inter;
                                }
                                else {
                                    _flg_car = false;
                                    for (int k = _actual_lift_up_time_car + 1; k < _total_loading_inter; k++) {
                                        if ((m_link_congested_car[_link_it.first][k - 1] == int(2) || m_link_congested_car[_link_it.first][k - 1] == int(3)) && 
											(m_link_congested_car[_link_it.first][k] == int(1) || m_link_congested_car[_link_it.first][k] == int(0))) 
										{
                                            m_queue_dissipated_time_car[_link_it.first][i] = k;
                                            _flg_car = true;
                                            break;
                                        }
                                    }
                                    if (!_flg_car) {
                                        m_queue_dissipated_time_car[_link_it.first][i] = 1.2 * _total_loading_inter;
                                    }
                                }
                            } 
                            else {
                                // TODO: boundary condition ?? jiachao
                                m_queue_dissipated_time_car[_link_it.first][i] = _actual_lift_up_time_car;
                            }
                        }
                        else if (dynamic_cast<MNM_Dlink_Pq_Multiclass *>(_link_multiclass) != nullptr) {
                            // PQ link as OD connectors always has sufficient capacity
                            m_queue_dissipated_time_car[_link_it.first][i] = _actual_lift_up_time_car;
                        }
                        else {
                            throw std::runtime_error("MNM_Dso_Multiclass::get_link_marginal_cost, Link type not implemented queue dissipate time car\n");
                        }
                    }
                    else {
                        throw std::runtime_error("MNM_Dso_Multiclass::get_link_marginal_cost, link congestion indicator has some abnormal value queue dissipate time car\n");
                    }
				}
			}

			// queue dissipation time - truck
			if (_actual_lift_up_time_truck == _total_loading_inter) {
				m_queue_dissipated_time_truck[_link_it.first][i] = 1.2 * _total_loading_inter;
			}
			else {
                if (m_link_congested_truck[_link_it.first][_actual_lift_up_time_truck] == int(2) || m_link_congested_truck[_link_it.first][_actual_lift_up_time_truck] == int(3)) {
                    if (_actual_lift_up_time_truck == _total_loading_inter - 1) {
                        // assign a large dissipate time 2 * _total_loading_inter
						m_queue_dissipated_time_truck[_link_it.first][i] = 1.2 * _total_loading_inter;
                    }
                    else {
                        _flg_truck = false;
                        for (int k = _actual_lift_up_time_truck + 1; k < _total_loading_inter; k++) {
							// check the breakpoint
							// k-1 time inter is congested (regime 2/3) for truck but k time inter is free flow (regime 1/0 - default)
                            if ((m_link_congested_truck[_link_it.first][k - 1] == int(2) || m_link_congested_truck[_link_it.first][k - 1] == int(3)) && 
								(m_link_congested_truck[_link_it.first][k] == int(1) || m_link_congested_truck[_link_it.first][k] == int(0))) 
							{
								// set k inter as the queue dissipate time
                                m_queue_dissipated_time_truck[_link_it.first][i] = k;
								// set flg = true, find the queue dissipate time
                                _flg_truck = true;
                                break;
                            }
                        }
						// loop finish but still no queue dissipate time is found
                        if (!_flg_truck) {
							// assign a large dissipate time 2 * _total_loading_inter
                            m_queue_dissipated_time_truck[_link_it.first][i] = 1.2 * _total_loading_inter;
                        }
                    }
                }
				else { // current time inter is not congested
					if (m_link_congested_truck[_link_it.first][_actual_lift_up_time_truck] == int(1) || 
						m_link_congested_truck[_link_it.first][_actual_lift_up_time_truck] == int(0)) {
                        
						// check if diff = false, nondifferentiable
						if (dynamic_cast<MNM_Dlink_Ctm_Multiclass*>(_link_multiclass) != nullptr || dynamic_cast<MNM_Dlink_Lq_Multiclass*>(_link_multiclass) != nullptr) {
                            if (m_link_diff_truck[_link_it.first][_actual_lift_up_time_truck] == false) {
                                if (_actual_lift_up_time_truck == _total_loading_inter - 1) {
                                    m_queue_dissipated_time_truck[_link_it.first][i] = 1.2 * _total_loading_inter;
                                }
                                else {
                                    _flg_truck = false;
                                    for (int k = _actual_lift_up_time_truck + 1; k < _total_loading_inter; k++) {
                                        if ((m_link_congested_truck[_link_it.first][k - 1] == int(2) || m_link_congested_truck[_link_it.first][k - 1] == int(3)) && 
											(m_link_congested_truck[_link_it.first][k] == int(1) || m_link_congested_truck[_link_it.first][k] == int(0))) 
										{
                                            m_queue_dissipated_time_truck[_link_it.first][i] = k;
                                            _flg_truck = true;
                                            break;
                                        }
                                    }
                                    if (!_flg_truck) {
                                        m_queue_dissipated_time_truck[_link_it.first][i] = 1.2 * _total_loading_inter;
                                    }
                                }
                            } 
                            else {
                                // TODO: boundary condition ?? jiachao
                                m_queue_dissipated_time_truck[_link_it.first][i] = _actual_lift_up_time_truck;
                            }
                        }
                        else if (dynamic_cast<MNM_Dlink_Pq_Multiclass *>(_link_multiclass) != nullptr) {
                            // PQ link as OD connectors always has sufficient capacity
                            m_queue_dissipated_time_truck[_link_it.first][i] = _actual_lift_up_time_truck;
                        }
                        else {
                            throw std::runtime_error("MNM_Dso_Multiclass::get_link_marginal_cost, Link type not implemented queue dissipate time truck\n");
                        }
                    }
                    else {
                        throw std::runtime_error("MNM_Dso_Multiclass::get_link_marginal_cost, link congestion indicator has some abnormal value queue dissipate time truck\n");
                    }
				}
			}

			IAssert(m_queue_dissipated_time_car[_link_it.first][i] >= _actual_lift_up_time_car);
			IAssert(m_queue_dissipated_time_truck[_link_it.first][i] >= _actual_lift_up_time_truck);

			// calculate gradient upper and lower bound
			// upper bounds are same = queue_dissipate_time - _actual_lift_up_time + freeflow
			m_lmc_car_upper[_link_it.first][i] = m_queue_dissipated_time_car[_link_it.first][i] - _actual_lift_up_time_car + _link_fftt_car;
			m_lmc_truck_upper[_link_it.first][i] = m_queue_dissipated_time_truck[_link_it.first][i] - _actual_lift_up_time_truck + _link_fftt_truck;

			// lower bounds depend on diff indicator
            if (m_link_diff_car[_link_it.first][i] == true) {
				m_lmc_car_lower[_link_it.first][i] = m_lmc_car_upper[_link_it.first][i];
			}
			else {
				IAssert(m_link_diff_car[_link_it.first][i] == false);
				m_lmc_car_lower[_link_it.first][i] = _link_fftt_car;
			}

			if (m_link_diff_truck[_link_it.first][i] == true) {
				m_lmc_truck_lower[_link_it.first][i] = m_lmc_truck_upper[_link_it.first][i];
			}
			else {
				IAssert(m_link_diff_truck[_link_it.first][i] == false);
				m_lmc_truck_lower[_link_it.first][i] = _link_fftt_truck;
			}
		}
		if (verbose) {
			std::cout << "********************** link " << _link_it.first() << " got gradient **********************\n";
		}
	}
	return 0;
}

int MNM_Dso_Multiclass::dso_load_pathset(bool verbose)
{
	MNM_ConfReader* _tmp_conf = new MNM_ConfReader(m_file_folder + "/config.conf", "FIXED");
	Path_Table *_path_table;

	if (verbose){
		printf("loading biclass path table\n");
	}

	if (_tmp_conf -> get_string("choice_portion") == "Buffer"){
      _path_table = MNM_IO::load_path_table(m_file_folder + "/" + _tmp_conf -> get_string("path_file_name"), 
                      m_base_dta -> m_graph, _tmp_conf -> get_int("num_path"), true);
    }
    else{
      _path_table = MNM_IO::load_path_table(m_file_folder + "/" + _tmp_conf -> get_string("path_file_name"), 
                      m_base_dta -> m_graph, _tmp_conf -> get_int("num_path"), false);
    }

	delete _tmp_conf;

	m_path_table_car = _path_table;
	m_path_table_truck = _path_table;

	MNM_Origin *_orig;
	MNM_Origin_Multiclass *_orig_multiclass;

	MNM_Destination *_dest;
	MNM_Destination_Multiclass *_dest_multiclass;

	TInt _orig_node_ID, _dest_node_ID, _tot_nonzero_path;

    MNM_Pathset *_path_set_car, *_path_set_truck;

    TFlt _tot_buffer, _tmp_cost, _min_cost;

	for (auto _it : m_base_dta -> m_od_factory -> m_destination_map){
		_dest = _it.second;
		_dest_multiclass = dynamic_cast<MNM_Destination_Multiclass *>(_dest);
		_dest_node_ID = _dest -> m_dest_node -> m_node_ID;

		for (auto _map_it : m_base_dta -> m_od_factory -> m_origin_map){
			_orig = _map_it.second;
			_orig_multiclass = dynamic_cast<MNM_Origin_Multiclass *>(_orig);
			_orig_node_ID = _orig -> m_origin_node -> m_node_ID;

			if (_orig_multiclass -> m_demand_car.find(_dest_multiclass) == _orig_multiclass -> m_demand_car.end()){
				continue;
			}

			_path_set_car = MNM::get_pathset(m_path_table_car, _orig_node_ID, _dest_node_ID);

			_path_set_truck = MNM::get_pathset(m_path_table_truck, _orig_node_ID, _dest_node_ID);

			// car
			for (int _col = 0; _col < m_total_assign_inter; _col++){
				_tot_buffer = 0.0;
				
				for (auto _tmp_path : _path_set_car -> m_path_vec){
					_tot_buffer += _tmp_path -> m_buffer[_col];
				}

				// flow adjustment for car
				for (auto _tmp_path : _path_set_car -> m_path_vec){
					_tmp_path -> m_buffer[_col] = _tmp_path -> m_buffer[_col]/_tot_buffer;
				}
			}

			// truck
			for (int _col = m_total_assign_inter; _col < 2 * m_total_assign_inter; _col++){
				_tot_buffer = 0.0;
				
				// find min flow and total flow for truck
				for (auto _tmp_path : _path_set_truck -> m_path_vec){
					_tot_buffer += _tmp_path -> m_buffer[_col];
				}

				// flow adjustment for truck
				for (auto _tmp_path : _path_set_truck -> m_path_vec){
					_tmp_path -> m_buffer[_col] = _tmp_path -> m_buffer[_col]/_tot_buffer;
				}
			}
		}
	}

	return 0;
}

TFlt MNM_Dso_Multiclass::dso_get_scheduled_delay_cost(TFlt depart_time, TFlt path_travel_time, TFlt alpha, TFlt beta, TFlt gamma, TFlt delta, TInt t_star)
{
	TFlt _schedule_delay_cost;

	if (t_star - delta > depart_time + path_travel_time) // t^* - delta > t + w(t)
	{
		_schedule_delay_cost = beta * (t_star - delta - depart_time - path_travel_time); // beta * (t^* - delta - t - w(t))
	}
	else if (t_star - delta <= depart_time + path_travel_time && depart_time + path_travel_time <= t_star + delta) // t^* - delta <= t + w(t) <= t^* + delta
	{
		_schedule_delay_cost = 0.0;
	}
	else if (depart_time + path_travel_time > t_star + delta) // t + w(t) > t^* + delta
	{
		_schedule_delay_cost = gamma * (depart_time + path_travel_time - t_star - delta); // gamma * (t + w(t) - t^* - delta)
	}
	else
	{
		throw std::runtime_error("MNM_Dso_Multiclass::dso_schedule_delay_cost, unexpected condition encountered\n");
	}
	return _schedule_delay_cost;
}

int MNM_Dso_Multiclass::dso_update_path_table_departure_time_choice_msa(MNM_Dta_Multiclass *test_dta, TInt v, TFlt weight_lower, TFlt weight_upper, TInt interactive_indicator)
{
	TFlt lambda = 1.0/(1.0 + TFlt(v));
	MNM_Origin *_orig;
	MNM_Origin_Multiclass *_orig_multiclass;

	MNM_Destination *_dest;
	MNM_Destination_Multiclass *_dest_multiclass;

	TInt _orig_node_ID, _dest_node_ID;

    MNM_Pathset *_path_set_car, *_path_set_truck;
	TFlt _path_travel_time_car, _path_travel_time_truck;
	TFlt _path_scheduled_delay_cost;

    // TFlt _tot_buffer, _tmp_cost, _min_cost, _demand_car, _demand_truck, _total_demand_car, _total_demand_truck;

	TFlt _gap_numerator = 0.0;
	TFlt _gap_denominator = 0.0;

	TFlt _gap_numerator_car = 0.0;
	TFlt _gap_denominator_car = 0.0;

	TFlt _gap_numerator_truck = 0.0;
	TFlt _gap_denominator_truck = 0.0;

	TFlt _total_interval_buffer, _total_interval_demand, _disaggregated_1min_demand;

	TInt _count_shortest_path_car, _count_shortest_path_truck;

	for (auto _it : test_dta -> m_od_factory -> m_destination_map)
	{
		_dest = _it.second;
		_dest_multiclass = dynamic_cast<MNM_Destination_Multiclass *>(_dest);
		_dest_node_ID = _dest -> m_dest_node -> m_node_ID;

		for (auto _map_it : test_dta -> m_od_factory -> m_origin_map)
		{
			_orig = _map_it.second;
			_orig_multiclass = dynamic_cast<MNM_Origin_Multiclass *>(_orig);
			_orig_node_ID = _orig -> m_origin_node -> m_node_ID;

			// no such OD pair
			if (_orig_multiclass -> m_demand_car.find(_dest_multiclass) == _orig_multiclass -> m_demand_car.end()){
				continue;
			}

			_path_set_car = MNM::get_pathset(m_path_table_car, _orig_node_ID, _dest_node_ID);
			_path_set_truck = MNM::get_pathset(m_path_table_truck, _orig_node_ID, _dest_node_ID);

			/****** car class ******/
			// the following variales are for all time intervals and all paths of this OD pair (_orig, _dest)
			TFlt _total_demand_car = 0.0;
			TFlt _tot_buffer_car = 0.0;
			TFlt _min_cost_car = TFlt(std::numeric_limits<double>::max());
			TFlt _tmp_cost_car;

			// total car demand for this OD pair
			for (int _col = 0; _col < m_total_assign_inter; _col++){
				// car demand of this assign interval (15 * 1 min)
				for (int _ass_col = _col * 15; _ass_col < (_col + 1) * 15; _ass_col++){
					_total_demand_car += _orig_multiclass -> m_demand_car[_dest_multiclass][_ass_col];
				}
			}

			// find the time-dependent path with minimal cost (the cost should consider arrival penalty)
			for (int _col = 0; _col < m_total_assign_inter; _col++)
			{
				for (auto _tmp_path : _path_set_car -> m_path_vec){
					_path_travel_time_car = dso_get_path_travel_time_car(_col * m_assign_freq, _tmp_path);
					_path_scheduled_delay_cost = dso_get_scheduled_delay_cost(_col * m_assign_freq, _path_travel_time_car, 1.0, 0.5, 2.0, 540.0, 900);
					_tmp_cost_car = _path_scheduled_delay_cost;

					if (interactive_indicator == 0) {
						_tmp_cost_car += dso_get_pmc_car(_col * m_assign_freq, _tmp_path, weight_lower, weight_upper);
					}
					else {
						_tmp_cost_car += dso_get_pmc_car_interactive(_col * m_assign_freq, _tmp_path, weight_lower, weight_upper);
					}
					_tot_buffer_car += _tmp_path -> m_buffer[_col];

					if (_min_cost_car > _tmp_cost_car){
						_min_cost_car = _tmp_cost_car;
						_count_shortest_path_car = 1; // reset the shortest path count
					}
					else if (_min_cost_car == _tmp_cost_car){
						_count_shortest_path_car += 1; // increment the shortest path count
					}
				}
			}

			// gap denominator for cars
			_gap_denominator += _total_demand_car * _min_cost_car;
			_gap_denominator_car += _total_demand_car * _min_cost_car;

			// flow adjustment MSA, PMC has scheduled delay term (PMC, not LMC)
			for (int _col = 0; _col < m_total_assign_inter; _col++)
			{
				for (auto _tmp_path : _path_set_car -> m_path_vec)
				{
					_path_travel_time_car = dso_get_path_travel_time_car(_col * m_assign_freq, _tmp_path);
					_path_scheduled_delay_cost = dso_get_scheduled_delay_cost(_col * m_assign_freq, _path_travel_time_car, 1.0, 0.5, 2.0, 540.0, 900);
					_tmp_cost_car = _path_scheduled_delay_cost;
					if (interactive_indicator == 0) {
						_tmp_cost_car += dso_get_pmc_car(_col * m_assign_freq, _tmp_path, weight_lower, weight_upper); // TODO: add schedule penalty
					}
					else {
						_tmp_cost_car += dso_get_pmc_car_interactive(_col * m_assign_freq, _tmp_path, weight_lower, weight_upper); // TODO: add schedule penalty
					}

					_gap_numerator += _total_demand_car * _tmp_path -> m_buffer[_col] / _tot_buffer_car * (_tmp_cost_car - _min_cost_car);
					_gap_numerator_car += _total_demand_car * _tmp_path -> m_buffer[_col] / _tot_buffer_car * (_tmp_cost_car - _min_cost_car);

					if (_tmp_cost_car == _min_cost_car) {
						_tmp_path -> m_buffer[_col] = (1 - lambda) * _tmp_path -> m_buffer[_col]/_tot_buffer_car + lambda / TFlt(_count_shortest_path_car); // MSA, if the path is one of the shortest paths, then add lambda / count_shortest_path
					}
					else {
						_tmp_path -> m_buffer[_col] = (1 - lambda) * _tmp_path -> m_buffer[_col]/_tot_buffer_car;
					}

					// printf("Path %d, col %d, buffer %f, cost %f\n", (int)_tmp_path -> m_path_ID, (int)_col, (double)_tmp_path -> m_buffer[_col], (double)_tmp_cost_car);
				}
			}

			// demand vector adjustment and buffer normalizaton for each assignment interval
			for (int _col = 0; _col < m_total_assign_inter; _col++)
			{
				// first normalize path buffer and calculate the interval total demand.
				_total_interval_buffer = 0.0;
				_total_interval_demand = 0.0;
				for (auto _tmp_path : _path_set_car -> m_path_vec)
				{
					_total_interval_buffer += _tmp_path -> m_buffer[_col];
				}

				// calculate the total demand for this assignment interval
				_total_interval_demand = _total_interval_buffer * _total_demand_car;
				_disaggregated_1min_demand = _total_interval_demand / 15.0;

				// update m_demand_car: disaggregate the total interval demand (15min) into 15 1-min demand
				for (int _ass_col = _col * 15; _ass_col < (_col + 1) * 15; _ass_col++)
				{
					_orig_multiclass -> m_demand_car[_dest_multiclass][_ass_col] = _disaggregated_1min_demand;
				}
			}

			/****** truck class ******/
			// the following variales are used for all assignment intervals and all paths for OD pair (_orig, _dest) because we are considering both departure time and route choice.
			TFlt _total_demand_truck = 0.0;
			TFlt _tot_buffer_truck = 0.0;
			TFlt _min_cost_truck = TFlt(std::numeric_limits<double>::max());
			TFlt _tmp_cost_truck;

			// total truck demand for this OD pair
			for (int _col = 0; _col < m_total_assign_inter; _col++){
				// demand of this assignment interval (15 * 1 min)
				for (int _ass_col = _col * 15; _ass_col < (_col + 1) * 15; _ass_col++){
					_total_demand_truck += _orig_multiclass -> m_demand_truck[_dest_multiclass][_ass_col];
				}
			}

			// find the time-dependent path with minimal cost (the cost should consider arrival penalty)
			for (int _col = 0; _col < m_total_assign_inter; _col++)
			{
				for (auto _tmp_path : _path_set_truck -> m_path_vec){
					_path_travel_time_truck = dso_get_path_travel_time_truck(_col * m_assign_freq, _tmp_path);
					_path_scheduled_delay_cost = dso_get_scheduled_delay_cost(_col * m_assign_freq, _path_travel_time_truck, 1.0, 0.5, 2.0, 540.0, 900);
					_tmp_cost_truck = _path_scheduled_delay_cost;

					if (interactive_indicator == 0) {
						_tmp_cost_truck += dso_get_pmc_truck(_col * m_assign_freq, _tmp_path, weight_lower, weight_upper);
					}
					else {
						_tmp_cost_truck += dso_get_pmc_truck_interactive(_col * m_assign_freq, _tmp_path, weight_lower, weight_upper);
					}
					_tot_buffer_truck += _tmp_path -> m_buffer[_col];

					if (_min_cost_truck > _tmp_cost_truck){
						_min_cost_truck = _tmp_cost_truck;
						_count_shortest_path_truck = 1; // reset the shortest path count
					}
					else if (_min_cost_truck == _tmp_cost_truck){
						_count_shortest_path_truck += 1; // increment the shortest path count
					}
				}
			}

			// gap denominator for trucks
			_gap_denominator += _total_demand_truck * _min_cost_truck;
			_gap_denominator_truck += _total_demand_truck * _min_cost_truck;

			// flow adjustment MSA, PMC has scheduled delay term (PMC, not LMC)
			for (int _col = 0; _col < m_total_assign_inter; _col++)
			{
				for (auto _tmp_path : _path_set_truck -> m_path_vec)
				{
					_path_travel_time_truck = dso_get_path_travel_time_truck(_col * m_assign_freq, _tmp_path);
					_path_scheduled_delay_cost = dso_get_scheduled_delay_cost(_col * m_assign_freq, _path_travel_time_truck, 1.0, 0.5, 2.0, 540.0, 900);
					_tmp_cost_truck = _path_scheduled_delay_cost;

					if (interactive_indicator == 0) {
						_tmp_cost_truck += dso_get_pmc_truck(_col * m_assign_freq, _tmp_path, weight_lower, weight_upper); // TODO: add schedule penalty
					}
					else {
						_tmp_cost_truck += dso_get_pmc_truck_interactive(_col * m_assign_freq, _tmp_path, weight_lower, weight_upper); // TODO: add schedule penalty
					}

					_gap_numerator += _total_demand_truck * _tmp_path -> m_buffer[_col] / _tot_buffer_truck * (_tmp_cost_truck - _min_cost_truck);
					_gap_numerator_truck += _total_demand_truck * _tmp_path -> m_buffer[_col] / _tot_buffer_truck * (_tmp_cost_truck - _min_cost_truck);

					if (_tmp_cost_truck == _min_cost_truck) {
						_tmp_path -> m_buffer[_col] = (1 - lambda) * _tmp_path -> m_buffer[_col]/_tot_buffer_truck + lambda / TFlt(_count_shortest_path_truck); // MSA, if the path is one of the shortest paths, then add lambda / count_shortest_path
					}
					else {
						_tmp_path -> m_buffer[_col] = (1 - lambda) * _tmp_path -> m_buffer[_col]/_tot_buffer_truck;
					}

					// printf("Path %d, col %d, buffer %f, cost %f\n", (int)_tmp_path -> m_path_ID, (int)_col, (double)_tmp_path -> m_buffer[_col], (double)_tmp_cost_truck);
				}
			} 

			// demand vector adjustment and buffer normalizaton for each assignment interval
			for (int _col = 0; _col < m_total_assign_inter; _col++)
			{
				// first normalize path buffer and calculate the interval total demand.
				_total_interval_buffer = 0.0;
				_total_interval_demand = 0.0;
				for (auto _tmp_path : _path_set_truck -> m_path_vec)
				{
					_total_interval_buffer += _tmp_path -> m_buffer[_col];
				}

				// calculate the total demand for this assignment interval
				_total_interval_demand = _total_interval_buffer * _total_demand_truck;
				_disaggregated_1min_demand = _total_interval_demand / 15.0;

				// update m_demand_truck: disaggregate the total interval demand (15min) into 15 1-min demand
				for (int _ass_col = _col * 15; _ass_col < (_col + 1) * 15; _ass_col++)
				{
					_orig_multiclass -> m_demand_truck[_dest_multiclass][_ass_col] = _disaggregated_1min_demand;
				}
			}
		}
	}

	m_gap_function_value = _gap_numerator / _gap_denominator;

	m_gap_function_value_car = _gap_numerator_car / _gap_denominator_car;
	m_gap_function_value_truck = _gap_numerator_truck / _gap_denominator_truck;

	return 0;
}

int MNM_Dso_Multiclass::dso_update_path_table(MNM_Dta_Multiclass *test_dta, TInt v, TFlt weight_lower, TFlt weight_upper, TInt interactive_indicator)
{
	TFlt lambda = 1.0/(1.0 + TFlt(v));
	MNM_Origin *_orig;
	MNM_Origin_Multiclass *_orig_multiclass;

	MNM_Destination *_dest;
	MNM_Destination_Multiclass *_dest_multiclass;

	TInt _orig_node_ID, _dest_node_ID;

    MNM_Pathset *_path_set_car, *_path_set_truck;

    TFlt _tot_buffer, _tmp_cost, _min_cost, _demand_car, _demand_truck;

	TFlt _gap_numerator = 0.0;
	TFlt _gap_denominator = 0.0;

	for (auto _it : test_dta -> m_od_factory -> m_destination_map){
		_dest = _it.second;
		_dest_multiclass = dynamic_cast<MNM_Destination_Multiclass *>(_dest);
		_dest_node_ID = _dest -> m_dest_node -> m_node_ID;

		for (auto _map_it : test_dta -> m_od_factory -> m_origin_map){
			_orig = _map_it.second;
			_orig_multiclass = dynamic_cast<MNM_Origin_Multiclass *>(_orig);
			_orig_node_ID = _orig -> m_origin_node -> m_node_ID;

			if (_orig_multiclass -> m_demand_car.find(_dest_multiclass) == _orig_multiclass -> m_demand_car.end()){
				continue;
			}

			_path_set_car = MNM::get_pathset(m_path_table_car, _orig_node_ID, _dest_node_ID);
			_path_set_truck = MNM::get_pathset(m_path_table_truck, _orig_node_ID, _dest_node_ID);

			// car
			for (int _col = 0; _col < m_total_assign_inter; _col++){
				// car demand of this assign interval (15 * 1 min)
				_demand_car = 0.0;
				for (int _ass_col = _col * 15; _ass_col < (_col + 1) * 15; _ass_col++){
					_demand_car += _orig_multiclass -> m_demand_car[_dest_multiclass][_ass_col];
				}

				_tot_buffer = 0.0;
				_min_cost = TFlt(std::numeric_limits<double>::max());

				// find min flow and total flow for car
				for (auto _tmp_path : _path_set_car -> m_path_vec){
					if (interactive_indicator == 0) {
						_tmp_cost = dso_get_pmc_car(_col * m_assign_freq, _tmp_path, weight_lower, weight_upper);
					}
					else {
						_tmp_cost = dso_get_pmc_car_interactive(_col * m_assign_freq, _tmp_path, weight_lower, weight_upper);
					}
					_tot_buffer += _tmp_path -> m_buffer[_col];

					if (_min_cost > _tmp_cost){
						_min_cost = _tmp_cost;
					}
				}

				_gap_denominator += _demand_car * _min_cost;

				// flow adjustment for car
				for (auto _tmp_path : _path_set_car -> m_path_vec){
					if (interactive_indicator == 0) {
						_tmp_cost = dso_get_pmc_car(_col * m_assign_freq, _tmp_path, weight_lower, weight_upper);
					}
					else {
						_tmp_cost = dso_get_pmc_car_interactive(_col * m_assign_freq, _tmp_path, weight_lower, weight_upper);
					}
					_gap_numerator += _demand_car * _tmp_path -> m_buffer[_col] / _tot_buffer * (_tmp_cost - _min_cost);

					if (_tmp_cost == _min_cost){
						_tmp_path -> m_buffer[_col] = (1 - lambda) * _tmp_path -> m_buffer[_col]/_tot_buffer + lambda;
					}
					else{
						_tmp_path -> m_buffer[_col] = (1 - lambda) * _tmp_path -> m_buffer[_col]/_tot_buffer;
					}
				}
			}

			// truck
			for (int _col = m_total_assign_inter; _col < 2 * m_total_assign_inter; _col++){
				// truck demand of this assign interval (15 * 1 min)
				_demand_truck = 0.0;
				for (int _ass_col = (_col - m_total_assign_inter) * 15; _ass_col < (_col - m_total_assign_inter + 1) * 15; _ass_col++){
					_demand_truck += _orig_multiclass -> m_demand_truck[_dest_multiclass][_ass_col];
				}

				_tot_buffer = 0.0;
				_min_cost = TFlt(std::numeric_limits<double>::max());

				// find min flow and total flow for truck
				for (auto _tmp_path : _path_set_truck -> m_path_vec){
					if (interactive_indicator == 0){
						_tmp_cost = dso_get_pmc_truck((_col - m_total_assign_inter) * m_assign_freq, _tmp_path, weight_lower, weight_upper);
					}
					else {
						_tmp_cost = dso_get_pmc_truck_interactive((_col - m_total_assign_inter) * m_assign_freq, _tmp_path, weight_lower, weight_upper);
					}
					_tot_buffer += _tmp_path -> m_buffer[_col];

					if (_min_cost > _tmp_cost){
						_min_cost = _tmp_cost;
					}	
				}

				_gap_denominator += _demand_truck * _min_cost;

				// flow adjustment for truck
				for (auto _tmp_path : _path_set_truck -> m_path_vec){
					if (interactive_indicator == 0){
						_tmp_cost = dso_get_pmc_truck((_col - m_total_assign_inter) * m_assign_freq, _tmp_path, weight_lower, weight_upper);
					}
					else {
						_tmp_cost = dso_get_pmc_truck_interactive((_col - m_total_assign_inter) * m_assign_freq, _tmp_path, weight_lower, weight_upper);
					}
					_gap_numerator += _demand_truck * _tmp_path -> m_buffer[_col] / _tot_buffer * (_tmp_cost - _min_cost);

					if (_tmp_cost == _min_cost){
						_tmp_path -> m_buffer[_col] = (1 - lambda) * _tmp_path -> m_buffer[_col]/_tot_buffer + lambda;
					}
					else{
						_tmp_path -> m_buffer[_col] = (1 - lambda) * _tmp_path -> m_buffer[_col]/_tot_buffer;
					}
				}
			}
		}
	}

	m_gap_function_value = _gap_numerator / _gap_denominator;

	return 0;
}

TFlt MNM_Dso_Multiclass::dso_get_path_travel_time_car(TInt depart_time, MNM_Path *path)
{
	// true path tt
    TInt _cur_time = depart_time;
    TInt _query_time;
    for (TInt _link_ID : path->m_link_vec) {
        _query_time = MNM_Ults::min(TInt(_cur_time), TInt(m_total_loading_inter - 1));
        _cur_time += m_link_tt_map_car[_link_ID][_query_time];
    }
    return _cur_time - depart_time;
}

TFlt MNM_Dso_Multiclass::dso_get_path_travel_time_truck(TInt depart_time, MNM_Path *path)
{
	// true path tt
    TInt _cur_time = depart_time;
    TInt _query_time;
    for (TInt _link_ID : path->m_link_vec) {
        _query_time = MNM_Ults::min(TInt(_cur_time), TInt(m_total_loading_inter - 1));
        _cur_time += m_link_tt_map_truck[_link_ID][_query_time];
    }
    return _cur_time - depart_time;
}

TFlt MNM_Dso_Multiclass::get_total_travel_cost(MNM_Dta_Multiclass *test_dta)
{
	MNM_Origin *_orig;
	MNM_Origin_Multiclass *_orig_multiclass;

    MNM_Destination *_dest;
	MNM_Destination_Multiclass *_dest_multiclass;

	TInt _orig_node_ID, _dest_node_ID;
	
	MNM_Pathset *_path_set_car, *_path_set_truck;

	TFlt _tot_travel_cost = 0.0;

	TFlt _demand_car, _demand_truck;

	TFlt _tot_buffer, _tmp_cost;

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

			_path_set_car = MNM::get_pathset(m_path_table_car, _orig_node_ID, _dest_node_ID);

			_path_set_truck = MNM::get_pathset(m_path_table_truck, _orig_node_ID, _dest_node_ID);

			// car
			for (int _col = 0; _col < m_total_assign_inter; _col++){
				// car demand of this assign interval (12 inter)
				_demand_car = 0.0;
				for (int _ass_col = _col * 15; _ass_col < (_col + 1) * 15; _ass_col++){
					_demand_car += _orig_multiclass -> m_demand_car[_dest_multiclass][_ass_col];
				}

				_tot_buffer = 0.0;

				// calculate total buffer
				for (auto _tmp_path : _path_set_car -> m_path_vec){
					_tot_buffer += _tmp_path -> m_buffer[_col];
				}

				// buffer fraction * travel cost
				for (auto _tmp_path : _path_set_car -> m_path_vec){
					_tmp_cost = dso_get_path_travel_time_car(_col * m_assign_freq, _tmp_path);
					_tot_travel_cost += _demand_car * _tmp_cost * _tmp_path -> m_buffer[_col] / _tot_buffer;
				}
			}

			// truck
			for (int _col = 0; _col < m_total_assign_inter; _col++){
				// truck demand of this assign interval (12 inter)
				_demand_truck = 0.0;
				for (int _ass_col = _col * 15; _ass_col < (_col + 1) * 15; _ass_col++){
					_demand_truck += _orig_multiclass -> m_demand_truck[_dest_multiclass][_ass_col];
				}

				_tot_buffer = 0.0;

				// calculate total buffer
				for (auto _tmp_path : _path_set_truck -> m_path_vec){
					_tot_buffer += _tmp_path -> m_buffer[_col + m_total_assign_inter];
				}

				// buffer fraction * travel cost
				for (auto _tmp_path : _path_set_truck -> m_path_vec){
					_tmp_cost = dso_get_path_travel_time_truck(_col * m_assign_freq, _tmp_path);
					_tot_travel_cost += _demand_truck * _tmp_cost * _tmp_path -> m_buffer[_col + m_total_assign_inter] / _tot_buffer;
				}
			}
		}
	}

	return _tot_travel_cost;
}

TFlt MNM_Dso_Multiclass::get_car_total_travel_cost(MNM_Dta_Multiclass *test_dta)
{
	MNM_Origin *_orig;
	MNM_Origin_Multiclass *_orig_multiclass;
    MNM_Destination *_dest;
	MNM_Destination_Multiclass *_dest_multiclass;
	TInt _orig_node_ID, _dest_node_ID;
	MNM_Pathset *_path_set_car;
	TFlt _tot_travel_cost = 0.0;
	TFlt _demand_car;
	TFlt _tot_buffer, _tmp_cost;

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

			_path_set_car = MNM::get_pathset(m_path_table_car, _orig_node_ID, _dest_node_ID);

			// car
			for (int _col = 0; _col < m_total_assign_inter; _col++){
				// car demand of this assign interval (12 inter)
				_demand_car = 0.0;
				for (int _ass_col = _col * 15; _ass_col < (_col + 1) * 15; _ass_col++){
					_demand_car += _orig_multiclass -> m_demand_car[_dest_multiclass][_ass_col];
				}
				
				// calculate total buffer
				_tot_buffer = 0.0;
				for (auto _tmp_path : _path_set_car -> m_path_vec){
					_tot_buffer += _tmp_path -> m_buffer[_col];
				}

				// buffer fraction * travel cost
				for (auto _tmp_path : _path_set_car -> m_path_vec){
					_tmp_cost = dso_get_path_travel_time_car(_col * m_assign_freq, _tmp_path);
					_tot_travel_cost += _demand_car * _tmp_cost * _tmp_path -> m_buffer[_col] / _tot_buffer;
				}
			}
		}
	}

	return _tot_travel_cost;
}

TFlt MNM_Dso_Multiclass::get_truck_total_travel_cost(MNM_Dta_Multiclass *test_dta)
{
	MNM_Origin *_orig;
	MNM_Origin_Multiclass *_orig_multiclass;
    MNM_Destination *_dest;
	MNM_Destination_Multiclass *_dest_multiclass;
	TInt _orig_node_ID, _dest_node_ID;
	MNM_Pathset *_path_set_truck;
	TFlt _tot_travel_cost = 0.0;
	TFlt _demand_truck;
	TFlt _tot_buffer, _tmp_cost;

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

			_path_set_truck = MNM::get_pathset(m_path_table_truck, _orig_node_ID, _dest_node_ID);

			// truck
			for (int _col = 0; _col < m_total_assign_inter; _col++){
				// truck demand of this assign interval (12 inter)
				_demand_truck = 0.0;
				for (int _ass_col = _col * 15; _ass_col < (_col + 1) * 15; _ass_col++){
					_demand_truck += _orig_multiclass -> m_demand_truck[_dest_multiclass][_ass_col];
				}

				// calculate total buffer
				_tot_buffer = 0.0;
				for (auto _tmp_path : _path_set_truck -> m_path_vec){
					_tot_buffer += _tmp_path -> m_buffer[_col + m_total_assign_inter];
				}

				// buffer fraction * travel cost
				for (auto _tmp_path : _path_set_truck -> m_path_vec){
					_tmp_cost = dso_get_path_travel_time_truck(_col * m_assign_freq, _tmp_path);
					_tot_travel_cost += _demand_truck * _tmp_cost * _tmp_path -> m_buffer[_col + m_total_assign_inter] / _tot_buffer;
				}
			}
		}
	}

	return _tot_travel_cost;
}

TFlt MNM_Dso_Multiclass::get_car_total_travel_cost_with_scheduled_delay(MNM_Dta_Multiclass *test_dta)
{
	TFlt _total_travel_cost = 0.0;

	m_total_sdc_car = 0.0;
	m_total_ttc_car = 0.0;

	MNM_Origin *_orig;
	MNM_Origin_Multiclass *_orig_multiclass;
    MNM_Destination *_dest;
	MNM_Destination_Multiclass *_dest_multiclass;
	TInt _orig_node_ID, _dest_node_ID;
	
	MNM_Pathset *_path_set_car;
	TFlt _demand_car;
	TFlt _tot_buffer, _tmp_cost, _path_travel_time_car, _path_scheduled_delay_cost;

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

			_path_set_car = MNM::get_pathset(m_path_table_car, _orig_node_ID, _dest_node_ID);

			// car
			for (int _col = 0; _col < m_total_assign_inter; _col++){
				// car demand of this assign interval (12 inter)
				_demand_car = 0.0;
				for (int _ass_col = _col * 15; _ass_col < (_col + 1) * 15; _ass_col++){
					_demand_car += _orig_multiclass -> m_demand_car[_dest_multiclass][_ass_col];
				}
				
				// calculate total buffer
				_tot_buffer = 0.0;
				for (auto _tmp_path : _path_set_car -> m_path_vec){
					_tot_buffer += _tmp_path -> m_buffer[_col];
				}

				// buffer fraction * travel cost
				for (auto _tmp_path : _path_set_car -> m_path_vec){
					_path_travel_time_car = dso_get_path_travel_time_car(_col * m_assign_freq, _tmp_path);
					_path_scheduled_delay_cost = dso_get_scheduled_delay_cost(_col * m_assign_freq, _path_travel_time_car, 1.0, 0.5, 2.0, 540.0, 900);

					_tmp_cost = _path_travel_time_car + _path_scheduled_delay_cost;
					_total_travel_cost += _demand_car * _tmp_cost * _tmp_path -> m_buffer[_col] / _tot_buffer;

					m_total_sdc_car += _demand_car * _path_scheduled_delay_cost * _tmp_path -> m_buffer[_col] / _tot_buffer;
					m_total_ttc_car += _demand_car * _path_travel_time_car * _tmp_path -> m_buffer[_col] / _tot_buffer;
				}
			}
		}
	}

	return _total_travel_cost;
}

TFlt MNM_Dso_Multiclass::get_truck_total_travel_cost_with_scheduled_delay(MNM_Dta_Multiclass *test_dta)
{
	TFlt _total_travel_cost = 0.0;

	m_total_sdc_truck = 0.0;
	m_total_ttc_truck = 0.0;

	MNM_Origin *_orig;
	MNM_Origin_Multiclass *_orig_multiclass;
    MNM_Destination *_dest;
	MNM_Destination_Multiclass *_dest_multiclass;
	TInt _orig_node_ID, _dest_node_ID;

	MNM_Pathset *_path_set_truck;
	TFlt _demand_truck;
	TFlt _tot_buffer, _tmp_cost, _path_travel_time_truck, _path_scheduled_delay_cost;

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

			_path_set_truck = MNM::get_pathset(m_path_table_truck, _orig_node_ID, _dest_node_ID);

			// truck
			for (int _col = 0; _col < m_total_assign_inter; _col++){
				// truck demand of this assign interval (12 inter)
				_demand_truck = 0.0;
				for (int _ass_col = _col * 15; _ass_col < (_col + 1) * 15; _ass_col++){
					_demand_truck += _orig_multiclass -> m_demand_truck[_dest_multiclass][_ass_col];
				}

				// calculate total buffer
				_tot_buffer = 0.0;
				for (auto _tmp_path : _path_set_truck -> m_path_vec){
					_tot_buffer += _tmp_path -> m_buffer[_col + m_total_assign_inter];
				}

				// buffer fraction * travel cost
				for (auto _tmp_path : _path_set_truck -> m_path_vec){
					_path_travel_time_truck = dso_get_path_travel_time_truck(_col * m_assign_freq, _tmp_path);
					_path_scheduled_delay_cost = dso_get_scheduled_delay_cost(_col * m_assign_freq, _path_travel_time_truck, 1.0, 0.5, 2.0, 540.0, 900);

					_tmp_cost = _path_travel_time_truck + _path_scheduled_delay_cost;
					_total_travel_cost += _demand_truck * _tmp_cost * _tmp_path -> m_buffer[_col + m_total_assign_inter] / _tot_buffer;

					m_total_sdc_truck += _demand_truck * _path_scheduled_delay_cost * _tmp_path -> m_buffer[_col + m_total_assign_inter] / _tot_buffer;
					m_total_ttc_truck += _demand_truck * _path_travel_time_truck * _tmp_path -> m_buffer[_col + m_total_assign_inter] / _tot_buffer;
				}
			}
		}
	}

	return _total_travel_cost;
}


TFlt MNM_Dso_Multiclass::get_gap_function_value()
{
	return m_gap_function_value;
}

int MNM_Dso_Multiclass::dso_print_pathbuffer_with_pmc(MNM_Dta_Multiclass *test_dta, TFlt weight_lower, TFlt weight_upper)
{
	MNM_Origin *_orig;
	MNM_Origin_Multiclass *_orig_multiclass;

    MNM_Destination *_dest;
	MNM_Destination_Multiclass *_dest_multiclass;

	TInt _orig_node_ID, _dest_node_ID;
	
	MNM_Pathset *_path_set_car, *_path_set_truck;

	std::string _buffer_car, _buffer_truck;

	TFlt _pmc_car, _pmc_truck;

	std::cout << "car" << "\n";

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

			_path_set_car = MNM::get_pathset(m_path_table_car, _orig_node_ID, _dest_node_ID);

			_path_set_truck = MNM::get_pathset(m_path_table_truck, _orig_node_ID, _dest_node_ID);

			for (auto _tmp_path : _path_set_car -> m_path_vec){
				_buffer_car = std::to_string(_orig_node_ID) + " " + std::to_string(_dest_node_ID) + " ";
				for (int _col = 0; _col < m_total_assign_inter; _col++){
					_pmc_car = int(dso_get_pmc_car(_col * m_assign_freq, _tmp_path, weight_lower, weight_upper));
					_buffer_car += std::to_string(_tmp_path -> m_buffer[_col]) + "(" + std::to_string(int(_pmc_car)) + ") ";
				}
				std::cout << _buffer_car << "\n";
			}
			
		}
	}

	std::cout << "truck" << "\n";
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

			_path_set_truck = MNM::get_pathset(m_path_table_truck, _orig_node_ID, _dest_node_ID);

			for (auto _tmp_path : _path_set_truck -> m_path_vec){
				_buffer_truck = std::to_string(_orig_node_ID) + " " + std::to_string(_dest_node_ID) + " ";
				for (int _col = 0; _col < m_total_assign_inter; _col++){
					_pmc_truck = int(dso_get_pmc_truck(_col * m_assign_freq, _tmp_path, weight_lower, weight_upper));
					_buffer_truck += std::to_string(_tmp_path -> m_buffer[_col + m_total_assign_inter]) + "(" + std::to_string(int(_pmc_truck)) + ") ";
				}
				std::cout << _buffer_truck << "\n";
			}
			
		}
	}
	return 0;
}

int MNM_Dso_Multiclass::save_dso_path_buffer(MNM_Dta_Multiclass *test_dta, std::string save_folder)
{
	MNM_Origin *_orig;
	MNM_Origin_Multiclass *_orig_multiclass;

    MNM_Destination *_dest;
	MNM_Destination_Multiclass *_dest_multiclass;

	TInt _orig_node_ID, _dest_node_ID;
	
	MNM_Pathset *_path_set_car, *_path_set_truck;

	std::string _buffer_car, _buffer_truck;

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

			_path_set_car = MNM::get_pathset(m_path_table_car, _orig_node_ID, _dest_node_ID);

			_path_set_truck = MNM::get_pathset(m_path_table_truck, _orig_node_ID, _dest_node_ID);

			for (auto _tmp_path : _path_set_car -> m_path_vec){
				for (int _col = 0; _col < m_total_assign_inter; _col++){
					_buffer_car += std::to_string(_tmp_path -> m_buffer[_col]) + " ";
				}
				_buffer_car += "\n";
			}

			for (auto _tmp_path : _path_set_truck -> m_path_vec){
				for (int _col = 0; _col < m_total_assign_inter; _col++){
					_buffer_truck += std::to_string(_tmp_path -> m_buffer[_col]) + " ";
				}
				_buffer_truck += "\n";
			}
		}
	}
	return 0;
}

int MNM_Dso_Multiclass::run_dso(bool verbose, std::string folder, int v_start, TFlt weight_lower, TFlt weight_upper, TInt interactive_indicator)
{
	std::string gap_file_name = folder + "/record/v2_gap_iteration_" + std::to_string(weight_lower).substr(0,3) + "_" + std::to_string(weight_upper).substr(0,3) + ".txt";
    std::ofstream gap_file;
    gap_file.open(gap_file_name, std::ofstream::out);
    if (!gap_file.is_open()){
        printf("Error happens when open gap_file\n");
        exit(-1);
    }
	initialize(verbose);
	MNM_Dta_Multiclass *test_dta;
	dso_load_pathset(verbose);
	for (int v = v_start; v < m_max_iter + v_start; v++){
		test_dta = dnl(verbose);
		build_link_cost_map(test_dta, verbose);
		get_link_marginal_cost_v2(test_dta, verbose);

		TFlt total_cost_car;
		TFlt total_cost_truck;
		// total_cost = get_total_travel_cost(test_dta);
		total_cost_car = get_car_total_travel_cost(test_dta);
		total_cost_truck = get_truck_total_travel_cost(test_dta);

		// dso_print_pathbuffer_with_pmc(test_dta, weight_lower, weight_upper);
		dso_update_path_table(test_dta, TInt(v), weight_lower, weight_upper, interactive_indicator);
		std::cout << "***** car travel cost = " << total_cost_car << " ***** truck travel cost = " << total_cost_truck << " ***** gap = " << m_gap_function_value << " ***** iter = " << (v - v_start) << " *****\n";
		gap_file << std::to_string(total_cost_car) + " " + std::to_string(total_cost_truck) + " " + std::to_string(m_gap_function_value) + "\n";
	}
	gap_file.close();
	return 0;
}

int MNM_Dso_Multiclass::run_dso_departure_time_choice_msa(bool verbose, std::string folder, int v_start, TFlt weight_lower, TFlt weight_upper, TInt interactive_indicator, std::string save_name, std::string pmc_version)
{
	std::string inter_ind;

	if (interactive_indicator == 0){
		inter_ind = "non_interactive";
	}
	else if (interactive_indicator == 1){
		inter_ind = "interactive";
	}
	else{
		std::cout << "Error: unknown interactive indicator: " << interactive_indicator << "\n";
		exit(-1);
	}

	std::string gap_file_name = folder + "/record/" + save_name + "_" + pmc_version + "_" + inter_ind + "_gap_iteration_" + std::to_string(weight_lower).substr(0,3) + "_" + std::to_string(weight_upper).substr(0,3) + ".txt";
    std::ofstream gap_file;
    gap_file.open(gap_file_name, std::ofstream::out);

    if (!gap_file.is_open()){
        printf("Error happens when open gap_file\n");
        exit(-1);
    }

	gap_file << "total_sdc_car total_sdc_truck total_ttc_car total_ttc_truck gap_car gap_truck total_gap\n";

	initialize(verbose);
	MNM_Dta_Multiclass *test_dta;
	dso_load_pathset(verbose);

	for (int v = v_start; v < m_max_iter + v_start; v++){
		test_dta = dnl(verbose);
		build_link_cost_map(test_dta, verbose);
		if (pmc_version == "v1"){
			get_link_marginal_cost_v1(test_dta, verbose);
		}
		else if (pmc_version == "v2"){
			get_link_marginal_cost_v2(test_dta, verbose);
		}
		else{
			std::cout << "Error: unknown pmc version: " << pmc_version << "\n";
		}

		TFlt total_cost_car;
		TFlt total_cost_truck;
		total_cost_car = get_car_total_travel_cost_with_scheduled_delay(test_dta);
		total_cost_truck = get_truck_total_travel_cost_with_scheduled_delay(test_dta);

		dso_update_path_table_departure_time_choice_msa(test_dta, TInt(v), weight_lower, weight_upper, interactive_indicator);
		std::cout << "***** car travel cost = " << total_cost_car << " ***** truck travel cost = " << total_cost_truck << " ***** gap = " << m_gap_function_value << " ***** iter = " << (v - v_start) << " *****\n";
		gap_file << std::to_string(m_total_sdc_car) + " " + 
					std::to_string(m_total_sdc_truck) + " " + 
					std::to_string(m_total_ttc_car) + " " +
					std::to_string(m_total_ttc_truck) + " " +
					std::to_string(m_gap_function_value_car) + " " +
					std::to_string(m_gap_function_value_truck) + " " +
					std::to_string(m_gap_function_value) + "\n";
	}

	return 0;
}
