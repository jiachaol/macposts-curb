#include "limits.h"
#include "multiclass.h"
#include "multiclass_curb.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <time.h>
#include <random>

/*************************************************************************
					Multiclass link model for curb modeling
		1. have curb-related cumulative curves and trees installed		
**************************************************************************/

MNM_Dlink_Multiclass_Curb::MNM_Dlink_Multiclass_Curb(TInt ID,
										TInt number_of_lane,
										TFlt length, // meters
										TFlt ffs_car, // Free-flow speed (m/s)
										TFlt ffs_truck)
	: MNM_Dlink_Multiclass::MNM_Dlink_Multiclass(ID, number_of_lane, length, ffs_car, ffs_truck) // Note: although m_ffs is not used in child class, let it be ffs_car
{
	m_ffs_car = ffs_car;
	m_ffs_truck = ffs_truck;

	// cumulative curve for non-stopping vehicles
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

	install_cumulative_curve_multiclass();
}

MNM_Dlink_Multiclass_Curb::~MNM_Dlink_Multiclass_Curb()
{
	if (m_N_in_car != nullptr) delete m_N_in_car;
	if (m_N_out_car != nullptr) delete m_N_out_car;
  	
	if (m_N_in_truck != nullptr) delete m_N_in_truck;
  	if (m_N_out_truck != nullptr) delete m_N_out_truck;
  	
	if (m_N_in_rh != nullptr) delete m_N_in_rh;
	if (m_N_out_rh != nullptr) delete m_N_out_rh;

	if (m_N_in_car_cc != nullptr) delete m_N_in_car_cc;
	if (m_N_out_car_cc != nullptr) delete m_N_out_car_cc;
  	
	if (m_N_in_truck_cc != nullptr) delete m_N_in_truck_cc;
  	if (m_N_out_truck_cc != nullptr) delete m_N_out_truck_cc;
  	
	if (m_N_in_rh_cc != nullptr) delete m_N_in_rh_cc;
	if (m_N_out_rh_cc != nullptr) delete m_N_out_rh_cc;
	
	if (m_N_in_car_all != nullptr) delete m_N_in_car_all;
	if (m_N_out_car_all != nullptr) delete m_N_out_car_all;
  	
	if (m_N_in_tree_car != nullptr) delete m_N_in_tree_car;
  	if (m_N_out_tree_car != nullptr) delete m_N_out_tree_car;
  	
	if (m_N_in_tree_truck != nullptr) delete m_N_in_tree_truck;
  	if (m_N_out_tree_truck != nullptr) delete m_N_out_tree_truck;
  	
	if (m_N_in_tree_rh != nullptr) delete m_N_in_tree_rh;
	if (m_N_out_tree_rh != nullptr) delete m_N_out_tree_rh;

	if (m_N_in_tree_curb_car != nullptr) delete m_N_in_tree_curb_car;
	if (m_N_out_tree_curb_car != nullptr) delete m_N_out_tree_curb_car;

	if (m_N_in_tree_curb_truck != nullptr) delete m_N_in_tree_curb_truck;
	if (m_N_out_tree_curb_truck != nullptr) delete m_N_out_tree_curb_truck;

	if (m_N_in_tree_curb_rh != nullptr) delete m_N_in_tree_curb_rh;
	if (m_N_out_tree_curb_rh != nullptr) delete m_N_out_tree_curb_rh;	
}

int MNM_Dlink_Multiclass_Curb::install_cumulative_curve_multiclass()
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

int MNM_Dlink_Multiclass_Curb::install_cumulative_curve_tree_multiclass()
{
	if (m_N_out_tree_car != nullptr) delete m_N_out_tree_car;
  	if (m_N_in_tree_car != nullptr) delete m_N_in_tree_car;

  	if (m_N_out_tree_truck != nullptr) delete m_N_out_tree_truck;
  	if (m_N_in_tree_truck != nullptr) delete m_N_in_tree_truck;

	if (m_N_in_tree_rh != nullptr) delete m_N_in_tree_rh;
	if (m_N_out_tree_rh != nullptr) delete m_N_out_tree_rh;

	m_N_in_tree_car = new MNM_Tree_Cumulative_Curve();
  	m_N_out_tree_car = new MNM_Tree_Cumulative_Curve();

	m_N_in_tree_truck = new MNM_Tree_Cumulative_Curve();
  	m_N_out_tree_truck = new MNM_Tree_Cumulative_Curve();

	m_N_in_tree_rh = new MNM_Tree_Cumulative_Curve();
	m_N_out_tree_rh = new MNM_Tree_Cumulative_Curve();

	return 0;
}

int MNM_Dlink_Multiclass_Curb::install_cumulative_curve_tree_multiclass_curb()
{
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

TFlt MNM_Dlink_Multiclass_Curb::get_link_freeflow_tt_car()
{
	return m_length/m_ffs_car;  // seconds, absolute tt
}

TFlt MNM_Dlink_Multiclass_Curb::get_link_freeflow_tt_truck()
{
	return m_length/m_ffs_truck;  // seconds, absolute tt
}

/*************************************************************************
					Multiclass CTM model for curb modeling
**************************************************************************/
MNM_Dlink_Ctm_Multiclass_Curb::MNM_Dlink_Ctm_Multiclass_Curb(TInt ID,
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
	: MNM_Dlink_Multiclass_Curb::MNM_Dlink_Multiclass_Curb(ID, number_of_lane, length, ffs_car, ffs_truck)
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
	// m_curb_cell_array = std::vector<Cell_Curb_Multiclass*>();

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

	// m_lane_rho_1_N > m_lane_critical_density_car and m_lane_critical_density_truck
	m_lane_rho_1_N = m_lane_hold_cap_car * (m_wave_speed_car / (m_ffs_truck + m_wave_speed_car));

	init_cell_array(unit_time, _std_cell_length, _last_cell_length);

	m_curb_cell_array = std::vector<Cell_Curb_Multiclass*>();

	install_curb();

	// // PMC
	// m_congested_car = int(0);
	// m_congested_truck = int(0);

	// m_diff_car = true;
	// m_diff_truck = true;
}

MNM_Dlink_Ctm_Multiclass_Curb::~MNM_Dlink_Ctm_Multiclass_Curb()
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

TFlt MNM_Dlink_Ctm_Multiclass_Curb::get_link_supply()
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

TFlt MNM_Dlink_Ctm_Multiclass_Curb::get_link_capacity()
{
	return m_lane_flow_cap_car * TFlt(m_number_of_lane) * m_unit_time;
}
	
int MNM_Dlink_Ctm_Multiclass_Curb::clear_incoming_array(TInt timestamp, TFlt _ratio_lane_closure)
{
	MNM_Veh_Multiclass_Curb* _veh;
	size_t _cur_size = m_incoming_array.size();
	for (size_t i = 0; i < _cur_size; ++i) {
		_veh = dynamic_cast<MNM_Veh_Multiclass_Curb *>(m_incoming_array.front());
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

TFlt MNM_Dlink_Ctm_Multiclass_Curb::get_link_flow_car()
{
	TInt _total_volume_car = 0;
	for (int i = 0; i < m_num_cells; ++i){
		_total_volume_car += m_cell_array[i] -> m_volume_car;
	}
	std::deque<MNM_Veh*>::iterator _veh_it;
	for (_veh_it = m_finished_array.begin(); _veh_it != m_finished_array.end(); _veh_it++){
		MNM_Veh_Multiclass_Curb *_veh = dynamic_cast<MNM_Veh_Multiclass_Curb *>(*_veh_it);
		if (_veh -> m_class == 0) _total_volume_car += 1;
	}
	return TFlt(_total_volume_car) / m_flow_scalar;
}

TFlt MNM_Dlink_Ctm_Multiclass_Curb::get_link_flow_truck()
{
	TInt _total_volume_truck = 0;
	for (int i = 0; i < m_num_cells; ++i){
		_total_volume_truck += m_cell_array[i] -> m_volume_truck;
	}
	std::deque<MNM_Veh*>::iterator _veh_it;
	for (_veh_it = m_finished_array.begin(); _veh_it != m_finished_array.end(); _veh_it++){
		MNM_Veh_Multiclass_Curb *_veh = dynamic_cast<MNM_Veh_Multiclass_Curb *>(*_veh_it);
		if (_veh -> m_class == 1) _total_volume_truck += 1;
	}
	return TFlt(_total_volume_truck) / m_flow_scalar;
}

TFlt MNM_Dlink_Ctm_Multiclass_Curb::get_link_flow()
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

TFlt MNM_Dlink_Ctm_Multiclass_Curb::get_link_tt()
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

TFlt MNM_Dlink_Ctm_Multiclass_Curb::get_link_tt_from_flow_car(TFlt flow)
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

TFlt MNM_Dlink_Ctm_Multiclass_Curb::get_link_tt_from_flow_truck(TFlt flow)
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

TInt MNM_Dlink_Ctm_Multiclass_Curb::get_link_freeflow_tt_loading_car()
{
	return m_num_cells;
}

TInt MNM_Dlink_Ctm_Multiclass_Curb::get_link_freeflow_tt_loading_truck()
{
	// due the random rounding, this is a random number
	return m_num_cells * m_ffs_car / m_ffs_truck;
}

int MNM_Dlink_Ctm_Multiclass_Curb::init_cell_array(TFlt unit_time, TFlt std_cell_length, TFlt last_cell_length)
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

int MNM_Dlink_Ctm_Multiclass_Curb::update_out_veh_curb(std::vector<TFlt> _ratio_lane_closure_list)
{
	TFlt _temp_out_flux_car, _supply_car, _demand_car;
	TFlt _temp_out_flux_truck, _supply_truck, _demand_truck;

	if ((m_num_cells > 1) && (int(_ratio_lane_closure_list.size()) == m_num_cells))
	{
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
	else 
	{
		printf("Error in update_out_veh_curb, the number of cells is %d\n", (int)m_num_cells);
		exit(-1);
	}

	// last cell
	m_cell_array[m_num_cells - 1] -> m_out_veh_car = m_cell_array[m_num_cells - 1] -> m_veh_queue_car.size();
	m_cell_array[m_num_cells - 1] -> m_out_veh_truck = m_cell_array[m_num_cells - 1] -> m_veh_queue_truck.size();

	return 0;
}

int MNM_Dlink_Ctm_Multiclass_Curb::move_last_cell_curb(TInt timestamp)
{
	// step 1: check how many vehicles are departing from curb cell, update departing queue
	m_curb_cell_array[m_num_cells - 1] -> move_veh_out_curb(timestamp);

	// move last cell
	TInt _num_veh_tomove_car = m_cell_array[m_num_cells - 1] -> m_out_veh_car; // m_out_veh_car = m_cell_array[m_num_cells - 1] -> m_veh_queue_car.size()
	TInt _num_veh_tomove_truck = m_cell_array[m_num_cells - 1] -> m_out_veh_truck;
	TFlt _pstar = TFlt(_num_veh_tomove_car)/TFlt(_num_veh_tomove_car + _num_veh_tomove_truck);
	MNM_Veh* _veh;
	MNM_Veh_Multiclass_Curb* _veh_multiclass;
	TFlt _r;

	while ((_num_veh_tomove_car > 0) || (_num_veh_tomove_truck > 0)){ // both car and truck have flow to move
		_r = MNM_Ults::rand_flt(); // randomly get a number to decide to move a car or truck.
		// probability = _pstar to move a car
		if (_r < _pstar){ // if _r is less than random number
			// and still has car to move
			if (_num_veh_tomove_car > 0){
				_veh = m_cell_array[m_num_cells - 1] -> m_veh_queue_car.front();
				m_cell_array[m_num_cells - 1] -> m_veh_queue_car.pop_front(); // delete the first element	
				_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass_Curb *>(_veh);

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
				_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass_Curb *>(_veh);

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
				_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass_Curb *>(_veh);

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
				_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass_Curb *>(_veh);

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

		_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass_Curb *>(_veh);

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

		_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass_Curb *>(_veh);

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

				_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass_Curb *>(_veh);

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

				_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass_Curb *>(_veh);

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

				_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass_Curb *>(_veh);

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

				_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass_Curb *>(_veh);

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
														int(m_curb_cell_array[m_num_cells - 1] -> m_veh_parking_truck.size()); // remove 2 * 

	m_curb_cell_array[m_num_cells - 1] -> m_veh_doubleparking_num = int(m_curb_cell_array[m_num_cells - 1] -> m_veh_doubleparking_rh.size()) +
														int(m_curb_cell_array[m_num_cells - 1] -> m_veh_doubleparking_truck.size()); // remove 2 * 

	return 0;
}

int MNM_Dlink_Ctm_Multiclass_Curb::install_curb()
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

int MNM_Dlink_Ctm_Multiclass_Curb::move_veh_queue_curb_biclass(std::deque<MNM_Veh*> *from_queue_car, // m_cell_array[i] -> m_veh_queue_car
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
	MNM_Veh_Multiclass_Curb* _veh_multiclass;
	
	for (int i = 0; i < number_tomove_car; ++i) {
		_veh = from_queue_car -> front();
		from_queue_car -> pop_front();

		_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass_Curb *>(_veh);

		// check if _veh_multiclass needs parking, if yes added into curb_arriving_queue
		/* TODO check if veh's interdest front item is this link's end node */
		if (_veh_multiclass -> m_curb_destination_list.front() == this -> m_link_ID){
			to_queue_curb -> push_back(_veh); // now the arriving is not real arriving, it is a buffer
		}
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

		_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass_Curb *>(_veh);

		// check if _veh_multiclass needs parking, if yes added into curb_arriving_queue
		if (_veh_multiclass -> m_curb_destination_list.front() == this -> m_link_ID){
			to_queue_curb -> push_back(_veh); // now the arriving is not real arriving, it is a buffer
		}
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

		_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass_Curb *>(_veh);

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
			else if ((_num_available_dp >= 2) && (_veh_multiclass -> m_parking_location * m_num_cells < (cell_ID+1)) && (_num_available_next < 2)){
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

		_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass_Curb *>(_veh);
		_veh_multiclass -> m_visual_position_on_link += float(1)/float(m_num_cells);
		_veh_multiclass -> m_curb_destination_list.pop_front(); // pop out current curb ID

		if ((_veh_multiclass -> m_class == 0) || (_veh_multiclass -> m_class == 2)){
			to_queue_car -> push_back(_veh);
		}

		if (_veh_multiclass -> m_class == 1){
			to_queue_truck -> push_back(_veh);
		}
	}
	return 0;
}

int MNM_Dlink_Ctm_Multiclass_Curb::evolve_curb(TInt timestamp)
{
	std::deque<MNM_Veh*>::iterator _veh_it;
	TInt _count_car = 0; // car number
	TInt _count_truck = 0; // truck number 
	TInt _count_tot_vehs = 0; // total number

	std::vector<TFlt> _ratio_lane_closure_list;

	/***********************************************************************************************
	Notes for dev: update the scalars in truncated fundamental diagram
	this scalar is related to occupancy of curb space (curb occupancy) and side lane (double parking)
	parameters: 0.2 for normal parking and 0.5 for double parking
	 ***********************************************************************************************/
	
	for (int i = 0; i < m_num_cells; ++i)
	{
		TInt _num_lane = this -> m_number_of_lane;
		TFlt _normal_parking_factor, _double_parking_factor, _total_factor;

		_normal_parking_factor = 0.02 * TFlt(m_curb_cell_array[i] -> m_veh_parking_num) / TFlt(m_curb_cell_array[i] -> m_cell_capacity);
		_double_parking_factor = 0.05 * TFlt(m_curb_cell_array[i] -> m_veh_doubleparking_num) / TFlt(m_curb_cell_array[i] -> m_cell_capacity);
		_total_factor = _normal_parking_factor + _double_parking_factor;

		if (_total_factor > 1.0){
			_total_factor = 1.0;
		}

		TFlt _num_lane_left = TFlt(this -> m_number_of_lane) - _total_factor;

		if (_num_lane_left < 0){
			_num_lane_left = 0.5; // at least 0.5 open, cannot fully blocked for DNL stability
		}

		_ratio_lane_closure_list.push_back(_num_lane_left/_num_lane);
	}

	update_out_veh_curb(_ratio_lane_closure_list);

	// printf("=== update_out_veh_curb done ===\n");

	TInt _num_veh_tomove_car, _num_veh_tomove_truck;

	/* CTM cells except last one */
	if (m_num_cells > 1){
		for (int i = 0; i < m_num_cells - 1; ++i)
		{
			/* step 1: check how many vehicles are departing from curb cell, update departing queue */
			m_curb_cell_array[i] -> move_veh_out_curb(timestamp);

			// step 2: moving vehs from link cell i to link cell i+1
			// check how many vehs are arriving, if space available, add them to arriving queue
			// moving departing vehs to next link cell i+1

			// printf("=== move_veh_out_curb done cell id %d===\n", i);
			// Car
			_num_veh_tomove_car = m_cell_array[i] -> m_out_veh_car; // this out flux is computed by FDs between cells
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
		MNM_Veh_Multiclass_Curb *_veh = dynamic_cast<MNM_Veh_Multiclass_Curb *>(*_veh_it);
		if (_veh -> m_class != 1) m_tot_wait_time_at_intersection_car += TFlt(1)/m_flow_scalar * m_unit_time;
		if (_veh -> m_class == 1) m_tot_wait_time_at_intersection_truck += TFlt(1)/m_flow_scalar * m_unit_time;
	}

	for (int i = 0; i < m_num_cells; ++i)
	{
		_ratio_lane_closure_list[i] = (TFlt(this -> m_number_of_lane) - 
		0.2 * TFlt(m_curb_cell_array[i] -> m_veh_parking_num) / TFlt(m_curb_cell_array[i] -> m_cell_capacity) - 
		0.5 * TFlt(m_curb_cell_array[i] -> m_veh_doubleparking_num) / TFlt(m_curb_cell_array[i] -> m_cell_capacity))/(TFlt(this -> m_number_of_lane));
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
		}
	}

	_count_car = 0;
	_count_truck = 0;
	// m_class: 0 - private car, 1 - truck
	for (_veh_it = m_finished_array.begin(); _veh_it != m_finished_array.end(); _veh_it++){
		MNM_Veh_Multiclass_Curb *_veh = dynamic_cast<MNM_Veh_Multiclass_Curb *>(*_veh_it);
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
	m_parking_num_car = 0;
	m_parking_num_truck = 0;
	m_parking_num_rh = 0;

	for (int m = 0; m < (m_num_cells - 1); ++m)
	{
		m_parking_num_car += TInt(m_curb_cell_array[m] -> m_veh_parking_car.size()) + TInt(m_curb_cell_array[m] -> m_veh_doubleparking_car.size());
		m_parking_num_truck += TInt(m_curb_cell_array[m] -> m_veh_parking_truck.size()) + TInt(m_curb_cell_array[m] -> m_veh_doubleparking_truck.size());
		m_parking_num_rh += TInt(m_curb_cell_array[m] -> m_veh_parking_rh.size()) + TInt(m_curb_cell_array[m] -> m_veh_doubleparking_rh.size());
	}
	return 0;
}

/**************************************************************************
						Multiclass CTM Cells
**************************************************************************/
MNM_Dlink_Ctm_Multiclass_Curb::Ctm_Cell_Multiclass::Ctm_Cell_Multiclass(TInt cell_ID, TFlt cell_length,
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

MNM_Dlink_Ctm_Multiclass_Curb::Ctm_Cell_Multiclass::~Ctm_Cell_Multiclass()
{
	m_veh_queue_car.clear();
	m_veh_queue_truck.clear();
}


int MNM_Dlink_Ctm_Multiclass_Curb::Ctm_Cell_Multiclass::update_perceived_density(TFlt _ratio_lane_closure)
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

TFlt MNM_Dlink_Ctm_Multiclass_Curb::Ctm_Cell_Multiclass::get_perceived_demand(TInt veh_type, TFlt _ratio_lane_closure)
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

TFlt MNM_Dlink_Ctm_Multiclass_Curb::Ctm_Cell_Multiclass::get_perceived_supply(TInt veh_type, TFlt _ratio_lane_closure)
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
 						Multiclass Curb cell for CTM link
**************************************************************************/

MNM_Dlink_Ctm_Multiclass_Curb::Cell_Curb_Multiclass::Cell_Curb_Multiclass(TInt cell_ID,
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

	m_cell_ID = cell_ID;
	m_link_ID = link_ID;
	m_dest_ID = dest_ID;
	m_cell_capacity = cell_capacity;
	m_unit_time = unit_time;
	m_current_time = TFlt(0);
	m_veh_parking_num = TInt(0);
	m_veh_doubleparking_num = TInt(0);
}

MNM_Dlink_Ctm_Multiclass_Curb::Cell_Curb_Multiclass::~Cell_Curb_Multiclass()
{
	m_veh_parking_car.clear();
	m_veh_parking_truck.clear();
	m_veh_parking_rh.clear();

	m_veh_doubleparking_car.clear();
	m_veh_doubleparking_truck.clear();
	m_veh_doubleparking_rh.clear();

	m_veh_arriving.clear();
	m_veh_departing.clear();
}

// Curb-Jiachao added moving veh from m_veh_arriving to m_veh_parking in curb cell
// from_queue is m_veh_arriving (after checking if vehs can park at the curb within the capacity)
// arriving vehs must arrive !!!
int MNM_Dlink_Ctm_Multiclass_Curb::Cell_Curb_Multiclass::move_veh_in_curb(TInt timestamp)
{
	MNM_Veh* _veh;
	MNM_Veh_Multiclass_Curb* _veh_multiclass;

	int _arriving_num = int(m_veh_arriving.size());

	for (int i = 0; i < _arriving_num; ++i){
		_veh = m_veh_arriving.front();
		m_veh_arriving.pop_front();
		_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass_Curb *>(_veh);
		
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


/* This is a function for moving veh from m_veh_parking in curb cell to m_veh_departing 
   first checking parking duration then deciding whether to move from m_veh_parking_car/truck/rh to m_veh_departing 
   result is getting m_veh_departing updated */

int MNM_Dlink_Ctm_Multiclass_Curb::Cell_Curb_Multiclass::move_veh_out_curb(TInt timestamp)
{
	MNM_Veh* _veh;
	MNM_Veh_Multiclass_Curb* _veh_multiclass;

	/* car departing */
	int _num_parking_car = int(m_veh_parking_car.size());
	for (int i = 0; i < _num_parking_car; ++i){
		_veh = m_veh_parking_car.front();
		m_veh_parking_car.pop_front();
		_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass_Curb *>(_veh);

		int stop_id = _veh_multiclass -> m_arrival_time_list.size(); // m_arrival_time_list is a vector to record arrival time, the last one is the current arrival time
		if ((timestamp - _veh_multiclass -> m_arrival_time_list.back()) >= (_veh_multiclass -> m_parking_duration_list[stop_id - 1])){
			_veh_multiclass -> m_departure_time_list.push_back(timestamp);
			_veh_multiclass -> m_complete_stop_current_link = TInt(1); // flag for vehs departing from curb
			m_veh_departing.push_back(_veh);
		}
		else{
			m_veh_parking_car.push_back(_veh);
		}
	}

	/* truck departing */
	int _num_parking_truck = int(m_veh_parking_truck.size());
	for (int j = 0; j < _num_parking_truck; ++j){
		_veh = m_veh_parking_truck.front();
		m_veh_parking_truck.pop_front();
		_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass_Curb *>(_veh);

		int stop_id = _veh_multiclass -> m_arrival_time_list.size();
		if ((timestamp - _veh_multiclass -> m_arrival_time_list.back()) >= (_veh_multiclass->m_parking_duration_list[stop_id - 1])){
			_veh_multiclass -> m_departure_time_list.push_back(timestamp);
			_veh_multiclass -> m_complete_stop_current_link = TInt(1);
			m_veh_departing.push_back(_veh);
		}
		else{
			m_veh_parking_truck.push_back(_veh);
		}
	}

	/* rh departing */
	int _num_parking_rh = int(m_veh_parking_rh.size());
	for (int k = 0; k < _num_parking_rh; ++k){
		_veh = m_veh_parking_rh.front();
		m_veh_parking_rh.pop_front();
		_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass_Curb *>(_veh);

		int stop_id = _veh_multiclass -> m_arrival_time_list.size();
		if ((timestamp - _veh_multiclass -> m_arrival_time_list.back()) >= (_veh_multiclass->m_parking_duration_list[stop_id - 1])){
			_veh_multiclass -> m_departure_time_list.push_back(timestamp);
			_veh_multiclass -> m_complete_stop_current_link = TInt(1);
			m_veh_departing.push_back(_veh);
		}
		else{
			m_veh_parking_rh.push_back(_veh);
		}
	}

	/* double parking cars departing */
	int _num_dp_car = int(m_veh_doubleparking_car.size());
	for (int i = 0; i < _num_dp_car; ++i){
		_veh = m_veh_doubleparking_car.front();
		m_veh_doubleparking_car.pop_front();
		_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass_Curb *>(_veh);

		int stop_id = _veh_multiclass -> m_arrival_time_list.size(); // m_arrival_time_list: std::vector<TInt> vector to record arrival time, the last one is the current arrival time
		if ((timestamp - _veh_multiclass -> m_arrival_time_list.back()) >= (_veh_multiclass -> m_parking_duration_list[stop_id - 1])){
			_veh_multiclass -> m_departure_time_list.push_back(timestamp);
			_veh_multiclass -> m_complete_stop_current_link = TInt(1);
			m_veh_departing.push_back(_veh);
		}
		else{
			m_veh_doubleparking_car.push_back(_veh);
		}
	}

	/* double parking trucks departing */
	int _num_dp_truck = int(m_veh_doubleparking_truck.size());
	for (int j = 0; j < _num_dp_truck; ++j){
		_veh = m_veh_doubleparking_truck.front();
		m_veh_doubleparking_truck.pop_front();
		_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass_Curb *>(_veh);

		int stop_id = _veh_multiclass -> m_arrival_time_list.size();
		if ((timestamp - _veh_multiclass -> m_arrival_time_list.back()) >= (_veh_multiclass->m_parking_duration_list[stop_id - 1])){
			_veh_multiclass -> m_departure_time_list.push_back(timestamp);
			_veh_multiclass -> m_complete_stop_current_link = TInt(1);
			m_veh_departing.push_back(_veh);
		}
		else{
			m_veh_doubleparking_truck.push_back(_veh);
		}
	}

	/* double parking RH departing */
	int _num_dp_rh = int(m_veh_doubleparking_rh.size());
	for (int k = 0; k < _num_dp_rh; ++k){
		_veh = m_veh_doubleparking_rh.front();
		m_veh_doubleparking_rh.pop_front();
		_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass_Curb *>(_veh);

		int stop_id = _veh_multiclass -> m_arrival_time_list.size();
		if ((timestamp - _veh_multiclass -> m_arrival_time_list.back()) >= _veh_multiclass->m_parking_duration_list[stop_id - 1]){
			_veh_multiclass -> m_departure_time_list.push_back(timestamp);
			_veh_multiclass -> m_complete_stop_current_link = TInt(1);
			m_veh_departing.push_back(_veh);
		}
		else{
			m_veh_doubleparking_rh.push_back(_veh);
		}
	}
	return 0;
}

/**************************************************************************
							Multiclass Link-Queue Model
**************************************************************************/
MNM_Dlink_Lq_Multiclass_Curb::MNM_Dlink_Lq_Multiclass_Curb(TInt ID,
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
  : MNM_Dlink_Multiclass_Curb::MNM_Dlink_Multiclass_Curb(ID, number_of_lane, length, ffs_car, ffs_truck)
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

	m_veh_doubleparking_car = std::deque<MNM_Veh*>();
	m_veh_doubleparking_truck = std::deque<MNM_Veh*>();
	m_veh_doubleparking_rh = std::deque<MNM_Veh*>();

	m_total_parking_num_car = TInt(0);
	m_total_parking_num_truck = TInt(0);
	m_total_parking_num_rh = TInt(0);

	m_total_doubleparking_num_car = TInt(0);
	m_total_doubleparking_num_truck = TInt(0);
	m_total_doubleparking_num_rh = TInt(0);

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

	m_curb_spaces = curb_spaces;
	m_curb_dest = curb_dest;
}

MNM_Dlink_Lq_Multiclass_Curb::~MNM_Dlink_Lq_Multiclass_Curb()
{
	m_veh_queue_car.clear();
	m_veh_queue_truck.clear();
	m_veh_out_buffer_car.clear();
	m_veh_out_buffer_truck.clear();

	m_veh_parking_car.clear();
	m_veh_parking_truck.clear();
	m_veh_parking_rh.clear();

	m_veh_doubleparking_car.clear();
	m_veh_doubleparking_truck.clear();
	m_veh_doubleparking_rh.clear();
}

TFlt MNM_Dlink_Lq_Multiclass_Curb::get_link_supply()
{
	TFlt _supply_car = m_space_fraction_car * std::min(m_C_car, TFlt(m_w_car * (m_k_j_car - m_perceived_density_car))) * m_unit_time;
	TFlt _supply_truck = m_space_fraction_truck * std::min(m_C_truck, TFlt(m_w_truck * (m_k_j_truck - m_perceived_density_truck))) * m_unit_time;

	// Only for short links, change the FD shape around rhoj:
    _supply_car = std::max(_supply_car, TFlt(m_space_fraction_car * m_w_car * 0.30 * (m_k_j_car - m_k_C_car) * m_unit_time));
    _supply_truck = std::max(_supply_truck, TFlt(m_space_fraction_truck * m_w_truck * 0.30 * (m_k_j_truck - m_k_C_truck) * m_unit_time));

	TFlt _supply = std::max(TFlt(0.0), _supply_car) + m_veh_convert_factor * std::max(TFlt(0.0), _supply_truck);
	return _supply;
}

TFlt MNM_Dlink_Lq_Multiclass_Curb::get_link_capacity()
{
	return m_C_car * m_unit_time;
}

int MNM_Dlink_Lq_Multiclass_Curb::clear_incoming_array(TInt timestamp, TFlt _ratio_lane_closure) {
  	MNM_Veh_Multiclass_Curb* _veh;
	size_t _cur_size = m_incoming_array.size();
	// loop for each veh in the incoming array, check if it needs to park at this LQ link
	for (size_t i = 0; i < _cur_size; ++i) {
		_veh = dynamic_cast<MNM_Veh_Multiclass_Curb *>(m_incoming_array.front());
		if (_veh -> m_class != TInt(1)) {
			// if parking at this link, add it to the parking array
			if (_veh -> m_curb_destination_list.front() == this -> m_link_ID){
				if (_veh -> m_class == TInt(0)){
					// m_veh_parking_car.push_back(m_incoming_array.front());
					// cars are not allowed to double park (and curb park here)
					m_veh_queue_car.push_back(_veh);
					m_incoming_array.pop_front();
				}
				if (_veh -> m_class == TInt(2)){
					if ((TInt)(m_veh_parking_rh.size() + m_veh_parking_truck.size()) < m_curb_spaces) {
						m_veh_parking_rh.push_back(m_incoming_array.front());
						m_total_parking_num_rh += TInt(1);
					}
					else {
						m_veh_doubleparking_rh.push_back(m_incoming_array.front());
						m_total_doubleparking_num_rh += TInt(1);
					}
					
					m_incoming_array.pop_front();
					_veh -> m_arrival_time_list.push_back(timestamp);
				}
			}
			// if not parking, add it back to queue array
			else{ 
				m_veh_queue_car.push_back(_veh);
				m_incoming_array.pop_front();
			}
		}
		else {
			assert(_veh -> m_class == TInt(1));
			// if parking at this link, add it to the parking array
			if (_veh -> m_curb_destination_list.front() == this -> m_link_ID){
				if ((TInt)(m_veh_parking_rh.size() + m_veh_parking_truck.size()) < m_curb_spaces) {
					m_veh_parking_truck.push_back(m_incoming_array.front());
					m_total_parking_num_truck += TInt(1);
				}
				else {
					m_veh_doubleparking_truck.push_back(m_incoming_array.front());
					m_total_doubleparking_num_truck += TInt(1);
				}
				
				m_incoming_array.pop_front();
				_veh -> m_arrival_time_list.push_back(timestamp);
			}
			// if not parking, add it to queue array
			else{
				m_veh_queue_truck.push_back(_veh);
				m_incoming_array.pop_front();
			}

			// // Jiachao revised for reservation, comment out above
			// assert(_veh -> m_class == TInt(1));
			// // if the truck is not parking, it should be in the queue
			// if (_veh -> m_curb_destination_list.front() == this -> m_link_ID){
			// 	if ((TInt)(m_veh_parking_rh.size() + m_veh_parking_truck.size()) < 2 * m_curb_spaces) {
			// 		m_veh_parking_truck.push_back(m_incoming_array.front());
			// 		m_total_parking_num_truck += TInt(1);
			// 	}
			// 	else if (m_veh_parking_rh.size() > 0){
			// 		m_veh_parking_truck.push_back(m_incoming_array.front());
			// 		m_total_parking_num_truck += TInt(1);
			// 		m_total_doubleparking_num_rh += TInt(1);
			// 		m_total_parking_num_rh -= TInt(1);
			// 	}
			// 	else {
			// 		m_veh_doubleparking_truck.push_back(m_incoming_array.front());
			// 		m_total_doubleparking_num_truck += TInt(1);
			// 	}
				
			// 	m_incoming_array.pop_front();
			// 	_veh -> m_arrival_time_list.push_back(timestamp);
			// }
			// // if not parking, add it to queue array
			// else{
			// 	m_veh_queue_truck.push_back(_veh);
			// 	m_incoming_array.pop_front();
			// }
		}
		_veh -> m_visual_position_on_link = 0.5;
	}
	return 0;
}

TFlt MNM_Dlink_Lq_Multiclass_Curb::get_link_flow_car()
{
	return TFlt(m_volume_car) / m_flow_scalar;
}

TFlt MNM_Dlink_Lq_Multiclass_Curb::get_link_flow_truck()
{
	return TFlt(m_volume_truck) / m_flow_scalar;
}

TFlt MNM_Dlink_Lq_Multiclass_Curb::get_link_flow()
{
	// For get_link_tt in adaptive routing
	return TFlt(m_volume_car + m_volume_truck) / m_flow_scalar;
}

TFlt MNM_Dlink_Lq_Multiclass_Curb::get_link_tt()
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

TFlt MNM_Dlink_Lq_Multiclass_Curb::get_link_tt_from_flow_car(TFlt flow)
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

TFlt MNM_Dlink_Lq_Multiclass_Curb::get_link_tt_from_flow_truck(TFlt flow)
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

TInt MNM_Dlink_Lq_Multiclass_Curb::get_link_freeflow_tt_loading_car() {
	// Jiachao fixed in 0627
    return MNM_Ults::round_up_time(m_length / (m_ffs_car * m_unit_time));
}

TInt MNM_Dlink_Lq_Multiclass_Curb::get_link_freeflow_tt_loading_truck() {
    // throw std::runtime_error("Error, MNM_Dlink_Lq_Multiclass::get_link_freeflow_tt_loading_truck NOT implemented");
	// jiachao fixed in 0627
	return MNM_Ults::round_up_time(m_length / (m_ffs_truck * m_unit_time));
}

int MNM_Dlink_Lq_Multiclass_Curb::update_perceived_density()
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
			m_space_fraction_truck = 0;
		}
		else {
			m_space_fraction_car = _space_fraction_car / (_space_fraction_car + _space_fraction_truck);
			m_space_fraction_truck = _space_fraction_truck / (_space_fraction_car + _space_fraction_truck);
		}
		
		// PMC 
		// cell_congested: 1 - free flow ; 2 - semi-congested ; 3 - fully-congested.
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

// check TODO 04/10/2025
int MNM_Dlink_Lq_Multiclass_Curb::evolve_curb(TInt timestamp)
{
	// Jiachao added in 03/12/2024 to model parking vehicles 
	MNM_Veh* _veh_p;
	MNM_Veh_Multiclass_Curb* _veh_multiclass;

	int _num_parking_car = int(m_veh_parking_car.size());

	// printf("before checking arrival - Parking car: %d\n", (int)_num_parking_car);

	for (int i = 0; i < _num_parking_car; ++i){
		_veh_p = m_veh_parking_car.front();
		m_veh_parking_car.pop_front();
		_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass_Curb *>(_veh_p);
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

	// departing double parking cars (actually this is not true)
	int _num_dp_car = int(m_veh_doubleparking_car.size());

	for (int i = 0; i < _num_dp_car; ++i){
		_veh_p = m_veh_doubleparking_car.front();
		m_veh_doubleparking_car.pop_front();
		_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass_Curb *>(_veh_p);
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
			m_veh_doubleparking_car.push_back(_veh_p);
		}
	}

	m_parking_num_car = m_veh_parking_car.size() + m_veh_doubleparking_car.size();
	m_curb_parking_num_car = m_veh_parking_car.size();
	m_curb_doubleparking_num_car = m_veh_doubleparking_car.size();
	// printf("Parking car: %d\n", (int)m_parking_num_car);

	int _num_parking_truck = int(m_veh_parking_truck.size());
	for (int i = 0; i < _num_parking_truck; ++i){
		_veh_p = m_veh_parking_truck.front();
		m_veh_parking_truck.pop_front();
		_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass_Curb *>(_veh_p);
		int stop_id = _veh_multiclass -> m_arrival_time_list.size();
		// parking complete
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

	// departing double parking trucks
	int _num_dp_truck = int(m_veh_doubleparking_truck.size());
	for (int j = 0; j < _num_dp_truck; ++j){
		_veh_p = m_veh_doubleparking_truck.front();
		m_veh_doubleparking_truck.pop_front();
		_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass_Curb *>(_veh_p);
		int stop_id = _veh_multiclass -> m_arrival_time_list.size();
		// parking complete
		if ((timestamp - _veh_multiclass -> m_arrival_time_list.back()) >= _veh_multiclass->m_parking_duration_list[stop_id - 1]){
			// record departure time
			_veh_multiclass -> m_departure_time_list.push_back(timestamp);
			_veh_multiclass -> m_complete_stop_current_link = TInt(1);
			m_veh_queue_truck.push_back(_veh_p);
		}
		// not complete
		else{
			m_veh_doubleparking_truck.push_back(_veh_p);
		}
	}

	m_parking_num_truck = m_veh_parking_truck.size() + m_veh_doubleparking_truck.size();
	m_curb_parking_num_truck = m_veh_parking_truck.size();
	m_curb_doubleparking_num_truck = m_veh_doubleparking_truck.size();
	// printf("Parking truck: %d\n", (int)m_parking_num_truck);

	int _num_parking_rh = int(m_veh_parking_rh.size());
	for (int i = 0; i < _num_parking_rh; ++i){
		_veh_p = m_veh_parking_rh.front();
		m_veh_parking_rh.pop_front();
		_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass_Curb *>(_veh_p);
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
			m_veh_parking_rh.push_back(_veh_p);
		}
	}

	// departing double parking RH
	int _num_dp_rh = int(m_veh_doubleparking_rh.size());
	for (int k = 0; k < _num_dp_rh; ++k){
		_veh_p = m_veh_doubleparking_rh.front();
		m_veh_doubleparking_rh.pop_front();
		_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass_Curb *>(_veh_p);
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
			m_veh_doubleparking_rh.push_back(_veh_p);
		}
	}

	m_parking_num_rh = m_veh_parking_rh.size() + m_veh_doubleparking_rh.size();
	m_curb_parking_num_rh = m_veh_parking_rh.size();
	m_curb_doubleparking_num_rh = m_veh_doubleparking_rh.size();
	// printf("Parking rh: %d\n", (int)m_parking_num_rh);

	/* Update volume, perceived density, space fraction, and demand/supply */
	std::deque<MNM_Veh*>::iterator _veh_it;
	TInt _count_car = 0;
	TInt _count_truck = 0;
	TInt _count_tot_vehs = 0;
	for (_veh_it = m_finished_array.begin(); _veh_it != m_finished_array.end(); _veh_it++){
		MNM_Veh_Multiclass_Curb *_veh = dynamic_cast<MNM_Veh_Multiclass_Curb *>(*_veh_it);
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
	// printf("Perceived density car: %.4f, truck: %.4f\n", m_perceived_density_car, m_perceived_density_truck);

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

	if ((m_veh_out_buffer_car.size() != 0) || (m_veh_out_buffer_car.size() != 0)){
		printf("Something wrong with our buffer, not empty!\n");
		exit(-1);
	}
	m_tot_wait_time_at_intersection += m_finished_array.size()/m_flow_scalar * m_unit_time;
	return 0;
}

/**************************************************************************
							Multiclass Point-Queue Model
**************************************************************************/
MNM_Dlink_Pq_Multiclass_Curb::MNM_Dlink_Pq_Multiclass_Curb(TInt ID,
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
  : MNM_Dlink_Multiclass_Curb::MNM_Dlink_Multiclass_Curb(ID, number_of_lane, length, ffs_car, ffs_truck)
{
    m_link_type = MNM_TYPE_PQ_MULTICLASS;

	// PQ only used for OD connectors, cap/rhoj are all 99999 
	// so no need to use truck parameters
	m_lane_hold_cap = lane_hold_cap_car;
	m_lane_flow_cap = lane_flow_cap_car;
	m_flow_scalar = flow_scalar;
	m_hold_cap = m_lane_hold_cap * TFlt(number_of_lane) * m_length;
	m_max_stamp = MNM_Ults::round(m_length/(ffs_car * unit_time));

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
}

MNM_Dlink_Pq_Multiclass_Curb::~MNM_Dlink_Pq_Multiclass_Curb()
{
	m_veh_pool.clear();
}

TFlt MNM_Dlink_Pq_Multiclass_Curb::get_link_supply()
{
	return m_lane_flow_cap * TFlt(m_number_of_lane) * m_unit_time;
}

TFlt MNM_Dlink_Pq_Multiclass_Curb::get_link_capacity()
{
	return m_lane_flow_cap * TFlt(m_number_of_lane) * m_unit_time;
}

int MNM_Dlink_Pq_Multiclass_Curb::clear_incoming_array(TInt timestamp, TFlt _ratio_lane_closure) {
	MNM_Veh_Multiclass_Curb *_veh;
	TFlt _to_be_moved = get_link_supply() * m_flow_scalar;
	while (!m_incoming_array.empty()) {
		if ( _to_be_moved > 0){
			_veh = dynamic_cast<MNM_Veh_Multiclass_Curb *>(m_incoming_array.front());
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
        auto *_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass_Curb *>(_veh_it.first);
        if (_veh_multiclass -> m_class == 0) m_volume_car += 1;
        if (_veh_multiclass -> m_class == 1) m_volume_truck += 1;
    }
    for (auto _veh_it : m_finished_array){
        auto *_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass_Curb *>(_veh_it);
        if (_veh_multiclass -> m_class == 0) m_volume_car += 1;
        if (_veh_multiclass -> m_class == 1) m_volume_truck += 1;
    }
	// printf("car: %d, truck: %d\n", m_volume_car, m_volume_truck);
	return 0;
}

TFlt MNM_Dlink_Pq_Multiclass_Curb::get_link_flow_car()
{
	return TFlt(m_volume_car) / m_flow_scalar;
}

TFlt MNM_Dlink_Pq_Multiclass_Curb::get_link_flow_truck()
{
	return TFlt(m_volume_truck) / m_flow_scalar;
}

TFlt MNM_Dlink_Pq_Multiclass_Curb::get_link_flow()
{
	return TFlt(m_volume_car + m_volume_truck) / m_flow_scalar;
}


TFlt MNM_Dlink_Pq_Multiclass_Curb::get_link_tt()
{
	return m_length/m_ffs_car;
}

TFlt MNM_Dlink_Pq_Multiclass_Curb::get_link_tt_from_flow_car(TFlt flow)
{
    return m_length/m_ffs_car;
}

TFlt MNM_Dlink_Pq_Multiclass_Curb::get_link_tt_from_flow_truck(TFlt flow)
{
    return m_length/m_ffs_car;
}

TInt MNM_Dlink_Pq_Multiclass_Curb::get_link_freeflow_tt_loading_car()
{
	return m_max_stamp;  // ensure this >= 1 in constructor
}

TInt MNM_Dlink_Pq_Multiclass_Curb::get_link_freeflow_tt_loading_truck()
{
	return m_max_stamp;  // ensure this >= 1 in constructor
}

int MNM_Dlink_Pq_Multiclass_Curb::evolve_curb(TInt timestamp)
{
	std::unordered_map<MNM_Veh*, TInt>::iterator _que_it = m_veh_pool.begin();
	MNM_Veh_Multiclass_Curb* _veh;
	TInt _num_car = 0, _num_truck = 0;
	while (_que_it != m_veh_pool.end()) {
		if (_que_it -> second >= m_max_stamp) {
			m_finished_array.push_back(_que_it -> first);
			_veh = dynamic_cast<MNM_Veh_Multiclass_Curb *>(m_finished_array.back());
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

/**************************************************************************
				Multiclass Node Model for curb modeling
**************************************************************************/

MNM_DMOND_Multiclass_Curb::MNM_DMOND_Multiclass_Curb(TInt ID, TFlt flow_scalar, TFlt veh_convert_factor)
	: MNM_DMOND::MNM_DMOND(ID, flow_scalar)
{
	m_veh_convert_factor = veh_convert_factor;
}

MNM_DMOND_Multiclass_Curb::~MNM_DMOND_Multiclass_Curb()
{
	;
}

int MNM_DMOND_Multiclass_Curb::evolve_curb(TInt timestamp)
{
	MNM_Dlink *_link;
  	MNM_Veh_Multiclass_Curb *_veh;
	MNM_Dlink_Pq_Multiclass_Curb *_next_link;

	// initialize out volume, set it to zeros
	for (unsigned i = 0; i < m_out_link_array.size(); ++i){
    	_link = m_out_link_array[i];
   		m_out_volume[_link] = 0;
  	}

	// compute out flow
	std::deque<MNM_Veh*>::iterator _que_it = m_in_veh_queue.begin();
  	while (_que_it != m_in_veh_queue.end()) 
	{
  		_veh = dynamic_cast<MNM_Veh_Multiclass_Curb *>(*_que_it);
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

	for (unsigned i = 0; i < m_out_link_array.size(); ++i)
	{
	    _link = m_out_link_array[i];
	    if ((_link -> get_link_supply() * m_flow_scalar) < TFlt(m_out_volume[_link])){
	      	m_out_volume[_link] = TInt(MNM_Ults::round(_link -> get_link_supply() * m_flow_scalar));
	    }
  	}

	/* move vehicle */
  	TInt _moved_car, _moved_truck, _moved_rh;
  	for (unsigned i = 0; i < m_out_link_array.size(); ++i)
	{
	    _link = m_out_link_array[i];
	    _moved_car = 0;
	    _moved_truck = 0;	
		_moved_rh = 0;    
	    _que_it = m_in_veh_queue.begin();
	    while (_que_it != m_in_veh_queue.end()) 
		{
	      	if (m_out_volume[_link] > 0)
			{
		        _veh = dynamic_cast<MNM_Veh_Multiclass_Curb *>(*_que_it);
		        if (_veh -> get_next_link() == _link)
				{
					_link -> m_incoming_array.push_back(_veh);
					_veh -> set_current_link(_link);
					if (_veh -> m_class == 0){
						m_out_volume[_link] -= 1;
						_moved_car += 1;
					}
					if (_veh -> m_class == 1) {

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
	    _next_link = dynamic_cast<MNM_Dlink_Pq_Multiclass_Curb *>(_link);
		// next link is the Pq link
	    if (_next_link -> m_N_in_car != nullptr) {
	      	_next_link -> m_N_in_car -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp + 1), TFlt(_moved_car)/m_flow_scalar)); 
	    }
	    if (_next_link -> m_N_in_truck != nullptr) {
	      	_next_link -> m_N_in_truck -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp + 1), TFlt(_moved_truck)/m_flow_scalar)); 
	    }
		if (_next_link -> m_N_in_rh != nullptr) {
			_next_link -> m_N_in_rh -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp + 1), TFlt(_moved_rh)/m_flow_scalar));
		}
		if (_next_link -> m_N_in_car_all != nullptr) {
			_next_link -> m_N_in_car_all -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp + 1), TFlt(_moved_car + _moved_rh)/m_flow_scalar));
		}
  	}
	
	return 0;
}

MNM_DMDND_Multiclass_Curb::MNM_DMDND_Multiclass_Curb(TInt ID, TFlt flow_scalar, TFlt veh_convert_factor)
	: MNM_DMDND::MNM_DMDND(ID, flow_scalar)
{
	m_veh_convert_factor = veh_convert_factor;
}

MNM_DMDND_Multiclass_Curb::~MNM_DMDND_Multiclass_Curb()
{
	;
}

int MNM_DMDND_Multiclass_Curb::evolve_curb(TInt timestamp)
{
	MNM_Dlink *_link;
  	MNM_Veh_Multiclass_Curb *_veh;
	MNM_Dlink_Pq_Multiclass_Curb *_from_link;

	size_t _size;
  	TInt _moved_car, _moved_truck, _moved_rh;

	for (size_t i = 0; i < m_in_link_array.size(); ++i)
	{
  		_moved_car = 0;
	    _moved_truck = 0;
		_moved_rh = 0;
	    _link = m_in_link_array[i];
	    _size = _link -> m_finished_array.size();
	    for (size_t j = 0; j < _size; ++j)
		{
			_veh = dynamic_cast<MNM_Veh_Multiclass_Curb *>(_link -> m_finished_array.front());
			if (_veh -> get_next_link() != nullptr){
				throw std::runtime_error ("invalid state next link is not nullptr");
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
	    _from_link = dynamic_cast<MNM_Dlink_Pq_Multiclass_Curb *>(_link);
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

MNM_Dnode_Inout_Multiclass_Curb::MNM_Dnode_Inout_Multiclass_Curb(TInt ID, TFlt flow_scalar, TFlt veh_convert_factor)
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

MNM_Dnode_Inout_Multiclass_Curb::~MNM_Dnode_Inout_Multiclass_Curb()
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

int MNM_Dnode_Inout_Multiclass_Curb::evolve_curb(TInt timestamp)
{
	prepare_supplyANDdemand();
	compute_flow();
	move_vehicle_curb(timestamp);
	return 0;
}

int MNM_Dnode_Inout_Multiclass_Curb::prepare_loading()
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

int MNM_Dnode_Inout_Multiclass_Curb::add_out_link(MNM_Dlink* out_link)
{
  	m_out_link_array.push_back(out_link);
  	return 0;
}

int MNM_Dnode_Inout_Multiclass_Curb::add_in_link(MNM_Dlink *in_link)
{
  	m_in_link_array.push_back(in_link);
  	return 0;
}

int MNM_Dnode_Inout_Multiclass_Curb::prepare_supplyANDdemand()
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
	for (size_t i = 0; i < _num_in; ++i){ // loop for each incoming link
		_in_link = m_in_link_array[i];
		for (_veh_it = _in_link -> m_finished_array.begin(); _veh_it != _in_link -> m_finished_array.end(); _veh_it++){ // what is this loop for?
			if (std::find(m_out_link_array.begin(), m_out_link_array.end(), (*_veh_it) -> get_next_link()) == m_out_link_array.end()){ 
				printf("Vehicle in the wrong node, no exit!\n");
        		printf("Vehicle is on link %d, node %d, next link ID is: %d\n", _in_link -> m_link_ID(), m_node_ID(), 
        			   (*_veh_it) -> get_next_link() -> m_link_ID());
				exit(-1);
			}
		}
		for (size_t j = 0; j < _num_out; ++j)
		{
			_out_link = m_out_link_array[j];
			_equiv_count = 0;
			for (_veh_it = _in_link -> m_finished_array.begin(); _veh_it != _in_link -> m_finished_array.end(); _veh_it++)
			{
        		MNM_Veh_Multiclass_Curb *_veh = dynamic_cast<MNM_Veh_Multiclass_Curb *>(*_veh_it);
        		if (_veh -> get_next_link() == _out_link) 
				{
        			if ((_veh -> m_class == TInt(0)) || (_veh -> m_class == TInt(2))) {
        				// car or rh
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
	    m_supply[j] = m_out_link_array[j] -> get_link_supply();
  	}

	return 0;
}

int MNM_Dnode_Inout_Multiclass_Curb::compute_flow()
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

int MNM_Dnode_Inout_Multiclass_Curb::move_vehicle_curb(TInt timestamp)
{
	MNM_Dlink *_in_link, *_out_link;
	MNM_Dlink_Multiclass_Curb *_ilink, *_olink;
	size_t _offset = m_out_link_array.size();
	TFlt _to_move;
	TFlt _equiv_num;
	TFlt _r;

	std::vector<size_t> _in_link_ind_array = std::vector<size_t>();
    for (size_t i=0; i<m_in_link_array.size(); ++i)
	{
        _in_link_ind_array.push_back(i);
    }

	// loop for each out link
	for (size_t j = 0; j < m_out_link_array.size(); ++j)
	{
		_out_link = m_out_link_array[j];

        // shuffle the in links, reserve FIFO
        // std::random_device rng; // random sequence
        // std::shuffle(_in_link_ind_array.begin(), _in_link_ind_array.end(), rng);
		std::random_shuffle (_in_link_ind_array.begin (),
                           _in_link_ind_array.end ());

		// loop for each in link
        for (size_t i : _in_link_ind_array) 
		{
			_in_link = m_in_link_array[i];
			_to_move = m_veh_flow[i * _offset + j] * m_flow_scalar;
			auto _veh_it = _in_link -> m_finished_array.begin();

			while (_veh_it != _in_link -> m_finished_array.end())
			{
				if (_to_move > 0)
				{
					MNM_Veh_Multiclass_Curb *_veh = dynamic_cast<MNM_Veh_Multiclass_Curb *>(*_veh_it);
					if (_veh -> get_next_link() == _out_link){
						if ((_veh -> m_class == TInt(0)) || (_veh -> m_class == TInt(2))) // private car & RH
						{
							_equiv_num = 1;
						}
						if (_veh -> m_class == TInt(1)) // truck
						{ 
							_equiv_num = m_veh_convert_factor;
						}
						if (_to_move < _equiv_num) {							
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
								if (_veh -> m_class == TInt(0)) 
								{
									// m_veh_moved_car[i * _offset + j] += 1;
									_ilink = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(_in_link);
									_olink = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(_out_link);

									// count curb departure
									// veh parked at in link (stop indicator == 1)
									if (_veh -> m_complete_stop_current_link == TInt(1))
									{
										_veh -> m_complete_stop_current_link = TInt(0);

										_ilink -> m_N_out_car_cc -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(1)/m_flow_scalar));

										if (_ilink -> m_N_out_tree_curb_car != nullptr){
											_ilink -> m_N_out_tree_curb_car -> add_flow(TFlt(timestamp + 1), 1/m_flow_scalar, _veh -> m_path, _veh -> m_assign_interval);
										}

										m_veh_moved_car_ilink_cc[i * _offset + j] += 1;
									}
									else { // veh didn't park at in link (stop indicator == 1)
										_ilink -> m_N_out_car -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(1)/m_flow_scalar));
										_ilink -> m_N_out_car_all -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(1)/m_flow_scalar));

										if (_ilink -> m_N_out_tree_car != nullptr) {
											_ilink -> m_N_out_tree_car -> add_flow(TFlt(timestamp + 1), 1/m_flow_scalar, _veh -> m_path, _veh -> m_assign_interval);
										}

										m_veh_moved_car_ilink[i * _offset + j] += 1;
									}

									// veh will not park at out link
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
									_ilink = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(_in_link);
									_olink = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(_out_link);

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
										if (_ilink -> m_N_out_tree_truck != nullptr) {
											_ilink -> m_N_out_tree_truck -> add_flow(TFlt(timestamp + 1), 1/m_flow_scalar, _veh -> m_path, _veh -> m_assign_interval);
										}
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
									else if (_veh -> m_curb_destination_list.front() == _out_link -> m_link_ID) {
										if (_olink -> m_N_in_tree_curb_truck != nullptr){
											_olink -> m_N_in_tree_curb_truck -> add_flow(TFlt(timestamp + 1), 1/m_flow_scalar, _veh -> m_path, _veh -> m_assign_interval);
										}
										m_veh_moved_truck_cc[i * _offset + j] += 1;
										_olink -> m_N_in_truck_cc -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(1)/m_flow_scalar));
									}
									else {
										throw std::runtime_error ("invalid state curb destination list");
									}
								}

								// ride-hailing mode
								if (_veh -> m_class == TInt(2)) {
									_ilink = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(_in_link);
									_olink = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(_out_link);

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
										if (_ilink -> m_N_out_tree_rh != nullptr) {
											_ilink -> m_N_out_tree_rh -> add_flow(TFlt(timestamp + 1), 1/m_flow_scalar, _veh -> m_path, _veh -> m_assign_interval);
										}
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
									else if (_veh -> m_curb_destination_list.front() == _out_link -> m_link_ID) {
										if (_olink -> m_N_in_tree_curb_rh != nullptr){
											_olink -> m_N_in_tree_curb_rh -> add_flow(TFlt(timestamp + 1), 1/m_flow_scalar, _veh -> m_path, _veh -> m_assign_interval);
										}
										m_veh_moved_car_cc[i * _offset + j] += 1;
										_olink -> m_N_in_rh_cc -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(1)/m_flow_scalar));
									}
									else {
										throw std::runtime_error ("invalid state curb destination list");
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
								_ilink = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(_in_link);
								_olink = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(_out_link);

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

									if (_ilink -> m_N_out_tree_car != nullptr) {
										_ilink -> m_N_out_tree_car -> add_flow(TFlt(timestamp + 1), 1/m_flow_scalar, _veh -> m_path, _veh -> m_assign_interval);
									}
								}
								
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
								_olink = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(_out_link);
								_ilink = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(_in_link);

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
									if (_ilink -> m_N_out_tree_truck != nullptr) {
										_ilink -> m_N_out_tree_truck -> add_flow(TFlt(timestamp + 1), 1/m_flow_scalar, _veh -> m_path, _veh -> m_assign_interval);
									}
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
								else if (_veh -> m_curb_destination_list.front() == _out_link -> m_link_ID) {
									// will stop at out link, count curb arrival
									if (_olink -> m_N_in_tree_curb_truck != nullptr){
										_olink -> m_N_in_tree_curb_truck -> add_flow(TFlt(timestamp + 1), 1/m_flow_scalar, _veh -> m_path, _veh -> m_assign_interval);
									}
									m_veh_moved_truck_cc[i * _offset + j] += 1;
									_olink -> m_N_in_truck_cc -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(1)/m_flow_scalar));
								}
								else {
										throw std::runtime_error ("invalid state curb destination list");
								}
							}
							// ride hailing
							if (_veh -> m_class == TInt(2)) {
								// m_veh_moved_car[i * _offset + j] += 1;
								_olink = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(_out_link);
								_ilink = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(_in_link);

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
									if (_ilink -> m_N_out_tree_rh != nullptr) {
										_ilink -> m_N_out_tree_rh -> add_flow(TFlt(timestamp + 1), 1/m_flow_scalar, _veh -> m_path, _veh -> m_assign_interval);
									}
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
								else if (_veh -> m_curb_destination_list.front() == _out_link -> m_link_ID) {
									if (_olink -> m_N_in_tree_curb_rh != nullptr){
										_olink -> m_N_in_tree_curb_rh -> add_flow(TFlt(timestamp + 1), 1/m_flow_scalar, _veh -> m_path, _veh -> m_assign_interval);
									}
									m_veh_moved_car_cc[i * _offset + j] += 1;
									_olink -> m_N_in_rh_cc -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(1)/m_flow_scalar));
								}
								else {
										throw std::runtime_error ("invalid state curb destination list");
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
			if (_to_move > 0.001){
				printf("Something wrong during the vehicle moving, remaining to move %.16f\n", (float)_to_move);
				exit(-1);
			}
		}
	}
    _in_link_ind_array.clear();
	return 0;
}

MNM_Curb_Factory_Multiclass::MNM_Curb_Factory_Multiclass()
{
	m_curb_destination_map = std::unordered_map<TInt, TInt>();
	m_curb_capacity_map = std::unordered_map<TInt, TInt>();

	m_path_inter_dest_map = std::unordered_map<TInt, std::vector<TInt>>();
	m_path_inter_dest_map_rh = std::unordered_map<TInt, std::vector<TInt>>(); 
    m_path_inter_dest_map_car = std::unordered_map<TInt, std::vector<TInt>>(); 

	m_od_inter_dest_map = std::unordered_map<TInt, std::unordered_map<TInt, std::vector<TInt>>>();
	m_interdest_curb_map = std::unordered_map<TInt, std::vector<TInt>>();
	m_interdest_list = std::vector<TInt>();
	m_curb_price_list = std::unordered_map<TInt, std::vector<TFlt>>();
	m_interdest_map = std::unordered_map<TInt, MNM_Destination*>();

	/* multiclass curb prices */
	m_curb_price_list_car = std::unordered_map<TInt, std::vector<TFlt>>();
	m_curb_price_list_truck = std::unordered_map<TInt, std::vector<TFlt>>();
	m_curb_price_list_rh = std::unordered_map<TInt, std::vector<TFlt>>();

	/* curb space reservation */
	m_od_slz_reserve_map = std::unordered_map<TInt, std::unordered_map<TInt, std::vector<TInt>>>();
}

/***** curb pricing *****/

MNM_Statistics_Curb::MNM_Statistics_Curb(const std::string& file_folder, MNM_ConfReader *conf_reader, MNM_ConfReader *record_config,
                                       MNM_OD_Factory *od_factory, MNM_Node_Factory *node_factory, MNM_Link_Factory *link_factory, MNM_Curb_Factory_Multiclass *curb_factory)
  : MNM_Statistics_Lrn::MNM_Statistics_Lrn(file_folder, conf_reader, record_config, od_factory, node_factory, link_factory)
{
  	m_record_interval_cost_curb_car = std::unordered_map<TInt, TFlt>();
	m_record_interval_cost_curb_truck = std::unordered_map<TInt, TFlt>();
	m_record_interval_cost_curb_rh = std::unordered_map<TInt, TFlt>();

	m_load_interval_tt_car = std::unordered_map<TInt, TFlt>();
	m_record_interval_tt_car = std::unordered_map<TInt, TFlt>();
	m_to_be_tt_car = std::unordered_map<TInt, TFlt>();

	m_record_interval_tt_truck = std::unordered_map<TInt, TFlt>();
	m_load_interval_tt_truck = std::unordered_map<TInt, TFlt>();
	m_to_be_tt_truck = std::unordered_map<TInt, TFlt>();

	// m_current_curb_parking_num_map = std::unordered_map<TInt, std::vector<TFlt>>();
	// m_current_curb_cap_map = std::unordered_map<TInt, std::vector<TFlt>>;

	// m_curb_price_list = &curb_factory -> m_curb_price_list;
	m_adaptive_config = new MNM_ConfReader(file_folder + "/config.conf", "ADAPTIVE");
	m_curb_factory = curb_factory;

	m_unit_time_cost_car = m_adaptive_config -> get_float("unit_time_cost_car"); // $/5s
	m_unit_time_cost_truck = m_adaptive_config -> get_float("unit_time_cost_truck"); // $/5s
	m_unit_distance_cost_truck = m_adaptive_config -> get_float("unit_dist_cost_truck"); // $/m
	m_unit_time_cost_rh = m_adaptive_config -> get_float("unit_time_cost_rh"); // $/5s
	m_unit_distance_cost_rh = m_adaptive_config -> get_float("unit_dist_cost_rh"); // $/m
	// m_routing_freq = m_self_config -> get_int("route_frq");
}

MNM_Statistics_Curb::~MNM_Statistics_Curb()
{
    m_record_interval_cost_curb_car.clear();
	m_record_interval_cost_curb_truck.clear();
	m_record_interval_cost_curb_rh.clear();

	m_load_interval_tt_car.clear();
	m_record_interval_tt_car.clear();
	m_to_be_tt_car.clear();

	m_record_interval_tt_truck.clear();
	m_load_interval_tt_truck.clear();
	m_to_be_tt_truck.clear();
}

int MNM_Statistics_Curb::init_record_curb()
{
	TInt _link_ID;
	TInt _curb_ID;

	if (m_record_tt){
		for (auto _link_it : m_link_factory -> m_link_map){
			_link_ID = _link_it.first;
			
			m_load_interval_tt_car.insert(std::pair<TInt, TFlt>(_link_ID, TFlt(0)));
			m_record_interval_tt_car.insert(std::pair<TInt, TFlt>(_link_ID, TFlt(0)));
			m_to_be_tt_car.insert(std::pair<TInt, TFlt>(_link_ID, TFlt(0)));

			m_record_interval_tt_truck.insert(std::pair<TInt, TFlt>(_link_ID, TFlt(0)));
			m_load_interval_tt_truck.insert(std::pair<TInt, TFlt>(_link_ID, TFlt(0)));
			m_to_be_tt_truck.insert(std::pair<TInt, TFlt>(_link_ID, TFlt(0)));

			m_record_interval_cost_curb_car.insert(std::pair<TInt, TFlt>(_link_ID, TFlt(0)));
			m_record_interval_cost_curb_truck.insert(std::pair<TInt, TFlt>(_link_ID, TFlt(0)));
			m_record_interval_cost_curb_rh.insert(std::pair<TInt, TFlt>(_link_ID, TFlt(0)));
    	}

		for (auto _curb_it : m_curb_factory -> m_curb_capacity_map){
			_curb_ID = _curb_it.first;
			m_record_all_interval_parking_num_car.insert(std::pair<TInt, std::vector<TFlt>>(_curb_ID, {TFlt(0)}));
			m_record_all_interval_parking_num_truck.insert(std::pair<TInt, std::vector<TFlt>>(_curb_ID, {TFlt(0)}));
			m_record_all_interval_parking_num_rh.insert(std::pair<TInt, std::vector<TFlt>>(_curb_ID, {TFlt(0)}));

			m_record_interval_curb_cap_cost_car.insert(std::pair<TInt, TFlt>(_curb_ID, TFlt(0)));
			m_record_interval_curb_cap_cost_truck.insert(std::pair<TInt, TFlt>(_curb_ID, TFlt(0)));
			m_record_interval_curb_cap_cost_rh.insert(std::pair<TInt, TFlt>(_curb_ID, TFlt(0)));
		}
	}
	return 0;
}

int MNM_Statistics_Curb::update_record_curb(TInt timestamp)
{
	MNM_Dlink *_link;
	MNM_Dlink_Multiclass_Curb *_link_m;
  	
	TFlt _tt_int_car;
	TFlt _tt_int_truck;

	TFlt _flow_car;
	TFlt _flow_truck;

	TFlt _penalty_dp = 40.0; // penalty for double parking

	if (m_record_tt){
		if ((timestamp) % m_n == 0 || timestamp == 0){
			for (auto _link_it : m_link_factory -> m_link_map){

				_link = _link_it.second;
				_link_m = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(_link);
				// interval flow
				_flow_car = _link_m -> get_link_flow_car();
				_flow_truck = _link_m -> get_link_flow_truck();

				// intrval tt
				_tt_int_car = _link_m -> get_link_tt_from_flow_car(_flow_car);
				_tt_int_truck = _link_m -> get_link_tt_from_flow_truck(_flow_truck);
				
				m_load_interval_tt_car.find(_link -> m_link_ID) -> second = _tt_int_car;
				m_load_interval_tt_truck.find(_link -> m_link_ID) -> second = _tt_int_truck;

				if (timestamp == 0) {
					m_record_interval_tt_car.find(_link -> m_link_ID) -> second = _tt_int_car;
					m_record_interval_tt_truck.find(_link -> m_link_ID) -> second = _tt_int_truck;
				}
				else {
					m_record_interval_tt_car.find(_link -> m_link_ID) -> second = m_to_be_tt_car.find(_link -> m_link_ID) -> second + _tt_int_car/TFlt(m_n);
					m_record_interval_tt_truck.find(_link -> m_link_ID) -> second = m_to_be_tt_truck.find(_link -> m_link_ID) -> second + _tt_int_truck/TFlt(m_n);
				}
				// reset
				m_to_be_tt_car.find(_link -> m_link_ID) -> second = TFlt(0);
				m_to_be_tt_truck.find(_link -> m_link_ID) -> second = TFlt(0);

				m_record_interval_cost_curb_car.find(_link -> m_link_ID) -> second = m_unit_time_cost_car * (m_record_interval_tt_car.find(_link -> m_link_ID) -> second);

				m_record_interval_cost_curb_truck.find(_link -> m_link_ID) -> second = m_unit_time_cost_truck * (m_record_interval_tt_truck.find(_link -> m_link_ID) -> second)
																					+ m_unit_distance_cost_truck * (_link -> m_length);
				
				m_record_interval_cost_curb_rh.find(_link -> m_link_ID) -> second = m_unit_time_cost_rh * (m_record_interval_tt_car.find(_link -> m_link_ID) -> second)
																					+ m_unit_distance_cost_rh * (_link -> m_length);
			}

			// update average curb parking num
			for (auto _curb_it : m_curb_factory -> m_curb_capacity_map){
				TInt _curb_ID = _curb_it.first;
				_link = m_link_factory -> m_link_map.find(_curb_ID) -> second;
				_link_m = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(_link);

				if (timestamp == 0){
					m_record_interval_curb_cap_cost_car.find(_curb_ID) -> second = TFlt(0);
					m_record_interval_curb_cap_cost_truck.find(_curb_ID) -> second = TFlt(0);
					m_record_interval_curb_cap_cost_rh.find(_curb_ID) -> second = TFlt(0);

					assert((TInt)m_record_all_interval_parking_num_car.find(_curb_ID) -> second.size() == timestamp + 1);
					assert((TInt)m_record_all_interval_parking_num_truck.find(_curb_ID) -> second.size() == timestamp + 1);
					assert((TInt)m_record_all_interval_parking_num_rh.find(_curb_ID) -> second.size() == timestamp + 1);
				}
				else{
					// use the last n elements in m_record_all_interval_parking_num_XXX to take average and update m_record_interval_curb_cap_cost_XXX
					m_record_all_interval_parking_num_car.find(_curb_ID) -> second.push_back(TFlt(_link_m -> m_parking_num_car));
					m_record_all_interval_parking_num_truck.find(_curb_ID) -> second.push_back(TFlt(_link_m -> m_parking_num_truck));
					m_record_all_interval_parking_num_rh.find(_curb_ID) -> second.push_back(TFlt(_link_m -> m_parking_num_rh));
					
					assert((TInt)m_record_all_interval_parking_num_car.find(_curb_ID) -> second.size() == timestamp + 1);
					assert((TInt)m_record_all_interval_parking_num_truck.find(_curb_ID) -> second.size() == timestamp + 1);
					assert((TInt)m_record_all_interval_parking_num_rh.find(_curb_ID) -> second.size() == timestamp + 1);
					
					assert(timestamp >= m_n);
					TFlt _ave_car = 0.0;
					TFlt _ave_truck = 0.0;
					TFlt _ave_rh = 0.0;

					for (int i = 0; i < m_n; ++i){
						_ave_car += m_record_all_interval_parking_num_car.find(_curb_ID) -> second[timestamp - i] / TFlt(m_n);
						_ave_truck += m_record_all_interval_parking_num_truck.find(_curb_ID) -> second[timestamp - i] / TFlt(m_n);
						_ave_rh += m_record_all_interval_parking_num_rh.find(_curb_ID) -> second[timestamp - i] / TFlt(m_n);
					}

					if ((_ave_car + _ave_truck + _ave_rh) >= m_curb_factory -> m_curb_capacity_map.find(_curb_ID) -> second){
						m_record_interval_curb_cap_cost_car.find(_curb_ID) -> second = _penalty_dp;
						m_record_interval_curb_cap_cost_truck.find(_curb_ID) -> second = _penalty_dp;
						m_record_interval_curb_cap_cost_rh.find(_curb_ID) -> second = _penalty_dp;
					}
					else{
						// now all modes share the same prices (TODO: different prices for different modes)
						m_record_interval_curb_cap_cost_car.find(_curb_ID) -> second = TFlt(0);
						m_record_interval_curb_cap_cost_truck.find(_curb_ID) -> second = TFlt(0);
						m_record_interval_curb_cap_cost_rh.find(_curb_ID) -> second = TFlt(0);
					}
				}
			}
			
		}
		else{
			// do not update adaptive metrics
			for (auto _link_it : m_link_factory -> m_link_map){
				_link = _link_it.second;
				_link_m = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(_link);
				// interval flow
				_flow_car = _link_m -> get_link_flow_car();
				_flow_truck = _link_m -> get_link_flow_truck();
				
				// intrval tt
				_tt_int_car = _link_m -> get_link_tt_from_flow_car(_flow_car);
				_tt_int_truck = _link_m -> get_link_tt_from_flow_truck(_flow_truck);
				
				m_load_interval_tt_car.find(_link -> m_link_ID) -> second = _tt_int_car;
				m_load_interval_tt_truck.find(_link -> m_link_ID) -> second = _tt_int_truck;

				m_to_be_tt_car.find(_link -> m_link_ID) -> second += _tt_int_car/TFlt(m_n);
				m_to_be_tt_truck.find(_link -> m_link_ID) -> second += _tt_int_truck/TFlt(m_n);
			}

			// record curb parking num from link factory
			for (auto _curb_it : m_curb_factory -> m_curb_capacity_map){
				TInt _curb_ID = _curb_it.first;
				_link = m_link_factory -> m_link_map.find(_curb_ID) -> second;
				_link_m = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(_link);
				m_record_all_interval_parking_num_car.find(_curb_ID) -> second.push_back(TFlt(_link_m -> m_parking_num_car));
				m_record_all_interval_parking_num_truck.find(_curb_ID) -> second.push_back(TFlt(_link_m -> m_parking_num_truck));
				m_record_all_interval_parking_num_rh.find(_curb_ID) -> second.push_back(TFlt(_link_m -> m_parking_num_rh));

				assert((TInt)m_record_all_interval_parking_num_car.find(_curb_ID) -> second.size() == timestamp + 1);
				assert((TInt)m_record_all_interval_parking_num_truck.find(_curb_ID) -> second.size() == timestamp + 1);
				assert((TInt)m_record_all_interval_parking_num_rh.find(_curb_ID) -> second.size() == timestamp + 1);
			}
		}
	}
	return 0;
}

// Jiachao added in Oct.
/******************************************************************************************************************
*******************************************************************************************************************
										Multiclass Routing for curb
*******************************************************************************************************************
******************************************************************************************************************/
MNM_Routing_Driving_Fixed::MNM_Routing_Driving_Fixed(PNEGraph &graph,
													MNM_OD_Factory *od_factory, MNM_Node_Factory *node_factory,
													MNM_Link_Factory *link_factory, TInt routing_frq,
													TInt buffer_length, TInt veh_class)
	: MNM_Routing_Biclass_Fixed::MNM_Routing_Biclass_Fixed(graph, od_factory, node_factory, link_factory, routing_frq, buffer_length)
{
	m_buffer_length = buffer_length;
	m_veh_class = veh_class;
	m_inter_dest_map = new std::unordered_map<TInt, std::vector<TInt>>();
}

MNM_Routing_Driving_Fixed::~MNM_Routing_Driving_Fixed()
{
	;
}

// TODO Jiachao
int MNM_Routing_Driving_Fixed::update_routing_curb_driving(TInt timestamp, MNM_Curb_Factory_Multiclass* curb_factory)
{
	MNM_Origin *_origin;
	MNM_DMOND *_origin_node;
	TInt _node_ID, _next_link_ID;
	MNM_Dlink *_next_link;
	MNM_Veh *_veh;
	MNM_Veh_Multiclass_Curb *_veh_multiclass;
	TInt _cur_ass_int;
	
	std::vector<TInt> _inter_dest_info;
	m_inter_dest_map = &curb_factory -> m_path_inter_dest_map_car;

	if (m_buffer_as_p){
		_cur_ass_int = TInt(timestamp / m_routing_freq); // m_routing_freq = 180
		if (_cur_ass_int < TInt(m_buffer_length / 3)){  // first third for car, second for truck and last third for RH
			MNM::copy_buffer_to_p(m_path_table, _cur_ass_int);
  			MNM::normalize_path_table_p(m_path_table);
		}
	}

	// loop for each origin node
	for (auto _origin_it = m_od_factory -> m_origin_map.begin(); _origin_it != m_od_factory -> m_origin_map.end(); _origin_it++){
		_origin = _origin_it -> second;
		_origin_node = _origin -> m_origin_node;
		_node_ID = _origin_node -> m_node_ID;
		for (auto _veh_it = _origin_node -> m_in_veh_queue.begin(); _veh_it != _origin_node -> m_in_veh_queue.end(); _veh_it++){ // m_in_veh_queue has been shuffled
			_veh = *_veh_it;
			_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass_Curb *>(_veh);
		
			if ((_veh -> m_type == MNM_TYPE_STATIC) && (_veh_multiclass -> m_class == TInt(0))) {
				// new release vehs, no tracker
				if (m_tracker.find(_veh) == m_tracker.end()){
					register_veh_curb(_veh, true);
					_next_link_ID = m_tracker.find(_veh) -> second -> front();
					_next_link = m_link_factory -> get_link(_next_link_ID);
					_veh -> set_next_link(_next_link);
					

					TInt _path_ID = _veh -> m_path -> m_path_ID;

					auto _check = m_inter_dest_map -> find(_path_ID);
					if (_check != m_inter_dest_map -> end()){

						_inter_dest_info = (*m_inter_dest_map)[_path_ID];

						TInt _num_stops = TInt(_inter_dest_info.size()/2);

						if (_num_stops > 0){
							for (int i = 0; i < _num_stops; ++i){
								_veh_multiclass -> m_destination_list.push_back(_inter_dest_info[i]);
								// original 90
								// changed to 240 11/05
								std::vector<int> in = {180};	
								// std::vector<int> in = {60, 60, 60, 60, 120, 120, 120, 180};
								// std::random_shuffle(in.begin(), in.end());
								_veh_multiclass -> m_parking_duration_list.push_back(TInt(in.front()));
							}

							for (int j = _num_stops; j < int(_inter_dest_info.size()); ++j){
								_veh_multiclass -> m_curb_destination_list.push_back(_inter_dest_info[j]);
							}
						}
					}
					
					_inter_dest_info.clear();
					m_tracker.find(_veh) -> second -> pop_front();
				}
			}
			// according to Dr. Wei Ma, add a nominal path to adaptive users for DAR extraction, not rigorous, but will do the DODE job
			else if ((_veh -> m_type == MNM_TYPE_ADAPTIVE) && (_veh_multiclass -> m_class == TInt(0))) {
				if (_veh -> m_path == nullptr) {
					register_veh_curb(_veh, false);
				}
				IAssert(_veh -> m_path != nullptr);
			}
		}
	}

	MNM_Destination *_veh_dest;
	MNM_Dlink *_link;
	for (auto _link_it = m_link_factory -> m_link_map.begin(); _link_it != m_link_factory -> m_link_map.end(); _link_it ++){
		_link = _link_it -> second;
		_node_ID = _link -> m_to_node -> m_node_ID;
		for (auto _veh_it = _link -> m_finished_array.begin(); _veh_it != _link -> m_finished_array.end(); _veh_it++){
			_veh = *_veh_it;
			_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass_Curb *>(_veh);
			// Here is the difference from single-class fixed routing
			if ((_veh -> m_type == MNM_TYPE_STATIC) && (_veh_multiclass -> m_class == TInt(0))){
			// Here is the difference from single-class fixed routing

				_veh_dest = _veh -> get_destination();
				if (_veh_dest -> m_dest_node -> m_node_ID == _node_ID){
					if (m_tracker.find(_veh) -> second -> size() != 0){
						printf("Something wrong in fixed routing!\n");
						exit(-1);
					}
					_veh -> set_next_link(nullptr);
				}
				else{
					if (m_tracker.find(_veh) == m_tracker.end()){
						printf("Vehicle not registered in link, impossible!\n");
						exit(-1);
					}
					if (_veh -> get_current_link() == _veh -> get_next_link()){
						_next_link_ID = m_tracker.find(_veh) -> second -> front();
						if (_next_link_ID == -1){
							printf("Something wrong in routing, wrong next link 2\n");
							printf("The node is %d, the vehicle should head to %d\n", (int)_node_ID, (int)_veh_dest -> m_dest_node -> m_node_ID);
							exit(-1);
						}
						_next_link = m_link_factory -> get_link(_next_link_ID);
						_veh -> set_next_link(_next_link);
						m_tracker.find(_veh) -> second -> pop_front();
					}
				} //end if-else
			} //end if veh->m_type
		} //end for veh_it
	} //end for link_it
	return 0;

}

int MNM_Routing_Driving_Fixed::register_veh_curb(MNM_Veh* veh, bool track)
{
	// generate a random float number [0,1]
	TFlt _r = MNM_Ults::rand_flt();
	// printf("%d\n", veh -> get_origin() -> m_origin_node  -> m_node_ID);
	// printf("%d\n", veh -> get_destination() -> m_dest_node  -> m_node_ID);
	
	// path set for current OD pair
	MNM_Pathset *_pathset = m_path_table -> find(veh -> get_origin() -> m_origin_node  -> m_node_ID) -> second
							-> find(veh -> get_destination() -> m_dest_node  -> m_node_ID) -> second;
	MNM_Path *_route_path = nullptr;

	// note m_path_vec is an ordered vector, not unordered
	// loop for each path in the current pathset for the OD
	for (MNM_Path *_path : _pathset -> m_path_vec){
		if (_path -> m_p >= _r) {
			_route_path = _path;
			// should also set the intermediate destination and curb
			// (TODO)
			
			break;
		}
		else{
			_r -= _path -> m_p;
		}
	}
	// printf("3\n");
	if (_route_path == nullptr){
		printf("Wrong probability!\n");
		exit(-1);
	}
	if (track) {
		std::deque<TInt> *_link_queue = new std::deque<TInt>();
		std::copy(_route_path -> m_link_vec.begin(), _route_path -> m_link_vec.end(), std::back_inserter(*_link_queue));  // copy links in the route to _link_queue https://www.cplusplus.com/reference/iterator/back_inserter/
		// printf("old link q is %d, New link queue is %d\n", _route_path -> m_link_vec.size(), _link_queue -> size());
		m_tracker.insert(std::pair<MNM_Veh*, std::deque<TInt>*>(veh, _link_queue));
	}
	veh -> m_path = _route_path;
	return 0;
}


int MNM_Routing_Driving_Fixed::update_routing(TInt timestamp)
{
	return 0;
}

MNM_Routing_Truck_Fixed::MNM_Routing_Truck_Fixed(PNEGraph &graph,
												MNM_OD_Factory *od_factory, MNM_Node_Factory *node_factory,
												MNM_Link_Factory *link_factory, TInt routing_frq,
												TInt buffer_length, TInt veh_class)
	: MNM_Routing_Biclass_Fixed::MNM_Routing_Biclass_Fixed(graph, od_factory, node_factory, link_factory, routing_frq, buffer_length)
{
	m_buffer_length = buffer_length;
	m_veh_class = veh_class;
	m_inter_dest_map = new std::unordered_map<TInt, std::vector<TInt>>();
}

MNM_Routing_Truck_Fixed::~MNM_Routing_Truck_Fixed()
{
	;
}

int MNM_Routing_Truck_Fixed::update_routing_curb_truck(TInt timestamp, MNM_Curb_Factory_Multiclass* curb_factory)
{
	MNM_Origin *_origin;
	MNM_DMOND *_origin_node;
	TInt _node_ID, _next_link_ID;
	MNM_Dlink *_next_link;
	MNM_Veh *_veh;
	MNM_Veh_Multiclass_Curb *_veh_multiclass;
	TInt _cur_ass_int;

	std::vector<TInt> _inter_dest_info;
	m_inter_dest_map = &curb_factory -> m_path_inter_dest_map;

	if (m_buffer_as_p){
		_cur_ass_int = TInt(timestamp / m_routing_freq);
		if (_cur_ass_int < TInt(m_buffer_length / 3)){  // first half for car, last half for truck
			MNM::copy_buffer_to_p(m_path_table, _cur_ass_int);
  			MNM::normalize_path_table_p(m_path_table);
		}
	}

	// loop for each origin node
	for (auto _origin_it = m_od_factory -> m_origin_map.begin(); _origin_it != m_od_factory -> m_origin_map.end(); _origin_it++){
		_origin = _origin_it -> second;
		_origin_node = _origin -> m_origin_node;
		_node_ID = _origin_node -> m_node_ID;
		for (auto _veh_it = _origin_node -> m_in_veh_queue.begin(); _veh_it != _origin_node -> m_in_veh_queue.end(); _veh_it++){ // m_in_veh_queue has been shuffled
			_veh = *_veh_it;
			_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass_Curb *>(_veh);

			if ((_veh -> m_type == MNM_TYPE_STATIC) && (_veh_multiclass -> m_class == TInt(1))) {

				if (m_tracker.find(_veh) == m_tracker.end()){
				// printf("Registering!\n");
					register_veh_curb(_veh, true);
					_next_link_ID = m_tracker.find(_veh) -> second -> front();
					_next_link = m_link_factory -> get_link(_next_link_ID);
					_veh -> set_next_link(_next_link);

					TInt _path_ID = _veh -> m_path -> m_path_ID;

					auto _check = m_inter_dest_map -> find(_path_ID);
					if (_check != m_inter_dest_map -> end()){

						_inter_dest_info = (*m_inter_dest_map)[_path_ID];

						TInt _num_stops = TInt(_inter_dest_info.size()/2);

						if (_num_stops > 0){
							for (int i = 0; i < _num_stops; ++i){
								_veh_multiclass -> m_destination_list.push_back(_inter_dest_info[i]);
								// original 60
								// changed to 240 11/05
								std::vector<int> in = {120};	
								// std::vector<int> in = {60, 60, 60, 60, 120, 120, 120, 180};
								// std::random_shuffle(in.begin(), in.end());
								_veh_multiclass -> m_parking_duration_list.push_back(TInt(in.front()));
							}

							for (int j = _num_stops; j < int(_inter_dest_info.size()); ++j){
								_veh_multiclass -> m_curb_destination_list.push_back(_inter_dest_info[j]);
							}
						}
					}
					
					_inter_dest_info.clear();
					m_tracker.find(_veh) -> second -> pop_front();
				}
			}
			// according to Dr. Wei Ma, add a nominal path to adaptive users for DAR extraction, not rigorous, but will do the DODE job
			else if ((_veh -> m_type == MNM_TYPE_ADAPTIVE) && (_veh_multiclass -> m_class == TInt(1))) {
				if (_veh -> m_path == nullptr) {
					register_veh_curb(_veh, false);
				}
				IAssert(_veh -> m_path != nullptr);
			}
		}
	}

	MNM_Destination *_veh_dest;
	MNM_Dlink *_link;
	for (auto _link_it = m_link_factory -> m_link_map.begin(); _link_it != m_link_factory -> m_link_map.end(); _link_it ++){
		_link = _link_it -> second;
		_node_ID = _link -> m_to_node -> m_node_ID;
		for (auto _veh_it = _link -> m_finished_array.begin(); _veh_it != _link -> m_finished_array.end(); _veh_it++){
			_veh = *_veh_it;
			_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass_Curb *>(_veh);

			// Here is the difference from single-class fixed routing
			if ((_veh -> m_type == MNM_TYPE_STATIC) && (_veh_multiclass -> m_class == TInt(1))){
			// Here is the difference from single-class fixed routing

				_veh_dest = _veh -> get_destination();
				if (_veh_dest -> m_dest_node -> m_node_ID == _node_ID){
					if (m_tracker.find(_veh) -> second -> size() != 0){
						printf("Something wrong in fixed routing!\n");
						exit(-1);
					}
					_veh -> set_next_link(nullptr);
				}
				else{
					if (m_tracker.find(_veh) == m_tracker.end()){
						printf("Vehicle not registered in link, impossible!\n");
						exit(-1);
					}
					if (_veh -> get_current_link() == _veh -> get_next_link()){
						_next_link_ID = m_tracker.find(_veh) -> second -> front();
						if (_next_link_ID == -1){
							printf("Something wrong in routing, wrong next link 2\n");
							printf("The node is %d, the vehicle should head to %d\n", (int)_node_ID, (int)_veh_dest -> m_dest_node -> m_node_ID);
							exit(-1);
						}
						_next_link = m_link_factory -> get_link(_next_link_ID);
						_veh -> set_next_link(_next_link);
						m_tracker.find(_veh) -> second -> pop_front();
					}
				} //end if-else
			} //end if veh->m_type
		} //end for veh_it
	} //end for link_it
	return 0;
}

int MNM_Routing_Truck_Fixed::update_routing(TInt timestamp)
{
	return 0;
}

int MNM_Routing_Truck_Fixed::register_veh_curb(MNM_Veh* veh, bool track)
{
	// generate a random float number [0,1]
	TFlt _r = MNM_Ults::rand_flt();
	// printf("%d\n", veh -> get_origin() -> m_origin_node  -> m_node_ID);
	// printf("%d\n", veh -> get_destination() -> m_dest_node  -> m_node_ID);
	
	// path set for current OD pair
	MNM_Pathset *_pathset = m_path_table -> find(veh -> get_origin() -> m_origin_node  -> m_node_ID) -> second
							-> find(veh -> get_destination() -> m_dest_node  -> m_node_ID) -> second;
	MNM_Path *_route_path = nullptr;

	// note m_path_vec is an ordered vector, not unordered
	// loop for each path in the current pathset for the OD
	for (MNM_Path *_path : _pathset -> m_path_vec){
		if (_path -> m_p >= _r) {
			_route_path = _path;
			break;
		}
		else{
			_r -= _path -> m_p;
		}
	}
	// printf("3\n");
	if (_route_path == nullptr){
		printf("Wrong probability!\n");
		exit(-1);
	}
	if (track) {
		std::deque<TInt> *_link_queue = new std::deque<TInt>();
		std::copy(_route_path -> m_link_vec.begin(), _route_path -> m_link_vec.end(), std::back_inserter(*_link_queue));  // copy links in the route to _link_queue https://www.cplusplus.com/reference/iterator/back_inserter/
		// printf("old link q is %d, New link queue is %d\n", _route_path -> m_link_vec.size(), _link_queue -> size());
		m_tracker.insert(std::pair<MNM_Veh*, std::deque<TInt>*>(veh, _link_queue));
	}
	veh -> m_path = _route_path;
	return 0;
}

MNM_Routing_Ridehail_Fixed::MNM_Routing_Ridehail_Fixed(PNEGraph &graph,
              											MNM_OD_Factory *od_factory, MNM_Node_Factory *node_factory, 
              											MNM_Link_Factory *link_factory, TInt routing_frq, TInt buffer_length, TInt veh_class)
	: MNM_Routing_Biclass_Fixed::MNM_Routing_Biclass_Fixed(graph, od_factory, node_factory, link_factory, routing_frq, buffer_length)
{
    m_buffer_length = buffer_length;
    m_veh_class = veh_class;
	m_inter_dest_map = new std::unordered_map<TInt, std::vector<TInt>>();
}

MNM_Routing_Ridehail_Fixed::~MNM_Routing_Ridehail_Fixed()
{
    ;
}

int MNM_Routing_Ridehail_Fixed::update_routing_curb_rh(TInt timestamp, MNM_Curb_Factory_Multiclass* curb_factory)
{
	MNM_Origin *_origin;
	MNM_DMOND *_origin_node;
	TInt _node_ID, _next_link_ID;
	MNM_Dlink *_next_link;
	MNM_Veh *_veh;
	MNM_Veh_Multiclass_Curb *_veh_multiclass;
	TInt _cur_ass_int;

	std::vector<TInt> _inter_dest_info;
	m_inter_dest_map = &curb_factory -> m_path_inter_dest_map_rh;

	// printf("my buffer length %d, %d\n", m_buffer_length(), m_routing_freq());
	if (m_buffer_as_p){
		_cur_ass_int = TInt(timestamp / m_routing_freq);
		if (_cur_ass_int < TInt(m_buffer_length / 3)){  // first half for car, last half for truck
			// printf("Changing biclass fixed\n");
			// change_choice_portion(_cur_ass_int);
			// MNM::copy_buffer_to_p(m_path_table, _cur_ass_int + m_veh_class * TInt(m_buffer_length / 3));
  			// printf("Finish copying\n");
  			// MNM::normalize_path_table_p(m_path_table);
			MNM::copy_buffer_to_p(m_path_table, _cur_ass_int);

  			MNM::normalize_path_table_p(m_path_table);
		}
	}

	// loop for each origin node
	for (auto _origin_it = m_od_factory -> m_origin_map.begin(); _origin_it != m_od_factory -> m_origin_map.end(); _origin_it++){
		_origin = _origin_it -> second;
		_origin_node = _origin -> m_origin_node;
		_node_ID = _origin_node -> m_node_ID;
		for (auto _veh_it = _origin_node -> m_in_veh_queue.begin(); _veh_it != _origin_node -> m_in_veh_queue.end(); _veh_it++){ // m_in_veh_queue has been shuffled
			_veh = *_veh_it;
			_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass_Curb *>(_veh);

			if ((_veh -> m_type == MNM_TYPE_STATIC) && (_veh_multiclass -> m_class == TInt(2))) {
			// Here is the difference from single-class fixed routing

				if (m_tracker.find(_veh) == m_tracker.end()){
				// printf("Registering!\n");
					register_veh_curb(_veh, true);
					_next_link_ID = m_tracker.find(_veh) -> second -> front();
					_next_link = m_link_factory -> get_link(_next_link_ID);
					_veh -> set_next_link(_next_link);

					TInt _path_ID = _veh -> m_path -> m_path_ID;

					auto _check = m_inter_dest_map -> find(_path_ID);
					if (_check != m_inter_dest_map -> end()){

						_inter_dest_info = (*m_inter_dest_map)[_path_ID];

						TInt _num_stops = TInt(_inter_dest_info.size()/2);

						for (int i = 0; i < _num_stops; ++i){
							_veh_multiclass -> m_destination_list.push_back(_inter_dest_info[i]);
							// change duration 0628 RH (20)
							// change duration 1105 to be longer for modeling long parking (240)
							std::vector<int> in = {60};
							// std::vector<int> in = {12, 12, 12, 12, 12, 12, 24, 24, 24, 24, 36, 36, 48, 60};
							// std::random_shuffle(in.begin(), in.end());
							_veh_multiclass -> m_parking_duration_list.push_back(TInt(in.front()));
						}

						for (int j = _num_stops; j < int(_inter_dest_info.size()); ++j){
							_veh_multiclass -> m_curb_destination_list.push_back(_inter_dest_info[j]);
						}
					}
					
					_inter_dest_info.clear();

					m_tracker.find(_veh) -> second -> pop_front();
				}
			}
			// according to Dr. Wei Ma, add a nominal path to adaptive users for DAR extraction, not rigorous, but will do the DODE job
			else if ((_veh -> m_type == MNM_TYPE_ADAPTIVE) && (_veh_multiclass -> m_class == TInt(2))) {
				if (_veh -> m_path == nullptr) {
					register_veh_curb(_veh, false);
				}
				IAssert(_veh -> m_path != nullptr);
			}
		}
	}

	MNM_Destination *_veh_dest;
	MNM_Dlink *_link;
	for (auto _link_it = m_link_factory -> m_link_map.begin(); _link_it != m_link_factory -> m_link_map.end(); _link_it ++){
		_link = _link_it -> second;
		_node_ID = _link -> m_to_node -> m_node_ID;
		for (auto _veh_it = _link -> m_finished_array.begin(); _veh_it != _link -> m_finished_array.end(); _veh_it++){
			_veh = *_veh_it;
			_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass_Curb *>(_veh);
			// Here is the difference from single-class fixed routing
			if ((_veh -> m_type == MNM_TYPE_STATIC) && (_veh_multiclass -> m_class == TInt(2))){
			// Here is the difference from single-class fixed routing

				_veh_dest = _veh -> get_destination();
				if (_veh_dest -> m_dest_node -> m_node_ID == _node_ID){
					if (m_tracker.find(_veh) -> second -> size() != 0){
						printf("Something wrong in fixed routing!\n");
						exit(-1);
					}
					_veh -> set_next_link(nullptr);
				}
				else{
					if (m_tracker.find(_veh) == m_tracker.end()){
						printf("Vehicle not registered in link, impossible!\n");
						exit(-1);
					}
					if (_veh -> get_current_link() == _veh -> get_next_link()){
						_next_link_ID = m_tracker.find(_veh) -> second -> front();
						if (_next_link_ID == -1){
							printf("Something wrong in routing, wrong next link 2\n");
							printf("The node is %d, the vehicle should head to %d\n", (int)_node_ID, (int)_veh_dest -> m_dest_node -> m_node_ID);
							exit(-1);
						}
						_next_link = m_link_factory -> get_link(_next_link_ID);
						_veh -> set_next_link(_next_link);
						m_tracker.find(_veh) -> second -> pop_front();
					}
				} //end if-else

				// if (_veh -> m_veh_ID == TInt(46)){
				// 	printf("Veh 46 current link = %d and next link = %d\n", (int)_veh -> m_current_link -> m_link_ID, (int)_veh -> m_next_link -> m_link_ID);
				// }

			} //end if veh->m_type
		} //end for veh_it
	} //end for link_it
	return 0;
}

int MNM_Routing_Ridehail_Fixed::update_routing(TInt timestamp)
{
	return 0;
}

int MNM_Routing_Ridehail_Fixed::register_veh_curb(MNM_Veh* veh, bool track)
{
	// generate a random float number [0,1]
	TFlt _r = MNM_Ults::rand_flt();
	// printf("%d\n", veh -> get_origin() -> m_origin_node  -> m_node_ID);
	// printf("%d\n", veh -> get_destination() -> m_dest_node  -> m_node_ID);
	
	// path set for current OD pair
	MNM_Pathset *_pathset = m_path_table -> find(veh -> get_origin() -> m_origin_node  -> m_node_ID) -> second
							-> find(veh -> get_destination() -> m_dest_node  -> m_node_ID) -> second;
	MNM_Path *_route_path = nullptr;

	// note m_path_vec is an ordered vector, not unordered
	// loop for each path in the current pathset for the OD
	for (MNM_Path *_path : _pathset -> m_path_vec){
		if (_path -> m_p >= _r) {
			_route_path = _path;
			
			break;
		}
		else{
			_r -= _path -> m_p;
		}
	}
	// printf("3\n");
	if (_route_path == nullptr){
		printf("Wrong probability!\n");
		exit(-1);
	}
	if (track) {
		std::deque<TInt> *_link_queue = new std::deque<TInt>();
		std::copy(_route_path -> m_link_vec.begin(), _route_path -> m_link_vec.end(), std::back_inserter(*_link_queue));  // copy links in the route to _link_queue https://www.cplusplus.com/reference/iterator/back_inserter/
		// printf("old link q is %d, New link queue is %d\n", _route_path -> m_link_vec.size(), _link_queue -> size());
		m_tracker.insert(std::pair<MNM_Veh*, std::deque<TInt>*>(veh, _link_queue));
	}
	veh -> m_path = _route_path;
	return 0;
}

MNM_Routing_Curb_Adaptive::MNM_Routing_Curb_Adaptive(const std::string& file_folder, 
							  PNEGraph &graph, 
							  MNM_Statistics* statistics,
							  MNM_Statistics_Curb* statistics_curb,
                       		  MNM_OD_Factory *od_factory, 
							  MNM_Node_Factory *node_factory, 
							  MNM_Link_Factory *link_factory,
							  MNM_Curb_Factory_Multiclass *curb_factory)
	: MNM_Routing_Adaptive::MNM_Routing_Adaptive(file_folder, graph, statistics, od_factory, node_factory, link_factory)
{
	m_statistics = statistics;
	m_statistics_curb = statistics_curb;
	m_self_config = new MNM_ConfReader(file_folder + "/config.conf", "ADAPTIVE");
	m_routing_freq = m_self_config -> get_int("route_frq");

	m_table = new Routing_Table();
	m_table_truck = new Routing_Table();
	m_table_rh = new Routing_Table();

	m_curb_factory = curb_factory;

	std::vector<TInt> m_intermediate_dest_list;

	m_file_folder = file_folder;
}

MNM_Routing_Curb_Adaptive::~MNM_Routing_Curb_Adaptive()
{
    ;
}

// Jiachao comments no need to implement
int MNM_Routing_Curb_Adaptive::init_routing(Path_Table *path_table)
{
	return 0;
}

// Jiachao (TODO)
int MNM_Routing_Curb_Adaptive::init_routing_curb()
{
	std::unordered_map<TInt, TInt> *_shortest_path_tree;
	std::unordered_map<TInt, TInt> *_shortest_path_tree_truck;
	std::unordered_map<TInt, TInt> *_shortest_path_tree_rh;

	// (DONE) load m_intermediate_dest_list
	// std::ifstream _inter_dest_file;

	// _inter_dest_file.open(m_file_folder + "/inter_dest", std::ios::in);

	// /* read file */
	// std::string _line;
	// std::vector<std::string> _words;

	// TInt _origin_node_ID, _dest_node_ID, _inter_node_ID;

	// if (_inter_dest_file.is_open()){
	// 	for (int i = 0; i < m_num_inter_dest; ++i){
	// 		std::getline(_inter_dest_file, _line);
	// 		_words = MNM_IO::split(_line, ' ');

	// 		if (_words.size() > 2){
	// 			for (int j = 2; j < int(_words.size()); ++j){
	// 				_inter_node_ID = TInt(std::stoi(_words[j]));
	// 				if (std::find(m_curb_factory -> m_interdest_list.begin(), m_curb_factory -> m_interdest_list.end(), _inter_node_ID) == m_curb_factory -> m_interdest_list.end()){
	// 					m_curb_factory -> m_interdest_list.push_back(_inter_node_ID);
	// 				}
	// 			}
	// 		}
	// 	}
	// }

	// (DONE) here we need not only destination node but also intermediate destination 
	for (auto _it = m_od_factory -> m_destination_map.begin(); _it != m_od_factory -> m_destination_map.end(); _it++){
		_shortest_path_tree = new std::unordered_map<TInt, TInt>();
		_shortest_path_tree_truck = new std::unordered_map<TInt, TInt>();
		_shortest_path_tree_rh = new std::unordered_map<TInt, TInt>();

		// Jiachao comment: _shortest_path_tree.first = node_ID, second = next_link_ID on shortest path
		m_table -> insert(std::pair<MNM_Destination*, std::unordered_map<TInt, TInt>*>(_it -> second, _shortest_path_tree));
		m_table_truck -> insert(std::pair<MNM_Destination*, std::unordered_map<TInt, TInt>*>(_it -> second, _shortest_path_tree_truck));
		m_table_rh -> insert(std::pair<MNM_Destination*, std::unordered_map<TInt, TInt>*>(_it -> second, _shortest_path_tree_rh));
		
	}

	for (auto _it_2 = m_curb_factory -> m_interdest_map.begin(); _it_2 != m_curb_factory -> m_interdest_map.end(); _it_2++){
		// _shortest_path_tree = new std::unordered_map<TInt, TInt>();
		_shortest_path_tree_truck = new std::unordered_map<TInt, TInt>();
		_shortest_path_tree_rh = new std::unordered_map<TInt, TInt>();

		// Jiachao comment: _shortest_path_tree.first = node_ID, second = next_link_ID on shortest path
		m_table -> insert(std::pair<MNM_Destination*, std::unordered_map<TInt, TInt>*>(_it_2 -> second, _shortest_path_tree));
		m_table_truck -> insert(std::pair<MNM_Destination*, std::unordered_map<TInt, TInt>*>(_it_2 -> second, _shortest_path_tree_truck));
		m_table_rh -> insert(std::pair<MNM_Destination*, std::unordered_map<TInt, TInt>*>(_it_2 -> second, _shortest_path_tree_rh));
		
	}
	return 0;
}

// Jiachao comment: no need to implement
int MNM_Routing_Curb_Adaptive::update_routing(TInt timestamp)
{
	return 0;
}

int MNM_Routing_Curb_Adaptive::update_statistics_dest(TInt _dest_node_ID, TInt _veh_type)
{
	std::vector<TInt> _curb_list;
	std::vector<TFlt> _curb_price_list;
	TFlt _curb_price;
	TFlt _curb_occ_cost;
	TInt _curb;
	TFlt _old_price;

	/* add curb prices and curb occupancy cost */
	if (_veh_type == TInt(0)){
		_curb_list = m_curb_factory -> m_interdest_curb_map.find(_dest_node_ID) -> second;
		for (int i = 0; i < int(_curb_list.size()); ++i){
			_curb = _curb_list[i];
			_curb_price_list = m_curb_factory -> m_curb_price_list.find(_curb) -> second;
			if (int(_curb_price_list.size()) == 1){
				_curb_price = _curb_price_list[0]; // use the same curb prices for all vehicles
			}
			else{
				printf("not implement multi-modal time-varying pricing\n");
			}
			// add curb occupancy cost
			_curb_occ_cost = m_statistics_curb -> m_record_interval_curb_cap_cost_car.find(_curb) -> second;

			_old_price = m_statistics_curb -> m_record_interval_cost_curb_car.find(_curb) -> second;

			m_statistics_curb -> m_record_interval_cost_curb_car.find(_curb) -> second = _old_price + _curb_price + _curb_occ_cost;
		}
	}

	if (_veh_type == TInt(1)){
		_curb_list = m_curb_factory -> m_interdest_curb_map.find(_dest_node_ID) -> second;
		for (int i = 0; i < int(_curb_list.size()); ++i){
			_curb = _curb_list[i];
			_curb_price_list = m_curb_factory -> m_curb_price_list.find(_curb) -> second;
			if (int(_curb_price_list.size()) == 1){
				_curb_price = _curb_price_list[0]; // use the same curb prices for all vehicles
			}
			else{
				printf("not implement multi-modal time-varying pricing\n");
			}
			// add curb occupancy cost
			_curb_occ_cost = m_statistics_curb -> m_record_interval_curb_cap_cost_truck.find(_curb) -> second;

			_old_price = m_statistics_curb -> m_record_interval_cost_curb_truck.find(_curb) -> second;

			m_statistics_curb -> m_record_interval_cost_curb_truck.find(_curb) -> second = _old_price + _curb_price + _curb_occ_cost;
		}
	}

	if (_veh_type == TInt(2)){
		_curb_list = m_curb_factory -> m_interdest_curb_map.find(_dest_node_ID) -> second;
		for (int i = 0; i < int(_curb_list.size()); ++i){
			_curb = _curb_list[i];
			_curb_price_list = m_curb_factory -> m_curb_price_list.find(_curb) -> second;
			if (int(_curb_price_list.size()) == 1){
				_curb_price = _curb_price_list[0]; // use the same curb prices for all vehicles
			}
			else{
				printf("not implement multi-modal time-varying pricing\n");
			}
			// add curb occupancy cost
			_curb_occ_cost = m_statistics_curb -> m_record_interval_curb_cap_cost_rh.find(_curb) -> second;

			_old_price = m_statistics_curb -> m_record_interval_cost_curb_rh.find(_curb) -> second;

			m_statistics_curb -> m_record_interval_cost_curb_rh.find(_curb) -> second = _old_price + _curb_price + _curb_occ_cost;
		}
	}

	return 0;
}

int MNM_Routing_Curb_Adaptive::return_statistics_dest(TInt _dest_node_ID, TInt _veh_type)
{
	std::vector<TInt> _curb_list;
	std::vector<TFlt> _curb_price_list;
	TFlt _curb_price;
	TInt _curb;
	TFlt _old_price;
	TFlt _curb_occ_cost;

	if (_veh_type == TInt(0)){
		_curb_list = m_curb_factory -> m_interdest_curb_map.find(_dest_node_ID) -> second;
		for (int i = 0; i < int(_curb_list.size()); ++i){
			_curb = _curb_list[i];
			_curb_price_list = m_curb_factory -> m_curb_price_list.find(_curb) -> second;
			if (int(_curb_price_list.size()) == 1){
				_curb_price = _curb_price_list[0];
			}
			else{
				printf("not implement time-varying pricing\n");
			}
			// add curb occupancy cost
			_curb_occ_cost = m_statistics_curb -> m_record_interval_curb_cap_cost_car.find(_curb) -> second;

			_old_price = m_statistics_curb -> m_record_interval_cost_curb_car.find(_curb) -> second;

			m_statistics_curb -> m_record_interval_cost_curb_car.find(_curb) -> second = _old_price - _curb_price - _curb_occ_cost;
		}
	}

	if (_veh_type == TInt(1)){
		_curb_list = m_curb_factory -> m_interdest_curb_map.find(_dest_node_ID) -> second;
		for (int i = 0; i < int(_curb_list.size()); ++i){
			_curb = _curb_list[i];
			_curb_price_list = m_curb_factory -> m_curb_price_list.find(_curb) -> second;
			if (int(_curb_price_list.size()) == 1){
				_curb_price = _curb_price_list[0];
			}
			else{
				printf("not implement time-varying pricing\n");
			}
			// add curb occupancy cost
			_curb_occ_cost = m_statistics_curb -> m_record_interval_curb_cap_cost_truck.find(_curb) -> second;

			_old_price = m_statistics_curb -> m_record_interval_cost_curb_truck.find(_curb) -> second;

			m_statistics_curb -> m_record_interval_cost_curb_truck.find(_curb) -> second = _old_price - _curb_price - _curb_occ_cost;
		}
	}

	if (_veh_type == TInt(2)){
		_curb_list = m_curb_factory -> m_interdest_curb_map.find(_dest_node_ID) -> second;
		for (int i = 0; i < int(_curb_list.size()); ++i){
			_curb = _curb_list[i];
			_curb_price_list = m_curb_factory -> m_curb_price_list.find(_curb) -> second;
			if (int(_curb_price_list.size()) == 1){
				_curb_price = _curb_price_list[0];
			}
			else{
				printf("not implement time-varying pricing\n");
			}
			// add curb occupancy cost
			_curb_occ_cost = m_statistics_curb -> m_record_interval_curb_cap_cost_rh.find(_curb) -> second;

			_old_price = m_statistics_curb -> m_record_interval_cost_curb_rh.find(_curb) -> second;

			m_statistics_curb -> m_record_interval_cost_curb_rh.find(_curb) -> second = _old_price - _curb_price - _curb_occ_cost;
		}
	}
	return 0;
}

/**************************************************************************************************************************************************

Jiachao defined new function for adaptive traveler with curb usage needs

1. adaptive routing for curb usage: add if condition to check veh is curb user



**************************************************************************************************************************************************/

int MNM_Routing_Curb_Adaptive::update_routing_curb_adaptive(TInt timestamp)
{
	MNM_Destination *_dest;
	TInt _dest_node_ID;
	std::unordered_map<TInt, TInt> *_shortest_path_tree;
	std::unordered_map<TInt, TInt> *_shortest_path_tree_truck;
	std::unordered_map<TInt, TInt> *_shortest_path_tree_rh;

	// update m_tables
	if ((timestamp) % m_routing_freq  == 0 || timestamp == 0) {
		// final destinations
		for (auto _it = m_od_factory -> m_destination_map.begin(); _it != m_od_factory -> m_destination_map.end(); _it++){
			_dest = _it -> second;
			_dest_node_ID = _dest -> m_dest_node -> m_node_ID;
			_shortest_path_tree = m_table -> find(_dest) -> second;
			_shortest_path_tree_truck = m_table_truck -> find(_dest) -> second;
			_shortest_path_tree_rh = m_table_rh -> find(_dest) -> second;
			MNM_Shortest_Path::all_to_one_Dijkstra(_dest_node_ID, m_graph, m_statistics_curb -> m_record_interval_cost_curb_car, *_shortest_path_tree);
			MNM_Shortest_Path::all_to_one_Dijkstra(_dest_node_ID, m_graph, m_statistics_curb -> m_record_interval_cost_curb_truck, *_shortest_path_tree_truck);
			MNM_Shortest_Path::all_to_one_Dijkstra(_dest_node_ID, m_graph, m_statistics_curb -> m_record_interval_cost_curb_rh, *_shortest_path_tree_rh);
		}

		// inter destinations
		for (auto _it = m_curb_factory -> m_interdest_map.begin(); _it != m_curb_factory -> m_interdest_map.end(); _it++){
			_dest = _it -> second;
			_dest_node_ID = _dest -> m_Dest_ID;

			_shortest_path_tree = m_table -> find(_dest) -> second;
			_shortest_path_tree_truck = m_table_truck -> find(_dest) -> second;
			_shortest_path_tree_rh = m_table_rh -> find(_dest) -> second;

			// add curb prices for inter-destinations
			update_statistics_dest(_dest_node_ID, TInt(0));
			MNM_Shortest_Path::all_to_one_Dijkstra(_dest_node_ID, m_graph, m_statistics_curb -> m_record_interval_cost_curb_car, *_shortest_path_tree);
			return_statistics_dest(_dest_node_ID, TInt(0));

			update_statistics_dest(_dest_node_ID, TInt(1));
			MNM_Shortest_Path::all_to_one_Dijkstra(_dest_node_ID, m_graph, m_statistics_curb -> m_record_interval_cost_curb_truck, *_shortest_path_tree_truck);
			return_statistics_dest(_dest_node_ID, TInt(1));

			update_statistics_dest(_dest_node_ID, TInt(2));
			MNM_Shortest_Path::all_to_one_Dijkstra(_dest_node_ID, m_graph, m_statistics_curb -> m_record_interval_cost_curb_rh, *_shortest_path_tree_rh);
			return_statistics_dest(_dest_node_ID, TInt(2));
		}
	}

	/* route the vehicle in Origin nodes */
	MNM_Origin *_origin;
	MNM_DMOND *_origin_node;
	TInt _node_ID, _next_link_ID;
	MNM_Dlink *_next_link;
	MNM_Veh *_veh;
	MNM_Veh_Multiclass_Curb *_veh_multiclass;
	TInt _interdest_ID;
	MNM_Destination *_interdest_node;

	/* for each origin node */
	for (auto _origin_it = m_od_factory->m_origin_map.begin(); _origin_it != m_od_factory->m_origin_map.end(); _origin_it++){
		_origin = _origin_it -> second;
		_origin_node = _origin -> m_origin_node;
		_node_ID = TInt(_origin_node -> m_node_ID);

		/* for each new released vehs waiting for routes */
		for (auto _veh_it = _origin_node -> m_in_veh_queue.begin(); _veh_it!=_origin_node -> m_in_veh_queue.end(); _veh_it++){
			
			_veh = *_veh_it;
			_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass_Curb *>(_veh);

			/* check if this veh needs to use curb */
			if (_veh_multiclass -> m_type == MNM_TYPE_ADAPTIVE_CURB){

				/* new released vehs cannot have empty m_destination_list */
				assert(_veh_multiclass -> m_destination_list.size() != 0);
				
				/* first inter-dest */
				_interdest_ID = _veh_multiclass -> m_destination_list.front();

				_interdest_node = m_curb_factory -> m_interdest_map.find(_interdest_ID) -> second;

				if (_veh_multiclass -> m_class == TInt(0)){
					_next_link_ID = m_table -> find(_interdest_node) -> second -> find(_node_ID) -> second;
				}

				if (_veh_multiclass -> m_class == TInt(1)){
					_next_link_ID = m_table_truck -> find(_interdest_node) -> second -> find(_node_ID) -> second;
				}

				if (_veh_multiclass -> m_class == TInt(2)){
					_next_link_ID = m_table_rh -> find(_interdest_node) -> second -> find(_node_ID) -> second;
				}

				if (_next_link_ID < 0){
					printf("Something wrong in adaptive routing curb, wrong next link\n");
					exit(-1);
				}

				_next_link = m_link_factory -> get_link(_next_link_ID);
				_veh -> set_next_link(_next_link);
				// _veh_multiclass -> m
			}
			else if (_veh_multiclass -> m_type == MNM_TYPE_ADAPTIVE){
				/* cannot be RH vehs */
				assert(_veh_multiclass -> m_class != TInt(2));
				if (_veh_multiclass -> m_class == TInt(0)){
					_next_link_ID = m_table -> find(_veh_multiclass -> get_destination()) -> second -> find(_node_ID) -> second;
					_next_link = m_link_factory -> get_link(_next_link_ID);
					_veh -> set_next_link(_next_link);
				}
				else if (_veh_multiclass -> m_class == TInt(1)){
					_next_link_ID = m_table_truck -> find(_veh_multiclass -> get_destination()) -> second -> find(_node_ID) -> second;
					_next_link = m_link_factory -> get_link(_next_link_ID);
					_veh -> set_next_link(_next_link);
				}
			}
		}
	}  

	MNM_Destination *_veh_dest;
	MNM_Dlink *_link;

	/* for each link, there exist a m_finished_array to store vehs waiting to exit */
	for (auto _link_it = m_link_factory -> m_link_map.begin(); _link_it != m_link_factory -> m_link_map.end(); _link_it ++){
		_link = _link_it -> second;
		/* _node_Id is the end node of this link 
		
		make sure when veh enters m_finished_array and they stopped on this link, they should pop out the inter-dest

		*/
		_node_ID = _link -> m_to_node -> m_node_ID;
		for (auto _veh_it = _link -> m_finished_array.begin(); _veh_it!=_link -> m_finished_array.end(); _veh_it++){
			_veh = *_veh_it;
			_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass_Curb *>(_veh);

			if (_veh_multiclass -> m_type == MNM_TYPE_ADAPTIVE_CURB){
				if (_link != _veh -> get_current_link()){
					printf("veh of MNM_TYPE_ADAPTIVE_CURB has wrong current link!\n");
					exit(-1);
				}
				
				/* indicator of whether next link is needed */
				TInt _flg = 0;

				/* condition 1 : no interdestinations left and use the real destination as _veh_dest, 
				current link is final link */

				if (_veh_multiclass -> m_destination_list.size() == 0){
					/* get the final dest */
					_veh_dest = _veh -> get_destination();

					/* final dest is current link end node */
					if (_veh_dest -> m_dest_node -> m_node_ID == _node_ID){
						/* no routing any more */
						_flg = 1;
						_veh -> set_next_link(nullptr);
					}
					else{
						/* condition 2 : no interdestinations left and use the real destination as _veh_dest, 
						current link is not final link, we still need to assign next link */
						_flg = 0;
						_veh_dest = _veh -> get_destination();
					}
				}
				else { 
					/* condition 3 : there exist intersections and use the first one to find next link */
					_flg = 0;
					_interdest_ID = _veh_multiclass -> m_destination_list.front();
					_veh_dest = m_curb_factory -> m_interdest_map.find(_interdest_ID) -> second;
				}
				
				/* if we need to find next link */

				if (_flg == 0){
					
					if (_veh_multiclass -> m_class == TInt(0)){
						_next_link_ID = m_table -> find(_veh_dest) -> second -> find(_node_ID) -> second;
					}

					if (_veh_multiclass -> m_class == TInt(1)){
						_next_link_ID = m_table_truck -> find(_veh_dest) -> second -> find(_node_ID) -> second;
					}

					if (_veh_multiclass -> m_class == TInt(2)){
						_next_link_ID = m_table_rh -> find(_veh_dest) -> second -> find(_node_ID) -> second;
					}
					
					/* next link error */ 
					if (_next_link_ID == -1){
						printf("Something wrong in routing, wrong next link \n");
						auto _node_I = m_graph -> GetNI(_node_ID);
						if (_node_I.GetOutDeg() > 0){
							/* assign next link randomly */
							_next_link_ID = _node_I.GetOutEId(MNM_Ults::mod(rand(), _node_I.GetOutDeg()));
						}
						else{
							printf("Can't do anything!\n");
						}
					}
					else{
						/* has next link */
						_next_link = m_link_factory -> get_link(_next_link_ID);
						_veh -> set_next_link(_next_link);

						/* add curb choice */
						/* TODO changed to checking if _next_link is in the set of all curbs of the interdest 06-13 */
						/* For now the code can only take ONE inter destination between each OD pair, meaning that adaptive vehicles can only have one item in m_destination_list */
						if (_veh_multiclass -> m_destination_list.size() != 0){
							assert(_veh_multiclass -> m_destination_list.size() == 1);
							// if _next_link is in th set of curbs of current interdest
							if (std::find(m_curb_factory -> m_interdest_curb_map.find(_veh_multiclass -> m_destination_list.front()) -> second.begin(), 
										  m_curb_factory -> m_interdest_curb_map.find(_veh_multiclass -> m_destination_list.front()) -> second.end(), 
										  _next_link_ID) != m_curb_factory -> m_interdest_curb_map.find(_veh_multiclass -> m_destination_list.front()) -> second.end()){
								if (_veh_multiclass -> m_destination_list.front() == _next_link -> m_to_node -> m_node_ID){
									_veh_multiclass -> m_curb_destination_list.push_front(_next_link_ID);
								}
							}
						}
					}					
				}
			}
			else if (_veh_multiclass -> m_type == MNM_TYPE_ADAPTIVE){
				if (_link != _veh -> get_current_link()){
					printf("veh of MNM_TYPE_ADAPTIVE_CURB has wrong current link!\n");
					exit(-1);
				}
				_veh_dest = _veh -> get_destination();
				/* final dest is current link end node */
				if (_veh_dest -> m_dest_node -> m_node_ID == _node_ID){
					/* no routing any more */
					_veh -> set_next_link(nullptr);
				}
				else{
					if (_veh_multiclass -> m_class == TInt(0)){
						_next_link_ID = m_table -> find(_veh_dest) -> second -> find(_node_ID) -> second;
					}

					if (_veh_multiclass -> m_class == TInt(1)){
						_next_link_ID = m_table_truck -> find(_veh_dest) -> second -> find(_node_ID) -> second;
					}

					/* next link error */ 
					if (_next_link_ID == -1){
						printf("Something wrong in routing, wrong next link \n");
						auto _node_I = m_graph -> GetNI(_node_ID);
						if (_node_I.GetOutDeg() > 0){
							/* assign next link randomly */
							_next_link_ID = _node_I.GetOutEId(MNM_Ults::mod(rand(), _node_I.GetOutDeg()));
						}
						else{
							printf("Can't do anything!\n");
						}
					}
					else{
						/* has next link */
						_next_link = m_link_factory -> get_link(_next_link_ID);
						_veh -> set_next_link(_next_link);
					}
				}
			}
		}
	}

	return 0;
}

MNM_Routing_Biclass_Hybrid_Curb::MNM_Routing_Biclass_Hybrid_Curb(const std::string& file_folder, 
																PNEGraph &graph, 
																MNM_Statistics* statistics,
																MNM_Statistics_Curb* statistics_curb,
																MNM_OD_Factory *od_factory, 
																MNM_Node_Factory *node_factory, 
																MNM_Link_Factory *link_factory, 
																MNM_Curb_Factory_Multiclass *curb_factory,
																TInt route_frq_fixed, 
																TInt buffer_length)

	: MNM_Routing_Biclass_Hybrid::MNM_Routing_Biclass_Hybrid(file_folder, graph, statistics, od_factory, node_factory, link_factory, route_frq_fixed, buffer_length)
{
	m_routing_adaptive = new MNM_Routing_Curb_Adaptive(file_folder, graph, statistics, statistics_curb, od_factory, node_factory, link_factory, curb_factory);

	m_routing_fixed_car = new MNM_Routing_Driving_Fixed(graph, od_factory, node_factory, link_factory, route_frq_fixed, buffer_length, TInt(0));

	m_routing_fixed_truck = new MNM_Routing_Truck_Fixed(graph, od_factory, node_factory, link_factory, route_frq_fixed, buffer_length, TInt(1));

	m_routing_fixed_ridehail = new MNM_Routing_Ridehail_Fixed(graph, od_factory, node_factory, link_factory, route_frq_fixed, buffer_length, TInt(2));

	m_curb_factory = curb_factory;

}

MNM_Routing_Biclass_Hybrid_Curb::~MNM_Routing_Biclass_Hybrid_Curb()
{
	delete m_routing_adaptive;

    delete m_routing_fixed_car;
  
    delete m_routing_fixed_truck;

	delete m_routing_fixed_ridehail;
}

int MNM_Routing_Biclass_Hybrid_Curb::init_routing_curb(Path_Table *path_table_car, Path_Table *path_table_truck, Path_Table *path_table_rh)
{
	m_routing_adaptive -> init_routing_curb();
  	// printf("Finished init all ADAPTIVE vehicles routing\n");
  	// m_routing_fixed_car -> init_routing(path_table);

	// no more init_routing functions !
	m_routing_fixed_car -> m_path_table = path_table_car;
  	// printf("Finished init STATIC cars routing\n");
  	// m_routing_fixed_truck -> init_routing(path_table);

	m_routing_fixed_truck -> m_path_table = path_table_truck;

	m_routing_fixed_ridehail -> m_path_table = path_table_rh;

	return 0;
}

int MNM_Routing_Biclass_Hybrid_Curb::init_routing(Path_Table *path_table)
{
	m_routing_adaptive -> init_routing(path_table);
  	// printf("Finished init all ADAPTIVE vehicles routing\n");
  	m_routing_fixed_car -> init_routing(path_table);
  	// printf("Finished init STATIC cars routing\n");
  	m_routing_fixed_truck -> init_routing(path_table);
	return 0;
}

int MNM_Routing_Biclass_Hybrid_Curb::update_routing(TInt timestamp)
{
	m_routing_adaptive -> update_routing(timestamp);
  	
	m_routing_fixed_car -> update_routing(timestamp);

    m_routing_fixed_truck -> update_routing(timestamp);

	m_routing_fixed_ridehail -> update_routing(timestamp);
	// register each vehicle with a route based on the portion of path flow
	return 0;
}

int MNM_Routing_Biclass_Hybrid_Curb::update_routing_curb(TInt timestamp)
{
	m_routing_adaptive -> update_routing_curb_adaptive(timestamp);
  	
	m_routing_fixed_car -> update_routing_curb_driving(timestamp, m_curb_factory);

    m_routing_fixed_truck -> update_routing_curb_truck(timestamp, m_curb_factory);

	m_routing_fixed_ridehail -> update_routing_curb_rh(timestamp, m_curb_factory);
	return 0;
}


MNM_Dta_Multiclass_Curb::MNM_Dta_Multiclass_Curb(const std::string& file_folder)
	: MNM_Dta::MNM_Dta(file_folder)
{
	// Re-run the multiclass version of initialize();
	initialize();
}

int MNM_Dta_Multiclass_Curb::pre_loading()
{
	MNM_Dnode *_node;
	// printf("MNM: Prepare loading!\n");
	m_statistics -> init_record();
	for (auto _node_it = m_node_factory -> m_node_map.begin(); _node_it != m_node_factory -> m_node_map.end(); _node_it++){
		_node = _node_it -> second;
		_node -> prepare_loading();
	}
	
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


MNM_Dta_Multiclass_Curb::~MNM_Dta_Multiclass_Curb()
{
	;
}

int MNM_Dta_Multiclass_Curb::initialize()
{
	if (m_veh_factory != nullptr) delete m_veh_factory;
  	if (m_node_factory != nullptr) delete m_node_factory;
  	if (m_link_factory != nullptr) delete m_link_factory;
  	if (m_od_factory != nullptr) delete m_od_factory;
  	if (m_config != nullptr) delete m_config;

	m_veh_factory = new MNM_Veh_Factory_Multiclass_Curb();
	// printf("1\n");
	m_node_factory = new MNM_Node_Factory_Multiclass_Curb();
	// printf("2\n");
	m_link_factory = new MNM_Link_Factory_Multiclass_Curb();
	// printf("3\n");
	m_od_factory = new MNM_OD_Factory_Multiclass_Curb();
	// printf("4\n");

	// jiachao added in Apr.
	// m_control_factory = new MNM_Control_Factory_Multiclass();
	// jiachao added in Sep.
	m_curb_factory = new MNM_Curb_Factory_Multiclass();

	m_config = new MNM_ConfReader(m_file_folder + "/config.conf", "DTA");
	m_unit_time = m_config -> get_int("unit_time");
	m_flow_scalar = m_config -> get_int("flow_scalar");
	// printf("5\n");
	m_emission = new MNM_Cumulative_Emission_Multiclass_Curb(TFlt(m_unit_time), 0);
    
	// the releasing strategy is assigning vehicles per 1 minute, so disaggregate 15-min demand into 1-min demand
	// change assign_freq to 12 (1 minute = 12 x 5 second / 60) and total_assign_interval to max_interval*_num_of_minute
	m_assign_freq = 60 / int(m_unit_time);  // # of unit intervals in 1 min = # of assign freq
	TInt _num_of_minute =  int(m_config -> get_int("assign_frq")) / m_assign_freq;  // 15 min, # of minutes in original assign interval
	m_total_assign_inter = m_config ->  get_int("max_interval") * _num_of_minute;  // how many 1-min intervals
	m_start_assign_interval = m_config -> get_int("start_assign_interval");

	return 0;
}

// function to build curb, node, link, od, graph, demand ... features in MNM_Dta_Multiclass_Curb
int MNM_Dta_Multiclass_Curb::build_from_files_separate()
{
	// jiachao added
	// jiachao tested new class MNM_IO_Multiclass_Curb 11/07/2024
	MNM_IO_Multiclass_Curb::build_curb_factory_multiclass(m_file_folder, m_config, m_curb_factory);

	MNM_IO_Multiclass_Curb::build_node_factory_multiclass(m_file_folder, m_config, m_node_factory);

	MNM_IO_Multiclass_Curb::build_link_factory_multiclass(m_file_folder, m_config, m_link_factory, m_curb_factory);

	MNM_IO_Multiclass_Curb::build_od_factory_multiclass_curb(m_file_folder, m_config, m_od_factory, m_node_factory);

	m_graph = MNM_IO_Multiclass_Curb::build_graph(m_file_folder, m_config);

	MNM_IO_Multiclass_Curb::build_demand_multiclass_curb(m_file_folder, m_config, m_od_factory);

	MNM_IO_Multiclass_Curb::load_inter_dest_curb_separate(m_file_folder, m_config, m_curb_factory);

	// build_workzone();
	m_workzone = nullptr;
	set_statistics();
	set_statistics_curb();
	// // set routing curb --- build m_routing which is MNM_Routing_Biclass_Hybrid_Curb
	set_routing_curb_separate();
	return 0;
}

int MNM_Dta_Multiclass_Curb::set_statistics_curb()
{
	MNM_ConfReader *_record_config = new MNM_ConfReader(m_file_folder + "/config.conf", "STAT");
	if (_record_config -> get_string("rec_mode") == "LRn"){
		m_statistics_curb = new MNM_Statistics_Curb(m_file_folder, m_config, _record_config,
									m_od_factory, m_node_factory, m_link_factory, m_curb_factory);
	}
	// printf("set_statistics_curb finished\n");
	m_statistics_curb -> init_record_curb();
	return 0;
}

Path_Table *MNM_Dta_Multiclass_Curb::load_path_table_curb(const std::string& file_name, const PNEGraph& graph,
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


int MNM_Dta_Multiclass_Curb::set_routing_curb_separate()
{
	if ((m_config -> get_string("routing_type") == "Biclass_Hybrid_curb") || (m_config -> get_string("routing_type") == "Biclass_Hybrid")){

		// fixed routing, using the path table

		// typedef std::unordered_map<TInt, std::unordered_map<TInt, MNM_Pathset*>*> Path_Table;

		MNM_ConfReader* _tmp_conf = new MNM_ConfReader(m_file_folder + "/config.conf", "FIXED");
		Path_Table *_path_table_truck;
		Path_Table *_path_table_car;
		Path_Table *_path_table_rh;

		// std::unordered_map<TInt, std::vector<TInt>> _path_inter_dest_table;

		if (_tmp_conf -> get_string("choice_portion") == "Buffer"){

			// printf("loading driving path table\n");
			_path_table_car = load_path_table_curb(m_file_folder + "/" + "path_table", 
							m_graph, _tmp_conf -> get_int("num_path"), true, TInt(0));

			// printf("loading truck path table\n");
			_path_table_truck = load_path_table_curb(m_file_folder + "/" + "path_table_curb", 
							m_graph, _tmp_conf -> get_int("num_path_curb"), true, TInt(1));

			// printf("loading ride-hailing path table\n");
			_path_table_rh = load_path_table_curb(m_file_folder + "/" + "path_table_curb_rh", 
							m_graph, _tmp_conf -> get_int("num_path_curb_rh"), true, TInt(2));

		}
		else{
			_path_table_car = load_path_table_curb(m_file_folder + "/" + "path_table", 
							m_graph, _tmp_conf -> get_int("num_path"), false, TInt(0));

			_path_table_truck = load_path_table_curb(m_file_folder + "/" + "path_table_curb", 
							m_graph, _tmp_conf -> get_int("num_path_curb"), false, TInt(1));

			_path_table_rh = load_path_table_curb(m_file_folder + "/" + "path_table_curb_rh", 
							m_graph, _tmp_conf -> get_int("num_path_curb_rh"), false, TInt(2));
		}

		TInt _buffer_len = _tmp_conf -> get_int("buffer_length");
		
		// for bi-class problem this is for curb usage ride-hailing and truck
		if (_buffer_len < 3 * m_config -> get_int("max_interval")) {
			_buffer_len = 3 * m_config -> get_int("max_interval");
		}

		TInt _route_freq_fixed = _tmp_conf -> get_int("route_frq");

		m_routing = new MNM_Routing_Biclass_Hybrid_Curb(m_file_folder, m_graph, m_statistics, m_statistics_curb, m_od_factory, 
												m_node_factory, m_link_factory, m_curb_factory,
												_route_freq_fixed, _buffer_len);

		m_routing -> init_routing_curb(_path_table_car, _path_table_truck, _path_table_rh);

		delete _tmp_conf;
	}
	else{
		printf("do not have other option for curb project\n");
	}
	return 0;	
}

int MNM_Dta_Multiclass_Curb::load_once_curb(bool verbose, TInt load_int, TInt assign_int)
{
	MNM_Origin *_origin;
	MNM_Origin_Multiclass_Curb *_origin_multiclass;
	MNM_Dnode *_node;
	MNM_Dlink *_link;
	MNM_Destination *_dest;
	MNM_Routing_Biclass_Hybrid_Curb *_routing_curb;
	
	if (verbose) printf("-------------------------------    Interval %d   ------------------------------ \n", (int)load_int);

	// step 1: Origin release vehicle
	if (verbose) printf("Releasing!\n");
	
	if (load_int % m_assign_freq == 0 || load_int==0){
		for (auto _origin_it = m_od_factory -> m_origin_map.begin(); _origin_it != m_od_factory -> m_origin_map.end(); _origin_it++){

			_origin = _origin_it -> second;

			_origin_multiclass = dynamic_cast<MNM_Origin_Multiclass_Curb *> (_origin);

			if (assign_int >= m_total_assign_inter) {
				if (m_config -> get_string("routing_type") == "Biclass_Hybrid"){
					_origin_multiclass -> release_one_interval_curb(load_int, m_veh_factory, -1, TFlt(-1), TFlt(-1), TFlt(-1));
				}
				if (m_config -> get_string("routing_type") == "Biclass_Hybrid_curb"){
					_origin_multiclass -> release_one_interval_curb_hybrid(load_int, m_veh_factory, m_curb_factory, -1, TFlt(-1), TFlt(-1), TFlt(-1), TFlt(-1), TFlt(-1));
				}
			}
			else{
				if (m_config -> get_string("routing_type") == "Biclass_Hybrid"){
					TFlt _ad_ratio_car = m_config -> get_float("adaptive_ratio_car");
					if (_ad_ratio_car > 1) _ad_ratio_car = 1;
					if (_ad_ratio_car < 0) _ad_ratio_car = 0;

					TFlt _ad_ratio_truck = m_config -> get_float("adaptive_ratio_truck");
					if (_ad_ratio_truck > 1) _ad_ratio_truck = 1;
					if (_ad_ratio_truck < 0) _ad_ratio_truck = 0;

					TFlt _ad_ratio_rh = m_config -> get_float("adaptive_ratio_rh");
					if (_ad_ratio_rh > 1) _ad_ratio_rh = 1;
					if (_ad_ratio_rh < 0) _ad_ratio_rh = 0;
				
					_origin_multiclass -> release_one_interval_curb(load_int, m_veh_factory, assign_int, _ad_ratio_car, _ad_ratio_truck, _ad_ratio_rh);
				}

				if (m_config -> get_string("routing_type") == "Biclass_Hybrid_curb"){
					TFlt _ad_ratio_car = m_config -> get_float("adaptive_ratio_car");
					if (_ad_ratio_car > 1) _ad_ratio_car = 1;
					if (_ad_ratio_car < 0) _ad_ratio_car = 0;

					TFlt _ad_ratio_truck = m_config -> get_float("adaptive_ratio_truck");
					if (_ad_ratio_truck > 1) _ad_ratio_truck = 1;
					if (_ad_ratio_truck < 0) _ad_ratio_truck = 0;

					TFlt _ad_ratio_rh = m_config -> get_float("adaptive_ratio_rh");
					if (_ad_ratio_rh > 1) _ad_ratio_rh = 1;
					if (_ad_ratio_rh < 0) _ad_ratio_rh = 0;

					TFlt _ad_ratio_car_curb = m_config -> get_float("adaptive_ratio_car_curb");
					if (_ad_ratio_car_curb > 1) _ad_ratio_car_curb = 1;
					if (_ad_ratio_car_curb < 0) _ad_ratio_car_curb = 0;

					TFlt _ad_ratio_truck_curb = m_config -> get_float("adaptive_ratio_truck_curb");
					if (_ad_ratio_truck_curb > 1) _ad_ratio_truck_curb = 1;
					if (_ad_ratio_truck_curb < 0) _ad_ratio_truck_curb = 0;

					_origin_multiclass -> release_one_interval_curb_hybrid(load_int, m_veh_factory, m_curb_factory, assign_int, 
																			_ad_ratio_car, _ad_ratio_car_curb, _ad_ratio_truck, _ad_ratio_truck_curb, _ad_ratio_rh);
				
				}
			}
		}
	}

	if (verbose) printf("Routing!\n");
	// step 2: route the vehicle
	_routing_curb = dynamic_cast<MNM_Routing_Biclass_Hybrid_Curb *> (m_routing);

	_routing_curb -> update_routing_curb(load_int);
  
	if (verbose) printf("Moving through node!\n");
	// step 3: move vehicles through node
	// TInt count = 1;
	for (auto _node_it = m_node_factory -> m_node_map.begin(); _node_it != m_node_factory -> m_node_map.end(); _node_it++)
	{
		_node = _node_it -> second;
		_node -> evolve_curb(load_int);
	}

	record_queue_vehicles();

	if (verbose) printf("Moving through link!\n");
	// step 4: move vehicles through link
	// for each link loop
	for (auto _link_it = m_link_factory -> m_link_map.begin(); _link_it != m_link_factory -> m_link_map.end(); _link_it++){
		_link = _link_it -> second;
		TFlt ratio_lane_closure = 1.0; // 1.0 by default meaning there is 100 % lane in use
		_link -> clear_incoming_array(load_int, ratio_lane_closure);
		// printf("cleared link %d\n", (int)_link -> m_link_ID());
		_link -> evolve_curb(load_int);
		// printf("Link %d\n", (int)_link -> m_link_ID());
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
	m_statistics_curb -> update_record_curb(load_int);

	record_enroute_vehicles();

	if (verbose){
		MNM::print_vehicle_statistics(m_veh_factory);
	}
	return 0;
}

int MNM_Dta_Multiclass_Curb::loading_curb(bool verbose)
{
	if (verbose){
		printf("\n\n\n====================================== Start loading w/ curbs ! =======================================\n");
	}

	TInt _current_inter = 0;
	TInt _assign_inter = m_start_assign_interval;

	while (!finished_loading(_current_inter) || _assign_inter < m_total_assign_inter){

		if (verbose) {
			printf("\nCurrent loading interval: %d, Current assignment interval: %d\n", _current_inter(), _assign_inter());
		}
	
		load_once_curb(verbose, _current_inter, _assign_inter);

		if (_current_inter % m_assign_freq == 0 || _current_inter == 0){
			_assign_inter += 1;
		}
		
		_current_inter += 1;
	}

	m_current_loading_interval = _current_inter;

	return 0;

}

MNM_Destination_Multiclass_Curb::MNM_Destination_Multiclass_Curb(TInt ID)
	: MNM_Destination::MNM_Destination(ID)
{
	;
}


MNM_Destination_Multiclass_Curb::~MNM_Destination_Multiclass_Curb()
{
	;
}

MNM_Origin_Multiclass_Curb::MNM_Origin_Multiclass_Curb(TInt ID, 
											 TInt max_interval,
											 TFlt flow_scalar,
											 TInt frequency)
	: MNM_Origin::MNM_Origin(ID, max_interval, flow_scalar, frequency)
{
	m_demand_car = std::unordered_map<MNM_Destination_Multiclass_Curb*, TFlt*>(); // destination node, time-varying demand list
	m_demand_truck = std::unordered_map<MNM_Destination_Multiclass_Curb*, TFlt*>(); // destination node, time-varying demand list
	// jiachao added in Sep
	m_demand_tnc = std::unordered_map<MNM_Destination_Multiclass_Curb*, TFlt*>(); // destination node, time-varying demand list
}

MNM_Origin_Multiclass_Curb::~MNM_Origin_Multiclass_Curb()
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

int MNM_Origin_Multiclass_Curb::release_one_interval_curb(TInt current_interval,
													MNM_Veh_Factory* veh_factory, 
													TInt assign_interval, 
													TFlt adaptive_ratio_car,
													TFlt adaptive_ratio_truck,
													TFlt adaptive_ratio_rh)
{

	if (assign_interval < 0) return 0;
	TInt _veh_to_release;
	MNM_Veh_Multiclass_Curb *_veh;
	MNM_Veh_Factory_Multiclass_Curb *_vfactory = dynamic_cast<MNM_Veh_Factory_Multiclass_Curb *>(veh_factory);
	
	// release all driving cars
	// loop for all dest
	// m_demand_car is {dest, m_demand_car}
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
			// how to set destination

			// _veh -> m_destination_list.push_back(_demand_it->first->m_Dest_ID);
			_veh -> set_destination(_demand_it -> first);
			_veh -> set_origin(this);
			// 0719 Jiachao set intermediate destinations
			// if (_veh -> m_type == MNM_TYPE_ADAPTIVE){

			// }
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

			// _veh -> m_destination_list.push_back(_demand_it->first->m_Dest_ID);
			_veh -> set_destination(_demand_it -> first);
			_veh -> set_origin(this);
			_veh -> m_assign_interval = assign_interval;
			m_origin_node -> m_in_veh_queue.push_back(_veh);
		}
	}

	// release all ride-hailing cars
	for (auto _demand_it = m_demand_tnc.begin(); _demand_it != m_demand_tnc.end(); _demand_it++) {
		_veh_to_release = TInt(MNM_Ults::round((_demand_it -> second)[assign_interval] * m_flow_scalar));
		for (int i = 0; i < _veh_to_release; ++i) {
			if (adaptive_ratio_rh == TFlt(0)){
				_veh = _vfactory -> make_veh_multiclass(current_interval, MNM_TYPE_STATIC, TInt(2));
			}
			else if (adaptive_ratio_rh == TFlt(1)){
				_veh = _vfactory -> make_veh_multiclass(current_interval, MNM_TYPE_ADAPTIVE, TInt(2));
			}
			else{
				TFlt _r = MNM_Ults::rand_flt();
				if (_r <= adaptive_ratio_rh){
					_veh = _vfactory -> make_veh_multiclass(current_interval, MNM_TYPE_ADAPTIVE, TInt(2));
				}
				else{
					_veh = _vfactory -> make_veh_multiclass(current_interval, MNM_TYPE_STATIC, TInt(2));
				}
			}
			// how to set destination ???
			_veh -> set_destination(_demand_it -> first);
			_veh -> set_origin(this);
			_veh -> m_assign_interval = assign_interval;
			m_origin_node -> m_in_veh_queue.push_back(_veh);
		}
	}

	random_shuffle(m_origin_node -> m_in_veh_queue.begin(), m_origin_node -> m_in_veh_queue.end());
	return 0;
}

int MNM_Origin_Multiclass_Curb::add_dest_demand_multiclass_curb(MNM_Destination_Multiclass_Curb *dest,
									   					   TFlt* demand_car,
														   TFlt* demand_tnc,
														   TFlt* demand_truck)
{
	
	TFlt* _demand_car = (TFlt*) malloc(sizeof(TFlt) * m_max_assign_interval * 15);
  	for (int i = 0; i < m_max_assign_interval * 15; ++i) {
  		_demand_car[i] =  TFlt(demand_car[i]);
  	}
  	m_demand_car.insert({dest, _demand_car});

	TFlt* _demand_tnc = (TFlt*) malloc(sizeof(TFlt) * m_max_assign_interval * 15);
  	for (int i = 0; i < m_max_assign_interval * 15; ++i) {
  		_demand_tnc[i] =  TFlt(demand_tnc[i]);
  	}
  	m_demand_tnc.insert({dest, _demand_tnc});

  	TFlt* _demand_truck = (TFlt*) malloc(sizeof(TFlt) * m_max_assign_interval * 15);
  	for (int i = 0; i < m_max_assign_interval * 15; ++i) {
  		_demand_truck[i] =  TFlt(demand_truck[i]);
  	}
  	m_demand_truck.insert({dest, _demand_truck});

	return 0;
}

int MNM_Origin_Multiclass_Curb::release_one_interval_curb_hybrid(TInt current_interval,
										MNM_Veh_Factory* veh_factory,
										MNM_Curb_Factory_Multiclass* curb_factory,
										TInt assign_interval,
										TFlt adaptive_ratio_car,
										TFlt adaptive_ratio_car_curb,// fraction of adaptive cars use hybrid routing for curb use
										TFlt adaptive_ratio_truck,
										TFlt adaptive_ratio_truck_curb, // fraction of adaptive trucks use hybrid routing for curb use
										TFlt adaptive_ratio_rh)
{
	if (assign_interval < 0) return 0;
	TInt _veh_to_release;
	MNM_Veh_Multiclass_Curb *_veh;
	MNM_Veh_Factory_Multiclass_Curb *_vfactory = dynamic_cast<MNM_Veh_Factory_Multiclass_Curb *>(veh_factory);
	
	// TInt _o_node_ID = m_origin_node -> m_node_ID;
	TInt _o_node_ID = m_Origin_ID;
	TInt _d_node_ID;
	std::vector<TInt> _inter_dest_list;
	bool _flg = true;
	
	// release all driving cars loop for all dest
	// m_demand_car is {dest, m_demand_car}
	for (auto _demand_it = m_demand_car.begin(); _demand_it != m_demand_car.end(); _demand_it++) {
		_veh_to_release = TInt(MNM_Ults::round((_demand_it -> second)[assign_interval] * m_flow_scalar));
		_d_node_ID = _demand_it -> first -> m_Dest_ID;

		for (int i = 0; i < _veh_to_release; ++i) {
			if (adaptive_ratio_car == TFlt(0)){
				_veh = _vfactory -> make_veh_multiclass(current_interval, MNM_TYPE_STATIC, TInt(0));
			}
			else if ((adaptive_ratio_car == TFlt(1)) && (adaptive_ratio_car_curb == TFlt(0))){
				_veh = _vfactory -> make_veh_multiclass(current_interval, MNM_TYPE_ADAPTIVE, TInt(0));
			}
			else{
				TFlt _r = MNM_Ults::rand_flt();
				if (_r <= adaptive_ratio_car){
					if (_r <= adaptive_ratio_car * adaptive_ratio_car_curb){
						if (_flg){
							_veh = _vfactory -> make_veh_multiclass_curb(current_interval, MNM_TYPE_ADAPTIVE_CURB, TInt(0), curb_factory, _o_node_ID, _d_node_ID);
						}
						else{
							_veh = _vfactory -> make_veh_multiclass(current_interval, MNM_TYPE_ADAPTIVE, TInt(0));
						}
					}
					else{
						_veh = _vfactory -> make_veh_multiclass(current_interval, MNM_TYPE_ADAPTIVE, TInt(0));
					}
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
		_d_node_ID = _demand_it -> first -> m_Dest_ID;

		for (int i = 0; i < _veh_to_release; ++i) {
			if (adaptive_ratio_truck == TFlt(0)){
				_veh = _vfactory -> make_veh_multiclass(current_interval, MNM_TYPE_STATIC, TInt(1));
			}
			else if ((adaptive_ratio_truck == TFlt(1)) && (adaptive_ratio_truck_curb == TFlt(0))){
				_veh = _vfactory -> make_veh_multiclass(current_interval, MNM_TYPE_ADAPTIVE, TInt(1));
			}
			else{
				TFlt _r = MNM_Ults::rand_flt();
				if (_r <= adaptive_ratio_truck){
					if (_r <= adaptive_ratio_truck * adaptive_ratio_truck_curb){
						if (_flg) {
							_veh = _vfactory -> make_veh_multiclass_curb(current_interval, MNM_TYPE_ADAPTIVE_CURB, TInt(1), curb_factory, _o_node_ID, _d_node_ID);
						}
						else{
							_veh = _vfactory -> make_veh_multiclass(current_interval, MNM_TYPE_ADAPTIVE, TInt(1));
						}						
					}
					else{
						_veh = _vfactory -> make_veh_multiclass(current_interval, MNM_TYPE_ADAPTIVE, TInt(1));
					}
				}
				else{
					_veh = _vfactory -> make_veh_multiclass(current_interval, MNM_TYPE_STATIC, TInt(1));
				}
			}

			// _veh -> m_destination_list.push_back(_demand_it->first->m_Dest_ID);
			_veh -> set_destination(_demand_it -> first);
			_veh -> set_origin(this);

			_veh -> m_assign_interval = assign_interval;
			m_origin_node -> m_in_veh_queue.push_back(_veh);
		}
	}

	// release all ride-hailing cars
	for (auto _demand_it = m_demand_tnc.begin(); _demand_it != m_demand_tnc.end(); _demand_it++) {
		_veh_to_release = TInt(MNM_Ults::round((_demand_it -> second)[assign_interval] * m_flow_scalar));
		_d_node_ID = _demand_it -> first -> m_Dest_ID;

		for (int i = 0; i < _veh_to_release; ++i) {
			if (adaptive_ratio_rh == TFlt(0)){
				_veh = _vfactory -> make_veh_multiclass(current_interval, MNM_TYPE_STATIC, TInt(2));
			}
			else if (adaptive_ratio_rh == TFlt(1)){
				if (_flg) {
					_veh = _vfactory -> make_veh_multiclass_curb(current_interval, MNM_TYPE_ADAPTIVE_CURB, TInt(2), curb_factory, _o_node_ID, _d_node_ID);
				}
				else{
					_veh = _vfactory -> make_veh_multiclass(current_interval, MNM_TYPE_STATIC, TInt(2));
				}
			}
			else{
				TFlt _r = MNM_Ults::rand_flt();
				if (_r <= adaptive_ratio_rh){
					if (_flg) {
						_veh = _vfactory -> make_veh_multiclass_curb(current_interval, MNM_TYPE_ADAPTIVE_CURB, TInt(2), curb_factory, _o_node_ID, _d_node_ID);	
					}
					else{
						_veh = _vfactory -> make_veh_multiclass(current_interval, MNM_TYPE_STATIC, TInt(2));
					}
				}
				else{
					_veh = _vfactory -> make_veh_multiclass(current_interval, MNM_TYPE_STATIC, TInt(2));
				}
			}
			// how to set destination ???
			_veh -> set_destination(_demand_it -> first);
			_veh -> set_origin(this);
			_veh -> m_assign_interval = assign_interval;
			m_origin_node -> m_in_veh_queue.push_back(_veh);
		}
	}
	return 0;
}

/******************************************************************************************************************
*******************************************************************************************************************
												Multiclass Vehicle for curb
*******************************************************************************************************************
******************************************************************************************************************/
MNM_Veh_Multiclass_Curb::MNM_Veh_Multiclass_Curb(TInt ID, TInt vehicle_class, TInt start_time)
	: MNM_Veh::MNM_Veh(ID, start_time)
{
	m_class = vehicle_class;  // 0: driving, 1: truck, 2: RH

	m_visual_position_on_link = 0.5; // default: visualize veh as at the middle point of link

	TFlt _r = MNM_Ults::rand_flt();

	if (_r > 1) {_r = 1;}

	m_parking_location = _r;

	m_complete_stop_current_link = TInt(0);

	m_reserved_slz = -1;

}

MNM_Veh_Multiclass_Curb::~MNM_Veh_Multiclass_Curb()
{
	;
}

/******************************************************************************************************************
*******************************************************************************************************************
												New Vehicle Factory
*******************************************************************************************************************
******************************************************************************************************************/

MNM_Veh_Factory_Multiclass_Curb::MNM_Veh_Factory_Multiclass_Curb()
	: MNM_Veh_Factory::MNM_Veh_Factory()
{
	;
}

MNM_Veh_Factory_Multiclass_Curb::~MNM_Veh_Factory_Multiclass_Curb()
{
	;
}

MNM_Veh_Multiclass_Curb* MNM_Veh_Factory_Multiclass_Curb::make_veh_multiclass(TInt timestamp, 
														 			Vehicle_type veh_type,
														 			TInt vehicle_cls)
{
	// printf("A vehicle is produce at time %d, ID is %d\n", (int)timestamp, (int)m_num_veh + 1);
	MNM_Veh_Multiclass_Curb *_veh = new MNM_Veh_Multiclass_Curb(m_num_veh + 1, vehicle_cls, timestamp);
	_veh -> m_type = veh_type;
	m_veh_map.insert({m_num_veh + 1, _veh});
	m_num_veh += 1;

	return _veh;
}

MNM_Veh_Multiclass_Curb* MNM_Veh_Factory_Multiclass_Curb::make_veh_multiclass_curb(TInt timestamp, 
														 			Vehicle_type veh_type,
														 			TInt vehicle_cls,
																	MNM_Curb_Factory_Multiclass* curb_factory,
																	TInt _o_node_ID,
																	TInt _d_node_ID)
{
	MNM_Veh_Multiclass_Curb *_veh = new MNM_Veh_Multiclass_Curb(m_num_veh + 1, vehicle_cls, timestamp);
	_veh -> m_type = veh_type;
	m_veh_map.insert({m_num_veh + 1, _veh});
	m_num_veh += 1;

	std::vector<TInt> _inter_dest_list;
	_inter_dest_list = curb_factory -> m_od_inter_dest_map[_o_node_ID][_d_node_ID];
	TInt _int_dest_num = TInt(_inter_dest_list.size());
	if (_int_dest_num == 0){
		_veh -> m_type = MNM_TYPE_STATIC;
	}
	else{
		for (int j = 0; j < _int_dest_num; ++j){
			_veh -> m_destination_list.push_back(TInt(_inter_dest_list[j]));
			if (vehicle_cls == TInt(0)){
				_veh -> m_parking_duration_list.push_back(TInt(120)); // parking duration change 90
			}
			else if (vehicle_cls == TInt(1)){
				_veh -> m_parking_duration_list.push_back(TInt(180)); // 60
			}
			else{
				_veh -> m_parking_duration_list.push_back(TInt(60)); // 20
			}
		}
	}

	return _veh;
}

MNM_Veh_Multiclass_Curb* MNM_Veh_Factory_Multiclass_Curb::make_veh_multiclass_curb_reserved(TInt timestamp, 
												Vehicle_type veh_type, 
												TInt vehicle_cls, 
												MNM_Curb_Factory_Multiclass* curb_factory,
												TInt _o_node_ID,
												TInt _d_node_ID)
{
	MNM_Veh_Multiclass_Curb *_veh = new MNM_Veh_Multiclass_Curb(m_num_veh + 1, vehicle_cls, timestamp);
	_veh -> m_type = veh_type;

	m_veh_map.insert({m_num_veh + 1, _veh});
	m_num_veh += 1;
	
	// assign curb space to this vehicle
	std::vector<TInt> _inter_dest_list;
	_inter_dest_list = curb_factory -> m_od_inter_dest_map[_o_node_ID][_d_node_ID];
	TInt _int_dest_num = TInt(_inter_dest_list.size());
	if (_int_dest_num == 0)
	{
		_veh -> m_type = MNM_TYPE_STATIC; // make sure if no interdest, this veh will not have interdest and curb usage
	}
	else{
		for (int j = 0; j < _int_dest_num; ++j){
			_veh -> m_destination_list.push_back(TInt(_inter_dest_list[j]));
			switch (vehicle_cls){
				case 0:
					_veh -> m_parking_duration_list.push_back(TInt(120)); // parking duration change 90
					break;
				case 1:
					_veh -> m_parking_duration_list.push_back(TInt(120)); // 60
					break;
				case 2:
					_veh -> m_parking_duration_list.push_back(TInt(120)); // 20
					break;
				default:
					printf("Wrong vehicle class.\n");
					exit(-1);
			}
			// _reserved_curb = curb_factory -> m_interdest_curb_map
			// _veh -> m_reserved_slz = _reserved_curb;
		}
	}
	return _veh;
}

/******************************************************************************************************************
*******************************************************************************************************************
												New OD Factory
*******************************************************************************************************************
******************************************************************************************************************/

MNM_OD_Factory_Multiclass_Curb::MNM_OD_Factory_Multiclass_Curb()
	: MNM_OD_Factory::MNM_OD_Factory()
{
	;
}

MNM_OD_Factory_Multiclass_Curb::~MNM_OD_Factory_Multiclass_Curb()
{
	;
}

MNM_Destination_Multiclass_Curb *MNM_OD_Factory_Multiclass_Curb::make_destination(TInt ID)
{
	MNM_Destination_Multiclass_Curb *_dest;
	_dest = new MNM_Destination_Multiclass_Curb(ID);
	m_destination_map.insert({ID, _dest});
	return _dest;
}

MNM_Origin_Multiclass_Curb *MNM_OD_Factory_Multiclass_Curb::make_origin(TInt ID, 
												TInt max_interval, 
												TFlt flow_scalar, 
												TInt frequency)
{
	MNM_Origin_Multiclass_Curb *_origin;
	_origin = new MNM_Origin_Multiclass_Curb(ID, max_interval, flow_scalar, frequency);
	m_origin_map.insert({ID, _origin});
	return _origin;
}

/*********************************************************************************************************************
                          new node factory - for extracting thw whole curb models from multiclass.cpp file (03/18/2025)
						  notes:
						  create a new node factory named MNM_Node_Factory_Multiclass_Curb inherit from MNM_Node_Factory
						  define the make_node_multiclass_curb function to create a new node
						  the return new node is MNM_DMOND_Multiclass_Curb, MNM_DMOND_Multiclass_Curb or MNM_Inout_Multiclass_Curb
*********************************************************************************************************************/

MNM_Node_Factory_Multiclass_Curb::MNM_Node_Factory_Multiclass_Curb()
	: MNM_Node_Factory::MNM_Node_Factory()
{
	;
}

MNM_Node_Factory_Multiclass_Curb::~MNM_Node_Factory_Multiclass_Curb()
{
	;
}

MNM_Dnode *MNM_Node_Factory_Multiclass_Curb::make_node_multiclass_curb(TInt ID, 
												  			DNode_type_multiclass node_type, 
												  			TFlt flow_scalar,
												  			TFlt veh_convert_factor)
{
	MNM_Dnode *_node;
	switch (node_type){
    	case MNM_TYPE_FWJ_MULTICLASS:
			_node = new MNM_Dnode_Inout_Multiclass_Curb(ID, flow_scalar, veh_convert_factor);
			break;
    	case MNM_TYPE_ORIGIN_MULTICLASS:
			_node = new MNM_DMOND_Multiclass_Curb(ID, flow_scalar, veh_convert_factor);
			break;
    	case MNM_TYPE_DEST_MULTICLASS:
			_node = new MNM_DMDND_Multiclass_Curb(ID, flow_scalar, veh_convert_factor);
			break;
    	default:
			printf("Wrong node type.\n");
			exit(-1);
	}

	m_node_map.insert({ID, _node});
	return _node;
}


/******************************************************************************************************************
*******************************************************************************************************************
												New link Factory
*******************************************************************************************************************
******************************************************************************************************************/

MNM_Link_Factory_Multiclass_Curb::MNM_Link_Factory_Multiclass_Curb()
	: MNM_Link_Factory::MNM_Link_Factory()
{
	;
}

MNM_Link_Factory_Multiclass_Curb::~MNM_Link_Factory_Multiclass_Curb()
{
	;
}

MNM_Dlink *MNM_Link_Factory_Multiclass_Curb::make_link_multiclass_curb(TInt ID,
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
	switch (link_type){
		case MNM_TYPE_CTM_MULTICLASS:
			_link = new MNM_Dlink_Ctm_Multiclass_Curb(ID,
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
			break;
		case MNM_TYPE_LQ_MULTICLASS:
			_link = new MNM_Dlink_Lq_Multiclass_Curb(ID,
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
			break;
		case MNM_TYPE_PQ_MULTICLASS:
			_link = new MNM_Dlink_Pq_Multiclass_Curb(ID,
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

/******************************************************************************************************************
*******************************************************************************************************************
											New Emission functions
*******************************************************************************************************************
******************************************************************************************************************/

MNM_Cumulative_Emission_Multiclass_Curb::MNM_Cumulative_Emission_Multiclass_Curb(TFlt unit_time, TInt freq)
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

MNM_Cumulative_Emission_Multiclass_Curb::~MNM_Cumulative_Emission_Multiclass_Curb(){ ; }

TFlt MNM_Cumulative_Emission_Multiclass_Curb::calculate_fuel_rate_truck(TFlt v)
{
	TFlt _convert_factor = 1.0;
	if (v < 25)
	{
		_convert_factor = 1.53;
	}
	else if (v < 55)
	{
		_convert_factor = 1.50;
	}
	else
	{
		_convert_factor = 1.55;
	}
	TFlt _fuel_rate_car = calculate_fuel_rate (v);
	TFlt _fuel_rate_truck = _fuel_rate_car * _convert_factor;
	return _fuel_rate_truck;
}

TFlt MNM_Cumulative_Emission_Multiclass_Curb::calculate_CO2_rate_truck(TFlt v)
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

TFlt MNM_Cumulative_Emission_Multiclass_Curb::calculate_HC_rate_truck(TFlt v)
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

TFlt MNM_Cumulative_Emission_Multiclass_Curb::calculate_CO_rate_truck(TFlt v)
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

TFlt MNM_Cumulative_Emission_Multiclass_Curb::calculate_NOX_rate_truck(TFlt v)
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

int MNM_Cumulative_Emission_Multiclass_Curb::update(MNM_Veh_Factory* veh_factory)
{
	TFlt _v;
	TFlt _v_converted;
	for (MNM_Dlink *link : m_link_vector){
		MNM_Dlink_Multiclass_Curb *_mlink = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(link);
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
	MNM_Veh_Multiclass_Curb * _veh_multiclass;
	for (auto pair_it : veh_factory -> m_veh_map){
		_veh =  pair_it.second;
		_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass_Curb *>(_veh);
		if (m_link_set.find(_veh_multiclass -> m_current_link) != m_link_set.end()){
			if (_veh_multiclass -> m_class != 1){
				m_car_set.insert(_veh_multiclass);
			}
			if (_veh_multiclass -> m_class == 1){
				m_truck_set.insert(_veh_multiclass);
			}		
		}
	}

	return 0;
}

std::string MNM_Cumulative_Emission_Multiclass_Curb::output_save()
{
	std::string _str;
	
	_str = "class trips VHT(hours) VMT(miles) ave_tt(min) ave_dis(mile) fuel(gallons) CO2(kg) HC(kg) CO(kg) NOX(kg)\n";
	_str += std::string("car ") + std::to_string(int(m_car_set.size())) + " " + std::to_string(m_VHT_car()) + " " + std::to_string(m_VMT()) + " " + std::to_string(m_VHT_car()/TFlt(m_car_set.size()) * TFlt(60)) + " " + std::to_string(m_VMT()/TFlt(m_car_set.size())) + " " + std::to_string(m_fuel()) + " " + std::to_string(m_CO2()/1000) + " " + std::to_string(m_HC()/1000) + " " + std::to_string(m_CO()/1000) + " " + std::to_string(m_NOX()/1000) + "\n";
	_str += std::string("truck ") + std::to_string(int(m_truck_set.size())) + " " + std::to_string(m_VHT_truck()) + " " + std::to_string(m_VMT_truck()) + " " + std::to_string(m_VHT_truck()/TFlt(m_truck_set.size()) * TFlt(60)) + " " + std::to_string(m_VMT_truck/TFlt(m_truck_set.size())) + " " + std::to_string(m_fuel_truck()) + " " + std::to_string(m_CO2_truck()/1000) + " " + std::to_string(m_HC_truck()/1000) + " " + std::to_string(m_CO_truck()/1000) + " " + std::to_string(m_NOX_truck()/1000) + "\n";
	
	printf("The emission stats for cars are: ");
	printf("fuel: %lf gallons, CO2: %lf g, HC: %lf g, CO: %lf g, NOX: %lf g, VMT: %lf miles, VHT: %lf hours, %d trips\n", 
		   m_fuel(), m_CO2(), m_HC(), m_CO(), m_NOX(), m_VMT(), m_VHT_car(), int(m_car_set.size()));

	printf("The emission stats for trucks are: ");
	printf("fuel: %lf gallons, CO2: %lf g, HC: %lf g, CO: %lf g, NOX: %lf g, VMT: %lf miles, VHT: %lf hours, %d trips\n", 
		   m_fuel_truck(), m_CO2_truck(), m_HC_truck(), m_CO_truck(), m_NOX_truck(), m_VMT_truck(), m_VHT_truck(), int(m_truck_set.size()));
	
	return _str;
}


int MNM_IO_Multiclass_Curb::build_node_factory_multiclass(const std::string& file_folder,
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

	MNM_Node_Factory_Multiclass_Curb* _node_factory = dynamic_cast<MNM_Node_Factory_Multiclass_Curb *>(node_factory);

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
					_node_factory -> make_node_multiclass_curb(_node_ID, 
														MNM_TYPE_FWJ_MULTICLASS, 
														_flow_scalar,
														_veh_convert_factor);
					continue;
				}				
				if (_type =="DMOND"){
					_node_factory -> make_node_multiclass_curb(_node_ID, 
														MNM_TYPE_ORIGIN_MULTICLASS, 
														_flow_scalar,
														_veh_convert_factor);
					continue;
				}
				if (_type =="DMDND"){
					_node_factory -> make_node_multiclass_curb(_node_ID, 
														MNM_TYPE_DEST_MULTICLASS, 
														_flow_scalar,
														_veh_convert_factor);
					continue;
				}
				printf("Wrong node type, %s\n", _type.c_str());
				exit(-1);
			}
			else {
				printf("MNM_IO_Multiclass::build node factory Multiclass: Wrong length of line.\n");
				exit(-1);
			}
		}
		_node_file.close();
	}
	return 0;
}

int MNM_IO_Multiclass_Curb::build_link_factory_multiclass(const std::string& file_folder,
                                                     MNM_ConfReader *conf_reader,
                                                     MNM_Link_Factory *link_factory,
													 MNM_Curb_Factory_Multiclass *curb_factory,
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

	MNM_Link_Factory_Multiclass_Curb* _link_factory = dynamic_cast<MNM_Link_Factory_Multiclass_Curb *>(link_factory);

	if (_link_file.is_open())
	{
		// printf("Start build link factory.\n");
		std::getline(_link_file,_line); //skip the first line
		for (int i = 0; i < _num_of_link; ++i){
			std::getline(_link_file,_line);
			_words = split(_line, ' ');
			if ((_words.size() == 11) && (_curb_install == TInt(1))){
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

				// curb space
				auto _check_cap = curb_factory->m_curb_capacity_map.find(_link_ID);
		
				if (_check_cap != curb_factory->m_curb_capacity_map.end()){
					_curb_spaces = curb_factory -> m_curb_capacity_map[_link_ID];
				}
				else{
					_curb_spaces = TInt(0);
				}

				auto _check_dest = curb_factory->m_curb_destination_map.find(_link_ID);

				if (_check_dest != curb_factory -> m_curb_destination_map.end()){
					_curb_dest = curb_factory -> m_curb_destination_map[_link_ID];
				}
				else{
					_curb_dest = TInt(-1);
				}

				/* build */
				if (_type == "PQ"){
					_link_factory -> make_link_multiclass_curb(_link_ID,
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
														_flow_scalar,
														_curb_spaces,
														_curb_dest);
					continue;
				}
				if (_type == "LQ"){
					_link_factory -> make_link_multiclass_curb(_link_ID,
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
														_flow_scalar,
														_curb_spaces,
														_curb_dest);
					continue;
				}
				if (_type =="CTM"){
					_link_factory -> make_link_multiclass_curb(_link_ID,
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
														_flow_scalar,
														_curb_spaces,
														_curb_dest);
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

int MNM_IO_Multiclass_Curb::build_demand_multiclass_curb(const std::string& file_folder,
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
	MNM_Origin_Multiclass_Curb *_origin;
	MNM_Destination_Multiclass_Curb *_dest;
	std::string _line;
	std::vector<std::string> _words;

	if (_demand_file.is_open())
	{
		// printf("Start build demand profile.\n");
		TFlt *_demand_vector_driving = (TFlt*) malloc(sizeof(TFlt) * _max_interval * _num_of_minute);
		TFlt *_demand_vector_tnc = (TFlt*) malloc(sizeof(TFlt) * _max_interval * _num_of_minute);
		TFlt *_demand_vector_truck = (TFlt*) malloc(sizeof(TFlt) * _max_interval * _num_of_minute);

		memset(_demand_vector_driving, 0x0, sizeof(TFlt) * _max_interval * _num_of_minute);
		memset(_demand_vector_tnc, 0x0, sizeof(TFlt) * _max_interval * _num_of_minute);
		memset(_demand_vector_truck, 0x0, sizeof(TFlt) * _max_interval * _num_of_minute);
		
		TFlt _demand_driving;
		TFlt _demand_tnc;
		TFlt _demand_truck;

		std::getline(_demand_file,_line); //skip the first line
		for (int i = 0; i < _num_OD; ++i){
			std::getline(_demand_file,_line);
			_words = split(_line, ' ');
			if (TInt(_words.size()) == (_max_interval * 3 + 2)) {
				_O_ID = TInt(std::stoi(_words[0]));
				_D_ID = TInt(std::stoi(_words[1]));
				// the releasing strategy is assigning vehicles per 1 minute, so disaggregate 15-min demand into 1-min demand
				for (int j = 0; j < _max_interval; ++j) {
					// [Driving_demand, RH_demand, Truck_demand]
					_demand_driving = TFlt(std::stod(_words[j + 2])) / TFlt(_num_of_minute);  
					_demand_truck = TFlt(std::stod(_words[j + _max_interval + 2])) / TFlt(_num_of_minute);
					_demand_tnc = TFlt(std::stod(_words[j + _max_interval * 2 + 2])) / TFlt(_num_of_minute);
					for (int k = 0; k < _num_of_minute; ++k){
						_demand_vector_driving[j * _num_of_minute + k] = _demand_driving;
						_demand_vector_truck[j * _num_of_minute + k] = _demand_truck;
						_demand_vector_tnc[j * _num_of_minute + k] = _demand_tnc;
					}
				}
				_origin = dynamic_cast<MNM_Origin_Multiclass_Curb *>(od_factory -> get_origin(_O_ID));
				_dest = dynamic_cast<MNM_Destination_Multiclass_Curb *>(od_factory -> get_destination(_D_ID));

				_origin -> add_dest_demand_multiclass_curb(_dest, _demand_vector_driving, _demand_vector_tnc, _demand_vector_truck);
			}
			else{
				printf("Something wrong in build_demand!\n");
				free(_demand_vector_driving);
				free(_demand_vector_tnc);
				free(_demand_vector_truck);
				exit(-1);
			}
		}
		free(_demand_vector_driving);
		free(_demand_vector_tnc);
		free(_demand_vector_truck);
		_demand_file.close();
	}

	return 0;
}								

int MNM_IO_Multiclass_Curb::build_curb_factory_multiclass(const std::string& file_folder,
 									 					MNM_ConfReader *conf_reader,
 									 					MNM_Curb_Factory_Multiclass *curb_factory)
{
	/* read config */
	int _curb_install = conf_reader -> get_int("curb_install");
	int _num_of_curb = conf_reader -> get_int("num_of_curb");
	TFlt _flow_scalar = conf_reader -> get_float("flow_scalar");

	// each OD pair has an exogenous inter-destination sequence (for now, only one interdestination for each Od pair for simplicity)
	int _num_od_interdest = conf_reader -> get_int("od_interdest"); 

	/* read multi-class curb prices */
	int _curb_price_multiclass = conf_reader -> get_int("curb_price_multiclass");

	if (_curb_install == 1){

		/* find curb dest file */
		std::string _curb_file_name = file_folder + "/MNM_input_curb_dest";
		std::ifstream _curb_file;
		_curb_file.open(_curb_file_name, std::ios::in);

		std::string _line;
		std::vector<std::string> _words;
		TInt _curb_ID;
		TInt _dest_node_ID;
		TInt _curb_cap;
		std::vector<TInt> _new_vector;

		MNM_Destination *_dest;

		if (_curb_file.is_open())
		{
			std::getline(_curb_file,_line); //skip the first line
			for (int i = 0; i < _num_of_curb; ++i){
				std::getline(_curb_file,_line);
				// printf("%d\n", i);
				_words = split(_line, ' ');
				if (_words.size() == 3) {
					_curb_ID = TInt(std::stoi(_words[0]));
					_dest_node_ID = TInt(std::stoi(_words[1]));
					_curb_cap = TInt(std::stoi(_words[2]));

					curb_factory -> m_curb_destination_map.insert({_curb_ID, _dest_node_ID});
					curb_factory -> m_curb_capacity_map.insert({_curb_ID, _curb_cap * _flow_scalar}); // Jiachao fixed this 11/07

					// if _dest_node_ID is not in m_interdest_list, add it into the list
					if (std::find(curb_factory -> m_interdest_list.begin(), curb_factory -> m_interdest_list.end(), _dest_node_ID) == curb_factory -> m_interdest_list.end()){
						if (_dest_node_ID != TInt(-1)){
							curb_factory -> m_interdest_list.push_back(_dest_node_ID);
							_dest = new MNM_Destination(_dest_node_ID);
  							curb_factory -> m_interdest_map.insert(std::pair<TInt, MNM_Destination*>(_dest_node_ID, _dest));
						}
					}

					if (_dest_node_ID != TInt(-1)){
						if (curb_factory -> m_interdest_curb_map.find(_dest_node_ID) == curb_factory -> m_interdest_curb_map.end()){					
							curb_factory -> m_interdest_curb_map.insert({_dest_node_ID, std::vector<TInt>()});
						}
					}

					curb_factory -> m_interdest_curb_map.find(_dest_node_ID) -> second.push_back(_curb_ID);
				}
				else {
					printf("MNM_IO_Multiclass::build curb_factory_multiclass: Wrong length of line.\n");
					exit(-1);
				}
			}
			_curb_file.close();
		}

		// read in OD interdest map from file
		std::string _od_interdest_file_name = file_folder + "/od_inter_dest";
		std::ifstream _od_interdest_file;
		TInt _o_node;
		TInt _d_node;
		TInt _inter_node_ID;
		std::vector<TInt> _interdest_vec;

		_od_interdest_file.open(_od_interdest_file_name, std::ios::in);
		if (_od_interdest_file.is_open()){
			for (int i = 0; i < _num_od_interdest; ++i){
				std::getline(_od_interdest_file,_line);
				_words = split(_line, ' ');
				if (_words.size() > 2) {
					_o_node = TInt(std::stoi(_words[0]));
					_d_node = TInt(std::stoi(_words[1]));

					for (int j = 2; j < int(_words.size()); ++j){
						_inter_node_ID = TInt(std::stoi(_words[j]));
						_interdest_vec.push_back(_inter_node_ID);
						if (std::find(curb_factory -> m_interdest_list.begin(), curb_factory -> m_interdest_list.end(), _inter_node_ID) == curb_factory -> m_interdest_list.end()){
							curb_factory -> m_interdest_list.push_back(_inter_node_ID);
						}
					}

					if (curb_factory -> m_od_inter_dest_map.find(_o_node) == curb_factory -> m_od_inter_dest_map.end()){
						curb_factory -> m_od_inter_dest_map.insert({_o_node, std::unordered_map<TInt, std::vector<TInt>>()});
					}

					curb_factory -> m_od_inter_dest_map.find(_o_node) -> second.insert({_d_node, _interdest_vec});
					_interdest_vec.clear();
				}
				else {
					printf("MNM_IO_Multiclass::build curb_factory_multiclass:read_od_interdest wrong length of line.\n");
					exit(-1);
				}
			}
			_od_interdest_file.close();
		}

		/* (TODO) read curb price */
		std::string _curb_price_line;
		std::vector<std::string> _curb_price_words;
		
		if (_curb_price_multiclass == 0){
			std::string _curb_price_name = file_folder + "/curb_price";
			std::ifstream _curb_price_file;

			_curb_price_file.open(_curb_price_name, std::ios::in);

			if (_curb_price_file.is_open())
			{
				for (int i = 0; i < _num_of_curb; ++i){
					std::getline(_curb_price_file, _curb_price_line);
					_curb_price_words = split(_curb_price_line, ' ');

					if (_curb_price_words.size() >= 2){
						
						_curb_ID = TInt(std::stoi(_curb_price_words[0]));

						if (curb_factory -> m_curb_price_list.find(_curb_ID) == curb_factory -> m_curb_price_list.end()){
							curb_factory -> m_curb_price_list.insert({_curb_ID, std::vector<TFlt>()});
						}

						for (int j = 1; j < int(_curb_price_words.size()); ++j){
							curb_factory -> m_curb_price_list.find(_curb_ID) -> second.push_back(TFlt(std::stof(_curb_price_words[j])));
						}	
					}
					else{
						printf("MNM_IO_Multiclass::build curb_factory_multiclass:curb_price wrong length of line input (smaller than 2).\n");
						exit(-1);
					}
				}
			}
		}
		else{
			assert(_curb_price_multiclass == 1);
			std::string _curb_price_name_car = file_folder + "/curb_price_car";
			std::string _curb_price_name_truck = file_folder + "/curb_price_truck";
			std::string _curb_price_name_rh = file_folder + "/curb_price_rh";
			std::ifstream _curb_price_file_car, _curb_price_file_truck, _curb_price_file_rh;

			_curb_price_file_car.open(_curb_price_name_car, std::ios::in);

			if (_curb_price_file_car.is_open())
			{
				for (int i = 0; i < _num_of_curb; ++i){
					std::getline(_curb_price_file_car, _curb_price_line);
					_curb_price_words = split(_curb_price_line, ' ');

					if (_curb_price_words.size() >= 2){
						
						_curb_ID = TInt(std::stoi(_curb_price_words[0]));

						if (curb_factory -> m_curb_price_list_car.find(_curb_ID) == curb_factory -> m_curb_price_list_car.end()){
							curb_factory -> m_curb_price_list_car.insert({_curb_ID, std::vector<TFlt>()});
						}

						for (int j = 1; j < int(_curb_price_words.size()); ++j){
							curb_factory -> m_curb_price_list_car.find(_curb_ID) -> second.push_back(TFlt(std::stof(_curb_price_words[j])));
						}	
					}
					else{
						printf("MNM_IO_Multiclass::build curb_factory_multiclass:curb_price_car wrong length of line input (smaller than 2).\n");
						exit(-1);
					}
				}
			}

			_curb_price_file_truck.open(_curb_price_name_truck, std::ios::in);

			if (_curb_price_file_truck.is_open())
			{
				for (int i = 0; i < _num_of_curb; ++i){
					std::getline(_curb_price_file_truck, _curb_price_line);
					_curb_price_words = split(_curb_price_line, ' ');

					if (_curb_price_words.size() >= 2){
						
						_curb_ID = TInt(std::stoi(_curb_price_words[0]));

						if (curb_factory -> m_curb_price_list_truck.find(_curb_ID) == curb_factory -> m_curb_price_list_truck.end()){
							curb_factory -> m_curb_price_list_truck.insert({_curb_ID, std::vector<TFlt>()});
						}

						for (int j = 1; j < int(_curb_price_words.size()); ++j){
							curb_factory -> m_curb_price_list_truck.find(_curb_ID) -> second.push_back(TFlt(std::stof(_curb_price_words[j])));
						}	
					}
					else{
						printf("MNM_IO_Multiclass::build curb_factory_multiclass:curb_price_truck wrong length of line input (smaller than 2).\n");
						exit(-1);
					}
				}
			}

			_curb_price_file_rh.open(_curb_price_name_rh, std::ios::in);

			if (_curb_price_file_rh.is_open())
			{
				for (int i = 0; i < _num_of_curb; ++i){
					std::getline(_curb_price_file_rh, _curb_price_line);
					_curb_price_words = split(_curb_price_line, ' ');

					if (_curb_price_words.size() >= 2){
						
						_curb_ID = TInt(std::stoi(_curb_price_words[0]));

						if (curb_factory -> m_curb_price_list_rh.find(_curb_ID) == curb_factory -> m_curb_price_list_rh.end()){
							curb_factory -> m_curb_price_list_rh.insert({_curb_ID, std::vector<TFlt>()});
						}

						for (int j = 1; j < int(_curb_price_words.size()); ++j){
							curb_factory -> m_curb_price_list_rh.find(_curb_ID) -> second.push_back(TFlt(std::stof(_curb_price_words[j])));
						}	
					}
					else{
						printf("MNM_IO_Multiclass::build curb_factory_multiclass:curb_price_rh wrong length of line input (smaller than 2).\n");
						exit(-1);
					}
				}
			}
		}
	}

	/* smart loading zone reservation */
	int _curb_reservation_num = conf_reader -> get_int("curb_reservation_num"); // num of lines (OD pairs) for curb reservation

	if (_curb_reservation_num != 0) {
		std::string _od_slz_file_name = file_folder + "/od_slz_reservation";
		std::ifstream _od_slz_file;
		TInt _od_slz_o_node;
		TInt _od_slz_d_node;
		TInt _od_slz_inter_node_ID;
		TInt _od_slz_curb_ID;
		std::string _od_slz_line;
		std::vector<std::string> _od_slz_words;
		std::vector<TInt> _temp_vec;

		_od_slz_file.open(_od_slz_file_name, std::ios::in);
		if (_od_slz_file.is_open()) {
			for (int i = 0; i < _curb_reservation_num; ++i) {
				std::getline(_od_slz_file, _od_slz_line);
				_od_slz_words = split(_od_slz_line, ' ');
				if (_od_slz_words.size() == 4) {
					_od_slz_o_node = TInt(std::stoi(_od_slz_words[0]));
					_od_slz_d_node = TInt(std::stoi(_od_slz_words[1]));
					_od_slz_inter_node_ID = TInt(std::stoi(_od_slz_words[2]));
					_od_slz_curb_ID = TInt(std::stoi(_od_slz_words[3]));

					_temp_vec.push_back(_od_slz_inter_node_ID);
					_temp_vec.push_back(_od_slz_curb_ID);

					if (curb_factory -> m_od_slz_reserve_map.find(_od_slz_o_node) == curb_factory -> m_od_slz_reserve_map.end()){
						curb_factory -> m_od_slz_reserve_map.insert({_od_slz_o_node, std::unordered_map<TInt, std::vector<TInt>>()});
					}

					curb_factory -> m_od_slz_reserve_map.find(_od_slz_o_node) -> second.insert({_od_slz_d_node, _temp_vec});
					_temp_vec.clear();
				}
				else {
					printf("MNM_IO_Multiclass::build curb_factory_multiclass:od_slz_reservation wrong length of line.\n");
					exit(-1);
				}

			}
			_od_slz_file.close();
		}
	}

	return 0;
}

int MNM_IO_Multiclass_Curb::load_inter_dest_curb_separate(const std::string& file_folder,
 									 						MNM_ConfReader *conf_reader,
															MNM_Curb_Factory_Multiclass *curb_factory)
{
	std::unordered_map<TInt, std::vector<TInt>> _path_inter_dest_curb;

	/* find file */
	std::string _file_name = file_folder + "/path_inter_dest_curb";
	std::ifstream _file;
	_file.open(_file_name, std::ios::in);

	TInt _num_path = conf_reader -> get_int("num_of_path_curb");

	TInt _num_inter_dest, _inter_dest_ID, _inter_curb_ID;

	std::string _line;
	std::vector<std::string> _words;
	std::vector<TInt> _num_vector;
	if (_file.is_open())
	{
		std::getline(_file,_line); //skip the first line

		for (int i = 0; i < _num_path; ++i){
			std::getline(_file,_line);
			_words = split(_line, ' ');

			// 0604
			_num_inter_dest = TInt(std::stoi(_words[0]));
			if (_num_inter_dest != TInt(0)){
				IAssert(_num_inter_dest * 2 + 1 == TInt(_words.size()));
				for (int j = 1; j < int(_words.size()); ++j){
					_num_vector.push_back(TInt(std::stoi(_words[j])));
				}
				_path_inter_dest_curb.insert({TInt(i), _num_vector});
				_num_vector.clear();
			}
		}
		_file.close();
	}

	curb_factory -> m_path_inter_dest_map = _path_inter_dest_curb;

	// RH
	std::unordered_map<TInt, std::vector<TInt>> _path_inter_dest_curb_rh;

	/* find file */
	std::string _file_name_rh = file_folder + "/path_inter_dest_curb_rh";
	std::ifstream _file_rh;
	_file_rh.open(_file_name_rh, std::ios::in);

	TInt _num_path_rh = conf_reader -> get_int("num_of_path_curb_rh");

	if (_file_rh.is_open())
	{
		std::getline(_file_rh,_line); //skip the first line

		for (int i = 0; i < _num_path_rh; ++i){
			std::getline(_file_rh,_line);
			_words = split(_line, ' ');

			_num_inter_dest = TInt(std::stoi(_words[0]));

			IAssert(_num_inter_dest * 2 + 1 == TInt(_words.size()));

			for (int j = 1; j < int(_words.size()); ++j){
				_num_vector.push_back(TInt(std::stoi(_words[j])));
			}

			_path_inter_dest_curb_rh.insert({TInt(i), _num_vector});
			_num_vector.clear();

		}
		_file_rh.close();
	}

	curb_factory -> m_path_inter_dest_map_rh = _path_inter_dest_curb_rh;

	// car (only for DUE)
	TInt _load_car = conf_reader -> get_int("load_car_curb_choice");

	if (_load_car == 1){
		std::unordered_map<TInt, std::vector<TInt>> _path_inter_dest_curb_car;

		/* find file */
		std::string _file_name_car = file_folder + "/path_inter_dest_curb_car";
		std::ifstream _file_car;
		_file_car.open(_file_name_car, std::ios::in);

		TInt _num_path_car = conf_reader -> get_int("num_of_path_curb_car");

		if (_file_car.is_open()){
			std::getline(_file_car,_line); //skip the first line
			for (int i = 0; i < _num_path_car; ++i){
				std::getline(_file_car,_line);
				_words = split(_line, ' ');
				_num_inter_dest = TInt(std::stoi(_words[0]));
				if (_num_inter_dest != TInt(0)){
					IAssert(_num_inter_dest * 2 + 1 == TInt(_words.size()));
					for (int j = 1; j < int(_words.size()); ++j){
						_num_vector.push_back(TInt(std::stoi(_words[j])));
					}
					_path_inter_dest_curb_car.insert({TInt(i), _num_vector});
					_num_vector.clear();
				}
			}
			_file_car.close();
		}

		curb_factory -> m_path_inter_dest_map_car = _path_inter_dest_curb_car;
	}

	return 0;
}

int MNM_IO_Multiclass_Curb::build_od_factory_multiclass_curb(const std::string& file_folder, 
															MNM_ConfReader *conf_reader,
															MNM_OD_Factory *od_factory, 
															MNM_Node_Factory *node_factory, 
															const std::string& file_name)
{

	/* find file */
	std::string _od_file_name = file_folder + "/" + file_name;
	std::ifstream _od_file;
	_od_file.open(_od_file_name, std::ios::in);

	/* read config */
	TInt _num_of_O = conf_reader -> get_int("num_of_O");
	TInt _num_of_D = conf_reader -> get_int("num_of_D");
	TFlt _flow_scalar = conf_reader -> get_float("flow_scalar");
	TInt _max_interval = conf_reader -> get_int("max_interval");
	TInt _frequency = conf_reader -> get_int("assign_frq");

	/* build */
	TInt _dest_ID, _origin_ID, _node_ID;
	std::string _line;
	std::vector<std::string> _words;
	MNM_Origin *_origin;
	MNM_Destination *_dest;
	if (_od_file.is_open())
	{
		// printf("Start build Origin-Destination factory.\n");
		std::getline(_od_file,_line); //skip the first line
		// printf("Processing Origin node.\n");
		for (int i=0; i < _num_of_O; ++i){
			std::getline(_od_file,_line);
			_words = split(_line, ' ');
			if (_words.size() == 2) {
				// std::cout << "Processing: " << _line << "\n";
				_origin_ID = TInt(std::stoi(_words[0]));
				_node_ID = TInt(std::stoi(_words[1]));
				_origin = od_factory -> make_origin(_origin_ID, _max_interval, _flow_scalar, _frequency);

				/* hook up */
				_origin ->  m_origin_node =  (MNM_DMOND*) node_factory -> get_node(_node_ID);
				((MNM_DMOND*)  node_factory -> get_node(_node_ID)) -> hook_up_origin(_origin);
			}
		}
		std::getline(_od_file,_line); // skip another line
		// printf("Processing Destination node.\n");
		for (int i=0; i < _num_of_D; ++i){
			std::getline(_od_file,_line);
			_words = split(_line, ' ');
			if (_words.size() == 2) {
				// std::cout << "Processing: " << _line << "\n";
				_dest_ID = TInt(std::stoi(_words[0]));
				_node_ID = TInt(std::stoi(_words[1]));
				_dest = od_factory -> make_destination(_dest_ID);

				/* hook up */
				_dest ->  m_dest_node =  (MNM_DMDND*) node_factory -> get_node(_node_ID);
				((MNM_DMDND*)  node_factory -> get_node(_node_ID)) -> hook_up_destination(_dest);
			}
		}      
	}
	_od_file.close();
	return 0;
}

PNEGraph MNM_IO_Multiclass_Curb::build_graph(const std::string& file_folder, MNM_ConfReader *conf_reader)
{
	/* find file */
	std::string _network_name = conf_reader -> get_string("network_name");
	std::string _graph_file_name = file_folder + "/" + _network_name;
	std::ifstream _graph_file;
	_graph_file.open(_graph_file_name, std::ios::in);

	TInt _num_of_link = conf_reader -> get_int("num_of_link");

	PNEGraph _graph = PNEGraph::TObj::New();
	
	int _link_ID, _from_ID, _to_ID;
	std::string _line;
	std::vector<std::string> _words;
	std::getline(_graph_file,_line); // skip one line
	for (int i = 0; i < _num_of_link; ++i){
		std::getline(_graph_file,_line);
		_words = split(_line, ' ');
		if (_words.size() == 3){
			_link_ID = TInt(std::stoi(_words[0]));
			_from_ID = TInt(std::stoi(_words[1]));
			_to_ID = TInt(std::stoi(_words[2]));
			if (! _graph -> IsNode(_from_ID)) { _graph -> AddNode(_from_ID); }
			if (! _graph -> IsNode(_to_ID)) { _graph -> AddNode(_to_ID); }
			_graph -> AddEdge(_from_ID, _to_ID, _link_ID);
		}
	}
	_graph -> Defrag();
	IAssert(_graph -> GetEdges() == _num_of_link);
	return _graph;
}

namespace MNM_DTA_GRADIENT_CURB
{

TFlt get_link_inflow_car(MNM_Dlink_Multiclass_Curb* link, 
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

TFlt get_link_inflow_rh(MNM_Dlink_Multiclass_Curb* link,
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

TFlt get_link_inflow_truck(MNM_Dlink_Multiclass_Curb *link, TFlt start_time, TFlt end_time)
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

	return link -> m_N_in_truck -> get_result(TFlt(end_time)) - link -> m_N_in_truck -> get_result(TFlt(start_time)) + link -> m_N_in_truck_cc -> get_result(TFlt(end_time)) - link -> m_N_in_truck_cc -> get_result(TFlt(start_time));
}

TFlt get_travel_time_car(MNM_Dlink_Multiclass_Curb* link, TFlt start_time, TFlt unit_interval, TInt end_loading_timestamp)
{
	if (link == nullptr){
		throw std::runtime_error("Error, get_travel_time_car link is null");
	}
	if (link -> m_N_in_car == nullptr){
		throw std::runtime_error("Error, get_travel_time_car link cumulative curve is not installed");
	}

	TFlt fftt = TFlt(int(link -> get_link_freeflow_tt_loading_car())); // actual intervals in loading

	if (link -> m_last_valid_time < 0){
		link -> m_last_valid_time = MNM_DTA_GRADIENT::get_last_valid_time(link -> m_N_in_car_all, link -> m_N_out_car_all, end_loading_timestamp);
	}

	IAssert(link -> m_last_valid_time >= 0);

	return MNM_DTA_GRADIENT::get_travel_time_from_cc(start_time, link -> m_N_in_car_all, link -> m_N_out_car_all, link -> m_last_valid_time, fftt);
}

TFlt get_travel_time_truck(MNM_Dlink_Multiclass_Curb* link, TFlt start_time, TFlt unit_interval, TInt end_loading_timestamp)
{
	if (link == nullptr){
		throw std::runtime_error("Error, get_travel_time_truck link is null");
	}
	if (link -> m_N_in_truck == nullptr){
		throw std::runtime_error("Error, get_travel_time_truck link cumulative curve is not installed");
	}

	TFlt fftt = TFlt(int(link -> get_link_freeflow_tt_loading_truck())); // actual intervals in loading

	if (link -> m_last_valid_time < 0){
		link -> m_last_valid_time = MNM_DTA_GRADIENT::get_last_valid_time(link -> m_N_in_truck, link -> m_N_out_truck, end_loading_timestamp);
	}

	IAssert(link -> m_last_valid_time >= 0);

	return MNM_DTA_GRADIENT::get_travel_time_from_cc(start_time, link -> m_N_in_truck, link -> m_N_out_truck, link -> m_last_valid_time, fftt);
}

TFlt get_curb_inflow_car(MNM_Dlink_Multiclass_Curb* link, TFlt start_time, TFlt end_time)
{
	if (link == nullptr){
		throw std::runtime_error("Error, get curb inflow car -->> link is null");
	}

	if (link -> m_N_in_car_cc == nullptr){
		throw std::runtime_error("Error, get curb inflow car -->> link cumulative curve cc (curb) is not installed");
	}

	return link -> m_N_in_car_cc -> get_result(TFlt(end_time)) - link -> m_N_in_car_cc -> get_result(TFlt(start_time));
}

TFlt get_curb_outflow_car(MNM_Dlink_Multiclass_Curb* link, TFlt start_time, TFlt end_time)
{
	if (link == nullptr){
		throw std::runtime_error("Error, get curb outflow car -->> link is null");
	}

	if (link -> m_N_out_car_cc == nullptr){
		throw std::runtime_error("Error, get curb outflow car -->> link cumulative curve cc (curb) is not installed");
	}

	return link -> m_N_out_car_cc -> get_result(TFlt(end_time)) - link -> m_N_out_car_cc -> get_result(TFlt(start_time));
} 

TFlt get_curb_inflow_truck(MNM_Dlink_Multiclass_Curb* link, TFlt start_time, TFlt end_time)
{
	if (link == nullptr){
		throw std::runtime_error("Error, get curb inflow truck -->> link is null");
	}

	if (link -> m_N_in_truck_cc == nullptr){
		throw std::runtime_error("Error, get curb inflow truck -->> link cumulative curve cc (curb) is not installed");
	}

	return link -> m_N_in_truck_cc -> get_result(TFlt(end_time)) - link -> m_N_in_truck_cc -> get_result(TFlt(start_time));
}

TFlt get_curb_outflow_truck(MNM_Dlink_Multiclass_Curb* link, TFlt start_time, TFlt end_time)
{
	if (link == nullptr){
		throw std::runtime_error("Error, get curb outflow truck -->> link is null");
	}

	if (link -> m_N_out_truck_cc == nullptr){
		throw std::runtime_error("Error, get curb outflow truck -->> link cumulative curve cc (curb) is not installed");
	}

	return link -> m_N_out_truck_cc -> get_result(TFlt(end_time)) - link -> m_N_out_truck_cc -> get_result(TFlt(start_time));
}

TFlt get_curb_inflow_rh(MNM_Dlink_Multiclass_Curb* link, TFlt start_time, TFlt end_time)
{
	if (link == nullptr){
		throw std::runtime_error("Error, get curb inflow rh -->> link is null");
	}

	if (link -> m_N_in_rh_cc == nullptr){
		throw std::runtime_error("Error, get curb inflow rh -->> link cumulative curve cc (curb) is not installed");
	}

	return link -> m_N_in_rh_cc -> get_result(TFlt(end_time)) - link -> m_N_in_rh_cc -> get_result(TFlt(start_time));
}

TFlt get_curb_outflow_rh(MNM_Dlink_Multiclass_Curb* link, TFlt start_time, TFlt end_time)
{
	if (link == nullptr){
		throw std::runtime_error("Error, get curb outflow rh -->> link is null");
	}

	if (link -> m_N_out_rh_cc == nullptr){
		throw std::runtime_error("Error, get curb outflow rh -->> link cumulative curve cc (curb) is not installed");
	}

	return link -> m_N_out_rh_cc -> get_result(TFlt(end_time)) - link -> m_N_out_rh_cc -> get_result(TFlt(start_time));
}

TFlt get_link_density_car_robust(MNM_Dlink_Multiclass_Curb* link, TFlt time, TInt end_loading_timestamp, TInt num_trials)
{
	if (link == nullptr){
    	throw std::runtime_error ("Error, get_link_density_car link is null");
    }
  	if ((link -> m_N_in_car == nullptr) || (link -> m_N_out_car == nullptr)){
    	throw std::runtime_error ("Error, get_link_density_car link cumulative curve is not installed");
    }

	if (link -> m_last_valid_time < 0){
    	link -> m_last_valid_time = MNM_DTA_GRADIENT::get_last_valid_time(link -> m_N_in_car, link -> m_N_out_car, end_loading_timestamp);
    }

  	IAssert (link -> m_last_valid_time >= 0);
	
	TFlt _total_density = 0.0;
	TFlt _time_tmp;

	for (int i = 0; i < num_trials; ++i){
		_time_tmp = TFlt(i - 2 + time) > TFlt(0) ? TFlt(i - 2 + time) : TFlt(0);
		_total_density += MNM_DTA_GRADIENT::get_link_density_from_cc(_time_tmp, link -> m_N_in_car, link -> m_N_out_car, link -> m_last_valid_time);
	}

	return _total_density/num_trials;
}

TFlt get_link_density_rh_robust(MNM_Dlink_Multiclass_Curb* link, TFlt time, TInt end_loading_timestamp, TInt num_trials)
{
	if (link == nullptr){
		throw std::runtime_error ("Error, get_link_density_rh_robust link is null");
	}
  	if ((link -> m_N_in_rh == nullptr) || (link -> m_N_out_rh == nullptr)){
		throw std::runtime_error ("Error, get_link_density_rh_robust link cumulative curve is not installed");
	}

	if (link -> m_last_valid_time < 0){
		link -> m_last_valid_time = MNM_DTA_GRADIENT::get_last_valid_time(link -> m_N_in_rh, link -> m_N_out_rh, end_loading_timestamp);
	}

  	IAssert (link -> m_last_valid_time >= 0);

	TFlt _total_density = 0.0;
	TFlt _time_tmp;

	for (int i = 0; i < num_trials; ++i){
		_time_tmp = TFlt(i - 2 + time) > TFlt(0) ? TFlt(i - 2 + time) : TFlt(0);
		_total_density += MNM_DTA_GRADIENT::get_link_density_from_cc(_time_tmp, link -> m_N_in_rh, link -> m_N_out_rh, link -> m_last_valid_time);
	}

	return _total_density/num_trials;
}

TFlt get_link_density_stop_rh_robust(MNM_Dlink_Multiclass_Curb* link, TFlt time, TInt end_loading_timestamp, TInt num_trials)
{
	if (link == nullptr){
    	throw std::runtime_error ("Error, get_link_density_stop_rh_robust link is null");
    }
	
  	if (link -> m_N_in_rh_cc == nullptr){
    	throw std::runtime_error ("Error, get_link_density_stop_rh_robust link cumulative curve is not installed");
    }

	if (link -> m_last_valid_time < 0){
    	link -> m_last_valid_time = MNM_DTA_GRADIENT::get_last_valid_time(link -> m_N_in_rh_cc, link -> m_N_out_rh_cc, end_loading_timestamp);
    }

  	IAssert (link -> m_last_valid_time >= 0);

	TFlt _total_density = 0.0;
	TFlt _time_tmp;

	for (int i = 0; i < num_trials; ++i){
		_time_tmp = TFlt(i - 2 + time) > TFlt(0) ? TFlt(i - 2 + time) : TFlt(0);
		_total_density += MNM_DTA_GRADIENT::get_link_density_from_cc(_time_tmp, link -> m_N_in_rh_cc, link -> m_N_out_rh_cc, link -> m_last_valid_time);
	}

	return _total_density/num_trials;
} 

TFlt get_link_density_truck_robust(MNM_Dlink_Multiclass_Curb* link, TFlt time, TInt end_loading_timestamp, TInt num_trials)
{
	if (link == nullptr){
    	throw std::runtime_error ("Error, get_link_density_truck link is null");
    }
  	if ((link -> m_N_in_truck == nullptr) || (link -> m_N_out_truck == nullptr)){
    	throw std::runtime_error ("Error, get_link_density_truck link cumulative curve is not installed");
    }

	if (link -> m_last_valid_time < 0){
    	link -> m_last_valid_time = MNM_DTA_GRADIENT::get_last_valid_time(link -> m_N_in_truck, link -> m_N_out_truck, end_loading_timestamp);
    }

  	IAssert (link -> m_last_valid_time >= 0);

	TFlt _total_density = 0.0;
	TFlt _time_tmp;

	for (int i = 0; i < num_trials; ++i){
		_time_tmp = TFlt(i - 2 + time) > TFlt(0) ? TFlt(i - 2 + time) : TFlt(0);
		_total_density += MNM_DTA_GRADIENT::get_link_density_from_cc(_time_tmp, link -> m_N_in_truck, link -> m_N_out_truck, link -> m_last_valid_time);
	}

	return _total_density/num_trials;
}

TFlt get_link_density_stop_truck_robust(MNM_Dlink_Multiclass_Curb* link, TFlt time, TInt end_loading_timestamp, TInt num_trials)
{
	if (link == nullptr){
    	throw std::runtime_error ("Error, get_link_density_stop_truck_robust link is null");
    }

  	if (link -> m_N_in_truck_cc == nullptr){
    	throw std::runtime_error ("Error, get_link_density_stop_truck_robust link cumulative curve is not installed");
    }

	if (link -> m_last_valid_time < 0){
    	link -> m_last_valid_time = MNM_DTA_GRADIENT::get_last_valid_time(link -> m_N_in_truck_cc, link -> m_N_out_truck_cc, end_loading_timestamp);
    }

  	IAssert (link -> m_last_valid_time >= 0);

	TFlt _total_density = 0.0;
	TFlt _time_tmp;

	for (int i = 0; i < num_trials; ++i){
		_time_tmp = TFlt(i - 2 + time) > TFlt(0) ? TFlt(i - 2 + time) : TFlt(0);
		_total_density += MNM_DTA_GRADIENT::get_link_density_from_cc(_time_tmp, link -> m_N_in_truck_cc, link -> m_N_out_truck_cc, link -> m_last_valid_time);
	}

	return _total_density/num_trials;
}

int add_dar_records_car(std::vector<dar_record*> &record, MNM_Dlink_Multiclass_Curb* link, 
						std::set<MNM_Path*> pathset, TFlt start_time, TFlt end_time)
{
	if (link == nullptr){
		throw std::runtime_error("Error, add dar records car link is null");
	}
	if (link -> m_N_in_tree_car == nullptr){
		throw std::runtime_error("Error, add dar records car link cumulative curve tree is not installed");
	}
	MNM_Path* _path;
	for (auto path_it : link -> m_N_in_tree_car -> m_record)
	{
		_path = path_it.first;
		if (pathset.find(_path) != pathset.end()) 
		{
			for (auto depart_it : path_it.second){
				TFlt tmp_flow = depart_it.second -> get_result(end_time) - depart_it.second -> get_result(start_time);
				if (tmp_flow > DBL_EPSILON){
					auto new_record = new dar_record();
					new_record -> path_ID = path_it.first -> m_path_ID;
					new_record -> assign_int = depart_it.first;
					new_record -> link_ID = link -> m_link_ID;
					new_record -> link_start_int = start_time;
					new_record -> flow = tmp_flow;
					record.push_back(new_record);
				}
			}
		}
	}
	return 0;
}

int add_dar_records_truck(std::vector<dar_record*> &record, MNM_Dlink_Multiclass_Curb* link, 
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
					new_record -> assign_int = depart_it.first;
					new_record -> link_ID = link -> m_link_ID;
					new_record -> link_start_int = start_time;
					new_record -> flow = tmp_flow;
					record.push_back(new_record);
				}
			}
		}
	}
	return 0;
}

int add_dar_records_rh(std::vector<dar_record*> &record, MNM_Dlink_Multiclass_Curb* link, 
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
					new_record -> assign_int = depart_it.first;
					new_record -> link_ID = link -> m_link_ID;
					new_record -> link_start_int = start_time;
					new_record -> flow = tmp_flow;
					record.push_back(new_record);
				}
			}
		}
	}
	return 0;
}

int add_dar_records_car_out(std::vector<dar_record*> &record, MNM_Dlink_Multiclass_Curb* link, 
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
					new_record -> assign_int = depart_it.first;
					new_record -> link_ID = link -> m_link_ID;
					new_record -> link_start_int = start_time;
					new_record -> flow = tmp_flow;
					record.push_back(new_record);
				}
			}
		}
	}
	return 0;
}

int add_dar_records_truck_out(std::vector<dar_record*> &record, MNM_Dlink_Multiclass_Curb* link, 
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
					new_record -> assign_int = depart_it.first;
					new_record -> link_ID = link -> m_link_ID;
					new_record -> link_start_int = start_time;
					new_record -> flow = tmp_flow;
					record.push_back(new_record);
				}
			}
		}
	}
	return 0;
}
int add_dar_records_rh_out(std::vector<dar_record*> &record, MNM_Dlink_Multiclass_Curb* link,
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
					new_record -> assign_int = depart_it.first;
					new_record -> link_ID = link -> m_link_ID;
					new_record -> link_start_int = start_time;
					new_record -> flow = tmp_flow;
					record.push_back(new_record);
				}
			}
		}
	}
	return 0;
}

int add_dar_records_curb_arrival_truck(std::vector<dar_record*> &record, MNM_Dlink_Multiclass_Curb* link, 
					std::set<MNM_Path*> pathset, TFlt start_time, TFlt end_time)
{
	if (link == nullptr){
		throw std::runtime_error("Error, add dar records curb arrival truck link is null");
	}
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
					new_record -> assign_int = depart_it.first;
					new_record -> link_ID = link -> m_link_ID;
					new_record -> link_start_int = start_time;
					new_record -> flow = tmp_flow;
					record.push_back(new_record);
				}
			}
		}
	}
	return 0;
}

int add_dar_records_curb_departure_truck(std::vector<dar_record*> &record, MNM_Dlink_Multiclass_Curb* link, 
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
			for (auto depart_it : path_it.second) {
				TFlt tmp_flow = depart_it.second -> get_result(end_time) - depart_it.second -> get_result(start_time);
				if (tmp_flow > DBL_EPSILON) {
					auto new_record = new dar_record();
					new_record -> path_ID = path_it.first -> m_path_ID;
					new_record -> assign_int = depart_it.first;
					new_record -> link_ID = link -> m_link_ID;
					new_record -> link_start_int = start_time;
					new_record -> flow = tmp_flow;
					record.push_back(new_record);
				}
			}
		}
	}
	return 0;
}

int add_dar_records_curb_arrival_rh(std::vector<dar_record*> &record, MNM_Dlink_Multiclass_Curb* link, 
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
					new_record -> assign_int = depart_it.first;
					new_record -> link_ID = link -> m_link_ID;
					new_record -> link_start_int = start_time;
					new_record -> flow = tmp_flow;
					record.push_back(new_record);
				}
			}
		}
	}
	return 0;
}

int add_dar_records_curb_departure_rh(std::vector<dar_record*> &record, MNM_Dlink_Multiclass_Curb* link, 
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
					new_record -> assign_int = depart_it.first;
					new_record -> link_ID = link -> m_link_ID;
					new_record -> link_start_int = start_time;
					new_record -> flow = tmp_flow;
					record.push_back(new_record);
				}
			}
		}
	}	
	return 0;
}

int add_dar_records_curb_arrival_car(std::vector<dar_record*> &record, MNM_Dlink_Multiclass_Curb* link, 
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
					new_record -> assign_int = depart_it.first;
					new_record -> link_ID = link -> m_link_ID;
					new_record -> link_start_int = start_time;
					new_record -> flow = tmp_flow;
					record.push_back(new_record);
				}
			}
		}
	}
	return 0;
}

int add_dar_records_curb_departure_car(std::vector<dar_record*> &record, MNM_Dlink_Multiclass_Curb* link, 
					std::set<MNM_Path*> pathset, TFlt start_time, TFlt end_time)
{
	if (link == nullptr){
		throw std::runtime_error("Error, add dar records curb departure car link is null");
	}
	if (link -> m_N_out_tree_curb_car == nullptr){
		throw std::runtime_error("Error, arrival curb CC tree for car on this link is not installed");
	}
	MNM_Path* _path;
	for (auto path_it : link -> m_N_out_tree_curb_car -> m_record) {
		_path = path_it.first;
		if (pathset.find(_path) != pathset.end()) {
			for (auto depart_it : path_it.second) {
				TFlt tmp_flow = depart_it.second -> get_result(end_time) - depart_it.second -> get_result(start_time);
				if (tmp_flow > DBL_EPSILON) {
					auto new_record = new dar_record();
					new_record -> path_ID = path_it.first -> m_path_ID;
					new_record -> assign_int = depart_it.first;
					new_record -> link_ID = link -> m_link_ID;
					new_record -> link_start_int = start_time;
					new_record -> flow = tmp_flow;
					record.push_back(new_record);
				}
			}
		}
	}
	return 0;
}

}// end namespace MNM_DTA_GRADIENT_CURB
