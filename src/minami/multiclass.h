#ifndef MULTICLASS_H
#define MULTICLASS_H

#include "dlink.h"
#include "dnode.h"
#include "vehicle.h"
#include "dta.h"
#include "dta_gradient_utls.h"
#include "emission.h"
#include "routing.h"

#include "Snap.h"
#include "ults.h"
#include "factory.h"
#include "enum.h"
#include "path.h"
#include "vms.h"
#include "workzone.h"

#include <string>
#include <vector>

/******************************************************************************************************************
*******************************************************************************************************************
												Link Models
*******************************************************************************************************************
******************************************************************************************************************/

class MNM_Dlink_Multiclass : public MNM_Dlink
{
public:
	MNM_Dlink_Multiclass(TInt ID,
						TInt number_of_lane,
						TFlt length,
						TFlt ffs_car, 
						TFlt ffs_truck);
	virtual ~MNM_Dlink_Multiclass() override;

	// use this one instead of the one in Dlink class
	int install_cumulative_curve_multiclass();
	// use this one instead of the one in Dlink class
	int install_cumulative_curve_tree_multiclass();
	// jiachao 0605
	int install_cumulative_curve_tree_multiclass_curb();

    virtual TFlt get_link_flow_car(){return 0;};
    virtual TFlt get_link_flow_truck(){return 0;};

    virtual TFlt get_link_tt_from_flow_car(TFlt flow){return 0;};
    virtual TFlt get_link_tt_from_flow_truck(TFlt flow){return 0;};
	TFlt get_link_freeflow_tt_car();
	TFlt get_link_freeflow_tt_truck();

	virtual TInt get_link_freeflow_tt_loading_car() {return -1;};  // intervals
	virtual TInt get_link_freeflow_tt_loading_truck() {return -1;};  // intervals

	DLink_type_multiclass m_link_type;

	TFlt m_ffs_car;
	TFlt m_ffs_truck;

	TFlt m_tot_wait_time_at_intersection; // seconds
	TFlt m_tot_wait_time_at_intersection_car; //seconds
	TFlt m_tot_wait_time_at_intersection_truck; //seconds
	bool m_spill_back;

	// Two seperate N-curves for private cars and trucks
	MNM_Cumulative_Curve *m_N_in_car;
  	MNM_Cumulative_Curve *m_N_out_car;

  	MNM_Cumulative_Curve *m_N_in_truck;
  	MNM_Cumulative_Curve *m_N_out_truck;

	MNM_Cumulative_Curve *m_N_in_rh;
	MNM_Cumulative_Curve *m_N_out_rh;

	MNM_Cumulative_Curve *m_N_in_car_cc; // cc: curb count
	MNM_Cumulative_Curve *m_N_out_car_cc;

  	MNM_Cumulative_Curve *m_N_in_truck_cc;
  	MNM_Cumulative_Curve *m_N_out_truck_cc;

	MNM_Cumulative_Curve *m_N_in_rh_cc;
	MNM_Cumulative_Curve *m_N_out_rh_cc;

	// Cumulative curve used to compute TT for car
	// N_in = N_in_car + N_in_rh; N_out = N_out_car + N_out_rh
	MNM_Cumulative_Curve *m_N_in_car_all;
	MNM_Cumulative_Curve *m_N_out_car_all;

  	// Two seperate N-curve_trees for private cars and trucks, tree is for DAR and separate for three modes
	MNM_Tree_Cumulative_Curve *m_N_in_tree_car;
  	MNM_Tree_Cumulative_Curve *m_N_out_tree_car;
  	MNM_Tree_Cumulative_Curve *m_N_in_tree_truck;
  	MNM_Tree_Cumulative_Curve *m_N_out_tree_truck;
	MNM_Tree_Cumulative_Curve *m_N_in_tree_rh;
  	MNM_Tree_Cumulative_Curve *m_N_out_tree_rh;	

	// Qiling & Jiachao
	TFlt m_last_valid_time_truck = TFlt(-1);

	// new added CC trees
	MNM_Tree_Cumulative_Curve *m_N_in_tree_curb_truck;
	MNM_Tree_Cumulative_Curve *m_N_in_tree_curb_rh;
	MNM_Tree_Cumulative_Curve *m_N_out_tree_curb_truck;
	MNM_Tree_Cumulative_Curve *m_N_out_tree_curb_rh;

	// new added CC trees for cars - Jiachao
	MNM_Tree_Cumulative_Curve *m_N_in_tree_curb_car;
	MNM_Tree_Cumulative_Curve *m_N_out_tree_curb_car;

	// PMC project - link level record congestion regime and differentiability
	bool m_diff_car;
	bool m_diff_truck;
	int m_congested_car; // regime 1 - free flow ; 2 - semi-congested ; 3 - fully-congested.
	int m_congested_truck;
	TFlt m_space_fraction_car; // space fraction of car
	TFlt m_space_fraction_truck; // space fraction of truck

	// PMC end

	// track parking vehicle numbers
	TInt m_parking_num_car;
	TInt m_parking_num_truck;
	TInt m_parking_num_rh;
};


/**************************************************************************
							Multiclass CTM
			(currently only for car & truck two classes)
	(see: Z. (Sean) Qian et al./Trans. Res. Part B 99 (2017) 183-204)
**************************************************************************/
class MNM_Dlink_Ctm_Multiclass : public MNM_Dlink_Multiclass
{
public:
	MNM_Dlink_Ctm_Multiclass(TInt ID,
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
							 TInt curb_dest);

	virtual ~MNM_Dlink_Ctm_Multiclass() override;
    virtual int evolve(TInt timestamp, TFlt _ratio_lane_closure) override;

	// Curb-Jiachao added in Aug 29 for modeling curb-link dynamic
	virtual int evolve_curb(TInt timestamp) override;

	virtual int evolve_control(TInt timestamp, TFlt _ratio_lane_closure, TFlt cell_position, TFlt cell_control_rate) override;

    virtual TFlt get_link_supply() override;
	// Jiachao added
	virtual TFlt get_link_capacity() override;
    virtual int clear_incoming_array(TInt timestamp, TFlt _ratio_lane_closure) override;
    virtual void print_info() override;

    virtual TFlt get_link_flow_car() override;
    virtual TFlt get_link_flow_truck() override;
    virtual TFlt get_link_flow() override;
    virtual TFlt get_link_tt() override;
    virtual TFlt get_link_tt_from_flow_car(TFlt flow) override;
    virtual TFlt get_link_tt_from_flow_truck(TFlt flow) override;

	virtual TInt get_link_freeflow_tt_loading_car() override;  // intervals
	virtual TInt get_link_freeflow_tt_loading_truck() override;  // intervals

    virtual int move_veh_queue(std::deque<MNM_Veh*> *from_queue,
						       std::deque<MNM_Veh*> *to_queue,
						       TInt number_tomove) override;

	int move_veh_queue_curb_biclass(std::deque<MNM_Veh*> *from_queue_car, // m_cell_array[i] -> m_veh_queue_car
									std::deque<MNM_Veh*> *from_queue_truck, // m_cell_array[i] -> m_veh_queue_truck
									std::deque<MNM_Veh*> *from_queue_curb,// m_curb_cell_array[i] -> m_veh_departing
									std::deque<MNM_Veh*> *to_queue_car, // m_cell_array[i+1] -> m_veh_queue_car
									std::deque<MNM_Veh*> *to_queue_truck, // m_cell_array[i+1] -> m_veh_queue_truck
									std::deque<MNM_Veh*> *to_queue_curb, // m_curb_cell_array[i] -> m_veh_arriving
									std::deque<MNM_Veh*> *to_queue_car_dp, // m_curb_cell_array[i] -> m_veh_doubleparking_car
									std::deque<MNM_Veh*> *to_queue_truck_dp,
									std::deque<MNM_Veh*> *to_queue_rh_dp,
									TInt number_tomove_car,
									TInt number_tomove_truck,
									TInt cell_ID,
									TInt timestamp);


	class Ctm_Cell_Multiclass;
	int init_cell_array(TFlt unit_time, TFlt std_cell_length, TFlt last_cell_length);
	int update_out_veh(TFlt _ratio_lane_closure);
	// Jiachao added in Aug 29
	int update_out_veh_curb(std::vector<TFlt> _ratio_lane_closure_list);

	int update_out_veh_control(TFlt _ratio_lane_closure, TInt _control_cell_ID, TFlt _control_cell_rate);
	int move_last_cell();
	int move_last_cell_curb(TInt timestamp);
	
	TFlt m_unit_time;
	TInt m_num_cells;
	TFlt m_lane_hold_cap_car;
	TFlt m_lane_hold_cap_truck;
	TFlt m_lane_critical_density_car;
	TFlt m_lane_critical_density_truck;
	TFlt m_lane_rho_1_N;
	TFlt m_lane_flow_cap_car;
	TFlt m_lane_flow_cap_truck;
	TFlt m_veh_convert_factor;
	TFlt m_flow_scalar;
	TFlt m_wave_speed_car;
	TFlt m_wave_speed_truck;
	// Jiachao added
	TInt m_curb_spaces;
	TInt m_curb_dest;

	std::vector<Ctm_Cell_Multiclass*> m_cell_array;
	
	class Cell_Curb_Multiclass;
	std::vector<Cell_Curb_Multiclass*> m_curb_cell_array;

	int install_curb();
};

class MNM_Dlink_Ctm_Multiclass::Cell_Curb_Multiclass
{
public:
	Cell_Curb_Multiclass(TInt cell_ID,
						 TInt link_ID,
						 TInt dest_ID,
						 TInt cell_capacity,
						 TFlt unit_time);
	~Cell_Curb_Multiclass();
	std::deque<MNM_Veh*> m_veh_parking_car;
	std::deque<MNM_Veh*> m_veh_parking_truck;
	std::deque<MNM_Veh*> m_veh_parking_rh;

	std::deque<MNM_Veh*> m_veh_doubleparking_car;
	std::deque<MNM_Veh*> m_veh_doubleparking_truck;
	std::deque<MNM_Veh*> m_veh_doubleparking_rh;

	// m_veh_departing/arriving_car/truck
	std::deque<MNM_Veh*> m_veh_departing;
	std::deque<MNM_Veh*> m_veh_departing_car;
	std::deque<MNM_Veh*> m_veh_departing_truck;
	// std::deque<MNM_Veh*> m_veh_departing_truck;

	std::deque<MNM_Veh*> m_veh_arriving;
	// std::deque<MNM_Veh*> m_veh_arriving_truck;

	TInt m_cell_ID;
	TInt m_link_ID;
	TInt m_dest_ID;
	TInt m_cell_capacity;
	TFlt m_unit_time;
	TFlt m_current_time;
	TInt m_veh_parking_num;
	TInt m_veh_doubleparking_num;

	int move_veh_in_curb(TInt timestamp);
						//  std::deque<MNM_Veh*> *from_queue,
						//  std::deque<MNM_Veh*> *to_queue_car,
						//  std::deque<MNM_Veh*> *to_queue_truck,
						//  std::deque<MNM_Veh*> *to_queue_rh);

	int move_veh_out_curb(TInt timestamp);
						//   std::deque<MNM_Veh*> *from_queue_car,
						//   std::deque<MNM_Veh*> *from_queue_truck,
						//   std::deque<MNM_Veh*> *from_queue_rh,
                        //   std::deque<MNM_Veh*> *to_queue);

};

class MNM_Dlink_Ctm_Multiclass::Ctm_Cell_Multiclass
{
public:
	Ctm_Cell_Multiclass(TInt cell_ID,
	                    TFlt cell_length,
						TFlt unit_time,
						TFlt hold_cap_car,
						TFlt hold_cap_truck, 
						TFlt critical_density_car,
						TFlt critical_density_truck,
						TFlt m_rho_1_N,
						TFlt flow_cap_car,
						TFlt flow_cap_truck,
						TFlt ffs_car,
						TFlt ffs_truck,
						TFlt wave_speed_car,
						TFlt wave_speed_truck,
						TFlt flow_scalar);
	~Ctm_Cell_Multiclass();
	TFlt get_perceived_demand(TInt veh_type, TFlt _ratio_lane_closure);
	TFlt get_perceived_supply(TInt veh_type, TFlt _ratio_lane_closure);
	int update_perceived_density(TFlt _ratio_lane_closure);

	TFlt get_perceived_demand_lane_closure(TInt veh_type, TFlt _ratio_lane_closure);
	TFlt get_perceived_supply_lane_closure(TInt veh_type, TFlt _ratio_lane_closure);
	int update_perceived_density_lane_closure(TFlt _ratio_lane_closure);

	TInt m_cell_ID;
	TFlt m_cell_length;
	TFlt m_unit_time;
	TFlt m_flow_scalar;
	
	TFlt m_hold_cap_car;
	TFlt m_hold_cap_truck;
	TFlt m_critical_density_car;
	TFlt m_critical_density_truck;
	TFlt m_rho_1_N;
	TFlt m_flow_cap_car;
	TFlt m_flow_cap_truck;
	TFlt m_ffs_car;
	TFlt m_ffs_truck;
	TFlt m_wave_speed_car;
	TFlt m_wave_speed_truck;

	TInt m_volume_car;
	TInt m_volume_truck;
	TFlt m_space_fraction_car;
	TFlt m_space_fraction_truck;
	TFlt m_perceived_density_car;
	TFlt m_perceived_density_truck;
	TInt m_out_veh_car;
	TInt m_out_veh_truck;
	std::deque<MNM_Veh*> m_veh_queue_car;
	std::deque<MNM_Veh*> m_veh_queue_truck;

	// PMC project - cell level record congestion regime and differentiability
	bool m_cell_diff_car;
	bool m_cell_diff_truck;

	int m_cell_congested_car;
	int m_cell_congested_truck;
	// PMC end

};


/**************************************************************************
							Multiclass Link-Queue Model
**************************************************************************/
class MNM_Dlink_Lq_Multiclass : public MNM_Dlink_Multiclass
{
public:
	MNM_Dlink_Lq_Multiclass(TInt ID,
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
							TFlt flow_scalar);
	virtual ~MNM_Dlink_Lq_Multiclass() override;
    virtual int evolve(TInt timestamp, TFlt _ratio_lane_closure) override;
	virtual int evolve_control(TInt timestamp, TFlt _ratio_lane_closure, TFlt cell_position, TFlt cell_control_rate) override;

	// Jiachao added for curb
	virtual int evolve_curb(TInt timestamp) override;
    virtual TFlt get_link_supply() override;
	virtual TFlt get_link_capacity() override;
    virtual int clear_incoming_array(TInt timestamp, TFlt _ratio_lane_closure) override;
    virtual void print_info() override;

    virtual TFlt get_link_flow_car() override;
    virtual TFlt get_link_flow_truck() override;
    virtual TFlt get_link_flow() override;
    virtual TFlt get_link_tt() override;
    virtual TFlt get_link_tt_from_flow_car(TFlt flow) override;
    virtual TFlt get_link_tt_from_flow_truck(TFlt flow) override;
	virtual TInt get_link_freeflow_tt_loading_car() override;  // intervals
	virtual TInt get_link_freeflow_tt_loading_truck() override;  // intervals

	int update_perceived_density();

	std::deque<MNM_Veh*> m_veh_queue_car;
	std::deque<MNM_Veh*> m_veh_queue_truck;
	std::deque<MNM_Veh*> m_veh_out_buffer_car;
	std::deque<MNM_Veh*> m_veh_out_buffer_truck;

	std::deque<MNM_Veh*> m_veh_parking_car;
	std::deque<MNM_Veh*> m_veh_parking_truck;
	std::deque<MNM_Veh*> m_veh_parking_rh;

	TInt m_volume_car; //vehicle number, without the flow scalar
	TInt m_volume_truck; //vehicle number, without the flow scalar
	TFlt m_flow_scalar;
	TFlt m_k_j_car; //jam density
	TFlt m_k_j_truck; //jam density
	TFlt m_C_car; //maximum free flow capacity
	TFlt m_C_truck; //maximum free flow capacity
	TFlt m_k_C_car; //maximum free flow density
	TFlt m_k_C_truck; //maximum free flow density
	TFlt m_w_car; //backward wave speed
	TFlt m_w_truck; //backward wave speed
	TFlt m_rho_1_N;

	TFlt m_space_fraction_car;
	TFlt m_space_fraction_truck;
	TFlt m_perceived_density_car;
	TFlt m_perceived_density_truck;

	TFlt m_unit_time;
	TFlt m_veh_convert_factor;

};


/**************************************************************************
							Multiclass Point-Queue Model
**************************************************************************/
class MNM_Dlink_Pq_Multiclass : public MNM_Dlink_Multiclass
{
public:
	MNM_Dlink_Pq_Multiclass(TInt ID,
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
							TFlt flow_scalar);
	virtual ~MNM_Dlink_Pq_Multiclass() override;
    virtual int evolve(TInt timestamp, TFlt _ratio_lane_closure) override;
	virtual int evolve_control(TInt timestamp, TFlt _ratio_lane_closure, TFlt cell_position, TFlt cell_control_rate) override;

	// Jiachao added for curb
	virtual int evolve_curb(TInt timestamp) override;
    virtual TFlt get_link_supply() override;
	virtual TFlt get_link_capacity() override;
    virtual int clear_incoming_array(TInt timestamp, TFlt _ratio_lane_closure) override;
    virtual void print_info() override;

    virtual TFlt get_link_flow_car() override;
    virtual TFlt get_link_flow_truck() override;
    virtual TFlt get_link_flow() override;
    virtual TFlt get_link_tt() override;
    virtual TFlt get_link_tt_from_flow_car(TFlt flow) override;
    virtual TFlt get_link_tt_from_flow_truck(TFlt flow) override;

	virtual TInt get_link_freeflow_tt_loading_car() override;  // intervals
	virtual TInt get_link_freeflow_tt_loading_truck() override;  // intervals

	std::unordered_map<MNM_Veh*, TInt> m_veh_pool;
	TInt m_volume_car; //vehicle number, without the flow scalar
	TInt m_volume_truck; //vehicle number, without the flow scalar
	TFlt m_lane_hold_cap;
	TFlt m_lane_flow_cap;
	TFlt m_flow_scalar;
	TFlt m_hold_cap;
	TInt m_max_stamp;
	TFlt m_unit_time;
	TFlt m_veh_convert_factor;

};


/******************************************************************************************************************
*******************************************************************************************************************
												Node Models
*******************************************************************************************************************
******************************************************************************************************************/

/**************************************************************************
                              Origin node
**************************************************************************/
class MNM_DMOND_Multiclass : public MNM_DMOND
{
public:
	MNM_DMOND_Multiclass(TInt ID, TFlt flow_scalar, TFlt veh_convert_factor);
	virtual ~MNM_DMOND_Multiclass() override;
    virtual int evolve(TInt timestamp) override;
	virtual int evolve_control(TInt timestamp, std::unordered_map<std::string, float> *control_map, std::unordered_map<std::string, std::vector<float>> *control_map_list, std::unordered_map<TInt, std::vector<TInt>> *lane_closure_map) override;
	TFlt m_veh_convert_factor;
	// MNM_Control_Factory_Multiclass *m_control_factory;
	virtual int evolve_curb(TInt timestamp) override;
};

/**************************************************************************
                              Destination node
**************************************************************************/
class MNM_DMDND_Multiclass : public MNM_DMDND
{
public:
	MNM_DMDND_Multiclass(TInt ID, TFlt flow_scalar, TFlt veh_convert_factor);
	virtual ~MNM_DMDND_Multiclass() override;
    virtual int evolve(TInt timestamp) override;
	virtual int evolve_control(TInt timestamp, std::unordered_map<std::string, float> *control_map, std::unordered_map<std::string, std::vector<float>> *control_map_list, std::unordered_map<TInt, std::vector<TInt>> *lane_closure_map) override;
	virtual int evolve_curb(TInt timestamp) override;

	TFlt m_veh_convert_factor;
	// MNM_Control_Factory_Multiclass *m_control_factory;
};

/**************************************************************************
                              In-Out node
**************************************************************************/
class MNM_Dnode_Inout_Multiclass : public MNM_Dnode
{
public:
	MNM_Dnode_Inout_Multiclass(TInt ID, TFlt flow_scalar, TFlt veh_convert_factor);
	virtual ~MNM_Dnode_Inout_Multiclass() override;
    virtual int evolve(TInt timestamp) override;
	virtual int evolve_control(TInt timestamp, std::unordered_map<std::string, float> *control_map, std::unordered_map<std::string, std::vector<float>> *control_map_list, std::unordered_map<TInt, std::vector<TInt>> *lane_closure_map) override;
    
	virtual int evolve_curb(TInt timestamp) override;
	
	virtual int prepare_loading() override;
    virtual int add_out_link(MNM_Dlink* out_link) override;
    virtual int add_in_link(MNM_Dlink* in_link) override;

protected:
	int prepare_supplyANDdemand();
	int prepare_supplyANDdemand_control(TInt timestamp, std::unordered_map<std::string, float> *control_map, std::unordered_map<std::string, std::vector<float>> *control_map_list, std::unordered_map<TInt, std::vector<TInt>> *lane_closure_map);

    virtual int compute_flow(){return 0;};
	// int flow_to_vehicle();
	int move_vehicle(TInt timestamp);

	int record_cumulative_curve(TInt timestamp);
	
	int move_vehicle_curb(TInt timestamp);

	int record_cumulative_curve_curb(TInt timestamp);	

	TFlt *m_demand; //2d
	TFlt *m_supply; //1d
	TFlt *m_veh_flow; //2d
	TFlt *m_veh_moved_car; //2d
	TFlt *m_veh_moved_truck; //2d
	TFlt m_veh_convert_factor;
	TFlt *m_veh_moved_car_cc; //2d
	TFlt *m_veh_moved_truck_cc; //2d

	TFlt *m_veh_moved_car_ilink;
	TFlt *m_veh_moved_car_ilink_cc;
	TFlt *m_veh_moved_truck_ilink;
	TFlt *m_veh_moved_truck_ilink_cc;
	// 
};

/*                           FWJ node
**************************************************************************/
class MNM_Dnode_FWJ_Multiclass : public MNM_Dnode_Inout_Multiclass
{
public:
	MNM_Dnode_FWJ_Multiclass(TInt ID, TFlt flow_scalar, TFlt veh_convert_factor);
	virtual ~MNM_Dnode_FWJ_Multiclass() override;
    virtual int compute_flow() override;
	// MNM_Control_Factory_Multiclass* m_link_control_factory = control_factory;
	// , ;
};

/*                  General Road Junction node
**************************************************************************/
class MNM_Dnode_GRJ_Multiclass : public MNM_Dnode_Inout_Multiclass
{
public:
	MNM_Dnode_GRJ_Multiclass(TInt ID, TFlt flow_scalar, TFlt veh_convert_factor);
    virtual ~MNM_Dnode_GRJ_Multiclass() override;
    virtual int compute_flow() override;
    virtual int prepare_loading() override;
private:
	std::vector<std::vector<MNM_Dlink*>> m_pow;
	TFlt get_theta();
	int prepare_outflux();
	TFlt *m_d_a; //1d array
	TFlt *m_C_a; //1d array
	template<typename T> std::vector<std::vector<T> > powerSet(const std::vector<T>& set);
	std::vector<int> getOnLocations(int a);
};


/******************************************************************************************************************
*******************************************************************************************************************
												Multiclass OD
*******************************************************************************************************************
******************************************************************************************************************/
class MNM_Destination_Multiclass : public MNM_Destination
{
public:
	explicit MNM_Destination_Multiclass(TInt ID);
	virtual ~MNM_Destination_Multiclass() override;
};

class MNM_Origin_Multiclass : public MNM_Origin
{
public:
	MNM_Origin_Multiclass(TInt ID, TInt max_interval, TFlt flow_scalar, TInt frequency);
	virtual ~MNM_Origin_Multiclass() override;
    virtual int release(MNM_Veh_Factory* veh_factory, TInt current_interval) override;
    virtual int release_one_interval(TInt current_interval,
									MNM_Veh_Factory* veh_factory, 
									TInt assign_interval, 
									TFlt adaptive_ratio) override;

    virtual int release_one_interval_biclass(TInt current_interval,
									MNM_Veh_Factory* veh_factory, 
									TInt assign_interval, 
									TFlt adaptive_ratio_car,
									TFlt adaptive_ratio_truck) override;
	
	// jiachao added in Sep
	// int release_one_interval_curb(TInt current_interval,
	// 							    MNM_Veh_Factory* veh_factory, 
	// 							    TInt assign_interval, 
	// 								TFlt adaptive_ratio_car,
	// 								TFlt adaptive_ratio_truck,
	// 								TFlt adaptive_ratio_rh);

    // int release_one_interval_curb_hybrid(TInt current_interval,
	// 									 MNM_Veh_Factory* veh_factory,
	// 									 MNM_Curb_Factory_Multiclass* curb_factory,
	// 									 TInt assign_interval,
	// 									 TFlt adaptive_ratio_car,
	// 									 TFlt adaptive_ratio_car_curb,// fraction of adaptive cars use hybrid routing for curb use
	// 									 TFlt adaptive_ratio_truck,
	// 									 TFlt adaptive_ratio_truck_curb, // fraction of adaptive trucks use hybrid routing for curb use
	// 									 TFlt adaptive_ratio_rh);

	// use this one instead of add_dest_demand in the base class
	int add_dest_demand_multiclass(MNM_Destination_Multiclass *dest, 
								TFlt* demand_car, 
								TFlt* demand_truck);
	// jiachao added in Sep
	// int add_dest_demand_multiclass_curb(MNM_Destination_Multiclass *dest,
	// 								   TFlt* demand_car,
	// 								   TFlt* demand_tnc,
	// 								   TFlt* demand_truck);
	// two new unordered_map for both classes
	std::unordered_map<MNM_Destination_Multiclass*, TFlt*> m_demand_car;
	std::unordered_map<MNM_Destination_Multiclass*, TFlt*> m_demand_truck;

	// jiachao added in Sep for another class -- TNC vehicles (releasing function needs update)
	std::unordered_map<MNM_Destination_Multiclass*, TFlt*> m_demand_tnc;
};

/******************************************************************************************************************
*******************************************************************************************************************
												Multiclass Vehicle
*******************************************************************************************************************
******************************************************************************************************************/
class MNM_Veh_Multiclass : public MNM_Veh
{
public:
	MNM_Veh_Multiclass(TInt ID, TInt vehicle_class, TInt start_time);
	virtual ~MNM_Veh_Multiclass() override;

    virtual TInt get_class() override {return m_class;};  // virtual getter
    virtual TInt get_bus_route_ID() override {return m_bus_route_ID;};  // virtual getter
    virtual bool get_ispnr() override {return m_pnr;}; // virtual getter

	TInt m_class;
	TFlt m_visual_position_on_link; //[0(start), 1(end)], for vehicle-based visualization
	TFlt m_parking_location;

	std::deque<TInt> m_curb_destination_list; // record the curb destination link for parking
	std::deque<TInt> m_destination_list; // predefined destination settings, initialize from input file and fixed during the whole loading
	std::vector<TInt> m_parking_duration_list; // predefined parking duration settings, but can be changed for real parking duration
	std::vector<TInt> m_arrival_time_list; // record the arrival time
	std::vector<TInt> m_departure_time_list; // record the departure time
	TInt m_complete_stop_current_link;
};


/******************************************************************************************************************
*******************************************************************************************************************
												Multiclass Factory
*******************************************************************************************************************
******************************************************************************************************************/

// jiachao added in Apr. 05
class MNM_Control_Factory_Multiclass
{
public:
	MNM_Control_Factory_Multiclass();
  	// virtual ~MNM_Control_Factory_Multiclass() override;
  	std::unordered_map<std::string, float> m_control_map;
	std::unordered_map<std::string, std::vector<float>> m_control_map_list;
	std::unordered_map<TInt, std::vector<TInt>> m_lane_closure_map;
	std::unordered_map<TInt, std::vector<float>> m_ramp_metering_map;
	std::unordered_map<TInt, std::vector<float>> m_rm_control_historical;
	TInt m_fix_signal_plan;
	TInt m_lane_closure_bin;
	TInt m_ramp_metering_bin;
	TInt m_alinea_control;
	TInt m_lsc_control;
};

class MNM_Veh_Factory_Multiclass : public MNM_Veh_Factory
{
public:
	MNM_Veh_Factory_Multiclass();
	virtual ~MNM_Veh_Factory_Multiclass() override;

	// use this one instead of make_veh in the base class
	MNM_Veh_Multiclass* make_veh_multiclass(TInt timestamp, Vehicle_type veh_type, TInt vehicle_cls);
	// MNM_Veh_Multiclass* make_veh_multiclass_curb(TInt timestamp, 
	// 											Vehicle_type veh_type, 
	// 											TInt vehicle_cls, 
	// 											MNM_Curb_Factory_Multiclass* curb_factory,
	// 											TInt _o_node_ID,
	// 											TInt _d_node_ID);
};

class MNM_Node_Factory_Multiclass : public MNM_Node_Factory
{
public:
	MNM_Node_Factory_Multiclass();
	virtual ~MNM_Node_Factory_Multiclass() override;

	// use this one instead of make_node in the base class
	MNM_Dnode *make_node_multiclass(TInt ID, DNode_type_multiclass node_type, TFlt flow_scalar, TFlt veh_convert_factor);
};

class MNM_Link_Factory_Multiclass : public MNM_Link_Factory
{
public:
	MNM_Link_Factory_Multiclass();
	virtual ~MNM_Link_Factory_Multiclass() override;

	// use this one instead of make_link in the base class
	MNM_Dlink *make_link_multiclass(TInt ID,
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
									TFlt flow_scalar);

	MNM_Dlink *make_link_multiclass_curb(TInt ID,
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
									TInt curb_dest);
};

class MNM_OD_Factory_Multiclass : public MNM_OD_Factory
{
public:
	MNM_OD_Factory_Multiclass();
	virtual ~MNM_OD_Factory_Multiclass() override;
	virtual MNM_Destination_Multiclass* make_destination(TInt ID) override;
	virtual MNM_Origin_Multiclass* make_origin(TInt ID, 
                                               TInt max_interval,
                                               TFlt flow_scalar,
											   TInt frequency) override;
	// std::vector<TInt> *m_intermediate_dest_list;
};


/******************************************************************************************************************
*******************************************************************************************************************
												Multiclass IO Functions
*******************************************************************************************************************
******************************************************************************************************************/
class MNM_IO_Multiclass : public MNM_IO
{
public:
	static int build_node_factory_multiclass(const std::string& file_folder,
											 MNM_ConfReader *conf_reader,
											 MNM_Node_Factory *node_factory,
                                             const std::string& file_name = "MNM_input_node");
 	static int build_link_factory_multiclass(const std::string& file_folder,
 											 MNM_ConfReader *conf_reader,
 											 MNM_Link_Factory *link_factory,
 											 const std::string& file_name = "MNM_input_link");
 	// static int build_od_factory_multiclass(std::string file_folder, 
 	// 										MNM_ConfReader *conf_reader, 
 	// 										MNM_OD_Factory *od_factory, 
 	// 										MNM_Node_Factory *node_factory) {
 	// 	return build_od_factory(file_folder, conf_reader, od_factory, node_factory);
 	// };
 	static int build_demand_multiclass(const std::string& file_folder,
 									   MNM_ConfReader *conf_reader,
 									   MNM_OD_Factory *od_factory,
                                       const std::string& file_name = "MNM_input_demand");
	
	// // jiachao added in Sep for building demand inout for ride-hailing, driving and truck (3 modes)
	// static int build_demand_multiclass_curb(const std::string& file_folder,
	// 									    MNM_ConfReader *conf_reader,
	// 									    MNM_OD_Factory *od_factory,
	// 									    const std::string& file_name = "MNM_input_demand");
	
	// jiachao added in Apr 05
	static int build_control_factory_multiclass(const std::string& file_folder,
												MNM_ConfReader *conf_reader,
 											 	MNM_Control_Factory_Multiclass *control_factory,
 											 	const std::string& file_name = "MNM_control");
	
	// // jiachao added in Sep 08
	// static int build_curb_factory_multiclass(const std::string& file_folder,
 	// 								 			MNM_ConfReader *conf_reader,
 	// 								 			MNM_Curb_Factory_Multiclass *curb_factory);
	
	// static int load_inter_dest_curb(const std::string& file_folder,
 	// 								MNM_ConfReader *conf_reader,
	// 								MNM_Curb_Factory_Multiclass *curb_factory,
 	// 								const std::string& file_name = "path_inter_dest_curb");
	
	// static int load_inter_dest_curb_separate(const std::string& file_folder,
 	// 								MNM_ConfReader *conf_reader,
	// 								MNM_Curb_Factory_Multiclass *curb_factory);

	// each path has a m_path_ID
	// this function returns a std::unordered_map<TInt (path_ID), std::vector<TInt>>
	// std::vector<TInt> is the intermediate destination ID
	/*******
	the input file is 
	# inter_dest_num inter_destination curb_usage
	1 3 2
	1 3 4
	1 4 3
	1 4 5
	2 3 4 2 5

	the first is <0, [3,2]>
	the second is <1, [3,4]>
	the last is <4, [3,4,2,5]>
	********/

};

/******************************************************************************************************************
*******************************************************************************************************************
												Multiclass DTA
*******************************************************************************************************************
******************************************************************************************************************/
class MNM_Dta_Multiclass : public MNM_Dta
{
public:
	explicit MNM_Dta_Multiclass(const std::string& file_folder);
	virtual ~MNM_Dta_Multiclass() override;
    virtual int initialize() override;
    // virtual int build_from_files() override;
	virtual int build_from_files_control();
	virtual int build_from_files_no_control();
	// virtual int build_from_file_no_routing();

	// jiachao 0604
	// virtual int build_from_files_separate();

    virtual int pre_loading() override;
	// jiachao added in Apr. 06
	MNM_Control_Factory_Multiclass *m_control_factory;
	// jiachao added in Sep.
	// MNM_Curb_Factory_Multiclass *m_curb_factory;

	// MNM_Statistics_Curb *m_statistics_curb;

	virtual int load_once_control(bool verbose, TInt load_int, TInt assign_int);

	virtual int loading_control(bool verbose);
	// virtual int loading_curb(bool verbose);

	virtual int build_control_file(std::string folder);
	// virtual int set_routing_curb();

	// jiachao 0604
	// virtual int set_routing_curb_separate();
	// end

	// virtual int set_statistics_curb();
	
	virtual Path_Table *load_path_table_curb(const std::string& file_name, const PNEGraph& graph, TInt num_path,
                                     bool w_buffer = false, TInt m_class = TInt(0), bool w_ID = false);

	virtual Path_Table *load_path_table_biclass(const std::string& file_name, const PNEGraph& graph, TInt num_path,
                                     bool w_buffer = false, TInt m_class = TInt(0), bool w_ID = false);
	// jiachao added in Sep 08
	// virtual int build_curbs();
	
}; 




/******************************************************************************************************************
*******************************************************************************************************************
										Multiclass DTA Gradient Utils
*******************************************************************************************************************
******************************************************************************************************************/
namespace MNM_DTA_GRADIENT
{
// curb project
// link-level emission metrics
TFlt get_link_fuel_rate_biclass(MNM_Dlink_Multiclass* link,TFlt m_unit_time);
TFlt get_link_CO2_rate_biclass(MNM_Dlink_Multiclass* link, TFlt m_unit_time);
TFlt get_link_HC_rate_biclass(MNM_Dlink_Multiclass* link, TFlt m_unit_time);
TFlt get_link_CO_rate_biclass(MNM_Dlink_Multiclass* link, TFlt m_unit_time);
TFlt get_link_NOX_rate_biclass(MNM_Dlink_Multiclass* link, TFlt m_unit_time);
TFlt get_link_VMT_biclass(MNM_Dlink_Multiclass* link, TFlt m_unit_time);
TFlt get_link_VHT_biclass(MNM_Dlink_Multiclass* link, TFlt m_unit_time);

TInt get_link_parking_num(MNM_Dlink_Ctm_Multiclass* link);
TInt get_link_dp_num(MNM_Dlink_Ctm_Multiclass* link);

// jiachao added
TInt get_link_parking_num_car(MNM_Dlink_Ctm_Multiclass* link);
TInt get_link_parking_num_truck(MNM_Dlink_Ctm_Multiclass* link);
TInt get_link_parking_num_rh(MNM_Dlink_Ctm_Multiclass* link);

TInt get_link_dp_num_car(MNM_Dlink_Ctm_Multiclass* link);
TInt get_link_dp_num_truck(MNM_Dlink_Ctm_Multiclass* link);
TInt get_link_dp_num_rh(MNM_Dlink_Ctm_Multiclass* link);

// jiachao added in Apr. 06
bool check_movement_exist(MNM_Dta_Multiclass* test_dta,
						std::string check_movement);
// jiachao added in Apr. 08
TFlt get_control_rate_one(MNM_Dta_Multiclass* test_dta, 
						std::string movement_id);

std::pair<std::vector<std::string>, std::vector<float>> get_control_rate_all(MNM_Dta_Multiclass* test_dta);

int change_control_rate_one(MNM_Dta_Multiclass* test_dta,
						std::string movement_id,
						TFlt control_rate);

int change_control_rate_all(MNM_Dta_Multiclass* test_dta,
						std::unordered_map<std::string, TFlt> change_movement_map);

TFlt get_link_inflow_car(MNM_Dlink_Multiclass* link, 
                    	TFlt start_time, TFlt end_time);
TFlt get_link_inflow_car(MNM_Dlink_Multiclass* link, 
                    	TInt start_time, TInt end_time);
TFlt get_link_inflow_truck(MNM_Dlink_Multiclass* link, 
                    	TFlt start_time, TFlt end_time);
TFlt get_link_inflow_truck(MNM_Dlink_Multiclass* link, 
                    	TInt start_time, TInt end_time);
TFlt get_link_inflow_rh(MNM_Dlink_Multiclass* link, 
                    	TFlt start_time, TFlt end_time);

TFlt get_link_outflow_car(MNM_Dlink_Multiclass* link, 
                    	TFlt start_time, TFlt end_time);
TFlt get_link_outflow_truck(MNM_Dlink_Multiclass* link, 
                    	TFlt start_time, TFlt end_time);

// new added Jiachao
TFlt get_curb_inflow_car(MNM_Dlink_Multiclass* link, 
                    	TFlt start_time, TFlt end_time);

TFlt get_curb_outflow_car(MNM_Dlink_Multiclass* link, 
                    	TFlt start_time, TFlt end_time);

TFlt get_curb_inflow_truck(MNM_Dlink_Multiclass* link, 
                    	TFlt start_time, TFlt end_time);

TFlt get_curb_outflow_truck(MNM_Dlink_Multiclass* link, 
                    	TFlt start_time, TFlt end_time);

TFlt get_curb_inflow_rh(MNM_Dlink_Multiclass* link, 
                    	TFlt start_time, TFlt end_time);

TFlt get_curb_outflow_rh(MNM_Dlink_Multiclass* link, 
                    	TFlt start_time, TFlt end_time);
// new added end

TFlt get_average_waiting_time_at_intersection(MNM_Dlink_Multiclass* link);

TInt get_is_spillback(MNM_Dlink_Multiclass* link); // 0 - no spillback, 1 - spillback

TFlt get_travel_time_from_FD_car(MNM_Dlink_Multiclass *link, TFlt start_time, TFlt unit_interval);
TFlt get_travel_time_from_FD_truck(MNM_Dlink_Multiclass *link, TFlt start_time, TFlt unit_interval);

TFlt get_travel_time_car(MNM_Dlink_Multiclass* link, TFlt start_time, TFlt unit_interval, TInt end_loading_timestamp);
TFlt get_travel_time_car_robust(MNM_Dlink_Multiclass* link, TFlt start_time, TFlt end_time, TFlt unit_interval, TInt end_loading_timestamp, TInt num_trials = TInt(10));
TFlt get_travel_time_truck(MNM_Dlink_Multiclass* link, TFlt start_time, TFlt unit_interval, TInt end_loading_timestamp);
TFlt get_travel_time_truck_robust(MNM_Dlink_Multiclass* link, TFlt start_time, TFlt end_time, TFlt unit_interval, TInt end_loading_timestamp, TInt num_trials = TInt(10));

int add_dar_records_car(std::vector<dar_record*> &record, MNM_Dlink_Multiclass* link, 
                    std::set<MNM_Path*> pathset, TFlt start_time, TFlt end_time);
int add_dar_records_truck(std::vector<dar_record*> &record, MNM_Dlink_Multiclass* link, 
                    std::set<MNM_Path*> pathset, TFlt start_time, TFlt end_time);
int add_dar_records_rh(std::vector<dar_record*> &record, MNM_Dlink_Multiclass* link, 
                    std::set<MNM_Path*> pathset, TFlt start_time, TFlt end_time);

// Fujitsu
int add_dar_records_car_out(std::vector<dar_record*> &record, MNM_Dlink_Multiclass* link, 
                    std::set<MNM_Path*> pathset, TFlt start_time, TFlt end_time);
int add_dar_records_truck_out(std::vector<dar_record*> &record, MNM_Dlink_Multiclass* link, 
                    std::set<MNM_Path*> pathset, TFlt start_time, TFlt end_time);
int add_dar_records_rh_out(std::vector<dar_record*> &record, MNM_Dlink_Multiclass* link,
					std::set<MNM_Path*> pathset, TFlt start_time, TFlt end_time);

int add_dar_records_car(std::vector<dar_record*> &record, MNM_Dlink_Multiclass* link, 
                    std::set<TInt> pathID_set, TFlt start_time, TFlt end_time);
int add_dar_records_truck(std::vector<dar_record*> &record, MNM_Dlink_Multiclass* link, 
                    std::set<TInt> pathID_set, TFlt start_time, TFlt end_time);
int add_dar_records_rh(std::vector<dar_record*> &record, MNM_Dlink_Multiclass* link, 
                    std::set<TInt> pathID_set, TFlt start_time, TFlt end_time);

// new added
int add_dar_records_curb_arrival_truck(std::vector<dar_record*> &record, MNM_Dlink_Multiclass* link, std::set<MNM_Path*> pathset, TFlt start_time, TFlt end_time);
int add_dar_records_curb_departure_truck(std::vector<dar_record*> &record, MNM_Dlink_Multiclass* link, std::set<MNM_Path*> pathset, TFlt start_time, TFlt end_time);

int add_dar_records_curb_arrival_rh(std::vector<dar_record*> &record, MNM_Dlink_Multiclass* link, std::set<MNM_Path*> pathset, TFlt start_time, TFlt end_time);
int add_dar_records_curb_departure_rh(std::vector<dar_record*> &record, MNM_Dlink_Multiclass* link, std::set<MNM_Path*> pathset, TFlt start_time, TFlt end_time);

int add_dar_records_curb_arrival_car(std::vector<dar_record*> &record, MNM_Dlink_Multiclass* link, std::set<MNM_Path*> pathset, TFlt start_time, TFlt end_time);
int add_dar_records_curb_departure_car(std::vector<dar_record*> &record, MNM_Dlink_Multiclass* link, std::set<MNM_Path*> pathset, TFlt start_time, TFlt end_time);
// end new added

TFlt get_departure_cc_slope_car(MNM_Dlink_Multiclass* link, TFlt start_time, TFlt end_time);
TFlt get_departure_cc_slope_truck(MNM_Dlink_Multiclass* link, TFlt start_time, TFlt end_time);

int add_ltg_records_veh(std::vector<ltg_record*> &record, MNM_Dlink_Multiclass *link,
						MNM_Path* path, int depart_time, int start_time, TFlt gradient);

// Fujitsu paper - density
TFlt get_link_density_from_cc(TFlt time, MNM_Cumulative_Curve *N_in, MNM_Cumulative_Curve *N_out, TFlt last_valid_time);

TFlt get_link_density_car(MNM_Dlink_Multiclass* link, TFlt time, TInt end_loading_timestamp);
TFlt get_link_density_truck(MNM_Dlink_Multiclass* link, TFlt time, TInt end_loading_timestamp);

TFlt get_link_density_stop_car(MNM_Dlink_Multiclass* link, TFlt time, TInt end_loading_timestamp);
TFlt get_link_density_stop_truck(MNM_Dlink_Multiclass* link, TFlt time, TInt end_loading_timestamp);

TFlt get_link_density_car_robust(MNM_Dlink_Multiclass* link, TFlt time, TInt end_loading_timestamp, TInt num_trials);
TFlt get_link_density_truck_robust(MNM_Dlink_Multiclass* link, TFlt time, TInt end_loading_timestamp, TInt num_trials);
TFlt get_link_density_rh_robust(MNM_Dlink_Multiclass* link, TFlt time, TInt end_loading_timestamp, TInt num_trials);

TFlt get_link_density_stop_rh_robust(MNM_Dlink_Multiclass* link, TFlt time, TInt end_loading_timestamp, TInt num_trials);
TFlt get_link_density_stop_truck_robust(MNM_Dlink_Multiclass* link, TFlt time, TInt end_loading_timestamp, TInt num_trials);

};




/******************************************************************************************************************
*******************************************************************************************************************
											Multiclass Emissions
*******************************************************************************************************************
******************************************************************************************************************/
class MNM_Cumulative_Emission_Multiclass : public MNM_Cumulative_Emission
{
public:
  MNM_Cumulative_Emission_Multiclass(TFlt unit_time, TInt freq);
  virtual ~MNM_Cumulative_Emission_Multiclass() override;

  // new functions for trucks
  TFlt calculate_fuel_rate_truck(TFlt v);
  TFlt calculate_CO2_rate_truck(TFlt v);
  TFlt calculate_HC_rate_truck(TFlt v);
  TFlt calculate_CO_rate_truck(TFlt v);
  TFlt calculate_NOX_rate_truck(TFlt v);

  virtual int update(MNM_Veh_Factory* veh_factory) override;
  virtual int output() override;

  virtual std::string output_save() override;

  TFlt m_fuel_truck;
  TFlt m_CO2_truck;
  TFlt m_HC_truck;
  TFlt m_CO_truck;
  TFlt m_NOX_truck;
  TFlt m_VMT_truck;

  TFlt m_VHT_truck;
  TFlt m_VHT_car;

  std::unordered_set<MNM_Veh*> m_car_set;
  std::unordered_set<MNM_Veh*> m_truck_set;
};

// /******************************************************************************************************************
// *******************************************************************************************************************
// 										Multiclass DUE with Curb
// *******************************************************************************************************************
// ******************************************************************************************************************/
// class MNM_Due_Curb {
// public:
//     MNM_Due_Curb(std::string file_folder);

//     virtual ~MNM_Due_Curb();

//     virtual int initialize();

// 	virtual int run_due_curb(bool verbose);

//     MNM_Dta_Multiclass *run_dta_curb(bool verbose);

//     virtual int update_path_table(MNM_Dta_Multiclass *mc_dta, int iter);

//     virtual int update_path_table_gp_fixed_departure_time_choice_fixed_pathset(MNM_Dta_Multiclass *mc_dta);

// 	virtual int update_path_table_gp_fixed_departure_time_choice_column_generation(MNM_Dta_Multiclass *mc_dta, int iter);

//     TFlt compute_merit_function(MNM_Dta_Multiclass *mc_dta);

//     TFlt compute_merit_function_fixed_departure_time_choice(MNM_Dta_Multiclass *mc_dta);

// 	int update_demand_from_path_table(MNM_Dta_Multiclass *mc_dta);

// 	int update_path_table_in_dta(MNM_Dta_Multiclass *mc_dta);

// 	int build_cost_map(MNM_Dta_Multiclass *mc_dta);

// 	int set_routing_from_path_table(MNM_Dta_Multiclass *mc_dta);

//     TFlt get_tt_car(TFlt depart_time, MNM_Path *path); // DONE

// 	TFlt get_tt_truck(TFlt depart_time, MNM_Path *path); // DONE

// 	TFlt get_tt_rh(TFlt depart_time, MNM_Path *path);

// 	TFlt get_disutility_car(TInt depart_int, TFlt tt, MNM_Path *_tmp_path);

// 	TFlt get_disutility_truck(TInt depart_int, TFlt tt, MNM_Path *_tmp_path);

// 	TFlt get_disutility_rh(TInt depart_int, TFlt tt, MNM_Path *_tmp_path);

// 	virtual int load_fixed_pathset(MNM_Dta_Multiclass *mc_dta);

//     std::string m_file_folder;
//     MNM_ConfReader *m_dta_config;
//     MNM_ConfReader *m_due_config;

//     TFlt m_unit_time;
//     TInt m_total_assign_inter; // different from m_mmdta -> m_total_assign_inter, which is actually a releasing interval
//     TInt m_total_loading_inter;
// 	TInt m_assign_freq_due;
	
// 	/* parameters in generalized costs */
// 	TFlt m_unit_time_cost_car;
// 	TFlt m_unit_time_cost_truck;
// 	TFlt m_unit_dist_cost_truck;
// 	TFlt m_unit_time_cost_rh;
// 	TFlt m_unit_dist_cost_rh;
// 	TFlt m_fixed_fee_rh;
// 	TFlt m_general_cost_car;
// 	TFlt m_parking_fee_car;
// 	TFlt m_step_size;
// 	TFlt m_max_iter;

// 	Path_Table *m_path_table_car;
// 	Path_Table *m_path_table_truck;
// 	Path_Table *m_path_table_rh;

// 	std::unordered_map<TInt, TFlt *> m_car_cost_map;

// 	std::unordered_map<TInt, TFlt *> m_truck_cost_map;

// 	std::unordered_map<TInt, std::vector<TInt>> m_path_inter_dest_map;
// 	std::unordered_map<TInt, std::vector<TInt>> m_path_inter_dest_map_rh;
// 	std::unordered_map<TInt, std::vector<TInt>> m_path_inter_dest_map_car;

// 	std::vector<TFlt> m_demand_car;
// 	std::vector<TFlt> m_demand_truck;
// 	std::vector<TFlt> m_demand_rh;

// 	MNM_Dta_Multiclass *m_base_dta;

// 	std::pair<MNM_Path *, TInt> get_best_car_route_for_one_interval(TInt interval, TInt o_node_ID, MNM_TDSP_Tree *tdsp_tree);

// 	std::pair<MNM_Path *, TInt> get_best_truck_route_for_one_interval(TInt interval, TInt o_node_ID, MNM_TDSP_Tree *tdsp_tree);

// 	std::pair<MNM_Path *, TInt> get_best_rh_route_for_one_interval(TInt interval, TInt o_node_ID, MNM_TDSP_Tree *tdsp_tree);
// };

/******************************************************************************************************************
*******************************************************************************************************************
												Multiclass DSO
*******************************************************************************************************************
******************************************************************************************************************/
class MNM_Dso_Multiclass
{
public:
	MNM_Dso_Multiclass(std::string file_folder);

	virtual ~MNM_Dso_Multiclass();

	virtual int initialize(bool verbose);

	virtual int run_dso(bool verbose, std::string folder, int v_start, TFlt weight_lower, TFlt weight_upper, TInt interactive_indicator);

	virtual int run_dso_departure_time_choice_msa(bool verbose, std::string folder, int v_start, TFlt weight_lower, TFlt weight_upper, 
													TInt interactive_indicator, std::string save_name, std::string pmc_version);
    // virtual int dnl(bool verbose);

	MNM_Dta_Multiclass *dnl(bool verbose);

	virtual int build_link_cost_map(MNM_Dta_Multiclass *test_dta, bool verbose);

	int get_link_marginal_cost_v1(MNM_Dta_Multiclass *test_dta, bool verbose);

	int get_link_marginal_cost_v2(MNM_Dta_Multiclass *test_dta, bool verbose);

	int get_link_marginal_cost_tt(MNM_Dta_Multiclass *test_dta, bool verbose);

	TFlt get_arrival_cc_slope(MNM_Dlink_Multiclass* link, TInt veh_class, TFlt start_time, TFlt end_time);

	TFlt get_departure_cc_slope(MNM_Dlink_Multiclass* link, TInt veh_class, TFlt start_time, TFlt end_time);

	TFlt dso_get_pmc_car(TInt depart_time, MNM_Path *path, TFlt weight_lower, TFlt weight_upper);

	TFlt dso_get_pmc_truck(TInt depart_time, MNM_Path *path, TFlt weight_lower, TFlt weight_upper);

	// jiachao added for interactive effect of biclass traffic: TODO
	TFlt dso_get_pmc_car_interactive(TInt depart_time, MNM_Path *path, TFlt weight_lower, TFlt weight_upper);

	TFlt dso_get_pmc_truck_interactive(TInt depart_time, MNM_Path *path, TFlt weight_lower, TFlt weight_upper);

	TFlt dso_get_path_travel_time_car(TInt depart_time, MNM_Path *path);

	TFlt dso_get_path_travel_time_truck(TInt depart_time, MNM_Path *path);

	TFlt dso_get_scheduled_delay_cost(TFlt depart_time, TFlt path_travel_time, TFlt alpha, TFlt beta, TFlt gamma, TFlt delta, TInt t_star);

	virtual int dso_update_path_table(MNM_Dta_Multiclass *test_dta, TInt v, TFlt weight_lower, TFlt weight_upper, TInt interactive_indicator);

	virtual int dso_update_path_table_departure_time_choice_msa(MNM_Dta_Multiclass *test_dta,  TInt v, TFlt weight_lower, TFlt weight_upper, TInt interactive_indicator);

	virtual int dso_load_pathset(bool verbose);

	virtual int dso_print_pathbuffer_with_pmc(MNM_Dta_Multiclass *test_dta, TFlt weight_lower, TFlt weight_upper);

	TFlt get_total_travel_cost(MNM_Dta_Multiclass *test_dta);

	TFlt get_car_total_travel_cost(MNM_Dta_Multiclass *test_dta);

	TFlt get_truck_total_travel_cost(MNM_Dta_Multiclass *test_dta);

	TFlt get_car_total_travel_cost_with_scheduled_delay(MNM_Dta_Multiclass *test_dta);

	TFlt get_truck_total_travel_cost_with_scheduled_delay(MNM_Dta_Multiclass *test_dta);

	TFlt get_gap_function_value();

	virtual int save_dso_path_buffer(MNM_Dta_Multiclass *test_dta, std::string save_folder);

	std::string m_file_folder;
    MNM_ConfReader *m_dta_config;
    MNM_ConfReader *m_dso_config;

    TFlt m_unit_time;
    TInt m_total_assign_inter; // different from m_mmdta -> m_total_assign_inter, which is actually a releasing interval
    TInt m_total_loading_inter;
	TInt m_assign_freq;

	TFlt m_step_size;
	TInt m_max_iter;

	MNM_Dta_Multiclass *m_base_dta;

	std::unordered_map<TInt, int *> m_link_congested_car; // 1 - free flow ; 2 - semi-congested ; 3 - fully-congested.

	std::unordered_map<TInt, int *> m_link_congested_truck;

	std::unordered_map<TInt, bool *> m_link_congested_car_tt;

	std::unordered_map<TInt, bool *> m_link_congested_truck_tt;

	std::unordered_map<TInt, bool *> m_link_diff_car;

	std::unordered_map<TInt, bool *> m_link_diff_truck;

	// record space fraction for cars
	std::unordered_map<TInt, TFlt *> m_link_space_fraction_map_car;

	// record space fraction for trucks
	std::unordered_map<TInt, TFlt *> m_link_space_fraction_map_truck;

	// time-varying queue dissipated time
    std::unordered_map<TInt, int *> m_queue_dissipated_time_car;

	std::unordered_map<TInt, int *> m_queue_dissipated_time_truck;

	std::unordered_map<TInt, TFlt *> m_link_tt_map_car;

	std::unordered_map<TInt, TFlt *> m_link_tt_map_truck;

	std::unordered_map<TInt, TFlt *> m_lmc_car_upper;

	std::unordered_map<TInt, TFlt *> m_lmc_car_lower;

	std::unordered_map<TInt, TFlt *> m_lmc_truck_upper;

	std::unordered_map<TInt, TFlt *> m_lmc_truck_lower;

	std::unordered_map<TInt, TFlt *> m_lmc_car_tt;

	std::unordered_map<TInt, TFlt *> m_lmc_truck_tt;

	Path_Table *m_path_table_car;
	Path_Table *m_path_table_truck;

	TFlt m_gap_function_value;
	TFlt m_gap_function_value_car;
	TFlt m_gap_function_value_truck;

	TFlt m_total_sdc_car; // total scheduled delay cost for car
	TFlt m_total_sdc_truck; // total scheduled delay cost for truck

	TFlt m_total_ttc_car; // total travel time cost for car
	TFlt m_total_ttc_truck; // total travel time cost for truck

};

#endif              