#ifndef MULTICLASS_CURB_H
#define MULTICLASS_CURB_H

#include "multiclass.h"
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
									DOE Curb Project Coding Instruction 
	1. Related code are with comment of 'Curb-Jiachao' before the modification for easy finding;
	2. Main change for curb:
		2.1 (DONE) MNM_Dlink_Ctm_Multiclass::evolve_curb -- new added to encapsulate curb dynamics;
			NOTE: only for CTM dlink since the curb space is attached to CTM cell
			Functions:
			(DONE) virtual int evolve_curb(TInt timestamp, std::vector<TFlt> _ratio_lane_closure_list)
		

		2.2 (DONE) Ctm_Cell_Multiclass::update_veh_out_curb -- new added to calculate flux after curb queue intervention
			Functions:
			int update_out_veh_curb(std::vector<TFlt> _ratio_lane_closure_list);
		

		2.3 (Done) class MNM_Veh_Multiclass -- new added to model ride-hailing, driving and commercial truck seperately 
			adding more features related to destination choice

		
		2.4 (Changed) MNM_Dlink_Ctm_Multiclass -- all curb related veh moves and info requests are within this factory
			Functions:
			(DONE) int install_curb() -- initialize curb cells for each CTM dlink;

			(DELETE) int get_curb_parking_info(TInt timestamp) -- output curb-related condition params

			(DELETE) int curb_dynamics(TInt timestamp) -- curb dynamics with road cell interaction
				   output lane_closure_ratio for link dynamics
		
		2.5 (DONE) Cell_Curb_Multiclass -- curb cell for storing parking vehs
			Functions:
			(DONE) move_veh_in_curb() -- move veh into the curb cell;
			(DONE) move_veh_out_curb() -- move veh out of curb cells;

		2.6 (DONE) IO function for curb spaces
			(DONE) build MNM_IO_Multiclass::build_curb_factory_multiclass with curb_install = 0/1
                * load curb, interdestinations and capacity
                * load OD and interdestinations
                * load curb prices
                * load interdest and curb choices for paths
		
		2.7 (DONE) IO build demand functions for three classes: build_demand_multiclass_curb
			    * New defined demand input file for three classes
                * New defined MNM_Origin_Multiclass::release_one_interval_curb/_hybrid

		2.8 (DONE) Hybrid routing
			(DONE) new defined path table for destination choice
			(TODO) hybrid routing for destination choice


*******************************************************************************************************************
******************************************************************************************************************/

class MNM_Dlink_Multiclass_Curb : public MNM_Dlink_Multiclass
{
public:
	MNM_Dlink_Multiclass_Curb(TInt ID,
						TInt number_of_lane,
						TFlt length,
						TFlt ffs_car, 
						TFlt ffs_truck);
	virtual ~MNM_Dlink_Multiclass_Curb() override;	

	int install_cumulative_curve_multiclass();
	int install_cumulative_curve_tree_multiclass();
	int install_cumulative_curve_tree_multiclass_curb(); // + install_cumulative_curve_tree_multiclass

	TFlt get_link_freeflow_tt_car();
	TFlt get_link_freeflow_tt_truck();

	virtual TFlt get_link_flow_car(){return 0;};
    virtual TFlt get_link_flow_truck(){return 0;};

    virtual TFlt get_link_tt_from_flow_car(TFlt flow){return 0;};
    virtual TFlt get_link_tt_from_flow_truck(TFlt flow){return 0;};

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

	// track parking vehicle numbers
	TInt m_parking_num_car;
	TInt m_parking_num_truck;
	TInt m_parking_num_rh;	

	TInt m_curb_parking_num_car; // curb parking car number
	TInt m_curb_parking_num_truck; // curb parking truck number
	TInt m_curb_parking_num_rh; // curb parking rh number

	TInt m_curb_doubleparking_num_car; // curb double parking car number
	TInt m_curb_doubleparking_num_truck; // curb double parking truck number
	TInt m_curb_doubleparking_num_rh; // curb double parking rh number

	TInt m_total_parking_num_car; // total parking car number
	TInt m_total_parking_num_truck; // total parking truck number
	TInt m_total_parking_num_rh; // total parking rh number

	TInt m_total_doubleparking_num_car; // total double parking car number
	TInt m_total_doubleparking_num_truck; // total double parking truck number
	TInt m_total_doubleparking_num_rh; // total double parking rh number

	TInt m_offstreet_parking_num; // only for the last cell 11/12/2024 Jiachao

	// PMC project - link level record congestion regime and differentiability
	bool m_diff_car;
	bool m_diff_truck;

	int m_congested_car;
	int m_congested_truck;
	// PMC end
};


class MNM_Dlink_Ctm_Multiclass_Curb : public MNM_Dlink_Multiclass_Curb
{
public:
	MNM_Dlink_Ctm_Multiclass_Curb(TInt ID,
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

	virtual ~MNM_Dlink_Ctm_Multiclass_Curb() override;	

	virtual TFlt get_link_supply() override;
	// Jiachao added
	virtual TFlt get_link_capacity() override;
    virtual int clear_incoming_array(TInt timestamp, TFlt _ratio_lane_closure) override;
    virtual TFlt get_link_flow_car() override;
    virtual TFlt get_link_flow_truck() override;
    virtual TFlt get_link_flow() override;
    virtual TFlt get_link_tt() override;
    virtual TFlt get_link_tt_from_flow_car(TFlt flow) override;
    virtual TFlt get_link_tt_from_flow_truck(TFlt flow) override;
	virtual TInt get_link_freeflow_tt_loading_car() override;  // intervals
	virtual TInt get_link_freeflow_tt_loading_truck() override;  // intervals

	class Ctm_Cell_Multiclass;
	int init_cell_array(TFlt unit_time, TFlt std_cell_length, TFlt last_cell_length);

	// Jiachao added in Aug 29
	int update_out_veh_curb(std::vector<TFlt> _ratio_lane_closure_list);

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
	TInt m_curb_spaces;
	TInt m_curb_dest;

	std::vector<Ctm_Cell_Multiclass*> m_cell_array;
	
	class Cell_Curb_Multiclass;
	std::vector<Cell_Curb_Multiclass*> m_curb_cell_array;

	int install_curb();

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
	
	virtual int evolve_curb(TInt timestamp) override;
};

class MNM_Dlink_Ctm_Multiclass_Curb::Ctm_Cell_Multiclass
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

class MNM_Dlink_Ctm_Multiclass_Curb::Cell_Curb_Multiclass
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

	std::deque<MNM_Veh*> m_offstreet_parking; // only for the last cell 11/12/2024 Jiachao

	// m_veh_departing/arriving_car/truck
	std::deque<MNM_Veh*> m_veh_departing;
	std::deque<MNM_Veh*> m_veh_departing_car;
	std::deque<MNM_Veh*> m_veh_departing_truck;

	std::deque<MNM_Veh*> m_veh_arriving;

	TInt m_cell_ID;
	TInt m_link_ID;
	TInt m_dest_ID;
	TInt m_cell_capacity;
	TFlt m_unit_time;
	TFlt m_current_time;
	TInt m_veh_parking_num;
	TInt m_veh_doubleparking_num;

	int move_veh_in_curb(TInt timestamp);
	int move_veh_out_curb(TInt timestamp);
};

/**************************************************************************
							Multiclass Link-Queue Model
**************************************************************************/

class MNM_Dlink_Lq_Multiclass_Curb : public MNM_Dlink_Multiclass_Curb
{
public:
	MNM_Dlink_Lq_Multiclass_Curb(TInt ID,
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

	virtual ~MNM_Dlink_Lq_Multiclass_Curb() override;	

	virtual TFlt get_link_supply() override;
	virtual TFlt get_link_capacity() override;
    virtual int clear_incoming_array(TInt timestamp, TFlt _ratio_lane_closure) override;

    virtual TFlt get_link_flow_car() override;
    virtual TFlt get_link_flow_truck() override;
    virtual TFlt get_link_flow() override;
    virtual TFlt get_link_tt() override;
    virtual TFlt get_link_tt_from_flow_car(TFlt flow) override;
    virtual TFlt get_link_tt_from_flow_truck(TFlt flow) override;
	virtual TInt get_link_freeflow_tt_loading_car() override;  // intervals
	virtual TInt get_link_freeflow_tt_loading_truck() override;  // intervals

	int update_perceived_density();

	virtual int evolve_curb(TInt timestamp) override;

	std::deque<MNM_Veh*> m_veh_queue_car;
	std::deque<MNM_Veh*> m_veh_queue_truck;
	std::deque<MNM_Veh*> m_veh_out_buffer_car;
	std::deque<MNM_Veh*> m_veh_out_buffer_truck;

	std::deque<MNM_Veh*> m_veh_parking_car;
	std::deque<MNM_Veh*> m_veh_parking_truck;
	std::deque<MNM_Veh*> m_veh_parking_rh;

	std::deque<MNM_Veh*> m_veh_doubleparking_car;
	std::deque<MNM_Veh*> m_veh_doubleparking_truck;
	std::deque<MNM_Veh*> m_veh_doubleparking_rh;

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

	TInt m_curb_spaces;
	TInt m_curb_dest;
};

class MNM_Dlink_Pq_Multiclass_Curb : public MNM_Dlink_Multiclass_Curb
{
public:
	MNM_Dlink_Pq_Multiclass_Curb(TInt ID,
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

	virtual ~MNM_Dlink_Pq_Multiclass_Curb() override;	

	virtual TFlt get_link_supply() override;
	virtual TFlt get_link_capacity() override;
    virtual int clear_incoming_array(TInt timestamp, TFlt _ratio_lane_closure) override;

    virtual TFlt get_link_flow_car() override;
    virtual TFlt get_link_flow_truck() override;
    virtual TFlt get_link_flow() override;

    virtual TFlt get_link_tt() override;
    virtual TFlt get_link_tt_from_flow_car(TFlt flow) override;
    virtual TFlt get_link_tt_from_flow_truck(TFlt flow) override;

	virtual TInt get_link_freeflow_tt_loading_car() override;  // intervals
	virtual TInt get_link_freeflow_tt_loading_truck() override;  // intervals

	virtual int evolve_curb(TInt timestamp) override;

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

class MNM_DMOND_Multiclass_Curb : public MNM_DMOND
{
public:
	MNM_DMOND_Multiclass_Curb(TInt ID, TFlt flow_scalar, TFlt veh_convert_factor);
	virtual ~MNM_DMOND_Multiclass_Curb() override;
	virtual int evolve_curb(TInt timestamp) override;
	TFlt m_veh_convert_factor;
};

/**************************************************************************
							   Destination node
**************************************************************************/
class MNM_DMDND_Multiclass_Curb : public MNM_DMDND
{
public:
	MNM_DMDND_Multiclass_Curb(TInt ID, TFlt flow_scalar, TFlt veh_convert_factor);
	virtual ~MNM_DMDND_Multiclass_Curb() override;
    virtual int evolve_curb(TInt timestamp) override;
	TFlt m_veh_convert_factor;
};

/**************************************************************************
                              In-Out node
**************************************************************************/
class MNM_Dnode_Inout_Multiclass_Curb : public MNM_Dnode
{
public:
	MNM_Dnode_Inout_Multiclass_Curb(TInt ID, TFlt flow_scalar, TFlt veh_convert_factor);
	virtual ~MNM_Dnode_Inout_Multiclass_Curb() override;
	virtual int prepare_loading() override;
    virtual int evolve_curb(TInt timestamp) override;
    virtual int add_out_link(MNM_Dlink* out_link) override;
    virtual int add_in_link(MNM_Dlink* in_link) override;

protected:
	int prepare_supplyANDdemand();
    int compute_flow();
	int move_vehicle_curb(TInt timestamp);

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



class MNM_Curb_Factory_Multiclass
{
public:
	MNM_Curb_Factory_Multiclass();
	std::unordered_map<TInt, TInt> m_curb_destination_map;
	std::unordered_map<TInt, TInt> m_curb_capacity_map;
	// Jiachao added
	std::unordered_map<TInt, std::vector<TInt>> m_path_inter_dest_map; // stores path_ID, inter dest IDs, curb IDs (in same order)
	std::unordered_map<TInt, std::vector<TInt>> m_path_inter_dest_map_rh;
	std::unordered_map<TInt, std::vector<TInt>> m_path_inter_dest_map_car;

	std::unordered_map<TInt, std::unordered_map<TInt, std::vector<TInt>>> m_od_inter_dest_map; // O D inter_dest_list
	std::unordered_map<TInt, std::vector<TInt>> m_interdest_curb_map; // inter_dest_ID curb_ID_list
	std::vector<TInt> m_interdest_list;
	std::unordered_map<TInt, std::vector<TFlt>> m_curb_price_list; 
	std::unordered_map<TInt, MNM_Destination*> m_interdest_map;

	/* multiclass curb prices */
	std::unordered_map<TInt, std::vector<TFlt>> m_curb_price_list_car;
	std::unordered_map<TInt, std::vector<TFlt>> m_curb_price_list_truck;
	std::unordered_map<TInt, std::vector<TFlt>> m_curb_price_list_rh;

	/* curb space reservation */
	std::unordered_map<TInt, std::unordered_map<TInt, std::vector<TInt>>> m_od_slz_reserve_map; // O -> D -> inter_dest_ID, curb_ID
};

class MNM_Statistics_Curb : public MNM_Statistics_Lrn
{
public:
	MNM_Statistics_Curb(const std::string& file_folder, MNM_ConfReader *conf_reader, MNM_ConfReader *record_config,
                 MNM_OD_Factory *od_factory, MNM_Node_Factory *node_factory, MNM_Link_Factory *link_factory, MNM_Curb_Factory_Multiclass *curb_factory);
  	virtual ~MNM_Statistics_Curb();	

	std::unordered_map<TInt, TFlt> m_record_interval_tt_car;
  	std::unordered_map<TInt, TFlt> m_load_interval_tt_car;
	std::unordered_map<TInt, TFlt> m_to_be_tt_car;

	std::unordered_map<TInt, TFlt> m_record_interval_tt_truck;
  	std::unordered_map<TInt, TFlt> m_load_interval_tt_truck;
	std::unordered_map<TInt, TFlt> m_to_be_tt_truck;

	std::unordered_map<TInt, TFlt> m_record_interval_cost_curb_truck;
	std::unordered_map<TInt, TFlt> m_record_interval_cost_curb_rh;
	std::unordered_map<TInt, TFlt> m_record_interval_cost_curb_car;
	// std::unordered_map<TInt, std::vector<TFlt>> *m_curb_price_list;

	// Jiachao added for recording curb occupancy for car, truck and rh
	std::unordered_map<TInt, std::vector<TFlt>> m_record_all_interval_parking_num_car;
	std::unordered_map<TInt, std::vector<TFlt>> m_record_all_interval_parking_num_truck;
	std::unordered_map<TInt, std::vector<TFlt>> m_record_all_interval_parking_num_rh;

	std::unordered_map<TInt, TFlt> m_record_interval_curb_cap_cost_car;
	std::unordered_map<TInt, TFlt> m_record_interval_curb_cap_cost_truck;
	std::unordered_map<TInt, TFlt> m_record_interval_curb_cap_cost_rh;

	MNM_ConfReader *m_adaptive_config;
	MNM_Curb_Factory_Multiclass *m_curb_factory;

	TFlt m_unit_time_cost_car; // $/s
	TFlt m_unit_time_cost_truck; // $/s
	TFlt m_unit_distance_cost_truck; // $/m
	TFlt m_unit_time_cost_rh; // $/s
	TFlt m_unit_distance_cost_rh; // $/m

	// Jiachao added for adaptive curb routing
	virtual int update_record_curb(TInt timestamp);
	virtual int init_record_curb();
};

/******************************************************************************************************************
*******************************************************************************************************************
												Curb Routing Models
*******************************************************************************************************************
******************************************************************************************************************/
class MNM_Routing_Driving_Fixed : public MNM_Routing_Biclass_Fixed
{
public:
    MNM_Routing_Driving_Fixed(PNEGraph &driving_graph,
                    			MNM_OD_Factory *od_factory, MNM_Node_Factory *node_factory,
                    			MNM_Link_Factory *link_factory, TInt route_frq = TInt(-1),
                    			TInt buffer_length=TInt(-1), TInt veh_class = TInt(0));

    virtual ~MNM_Routing_Driving_Fixed() override;
    // virtual int init_routing(Path_Table *driving_path_table=nullptr) override;
    virtual int update_routing(TInt timestamp) override;

	virtual int update_routing_curb_driving(TInt timestamp, MNM_Curb_Factory_Multiclass *curb_factory);
	virtual int register_veh_curb(MNM_Veh* veh, bool track = true);

	std::unordered_map<TInt, std::vector<TInt>> *m_inter_dest_map;
};

class MNM_Routing_Truck_Fixed : public MNM_Routing_Biclass_Fixed
{
public:
    MNM_Routing_Truck_Fixed(PNEGraph &driving_graph,
							MNM_OD_Factory *od_factory, MNM_Node_Factory *node_factory,
							MNM_Link_Factory *link_factory, TInt route_frq = TInt(-1),
							TInt buffer_length=TInt(-1), TInt veh_class = TInt(1));
    virtual ~MNM_Routing_Truck_Fixed() override;
    // virtual int init_routing(Path_Table *driving_path_table=nullptr) override;
    virtual int update_routing(TInt timestamp) override;

	virtual int update_routing_curb_truck(TInt timestamp,  MNM_Curb_Factory_Multiclass *curb_factory);
	virtual int register_veh_curb(MNM_Veh* veh, bool track = true);

	std::unordered_map<TInt, std::vector<TInt>> *m_inter_dest_map;
};

class MNM_Routing_Ridehail_Fixed : public MNM_Routing_Biclass_Fixed
{
public:
    MNM_Routing_Ridehail_Fixed(PNEGraph &driving_graph,
                    			MNM_OD_Factory *od_factory, MNM_Node_Factory *node_factory,
                    			MNM_Link_Factory *link_factory, TInt route_frq = TInt(-1),
                    			TInt buffer_length=TInt(-1), TInt veh_class = TInt(2));
    virtual ~MNM_Routing_Ridehail_Fixed() override;
    // virtual int init_routing(Path_Table *driving_path_table=nullptr) override;

    virtual int update_routing(TInt timestamp) override;

	virtual int update_routing_curb_rh(TInt timestamp,  MNM_Curb_Factory_Multiclass *curb_factory);

	virtual int register_veh_curb(MNM_Veh* veh, bool track = true);

	std::unordered_map<TInt, std::vector<TInt>> *m_inter_dest_map;
};

class MNM_Routing_Curb_Adaptive : public MNM_Routing_Adaptive
{
public:
	MNM_Routing_Curb_Adaptive(const std::string& file_folder, 
							  PNEGraph &graph, 
							  MNM_Statistics* statistics,
							  MNM_Statistics_Curb* statistics_curb,
                       		  MNM_OD_Factory *od_factory, 
							  MNM_Node_Factory *node_factory, 
							  MNM_Link_Factory *link_factory,
							  MNM_Curb_Factory_Multiclass *curb_factory);

	virtual ~MNM_Routing_Curb_Adaptive() override;

	virtual int init_routing(Path_Table *path_table=nullptr) override;

	virtual int update_routing(TInt timestamp) override;

	virtual int update_routing_curb_adaptive(TInt timestamp);

	virtual int init_routing_curb();

	virtual int update_statistics_dest(TInt _dest_node_ID, TInt _veh_type);

	virtual int return_statistics_dest(TInt _dest_node_ID, TInt _veh_type);

	// private:
	MNM_Statistics* m_statistics;
	MNM_Statistics_Curb* m_statistics_curb;
	Routing_Table *m_table;
	// Jiachao added
	Routing_Table *m_table_truck;
	Routing_Table *m_table_rh;

	TInt m_routing_freq;
	MNM_ConfReader *m_self_config;

	MNM_Curb_Factory_Multiclass *m_curb_factory;

	std::vector<TInt> m_intermediate_dest_list;

	std::string m_file_folder;
};

class MNM_Routing_Biclass_Hybrid_Curb : public MNM_Routing_Biclass_Hybrid
{
public:
	MNM_Routing_Biclass_Hybrid_Curb(const std::string& file_folder, 
									PNEGraph &graph, 
									MNM_Statistics* statistics,
									MNM_Statistics_Curb* statistics_curb,
  									MNM_OD_Factory *od_factory, 
									MNM_Node_Factory *node_factory, 
									MNM_Link_Factory *link_factory, 
									MNM_Curb_Factory_Multiclass *curb_factory,
									TInt route_frq_fixed, 
									TInt buffer_length);

	virtual ~MNM_Routing_Biclass_Hybrid_Curb() override;
	virtual int init_routing(Path_Table *path_table=NULL) override;
  	virtual int update_routing(TInt timestamp) override;
	// virtual int update_routing_curb(TInt timestamp, std::unordered_map<TInt, std::vector<TInt>> *inter_dest_map);
	virtual int update_routing_curb(TInt timestamp);

	// Jiachao 0604
	// virtual int update_routing_curb_separate(TInt timestamp, std::unordered_map<TInt, std::vector<TInt>> *inter_dest_map, std::unordered_map<TInt, std::vector<TInt>> *inter_dest_map_rh);
	// virtual int update_routing_curb_separate(TInt timestamp);

	virtual int init_routing_curb(Path_Table *path_table_car=NULL, Path_Table *path_table_truck=NULL, Path_Table *path_table_rh=NULL);

  	MNM_Routing_Curb_Adaptive* m_routing_adaptive;
  	MNM_Routing_Driving_Fixed* m_routing_fixed_car;
  	MNM_Routing_Truck_Fixed* m_routing_fixed_truck;
	MNM_Routing_Ridehail_Fixed* m_routing_fixed_ridehail;

	MNM_Curb_Factory_Multiclass* m_curb_factory;

};

class MNM_Dta_Multiclass_Curb : public MNM_Dta
{
public:
	explicit MNM_Dta_Multiclass_Curb(const std::string& file_folder);
	virtual ~MNM_Dta_Multiclass_Curb() override;
	virtual int initialize() override;

	virtual int build_from_files_separate();

	virtual int pre_loading() override;

	MNM_Curb_Factory_Multiclass *m_curb_factory;

	MNM_Statistics_Curb *m_statistics_curb;

	// MNM_Node_Factory_Multiclass_Curb *m_node_factory;

	virtual int set_statistics_curb();

	virtual int set_routing_curb_separate();

	virtual Path_Table *load_path_table_curb(const std::string& file_name, const PNEGraph& graph, TInt num_path,
                                     bool w_buffer = false, TInt m_class = TInt(0), bool w_ID = false);

	virtual int load_once_curb(bool verbose, TInt load_int, TInt assign_int);

	virtual int loading_curb(bool verbose);

};

/******************************************************************************************************************
*******************************************************************************************************************
												Multiclass OD
*******************************************************************************************************************
******************************************************************************************************************/

class MNM_Destination_Multiclass_Curb : public MNM_Destination
{
public:
	explicit MNM_Destination_Multiclass_Curb(TInt ID);
	virtual ~MNM_Destination_Multiclass_Curb() override;
};

class MNM_Origin_Multiclass_Curb : public MNM_Origin
{
public:
	MNM_Origin_Multiclass_Curb(TInt ID, TInt max_interval, TFlt flow_scalar, TInt frequency);
	virtual ~MNM_Origin_Multiclass_Curb() override;

	// jiachao added in Sep
	int release_one_interval_curb(TInt current_interval,
								    MNM_Veh_Factory* veh_factory, 
								    TInt assign_interval, 
									TFlt adaptive_ratio_car,
									TFlt adaptive_ratio_truck,
									TFlt adaptive_ratio_rh);

    int release_one_interval_curb_hybrid(TInt current_interval,
										 MNM_Veh_Factory* veh_factory,
										 MNM_Curb_Factory_Multiclass* curb_factory,
										 TInt assign_interval,
										 TFlt adaptive_ratio_car,
										 TFlt adaptive_ratio_car_curb,// fraction of adaptive cars use hybrid routing for curb use
										 TFlt adaptive_ratio_truck,
										 TFlt adaptive_ratio_truck_curb, // fraction of adaptive trucks use hybrid routing for curb use
										 TFlt adaptive_ratio_rh);
	// jiachao added in Sep
	int add_dest_demand_multiclass_curb(MNM_Destination_Multiclass_Curb *dest,
									   TFlt* demand_car,
									   TFlt* demand_tnc,
									   TFlt* demand_truck);

	// two new unordered_map for both classes
	std::unordered_map<MNM_Destination_Multiclass_Curb*, TFlt*> m_demand_car;
	std::unordered_map<MNM_Destination_Multiclass_Curb*, TFlt*> m_demand_truck;
	std::unordered_map<MNM_Destination_Multiclass_Curb*, TFlt*> m_demand_tnc;
};

/******************************************************************************************************************
*******************************************************************************************************************
												Multiclass Vehicle
*******************************************************************************************************************
******************************************************************************************************************/

class MNM_Veh_Multiclass_Curb : public MNM_Veh
{
public:
	MNM_Veh_Multiclass_Curb(TInt ID, TInt vehicle_class, TInt start_time);
	virtual ~MNM_Veh_Multiclass_Curb() override;

    virtual TInt get_class() override {return m_class;};  // virtual getter

	TInt m_class;
	TFlt m_visual_position_on_link; //[0(start), 1(end)], for vehicle-based visualization
	TFlt m_parking_location;

	std::deque<TInt> m_curb_destination_list; // record the curb destination link for parking
	std::deque<TInt> m_destination_list; // predefined destination settings, initialize from input file and fixed during the whole loading
	std::vector<TInt> m_parking_duration_list; // predefined parking duration settings, but can be changed for real parking duration
	std::vector<TInt> m_arrival_time_list; // record the arrival time
	std::vector<TInt> m_departure_time_list; // record the departure time
	TInt m_complete_stop_current_link;
	TInt m_reserved_slz; // new feature to indicate the curb space (only) in smart loading zone is reserved for this vehicle
};

/******************************************************************************************************************
*******************************************************************************************************************
												New Vehicle Factory
*******************************************************************************************************************
******************************************************************************************************************/

class MNM_Veh_Factory_Multiclass_Curb : public MNM_Veh_Factory
{
public:
	MNM_Veh_Factory_Multiclass_Curb();
	virtual ~MNM_Veh_Factory_Multiclass_Curb() override;

	// use this one instead of make_veh in the base class
	MNM_Veh_Multiclass_Curb* make_veh_multiclass(TInt timestamp, Vehicle_type veh_type, TInt vehicle_cls);

	MNM_Veh_Multiclass_Curb* make_veh_multiclass_curb(TInt timestamp, 
												Vehicle_type veh_type, 
												TInt vehicle_cls, 
												MNM_Curb_Factory_Multiclass* curb_factory,
												TInt _o_node_ID,
												TInt _d_node_ID);
	
	// new feature for vehicles to reserve curb spaces - based on OD pair
	MNM_Veh_Multiclass_Curb* make_veh_multiclass_curb_reserved(TInt timestamp, 
												Vehicle_type veh_type, 
												TInt vehicle_cls, 
												MNM_Curb_Factory_Multiclass* curb_factory,
												TInt _o_node_ID,
												TInt _d_node_ID);
};

/******************************************************************************************************************
*******************************************************************************************************************
												New OD Factory
*******************************************************************************************************************
******************************************************************************************************************/

class MNM_OD_Factory_Multiclass_Curb : public MNM_OD_Factory
{
public:
	MNM_OD_Factory_Multiclass_Curb();
	virtual ~MNM_OD_Factory_Multiclass_Curb() override;
	virtual MNM_Destination_Multiclass_Curb* make_destination(TInt ID) override;
	virtual MNM_Origin_Multiclass_Curb* make_origin(TInt ID, 
                                               TInt max_interval,
                                               TFlt flow_scalar,
											   TInt frequency) override;
};

/******************************************************************************************************************
*******************************************************************************************************************
												New Link Factory
*******************************************************************************************************************
******************************************************************************************************************/

class MNM_Link_Factory_Multiclass_Curb : public MNM_Link_Factory
{
public:
	MNM_Link_Factory_Multiclass_Curb();
	virtual ~MNM_Link_Factory_Multiclass_Curb() override;

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

/******************************************************************************************************************
*******************************************************************************************************************
												New Node Factory
*******************************************************************************************************************
******************************************************************************************************************/

class MNM_Node_Factory_Multiclass_Curb : public MNM_Node_Factory
{
public:
	MNM_Node_Factory_Multiclass_Curb();
	virtual ~MNM_Node_Factory_Multiclass_Curb() override;
	
	MNM_Dnode *make_node_multiclass_curb(TInt ID, DNode_type_multiclass node_type, TFlt flow_scalar, TFlt veh_convert_factor);
};

/******************************************************************************************************************
*******************************************************************************************************************
											New Emission class
*******************************************************************************************************************
******************************************************************************************************************/

class MNM_Cumulative_Emission_Multiclass_Curb : public MNM_Cumulative_Emission
{
public:
	MNM_Cumulative_Emission_Multiclass_Curb(TFlt unit_time, TInt freq);
	virtual ~MNM_Cumulative_Emission_Multiclass_Curb() override;

	// new functions for trucks
	TFlt calculate_fuel_rate_truck(TFlt v);
	TFlt calculate_CO2_rate_truck(TFlt v);
	TFlt calculate_HC_rate_truck(TFlt v);
	TFlt calculate_CO_rate_truck(TFlt v);
	TFlt calculate_NOX_rate_truck(TFlt v);

	virtual int update(MNM_Veh_Factory* veh_factory) override;
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

class MNM_IO_Multiclass_Curb : public MNM_IO
{
public:
	static int build_node_factory_multiclass(const std::string& file_folder,
											 MNM_ConfReader *conf_reader,
											 MNM_Node_Factory *node_factory,
                                             const std::string& file_name = "MNM_input_node");
 	
	static int build_link_factory_multiclass(const std::string& file_folder,
 											 MNM_ConfReader *conf_reader,
 											 MNM_Link_Factory *link_factory,
											 MNM_Curb_Factory_Multiclass *curb_factory,
 											 const std::string& file_name = "MNM_input_link");
	static int build_demand_multiclass_curb(const std::string& file_folder,
										    MNM_ConfReader *conf_reader,
										    MNM_OD_Factory *od_factory,
										    const std::string& file_name = "MNM_input_demand");
	
	static int build_curb_factory_multiclass(const std::string& file_folder,
 									 			MNM_ConfReader *conf_reader,
 									 			MNM_Curb_Factory_Multiclass *curb_factory);
	
	static int load_inter_dest_curb_separate(const std::string& file_folder,
 												MNM_ConfReader *conf_reader,
												MNM_Curb_Factory_Multiclass *curb_factory);
	
	static int build_od_factory_multiclass_curb(const std::string& file_folder, 
												MNM_ConfReader *conf_reader,
                               					MNM_OD_Factory *od_factory, 
												MNM_Node_Factory *node_factory,
												const std::string& file_name = "MNM_input_od");
	
	static PNEGraph build_graph(const std::string& file_folder, MNM_ConfReader *conf_reader);

};

namespace MNM_DTA_GRADIENT_CURB
{
	TFlt get_link_inflow_car(MNM_Dlink_Multiclass_Curb* link, 
                    	TFlt start_time, TFlt end_time);
	
	TFlt get_link_inflow_rh(MNM_Dlink_Multiclass_Curb* link, 
						TFlt start_time, TFlt end_time);

	TFlt get_link_inflow_truck(MNM_Dlink_Multiclass_Curb* link, 
						TFlt start_time, TFlt end_time);
	
	TFlt get_travel_time_car(MNM_Dlink_Multiclass_Curb* link, TFlt start_time, TFlt unit_interval, TInt end_loading_timestamp);

	TFlt get_travel_time_truck(MNM_Dlink_Multiclass_Curb* link, TFlt start_time, TFlt unit_interval, TInt end_loading_timestamp);

	// // curb parking state (veh num)
	// TFlt get_curb_parking_num_car(MNM_Dlink_Multiclass_Curb* link);
	// TFlt get_curb_parking_num_truck(MNM_Dlink_Multiclass_Curb* link);
	// TFlt get_curb_parking_num_rh(MNM_Dlink_Multiclass_Curb* link);

	// TFlt get_curb_doubleparking_num_car(MNM_Dlink_Multiclass_Curb* link);
	// TFlt get_curb_doubleparking_num_truck(MNM_Dlink_Multiclass_Curb* link);
	// TFlt get_curb_doubleparking_num_rh(MNM_Dlink_Multiclass_Curb* link);

	// curb in-out flow for three modes
	TFlt get_curb_inflow_car(MNM_Dlink_Multiclass_Curb* link, TFlt start_time, TFlt end_time);
	
	TFlt get_curb_outflow_car(MNM_Dlink_Multiclass_Curb* link, TFlt start_time, TFlt end_time);

	TFlt get_curb_inflow_truck(MNM_Dlink_Multiclass_Curb* link, TFlt start_time, TFlt end_time);

	TFlt get_curb_outflow_truck(MNM_Dlink_Multiclass_Curb* link, TFlt start_time, TFlt end_time);

	TFlt get_curb_inflow_rh(MNM_Dlink_Multiclass_Curb* link, TFlt start_time, TFlt end_time);

	TFlt get_curb_outflow_rh(MNM_Dlink_Multiclass_Curb* link, TFlt start_time, TFlt end_time);

	// moving car density
	TFlt get_link_density_car_robust(MNM_Dlink_Multiclass_Curb* link, TFlt time, TInt end_loading_timestamp, TInt num_trials);

	// moving rh density
	TFlt get_link_density_rh_robust(MNM_Dlink_Multiclass_Curb* link, TFlt time, TInt end_loading_timestamp, TInt num_trials);

	// parking rh density
	TFlt get_link_density_stop_rh_robust(MNM_Dlink_Multiclass_Curb* link, TFlt time, TInt end_loading_timestamp, TInt num_trials);

	// moving truck density
	TFlt get_link_density_truck_robust(MNM_Dlink_Multiclass_Curb* link, TFlt time, TInt end_loading_timestamp, TInt num_trials);

	// parking truck density
	TFlt get_link_density_stop_truck_robust(MNM_Dlink_Multiclass_Curb* link, TFlt time, TInt end_loading_timestamp, TInt num_trials);

	// DAR matrices
	int add_dar_records_car(std::vector<dar_record*> &record, MNM_Dlink_Multiclass_Curb* link, 
						std::set<MNM_Path*> pathset, TFlt start_time, TFlt end_time);
	int add_dar_records_truck(std::vector<dar_record*> &record, MNM_Dlink_Multiclass_Curb* link, 
						std::set<MNM_Path*> pathset, TFlt start_time, TFlt end_time);
	int add_dar_records_rh(std::vector<dar_record*> &record, MNM_Dlink_Multiclass_Curb* link, 
						std::set<MNM_Path*> pathset, TFlt start_time, TFlt end_time);

	// Fujitsu
	int add_dar_records_car_out(std::vector<dar_record*> &record, MNM_Dlink_Multiclass_Curb* link, 
						std::set<MNM_Path*> pathset, TFlt start_time, TFlt end_time);
	int add_dar_records_truck_out(std::vector<dar_record*> &record, MNM_Dlink_Multiclass_Curb* link, 
						std::set<MNM_Path*> pathset, TFlt start_time, TFlt end_time);
	int add_dar_records_rh_out(std::vector<dar_record*> &record, MNM_Dlink_Multiclass_Curb* link,
						std::set<MNM_Path*> pathset, TFlt start_time, TFlt end_time);

	int add_dar_records_curb_arrival_truck(std::vector<dar_record*> &record, MNM_Dlink_Multiclass_Curb* link, 
						std::set<MNM_Path*> pathset, TFlt start_time, TFlt end_time);
	int add_dar_records_curb_departure_truck(std::vector<dar_record*> &record, MNM_Dlink_Multiclass_Curb* link, 
						std::set<MNM_Path*> pathset, TFlt start_time, TFlt end_time);

	int add_dar_records_curb_arrival_rh(std::vector<dar_record*> &record, MNM_Dlink_Multiclass_Curb* link, 
						std::set<MNM_Path*> pathset, TFlt start_time, TFlt end_time);
	int add_dar_records_curb_departure_rh(std::vector<dar_record*> &record, MNM_Dlink_Multiclass_Curb* link, 
						std::set<MNM_Path*> pathset, TFlt start_time, TFlt end_time);

	int add_dar_records_curb_arrival_car(std::vector<dar_record*> &record, MNM_Dlink_Multiclass_Curb* link, 
						std::set<MNM_Path*> pathset, TFlt start_time, TFlt end_time);
	int add_dar_records_curb_departure_car(std::vector<dar_record*> &record, MNM_Dlink_Multiclass_Curb* link, 
						std::set<MNM_Path*> pathset, TFlt start_time, TFlt end_time);

};


#endif