#include "io.h"
// #include "multiclass_curb.h"
#include "multiclass.h"
// #include "dso.h"
#include "Snap.h"
#include <iostream>
#include <vector>

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

						false - m_link_diff_car/truck = true
					
						true - m_link_diff_car/truck = false

		2.3 build_link_cost_map: get link tt for cars and trucks

		2.4 get_link_marginal_cost: compute link marginal cost for each link, including upper and lower bound

			m_lmc_car_lower/upper
			m_lmc_truck_lower/upper

*******************************************************************************************************************
******************************************************************************************************************/

int main()
{
	std::string folder = "/srv/data/jiachao/MAC-POSTS/data/input_files_nie_lq_dtc_new_v2";
	// lower PMC weight and upper PMC weight
	// std::vector<TFlt> weight_lower = {0.2, 0.0};
	// std::vector<TFlt> weight_upper = {0.8, 1.0};
	std::vector<TFlt> weight_lower = {0.4, 0.2};
	std::vector<TFlt> weight_upper = {0.6, 0.8};

	for (size_t i = 0; i < weight_lower.size(); ++i) {
		printf("\nRunning DSO with weight_lower = %f, weight_upper = %f\n", static_cast<double>(weight_lower[i]), static_cast<double>(weight_upper[i]));
		MNM_Dso_Multiclass *test_dso = new MNM_Dso_Multiclass(folder);
		test_dso -> run_dso_departure_time_choice_msa(false, folder, int(1), weight_lower[i], weight_upper[i], TInt(1), "scaled_demand", "v1");
		delete test_dso;
	}

	printf("\nFinished test_dso!\n");
	return 0;
}

// new function

