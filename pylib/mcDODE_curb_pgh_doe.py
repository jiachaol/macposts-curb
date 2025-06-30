import os
import sys
import numpy as np
import pandas as pd
import hashlib
import time
import shutil
from scipy.sparse import coo_matrix, csr_matrix
import pickle
import torch
import MNMAPI
MNM_nb_folder = "/srv/data/jiachao/MAC-POSTS/side_project/network_builder"
sys.path.append(MNM_nb_folder)
from MNM_mcnb_curb_pgh import *

class MCDODE_Curb():
    def __init__(self, nb, config):
        self.config = config
        self.nb = nb
        self.num_assign_interval = nb.config.config_dict['DTA']['max_interval']
        self.ass_freq = nb.config.config_dict['DTA']['assign_frq']
        self.num_link = nb.config.config_dict['DTA']['num_of_link']
        self.num_path = nb.config.config_dict['FIXED']['num_path'] # car paths
        self.num_path_curb = nb.config.config_dict['FIXED']['num_path_curb'] # truck paths
        self.num_path_curb_rh = nb.config.config_dict['FIXED']['num_path_curb_rh'] # rh paths
        self.num_loading_interval = self.num_assign_interval * self.ass_freq
        self.data_dict = dict()
        self.num_data = self.config['num_data']

        self.observed_links_count = self.config['observed_links_count'] # count observed links
        self.observed_links_tt = self.config['observed_links_tt'] # tt observed links
        self.observed_links_curb = self.config['observed_links_curb'] # curb arrival and departure observed links

        self.paths_list_car = self.config['paths_list_car'] # car paths
        self.paths_list_truck = self.config['paths_list_truck'] # truck paths
        self.paths_list_rh = self.config['paths_list_rh'] # rh paths
        
        self.car_count_agg_L_list = None
        self.truck_count_agg_L_list = None

        self.truck_curb_agg_N_list = None
        self.truck_curb_agg_H_list = None

        self.rh_curb_agg_N_list = None
        self.rh_curb_agg_H_list = None

        self.store_folder = None

        assert (len(self.paths_list_car) == self.num_path)
        assert (len(self.paths_list_truck) == self.num_path_curb)
        assert (len(self.paths_list_rh) == self.num_path_curb_rh)
    
    def _add_car_link_flow_data(self, link_flow_df_list):
        assert (self.num_data == len(link_flow_df_list))
        self.data_dict['car_link_flow'] = link_flow_df_list
    
    def _add_rh_link_flow_data(self, link_flow_df_list):
        assert (self.num_data == len(link_flow_df_list))
        self.data_dict['rh_link_flow'] = link_flow_df_list

    def _add_truck_link_flow_data(self, link_flow_df_list):
        assert (self.num_data == len(link_flow_df_list))
        self.data_dict['truck_link_flow'] = link_flow_df_list

    def _add_car_link_tt_data(self, link_spd_df_list):
        assert (self.num_data == len(link_spd_df_list))
        self.data_dict['car_link_tt'] = link_spd_df_list

    def _add_truck_link_tt_data(self, link_spd_df_list):
        assert (self.num_data == len(link_spd_df_list))
        self.data_dict['truck_link_tt'] = link_spd_df_list
    
    def _add_rh_curb_arr_data(self, rh_link_curb_arr_list):
        assert (self.num_data == len(rh_link_curb_arr_list))
        self.data_dict['rh_curb_inflow'] = rh_link_curb_arr_list
    
    def _add_rh_curb_dep_data(self, rh_link_curb_dep_list):
        assert (self.num_data == len(rh_link_curb_dep_list))
        self.data_dict['rh_curb_outflow'] = rh_link_curb_dep_list
    
    def _add_truck_curb_arr_data(self, truck_link_curb_arr_list):
        assert (self.num_data == len(truck_link_curb_arr_list))
        self.data_dict['truck_curb_inflow'] = truck_link_curb_arr_list
    
    def _add_truck_curb_dep_data(self, truck_link_curb_dep_list):
        assert (self.num_data == len(truck_link_curb_dep_list))
        self.data_dict['truck_curb_outflow'] = truck_link_curb_dep_list
    
    # def _add_link_curb_data_truck(self, truck_link_curb_arr_list, truck_link_curb_dep_list):
    #     assert (self.num_data == len(truck_link_curb_arr_list))
    #     assert (self.num_data == len(truck_link_curb_dep_list))
    #     self.data_dict['truck_curb_inflow'] = truck_link_curb_arr_list 
    #     self.data_dict['truck_curb_outflow'] = truck_link_curb_dep_list 
        
    # def _add_link_curb_data_rh(self, rh_link_curb_arr_list, rh_link_curb_dep_list):
    #     assert (self.num_data == len(rh_link_curb_arr_list))
    #     assert (self.num_data == len(rh_link_curb_dep_list))
    #     self.data_dict['rh_curb_inflow'] = rh_link_curb_arr_list 
    #     self.data_dict['rh_curb_outflow'] = rh_link_curb_dep_list 
    
    # def _add_prior_od_data(self, prior_od_list_car, prior_od_list_truck, prior_od_list_rh):
    #     assert (self.num_data == len(prior_od_list_car))
    #     assert (self.num_data == len(prior_od_list_truck))
    #     assert (self.num_data == len(prior_od_list_rh))
    #     self.data_dict['car_prior_od'] = prior_od_list_car
    #     self.data_dict['truck_prior_od'] = prior_od_list_truck
    #     self.data_dict['rh_prior_od'] = prior_od_list_rh

    def aggregate_f(self, f_car, f_truck, f_rh):
        od_demand_est_car = dict()
        od_demand_est_truck = dict()
        od_demand_est_rh = dict()

        f_car = f_car.reshape(-1, self.num_assign_interval)
        f_truck = f_truck.reshape(-1, self.num_assign_interval)
        f_rh = f_rh.reshape(-1, self.num_assign_interval)

        for i, path_ID in enumerate(self.nb.path_table.ID2path.keys()):
            path = self.nb.path_table.ID2path[path_ID]
            O_node = path.origin_node
            D_node = path.destination_node
            O = self.nb.od.O_dict.inv[O_node]
            D = self.nb.od.D_dict.inv[D_node]
            if O not in od_demand_est_car:
                od_demand_est_car[O] = dict()
            else:
                if D not in od_demand_est_car[O]:
                    od_demand_est_car[O][D] = np.zeros(self.num_assign_interval)
            
            od_demand_est_car[O][D] = od_demand_est_car[O][D] + f_car[i,:]
            
        for j, path_ID in enumerate(self.nb.path_table_curb.ID2path.keys()):
            path = self.nb.path_table_curb.ID2path[path_ID]
            O_node = path.origin_node
            D_node = path.destination_node
            O = self.nb.od.O_dict.inv[O_node]
            D = self.nb.od.D_dict.inv[D_node]
            if O not in od_demand_est_truck:
                od_demand_est_truck[O] = dict()
            else:
                if D not in od_demand_est_truck[O]:
                    od_demand_est_truck[O][D] = np.zeros(self.num_assign_interval)
            
            od_demand_est_truck[O][D] = od_demand_est_truck[O][D] + f_truck[j,:]
        
        for k, path_ID in enumerate(self.nb.path_table_curb_rh.ID2path.keys()):
            path = self.nb.path_table_curb_rh.ID2path[path_ID]
            O_node = path.origin_node
            D_node = path.destination_node
            O = self.nb.od.O_dict.inv[O_node]
            D = self.nb.od.D_dict.inv[D_node]
            if O not in od_demand_est_rh:
                od_demand_est_rh[O] = dict()
            else:
                if D not in od_demand_est_rh[O]:
                    od_demand_est_rh[O][D] = np.zeros(self.num_assign_interval)
            
            od_demand_est_rh[O][D] = od_demand_est_rh[O][D] + f_rh[k,:]
        
        f_car = f_car.flatten()
        f_truck = f_truck.flatten()
        f_rh = f_rh.flatten()
        
        return od_demand_est_car, od_demand_est_truck, od_demand_est_rh

    def add_data(self, data_dict):
        if self.config['car_count_agg']:
            self.car_count_agg_L_list = data_dict['car_count_agg_L_list']

        if self.config['truck_count_agg']:
            self.truck_count_agg_L_list = data_dict['truck_count_agg_L_list']
        
        if self.config['truck_curb_agg']:
            self.truck_curb_agg_N_list = data_dict['truck_curb_agg_N']
            self.truck_curb_agg_H_list = data_dict['truck_curb_agg_H']

        if self.config['rh_curb_agg']:
            self.rh_curb_agg_N_list = data_dict['rh_curb_agg_N']
            self.rh_curb_agg_H_list = data_dict['rh_curb_agg_H']

        if self.config['use_car_link_flow']:
            self._add_car_link_flow_data(data_dict['car_link_flow'])

        if self.config['use_truck_link_flow']:
            self._add_truck_link_flow_data(data_dict['truck_link_flow'])
        
        if self.config['use_rh_link_flow']:
            self._add_rh_link_flow_data(data_dict['rh_link_flow'])

        if self.config['use_car_link_tt']:
            self._add_car_link_tt_data(data_dict['car_link_tt'])

        if self.config['use_truck_link_tt']:
            self._add_truck_link_tt_data(data_dict['truck_link_tt'])

        if self.config['use_truck_curb_arr']:
            self._add_truck_curb_arr_data(data_dict['truck_curb_inflow'])

        if self.config['use_truck_curb_dep']:
            self._add_truck_curb_dep_data(data_dict['truck_curb_outflow'])
            
        if self.config['use_rh_curb_arr']:
            self._add_rh_curb_arr_data(data_dict['rh_curb_inflow'])
        
        if self.config['use_rh_curb_dep']:
            self._add_rh_curb_dep_data(data_dict['rh_curb_outflow'])

    # DAR matrix for count observed links
    def get_dar_count(self, dta, f_car, f_truck, f_rh):
        car_dar = csr_matrix((self.num_assign_interval * len(self.observed_links_count), self.num_assign_interval * len(self.paths_list_car)))
        truck_dar = csr_matrix((self.num_assign_interval * len(self.observed_links_count), self.num_assign_interval * len(self.paths_list_truck)))
        rh_dar = csr_matrix((self.num_assign_interval * len(self.observed_links_count), self.num_assign_interval * len(self.paths_list_rh)))

        if self.config['use_car_link_flow']:
            # (num_assign_timesteps x num_links x num_path x num_assign_timesteps) x 5
            raw_car_dar = dta.get_car_dar_matrix_sep(np.arange(0, self.num_loading_interval, self.ass_freq), 
                                                np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq)
            # print("raw car dar", raw_car_dar)
            # num_assign_interval * num_e_link, num_assign_interval * num_e_path
            car_dar = self._massage_raw_dar(raw_car_dar, self.ass_freq, f_car, self.num_assign_interval, self.paths_list_car, self.observed_links_count)
        
            # (num_assign_timesteps x num_links x num_path x num_assign_timesteps) x 5
            raw_rh_dar = dta.get_rh_dar_matrix_sep(np.arange(0, self.num_loading_interval, self.ass_freq), 
                                                    np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq)
            # num_assign_interval * num_e_link, num_assign_interval * num_e_path
            rh_dar = self._massage_raw_dar(raw_rh_dar, self.ass_freq, f_rh, self.num_assign_interval, self.paths_list_rh, self.observed_links_count)
        
        if self.config['use_truck_link_flow']:
            # (num_assign_timesteps x num_links x num_path x num_assign_timesteps) x 5
            raw_truck_dar = dta.get_truck_dar_matrix_sep(np.arange(0, self.num_loading_interval, self.ass_freq), 
                                                    np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq)
            # num_assign_interval * num_e_link, num_assign_interval * num_e_path
            truck_dar = self._massage_raw_dar(raw_truck_dar, self.ass_freq, f_truck, self.num_assign_interval, self.paths_list_truck, self.observed_links_count)
            
        return (car_dar, truck_dar, rh_dar)

    # DAR matrix for TT observed links
    def get_dar_tt(self, dta, f_car, f_truck, f_rh):
        car_dar = csr_matrix((self.num_assign_interval * len(self.observed_links_tt), self.num_assign_interval * len(self.paths_list_car)))
        truck_dar = csr_matrix((self.num_assign_interval * len(self.observed_links_tt), self.num_assign_interval * len(self.paths_list_truck)))
        rh_dar = csr_matrix((self.num_assign_interval * len(self.observed_links_tt), self.num_assign_interval * len(self.paths_list_rh)))

        if self.config['use_car_link_tt']:
            raw_car_dar = dta.get_car_dar_matrix_tt(np.arange(0, self.num_loading_interval, self.ass_freq), 
                                                np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq)
            car_dar = self._massage_raw_dar(raw_car_dar, self.ass_freq, f_car, self.num_assign_interval, self.paths_list_car, self.observed_links_tt)
        
            raw_rh_dar = dta.get_rh_dar_matrix_tt(np.arange(0, self.num_loading_interval, self.ass_freq), 
                                                    np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq)
            rh_dar = self._massage_raw_dar(raw_rh_dar, self.ass_freq, f_rh, self.num_assign_interval, self.paths_list_rh, self.observed_links_tt)
        
        if self.config['use_truck_link_tt']:
            raw_truck_dar = dta.get_truck_dar_matrix_tt(np.arange(0, self.num_loading_interval, self.ass_freq), 
                                                    np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq)
            truck_dar = self._massage_raw_dar(raw_truck_dar, self.ass_freq, f_truck, self.num_assign_interval, self.paths_list_truck, self.observed_links_tt)
        
        return (car_dar, truck_dar, rh_dar)

    # DAR matrix for curbside arrival and departure
    def get_dar_arrival(self, dta, f_truck, f_rh):
        truck_dar_arrival = csr_matrix((self.num_assign_interval * len(self.observed_links_curb), self.num_assign_interval * len(self.paths_list_truck)))
        rh_dar_arrival = csr_matrix((self.num_assign_interval * len(self.observed_links_curb), self.num_assign_interval * len(self.paths_list_rh)))

        if self.config['use_truck_curb_arr']:
            # truck arrival DAR
            raw_truck_dar_arrival = dta.get_truck_dar_arrival_matrix_sep(np.arange(0, self.num_loading_interval, self.ass_freq), 
                                                    np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq)
            truck_dar_arrival = self._massage_raw_dar(raw_truck_dar_arrival, self.ass_freq, f_truck, self.num_assign_interval, self.paths_list_truck, self.observed_links_curb)
        
        if self.config['use_rh_curb_arr']:
            # RH arrival DAR
            raw_rh_dar_arrival = dta.get_rh_dar_arrival_matrix_sep(np.arange(0, self.num_loading_interval, self.ass_freq), 
                                                    np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq)
            rh_dar_arrival = self._massage_raw_dar(raw_rh_dar_arrival, self.ass_freq, f_rh, self.num_assign_interval, self.paths_list_rh, self.observed_links_curb)

        return (truck_dar_arrival, rh_dar_arrival)
    
    def get_dar_departure(self, dta, f_truck, f_rh):
        truck_dar_departure = csr_matrix((self.num_assign_interval * len(self.observed_links_curb), self.num_assign_interval * len(self.paths_list_truck)))
        rh_dar_departure = csr_matrix((self.num_assign_interval * len(self.observed_links_curb), self.num_assign_interval * len(self.paths_list_rh)))

        if self.config['use_truck_curb_dep']:
            # truck departure DAR
            raw_truck_dar_departure = dta.get_truck_dar_departure_matrix_sep(np.arange(0, self.num_loading_interval, self.ass_freq), 
                                                    np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq)
            truck_dar_departure = self._massage_raw_dar(raw_truck_dar_departure, self.ass_freq, f_truck, self.num_assign_interval, self.paths_list_truck, self.observed_links_curb)
        
        if self.config['use_rh_curb_dep']:
            # RH departure DAR
            raw_rh_dar_departure = dta.get_rh_dar_departure_matrix(np.arange(0, self.num_loading_interval, self.ass_freq), 
                                                    np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq)
            rh_dar_departure = self._massage_raw_dar(raw_rh_dar_departure, self.ass_freq, f_rh, self.num_assign_interval, self.paths_list_rh, self.observed_links_curb)
        
        return (truck_dar_departure, rh_dar_departure)

    def _massage_raw_dar(self, raw_dar, ass_freq, f, num_assign_interval, paths_list, observed_links):
        num_e_path = len(paths_list)
        num_e_link = len(observed_links)
        small_assign_freq = ass_freq * self.nb.config.config_dict['DTA']['unit_time'] / 60
        raw_dar = raw_dar[(raw_dar[:, 1] < self.num_assign_interval * small_assign_freq) & (raw_dar[:, 3] < self.num_loading_interval), :]

        link_seq = (np.array(list(map(lambda x: observed_links.index(x), raw_dar[:, 2].astype(int))))
                    + (raw_dar[:, 3] / ass_freq).astype(int) * num_e_link).astype(int)
        path_seq = (raw_dar[:, 0].astype(int) + (raw_dar[:, 1] / small_assign_freq).astype(int) * num_e_path).astype(int)
        p = raw_dar[:, 4] / f[path_seq]
        mat = coo_matrix((p, (link_seq, path_seq)), shape=(num_assign_interval * num_e_link, num_assign_interval * num_e_path))
        mat = mat.tocsr()

        # change element in mat to be 1 if it is larger than 1

        return mat   
    
    def init_demand_vector_car(self, num_assign_interval, num_col, scale=1):
        # uniform
        d = np.random.rand(num_assign_interval * num_col) * scale
        return d
    
    # Jiachao added for different init_demand for truck path set (stop/non-stop)
    def init_demand_vector_truck(self, num_assign_interval, num_col_nonstop, num_col_stop, scale_nonstop, scale_stop):
        d_nonstop = np.random.rand(num_assign_interval * num_col_nonstop) * scale_nonstop
        d_stop = np.ones(num_assign_interval * num_col_stop) * scale_stop
        d_nonstop = d_nonstop.reshape(num_assign_interval, num_col_nonstop) 
        d_stop = d_stop.reshape(num_assign_interval, num_col_stop)
        d = np.hstack((d_nonstop, d_stop))
        assert(d.shape[0] == num_assign_interval)
        assert(d.shape[1] == (d_nonstop.shape[1] + d_stop.shape[1]))
        d = d.flatten()
        return d

    def init_demand_vector_rh(self, num_assign_interval, num_col, scale=1):
        # uniform
        d = np.ones(num_assign_interval * num_col) * scale
        return d

    def init_path_flow(self, car_scale, truck_scale, rh_scale):
        f_car = self.init_demand_vector_car(self.num_assign_interval, self.num_path, car_scale) 
        f_truck = self.init_demand_vector_truck(self.num_assign_interval, self.num_path, self.num_path_curb_rh, truck_scale, rh_scale) 
        f_rh = self.init_demand_vector_rh(self.num_assign_interval, self.num_path_curb_rh, rh_scale)
        return f_car, f_truck, f_rh
    
    def _get_one_data(self, j):
        assert (self.num_data > j)
        one_data_dict = dict()

        if self.config['use_car_link_flow']:
            one_data_dict['car_link_flow'] = self.data_dict['car_link_flow'][j]
        
        if self.config['use_truck_link_flow']:
            one_data_dict['truck_link_flow'] = self.data_dict['truck_link_flow'][j]
        
        if self.config['use_rh_link_flow']:
            one_data_dict['rh_link_flow'] = self.data_dict['rh_link_flow'][j]

        if self.config['use_car_link_tt']:
            one_data_dict['car_link_tt'] = self.data_dict['car_link_tt'][j]
        
        if self.config['use_truck_link_tt']:
            one_data_dict['truck_link_tt'] = self.data_dict['truck_link_tt'][j]
        
        if self.config['car_count_agg']:
            one_data_dict['car_count_agg_L'] = self.car_count_agg_L_list[j]
        
        if self.config['truck_count_agg']:
            one_data_dict['truck_count_agg_L'] = self.truck_count_agg_L_list[j]
        
        if self.config['rh_count_agg']:
            one_data_dict['rh_count_agg_L'] = self.truck_count_agg_L_list[j]
        
        # truck and rh curb agg matrix N and H
        if self.config['truck_curb_agg']:
            one_data_dict['truck_curb_agg_N'] = self.truck_curb_agg_N_list[j]
            one_data_dict['truck_curb_agg_H'] = self.truck_curb_agg_H_list[j]

        if self.config['rh_curb_agg']:
            one_data_dict['rh_curb_agg_N'] = self.rh_curb_agg_N_list[j]
            one_data_dict['rh_curb_agg_H'] = self.rh_curb_agg_H_list[j]
        
        # truck and rh curb arrival and departure counts
        if self.config['use_truck_curb_arr'] and self.config['use_rh_curb_arr']:
            one_data_dict['truck_curb_inflow'] = self.data_dict['truck_curb_inflow'][j]
            one_data_dict['rh_curb_inflow'] = self.data_dict['rh_curb_inflow'][j]
        
        if self.config['use_truck_curb_dep'] and self.config['use_rh_curb_dep']:
            one_data_dict['truck_curb_outflow'] = self.data_dict['truck_curb_outflow'][j] 
            one_data_dict['rh_curb_outflow'] = self.data_dict['rh_curb_outflow'][j]
        
        return one_data_dict

    def print_separate_accuracy(self, loss_dict):
        tmp_str = ""
        for loss_type, loss_value in loss_dict.items():
            tmp_str += loss_type + ": " + str(np.round(loss_value, 2)) + "|"
        return tmp_str
    
    def save_simulation_input_files_curb(self, folder_path, f_car=None, f_truck=None, f_rh=None):

        if not os.path.exists(folder_path):
            os.mkdir(folder_path)

        # modify demand based on input path flows
        if (f_car is not None) and (f_truck is not None) and (f_rh is not None):

            # use new function in MNM_mcnb_curb_separate.py
            self.nb.update_demand_path_curb(f_car, f_truck, f_rh)
   
        self.nb.config.config_dict['DTA']['total_interval'] = self.num_loading_interval  # if only count data is used

        self.nb.config.config_dict['DTA']['routing_type'] = 'Biclass_Hybrid_curb'

        # no output files saved from DNL
        self.nb.config.config_dict['STAT']['rec_volume'] = 1
        self.nb.config.config_dict['STAT']['volume_load_automatic_rec'] = 0
        self.nb.config.config_dict['STAT']['volume_record_automatic_rec'] = 0
        self.nb.config.config_dict['STAT']['rec_tt'] = 1
        self.nb.config.config_dict['STAT']['tt_load_automatic_rec'] = 0
        self.nb.config.config_dict['STAT']['tt_record_automatic_rec'] = 0

        # save modified files in new_folder
        self.nb.dump_to_folder(folder_path)

    def _run_simulation_curb(self, f_car, f_truck, f_rh, counter, show_loading = True):
        # create a new_folder with a unique name
        hash1 = hashlib.sha1()
        hash1.update((str(time.time()) + str(counter)).encode('utf-8'))
        new_folder = hash1.hexdigest()
        self.save_simulation_input_files_curb(new_folder, f_car, f_truck, f_rh)

        # invoke MNMAPI
        a = MNMAPI.Mcdta_Api()
        # read all files in new_folder
        a.initialize_curb_sep(new_folder)
        
        # register links separately
        a.register_links_count(self.observed_links_count) # m_link_vec_links
        a.register_links_tt(self.observed_links_tt) # m_link_vec_tt
        a.register_links_curb(self.observed_links_curb) # m_link_vec_curb

        # register paths
        a.register_paths_car(self.paths_list_car) # m_path_vec, m_path_set
        a.register_paths_truck(self.paths_list_truck) # m_path_vec_truck, m_path_set_truck
        a.register_paths_rh(self.paths_list_rh) # m_path_vec_rh, m_path_set_rh

        a.install_cc_separate_with_trees_curb()

        # run DNL
        if show_loading:
            a.run_whole_curb()
        else:
            a.run_whole_curb_false()

        # delete new_folder and all files and subdirectories below it.
        shutil.rmtree(new_folder)

        return a
    
    # grad = - L^T (link_flow_observation - link_flow_estimated) 
    # x_e = link_flow_estimated
    def _compute_count_loss_grad_on_truck_link_flow(self, dta, one_data_dict):
        link_flow_array = one_data_dict['truck_link_flow']
        x_e = dta.get_link_truck_inflow_sep(np.arange(0, self.num_loading_interval, self.ass_freq), 
                                        np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq).flatten(order='F')
        if self.config['truck_count_agg']:
            x_e = one_data_dict['truck_count_agg_L'].dot(x_e)
        discrepancy = np.nan_to_num(link_flow_array - x_e)
        grad = - discrepancy
        if self.config['truck_count_agg']:
            grad = one_data_dict['truck_count_agg_L'].T.dot(grad)
        return grad, x_e
    
    def _compute_count_loss_grad_on_car_rh_link_flow(self, dta, one_data_dict):
        link_flow_array = one_data_dict['car_link_flow']
        x_e = dta.get_link_car_inflow_sep(np.arange(0, self.num_loading_interval, self.ass_freq), 
                                      np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq).flatten(order='F') + \
              dta.get_link_rh_inflow_sep(np.arange(0, self.num_loading_interval, self.ass_freq), 
                                      np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq).flatten(order='F')
        if self.config['car_count_agg']:
            x_e = one_data_dict['car_count_agg_L'].dot(x_e)
        discrepancy = np.nan_to_num(link_flow_array - x_e)
        grad_car = - discrepancy
        grad_rh = - discrepancy
        if self.config['car_count_agg']:
            grad_car = one_data_dict['car_count_agg_L'].T.dot(grad_car)
            grad_rh = one_data_dict['car_count_agg_L'].T.dot(grad_rh) 
        return grad_car, grad_rh, x_e

    def _compute_tt_loss_grad_on_car_link_tt(self, dta, one_data_dict):
        tt_e = dta.get_car_link_tt_sep(np.arange(0, self.num_loading_interval, self.ass_freq)).flatten(order='F')
        tt_free = np.tile(dta.get_car_link_fftt(self.observed_links_tt), (self.num_assign_interval))
        tt_e = np.maximum(tt_e, tt_free)
        tt_o = np.maximum(one_data_dict['car_link_tt'], tt_free) 
        discrepancy = np.nan_to_num(tt_o - tt_e)
        grad = -discrepancy
        return -grad, tt_e

    def _compute_tt_loss_grad_on_truck_link_tt(self, dta, one_data_dict):
        tt_e = dta.get_truck_link_tt_sep(np.arange(0, self.num_loading_interval, self.ass_freq)).flatten(order='F')
        tt_free = np.tile(dta.get_truck_link_fftt(self.observed_links_tt), (self.num_assign_interval))
        tt_e = np.maximum(tt_e, tt_free)
        tt_o = np.maximum(one_data_dict['truck_link_tt'], tt_free)
        discrepancy = np.nan_to_num(tt_o - tt_e)
        grad = -discrepancy
        return -grad, tt_e

    def _compute_tt_loss_grad_on_rh_link_tt(self, dta, one_data_dict):
        tt_e = dta.get_car_link_tt_sep(np.arange(0, self.num_loading_interval, self.ass_freq)).flatten(order='F')
        tt_free = np.tile(dta.get_car_link_fftt(self.observed_links_tt), (self.num_assign_interval))
        tt_e = np.maximum(tt_e, tt_free)
        tt_o = np.maximum(one_data_dict['car_link_tt'], tt_free) 
        discrepancy = np.nan_to_num(tt_o - tt_e)
        grad = -discrepancy
        return -grad, tt_e

    # curb arrival and departure grad
    def _compute_curb_arr_loss_grad_on_truck(self, dta, one_data_dict):
        truck_arrival_e = dta.get_link_truck_curb_inflow_sep(np.arange(0, self.num_loading_interval, self.ass_freq), 
                                                         np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq).flatten(order='F')
        truck_arrival_o = one_data_dict['truck_curb_inflow']

        if self.config['truck_curb_agg']:
            truck_arrival_e = one_data_dict['truck_curb_agg_N'].dot(truck_arrival_e) # N is for arrival (identity matrix with dimension of link_curb)
            
        discrepancy_arr = np.nan_to_num(truck_arrival_o - truck_arrival_e)
        
        grad_arr = - discrepancy_arr

        if self.config['truck_curb_agg']:
            grad_arr = one_data_dict['truck_curb_agg_N'].T.dot(grad_arr)

        return grad_arr, truck_arrival_e
    
    def _compute_curb_dep_loss_grad_on_truck(self, dta, one_data_dict):
        truck_departure_e = dta.get_link_truck_curb_outflow_sep(np.arange(0, self.num_loading_interval, self.ass_freq), 
                                                            np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq).flatten(order='F')
        truck_departure_o = one_data_dict['truck_curb_outflow']

        if self.config['truck_curb_agg']:
            truck_departure_e = one_data_dict['truck_curb_agg_H'].dot(truck_departure_e) # H is for departure (identity matrix with dimension of link_curb)
        
        discrepancy_dep = np.nan_to_num(truck_departure_o - truck_departure_e)

        grad_dep = - discrepancy_dep

        if self.config['truck_curb_agg']:
            grad_dep = one_data_dict['truck_curb_agg_H'].T.dot(grad_dep) 

        return grad_dep, truck_departure_e
    
    def _compute_curb_arr_loss_grad_on_rh(self, dta, one_data_dict):
        rh_arrival_e = dta.get_link_rh_curb_inflow_sep(np.arange(0, self.num_loading_interval, self.ass_freq), 
                                                   np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq).flatten(order='F')
        rh_arrival_o = one_data_dict['rh_curb_inflow']
        
        if self.config['rh_curb_agg']:
            rh_arrival_e = one_data_dict['rh_curb_agg_N'].dot(rh_arrival_e) # N is for arrival (identity matrix with dimension of link_curb)
            
        discrepancy_arr = np.nan_to_num(rh_arrival_o - rh_arrival_e) 

        grad_arr = - discrepancy_arr

        if self.config['rh_curb_agg']:
            grad_arr = one_data_dict['rh_curb_agg_N'].T.dot(grad_arr)
            
        return grad_arr, rh_arrival_e
    
    def _compute_curb_dep_loss_grad_on_rh(self, dta, one_data_dict):
        rh_departure_e = dta.get_link_rh_curb_outflow_sep(np.arange(0, self.num_loading_interval, self.ass_freq), 
                                                      np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq).flatten(order='F')
        
        rh_departure_o = one_data_dict['rh_curb_outflow']

        if self.config['rh_curb_agg']:
            rh_departure_e = one_data_dict['rh_curb_agg_H'].dot(rh_departure_e) # H is for departure (identity matrix with dimension of link_curb)
        
        discrepancy_dep = np.nan_to_num(rh_departure_o - rh_departure_e)

        grad_dep = - discrepancy_dep

        if self.config['rh_curb_agg']:
            grad_dep = one_data_dict['rh_curb_agg_H'].T.dot(grad_dep)

        return grad_dep, rh_departure_e

    # TODO revise this function 04/07/2025
    def compute_path_flow_grad_and_loss(self, one_data_dict, f_car, f_truck, f_rh, counter=0):
        # run one DNL
        dta = self._run_simulation_curb(f_car, f_truck, f_rh, counter, show_loading = False)

        x_e_car, x_e_truck, tt_e_car, tt_e_truck = None, None, None, None

        truck_arrival_e, truck_departure_e, rh_arrival_e, rh_departure_e = None, None, None, None

        f_car_grad = np.zeros(self.num_path * self.num_assign_interval)
        f_truck_grad = np.zeros(self.num_path_curb * self.num_assign_interval)
        f_rh_grad = np.zeros(self.num_path_curb_rh * self.num_assign_interval)

        #-------- Count gradients --------#
        if self.config['use_car_link_flow'] and self.config['use_truck_link_flow']:
            car_grad = np.zeros(len(self.observed_links_count) * self.num_assign_interval)
            truck_grad = np.zeros(len(self.observed_links_count) * self.num_assign_interval)
            rh_grad = np.zeros(len(self.observed_links_count) * self.num_assign_interval)

            # count DAR matrix
            (car_dar, truck_dar, rh_dar) = self.get_dar_count(dta, f_car, f_truck, f_rh)

            # compute gradient for car and rh
            grad_car, grad_rh, x_e_car = self._compute_count_loss_grad_on_car_rh_link_flow(dta, one_data_dict)
            car_grad += self.config['link_car_flow_weight'] * grad_car # = -w L^T (link_flow_observation - link_flow_estimated) 
            rh_grad += self.config['rh_curb_arr_weight'] * grad_rh

            # compute gradient for truck
            grad, x_e_truck = self._compute_count_loss_grad_on_truck_link_flow(dta, one_data_dict)
            truck_grad += self.config['link_truck_flow_weight'] * grad

            # chain rule
            f_car_grad = car_dar.T.dot(car_grad) # -w rho^T L^T (link_flow_observation - link_flow_estimated) 
            f_truck_grad = truck_dar.T.dot(truck_grad)
            f_rh_grad = rh_dar.T.dot(rh_grad)

        #-------- Travel time gradients --------#
        if self.config['use_car_link_tt'] and self.config['use_truck_link_tt']:
            car_grad = np.zeros(len(self.observed_links_tt) * self.num_assign_interval)
            truck_grad = np.zeros(len(self.observed_links_tt) * self.num_assign_interval)
            rh_grad = np.zeros(len(self.observed_links_tt) * self.num_assign_interval)

            # travel time DAR matrix
            (car_dar_tt, truck_dar_tt, rh_dar_tt) = self.get_dar_tt(dta, f_car, f_truck, f_rh)

            # compute gradient for car
            grad, tt_e_car = self._compute_tt_loss_grad_on_car_link_tt(dta, one_data_dict)
            car_grad += self.config['link_car_tt_weight'] * grad
            f_car_grad += car_dar_tt.T.dot(car_grad)

            # compute gradient for rh
            grad, _ = self._compute_tt_loss_grad_on_rh_link_tt(dta, one_data_dict)
            rh_grad += self.config['link_car_tt_weight'] * grad
            f_rh_grad += rh_dar_tt.T.dot(rh_grad)

            # compute gradient for truck
            grad, tt_e_truck = self._compute_tt_loss_grad_on_truck_link_tt(dta, one_data_dict)
            truck_grad += self.config['link_truck_tt_weight'] * grad
            f_truck_grad += truck_dar_tt.T.dot(truck_grad)
            

        #-------- Curb gradients --------#
        # curb arrival
        if self.config['use_truck_curb_arr'] and self.config['use_rh_curb_arr']:
            # get dar for curb-related count
            truck_dar_arr, rh_dar_arr = self.get_dar_arrival(dta, f_truck, f_rh)

            # compute gradient for truck and chain rule
            truck_grad_arr, truck_arrival_e = self._compute_curb_arr_loss_grad_on_truck(dta, one_data_dict)
            f_truck_grad += truck_dar_arr.T.dot(self.config['truck_curb_arr_weight'] * truck_grad_arr)
        
            # compute gradient for rh and chain rule
            rh_grad_arr, rh_arrival_e = self._compute_curb_arr_loss_grad_on_rh(dta, one_data_dict)
            f_rh_grad += rh_dar_arr.T.dot(self.config['rh_curb_arr_weight'] * rh_grad_arr)

        # curb departure
        if self.config['use_truck_curb_dep'] and self.config['use_rh_curb_dep']:
            # get dar for curb-related count
            truck_dar_dep, rh_dar_dep = self.get_dar_departure(dta, f_truck, f_rh)

            # compute gradient for truck and chain rule
            truck_grad_dep, truck_departure_e = self._compute_curb_dep_loss_grad_on_truck(dta, one_data_dict)
            f_truck_grad += truck_dar_dep.T.dot(self.config['truck_curb_dep_weight'] * truck_grad_dep)
        
            # compute gradient for rh and chain rule
            rh_grad_dep, rh_departure_e = self._compute_curb_dep_loss_grad_on_rh(dta, one_data_dict)
            f_rh_grad += rh_dar_dep.T.dot(self.config['rh_curb_dep_weight'] * rh_grad_dep)

        # compute loss
        loss_dict = dict()

        # count loss
        if self.config['use_car_link_flow']:
            loss = self.config['link_car_flow_weight'] * np.linalg.norm(np.nan_to_num(x_e_car - one_data_dict['car_link_flow']))
            loss_dict['car_count_loss'] = loss

        if self.config['use_truck_link_flow']:
            loss = self.config['link_truck_flow_weight'] * np.linalg.norm(np.nan_to_num(x_e_truck - one_data_dict['truck_link_flow']))
            loss_dict['truck_count_loss'] = loss

        # travel time loss
        if self.config['use_car_link_tt']:
            loss = self.config['link_car_tt_weight'] * np.linalg.norm(np.nan_to_num(tt_e_car - one_data_dict['car_link_tt']))
            loss_dict['car_tt_loss'] = loss

        if self.config['use_truck_link_tt']:
            loss = self.config['link_truck_tt_weight'] * np.linalg.norm(np.nan_to_num(tt_e_truck - one_data_dict['truck_link_tt']))
            loss_dict['truck_tt_loss'] = loss

        # loss for truck and RH curb arrival and departures
        if self.config['use_truck_curb_arr'] and self.config['use_rh_curb_arr']:
            loss_truck_arr = self.config['truck_curb_arr_weight'] * \
                                np.linalg.norm(np.nan_to_num(truck_arrival_e - one_data_dict['truck_curb_inflow']))
        
            loss_rh_arr = self.config['rh_curb_arr_weight'] * \
                                np.linalg.norm(np.nan_to_num(rh_arrival_e - one_data_dict['rh_curb_inflow']))

            
            loss_dict['truck_arr_loss'] = loss_truck_arr
            loss_dict['rh_arr_loss'] = loss_rh_arr

        if self.config['use_truck_curb_dep'] and self.config['use_rh_curb_dep']:
            loss_truck_dep = self.config['truck_curb_dep_weight'] * \
                                np.linalg.norm(np.nan_to_num(truck_departure_e - one_data_dict['truck_curb_outflow']))

            loss_rh_dep = self.config['rh_curb_dep_weight'] * \
                                np.linalg.norm(np.nan_to_num(rh_departure_e - one_data_dict['rh_curb_outflow']))

            loss_dict['truck_dep_loss'] = loss_truck_dep
            loss_dict['rh_dep_loss'] = loss_rh_dep

        total_loss = 0.0
        for _, loss_value in loss_dict.items():
            total_loss += loss_value

        return f_car_grad, f_truck_grad, f_rh_grad, total_loss, loss_dict, dta, x_e_car, x_e_truck, tt_e_car, tt_e_truck, truck_arrival_e, truck_departure_e, rh_arrival_e, rh_departure_e

    def estimate_path_flow_pytorch_curb(self, 
                                        car_step_size = 0.1, 
                                        truck_step_size = 0.1, 
                                        rh_step_size = 0.1,
                                        link_car_flow_weight = 1, 
                                        link_truck_flow_weight = 1, 
                                        link_car_tt_weight = 1, 
                                        link_truck_tt_weight = 1,
                                        truck_curb_arr_weight = 1,
                                        truck_curb_dep_weight = 1,
                                        rh_curb_arr_weight = 1,
                                        rh_curb_dep_weight = 1, 
                                        max_epoch=100, 
                                        algo='NAdam', 
                                        normalized_by_scale = True,
                                        car_init_scale = 0.5,
                                        truck_init_scale = 0.1, 
                                        rh_init_scale = 0.1,
                                        store_folder=None, 
                                        use_file_as_init=None,
                                        starting_epoch=0):

        self.store_folder = store_folder

        if np.isscalar(link_car_flow_weight):
            link_car_flow_weight = np.ones(max_epoch, dtype=bool) * link_car_flow_weight
        assert(len(link_car_flow_weight) == max_epoch)

        if np.isscalar(link_truck_flow_weight):
            link_truck_flow_weight = np.ones(max_epoch, dtype=bool) * link_truck_flow_weight
        assert(len(link_truck_flow_weight) == max_epoch)

        if np.isscalar(link_car_tt_weight):
            link_car_tt_weight = np.ones(max_epoch, dtype=bool) * link_car_tt_weight
        assert(len(link_car_tt_weight) == max_epoch)

        if np.isscalar(link_truck_tt_weight):
            link_truck_tt_weight = np.ones(max_epoch, dtype=bool) * link_truck_tt_weight
        assert(len(link_truck_tt_weight) == max_epoch)

        # add weights of truck arr and dep
        if np.isscalar(truck_curb_arr_weight):
            truck_curb_arr_weight = np.ones(max_epoch, dtype = bool) * truck_curb_arr_weight
        assert(len(truck_curb_arr_weight) == max_epoch)

        if np.isscalar(truck_curb_dep_weight):
            truck_curb_dep_weight = np.ones(max_epoch, dtype = bool) * truck_curb_dep_weight
        assert(len(truck_curb_dep_weight) == max_epoch)
        
        # add weights of rh arr and dep
        if np.isscalar(rh_curb_arr_weight):
            rh_curb_arr_weight = np.ones(max_epoch, dtype = bool) * rh_curb_arr_weight
        assert(len(rh_curb_arr_weight) == max_epoch)

        if np.isscalar(rh_curb_dep_weight):
            rh_curb_dep_weight = np.ones(max_epoch, dtype = bool) * rh_curb_dep_weight
        assert(len(rh_curb_dep_weight) == max_epoch)

        loss_list = list()
        best_epoch = starting_epoch
        best_f_car_driving, best_f_truck_driving, best_f_rh_driving = None, None, None
        best_x_e_car, best_x_e_truck, best_tt_e_car, best_tt_e_truck = None, None, None, None
        best_truck_arrival_e, best_truck_departure_e, best_rh_arrival_e, best_rh_departure_e = None, None, None, None

        if use_file_as_init is None:
            (f_car, f_truck, f_rh) = self.init_path_flow(car_scale = car_init_scale, truck_scale = truck_init_scale, rh_scale = rh_init_scale)
        else:
            loss, loss_dict, loss_list, best_epoch, \
            best_f_car_driving, best_f_truck_driving, best_f_rh_driving, \
            best_x_e_car, best_x_e_truck, best_tt_e_car, best_tt_e_truck, \
            best_truck_arrival_e, best_truck_departure_e, best_rh_arrival_e, best_rh_departure_e\
                = pickle.load(open(use_file_as_init, 'rb'))
            
            f_car, f_truck, f_rh = best_f_car_driving, best_f_truck_driving, best_f_rh_driving

        if normalized_by_scale:
            f_car_tensor = torch.from_numpy(f_car / np.maximum(car_init_scale, 1e-6))
            f_truck_tensor = torch.from_numpy(f_truck / np.maximum(truck_init_scale, 1e-6))
            f_rh_tensor = torch.from_numpy(f_rh / np.maximum(rh_init_scale, 1e-6))
        else:
            f_car_tensor = torch.from_numpy(f_car)
            f_truck_tensor = torch.from_numpy(f_truck)
            f_rh_tensor = torch.from_numpy(f_rh)

        f_car_tensor.requires_grad = True
        f_truck_tensor.requires_grad = True
        f_rh_tensor.requires_grad = True

        params = [
            {'params': f_car_tensor, 'lr': car_step_size},
            {'params': f_truck_tensor, 'lr': truck_step_size},
            {'params': f_rh_tensor, 'lr': rh_step_size},
        ]

        algo_dict = {
            "SGD": torch.optim.SGD,
            "NAdam": torch.optim.NAdam,
            "Adam": torch.optim.Adam,
            "Adamax": torch.optim.Adamax,
            "AdamW": torch.optim.AdamW,
            "RAdam": torch.optim.RAdam,
            "Adagrad": torch.optim.Adagrad,
            "Adadelta": torch.optim.Adadelta
        }
        optimizer = algo_dict[algo](params)
        
        # epoch iteration loop
        for i in range(max_epoch):
      
            seq = np.random.permutation(self.num_data)
            loss = float(0)

            loss_dict = {'car_count_loss': 0.0, 'truck_count_loss': 0.0, 'car_tt_loss': 0.0, 'truck_tt_loss': 0.0, 'truck_arr_loss': 0.0, 'truck_dep_loss': 0.0, 'rh_arr_loss': 0.0, 'rh_dep_loss': 0.0}

            self.config['link_car_flow_weight'] = link_car_flow_weight[i]
            self.config['link_truck_flow_weight'] = link_truck_flow_weight[i]
            
            self.config['link_car_tt_weight'] = link_car_tt_weight[i]
            self.config['link_truck_tt_weight'] = link_truck_tt_weight[i]

            self.config['truck_curb_arr_weight'] = truck_curb_arr_weight[i]
            self.config['truck_curb_dep_weight'] = truck_curb_dep_weight[i]

            self.config['rh_curb_arr_weight'] = rh_curb_arr_weight[i]
            self.config['rh_curb_dep_weight'] = rh_curb_dep_weight[i] 

            # data iteration loop
            for j in seq:

                # one data sample
                one_data_dict = self._get_one_data(j)

                # main function for gradient and loss
                car_grad, truck_grad, rh_grad, tmp_loss, tmp_loss_dict, _dta, \
                x_e_car, x_e_truck, tt_e_car, tt_e_truck, \
                truck_arrival_e, truck_departure_e, rh_arrival_e, rh_departure_e = self.compute_path_flow_grad_and_loss(one_data_dict, f_car, f_truck, f_rh)

                optimizer.zero_grad()

                if normalized_by_scale:
                    f_car_tensor.grad = torch.from_numpy(car_grad * car_init_scale)
                    f_truck_tensor.grad = torch.from_numpy(truck_grad * truck_init_scale)
                    f_rh_tensor.grad = torch.from_numpy(rh_grad * rh_init_scale)
                else:
                    f_car_tensor.grad = torch.from_numpy(car_grad)
                    f_truck_tensor.grad = torch.from_numpy(truck_grad)
                    f_rh_tensor.grad = torch.from_numpy(rh_grad)

                optimizer.step()

                car_grad, truck_grad, rh_grad = 0, 0, 0
                optimizer.zero_grad()

                if normalized_by_scale:
                    f_car = f_car_tensor.data.cpu().numpy() * car_init_scale
                    f_truck = f_truck_tensor.data.cpu().numpy() * truck_init_scale
                    f_rh = f_rh_tensor.data.cpu().numpy() * rh_init_scale
                else:
                    f_car = f_car_tensor.data.cpu().numpy()
                    f_truck = f_truck_tensor.data.cpu().numpy()
                    f_rh = f_rh_tensor.data.cpu().numpy()
            
                f_car = np.maximum(f_car, 0.0001)
                # f_truck = np.maximum(f_truck, 0.01)
                # TODO diffrentiate the min value for stop and nonstop path flow
                # tricks here
                f_truck_temp = f_truck.reshape(self.num_assign_interval, self.num_path_curb)
                f_truck_nonstop = f_truck_temp[:, :self.num_path]
                f_truck_stop = f_truck_temp[:, self.num_path:]
                f_truck_nonstop = np.maximum(f_truck_nonstop, 0.0001)
                f_truck_stop = np.maximum(f_truck_stop, 0.5) # 0.3
                f_truck = np.hstack((f_truck_nonstop, f_truck_stop))
                f_truck = f_truck.flatten()

                f_rh = np.maximum(f_rh, 0.5) # 0.1
                f_rh = np.minimum(f_rh, 6.0)

                loss += tmp_loss / float(self.num_data)
                for loss_type, loss_value in tmp_loss_dict.items():
                    loss_dict[loss_type] += loss_value / float(self.num_data)

            print("Epoch:", starting_epoch + i, "Loss:", loss, self.print_separate_accuracy(loss_dict))
            loss_list.append([loss, loss_dict])
                    
            if (best_epoch == 0) or (loss_list[best_epoch][0] > loss_list[-1][0]):
                best_epoch = starting_epoch + i
                best_f_car_driving, best_f_truck_driving, best_f_rh_driving = f_car, f_truck, f_rh
                best_x_e_car, best_x_e_truck, best_tt_e_car, best_tt_e_truck = x_e_car, x_e_truck, tt_e_car, tt_e_truck
                best_truck_arrival_e, best_truck_departure_e, best_rh_arrival_e, best_rh_departure_e = truck_arrival_e, truck_departure_e, rh_arrival_e, rh_departure_e
                
                if store_folder is not None:
                    self.save_simulation_input_files_curb(os.path.join(store_folder, 'input_files_estimate_path_flow'), 
                                                     best_f_car_driving, best_f_truck_driving, best_f_rh_driving)
            
            if store_folder is not None:
                pickle.dump((loss, loss_dict, loss_list, best_epoch, f_car, f_truck, f_rh,
                            x_e_car, x_e_truck, tt_e_car, tt_e_truck, truck_arrival_e, truck_departure_e, rh_arrival_e, rh_departure_e), 
                            open(os.path.join(store_folder, str(starting_epoch + i) + '_iteration.pickle'), 'wb'))

        print("Best loss at Epoch:", best_epoch, "Loss:", loss_list[best_epoch][0], self.print_separate_accuracy(loss_list[best_epoch][1]))
        return best_f_car_driving, best_f_truck_driving, best_f_rh_driving, \
                best_x_e_car, best_x_e_truck, best_tt_e_car, best_tt_e_truck, \
                best_truck_arrival_e, best_truck_departure_e, best_rh_arrival_e, best_rh_departure_e, loss_list
