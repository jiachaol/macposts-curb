import os
import numpy as np
import pandas as pd
import hashlib
import time
import shutil
from scipy.sparse import coo_matrix, csr_matrix
import pickle
import multiprocessing as mp
import torch

import MNMAPI

class MCDODE():
    def __init__(self, nb, config):
        self.config = config
        self.nb = nb
        self.num_assign_interval = nb.config.config_dict['DTA']['max_interval']
        self.ass_freq = nb.config.config_dict['DTA']['assign_frq']
        self.num_link = nb.config.config_dict['DTA']['num_of_link']
        self.num_path = nb.config.config_dict['FIXED']['num_path']
        self.num_path_curb = nb.config.config_dict['FIXED']['num_path_curb'] # Done

        self.num_loading_interval = self.num_assign_interval * self.ass_freq
        self.data_dict = dict()
        self.num_data = self.config['num_data']
        self.observed_links = self.config['observed_links']

        self.paths_list = self.config['paths_list']
        self.paths_list_cc = self.config['paths_list_cc']
        
        self.car_count_agg_L_list = None
        self.truck_count_agg_L_list = None

        self.car_curb_agg_N_list = None
        self.car_curb_agg_H_list = None

        self.truck_curb_agg_N_list = None
        self.truck_curb_agg_H_list = None

        self.rh_curb_agg_N_list = None
        self.rh_curb_agg_H_list = None

        self.store_folder = None

        assert (len(self.paths_list) == self.num_path)
        assert (len(self.paths_list_cc) == self.num_path_curb)
    
    def _add_car_link_flow_data(self, link_flow_df_list):
        # assert (self.config['use_car_link_flow'])
        assert (self.num_data == len(link_flow_df_list))
        self.data_dict['car_link_flow'] = link_flow_df_list
    
    def _add_rh_link_flow_data(self, link_flow_df_list):
        assert (self.num_data == len(link_flow_df_list))
        self.data_dict['rh_link_flow'] = link_flow_df_list

    def _add_truck_link_flow_data(self, link_flow_df_list):
        # assert (self.config['use_truck_link_flow'])
        assert (self.num_data == len(link_flow_df_list))
        self.data_dict['truck_link_flow'] = link_flow_df_list

    def _add_car_link_tt_data(self, link_spd_df_list):
        # assert (self.config['use_car_link_tt'])
        assert (self.num_data == len(link_spd_df_list))
        self.data_dict['car_link_tt'] = link_spd_df_list

    def _add_truck_link_tt_data(self, link_spd_df_list):
        # assert (self.config['use_truck_link_tt'])
        assert (self.num_data == len(link_spd_df_list))
        self.data_dict['truck_link_tt'] = link_spd_df_list
    
    def _add_link_curb_data_car(self, car_link_curb_arr_list, car_link_curb_dep_list):
        assert (self.num_data == len(car_link_curb_arr_list))
        assert (self.num_data == len(car_link_curb_dep_list))
        self.data_dict['car_curb_inflow'] = car_link_curb_arr_list 
        self.data_dict['car_curb_outflow'] = car_link_curb_dep_list

    def _add_link_curb_data_truck(self, truck_link_curb_arr_list, truck_link_curb_dep_list):
        assert (self.num_data == len(truck_link_curb_arr_list))
        assert (self.num_data == len(truck_link_curb_dep_list))
        self.data_dict['truck_curb_inflow'] = truck_link_curb_arr_list 
        self.data_dict['truck_curb_outflow'] = truck_link_curb_dep_list 
        
    def _add_link_curb_data_rh(self, rh_link_curb_arr_list, rh_link_curb_dep_list):
        assert (self.num_data == len(rh_link_curb_arr_list))
        assert (self.num_data == len(rh_link_curb_dep_list))
        self.data_dict['rh_curb_inflow'] = rh_link_curb_arr_list 
        self.data_dict['rh_curb_outflow'] = rh_link_curb_dep_list 

    def add_data(self, data_dict):
        if self.config['car_count_agg']:
            self.car_count_agg_L_list = data_dict['car_count_agg_L_list']

        if self.config['truck_count_agg']:
            self.truck_count_agg_L_list = data_dict['truck_count_agg_L_list']
        
        if self.config['car_curb_agg']:
            self.car_curb_agg_N_list = data_dict['car_curb_agg_N']
            self.car_curb_agg_H_list = data_dict['car_curb_agg_H']

        if self.config['truck_curb_agg']:
            self.truck_curb_agg_N_list = data_dict['truck_curb_agg_N']
            self.truck_curb_agg_H_list = data_dict['truck_curb_agg_H']

        if self.config['rh_curb_agg']:
            self.rh_curb_agg_N_list = data_dict['rh_curb_agg_N']
            self.rh_curb_agg_H_list = data_dict['rh_curb_agg_H']

        if self.config['use_car_link_flow'] or self.config['compute_car_link_flow_loss']:
            self._add_car_link_flow_data(data_dict['car_link_flow'])

        if self.config['use_truck_link_flow'] or self.config['compute_truck_link_flow_loss'] :
            self._add_truck_link_flow_data(data_dict['truck_link_flow'])
        
        if self.config['use_rh_link_flow']:
            self._add_rh_link_flow_data(data_dict['rh_link_flow'])

        if self.config['use_car_link_tt'] or self.config['compute_car_link_tt_loss']:
            self._add_car_link_tt_data(data_dict['car_link_tt'])

        if self.config['use_truck_link_tt'] or self.config['compute_car_link_tt_loss']:
            self._add_truck_link_tt_data(data_dict['truck_link_tt'])

        if self.config['use_car_curb_count'] or self.config['compute_car_curb_count_loss']:
            self._add_link_curb_data_car(data_dict['car_curb_inflow'], data_dict['car_curb_outflow'])

        if self.config['use_truck_curb_count'] or self.config['compute_truck_curb_count_loss']:
            self._add_link_curb_data_truck(data_dict['truck_curb_inflow'], data_dict['truck_curb_outflow'])
        
        if self.config['use_rh_curb_count'] or self.config['compute_rh_curb_count_loss']:
            self._add_link_curb_data_rh(data_dict['rh_curb_inflow'], data_dict['rh_curb_outflow'])
        
        if 'mask_driving_link' in data_dict:
            self.data_dict['mask_driving_link'] = np.tile(data_dict['mask_driving_link'], self.num_assign_interval)
        else:
            self.data_dict['mask_driving_link'] = np.ones(len(self.observed_links) * self.num_assign_interval, dtype=bool)

    # def save_simulation_input_files(self, folder_path, f_car=None, f_truck=None):

    #     if not os.path.exists(folder_path):
    #         os.mkdir(folder_path)

    #     # modify demand based on input path flows
    #     if (f_car is not None) and (f_truck is not None):
    #         self.nb.update_demand_path2(f_car, f_truck)
   
    #     # self.nb.config.config_dict['DTA']['flow_scalar'] = 3
    #     if self.config['use_car_link_tt'] or self.config['use_truck_link_tt']:
    #         self.nb.config.config_dict['DTA']['total_interval'] = self.num_loading_interval # * 2  # hopefully this is sufficient 
    #     else:
    #         self.nb.config.config_dict['DTA']['total_interval'] = self.num_loading_interval  # if only count data is used

    #     self.nb.config.config_dict['DTA']['routing_type'] = 'Biclass_Hybrid'

    #     # no output files saved from DNL
    #     self.nb.config.config_dict['STAT']['rec_volume'] = 1
    #     self.nb.config.config_dict['STAT']['volume_load_automatic_rec'] = 0
    #     self.nb.config.config_dict['STAT']['volume_record_automatic_rec'] = 0
    #     self.nb.config.config_dict['STAT']['rec_tt'] = 1
    #     self.nb.config.config_dict['STAT']['tt_load_automatic_rec'] = 0
    #     self.nb.config.config_dict['STAT']['tt_record_automatic_rec'] = 0
    #     # save modified files in new_folder
    #     self.nb.dump_to_folder(folder_path)

    def get_dar_curb(self, dta, f_car, f_truck, f_rh):
        car_dar = csr_matrix((self.num_assign_interval * len(self.observed_links), self.num_assign_interval * len(self.paths_list)))
        truck_dar = csr_matrix((self.num_assign_interval * len(self.observed_links), self.num_assign_interval * len(self.paths_list_cc)))
        rh_dar = csr_matrix((self.num_assign_interval * len(self.observed_links), self.num_assign_interval * len(self.paths_list_cc)))

        if self.config['use_car_link_flow'] or self.config['use_car_link_tt']:
            # (num_assign_timesteps x num_links x num_path x num_assign_timesteps) x 5
            raw_car_dar = dta.get_car_dar_matrix(np.arange(0, self.num_loading_interval, self.ass_freq), 
                                                np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq)
            # print("raw car dar", raw_car_dar)
            # num_assign_interval * num_e_link, num_assign_interval * num_e_path
            car_dar = self._massage_raw_dar(raw_car_dar, self.ass_freq, f_car, self.num_assign_interval, self.paths_list)
        
        if self.config['use_truck_link_flow'] or self.config['use_truck_link_tt']:
            # (num_assign_timesteps x num_links x num_path x num_assign_timesteps) x 5
            raw_truck_dar = dta.get_truck_dar_matrix(np.arange(0, self.num_loading_interval, self.ass_freq), 
                                                    np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq)
            # num_assign_interval * num_e_link, num_assign_interval * num_e_path
            truck_dar = self._massage_raw_dar(raw_truck_dar, self.ass_freq, f_truck, self.num_assign_interval, self.paths_list_cc)
        
        if self.config['use_rh_link_flow'] or self.config['use_rh_link_tt']:
            # (num_assign_timesteps x num_links x num_path x num_assign_timesteps) x 5
            raw_rh_dar = dta.get_rh_dar_matrix(np.arange(0, self.num_loading_interval, self.ass_freq), 
                                                    np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq)
            # num_assign_interval * num_e_link, num_assign_interval * num_e_path
            rh_dar = self._massage_raw_dar(raw_rh_dar, self.ass_freq, f_rh, self.num_assign_interval, self.paths_list_cc)
        
        # print("dar", car_dar, truck_dar)
        return (car_dar, truck_dar, rh_dar)

    # TODO dar matrix for arrival and departure
    # add car curb arrival 2024
    def get_dar_arrival(self, dta, f_car, f_truck, f_rh):
        car_arr_arrival = csr_matrix((self.num_assign_interval * len(self.observed_links), self.num_assign_interval * len(self.paths_list)))
        truck_dar_arrival = csr_matrix((self.num_assign_interval * len(self.observed_links), self.num_assign_interval * len(self.paths_list_cc)))
        rh_dar_arrival = csr_matrix((self.num_assign_interval * len(self.observed_links), self.num_assign_interval * len(self.paths_list_cc)))
       
        if self.config['use_car_link_curb_arrival']:
            raw_car_dar_arrival = dta.get_car_dar_arrival_matrix(np.arange(0, self.num_loading_interval, self.ass_freq), 
                                                    np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq)
            car_arr_arrival = self._massage_raw_dar(raw_car_dar_arrival, self.ass_freq, f_car, self.num_assign_interval, self.paths_list)

        if self.config['use_truck_link_curb_arrival']:
            raw_truck_dar_arrival = dta.get_truck_dar_arrival_matrix(np.arange(0, self.num_loading_interval, self.ass_freq), 
                                                    np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq)
            truck_dar_arrival = self._massage_raw_dar(raw_truck_dar_arrival, self.ass_freq, f_truck, self.num_assign_interval, self.paths_list_cc)
        
        if self.config['use_rh_link_curb_arrival']:
            raw_rh_dar_arrival = dta.get_rh_dar_arrival_matrix(np.arange(0, self.num_loading_interval, self.ass_freq), 
                                                    np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq)
            rh_dar_arrival = self._massage_raw_dar(raw_rh_dar_arrival, self.ass_freq, f_rh, self.num_assign_interval, self.paths_list_cc)

        return (car_arr_arrival, truck_dar_arrival, rh_dar_arrival)
    
    def get_dar_departure(self, dta, f_car, f_truck, f_rh):
        car_dar_departure = csr_matrix((self.num_assign_interval * len(self.observed_links), self.num_assign_interval * len(self.paths_list)))
        truck_dar_departure = csr_matrix((self.num_assign_interval * len(self.observed_links), self.num_assign_interval * len(self.paths_list_cc)))
        rh_dar_departure = csr_matrix((self.num_assign_interval * len(self.observed_links), self.num_assign_interval * len(self.paths_list_cc)))

        if self.config['use_car_link_curb_departure']:
            raw_car_dar_departure = dta.get_car_dar_departure_matrix(np.arange(0, self.num_loading_interval, self.ass_freq), 
                                                    np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq)
            car_dar_departure = self._massage_raw_dar(raw_car_dar_departure, self.ass_freq, f_car, self.num_assign_interval, self.paths_list)

        if self.config['use_truck_link_curb_departure']:
            raw_truck_dar_departure = dta.get_truck_dar_departure_matrix(np.arange(0, self.num_loading_interval, self.ass_freq), 
                                                    np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq)
            truck_dar_departure = self._massage_raw_dar(raw_truck_dar_departure, self.ass_freq, f_truck, self.num_assign_interval, self.paths_list_cc)
        
        if self.config['use_rh_link_curb_departure']:
            raw_rh_dar_departure = dta.get_rh_dar_departure_matrix(np.arange(0, self.num_loading_interval, self.ass_freq), 
                                                    np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq)
            rh_dar_departure = self._massage_raw_dar(raw_rh_dar_departure, self.ass_freq, f_rh, self.num_assign_interval, self.paths_list_cc)
        
        return (car_dar_departure, truck_dar_departure, rh_dar_departure)

    def _massage_raw_dar(self, raw_dar, ass_freq, f, num_assign_interval, paths_list):
        num_e_path = len(paths_list)
        num_e_link = len(self.observed_links)
        # 15 min
        small_assign_freq = ass_freq * self.nb.config.config_dict['DTA']['unit_time'] / 60

        raw_dar = raw_dar[(raw_dar[:, 1] < self.num_assign_interval * small_assign_freq) & (raw_dar[:, 3] < self.num_loading_interval), :]

        # raw_dar[:, 2]: link no.
        # raw_dar[:, 3]: the count of unit time interval (5s)
        # In Python 3, map() returns an iterable while, in Python 2, it returns a list.
        link_seq = (np.array(list(map(lambda x: self.observed_links.index(x), raw_dar[:, 2].astype(int))))
                    + (raw_dar[:, 3] / ass_freq).astype(int) * num_e_link).astype(int)
        # raw_dar[:, 0]: path no.
        # raw_dar[:, 1]: the count of 1 min interval
        path_seq = (raw_dar[:, 0].astype(int) + (raw_dar[:, 1] / small_assign_freq).astype(int) * num_e_path).astype(int)
        # print(path_seq)
        # raw_dar[:, 4]: flow
        p = raw_dar[:, 4] / f[path_seq]
        # print("Creating the coo matrix", time.time())
        mat = coo_matrix((p, (link_seq, path_seq)), 
                        shape=(num_assign_interval * num_e_link, num_assign_interval * num_e_path))
        # pickle.dump((p, link_seq, path_seq), open('test.pickle', 'wb'))
        # print('converting the csr', time.time())
        mat = mat.tocsr()
        # print('finish converting', time.time())
        return mat   

    def get_ltg(self, dta):
        car_ltg_matrix = csr_matrix((self.num_assign_interval * len(self.observed_links), 
                                             self.num_assign_interval * len(self.paths_list)))
     
        truck_ltg_matrix = csr_matrix((self.num_assign_interval * len(self.observed_links), 
                                               self.num_assign_interval * len(self.paths_list_cc)))
        
        rh_ltg_matrix = csr_matrix((self.num_assign_interval * len(self.observed_links), 
                                               self.num_assign_interval * len(self.paths_list_cc)))

        if self.config['use_car_link_tt']:
            car_ltg_matrix = self._compute_link_tt_grad_on_path_flow_car(dta)
            # if car_ltg_matrix.max() == 0.:
            #     print("car_ltg_matrix is empty!")
            
        if self.config['use_truck_link_tt']:
            truck_ltg_matrix = self._compute_link_tt_grad_on_path_flow_truck(dta)
            # if truck_ltg_matrix.max() == 0.:
            #     print("truck_ltg_matrix is empty!")
        
        if self.config['use_car_link_tt']:
            rh_ltg_matrix = self._compute_link_tt_grad_on_path_flow_rh(dta)
            # if rh_ltg_matrix.max() == 0.:
            #     print("rh_ltg_matrix is empty!")

        return car_ltg_matrix, truck_ltg_matrix, rh_ltg_matrix
    
    def init_demand_vector(self, num_assign_interval, num_col, scale=1):
        # uniform
        d = np.random.rand(num_assign_interval * num_col) * scale

        # Kaiming initialization (not working)
        # d = np.random.normal(0, 1, num_assign_interval * num_col) * scale
        # d *= np.sqrt(2 / len(d))
        # d = np.abs(d)

        # Xavier initialization
        # x = torch.Tensor(num_assign_interval * num_col, 1)
        # d = torch.abs(nn.init.xavier_uniform_(x)).squeeze().data.numpy() * scale
        # d = d.astype(float)
        return d

    def init_path_flow(self, car_scale = 1, truck_scale = 0.1, rh_scale = 1):
        f_car = self.init_demand_vector(self.num_assign_interval, self.num_path, car_scale) 
        f_truck = self.init_demand_vector(self.num_assign_interval, self.num_path_curb, truck_scale) 
        f_rh = self.init_demand_vector(self.num_assign_interval, self.num_path_curb, rh_scale)
        return f_car, f_truck, f_rh
    
    def _compute_link_tt_grad_on_path_flow_car(self, dta):
        # dta.build_link_cost_map() is invoked already
        assign_intervals = np.arange(0, self.num_loading_interval, self.ass_freq)
        num_assign_intervals = len(assign_intervals)

        release_freq = 60
        # this is in terms of 5-s intervals
        release_intervals = np.arange(0, self.num_loading_interval, release_freq // self.nb.config.config_dict['DTA']['unit_time'])

        raw_ltg = dta.get_car_ltg_matrix(release_intervals, self.num_loading_interval)

        ltg = self._massage_raw_ltg(raw_ltg, self.ass_freq, num_assign_intervals, self.paths_list, self.observed_links)
        return ltg

    def _compute_link_tt_grad_on_path_flow_truck(self, dta):
        # dta.build_link_cost_map() is invoked already
        assign_intervals = np.arange(0, self.num_loading_interval, self.ass_freq)
        num_assign_intervals = len(assign_intervals)

        release_freq = 60
        # this is in terms of 5-s intervals
        release_intervals = np.arange(0, self.num_loading_interval, release_freq // self.nb.config.config_dict['DTA']['unit_time'])

        raw_ltg = dta.get_truck_ltg_matrix(release_intervals, self.num_loading_interval)

        ltg = self._massage_raw_ltg(raw_ltg, self.ass_freq, num_assign_intervals, self.paths_list_cc, self.observed_links)
        return ltg
    
    def _compute_link_tt_grad_on_path_flow_rh(self, dta):
        # dta.build_link_cost_map() is invoked already
        assign_intervals = np.arange(0, self.num_loading_interval, self.ass_freq)
        num_assign_intervals = len(assign_intervals)

        release_freq = 60
        # this is in terms of 5-s intervals
        release_intervals = np.arange(0, self.num_loading_interval, release_freq // self.nb.config.config_dict['DTA']['unit_time'])

        raw_ltg = dta.get_rh_ltg_matrix(release_intervals, self.num_loading_interval)

        ltg = self._massage_raw_ltg(raw_ltg, self.ass_freq, num_assign_intervals, self.paths_list_cc, self.observed_links)
        
        return ltg

    def _massage_raw_ltg(self, raw_ltg, ass_freq, num_assign_interval, paths_list, observed_links):
        assert(raw_ltg.shape[1] == 5)
        if raw_ltg.shape[0] == 0:
            # print("No ltg. No congestion.") # no congestion then no record?
            return csr_matrix((num_assign_interval * len(observed_links), 
                               num_assign_interval * len(paths_list)))

        num_e_path = len(paths_list)
        num_e_link = len(observed_links)
        # 15 min
        small_assign_freq = ass_freq * self.nb.config.config_dict['DTA']['unit_time'] / 60

        raw_ltg = raw_ltg[(raw_ltg[:, 1] < self.num_loading_interval) & (raw_ltg[:, 3] < self.num_loading_interval), :]

        # raw_ltg[:, 0]: path no.
        # raw_ltg[:, 1]: the count of 1 min interval in terms of 5s intervals
                
        if type(paths_list) == np.ndarray:
            # with mp.Pool(5) as p:
            #     pp = p.map(lambda x: True if len(np.where(paths_list == x)[0]) > 0 else False, raw_ltg[:, 0].astype(int))
            ind = np.array(list(map(lambda x: True if len(np.where(paths_list == x)[0]) > 0 else False, raw_ltg[:, 0].astype(int)))).astype(bool)
            assert(np.sum(ind) == len(ind))
            path_seq = (np.array(list(map(lambda x: np.where(paths_list == x)[0][0], raw_ltg[ind, 0].astype(int))))
                        + (raw_ltg[ind, 1] / ass_freq).astype(int) * num_e_path).astype(int)
        elif type(paths_list) == list:
            ind = np.array(list(map(lambda x: True if x in paths_list else False, raw_ltg[:, 0].astype(int)))).astype(bool)
            assert(np.sum(ind) == len(ind))
            path_seq = (np.array(list(map(lambda x: paths_list.index(x), raw_ltg[ind, 0].astype(int))))
                        + (raw_ltg[ind, 1] / ass_freq).astype(int) * num_e_path).astype(int)

        # raw_ltg[:, 2]: link no.
        # raw_ltg[:, 3]: the count of unit time interval (5s)
        if type(observed_links) == np.ndarray:
            # In Python 3, map() returns an iterable while, in Python 2, it returns a list.
            link_seq = (np.array(list(map(lambda x: np.where(observed_links == x)[0][0], raw_ltg[ind, 2].astype(int))))
                        + (raw_ltg[ind, 3] / ass_freq).astype(int) * num_e_link).astype(int)
        elif type(observed_links) == list:
            link_seq = (np.array(list(map(lambda x: observed_links.index(x), raw_ltg[ind, 2].astype(int))))
                        + (raw_ltg[ind, 3] / ass_freq).astype(int) * num_e_link).astype(int)
                    
        # print(path_seq)
        # raw_ltg[:, 4]: gradient, to be averaged for each large assign interval 
        p = raw_ltg[ind, 4] / (ass_freq * small_assign_freq)
        
        # print("Creating the coo matrix", time.time()), coo_matrix permits duplicate entries
        mat = coo_matrix((p, (link_seq, path_seq)), shape=(num_assign_interval * num_e_link, num_assign_interval * num_e_path))
        # pickle.dump((p, link_seq, path_seq), open('test.pickle', 'wb'))
        # print('converting the csr', time.time())
        
        # sum duplicate entries in coo_matrix
        mat = mat.tocsr()
        # print('finish converting', time.time())
        return mat
    
    def _get_one_data(self, j):
        assert (self.num_data > j)
        one_data_dict = dict()

        if self.config['use_car_link_flow'] or self.config['compute_car_link_flow_loss']:
            one_data_dict['car_link_flow'] = self.data_dict['car_link_flow'][j]
        
        if self.config['use_truck_link_flow']or self.config['compute_truck_link_flow_loss']:
            one_data_dict['truck_link_flow'] = self.data_dict['truck_link_flow'][j]
        
        if self.config['use_rh_link_flow']:
            one_data_dict['rh_link_flow'] = self.data_dict['rh_link_flow'][j]

        if self.config['use_car_link_tt'] or self.config['compute_car_link_tt_loss']:
            one_data_dict['car_link_tt'] = self.data_dict['car_link_tt'][j]
        
        if self.config['use_truck_link_tt'] or self.config['compute_truck_link_tt_loss']:
            one_data_dict['truck_link_tt'] = self.data_dict['truck_link_tt'][j]
        
        if self.config['car_count_agg']:
            one_data_dict['car_count_agg_L'] = self.car_count_agg_L_list[j]
        
        if self.config['truck_count_agg']:
            one_data_dict['truck_count_agg_L'] = self.truck_count_agg_L_list[j]
        
        if self.config['rh_count_agg']:
            one_data_dict['rh_count_agg_L'] = self.truck_count_agg_L_list[j]
        
        # car, truck and rh curb agg matrix N and H
        if self.config['car_curb_agg']:
            one_data_dict['car_curb_agg_N'] = self.car_curb_agg_N_list[j]
            one_data_dict['car_curb_agg_H'] = self.car_curb_agg_H_list[j]

        if self.config['truck_curb_agg']:
            one_data_dict['truck_curb_agg_N'] = self.truck_curb_agg_N_list[j]
            one_data_dict['truck_curb_agg_H'] = self.truck_curb_agg_H_list[j]

        if self.config['rh_curb_agg']:
            one_data_dict['rh_curb_agg_N'] = self.rh_curb_agg_N_list[j]
            one_data_dict['rh_curb_agg_H'] = self.rh_curb_agg_H_list[j]
        
        # car, truck and rh curb arrival and departure counts
        if self.config['use_car_curb_count'] or self.config['compute_car_curb_count_loss']:
            one_data_dict['car_curb_inflow'] = self.data_dict['car_curb_inflow'][j]
            one_data_dict['car_curb_outflow'] = self.data_dict['car_curb_outflow'][j]     
        
        if self.config['use_truck_curb_count'] or self.config['compute_truck_curb_count_loss']:
            one_data_dict['truck_curb_inflow'] = self.data_dict['truck_curb_inflow'][j]
            one_data_dict['truck_curb_outflow'] = self.data_dict['truck_curb_outflow'][j] 
        
        if self.config['use_rh_curb_count'] or self.config['compute_rh_curb_count_loss']:
            one_data_dict['rh_curb_inflow'] = self.data_dict['rh_curb_inflow'][j]
            one_data_dict['rh_curb_outflow'] = self.data_dict['rh_curb_outflow'][j]

        if 'mask_driving_link' in self.data_dict:
            one_data_dict['mask_driving_link'] = self.data_dict['mask_driving_link']
        else:
            one_data_dict['mask_driving_link'] = np.ones(len(self.observed_links) * self.num_assign_interval, dtype=bool)
        
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

            # use new function in MNM_mcnb.py
            self.nb.update_demand_path_curb(f_car, f_truck, f_rh)
   
        # self.nb.config.config_dict['DTA']['flow_scalar'] = 3
        if self.config['use_car_link_tt'] or self.config['use_truck_link_tt']:
            self.nb.config.config_dict['DTA']['total_interval'] = self.num_loading_interval # * 2  # hopefully this is sufficient 
        else:
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
        # python 2
        # hash1.update(str(time.time()) + str(counter))
        # python 3
        hash1.update((str(time.time()) + str(counter)).encode('utf-8'))
        
        new_folder = hash1.hexdigest()

        # new_folder_path = os.path.join(self.store_folder, str(new_folder))

        self.save_simulation_input_files_curb(new_folder, f_car, f_truck, f_rh)

        # invoke MNMAPI
        a = MNMAPI.mcdta_api()
        # read all files in new_folder
        a.initialize_curb(new_folder)
        
        # register links and paths
        a.register_links(self.observed_links)
        a.register_paths(self.paths_list)

        # TODO register path with curb choice
        # TODO add function in dta_api and change ltg funcs
        # m_path_vec should be different from m_path_vec_curb !!!!
        a.register_paths_cc(self.paths_list_cc)

        # install cc and cc_tree on registered links
        a.install_cc()
        a.install_cc_tree()
        # run DNL
        if show_loading:
            a.run_whole_curb()
        else:
            a.run_whole_curb_false()
        
        # print("Finish simulation\n")

        # travel_stats = a.get_travel_stats()
        # assert(len(travel_stats) == 4)
        # print("\n************ travel stats ************")
        # print("car count: {}".format(travel_stats[0]))
        # print("truck count: {}".format(travel_stats[1]))
        # print("car total travel time (hours): {}".format(travel_stats[2]))
        # print("truck total travel time (hours): {}".format(travel_stats[3]))
        # print("************ travel stats ************\n")

        # print_emission_stats() only works if folder is not removed, cannot find reason
        # a.print_emission_stats()

        # delete new_folder and all files and subdirectories below it.
        shutil.rmtree(new_folder)

        return a
    
    # grad = - L^T (link_flow_observation - link_flow_estimated) 
    # x_e = link_flow_estimated
    def _compute_count_loss_grad_on_car_link_flow(self, dta, one_data_dict):
        link_flow_array = one_data_dict['car_link_flow']
        x_e = dta.get_link_car_inflow(np.arange(0, self.num_loading_interval, self.ass_freq), 
                                      np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq).flatten(order='F')
        # print("x_e", x_e, link_flow_array)
        if self.config['car_count_agg']:
            x_e = one_data_dict['car_count_agg_L'].dot(x_e)
        discrepancy = np.nan_to_num(link_flow_array - x_e)
        grad = - discrepancy
        if self.config['car_count_agg']:
            grad = one_data_dict['car_count_agg_L'].T.dot(grad)
        # print("final link grad", grad)
        # assert(np.all(~np.isnan(grad)))
        return grad, x_e
  
    def _compute_count_loss_grad_on_truck_link_flow(self, dta, one_data_dict):
        link_flow_array = one_data_dict['truck_link_flow']
        x_e = dta.get_link_truck_inflow(np.arange(0, self.num_loading_interval, self.ass_freq), 
                                        np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq).flatten(order='F')
        if self.config['truck_count_agg']:
            x_e = one_data_dict['truck_count_agg_L'].dot(x_e)
        discrepancy = np.nan_to_num(link_flow_array - x_e)
        grad = - discrepancy
        if self.config['truck_count_agg']:
            grad = one_data_dict['truck_count_agg_L'].T.dot(grad)
        # assert(np.all(~np.isnan(grad)))
        return grad, x_e

    def _compute_count_loss_grad_on_rh_link_flow(self, dta, one_data_dict):
        link_flow_array = one_data_dict['rh_link_flow']
        x_e = dta.get_link_rh_inflow(np.arange(0, self.num_loading_interval, self.ass_freq), 
                                      np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq).flatten(order='F')
        # print("x_e", x_e, link_flow_array)
        if self.config['rh_count_agg']:
            x_e = one_data_dict['rh_count_agg_L'].dot(x_e)
        discrepancy = np.nan_to_num(link_flow_array - x_e)
        grad = - discrepancy
        if self.config['rh_count_agg']:
            grad = one_data_dict['rh_count_agg_L'].T.dot(grad)
        # print("final link grad", grad)
        # assert(np.all(~np.isnan(grad)))
        return grad, x_e

    def _compute_tt_loss_grad_on_car_link_tt(self, dta, one_data_dict):
        tt_e = dta.get_car_link_tt(np.arange(0, self.num_loading_interval, self.ass_freq)).flatten(order='F')
        tt_free = np.tile(dta.get_car_link_fftt(self.observed_links), (self.num_assign_interval))
        tt_e = np.maximum(tt_e, tt_free)
        tt_o = np.maximum(one_data_dict['car_link_tt'], tt_free) 
        discrepancy = np.nan_to_num(tt_o - tt_e)
        grad = - discrepancy
        return grad, tt_e

    def _compute_tt_loss_grad_on_truck_link_tt(self, dta, one_data_dict):
        tt_e = dta.get_truck_link_tt(np.arange(0, self.num_loading_interval, self.ass_freq)).flatten(order='F')
        tt_free = np.tile(dta.get_truck_link_fftt(self.observed_links), (self.num_assign_interval))
        tt_e = np.maximum(tt_e, tt_free)
        tt_o = np.maximum(one_data_dict['truck_link_tt'], tt_free)
        discrepancy = np.nan_to_num(tt_o - tt_e)
        grad = - discrepancy
        return grad, tt_e

    def _compute_tt_loss_grad_on_rh_link_tt(self, dta, one_data_dict):
        tt_e = dta.get_car_link_tt(np.arange(0, self.num_loading_interval, self.ass_freq)).flatten(order='F')
        tt_free = np.tile(dta.get_car_link_fftt(self.observed_links), (self.num_assign_interval))
        tt_e = np.maximum(tt_e, tt_free)
        tt_o = np.maximum(one_data_dict['car_link_tt'], tt_free) 
        discrepancy = np.nan_to_num(tt_o - tt_e)
        grad = - discrepancy
        return grad, tt_e

    def _compute_curb_loss_grad_on_car(self, dta, one_data_dict):
        car_arrival_e = dta.get_link_car_curb_inflow(np.arange(0, self.num_loading_interval, self.ass_freq), 
                                                     np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq).flatten(order='F')
        car_departure_e = dta.get_link_car_curb_outflow(np.arange(0, self.num_loading_interval, self.ass_freq), 
                                                        np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq).flatten(order='F')
        car_arrival_o = one_data_dict['car_curb_inflow']
        car_departure_o = one_data_dict['car_curb_outflow']

        if self.config['car_curb_agg']:
            car_arrival_e = one_data_dict['car_curb_agg_N'].dot(car_arrival_e) # N is for arrival and H is for departure
            car_departure_e = one_data_dict['car_curb_agg_H'].dot(car_departure_e) 

        discrepancy_arr = np.nan_to_num(car_arrival_o - car_arrival_e)
        
        discrepancy_dep = np.nan_to_num(car_departure_o - car_departure_e)

        grad_arr = - discrepancy_arr

        grad_dep = - discrepancy_dep

        if self.config['car_curb_agg']:
            grad_arr = one_data_dict['car_curb_agg_N'].T.dot(grad_arr)
            grad_dep = one_data_dict['car_curb_agg_H'].T.dot(grad_dep) 

        return grad_arr, grad_dep, car_arrival_e, car_departure_e
    
    def _compute_curb_loss_grad_on_truck(self, dta, one_data_dict):
        truck_arrival_e = dta.get_link_truck_curb_inflow(np.arange(0, self.num_loading_interval, self.ass_freq), 
                                                         np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq).flatten(order='F')
        truck_departure_e = dta.get_link_truck_curb_outflow(np.arange(0, self.num_loading_interval, self.ass_freq), 
                                                            np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq).flatten(order='F')
        truck_arrival_o = one_data_dict['truck_curb_inflow']
        truck_departure_o = one_data_dict['truck_curb_outflow']

        if self.config['truck_curb_agg']:
            truck_arrival_e = one_data_dict['truck_curb_agg_N'].dot(truck_arrival_e) # N is for arrival and H is for departure
            truck_departure_e = one_data_dict['truck_curb_agg_H'].dot(truck_departure_e) 
        
        discrepancy_arr = np.nan_to_num(truck_arrival_o - truck_arrival_e)
        
        discrepancy_dep = np.nan_to_num(truck_departure_o - truck_departure_e)

        grad_arr = - discrepancy_arr

        grad_dep = - discrepancy_dep

        if self.config['truck_curb_agg']:
            grad_arr = one_data_dict['truck_curb_agg_N'].T.dot(grad_arr)
            grad_dep = one_data_dict['truck_curb_agg_H'].T.dot(grad_dep) 

        return grad_arr, grad_dep, truck_arrival_e, truck_departure_e
    
    def _compute_curb_loss_grad_on_rh(self, dta, one_data_dict):
        rh_arrival_e = dta.get_link_rh_curb_inflow(np.arange(0, self.num_loading_interval, self.ass_freq), 
                                                   np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq).flatten(order='F')
        rh_departure_e = dta.get_link_rh_curb_outflow(np.arange(0, self.num_loading_interval, self.ass_freq), 
                                                      np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq).flatten(order='F')
        
        rh_arrival_o = one_data_dict['rh_curb_inflow']
        rh_departure_o = one_data_dict['rh_curb_outflow']

        if self.config['rh_curb_agg']:
            rh_arrival_e = one_data_dict['rh_curb_agg_N'].dot(rh_arrival_e)
            rh_departure_e = one_data_dict['rh_curb_agg_H'].dot(rh_departure_e)
        
        discrepancy_arr = np.nan_to_num(rh_arrival_o - rh_arrival_e) 

        discrepancy_dep = np.nan_to_num(rh_departure_o - rh_departure_e)

        grad_arr = - discrepancy_arr

        grad_dep = - discrepancy_dep

        if self.config['rh_curb_agg']:
            grad_arr = one_data_dict['rh_curb_agg_N'].T.dot(grad_arr)
            grad_dep = one_data_dict['rh_curb_agg_H'].T.dot(grad_dep)

        return grad_arr, grad_dep, rh_arrival_e, rh_departure_e

    def _get_loss(self, one_data_dict, dta):
        loss_dict = dict()
        assign_intervals = np.arange(0, self.num_loading_interval, self.ass_freq)

        if self.config['use_car_link_flow'] or self.config['compute_car_link_flow_loss']:
            x_e = dta.get_link_car_inflow(assign_intervals, assign_intervals + self.ass_freq).flatten(order='F')
            # print(x_e)
            if self.config['car_count_agg']:
                x_e = one_data_dict['car_count_agg_L'].dot(x_e)
            loss = self.config['link_car_flow_weight'] * np.linalg.norm(np.nan_to_num(x_e - one_data_dict['car_link_flow']))
            # loss = np.linalg.norm(np.nan_to_num(x_e[one_data_dict['mask_driving_link']] - one_data_dict['car_link_flow'][one_data_dict['mask_driving_link']]))
            loss_dict['car_count_loss'] = loss

        if self.config['use_truck_link_flow'] or self.config['compute_truck_link_flow_loss']:
            x_e = dta.get_link_truck_inflow(assign_intervals, assign_intervals + self.ass_freq).flatten(order='F')
            # print(x_e)
            if self.config['truck_count_agg']:
                x_e = one_data_dict['truck_count_agg_L'].dot(x_e)
            loss = self.config['link_truck_flow_weight'] * np.linalg.norm(np.nan_to_num(x_e - one_data_dict['truck_link_flow']))
            # loss = np.linalg.norm(np.nan_to_num(x_e[one_data_dict['mask_driving_link']] - one_data_dict['truck_link_flow'][one_data_dict['mask_driving_link']]))
            loss_dict['truck_count_loss'] = loss

        if self.config['use_car_link_tt'] or self.config['compute_car_link_tt_loss']:
            x_tt_e = dta.get_car_link_tt(assign_intervals).flatten(order='F')
            # print(x_tt_e)
            x_tt_r = one_data_dict['car_link_tt'] 
            # x_tt_e = dta.get_car_link_tt_robust(assign_intervals, assign_intervals + self.ass_freq, self.ass_freq, False).flatten(order='F')
            loss = self.config['link_car_tt_weight'] * np.linalg.norm(np.nan_to_num(x_tt_e[one_data_dict['mask_driving_link']] - x_tt_r[one_data_dict['mask_driving_link']]))
            # loss = np.linalg.norm(np.nan_to_num(x_tt_e[one_data_dict['mask_driving_link']] - one_data_dict['car_link_tt'][one_data_dict['mask_driving_link']]))
            loss_dict['car_tt_loss'] = loss

        if self.config['use_truck_link_tt'] or self.config['compute_truck_link_tt_loss']:
            x_tt_e = dta.get_truck_link_tt(assign_intervals).flatten(order='F')
            # print(x_tt_e)
            x_tt_r = one_data_dict['truck_link_tt']
            # x_tt_e = dta.get_truck_link_tt_robust(assign_intervals, assign_intervals + self.ass_freq, self.ass_freq, False).flatten(order='F')
            loss = self.config['link_truck_tt_weight'] * np.linalg.norm(np.nan_to_num(x_tt_e[one_data_dict['mask_driving_link']] - x_tt_r[one_data_dict['mask_driving_link']]))
            # loss = np.linalg.norm(np.nan_to_num(x_tt_e[one_data_dict['mask_driving_link']] - one_data_dict['truck_link_tt'][one_data_dict['mask_driving_link']]))
            loss_dict['truck_tt_loss'] = loss

        # add loss for car, truck and RH curb arrival and departures
        if self.config['use_car_curb_count'] or self.config['compute_car_curb_count_loss']:
            car_arrival_e = dta.get_link_car_curb_inflow(np.arange(0, self.num_loading_interval, self.ass_freq), 
                                                             np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq).flatten(order='F')
            
            car_departure_e = dta.get_link_car_curb_outflow(np.arange(0, self.num_loading_interval, self.ass_freq), 
                                                                np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq).flatten(order='F')
            
            if self.config['car_curb_agg']:
                car_arrival_e = one_data_dict['car_curb_agg_N'].dot(car_arrival_e)
                car_departure_e = one_data_dict['car_curb_agg_H'].dot(car_departure_e) 
            
            loss_car_arr = self.config['car_curb_arr_weight'] * \
                                np.linalg.norm(np.nan_to_num(car_arrival_e - one_data_dict['car_curb_inflow']))

            loss_car_dep = self.config['car_curb_dep_weight'] * \
                                np.linalg.norm(np.nan_to_num(car_departure_e - one_data_dict['car_curb_outflow']))

            loss_dict['car_arr_loss'] = loss_car_arr
            loss_dict['car_dep_loss'] = loss_car_dep

        if self.config['use_truck_curb_count'] or self.config['compute_truck_curb_count_loss']:
            truck_arrival_e = dta.get_link_truck_curb_inflow(np.arange(0, self.num_loading_interval, self.ass_freq), 
                                                             np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq).flatten(order='F')
            
            truck_departure_e = dta.get_link_truck_curb_outflow(np.arange(0, self.num_loading_interval, self.ass_freq), 
                                                                np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq).flatten(order='F')
            
            # print(truck_departure_e)

            # print(one_data_dict['truck_curb_outflow'])
            if self.config['truck_curb_agg']:
                truck_arrival_e = one_data_dict['truck_curb_agg_N'].dot(truck_arrival_e)
                truck_departure_e = one_data_dict['truck_curb_agg_H'].dot(truck_departure_e) 

            loss_truck_arr = self.config['truck_curb_arr_weight'] * \
                                np.linalg.norm(np.nan_to_num(truck_arrival_e - one_data_dict['truck_curb_inflow']))

            loss_truck_dep = self.config['truck_curb_dep_weight'] * \
                                np.linalg.norm(np.nan_to_num(truck_departure_e - one_data_dict['truck_curb_outflow']))

            loss_dict['truck_arr_loss'] = loss_truck_arr
            loss_dict['truck_dep_loss'] = loss_truck_dep

        if self.config['use_rh_curb_count'] or self.config['compute_rh_curb_count_loss']:
            rh_arrival_e = dta.get_link_rh_curb_inflow(np.arange(0, self.num_loading_interval, self.ass_freq), 
                                                             np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq).flatten(order='F')
            
            rh_departure_e = dta.get_link_rh_curb_outflow(np.arange(0, self.num_loading_interval, self.ass_freq), 
                                                                np.arange(0, self.num_loading_interval, self.ass_freq) + self.ass_freq).flatten(order='F')
            
            # print(rh_departure_e)

            # print(one_data_dict['rh_curb_outflow'])
            if self.config['rh_curb_agg']:
                rh_arrival_e = one_data_dict['rh_curb_agg_N'].dot(rh_arrival_e)
                rh_departure_e = one_data_dict['rh_curb_agg_N'].dot(rh_departure_e)

            loss_rh_arr = self.config['rh_curb_arr_weight'] * \
                                np.linalg.norm(np.nan_to_num(rh_arrival_e - one_data_dict['rh_curb_inflow']))

            loss_rh_dep = self.config['rh_curb_dep_weight'] * \
                                np.linalg.norm(np.nan_to_num(rh_departure_e - one_data_dict['rh_curb_outflow']))

            loss_dict['rh_arr_loss'] = loss_rh_arr
            loss_dict['rh_dep_loss'] = loss_rh_dep

        total_loss = 0.0
        for loss_type, loss_value in loss_dict.items():
            total_loss += loss_value
        return total_loss, loss_dict

    def compute_path_flow_grad_and_loss(self, one_data_dict, f_car, f_truck, f_rh, counter=0):
        # print("Running simulation", time.time())
        dta = self._run_simulation_curb(f_car, f_truck, f_rh, counter, show_loading = False)
        # print("Running done", time.time())

        if self.config['use_car_link_tt'] or self.config['use_truck_link_tt']:
            dta.build_link_cost_map(True)
            dta.get_link_queue_dissipated_time()
            # car_ltg_matrix, truck_ltg_matrix, rh_ltg_matrix = self.get_ltg(dta)

        # print("Getting DAR", time.time())
        (car_dar, truck_dar, rh_dar) = self.get_dar_curb(dta, f_car, f_truck, f_rh)

        x_e_car, x_e_truck, x_e_rh, tt_e_car, tt_e_truck = None, None, None, None, None

        truck_arrival_e, truck_departure_e, rh_arrival_e, rh_departure_e = None, None, None, None

        # print("Evaluating grad", time.time())
        car_grad = np.zeros(len(self.observed_links) * self.num_assign_interval)
        truck_grad = np.zeros(len(self.observed_links) * self.num_assign_interval)
        rh_grad = np.zeros(len(self.observed_links) * self.num_assign_interval)

        if self.config['use_car_link_flow']:
            # print("car link flow", time.time())
            grad, x_e_car = self._compute_count_loss_grad_on_car_link_flow(dta, one_data_dict)
            car_grad += self.config['link_car_flow_weight'] * grad # = -w L^T (link_flow_observation - link_flow_estimated) 

        if self.config['use_truck_link_flow']:
            grad, x_e_truck = self._compute_count_loss_grad_on_truck_link_flow(dta, one_data_dict)
            truck_grad += self.config['link_truck_flow_weight'] * grad

        if self.config['use_rh_link_flow']:
            grad, x_e_rh = self._compute_count_loss_grad_on_rh_link_flow(dta, one_data_dict)
            rh_grad += self.config['link_rh_flow_weight'] * grad

        f_car_grad = car_dar.T.dot(car_grad) # -w rho^T L^T (link_flow_observation - link_flow_estimated) 
        f_truck_grad = truck_dar.T.dot(truck_grad)
        f_rh_grad = rh_dar.T.dot(rh_grad)

        # Travel time
        car_grad = np.zeros(len(self.observed_links) * self.num_assign_interval)
        truck_grad = np.zeros(len(self.observed_links) * self.num_assign_interval)
        rh_grad = np.zeros(len(self.observed_links) * self.num_assign_interval)

        if self.config['use_car_link_tt']:
            grad, tt_e_car = self._compute_tt_loss_grad_on_car_link_tt(dta, one_data_dict)
            car_grad += self.config['link_car_tt_weight'] * grad
            ## IPMC method
            # f_car_grad += car_ltg_matrix.T.dot(car_grad)

            # Wei's method
            f_car_grad += car_dar.T.dot(car_grad)

        if self.config['use_truck_link_tt']:
            grad, tt_e_truck = self._compute_tt_loss_grad_on_truck_link_tt(dta, one_data_dict)
            truck_grad += self.config['link_truck_tt_weight'] * grad
            ## IPMC method
            # f_truck_grad += truck_ltg_matrix.T.dot(truck_grad)

            # Wei's method
            f_truck_grad += truck_dar.T.dot(truck_grad)

        if self.config['use_rh_link_tt']:
            grad, _ = self._compute_tt_loss_grad_on_rh_link_tt(dta, one_data_dict)
            rh_grad += self.config['link_rh_tt_weight'] * grad
            ## IPMC method
            # debugg for rh ltg matrix shape should be 50 not 30
            # f_rh_grad += rh_ltg_matrix.T.dot(rh_grad)

            # Wei's method
            f_rh_grad += rh_dar.T.dot(rh_grad)
        
        # curb arrival and departure
        # get dar for curb-related count
        car_dar_arr, truck_dar_arr, rh_dar_arr = self.get_dar_arrival(dta, f_car, f_truck, f_rh)

        car_dar_dep, truck_dar_dep, rh_dar_dep = self.get_dar_departure(dta, f_car, f_truck, f_rh)

        # car_curb_arr_weight, car_curb_dep_weight, truck_curb_arr_weight, truck_curb_dep_weight, rh_curb_arr_weight, rh_curb_dep_weight
        if self.config['use_car_curb_count']:
            car_grad_arr, car_grad_dep, car_arrival_e, car_departure_e = self._compute_curb_loss_grad_on_car(dta, one_data_dict)
            f_car_grad += car_dar_arr.T.dot(self.config['car_curb_arr_weight'] * car_grad_arr) + \
                          car_dar_dep.T.dot(self.config['car_curb_dep_weight'] * car_grad_dep)
            
        if self.config['use_truck_curb_count']:
            truck_grad_arr, truck_grad_dep, truck_arrival_e, truck_departure_e = self._compute_curb_loss_grad_on_truck(dta, one_data_dict)
            f_truck_grad += truck_dar_arr.T.dot(self.config['truck_curb_arr_weight'] * truck_grad_arr) + \
                            truck_dar_dep.T.dot(self.config['truck_curb_dep_weight'] * truck_grad_dep)
        
        if self.config['use_rh_curb_count']:
            rh_grad_arr, rh_grad_dep, rh_arrival_e, rh_departure_e = self._compute_curb_loss_grad_on_rh(dta, one_data_dict)
            f_rh_grad += rh_dar_arr.T.dot(self.config['rh_curb_arr_weight'] * rh_grad_arr) + \
                         rh_dar_dep.T.dot(self.config['rh_curb_dep_weight'] * rh_grad_dep)
        
        total_loss, loss_dict = self._get_loss(one_data_dict, dta)
        return f_car_grad, f_truck_grad, f_rh_grad, total_loss, loss_dict, dta, x_e_car, x_e_truck, x_e_rh, tt_e_car, tt_e_truck, car_arrival_e, car_departure_e, truck_arrival_e, truck_departure_e, rh_arrival_e, rh_departure_e

    # TODO add ridehailing flow
    def estimate_path_flow_pytorch_curb(self, 
                                        car_step_size = 0.5, 
                                        truck_step_size = 0.2, 
                                        link_car_flow_weight = 1, 
                                        link_truck_flow_weight = 1, 
                                        link_car_tt_weight = 0.05, 
                                        link_truck_tt_weight = 0.01,
                                        car_curb_arr_weight = 1,
                                        car_curb_dep_weight = 0.5,
                                        truck_curb_arr_weight = 1,
                                        truck_curb_dep_weight = 0.5,
                                        rh_curb_arr_weight = 1,
                                        rh_curb_dep_weight = 0.5, 
                                        max_epoch = 150, 
                                        algo = 'Adagrad', 
                                        normalized_by_scale = True,
                                        car_init_scale = 10,
                                        truck_init_scale = 1, 
                                        rh_init_scale = 10,
                                        store_folder = None, 
                                        use_file_as_init = None,
                                        starting_epoch = 0):

        self.store_folder = store_folder
        # if np.isscalar(column_generation):
        #     column_generation = np.ones(max_epoch, dtype=int) * column_generation
        # assert(len(column_generation) == max_epoch)
    
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

        # add weights of car arr and dep
        if np.isscalar(car_curb_arr_weight):
            car_curb_arr_weight = np.ones(max_epoch, dtype = bool) * car_curb_arr_weight
        assert(len(car_curb_arr_weight) == max_epoch)

        if np.isscalar(car_curb_dep_weight):
            car_curb_dep_weight = np.ones(max_epoch, dtype = bool) * car_curb_dep_weight
        assert(len(car_curb_dep_weight) == max_epoch)
        
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
        best_car_arrival_e, best_car_departure_e, best_truck_arrival_e, best_truck_departure_e, best_rh_arrival_e, best_rh_departure_e = None, None, None, None, None, None

        # here the basic variables to be estimated are path flows, not OD demand, so no route choice model, unlike in sDODE.py
        if use_file_as_init is None:
            (f_car, f_truck, f_rh) = self.init_path_flow(car_scale = car_init_scale, truck_scale = truck_init_scale, rh_scale = rh_init_scale)
        else:
            # most recent
            _, _, loss_list, best_epoch, _, _, _, \
            _, _, _, _, _, \
            _, _, _, _, _, _ = pickle.load(open(use_file_as_init, 'rb'))
            # best 
            use_file_as_init = os.path.join(store_folder, '{}_iteration.pickle'.format(best_epoch))
            loss, loss_dict, loss_list, best_epoch, best_f_car_driving, best_f_truck_driving, best_f_rh_driving, \
            best_x_e_car, best_x_e_truck, best_x_e_rh, best_tt_e_car, best_tt_e_truck, \
            best_car_arrival_e, best_car_departure_e, best_truck_arrival_e, best_truck_departure_e, best_rh_arrival_e, best_rh_departure_e\
                          = pickle.load(open(use_file_as_init, 'rb'))
            
            f_car, f_truck, f_rh = best_f_car_driving, best_f_truck_driving, best_f_rh_driving

        if normalized_by_scale:
            f_car_tensor = torch.from_numpy(f_car / np.maximum(car_init_scale, 0.000001))
            f_truck_tensor = torch.from_numpy(f_truck / np.maximum(truck_init_scale, 0.000001))
            f_rh_tensor = torch.from_numpy(f_rh / np.maximum(car_init_scale, 0.000001))
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
            {'params': f_rh_tensor, 'lr': car_step_size},
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
            loss = np.float(0)
            # print("Start iteration", time.time())
            loss_dict = {'car_count_loss': 0.0, 'truck_count_loss': 0.0, 'car_tt_loss': 0.0, 'truck_tt_loss': 0.0, 'car_arr_loss': 0.0, 'car_dep_loss': 0.0, 'truck_arr_loss': 0.0, 'truck_dep_loss': 0.0, 'rh_arr_loss': 0.0, 'rh_dep_loss': 0.0}

            self.config['link_car_flow_weight'] = link_car_flow_weight[i] * (self.config['use_car_link_flow'] or self.config['compute_car_link_flow_loss'])
            self.config['link_truck_flow_weight'] = link_truck_flow_weight[i] * (self.config['use_truck_link_flow'] or self.config['compute_truck_link_flow_loss'])
            
            self.config['link_car_tt_weight'] = link_car_tt_weight[i] * (self.config['use_car_link_tt'] or self.config['compute_car_link_tt_loss'])
            self.config['link_truck_tt_weight'] = link_truck_tt_weight[i] * (self.config['use_truck_link_tt'] or self.config['compute_truck_link_tt_loss'])

            # add weights for curb arrival and departure
            # car_curb_arr_weight, car_curb_dep_weight, truck_curb_arr_weight, truck_curb_dep_weight, rh_curb_arr_weight, rh_curb_dep_weight
            self.config['car_curb_arr_weight'] = car_curb_arr_weight[i]
            self.config['car_curb_dep_weight'] = car_curb_dep_weight[i]
            self.config['truck_curb_arr_weight'] = truck_curb_arr_weight[i]
            self.config['truck_curb_dep_weight'] = truck_curb_dep_weight[i]
            self.config['rh_curb_arr_weight'] = rh_curb_arr_weight[i]
            self.config['rh_curb_dep_weight'] = rh_curb_dep_weight[i] 

            # data iteration loop
            for j in seq:

                # one data sample
                one_data_dict = self._get_one_data(j)

                # main function for gradient and loss
                car_grad, truck_grad, rh_grad, tmp_loss, tmp_loss_dict, _, \
                x_e_car, x_e_truck, x_e_rh, tt_e_car, tt_e_truck, \
                car_arrival_e, car_departure_e, truck_arrival_e, truck_departure_e, rh_arrival_e, rh_departure_e = self.compute_path_flow_grad_and_loss(one_data_dict, f_car, f_truck, f_rh)
                
                # print("gradient", car_grad, truck_grad, rh_grad)

                optimizer.zero_grad()

                if normalized_by_scale:
                    f_car_tensor.grad = torch.from_numpy(car_grad * car_init_scale)
                    f_truck_tensor.grad = torch.from_numpy(truck_grad * truck_init_scale)
                    f_rh_tensor.grad = torch.from_numpy(rh_grad * car_init_scale)
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
                    f_rh = f_rh_tensor.data.cpu().numpy() * car_init_scale
                else:
                    f_car = f_car_tensor.data.cpu().numpy()
                    f_truck = f_truck_tensor.data.cpu().numpy()
                    f_rh = f_rh_tensor.data.cpu().numpy()
            
                f_car = np.maximum(f_car, 0.000001)
                f_truck = np.maximum(f_truck, 0.000001)
                f_rh = np.maximum(f_rh, 0.000001)

                loss += tmp_loss / np.float(self.num_data)
                for loss_type, loss_value in tmp_loss_dict.items():
                    loss_dict[loss_type] += loss_value / np.float(self.num_data)

            print("Epoch:", starting_epoch + i, "Loss:", loss, self.print_separate_accuracy(loss_dict))
            loss_list.append([loss, loss_dict])
                    
            if (best_epoch == 0) or (loss_list[best_epoch][0] > loss_list[-1][0]):
                best_epoch = starting_epoch + i
                best_f_car_driving, best_f_truck_driving, best_f_rh_driving = f_car, f_truck, f_rh
                best_x_e_car, best_x_e_truck, best_x_e_rh, best_tt_e_car, best_tt_e_truck = x_e_car, x_e_truck, x_e_rh, tt_e_car, tt_e_truck
                best_car_arrival_e, best_car_departure_e, best_truck_arrival_e, best_truck_departure_e, best_rh_arrival_e, best_rh_departure_e = car_arrival_e, car_departure_e, truck_arrival_e, truck_departure_e, rh_arrival_e, rh_departure_e
                
                if store_folder is not None:
                    self.save_simulation_input_files_curb(os.path.join(store_folder, 'input_files_estimate_path_flow'), 
                                                     best_f_car_driving, best_f_truck_driving, best_f_rh_driving)
             
            # print(f_car, f_truck, f_rh)
            # break
            if store_folder is not None:
                pickle.dump((loss, loss_dict, loss_list, best_epoch, f_car, f_truck, f_rh,
                                x_e_car, x_e_truck, x_e_rh, tt_e_car, tt_e_truck, car_arrival_e, car_departure_e, truck_arrival_e, truck_departure_e, rh_arrival_e, rh_departure_e), 
                                open(os.path.join(store_folder, str(starting_epoch + i) + '_iteration.pickle'), 'wb'))

        print("Best loss at Epoch:", best_epoch, "Loss:", loss_list[best_epoch][0], self.print_separate_accuracy(loss_list[best_epoch][1]))
        return best_f_car_driving, best_f_truck_driving, best_f_rh_driving, \
               best_x_e_car, best_x_e_truck, best_x_e_rh, best_tt_e_car, best_tt_e_truck, \
               best_car_arrival_e, best_car_departure_e, best_truck_arrival_e, best_truck_departure_e, best_rh_arrival_e, best_rh_departure_e, loss_list


