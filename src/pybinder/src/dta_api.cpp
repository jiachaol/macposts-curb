#include <pybind11/pybind11.h>
#include "dta_api.h"
#include "typecast_ground.h"

#include "dta_gradient_utls.h"
// #include "multiclass.h"
#include "multiclass_curb.h"

#include <unordered_map>
#include <vector>

namespace py = pybind11;


Test_Types::Test_Types()
{

}

Test_Types::~Test_Types()
{

}


py::list Test_Types::get_list()
{
  py::list v;
  v.append(3);
  v.append(2.2);
  v.append("dfdf");
  return v;
}

DenseMatrixR Test_Types::get_matrix(){
  Eigen::MatrixXd mat(5, 6);
  mat << 0,  3,  0,  0,  0, 11,
           22, 0,  0,  0, 17, 11,
           7,  5,  0,  1,  0, 11,
           0,  0,  0,  0,  0, 11,
           0,  0, 14,  0,  8, 11;
  return DenseMatrixR(mat);
}

SparseMatrixR Test_Types::get_sparse_matrix(){
  Eigen::MatrixXd mat(5, 6);
  mat << 0,  3,  0,  0,  0, 11,
           22, 0,  0,  0, 17, 11,
           7,  5,  0,  1,  0, 11,
           0,  0,  0,  0,  0, 11,
           0,  0, 14,  0,  8, 11;
  return Eigen::SparseView<Eigen::MatrixXd>(mat);
}

SparseMatrixR Test_Types::get_sparse_matrix2(int num){
  int m = num;
  std::vector<Eigen::Triplet<double>> tripletList;
  tripletList.reserve(5000);
  for(int i=1; i < m-1; ++i){
    tripletList.push_back(Eigen::Triplet<double>(i,i,1));
    tripletList.push_back(Eigen::Triplet<double>(i,i+1,1));
    tripletList.push_back(Eigen::Triplet<double>(i-1,i,1));
  }
  SparseMatrixR mat(m, m);
  mat.setFromTriplets(tripletList.begin(), tripletList.end());
  return mat;
}


/**********************************************************************************************************
***********************************************************************************************************
                        run function
***********************************************************************************************************
***********************************************************************************************************/
int run_dta(std::string folder) {
  printf("Current working directory is......\n");
  std::cout << folder << std::endl;

  MNM_Dta *test_dta = new MNM_Dta(folder);
  test_dta -> build_from_files();
  printf("Hooking......\n");
  test_dta -> hook_up_node_and_link();
  // printf("Checking......\n");
  // test_dta -> is_ok();
  test_dta -> loading(false);


  delete test_dta;

  return 0;
}


/**********************************************************************************************************
***********************************************************************************************************
                        Singleclass
***********************************************************************************************************
***********************************************************************************************************/
Dta_Api::Dta_Api()
{
  m_dta = nullptr;
  m_link_vec = std::vector<MNM_Dlink*>();
  m_path_vec = std::vector<MNM_Path*>();
  m_path_map = std::unordered_map<MNM_Path*, int>(); 
  m_ID_path_mapping = std::unordered_map<TInt, MNM_Path*>();
  // m_link_map = std::unordered_map<MNM_Dlink*, int>();
}

Dta_Api::~Dta_Api()
{
  if (m_dta != nullptr){
    delete m_dta;
  }
  m_link_vec.clear();
  m_path_vec.clear();
  // m_link_map.clear();
  m_ID_path_mapping.clear();
  
}

int Dta_Api::initialize(std::string folder)
{
  m_dta = new MNM_Dta(folder);
  m_dta -> build_from_files();
  m_dta -> hook_up_node_and_link();
  // m_dta -> is_ok();
  // printf("start load ID path mapping 0\n");
  if (MNM_Routing_Fixed *_routing = dynamic_cast<MNM_Routing_Fixed *>(m_dta -> m_routing)){
    MNM::get_ID_path_mapping(m_ID_path_mapping, _routing -> m_path_table);
    return 0;
  }
  if (MNM_Routing_Hybrid *_routing = dynamic_cast<MNM_Routing_Hybrid *>(m_dta -> m_routing)){
    // printf("start load ID path mapping\n");
    MNM::get_ID_path_mapping(m_ID_path_mapping, _routing -> m_routing_fixed -> m_path_table);
    // printf("mapping size %d\n", m_ID_path_mapping.size());
    return 0;
  }
  std::runtime_error("Dta_Api:: Routing type not implemented in API");
  return -1;
}

int Dta_Api::run_once()
{
  return 0;
}

int Dta_Api::install_cc()
{
  for (size_t i = 0; i<m_link_vec.size(); ++i){
    m_link_vec[i] -> install_cumulative_curve();
  }
  return 0;
}

int Dta_Api::install_cc_tree()
{
  for (size_t i = 0; i<m_link_vec.size(); ++i){
    m_link_vec[i] -> install_cumulative_curve_tree();
  }
  return 0;
}

int Dta_Api::run_whole()
{
  m_dta -> pre_loading();
  m_dta -> loading(false);
  return 0;
}

int Dta_Api::get_cur_loading_interval()
{
  return m_dta -> m_current_loading_interval();
}

int Dta_Api::register_links(py::array_t<int> links)
{
  if (m_link_vec.size() > 0){
    printf("Warning, Dta_Api::register_links, link exists\n");
    m_link_vec.clear();
  }
  // https://people.duke.edu/~ccc14/cspy/18G_C++_Python_pybind11.html#Using-numpy-arrays-as-function-arguments-and-return-values
  // https://www.linyuanshi.me/post/pybind11-array/
  // The properties of the numpy array can be obtained by calling its request() method
  auto links_buf = links.request();
  if (links_buf.ndim != 1){  // dimensions
    throw std::runtime_error("Number of dimensions must be one");
  }
  // obtain the pointer with the type cast to access and modify the elements of the array
  int *links_ptr = (int *) links_buf.ptr;
  MNM_Dlink *_link;
  for (int i = 0; i < links_buf.shape[0]; i++){
    _link = m_dta -> m_link_factory -> get_link(TInt(links_ptr[i]));
    // printf("%d\n", links_ptr[i]);
    if(std::find(m_link_vec.begin(), m_link_vec.end(), _link) != m_link_vec.end()) {
      throw std::runtime_error("Error, Dta_Api::register_links, link does not exist");
    } 
    else {
      m_link_vec.push_back(_link);
      // m_link_map.insert(std::make_pair(_link, i));
    }
  }
  return 0;
}

int Dta_Api::register_paths(py::array_t<int> paths)
{
  if (m_path_vec.size() > 0){
    printf("Warning, Dta_Api::register_paths, path exists\n");
    m_path_vec.clear();
    m_path_map.clear();
  }
  auto paths_buf = paths.request();
  if (paths_buf.ndim != 1){
    throw std::runtime_error("Dta_Api::register_paths: Number of dimensions must be one");
  }
  int *paths_ptr = (int *) paths_buf.ptr; 
  TInt _path_ID;
  for (int i = 0; i < paths_buf.shape[0]; i++){
    _path_ID = TInt(paths_ptr[i]);
    // printf("registering path %d, %d\n", _path_ID(), (int)m_ID_path_mapping.size());
    if (m_ID_path_mapping.find(_path_ID) == m_ID_path_mapping.end()){
      throw std::runtime_error("Dta_Api::register_paths: No such path");
    }
    else {
      m_path_vec.push_back(m_ID_path_mapping[_path_ID]);
      m_path_map.insert(std::make_pair(m_ID_path_mapping[_path_ID], i));
    }
  }
  // m_path_set = std::set<MNM_Path*> (m_path_vec.begin(), m_path_vec.end());
  return 0;
}

py::array_t<double> Dta_Api::get_link_inflow(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
{
  auto start_buf = start_intervals.request();
  auto end_buf = end_intervals.request();
  if (start_buf.ndim != 1 || end_buf.ndim != 1){
    throw std::runtime_error("Error, Dta_Api::get_link_inflow, input dimension mismatch");
  }
  if (start_buf.shape[0] != end_buf.shape[0]){
    throw std::runtime_error("Error, Dta_Api::get_link_inflow, input length mismatch");
  }
  // number of time steps from input
  int l = start_buf.shape[0];
  int new_shape [2] = { (int) m_link_vec.size(), l};
  // creat a new py::array_t<double> as output, here ndim == 2
  auto result = py::array_t<double>(new_shape);
  // request() method of py::array_t()
  auto result_buf = result.request();
  // obtain the pointer to manipulate the created array
  double *result_prt = (double *) result_buf.ptr;
  int *start_prt = (int *) start_buf.ptr;
  int *end_prt = (int *) end_buf.ptr;
  for (int t = 0; t < l; ++t){
    for (size_t i = 0; i<m_link_vec.size(); ++i){
      if (end_prt[t] < start_prt[t]){
        throw std::runtime_error("Error, Dta_Api::get_link_inflow, end time smaller than start time");
      }
      if (end_prt[t] > get_cur_loading_interval()){
        throw std::runtime_error("Error, Dta_Api::get_link_inflow, loaded data not enough");
      }
      // matrix is stored as a row-major array
      result_prt[i * l + t] = MNM_DTA_GRADIENT::get_link_inflow(m_link_vec[i], TFlt(start_prt[t]), TFlt(end_prt[t]))();
      // printf("i %d, t %d, %f\n", i, t, result_prt[i * l + t]);
    }
  }
  // return the created array
  return result;
}

py::array_t<double> Dta_Api::get_link_tt(py::array_t<int>start_intervals)
{
  auto start_buf = start_intervals.request();
  if (start_buf.ndim != 1){
    throw std::runtime_error("Error, Dta_Api::get_link_tt, input dimension mismatch");
  }
  int l = start_buf.shape[0];
  int new_shape [2] = { (int) m_link_vec.size(), l}; 

  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  int *start_prt = (int *) start_buf.ptr;
  for (int t = 0; t < l; ++t){
    for (size_t i = 0; i<m_link_vec.size(); ++i){
      if (start_prt[t] > get_cur_loading_interval()){
        throw std::runtime_error("Error, Dta_Api::get_link_tt, loaded data not enough");
      }
      result_prt[i * l + t] = MNM_DTA_GRADIENT::get_travel_time(m_link_vec[i], TFlt(start_prt[t]), m_dta -> m_unit_time, get_cur_loading_interval())() * m_dta -> m_unit_time; // second
    }
  }
  return result;
}

py::array_t<double> Dta_Api::get_path_tt(py::array_t<int>start_intervals)
{
  auto start_buf = start_intervals.request();
  if (start_buf.ndim != 1){
    throw std::runtime_error("Error, Dta_Api::get_path_tt, input dimension mismatch");
  }
  int l = start_buf.shape[0];
  int new_shape [2] = { (int) m_path_vec.size(), l}; 
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  int *start_prt = (int *) start_buf.ptr;
  for (int t = 0; t < l; ++t){
    if (start_prt[t] > get_cur_loading_interval()){
      throw std::runtime_error("Error, Dta_Api::get_path_tt, loaded data not enough");
    }
    for (size_t i = 0; i<m_path_vec.size(); ++i){
      result_prt[i * l + t] = MNM_DTA_GRADIENT::get_path_travel_time(
              m_path_vec[i], TFlt(start_prt[t]), m_dta -> m_link_factory, m_dta -> m_unit_time, get_cur_loading_interval())() * m_dta -> m_unit_time;  // second
    }
  }
  return result;
}



py::array_t<double> Dta_Api::get_link_in_cc(int link_ID)
{
  if (m_dta -> m_link_factory -> get_link(TInt(link_ID)) -> m_N_in == nullptr){
    throw std::runtime_error("Error, Dta_Api::get_link_in_cc, cc not installed");
  }
  std::deque<std::pair<TFlt, TFlt>> _record = m_dta -> m_link_factory -> get_link(TInt(link_ID)) -> m_N_in -> m_recorder;
  int new_shape [2] = { (int) _record.size(), 2}; 
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  for (size_t i=0; i< _record.size(); ++i){
    result_prt[i * 2 ] = _record[i].first();
    result_prt[i * 2 + 1 ] =  _record[i].second();
  }
  return result;
}


py::array_t<double> Dta_Api::get_link_out_cc(int link_ID)
{
  if (m_dta -> m_link_factory -> get_link(TInt(link_ID)) -> m_N_out == nullptr){
    throw std::runtime_error("Error, Dta_Api::get_link_out_cc, cc not installed");
  }
  std::deque<std::pair<TFlt, TFlt>> _record = m_dta -> m_link_factory -> get_link(TInt(link_ID)) -> m_N_out -> m_recorder;
  int new_shape [2] = { (int) _record.size(), 2}; 
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  for (size_t i=0; i< _record.size(); ++i){
    result_prt[i * 2 ] = _record[i].first();
    result_prt[i * 2 + 1 ] =  _record[i].second();
  }
  return result;
}


py::array_t<double> Dta_Api::get_dar_matrix(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
{
  auto start_buf = start_intervals.request();
  auto end_buf = end_intervals.request();
  if (start_buf.ndim != 1 || end_buf.ndim != 1){
    throw std::runtime_error("Error, Dta_Api::get_link_inflow, input dimension mismatch");
  }
  if (start_buf.shape[0] != end_buf.shape[0]){
    throw std::runtime_error("Error, Dta_Api::get_link_inflow, input length mismatch");
  }
  int l = start_buf.shape[0];
  int *start_prt = (int *) start_buf.ptr;
  int *end_prt = (int *) end_buf.ptr;
  std::vector<dar_record*> _record = std::vector<dar_record*>();
  // for (size_t i = 0; i<m_link_vec.size(); ++i){
  //   m_link_vec[i] -> m_N_in_tree -> print_out();
  // }
  for (int t = 0; t < l; ++t){
      if (end_prt[t] < start_prt[t]){
          throw std::runtime_error("Error, Dta_Api::get_dar_matrix, end time smaller than start time");
      }
      if (end_prt[t] > get_cur_loading_interval()){
          throw std::runtime_error("Error, Dta_Api::get_dar_matrix, loaded data not enough");
      }
      for (size_t i = 0; i<m_link_vec.size(); ++i){
          MNM_DTA_GRADIENT::add_dar_records(_record, m_link_vec[i], m_path_map, TFlt(start_prt[t]), TFlt(end_prt[t]));
      }
  }
  // path_ID, assign_time, link_ID, start_int, flow
  int new_shape [2] = { (int) _record.size(), 5}; 
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  dar_record* tmp_record;
  for (size_t i = 0; i < _record.size(); ++i){
    tmp_record = _record[i];
    result_prt[i * 5 + 0] = (double) tmp_record -> path_ID();
    // the count of 15 min interval
    result_prt[i * 5 + 1] = (double) tmp_record -> assign_int();
    result_prt[i * 5 + 2] = (double) tmp_record -> link_ID();
    // the count of unit time interval (5s)
    result_prt[i * 5 + 3] = (double) tmp_record -> link_start_int();
    result_prt[i * 5 + 4] = tmp_record -> flow();
  }
  for (size_t i = 0; i < _record.size(); ++i){
    delete _record[i];
  }
  _record.clear();
  return result;
}

SparseMatrixR Dta_Api::get_complete_dar_matrix(py::array_t<int>start_intervals, py::array_t<int>end_intervals,
                                               int num_intervals, py::array_t<double> f)
{
  int _num_e_path = m_path_map.size();
  int _num_e_link = m_link_vec.size();
  auto start_buf = start_intervals.request();
  auto end_buf = end_intervals.request();
  auto f_buf = f.request();
  if (start_buf.ndim != 1 || end_buf.ndim != 1){
    throw std::runtime_error("Error, Dta_Api::get_link_inflow, input dimension mismatch");
  }
  if (start_buf.shape[0] != end_buf.shape[0]){
    throw std::runtime_error("Error, Dta_Api::get_link_inflow, input length mismatch");
  }
  if (f_buf.ndim != 1){
    throw std::runtime_error("Error, Dta_Api::get_link_inflow, input path flow mismatch");
  }
  int l = start_buf.shape[0];
  int *start_prt = (int *) start_buf.ptr;
  int *end_prt = (int *) end_buf.ptr;
  double *f_ptr = (double *) f_buf.ptr;

  std::vector<Eigen::Triplet<double>> _record;
  // pre-allocate sufficient space for dar
  _record.reserve(100000);
  for (int t = 0; t < l; ++t){
      if (end_prt[t] < start_prt[t]){
          throw std::runtime_error("Error, Dta_Api::get_complete_dar_matrix, end time smaller than start time");
      }
      if (end_prt[t] > get_cur_loading_interval()){
          throw std::runtime_error("Error, Dta_Api::get_complete_dar_matrix, loaded data not enough");
      }
      for (size_t i = 0; i<m_link_vec.size(); ++i){

          MNM_DTA_GRADIENT::add_dar_records_eigen(_record, m_link_vec[i], m_path_map,
                                                  TFlt(start_prt[t]), TFlt(end_prt[t]),
                                                  i, t, _num_e_link, _num_e_path, f_ptr);
     }
  }
  // https://eigen.tuxfamily.org/dox/group__TutorialSparse.html
  // dar matrix rou
  SparseMatrixR mat(num_intervals * _num_e_link, num_intervals * _num_e_path);
  // https://eigen.tuxfamily.org/dox/classEigen_1_1SparseMatrix.html#acc35051d698e3973f1de5b9b78dbe345
  mat.setFromTriplets(_record.begin(), _record.end());
  return mat;
}

/**********************************************************************************************************
***********************************************************************************************************
                        Biclass - Fujitsu
***********************************************************************************************************
***********************************************************************************************************/

Mcdta_Api_Biclass::Mcdta_Api_Biclass()
{
  m_mcdta = nullptr;
  m_link_vec_count = std::vector<MNM_Dlink_Multiclass*>();
  m_link_vec_tt = std::vector<MNM_Dlink_Multiclass*>();
  m_link_vec_density = std::vector<MNM_Dlink_Multiclass*>();

  m_path_vec_car = std::vector<MNM_Path*>();
  m_path_set_car = std::set<MNM_Path*>(); 
  m_ID_path_mapping_car = std::unordered_map<TInt, MNM_Path*>();

  m_path_vec_truck = std::vector<MNM_Path*>();
  m_path_set_truck = std::set<MNM_Path*>();
  m_ID_path_mapping_truck = std::unordered_map<TInt, MNM_Path*>();
}

Mcdta_Api_Biclass::~Mcdta_Api_Biclass()
{
  if (m_mcdta != nullptr){
    delete m_mcdta;
  }
}

int Mcdta_Api_Biclass::initialize_biclass_sep(std::string folder)
{
  m_mcdta = new MNM_Dta_Multiclass(folder);
  m_mcdta -> build_from_files_control();
  m_mcdta -> hook_up_node_and_link();

  MNM_Routing_Biclass_Hybrid *_routing = dynamic_cast<MNM_Routing_Biclass_Hybrid *>(m_mcdta -> m_routing);

  MNM::get_ID_path_mapping(m_ID_path_mapping_car, _routing -> m_routing_fixed_car -> m_path_table);

  MNM::get_ID_path_mapping(m_ID_path_mapping_truck, _routing -> m_routing_fixed_truck -> m_path_table);

  return 0;
}

int Mcdta_Api_Biclass::preloading()
{
    m_mcdta -> pre_loading();
    return 0;
}

int Mcdta_Api_Biclass::run_once_control(int load_int, int assign_int)
{
  m_mcdta -> load_once_control(true, load_int, assign_int);
  return 0;
}

int Mcdta_Api_Biclass::run_whole_control()
{
  m_mcdta -> pre_loading();
  m_mcdta -> loading_control(true);
  return 0;
}

int Mcdta_Api_Biclass::run_whole_control_false()
{
  m_mcdta -> pre_loading();
  m_mcdta -> loading_control(false);
  return 0;
}

int Mcdta_Api_Biclass::register_paths_car(py::array_t<int> paths_car)
{
  if (m_path_vec_car.size() > 0){
    printf("Warning, Mcdta_Api_Biclass::register_paths_car, path exists\n");
    m_path_vec_car.clear();
    m_path_set_car.clear();
  }

  auto paths_buf = paths_car.request();
  if (paths_buf.ndim != 1){
    throw std::runtime_error("Mcdta_Api_Biclass::register_paths_car: Number of dimensions must be one");
  }

  int *paths_ptr = (int *) paths_buf.ptr; 
  TInt _path_ID;
  for (int i = 0; i < paths_buf.shape[0]; i++){
    _path_ID = TInt(paths_ptr[i]);
    if (m_ID_path_mapping_car.find(_path_ID) == m_ID_path_mapping_car.end()){
      throw std::runtime_error("Mcdta_Api_Biclass::register_paths_car: No such path");
    }
    else {
      m_path_vec_car.push_back(m_ID_path_mapping_car[_path_ID]);
    }
  }
  m_path_set_car = std::set<MNM_Path*> (m_path_vec_car.begin(), m_path_vec_car.end());
  return 0;
}

int Mcdta_Api_Biclass::register_paths_truck(py::array_t<int> paths_truck)
{
  if (m_path_vec_truck.size() > 0){
    printf("Warning, Mcdta_Api_Biclass::register_paths_truck, path exists\n");
    m_path_vec_truck.clear();
    m_path_set_truck.clear();
  }
  
  auto paths_buf = paths_truck.request();
  if (paths_buf.ndim != 1){
    throw std::runtime_error("Mcdta_Api_Biclass::register_paths_truck: Number of dimensions must be one");
  }

  int *paths_ptr = (int *) paths_buf.ptr; 
  TInt _path_ID;
  for (int i = 0; i < paths_buf.shape[0]; i++){
    _path_ID = TInt(paths_ptr[i]);
    if (m_ID_path_mapping_truck.find(_path_ID) == m_ID_path_mapping_truck.end()){
      throw std::runtime_error("Mcdta_Api_Biclass::register_paths_truck: No such path");
    }
    else {
      m_path_vec_truck.push_back(m_ID_path_mapping_truck[_path_ID]);
    }
  }
  m_path_set_truck = std::set<MNM_Path*> (m_path_vec_truck.begin(), m_path_vec_truck.end());
  return 0;
}


int Mcdta_Api_Biclass::register_links_count(py::array_t<int> links)
{
  if (m_link_vec_count.size() > 0){
    printf("Warning, Mcdta_Api_Biclass::register links_count, count link exists\n");
    m_link_vec_count.clear();
  }
  auto links_buf = links.request();
  if (links_buf.ndim != 1){
    throw std::runtime_error("Number of dimensions must be one");
  }
  int *links_ptr = (int *) links_buf.ptr;
  MNM_Dlink *_link;
  for (int i = 0; i < links_buf.shape[0]; i++){
    _link = m_mcdta -> m_link_factory -> get_link(TInt(links_ptr[i]));
    // printf("%d\n", links_ptr[i]);
    if (MNM_Dlink_Multiclass * _mclink = dynamic_cast<MNM_Dlink_Multiclass *>(_link)){
      if(std::find(m_link_vec_count.begin(), m_link_vec_count.end(), _link) != m_link_vec_count.end()) {
        throw std::runtime_error("Error, Mcdta_Api_Biclass::register links_count, count link does not exist");
      } 
      else {
        m_link_vec_count.push_back(_mclink);
      }
    }
    else{
      throw std::runtime_error("Mcdta_Api_Biclass::register links_count: count link type is not multiclass");
    }
  }
  return 0;
}

int Mcdta_Api_Biclass::register_links_tt(py::array_t<int> links)
{
  if (m_link_vec_tt.size() > 0){
    printf("Warning, Mcdta_Api_Biclass::register_links_tt, tt link exists\n");
    m_link_vec_tt.clear();
  }
  auto links_buf = links.request();
  if (links_buf.ndim != 1){
    throw std::runtime_error("Number of dimensions must be one");
  }
  int *links_ptr = (int *) links_buf.ptr;
  MNM_Dlink *_link;
  for (int i = 0; i < links_buf.shape[0]; i++){
    _link = m_mcdta -> m_link_factory -> get_link(TInt(links_ptr[i]));
    // printf("%d\n", links_ptr[i]);
    if (MNM_Dlink_Multiclass * _mclink = dynamic_cast<MNM_Dlink_Multiclass *>(_link)){
      if(std::find(m_link_vec_tt.begin(), m_link_vec_tt.end(), _link) != m_link_vec_tt.end()) {
        throw std::runtime_error("Error, Mcdta_Api_Biclass::register_links_tt, tt link does not exist");
      } 
      else {
        m_link_vec_tt.push_back(_mclink);
      }
    }
    else{
      throw std::runtime_error("Mcdta_Api_Biclass::register_links_tt: tt link type is not multiclass");
    }
  }
  return 0;
}

int Mcdta_Api_Biclass::register_links_density(py::array_t<int> links)
{
  if (m_link_vec_density.size() > 0){
    printf("Warning, Mcdta_Api_Biclass::register links density, curb link exists\n");
    m_link_vec_density.clear();
  }

  auto links_buf = links.request();
  if (links_buf.ndim != 1){
    throw std::runtime_error("Number of dimensions must be one");
  }

  int *links_ptr = (int *) links_buf.ptr;
  MNM_Dlink *_link;
  for (int i = 0; i < links_buf.shape[0]; i++){
    _link = m_mcdta -> m_link_factory -> get_link(TInt(links_ptr[i]));
    // printf("%d\n", links_ptr[i]);
    if (MNM_Dlink_Multiclass * _mclink = dynamic_cast<MNM_Dlink_Multiclass *>(_link)){
      // explain the following code

      if(std::find(m_link_vec_density.begin(), m_link_vec_density.end(), _link) != m_link_vec_density.end()) 
      {
        printf("current link id = %d\n", links_ptr[i]);
        throw std::runtime_error("Error, Mcdta_Api_Biclass::register links density, density link already exists");
      } 
      else {
        m_link_vec_density.push_back(_mclink);
      }
    }
    else{
      throw std::runtime_error("Mcdta_Api_Biclass::register links density: curb link type is not multiclass");
    }
  }
  return 0;
}

int Mcdta_Api_Biclass::install_cc_separate_with_trees_density()
{
  // count link
  for (size_t i = 0; i < m_link_vec_count.size(); ++i){
    m_link_vec_count[i] -> install_cumulative_curve_multiclass();
    m_link_vec_count[i] -> install_cumulative_curve_tree_multiclass();
  }

  // tt link
  for (size_t i = 0; i < m_link_vec_tt.size(); ++i){
    m_link_vec_tt[i] -> install_cumulative_curve_multiclass();
    m_link_vec_tt[i] -> install_cumulative_curve_tree_multiclass();
  }

  // density link
  for (size_t i = 0; i < m_link_vec_density.size(); ++i){
    m_link_vec_density[i] -> install_cumulative_curve_multiclass();
    m_link_vec_density[i] -> install_cumulative_curve_tree_multiclass();
  }
  
  return 0;
}

int Mcdta_Api_Biclass::get_cur_loading_interval()
{
  return m_mcdta -> m_current_loading_interval();
}

// count
py::array_t<double> Mcdta_Api_Biclass::get_link_inflow_car(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
{
  auto start_buf = start_intervals.request();
  auto end_buf = end_intervals.request();
  if (start_buf.ndim != 1 || end_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api_Biclass::get link_car_inflow_sep, input dimension mismatch");
  }
  if (start_buf.shape[0] != end_buf.shape[0]){
    throw std::runtime_error("Error, Mcdta_Api_Biclass::get link_car_inflow_sep, input length mismatch");
  }
  int l = start_buf.shape[0];
  int new_shape [2] = { (int) m_link_vec_count.size(), l};
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  int *start_prt = (int *) start_buf.ptr;
  int *end_prt = (int *) end_buf.ptr;
  for (int t = 0; t < l; ++t){
    if (end_prt[t] < start_prt[t]){
      throw std::runtime_error("Error, Mcdta_Api_Biclass::get link_car_inflow_sep, end time smaller than start time");
    }
    if (end_prt[t] > get_cur_loading_interval()){
      throw std::runtime_error("Error, Mcdta_Api_Biclass::get link_car_inflow_sep, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_count.size(); ++i){
      result_prt[i * l + t] = MNM_DTA_GRADIENT::get_link_inflow_car(m_link_vec_count[i], TFlt(start_prt[t]), TFlt(end_prt[t]))();
    }
  }
  return result;
}

py::array_t<double> Mcdta_Api_Biclass::get_link_inflow_truck(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
{
  auto start_buf = start_intervals.request();
  auto end_buf = end_intervals.request();
  if (start_buf.ndim != 1 || end_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api_Biclass::get_link_truck_inflow_sep, input dimension mismatch");
  }
  if (start_buf.shape[0] != end_buf.shape[0]){
    throw std::runtime_error("Error, Mcdta_Api_Biclass::get_link_truck_inflow_sep, input length mismatch");
  }
  int l = start_buf.shape[0];
  int new_shape [2] = { (int) m_link_vec_count.size(), l};
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  int *start_prt = (int *) start_buf.ptr;
  int *end_prt = (int *) end_buf.ptr;
  for (int t = 0; t < l; ++t){
    if (end_prt[t] < start_prt[t]){
      throw std::runtime_error("Error, Mcdta_Api_Biclass::get_link_truck_inflow_sep, end time smaller than start time");
    }
    if (end_prt[t] > get_cur_loading_interval()){
      throw std::runtime_error("Error, Mcdta_Api_Biclass::get_link_truck_inflow_sep, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_count.size(); ++i){
      result_prt[i * l + t] = MNM_DTA_GRADIENT::get_link_inflow_truck(m_link_vec_count[i], TFlt(start_prt[t]), TFlt(end_prt[t]))();
    }
  }
  return result;
}

py::array_t<double> Mcdta_Api_Biclass::get_link_outflow_car(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
{
  auto start_buf = start_intervals.request();
  auto end_buf = end_intervals.request();
  if (start_buf.ndim != 1 || end_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api_Biclass::, input dimension mismatch");
  }
  if (start_buf.shape[0] != end_buf.shape[0]){
    throw std::runtime_error("Error, Mcdta_Api_Biclass::, input length mismatch");
  }
  int l = start_buf.shape[0];
  int new_shape [2] = { (int) m_link_vec_count.size(), l};
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  int *start_prt = (int *) start_buf.ptr;
  int *end_prt = (int *) end_buf.ptr;
  for (int t = 0; t < l; ++t){
    if (end_prt[t] < start_prt[t]){
      throw std::runtime_error("Error, Mcdta_Api_Biclass::, end time smaller than start time");
    }
    if (end_prt[t] > get_cur_loading_interval()){
      throw std::runtime_error("Error, Mcdta_Api_Biclass::, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_count.size(); ++i){
      result_prt[i * l + t] = MNM_DTA_GRADIENT::get_link_outflow_car(m_link_vec_count[i], TFlt(start_prt[t]), TFlt(end_prt[t]))();
    }
  }
  return result;
}

py::array_t<double> Mcdta_Api_Biclass::get_link_outflow_truck(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
{
  auto start_buf = start_intervals.request();
  auto end_buf = end_intervals.request();
  if (start_buf.ndim != 1 || end_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api_Biclass::get_link_truck_inflow_sep, input dimension mismatch");
  }
  if (start_buf.shape[0] != end_buf.shape[0]){
    throw std::runtime_error("Error, Mcdta_Api_Biclass::get_link_truck_inflow_sep, input length mismatch");
  }
  int l = start_buf.shape[0];
  int new_shape [2] = { (int) m_link_vec_count.size(), l};
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  int *start_prt = (int *) start_buf.ptr;
  int *end_prt = (int *) end_buf.ptr;
  for (int t = 0; t < l; ++t){
    if (end_prt[t] < start_prt[t]){
      throw std::runtime_error("Error, Mcdta_Api_Biclass::get_link_truck_inflow_sep, end time smaller than start time");
    }
    if (end_prt[t] > get_cur_loading_interval()){
      throw std::runtime_error("Error, Mcdta_Api_Biclass::get_link_truck_inflow_sep, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_count.size(); ++i){
      result_prt[i * l + t] = MNM_DTA_GRADIENT::get_link_outflow_truck(m_link_vec_count[i], TFlt(start_prt[t]), TFlt(end_prt[t]))();
    }
  }
  return result;
}


py::array_t<double> Mcdta_Api_Biclass::get_link_tt_car(py::array_t<double>start_intervals)
{
  auto start_buf = start_intervals.request();
  if (start_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api_Biclass::get_car_link_tt_sep, input dimension mismatch");
  }
  int l = start_buf.shape[0];
  int new_shape [2] = { (int) m_link_vec_tt.size(), l}; 

  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  double *start_prt = (double *) start_buf.ptr;
  for (int t = 0; t < l; ++t){
    if (start_prt[t] > get_cur_loading_interval()){
      throw std::runtime_error("Error, Mcdta_Api_Biclass::get_car_link_tt_sep, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_tt.size(); ++i){
      double _tmp = MNM_DTA_GRADIENT::get_travel_time_car(m_link_vec_tt[i], TFlt(start_prt[t]), m_mcdta->m_unit_time, get_cur_loading_interval())();
      result_prt[i * l + t] = _tmp * m_mcdta -> m_unit_time;
    }
  }
  return result;
}

py::array_t<double> Mcdta_Api_Biclass::get_link_tt_truck(py::array_t<double>start_intervals)
{
  auto start_buf = start_intervals.request();
  if (start_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api_Biclass::get_truck_link_tt_sep, input dimension mismatch");
  }
  int l = start_buf.shape[0];
  int new_shape [2] = { (int) m_link_vec_tt.size(), l}; 

  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  double *start_prt = (double *) start_buf.ptr;
  for (int t = 0; t < l; ++t){
    if (start_prt[t] > get_cur_loading_interval()){
      throw std::runtime_error("Error, Mcdta_Api_Biclass::get_truck_link_tt_sep, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_tt.size(); ++i){
      double _tmp = MNM_DTA_GRADIENT::get_travel_time_truck(m_link_vec_tt[i], TFlt(start_prt[t]), m_mcdta -> m_unit_time, get_cur_loading_interval())();
      
      result_prt[i * l + t] = _tmp * m_mcdta -> m_unit_time;  // in unit of second (not interval)
    }
  }
  return result;
}

py::array_t<double> Mcdta_Api_Biclass::get_link_fftt_car(py::array_t<int>link_IDs)
{
    auto start_buf = link_IDs.request();
    if (start_buf.ndim != 1){
        throw std::runtime_error("Error, Mcdta_Api::get_link_fftt_car, input dimension mismatch");
    }
    int l = start_buf.shape[0];
    
    auto result = py::array_t<double>(l);
    auto result_buf = result.request();
    double *result_prt = (double *) result_buf.ptr;
    int *start_prt = (int *) start_buf.ptr;

    for (int i = 0; i < l; ++i) {
        result_prt[i] = dynamic_cast<MNM_Dlink_Multiclass*>(m_mcdta -> m_link_factory -> get_link(start_prt[i])) -> get_link_freeflow_tt_loading_car() * m_mcdta -> m_unit_time;  // seconds
    }
    return result;
}

py::array_t<double> Mcdta_Api_Biclass::get_link_fftt_truck(py::array_t<int>link_IDs)
{
    auto start_buf = link_IDs.request();
    if (start_buf.ndim != 1){
        throw std::runtime_error("Error, Mcdta_Api::get_link_fftt_truck, input dimension mismatch");
    }
    int l = start_buf.shape[0];
    
    auto result = py::array_t<double>(l);
    auto result_buf = result.request();
    double *result_prt = (double *) result_buf.ptr;
    int *start_prt = (int *) start_buf.ptr;

    for (int i = 0; i < l; ++i) {
        result_prt[i] = dynamic_cast<MNM_Dlink_Multiclass*>(m_mcdta -> m_link_factory -> get_link(start_prt[i])) -> get_link_freeflow_tt_loading_truck() * m_mcdta -> m_unit_time;  // seconds
    }
    return result;
}

py::array_t<double> Mcdta_Api_Biclass::get_link_density_car(py::array_t<int>timestamps)
{
  auto timestamp_buf = timestamps.request();
  if (timestamp_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api_Biclass::get_link_density_car, input dimension mismatch");
  }
  int l = timestamp_buf.shape[0];
  int new_shape [2] = { (int) m_link_vec_density.size(), l}; 

  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  int *timestamp_prt = (int *) timestamp_buf.ptr;

  for (int t = 0; t < l; ++t){
    if (timestamp_prt[t] > get_cur_loading_interval()){
      throw std::runtime_error("Error, Mcdta_Api_Biclass::get_link_density_car, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_density.size(); ++i){
      double _density = MNM_DTA_GRADIENT::get_link_density_car(m_link_vec_density[i], TFlt(timestamp_prt[t]), m_mcdta -> m_current_loading_interval()); // seconds
      result_prt[i * l + t] = _density; // vehicle number (not real density, real density should be divided by link length)
    }
  }
  return result;
}

py::array_t<double> Mcdta_Api_Biclass::get_link_density_truck(py::array_t<int>timestamps)
{
  auto timestamp_buf = timestamps.request();
  if (timestamp_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api_Biclass::get_link_density_truck, input dimension mismatch");
  }
  int l = timestamp_buf.shape[0];
  int new_shape [2] = { (int) m_link_vec_density.size(), l}; 

  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  int *timestamp_prt = (int *) timestamp_buf.ptr;

  for (int t = 0; t < l; ++t){
    if (timestamp_prt[t] > get_cur_loading_interval()){
      throw std::runtime_error("Error, Mcdta_Api_Biclass::get_link_density_truck, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_density.size(); ++i){
      double _density = MNM_DTA_GRADIENT::get_link_density_truck(m_link_vec_density[i], TFlt(timestamp_prt[t]), m_mcdta -> m_current_loading_interval()); // seconds
      result_prt[i * l + t] = _density; // vehicle number (not real density, real density should be divided by link length)
    }
  }
  return result;
}

py::array_t<double> Mcdta_Api_Biclass::get_link_density_car_robust(pybind11::array_t<int> timestamps)
{
  auto timestamp_buf = timestamps.request();
  if (timestamp_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api_Biclass::get_link_density_car_robust, input dimension mismatch");
  }
  int l = timestamp_buf.shape[0];
  int new_shape [2] = { (int) m_link_vec_density.size(), l}; 

  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  int *timestamp_prt = (int *) timestamp_buf.ptr;

  for (int t = 0; t < l; ++t){
    if (timestamp_prt[t] > get_cur_loading_interval()){
      throw std::runtime_error("Error, Mcdta_Api_Biclass::get_link_density_car_robust, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_density.size(); ++i){
      double _density = MNM_DTA_GRADIENT::get_link_density_car_robust(m_link_vec_density[i], TFlt(timestamp_prt[t]), m_mcdta -> m_current_loading_interval(), TInt(5)); // seconds
      result_prt[i * l + t] = _density; // vehicle number (not real density, real density should be divided by link length)
    }
  }
  return result;
}

py::array_t<double> Mcdta_Api_Biclass::get_link_density_truck_robust(pybind11::array_t<int> timestamps)
{
  auto timestamp_buf = timestamps.request();
  if (timestamp_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api_Biclass::get_link_density_truck_robust, input dimension mismatch");
  }
  int l = timestamp_buf.shape[0];
  int new_shape [2] = { (int) m_link_vec_density.size(), l}; 

  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  int *timestamp_prt = (int *) timestamp_buf.ptr;

  for (int t = 0; t < l; ++t){
    if (timestamp_prt[t] > get_cur_loading_interval()){
      throw std::runtime_error("Error, Mcdta_Api_Biclass::get_link_density_truck_robust, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_density.size(); ++i){
      double _density = MNM_DTA_GRADIENT::get_link_density_truck_robust(m_link_vec_density[i], TFlt(timestamp_prt[t]), m_mcdta -> m_current_loading_interval(), TInt(5)); // seconds
      result_prt[i * l + t] = _density; // vehicle number (not real density, real density should be divided by link length)
    }
  }
  return result;
}

// count DAR
py::array_t<double> Mcdta_Api_Biclass::get_car_dar_matrix_count(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
{
  auto start_buf = start_intervals.request();
  auto end_buf = end_intervals.request();
  if (start_buf.ndim != 1 || end_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::, input dimension mismatch");
  }
  if (start_buf.shape[0] != end_buf.shape[0]){
    throw std::runtime_error("Error, Mcdta_Api::, input length mismatch");
  }

  int l = start_buf.shape[0];
  int *start_prt = (int *) start_buf.ptr;
  int *end_prt = (int *) end_buf.ptr;
  std::vector<dar_record*> _record = std::vector<dar_record*>();

  for (int t = 0; t < l; ++t){
    // printf("Current processing time: %d\n", t);
    if (end_prt[t] < start_prt[t]){
        throw std::runtime_error("Error, Mcdta_Api::, end time smaller than start time");
    }
    if (end_prt[t] > get_cur_loading_interval()){
        throw std::runtime_error("Error, Mcdta_Api::, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_count.size(); ++i){
      MNM_DTA_GRADIENT::add_dar_records_car(_record, m_link_vec_count[i], m_path_set_car, TFlt(start_prt[t]), TFlt(end_prt[t]));
    }
  }
  // _record.size() = num_timesteps x num_links x num_path x num_assign_timesteps
  // path_ID, assign_time, link_ID, start_int, flow
  int new_shape [2] = { (int) _record.size(), 5}; 
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  dar_record* tmp_record;
  for (size_t i = 0; i < _record.size(); ++i){
    tmp_record = _record[i];
    result_prt[i * 5 + 0] = (double) tmp_record -> path_ID();
    // the count of 1 min interval
    result_prt[i * 5 + 1] = (double) tmp_record -> assign_int();
    result_prt[i * 5 + 2] = (double) tmp_record -> link_ID();
    // the count of unit time interval (5s)
    result_prt[i * 5 + 3] = (double) tmp_record -> link_start_int();
    result_prt[i * 5 + 4] = tmp_record -> flow();
  }
  for (size_t i = 0; i < _record.size(); ++i){
    delete _record[i];
  }
  _record.clear();
  return result;
}

py::array_t<double> Mcdta_Api_Biclass::get_truck_dar_matrix_count(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
{
  auto start_buf = start_intervals.request();
  auto end_buf = end_intervals.request();
  if (start_buf.ndim != 1 || end_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_matrix_sep, input dimension mismatch");
  }
  if (start_buf.shape[0] != end_buf.shape[0]){
    throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_matrix_sep, input length mismatch");
  }
  int l = start_buf.shape[0];
  int *start_prt = (int *) start_buf.ptr;
  int *end_prt = (int *) end_buf.ptr;
  std::vector<dar_record*> _record = std::vector<dar_record*>();

  for (int t = 0; t < l; ++t){
    if (end_prt[t] < start_prt[t]){
        throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_matrix_sep, end time smaller than start time");
    }
    if (end_prt[t] > get_cur_loading_interval()){
        throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_matrix_sep, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_count.size(); ++i){
        MNM_DTA_GRADIENT::add_dar_records_truck(_record, m_link_vec_count[i], m_path_set_truck, TFlt(start_prt[t]), TFlt(end_prt[t]));
    }
  }
  // path_ID, assign_time, link_ID, start_int, flow
  int new_shape [2] = { (int) _record.size(), 5}; 
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  dar_record* tmp_record;
  for (size_t i = 0; i < _record.size(); ++i){
    tmp_record = _record[i];
    result_prt[i * 5 + 0] = (double) tmp_record -> path_ID();
    // the count of 1 min interval
    result_prt[i * 5 + 1] = (double) tmp_record -> assign_int();
    result_prt[i * 5 + 2] = (double) tmp_record -> link_ID();
    // the count of unit time interval (5s)
    result_prt[i * 5 + 3] = (double) tmp_record -> link_start_int();
    result_prt[i * 5 + 4] = tmp_record -> flow();
  }
  for (size_t i = 0; i < _record.size(); ++i){
    delete _record[i];
  }
  _record.clear();
  return result;
}

// TT DAR
py::array_t<double> Mcdta_Api_Biclass::get_car_dar_matrix_tt(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
{
  auto start_buf = start_intervals.request();
  auto end_buf = end_intervals.request();
  if (start_buf.ndim != 1 || end_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get car_dar_matrix_tt, input dimension mismatch");
  }
  if (start_buf.shape[0] != end_buf.shape[0]){
    throw std::runtime_error("Error, Mcdta_Api::get car_dar_matrix_tt, input length mismatch");
  }

  int l = start_buf.shape[0];
  int *start_prt = (int *) start_buf.ptr;
  int *end_prt = (int *) end_buf.ptr;
  std::vector<dar_record*> _record = std::vector<dar_record*>();

  for (int t = 0; t < l; ++t){
    // printf("Current processing time: %d\n", t);
    if (end_prt[t] < start_prt[t]){
        throw std::runtime_error("Error, Mcdta_Api::get car_dar_matrix_tt, end time smaller than start time");
    }
    if (end_prt[t] > get_cur_loading_interval()){
        throw std::runtime_error("Error, Mcdta_Api::get car_dar_matrix_tt, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_tt.size(); ++i){
      MNM_DTA_GRADIENT::add_dar_records_car(_record, m_link_vec_tt[i], m_path_set_car, TFlt(start_prt[t]), TFlt(end_prt[t]));
    }
  }
  // _record.size() = num_timesteps x num_links x num_path x num_assign_timesteps
  // path_ID, assign_time, link_ID, start_int, flow
  int new_shape [2] = { (int) _record.size(), 5}; 
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  dar_record* tmp_record;
  for (size_t i = 0; i < _record.size(); ++i){
    tmp_record = _record[i];
    result_prt[i * 5 + 0] = (double) tmp_record -> path_ID();
    // the count of 1 min interval
    result_prt[i * 5 + 1] = (double) tmp_record -> assign_int();
    result_prt[i * 5 + 2] = (double) tmp_record -> link_ID();
    // the count of unit time interval (5s)
    result_prt[i * 5 + 3] = (double) tmp_record -> link_start_int();
    result_prt[i * 5 + 4] = tmp_record -> flow();
  }
  for (size_t i = 0; i < _record.size(); ++i){
    delete _record[i];
  }
  _record.clear();
  return result;
}

py::array_t<double> Mcdta_Api_Biclass::get_truck_dar_matrix_tt(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
{
  auto start_buf = start_intervals.request();
  auto end_buf = end_intervals.request();
  if (start_buf.ndim != 1 || end_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get truck_dar_matrix_tt, input dimension mismatch");
  }
  if (start_buf.shape[0] != end_buf.shape[0]){
    throw std::runtime_error("Error, Mcdta_Api::get truck_dar_matrix_tt, input length mismatch");
  }
  int l = start_buf.shape[0];
  int *start_prt = (int *) start_buf.ptr;
  int *end_prt = (int *) end_buf.ptr;
  std::vector<dar_record*> _record = std::vector<dar_record*>();

  for (int t = 0; t < l; ++t){
    if (end_prt[t] < start_prt[t]){
        throw std::runtime_error("Error, Mcdta_Api::get truck_dar_matrix_tt, end time smaller than start time");
    }
    if (end_prt[t] > get_cur_loading_interval()){
        throw std::runtime_error("Error, Mcdta_Api::get truck_dar_matrix_tt, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_tt.size(); ++i){
        MNM_DTA_GRADIENT::add_dar_records_truck(_record, m_link_vec_tt[i], m_path_set_truck, TFlt(start_prt[t]), TFlt(end_prt[t]));
    }
  }
  // path_ID, assign_time, link_ID, start_int, flow
  int new_shape [2] = { (int) _record.size(), 5}; 
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  dar_record* tmp_record;
  for (size_t i = 0; i < _record.size(); ++i){
    tmp_record = _record[i];
    result_prt[i * 5 + 0] = (double) tmp_record -> path_ID();
    // the count of 1 min interval
    result_prt[i * 5 + 1] = (double) tmp_record -> assign_int();
    result_prt[i * 5 + 2] = (double) tmp_record -> link_ID();
    // the count of unit time interval (5s)
    result_prt[i * 5 + 3] = (double) tmp_record -> link_start_int();
    result_prt[i * 5 + 4] = tmp_record -> flow();
  }
  for (size_t i = 0; i < _record.size(); ++i){
    delete _record[i];
  }
  _record.clear();
  return result;
}

// Fujitsu density DARs
py::array_t<double> Mcdta_Api_Biclass::get_car_dar_matrix_density_in(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
{
  auto start_buf = start_intervals.request();
  auto end_buf = end_intervals.request();
  if (start_buf.ndim != 1 || end_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get_car_dar_out_matrix, input dimension mismatch");
  }
  if (start_buf.shape[0] != end_buf.shape[0]){
    throw std::runtime_error("Error, Mcdta_Api::get_car_dar_out_matrix, input length mismatch");
  }
  int l = start_buf.shape[0];
  int *start_prt = (int *) start_buf.ptr;
  int *end_prt = (int *) end_buf.ptr;
  std::vector<dar_record*> _record = std::vector<dar_record*>();
  for (int t = 0; t < l; ++t){
    if (end_prt[t] < start_prt[t]){
      throw std::runtime_error("Error, Mcdta_Api::get_car_dar_out_matrix, end time smaller than start time");
    }
    if (end_prt[t] > get_cur_loading_interval()){
      throw std::runtime_error("Error, Mcdta_Api::get_car_dar_out_matrix, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_density.size(); ++i){
      MNM_DTA_GRADIENT::add_dar_records_car(_record, m_link_vec_density[i], m_path_set_car, TFlt(start_prt[t]), TFlt(end_prt[t]));
    }
  }
  // _record.size() = num_timesteps x num_links x num_path x num_assign_timesteps
  // path_ID, assign_time, link_ID, start_int, flow
  int new_shape [2] = { (int) _record.size(), 5}; 
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  dar_record* tmp_record;
  for (size_t i = 0; i < _record.size(); ++i){
    tmp_record = _record[i];
    result_prt[i * 5 + 0] = (double) tmp_record -> path_ID();
    // the count of 1 min interval
    result_prt[i * 5 + 1] = (double) tmp_record -> assign_int();
    result_prt[i * 5 + 2] = (double) tmp_record -> link_ID();
    // the count of unit time interval (5s)
    result_prt[i * 5 + 3] = (double) tmp_record -> link_start_int();
    result_prt[i * 5 + 4] = tmp_record -> flow();
  }
  for (size_t i = 0; i < _record.size(); ++i){
    delete _record[i];
  }
  _record.clear();
  return result;
}

py::array_t<double> Mcdta_Api_Biclass::get_truck_dar_matrix_density_in(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
{
  auto start_buf = start_intervals.request();
  auto end_buf = end_intervals.request();
  if (start_buf.ndim != 1 || end_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_out_matrix, input dimension mismatch");
  }
  if (start_buf.shape[0] != end_buf.shape[0]){
    throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_out_matrix, input length mismatch");
  }
  int l = start_buf.shape[0];
  int *start_prt = (int *) start_buf.ptr;
  int *end_prt = (int *) end_buf.ptr;
  std::vector<dar_record*> _record = std::vector<dar_record*>();
  for (int t = 0; t < l; ++t){
    if (end_prt[t] < start_prt[t]){
      throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_out_matrix, end time smaller than start time");
    }
    if (end_prt[t] > get_cur_loading_interval()){
      throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_out_matrix, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_density.size(); ++i){
      MNM_DTA_GRADIENT::add_dar_records_truck(_record, m_link_vec_density[i], m_path_set_truck, TFlt(start_prt[t]), TFlt(end_prt[t]));
    }
  }
  // path_ID, assign_time, link_ID, start_int, flow
  int new_shape [2] = { (int) _record.size(), 5}; 
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  dar_record* tmp_record;
  for (size_t i = 0; i < _record.size(); ++i){
    tmp_record = _record[i];
    result_prt[i * 5 + 0] = (double) tmp_record -> path_ID();
    // the count of 1 min interval
    result_prt[i * 5 + 1] = (double) tmp_record -> assign_int();
    result_prt[i * 5 + 2] = (double) tmp_record -> link_ID();
    // the count of unit time interval (5s)
    result_prt[i * 5 + 3] = (double) tmp_record -> link_start_int();
    result_prt[i * 5 + 4] = tmp_record -> flow();
  }
  for (size_t i = 0; i < _record.size(); ++i){
    delete _record[i];
  }
  _record.clear();
  return result;
}

py::array_t<double> Mcdta_Api_Biclass::get_car_dar_matrix_density_out(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
{
  auto start_buf = start_intervals.request();
  auto end_buf = end_intervals.request();
  if (start_buf.ndim != 1 || end_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get_car_dar_out_matrix, input dimension mismatch");
  }
  if (start_buf.shape[0] != end_buf.shape[0]){
    throw std::runtime_error("Error, Mcdta_Api::get_car_dar_out_matrix, input length mismatch");
  }
  int l = start_buf.shape[0];
  int *start_prt = (int *) start_buf.ptr;
  int *end_prt = (int *) end_buf.ptr;
  std::vector<dar_record*> _record = std::vector<dar_record*>();
  for (int t = 0; t < l; ++t){
    if (end_prt[t] < start_prt[t]){
      throw std::runtime_error("Error, Mcdta_Api::get_car_dar_out_matrix, end time smaller than start time");
    }
    if (end_prt[t] > get_cur_loading_interval()){
      throw std::runtime_error("Error, Mcdta_Api::get_car_dar_out_matrix, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_density.size(); ++i){
      MNM_DTA_GRADIENT::add_dar_records_car_out(_record, m_link_vec_density[i], m_path_set_car, TFlt(start_prt[t]), TFlt(end_prt[t]));
    }
  }
  // _record.size() = num_timesteps x num_links x num_path x num_assign_timesteps
  // path_ID, assign_time, link_ID, start_int, flow
  int new_shape [2] = { (int) _record.size(), 5}; 
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  dar_record* tmp_record;
  for (size_t i = 0; i < _record.size(); ++i){
    tmp_record = _record[i];
    result_prt[i * 5 + 0] = (double) tmp_record -> path_ID();
    // the count of 1 min interval
    result_prt[i * 5 + 1] = (double) tmp_record -> assign_int();
    result_prt[i * 5 + 2] = (double) tmp_record -> link_ID();
    // the count of unit time interval (5s)
    result_prt[i * 5 + 3] = (double) tmp_record -> link_start_int();
    result_prt[i * 5 + 4] = tmp_record -> flow();
  }
  for (size_t i = 0; i < _record.size(); ++i){
    delete _record[i];
  }
  _record.clear();
  return result;
}

py::array_t<double> Mcdta_Api_Biclass::get_truck_dar_matrix_density_out(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
{
  auto start_buf = start_intervals.request();
  auto end_buf = end_intervals.request();
  if (start_buf.ndim != 1 || end_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_out_matrix, input dimension mismatch");
  }
  if (start_buf.shape[0] != end_buf.shape[0]){
    throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_out_matrix, input length mismatch");
  }
  int l = start_buf.shape[0];
  int *start_prt = (int *) start_buf.ptr;
  int *end_prt = (int *) end_buf.ptr;
  std::vector<dar_record*> _record = std::vector<dar_record*>();
  for (int t = 0; t < l; ++t){
    if (end_prt[t] < start_prt[t]){
      throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_out_matrix, end time smaller than start time");
    }
    if (end_prt[t] > get_cur_loading_interval()){
      throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_out_matrix, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_density.size(); ++i){
      MNM_DTA_GRADIENT::add_dar_records_truck_out(_record, m_link_vec_density[i], m_path_set_truck, TFlt(start_prt[t]), TFlt(end_prt[t]));
    }
  }
  // path_ID, assign_time, link_ID, start_int, flow
  int new_shape [2] = { (int) _record.size(), 5}; 
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  dar_record* tmp_record;
  for (size_t i = 0; i < _record.size(); ++i){
    tmp_record = _record[i];
    result_prt[i * 5 + 0] = (double) tmp_record -> path_ID();
    // the count of 1 min interval
    result_prt[i * 5 + 1] = (double) tmp_record -> assign_int();
    result_prt[i * 5 + 2] = (double) tmp_record -> link_ID();
    // the count of unit time interval (5s)
    result_prt[i * 5 + 3] = (double) tmp_record -> link_start_int();
    result_prt[i * 5 + 4] = tmp_record -> flow();
  }
  for (size_t i = 0; i < _record.size(); ++i){
    delete _record[i];
  }
  _record.clear();
  return result;
}

/**********************************************************************************************************
***********************************************************************************************************
                        Multiclass
***********************************************************************************************************
***********************************************************************************************************/

Mcdta_Api::Mcdta_Api()
{
  m_mcdta = nullptr;
  m_link_vec = std::vector<MNM_Dlink_Multiclass*>();
  m_link_vec_count = std::vector<MNM_Dlink_Multiclass*>();
  m_link_vec_tt = std::vector<MNM_Dlink_Multiclass*>();
  m_link_vec_density = std::vector<MNM_Dlink_Multiclass*>();
  m_link_vec_curb = std::vector<MNM_Dlink_Multiclass*>();

  m_path_vec = std::vector<MNM_Path*>();
  m_path_set = std::set<MNM_Path*>(); 
  m_ID_path_mapping = std::unordered_map<TInt, MNM_Path*>();

  m_path_vec_truck = std::vector<MNM_Path*>();
  m_path_set_truck = std::set<MNM_Path*>();
  m_ID_path_mapping_truck = std::unordered_map<TInt, MNM_Path*>();

  m_path_vec_rh = std::vector<MNM_Path*>();
  m_path_set_rh = std::set<MNM_Path*>();
  m_ID_path_mapping_rh = std::unordered_map<TInt, MNM_Path*>();

}

Mcdta_Api::~Mcdta_Api()
{
  if (m_mcdta != nullptr){
    delete m_mcdta;
  }
  m_link_vec.clear();
  m_path_vec.clear();
  
}

int Mcdta_Api::initialize(std::string folder)
{
  m_mcdta = new MNM_Dta_Multiclass_Curb(folder);
  m_mcdta -> build_from_files();
  m_mcdta -> hook_up_node_and_link();
  m_mcdta -> is_ok();
  if (MNM_Routing_Fixed *_routing = dynamic_cast<MNM_Routing_Fixed *>(m_mcdta -> m_routing)){
    MNM::get_ID_path_mapping(m_ID_path_mapping, _routing -> m_path_table);
    return 0;
  }
  if (MNM_Routing_Hybrid *_routing = dynamic_cast<MNM_Routing_Hybrid *>(m_mcdta -> m_routing)){
    // printf("start load ID path mapping\n");
    MNM::get_ID_path_mapping(m_ID_path_mapping, _routing -> m_routing_fixed -> m_path_table);
    return 0;
    // printf("mapping size %d\n", m_ID_path_mapping.size());
  }
  if (MNM_Routing_Biclass_Hybrid *_routing = dynamic_cast<MNM_Routing_Biclass_Hybrid *>(m_mcdta -> m_routing)){
    printf("MNM_Routing_Biclass_Hybrid start load ID path mapping\n");
    MNM::get_ID_path_mapping(m_ID_path_mapping, _routing -> m_routing_fixed_car -> m_path_table);
    printf("MNM_Routing_Biclass_Hybrid mapping size %d\n", (int)m_ID_path_mapping.size());
    return 0;
  }
  printf("xxx\n");
  std::runtime_error("Mcdta_Api:: Routing type not implemented in API");
  return -1;
}

int Mcdta_Api::initialize_curb(std::string folder)
{
  // Jiachao change to multiclass-curb class
  m_mcdta = new MNM_Dta_Multiclass_Curb(folder);
  m_mcdta -> build_from_files_separate();
  m_mcdta -> hook_up_node_and_link();
  // m_mcdta -> is_ok();

  MNM_Routing_Biclass_Hybrid_Curb *_routing = dynamic_cast<MNM_Routing_Biclass_Hybrid_Curb *>(m_mcdta -> m_routing);

  MNM::get_ID_path_mapping(m_ID_path_mapping, _routing -> m_routing_fixed_car -> m_path_table);

  MNM::get_ID_path_mapping(m_ID_path_mapping_truck, _routing -> m_routing_fixed_truck -> m_path_table);

  MNM::get_ID_path_mapping(m_ID_path_mapping_rh, _routing -> m_routing_fixed_ridehail -> m_path_table);
  return 0;
}

int Mcdta_Api::install_cc()
{
  for (size_t i = 0; i < m_link_vec.size(); ++i){
    m_link_vec[i] -> install_cumulative_curve_multiclass();
  }
  return 0;
}

int Mcdta_Api::install_cc_tree()
{
  for (size_t i = 0; i < m_link_vec.size(); ++i){
    m_link_vec[i] -> install_cumulative_curve_tree_multiclass();
    m_link_vec[i] -> install_cumulative_curve_tree_multiclass_curb();
  }
  return 0;
}

int Mcdta_Api::run_whole()
{
  m_mcdta -> pre_loading();
  m_mcdta -> loading(true);
  return 0;
}

// jiachao added May 02
int Mcdta_Api::preloading()
{
    m_mcdta -> pre_loading();
    return 0;
}

int Mcdta_Api::run_once_control(int load_int, int assign_int)
{
  m_mcdta_bi -> load_once_control(true, load_int, assign_int);
  return 0;
}

int Mcdta_Api::run_whole_control()
{
  m_mcdta_bi -> pre_loading();
  m_mcdta_bi -> loading_control(true);
  return 0;
}

int Mcdta_Api::run_whole_control_false()
{
  m_mcdta_bi -> pre_loading();
  m_mcdta_bi -> loading_control(false);
  return 0;
}

// jiachao
int Mcdta_Api::run_whole_curb()
{
  m_mcdta -> pre_loading();
  m_mcdta -> loading_curb(true);
  delete m_mcdta->m_veh_factory;
  m_mcdta->m_veh_factory = nullptr;
  return 0;
}

int Mcdta_Api::run_whole_curb_false()
{
  m_mcdta -> pre_loading();
  m_mcdta -> loading_curb(false);
  delete m_mcdta->m_veh_factory;
  m_mcdta->m_veh_factory = nullptr;
  return 0;
}

int Mcdta_Api::register_links(py::array_t<int> links)
{
  if (m_link_vec.size() > 0){
    printf("Warning, Mcdta_Api::register_links, link exists\n");
    m_link_vec.clear();
  }
  auto links_buf = links.request();
  if (links_buf.ndim != 1){
    throw std::runtime_error("Number of dimensions must be one");
  }
  int *links_ptr = (int *) links_buf.ptr;
  MNM_Dlink *_link;
  for (int i = 0; i < links_buf.shape[0]; i++){
    _link = m_mcdta -> m_link_factory -> get_link(TInt(links_ptr[i]));
    // printf("%d\n", links_ptr[i]);
    if (MNM_Dlink_Multiclass * _mclink = dynamic_cast<MNM_Dlink_Multiclass *>(_link)){
      if(std::find(m_link_vec.begin(), m_link_vec.end(), _link) != m_link_vec.end()) {
        throw std::runtime_error("Error, Mcdta_Api::register_links, link does not exist");
      } 
      else {
        m_link_vec.push_back(_mclink);
      }
    }
    else{
      throw std::runtime_error("Mcdta_Api::register_links: link type is not multiclass");
    }
  }
  return 0;
}

int Mcdta_Api::initialize_curb_sep(std::string folder)
{
  m_mcdta = new MNM_Dta_Multiclass_Curb(folder);
  m_mcdta -> build_from_files_separate();
  m_mcdta -> hook_up_node_and_link();
  m_mcdta -> is_ok();

  MNM_Routing_Biclass_Hybrid_Curb *_routing = dynamic_cast<MNM_Routing_Biclass_Hybrid_Curb *>(m_mcdta -> m_routing);

  MNM::get_ID_path_mapping(m_ID_path_mapping, _routing -> m_routing_fixed_car -> m_path_table);

  MNM::get_ID_path_mapping(m_ID_path_mapping_truck, _routing -> m_routing_fixed_truck -> m_path_table);

  MNM::get_ID_path_mapping(m_ID_path_mapping_rh, _routing -> m_routing_fixed_ridehail -> m_path_table);

  return 0;
}

int Mcdta_Api::initialize_biclass_sep(std::string folder)
{
  m_mcdta_bi = new MNM_Dta_Multiclass(folder);
  m_mcdta_bi -> build_from_files_control();
  m_mcdta_bi -> hook_up_node_and_link();

  MNM_Routing_Biclass_Hybrid *_routing = dynamic_cast<MNM_Routing_Biclass_Hybrid *>(m_mcdta_bi -> m_routing);

  MNM::get_ID_path_mapping(m_ID_path_mapping, _routing -> m_routing_fixed_car -> m_path_table);

  MNM::get_ID_path_mapping(m_ID_path_mapping_truck, _routing -> m_routing_fixed_truck -> m_path_table);

  return 0;
}

// jiachao 0605
int Mcdta_Api::register_paths_car(py::array_t<int> paths_car)
{
  if (m_path_vec.size() > 0){
    printf("Warning, Mcdta_Api::register_paths_car, path exists\n");
    m_path_vec.clear();
    m_path_set.clear();
  }

  auto paths_buf = paths_car.request();
  if (paths_buf.ndim != 1){
    throw std::runtime_error("Mcdta_Api::register_paths_car: Number of dimensions must be one");
  }

  int *paths_ptr = (int *) paths_buf.ptr; 
  TInt _path_ID;
  for (int i = 0; i < paths_buf.shape[0]; i++){
    _path_ID = TInt(paths_ptr[i]);
    if (m_ID_path_mapping.find(_path_ID) == m_ID_path_mapping.end()){
      throw std::runtime_error("Mcdta_Api::register_paths_car: No such path");
    }
    else {
      m_path_vec.push_back(m_ID_path_mapping[_path_ID]);
    }
  }
  m_path_set = std::set<MNM_Path*> (m_path_vec.begin(), m_path_vec.end());
  return 0;
}

int Mcdta_Api::register_paths_truck(py::array_t<int> paths_truck)
{
  if (m_path_vec_truck.size() > 0){
    printf("Warning, Mcdta_Api::register_paths_truck, path exists\n");
    m_path_vec_truck.clear();
    m_path_set_truck.clear();
  }
  
  auto paths_buf = paths_truck.request();
  if (paths_buf.ndim != 1){
    throw std::runtime_error("Mcdta_Api::register_paths_truck: Number of dimensions must be one");
  }

  int *paths_ptr = (int *) paths_buf.ptr; 
  TInt _path_ID;
  for (int i = 0; i < paths_buf.shape[0]; i++){
    _path_ID = TInt(paths_ptr[i]);
    if (m_ID_path_mapping_truck.find(_path_ID) == m_ID_path_mapping_truck.end()){
      throw std::runtime_error("Mcdta_Api::register_paths_truck: No such path");
    }
    else {
      m_path_vec_truck.push_back(m_ID_path_mapping_truck[_path_ID]);
    }
  }
  m_path_set_truck = std::set<MNM_Path*> (m_path_vec_truck.begin(), m_path_vec_truck.end());
  return 0;
}

int Mcdta_Api::register_paths_rh(py::array_t<int> paths_rh)
{
  if (m_path_vec_rh.size() > 0){
    printf("Warning, Mcdta_Api::register_paths_rh, path exists\n");
    m_path_vec_rh.clear();
    m_path_set_rh.clear();
  } 
  
  auto paths_buf = paths_rh.request();
  if (paths_buf.ndim != 1){
    throw std::runtime_error("Mcdta_Api::register_paths_rh: Number of dimensions must be one");
  }

  int *paths_ptr = (int *) paths_buf.ptr; 
  TInt _path_ID;
  for (int i = 0; i < paths_buf.shape[0]; i++){
    _path_ID = TInt(paths_ptr[i]);
    if (m_ID_path_mapping_rh.find(_path_ID) == m_ID_path_mapping_rh.end()){
      throw std::runtime_error("Mcdta_Api::register_paths_rh: No such path");
    }
    else {
      m_path_vec_rh.push_back(m_ID_path_mapping_rh[_path_ID]);
    }
  }
  m_path_set_rh = std::set<MNM_Path*> (m_path_vec_rh.begin(), m_path_vec_rh.end());
  return 0;
}

int Mcdta_Api::register_links_count(py::array_t<int> links)
{
  if (m_link_vec_count.size() > 0){
    printf("Warning, Mcdta_Api::register links_count, count link exists\n");
    m_link_vec_count.clear();
  }
  auto links_buf = links.request();
  if (links_buf.ndim != 1){
    throw std::runtime_error("Number of dimensions must be one");
  }
  int *links_ptr = (int *) links_buf.ptr;
  MNM_Dlink *_link;
  for (int i = 0; i < links_buf.shape[0]; i++){
    _link = m_mcdta -> m_link_factory -> get_link(TInt(links_ptr[i]));
    // printf("%d\n", links_ptr[i]);
    if (MNM_Dlink_Multiclass * _mclink = dynamic_cast<MNM_Dlink_Multiclass *>(_link)){
      if(std::find(m_link_vec_count.begin(), m_link_vec_count.end(), _link) != m_link_vec_count.end()) {
        throw std::runtime_error("Error, Mcdta_Api::register links_count, count link does not exist");
      } 
      else {
        m_link_vec_count.push_back(_mclink);
      }
    }
    else{
      throw std::runtime_error("Mcdta_Api::register links_count: count link type is not multiclass");
    }
  }
  return 0;
}

int Mcdta_Api::register_links_tt(py::array_t<int> links)
{
  if (m_link_vec_tt.size() > 0){
    printf("Warning, Mcdta_Api::register_links_tt, tt link exists\n");
    m_link_vec_tt.clear();
  }
  auto links_buf = links.request();
  if (links_buf.ndim != 1){
    throw std::runtime_error("Number of dimensions must be one");
  }
  int *links_ptr = (int *) links_buf.ptr;
  MNM_Dlink *_link;
  for (int i = 0; i < links_buf.shape[0]; i++){
    _link = m_mcdta -> m_link_factory -> get_link(TInt(links_ptr[i]));
    // printf("%d\n", links_ptr[i]);
    if (MNM_Dlink_Multiclass * _mclink = dynamic_cast<MNM_Dlink_Multiclass *>(_link)){
      if(std::find(m_link_vec_tt.begin(), m_link_vec_tt.end(), _link) != m_link_vec_tt.end()) {
        throw std::runtime_error("Error, Mcdta_Api::register_links_tt, tt link does not exist");
      } 
      else {
        m_link_vec_tt.push_back(_mclink);
      }
    }
    else{
      throw std::runtime_error("Mcdta_Api::register_links_tt: tt link type is not multiclass");
    }
  }
  return 0;
}

int Mcdta_Api::register_links_curb(py::array_t<int> links)
{
  if (m_link_vec_curb.size() > 0){
    printf("Warning, Mcdta_Api::register_links_curb, curb link exists\n");
    m_link_vec_curb.clear();
  }
  auto links_buf = links.request();
  if (links_buf.ndim != 1){
    throw std::runtime_error("Number of dimensions must be one");
  }
  int *links_ptr = (int *) links_buf.ptr;
  MNM_Dlink *_link;
  for (int i = 0; i < links_buf.shape[0]; i++){
    _link = m_mcdta -> m_link_factory -> get_link(TInt(links_ptr[i]));
    // printf("%d\n", links_ptr[i]);
    if (MNM_Dlink_Multiclass * _mclink = dynamic_cast<MNM_Dlink_Multiclass *>(_link)){
      if(std::find(m_link_vec_curb.begin(), m_link_vec_curb.end(), _link) != m_link_vec_curb.end()) {
        throw std::runtime_error("Error, Mcdta_Api::register_links_curb, curb link does not exist");
      } 
      else {
        m_link_vec_curb.push_back(_mclink);
      }
    }
    else{
      throw std::runtime_error("Mcdta_Api::register_links_curb: curb link type is not multiclass");
    }
  }
  return 0;
}

int Mcdta_Api::register_links_density(py::array_t<int> links)
{
  if (m_link_vec_density.size() > 0){
    printf("Warning, Mcdta_Api::register links density, curb link exists\n");
    m_link_vec_density.clear();
  }

  auto links_buf = links.request();
  if (links_buf.ndim != 1){
    throw std::runtime_error("Number of dimensions must be one");
  }

  int *links_ptr = (int *) links_buf.ptr;
  MNM_Dlink *_link;
  for (int i = 0; i < links_buf.shape[0]; i++){
    _link = m_mcdta -> m_link_factory -> get_link(TInt(links_ptr[i]));
    // printf("%d\n", links_ptr[i]);
    if (MNM_Dlink_Multiclass * _mclink = dynamic_cast<MNM_Dlink_Multiclass *>(_link)){
      if(std::find(m_link_vec_density.begin(), m_link_vec_density.end(), _link) != m_link_vec_density.end()) {
        throw std::runtime_error("Error, Mcdta_Api::register links density, curb link does not exist");
      } 
      else {
        m_link_vec_density.push_back(_mclink);
      }
    }
    else{
      throw std::runtime_error("Mcdta_Api::register links density: curb link type is not multiclass");
    }
  }
  return 0;
}

int Mcdta_Api::install_cc_separate_with_trees_curb()
{
  MNM_Dlink_Multiclass_Curb * _mclink_curb;

  // count link
  for (size_t i = 0; i < m_link_vec_count.size(); ++i){
    _mclink_curb = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(m_link_vec_count[i]);
    _mclink_curb -> install_cumulative_curve_multiclass();
    _mclink_curb -> install_cumulative_curve_tree_multiclass();
    _mclink_curb -> install_cumulative_curve_tree_multiclass_curb();
  }

  // tt link
  for (size_t i = 0; i < m_link_vec_tt.size(); ++i){
    _mclink_curb = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(m_link_vec_tt[i]);
    _mclink_curb -> install_cumulative_curve_multiclass();
    _mclink_curb -> install_cumulative_curve_tree_multiclass();
    _mclink_curb -> install_cumulative_curve_tree_multiclass_curb();
  }

  // curb link
  // tt link
  for (size_t i = 0; i < m_link_vec_curb.size(); ++i){
    _mclink_curb = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(m_link_vec_curb[i]);
    _mclink_curb -> install_cumulative_curve_multiclass();
    _mclink_curb -> install_cumulative_curve_tree_multiclass();
    _mclink_curb -> install_cumulative_curve_tree_multiclass_curb();
  }
}

int Mcdta_Api::install_cc_separate_with_trees_density()
{
  MNM_Dlink_Multiclass_Curb * _mclink_curb;
  // count link
  for (size_t i = 0; i < m_link_vec_count.size(); ++i){
    _mclink_curb = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(m_link_vec_count[i]);
    _mclink_curb -> install_cumulative_curve_multiclass();
    _mclink_curb -> install_cumulative_curve_tree_multiclass();
    _mclink_curb -> install_cumulative_curve_tree_multiclass_curb();
  }

  // tt link
  for (size_t i = 0; i < m_link_vec_tt.size(); ++i){
    _mclink_curb = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(m_link_vec_tt[i]);
    _mclink_curb -> install_cumulative_curve_multiclass();
    _mclink_curb -> install_cumulative_curve_tree_multiclass();
    _mclink_curb -> install_cumulative_curve_tree_multiclass_curb();
  }

  // density link
  for (size_t i = 0; i < m_link_vec_density.size(); ++i){
    _mclink_curb = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(m_link_vec_density[i]);
    _mclink_curb -> install_cumulative_curve_multiclass();
    _mclink_curb -> install_cumulative_curve_tree_multiclass();
    _mclink_curb -> install_cumulative_curve_tree_multiclass_curb();
  }
  
  return 0;
}

int Mcdta_Api::delete_mcdta()
{
  if (m_mcdta != nullptr){
    delete m_mcdta;
  }
  return 0;
}

int Mcdta_Api::install_cc_separate()
{
  // count link
  for (size_t i = 0; i < m_link_vec_count.size(); ++i){
    m_link_vec_count[i] -> install_cumulative_curve_multiclass();
  }

  // tt link
  for (size_t i = 0; i < m_link_vec_tt.size(); ++i){
    m_link_vec_tt[i] -> install_cumulative_curve_multiclass();
  }

  // curb link
  for (size_t i = 0; i < m_link_vec_curb.size(); ++i){
    m_link_vec_curb[i] -> install_cumulative_curve_multiclass();
  }

  return 0;
}

int Mcdta_Api::install_cc_tree_separate()
{
  // for count DAR
  for (size_t i = 0; i < m_link_vec_count.size(); ++i){
    m_link_vec_count[i] -> install_cumulative_curve_tree_multiclass();
  }

  // for TT DAR
  for (size_t i = 0; i < m_link_vec_tt.size(); ++i){
    m_link_vec_tt[i] -> install_cumulative_curve_tree_multiclass();
  }

  // for curb DAR
  for (size_t i = 0; i < m_link_vec_curb.size(); ++i){
    m_link_vec_curb[i] -> install_cumulative_curve_tree_multiclass_curb();
  }
  return 0;
}

// end add

int Mcdta_Api::get_cur_loading_interval()
{
  return m_mcdta -> m_current_loading_interval();
}

// Jiachao added in Feb 2023
int Mcdta_Api::build_link_cost_map(bool with_congestion_indicator)
{
    MNM_Dlink_Multiclass *_link;
    // TODO: what if not hybrid routing, better way to get vot
    // TFlt _vot = dynamic_cast<MNM_Routing_Biclass_Hybrid*>(m_mcdta -> m_routing) -> m_routing_adaptive -> m_vot;

    // for DoE curb, currently m_link_tt_map == m_link_cost_map
    TFlt _vot = 1.0;
    for (auto _link_it : m_mcdta->m_link_factory->m_link_map) {
        // #pragma omp task 
        _link = dynamic_cast<MNM_Dlink_Multiclass*>(_link_it.second);

        if (m_link_tt_map.find(_link_it.first) == m_link_tt_map.end()) {
            m_link_tt_map[_link_it.first] = new TFlt[get_cur_loading_interval()];
        }
        if (m_link_cost_map.find(_link_it.first) == m_link_cost_map.end()) {
            m_link_cost_map[_link_it.first] = new TFlt[get_cur_loading_interval()];
        }
        if (m_link_tt_map_truck.find(_link_it.first) == m_link_tt_map_truck.end()) {
            m_link_tt_map_truck[_link_it.first] = new TFlt[get_cur_loading_interval()];
        }
        if (m_link_cost_map_truck.find(_link_it.first) == m_link_cost_map_truck.end()) {
            m_link_cost_map_truck[_link_it.first] = new TFlt[get_cur_loading_interval()];
        }

        if (with_congestion_indicator) {
            if (m_link_congested_car.find(_link_it.first) == m_link_congested_car.end()) {
                m_link_congested_car[_link_it.first] = new bool[get_cur_loading_interval()];
            }
            if (m_link_congested_truck.find(_link_it.first) == m_link_congested_truck.end()) {
                m_link_congested_truck[_link_it.first] = new bool[get_cur_loading_interval()];
            }
        }
        
        // std::cout << "********************** build_link_cost_map link " << _link -> m_link_ID() << " **********************\n";
        for (int i = 0; i < get_cur_loading_interval(); i++) {
            
            m_link_tt_map[_link_it.first][i] = MNM_DTA_GRADIENT::get_travel_time_car(_link, TFlt(i+1), m_mcdta -> m_unit_time, get_cur_loading_interval());
            
            // m_link_cost_map[_link_it.first][i] = _vot * m_link_tt_map[_link_it.first][i] + _link -> m_toll;

            // toll is not needed here, so cost is pure TT
            m_link_cost_map[_link_it.first][i] = _vot * m_link_tt_map[_link_it.first][i];

            m_link_tt_map_truck[_link_it.first][i] = MNM_DTA_GRADIENT::get_travel_time_truck(_link, TFlt(i+1), m_mcdta -> m_unit_time, get_cur_loading_interval());
            
            // m_link_cost_map_truck[_link_it.first][i] = _vot * m_link_tt_map_truck[_link_it.first][i] + _link -> m_toll;
            m_link_cost_map_truck[_link_it.first][i] = _vot * m_link_tt_map_truck[_link_it.first][i];

            if (with_congestion_indicator) {
                m_link_congested_car[_link_it.first][i] = m_link_tt_map[_link_it.first][i] > _link -> get_link_freeflow_tt_loading_car();
                
                m_link_congested_truck[_link_it.first][i] = m_link_tt_map_truck[_link_it.first][i] > _link -> get_link_freeflow_tt_loading_truck();
            }
        }
    }
    return 0;
}

int Mcdta_Api::get_link_queue_dissipated_time()
{
    // suppose m_link_congested_car and m_link_congested_truck are constructed already in build_link_cost_map() // checked
    MNM_Dlink_Multiclass *_link;
    int _total_loading_inter = get_cur_loading_interval();
    IAssert(_total_loading_inter > 0);

    bool _flg;
    // std::cout << "\n********************** Begin get_link_queue_dissipated_time **********************\n";
    for (int i = 0; i < _total_loading_inter; i++) {
        // std::cout << "********************** get_link_queue_dissipated_time interval " << i << " **********************\n";
        for (auto _link_it : m_mcdta->m_link_factory->m_link_map) {

            // ************************** car **************************
            if (m_queue_dissipated_time_car.find(_link_it.first) == m_queue_dissipated_time_car.end()) {
                m_queue_dissipated_time_car[_link_it.first] = new int[_total_loading_inter];
            }

            // congested case
            if (m_link_congested_car[_link_it.first][i]) {
                // last interation
                if (i == _total_loading_inter - 1) {
                    m_queue_dissipated_time_car[_link_it.first][i] = _total_loading_inter;
                }
                else {
                    // check the first uncongested downstream link
                    _flg = false;
                    for (int k = i + 1; k < _total_loading_inter; k++) {
                        if (m_link_congested_car[_link_it.first][k - 1] && !m_link_congested_car[_link_it.first][k]) {
                            m_queue_dissipated_time_car[_link_it.first][i] = k;
                            _flg = true;
                            break;
                        }
                    }
                    if (!_flg) {
                        m_queue_dissipated_time_car[_link_it.first][i] = _total_loading_inter;
                    }
                }
            }
            // not congested case
            else {
                _link = dynamic_cast<MNM_Dlink_Multiclass*>(_link_it.second);
                if (MNM_Ults::approximate_equal(m_link_tt_map[_link_it.first][i], (float)_link -> get_link_freeflow_tt_loading_car())) {
                    // based on subgradient paper, when out flow = capacity and link tt = fftt, this is critical state where the subgradient applies
                    if (dynamic_cast<MNM_Dlink_Ctm_Multiclass*>(_link) != nullptr) {
                        // TODO: use spline to interpolate the N_out and extract the deriviative (out flow rate) and compare it with the capacity
                        // https://kluge.in-chemnitz.de/opensource/spline/spline.h
                        // tk::spline s;
                        // s.set_boundary(tk::spline::second_deriv, 0.0,
                        //                tk::spline::second_deriv, 0.0);
                        // s.set_points(X,Y,tk::spline::cspline);
                        // s.make_monotonic();
                        // s.deriv(1, X[i])

                        // need to add by Jiachao
                        TFlt _outflow_rate = MNM_DTA_GRADIENT::get_departure_cc_slope_car(_link, 
                                                                                          TFlt(i + (int)_link -> get_link_freeflow_tt_loading_car()), 
                                                                                          TFlt(i + (int)_link -> get_link_freeflow_tt_loading_car() + 1));  // veh / 5s
                        TFlt _cap = dynamic_cast<MNM_Dlink_Ctm_Multiclass*>(_link) -> m_cell_array.back() -> m_flow_cap_car * m_mcdta -> m_unit_time;  // veh / 5s
                        if (MNM_Ults::approximate_equal(_outflow_rate * m_mcdta -> m_flow_scalar, floor(_cap * m_mcdta -> m_flow_scalar))) {
                            if (i == _total_loading_inter - 1) {
                                m_queue_dissipated_time_car[_link_it.first][i] = _total_loading_inter;
                            }
                            else {
                                // to compute lift up time for the departure cc
                                _flg = false;
                                for (int k = i + 1; k < _total_loading_inter; k++) {
                                    if (m_link_congested_car[_link_it.first][k - 1] && !m_link_congested_car[_link_it.first][k]) {
                                        m_queue_dissipated_time_car[_link_it.first][i] = k;
                                        _flg = true;
                                        break;
                                    }
                                }
                                if (!_flg) {
                                    m_queue_dissipated_time_car[_link_it.first][i] = _total_loading_inter;
                                }
                            }
                        } 
                        else {
                            // TODO: boundary condition
                            m_queue_dissipated_time_car[_link_it.first][i] = i;
                        }
                    }
                    else if (dynamic_cast<MNM_Dlink_Pq_Multiclass*>(_link) != nullptr) {
                        // PQ link as OD connectors always has sufficient capacity
                        m_queue_dissipated_time_car[_link_it.first][i] = i;
                    }
                    else {
                        throw std::runtime_error("Mcdta_Api::get_link_queue_dissipated_time, Link type not implemented");
                    }
                }
                else {
                    // m_queue_dissipated_time_car[_link_it.first][i] = i;
                    throw std::runtime_error("Mcdta_Api::get_link_queue_dissipated_time, Link travel time less than fftt");
                }
                // m_queue_dissipated_time_car[_link_it.first][i] = i;
            }

            // ************************** truck **************************
            if (m_queue_dissipated_time_truck.find(_link_it.first) == m_queue_dissipated_time_truck.end()) {
                m_queue_dissipated_time_truck[_link_it.first] = new int[_total_loading_inter];
            }
            if (m_link_congested_truck[_link_it.first][i]) {
                if (i == _total_loading_inter - 1) {
                    m_queue_dissipated_time_truck[_link_it.first][i] = _total_loading_inter;
                }
                else {
                    _flg = false;
                    for (int k = i + 1; k < _total_loading_inter; k++) {
                        if (m_link_congested_truck[_link_it.first][k - 1] && !m_link_congested_truck[_link_it.first][k]) {
                            m_queue_dissipated_time_truck[_link_it.first][i] = k;
                            _flg = true;
                            break;
                        }
                    }
                    if (!_flg) {
                        m_queue_dissipated_time_truck[_link_it.first][i] = _total_loading_inter;
                    }
                }
            }
            else {
                _link = dynamic_cast<MNM_Dlink_Multiclass*>(_link_it.second);
                if (MNM_Ults::approximate_equal(m_link_tt_map_truck[_link_it.first][i], (float)_link -> get_link_freeflow_tt_loading_truck())) {
                    // based on subgradient paper, when out flow = capacity and link tt = fftt, this is critical state where the subgradient applies
                    if (dynamic_cast<MNM_Dlink_Ctm_Multiclass*>(_link) != nullptr) {
                        // TODO: use spline to interpolate the N_out and extract the deriviative (out flow rate) and compare it with the capacity
                        // https://kluge.in-chemnitz.de/opensource/spline/spline.h
                        // tk::spline s;
                        // s.set_boundary(tk::spline::second_deriv, 0.0,
                        //                tk::spline::second_deriv, 0.0);
                        // s.set_points(X,Y,tk::spline::cspline);
                        // s.make_monotonic();
                        // s.deriv(1, X[i])
                        TFlt _outflow_rate = MNM_DTA_GRADIENT::get_departure_cc_slope_truck(_link, 
                                                                                            TFlt(i + (int)_link -> get_link_freeflow_tt_loading_truck()), 
                                                                                            TFlt(i + (int)_link -> get_link_freeflow_tt_loading_truck() + 1));  // veh / 5s
                        TFlt _cap = dynamic_cast<MNM_Dlink_Ctm_Multiclass*>(_link) -> m_cell_array.back() -> m_flow_cap_truck * m_mcdta -> m_unit_time;  // veh / 5s
                        if (MNM_Ults::approximate_equal(_outflow_rate * m_mcdta -> m_flow_scalar, floor(_cap * m_mcdta -> m_flow_scalar))) {
                            if (i == _total_loading_inter - 1) {
                                m_queue_dissipated_time_truck[_link_it.first][i] = _total_loading_inter;
                            }
                            else {
                                // to compute lift up time for the departure cc
                                _flg = false;
                                for (int k = i + 1; k < _total_loading_inter; k++) {
                                    if (m_link_congested_truck[_link_it.first][k - 1] && !m_link_congested_truck[_link_it.first][k]) {
                                        m_queue_dissipated_time_truck[_link_it.first][i] = k;
                                        _flg = true;
                                        break;
                                    }
                                }
                                if (!_flg) {
                                    m_queue_dissipated_time_truck[_link_it.first][i] = _total_loading_inter;
                                }
                            }
                        } 
                        else {
                            // TODO: boundary condition
                            m_queue_dissipated_time_truck[_link_it.first][i] = i;
                        }
                    }
                    else if (dynamic_cast<MNM_Dlink_Pq_Multiclass*>(_link) != nullptr) {
                        // PQ link as OD connectors always has sufficient capacity
                        m_queue_dissipated_time_truck[_link_it.first][i] = i;
                    }
                    else {
                        throw std::runtime_error("Mcdta_Api::get_link_queue_dissipated_time, Link type not implemented");
                    }
                }
                else {
                    // m_queue_dissipated_time_truck[_link_it.first][i] = i;
                    throw std::runtime_error("Mcdta_Api::get_link_queue_dissipated_time, Link travel time less than fftt");
                }
                // m_queue_dissipated_time_truck[_link_it.first][i] = i;
            }
        }
    }
    // std::cout << "********************** End get_link_queue_dissipated_time **********************\n";
    return 0;
}
// added end

int Mcdta_Api::print_emission_stats()
{
  m_mcdta -> m_emission -> output();
  return 0;
}

int Mcdta_Api::print_simulation_results(std::string folder, int cong_frequency)
{
    bool output_link_cong = true; // if true output link congestion level every cong_frequency
    // TInt cong_frequency = 180; // 15 minutes

    MNM_Dlink *_link;
    MNM_Dlink_Multiclass *_link_m;
    std::string _str1;
    TInt _current_inter = m_mcdta -> m_current_loading_interval;
    std::ofstream _vis_file2;
    if (output_link_cong){
        _vis_file2.open(folder + "/driving_link_cong_raw.txt", std::ofstream::out);
        if (! _vis_file2.is_open()){
            printf("Error happens when open _vis_file2\n");
            exit(-1);
        }

        _str1 = "timestamp (intervals), driving_link_ID, car_inflow, truck_inflow, car_tt (s), truck_tt (s), car_fftt (s), truck_fftt (s), car_speed (mph), truck_speed (mph)\n";
        _vis_file2 << _str1;

        TInt _iter = 0;
        while (_iter + cong_frequency <= _current_inter){
            if (_iter % cong_frequency == 0 || _iter == _current_inter - 1){
                printf("Current loading interval: %d\n", int(_iter));
                for (auto _link_it : m_mcdta -> m_link_factory -> m_link_map){
                    _link = _link_it.second;
                    _link_m = dynamic_cast<MNM_Dlink_Multiclass*>(_link);
                    _str1 = std::to_string(int(_iter)) + " ";
                    _str1 += std::to_string(_link -> m_link_ID()) + " ";
                    _str1 += std::to_string(MNM_DTA_GRADIENT::get_link_inflow_car(_link_m, _iter, _iter+cong_frequency)) + " ";
                    _str1 += std::to_string(MNM_DTA_GRADIENT::get_link_inflow_truck(_link_m, _iter, _iter+cong_frequency)) + " ";
                    _str1 += std::to_string(MNM_DTA_GRADIENT::get_travel_time_car(_link_m, TFlt(_iter), m_mcdta -> m_unit_time, get_cur_loading_interval()) * m_mcdta -> m_unit_time) + " ";
                    _str1 += std::to_string(MNM_DTA_GRADIENT::get_travel_time_truck(_link_m, TFlt(_iter), m_mcdta -> m_unit_time, get_cur_loading_interval()) * m_mcdta -> m_unit_time) + " ";
                    _str1 += std::to_string(_link_m -> get_link_freeflow_tt_car()) + " ";
                    _str1 += std::to_string(_link_m -> get_link_freeflow_tt_truck()) + " ";
                    _str1 += std::to_string(_link_m -> m_length/(MNM_DTA_GRADIENT::get_travel_time_car(_link_m, TFlt(_iter), m_mcdta -> m_unit_time, get_cur_loading_interval()) * m_mcdta -> m_unit_time) * 3600 / 1600) + " ";
                    _str1 += std::to_string(_link_m -> m_length/(MNM_DTA_GRADIENT::get_travel_time_truck(_link_m, TFlt(_iter), m_mcdta -> m_unit_time, get_cur_loading_interval()) * m_mcdta -> m_unit_time) * 3600 / 1600) + "\n";
                    _vis_file2 << _str1;
                }
            }
            _iter += 1;
        }

        // // save cc of some links
        // _str = "\n\n **************************** driving link cc ****************************";
        // for (auto _link_it : m_mcdta->m_link_factory->m_link_map) {
        //     _link = _link_it.second;
        //     if (_link->m_link_ID() == 4) {
        //         _link_m = dynamic_cast<MNM_Dlink_Multiclass *>(_link);
        //         _str += "\nlink_ID: " + std::to_string(_link->m_link_ID());
        //         _str +="\nm_N_in_car: \n";
        //         _str += _link_m->m_N_in_car->to_string();
        //         _str +="\nm_N_out_car: \n";
        //         _str += _link_m->m_N_out_car->to_string();
        //         _str +="\nm_N_in_truck: \n";
        //         _str += _link_m->m_N_in_truck->to_string();
        //         _str +="\nm_N_out_truck: \n";
        //         _str += _link_m->m_N_out_truck->to_string();
        //         _vis_file2 << _str;
        //     }
        // }

        if (_vis_file2.is_open()) _vis_file2.close();
    }
    return 0;
}

py::array_t<double> Mcdta_Api::get_travel_stats()
{
    TInt _count_car = 0, _count_truck = 0;
    TFlt _tot_tt_car = 0.0, _tot_tt_truck = 0.0;
    MNM_Veh_Multiclass* _veh;
    int _end_time = get_cur_loading_interval();
    for (auto _map_it : m_mcdta -> m_veh_factory -> m_veh_map){
        _veh = dynamic_cast<MNM_Veh_Multiclass *>(_map_it.second);
        if ((_veh -> m_class == 0) || (_veh -> m_class == 2)){
            _count_car += 1;
            if (_veh -> m_finish_time > 0) {
                _tot_tt_car += (_veh -> m_finish_time - _veh -> m_start_time) * m_mcdta -> m_unit_time / 3600.0;
            }
            else {
                _tot_tt_car += (_end_time - _veh -> m_start_time) * m_mcdta -> m_unit_time / 3600.0;
            }
        }
        else {
            _count_truck += 1;
            if (_veh -> m_finish_time > 0) {
                _tot_tt_truck += (_veh -> m_finish_time - _veh -> m_start_time) * m_mcdta -> m_unit_time / 3600.0;
            }
            else {
                _tot_tt_truck += (_end_time - _veh -> m_start_time) * m_mcdta -> m_unit_time / 3600.0;
            }
        }
    }
//     printf("\n\nTotal car: %d, Total truck: %d, Total car tt: %.2f hours, Total truck tt: %.2f hours\n\n", 
//            int(_count_car/m_mcdta -> m_flow_scalar), int(_count_truck/m_mcdta -> m_flow_scalar), 
//            float(_tot_tt_car/m_mcdta -> m_flow_scalar), float(_tot_tt_truck/m_mcdta -> m_flow_scalar));
//     m_mcdta -> m_emission -> output();
    
    int new_shape[1] = {4};
    auto result = py::array_t<double>(new_shape);
    auto result_buf = result.request();
    double *result_ptr = (double *)result_buf.ptr;
    result_ptr[0] = _count_car/m_mcdta -> m_flow_scalar;
    result_ptr[1] = _count_truck/m_mcdta -> m_flow_scalar;
    result_ptr[2] = _tot_tt_car/m_mcdta -> m_flow_scalar;
    result_ptr[3] = _tot_tt_truck/m_mcdta -> m_flow_scalar;
    
    return result;
}

py::array_t<double> Mcdta_Api::get_waiting_time_at_intersections()
{
  int new_shape [1] = { (int) m_link_vec.size()}; 
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  for (size_t i = 0; i < m_link_vec.size(); ++i){  
    result_prt[i] = MNM_DTA_GRADIENT::get_average_waiting_time_at_intersection(m_link_vec[i])();  // seconds
  }
    
  return result;
}

py::array_t<int> Mcdta_Api::get_link_spillback()
{
  int new_shape [1] = { (int) m_link_vec.size()}; 
  auto result = py::array_t<int>(new_shape);
  auto result_buf = result.request();
  int *result_prt = (int *) result_buf.ptr;
  for (size_t i = 0; i < m_link_vec.size(); ++i){  
    result_prt[i] = MNM_DTA_GRADIENT::get_is_spillback(m_link_vec[i])();
  }
    
  return result;
}

py::array_t<double> Mcdta_Api::get_path_tt_car(py::array_t<int>link_IDs, py::array_t<double>start_intervals)
{
  auto start_buf = start_intervals.request();
  int num_int = start_buf.shape[0];
    
  auto links_buf = link_IDs.request();
  int num_link = links_buf.shape[0];
    
  int new_shape [1] = { num_link }; 
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  
  double *start_prt = (double *) start_buf.ptr;
  int *links_ptr = (int *) links_buf.ptr;
  MNM_Dlink *_link;
  for (int i = 0; i < links_buf.shape[0]; i++){
    _link = m_mcdta -> m_link_factory -> get_link(TInt(links_ptr[i]));
    if (MNM_Dlink_Multiclass * _mclink = dynamic_cast<MNM_Dlink_Multiclass *>(_link)){
      double avg_tt = 0;
      for (int t = 0; t < num_int; ++t){
          double _tmp = MNM_DTA_GRADIENT::get_travel_time_car(_mclink, TFlt(start_prt[t]), m_mcdta -> m_unit_time, get_cur_loading_interval())() * m_mcdta -> m_unit_time;
          if (_tmp > 20 * (_mclink -> m_length / _mclink -> m_ffs_car)){
              _tmp = 20 * _mclink -> m_length / _mclink -> m_ffs_car;
          }
          avg_tt += _tmp; // seconds
      }
      avg_tt /= num_int;
      result_prt[i] = avg_tt;
    }
    else{
      throw std::runtime_error("Mcdta_Api::get_path_tt_car: link type is not multiclass");
    }
  }
  
  return result;
}

py::array_t<double> Mcdta_Api::get_path_tt_truck(py::array_t<int>link_IDs, py::array_t<double>start_intervals)
{
  auto start_buf = start_intervals.request();
  int num_int = start_buf.shape[0];
    
  auto links_buf = link_IDs.request();
  int num_link = links_buf.shape[0];
    
  int new_shape [1] = { num_link }; 
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  
  double *start_prt = (double *) start_buf.ptr;
  int *links_ptr = (int *) links_buf.ptr;
  MNM_Dlink *_link;
  for (int i = 0; i < links_buf.shape[0]; i++){
    _link = m_mcdta -> m_link_factory -> get_link(TInt(links_ptr[i]));
    if (MNM_Dlink_Multiclass * _mclink = dynamic_cast<MNM_Dlink_Multiclass *>(_link)){
      double avg_tt = 0;
      for (int t = 0; t < num_int; ++t){
          double _tmp = MNM_DTA_GRADIENT::get_travel_time_truck(_mclink, TFlt(start_prt[t]), m_mcdta -> m_unit_time, get_cur_loading_interval())() * m_mcdta -> m_unit_time;
          if (_tmp > 20 * (_mclink -> m_length / _mclink -> m_ffs_truck)){
              _tmp = 20 * _mclink -> m_length / _mclink -> m_ffs_truck;
          }
          avg_tt += _tmp; // seconds
      }
      avg_tt /= num_int;
      result_prt[i] = avg_tt;
    }
    else{
      throw std::runtime_error("Mcdta_Api::get_path_tt_truck: link type is not multiclass");
    }
  }
    
  return result;
}

py::array_t<double> Mcdta_Api::get_car_link_fftt(py::array_t<int>link_IDs)
{
    auto start_buf = link_IDs.request();
    if (start_buf.ndim != 1){
        throw std::runtime_error("Error, Mcdta_Api::get_car_link_fftt, input dimension mismatch");
    }
    int l = start_buf.shape[0];
    
    auto result = py::array_t<double>(l);
    auto result_buf = result.request();
    double *result_prt = (double *) result_buf.ptr;
    int *start_prt = (int *) start_buf.ptr;

    for (int i = 0; i < l; ++i) {
        result_prt[i] = dynamic_cast<MNM_Dlink_Multiclass*>(m_mcdta -> m_link_factory -> get_link(start_prt[i])) -> get_link_freeflow_tt_loading_car() * m_mcdta -> m_unit_time;  // seconds
    }
    return result;
}

py::array_t<double> Mcdta_Api::get_truck_link_fftt(py::array_t<int>link_IDs)
{
    auto start_buf = link_IDs.request();
    if (start_buf.ndim != 1){
        throw std::runtime_error("Error, Mcdta_Api::get_truck_link_fftt, input dimension mismatch");
    }
    int l = start_buf.shape[0];
    
    auto result = py::array_t<double>(l);
    auto result_buf = result.request();
    double *result_prt = (double *) result_buf.ptr;
    int *start_prt = (int *) start_buf.ptr;

    for (int i = 0; i < l; ++i) {
        result_prt[i] = dynamic_cast<MNM_Dlink_Multiclass*>(m_mcdta -> m_link_factory -> get_link(start_prt[i])) -> get_link_freeflow_tt_loading_truck() * m_mcdta -> m_unit_time;  // seconds
    }
    return result;
}

// unit: m_mcdta -> m_unit_time (eg: 5 seconds)
// TODO check 
py::array_t<double> Mcdta_Api::get_car_link_tt(py::array_t<double>start_intervals)
{
  auto start_buf = start_intervals.request();
  if (start_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get_car_link_tt, input dimension mismatch");
  }
  int l = start_buf.shape[0];
  int new_shape [2] = { (int) m_link_vec.size(), l}; 

  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  double *start_prt = (double *) start_buf.ptr;
  for (int t = 0; t < l; ++t){
    if (start_prt[t] > get_cur_loading_interval()){
      throw std::runtime_error("Error, Mcdta_Api::get_car_link_tt, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec.size(); ++i){
      double _tmp = MNM_DTA_GRADIENT::get_travel_time_car(m_link_vec[i], TFlt(start_prt[t]), m_mcdta->m_unit_time, get_cur_loading_interval())();
      // if (_tmp * m_mcdta -> m_unit_time > 20 * (m_link_vec[i] -> m_length / m_link_vec[i] -> m_ffs_car)){
      //     _tmp = 20 * m_link_vec[i] -> m_length / m_link_vec[i] -> m_ffs_car / m_mcdta -> m_unit_time;
      // }
      result_prt[i * l + t] = _tmp * m_mcdta -> m_unit_time;
    }
  }
  return result;
}

py::array_t<double> Mcdta_Api::get_car_link_tt_robust(py::array_t<double>start_intervals, py::array_t<double>end_intervals)
{
  auto start_buf = start_intervals.request();
  auto end_buf = end_intervals.request();
  if (start_buf.ndim != 1 || end_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get_car_link_tt_robust, input dimension mismatch");
  }
  if (start_buf.shape[0] != end_buf.shape[0]){
    throw std::runtime_error("Error, Mcdta_Api::get_car_link_tt_robust, input length mismatch");
  }
  int l = start_buf.shape[0];
  int new_shape [2] = { (int) m_link_vec.size(), l}; 

  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  double *start_prt = (double *) start_buf.ptr;
  double *end_prt = (double *) end_buf.ptr;
  for (int t = 0; t < l; ++t){
    if (start_prt[t] > get_cur_loading_interval()){
      throw std::runtime_error("Error, Mcdta_Api::get_car_link_tt_robust, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec.size(); ++i){
      double _tmp = MNM_DTA_GRADIENT::get_travel_time_car_robust(m_link_vec[i], TFlt(start_prt[t]), TFlt(end_prt[t]), m_mcdta -> m_unit_time, get_cur_loading_interval())();
      // if (_tmp * m_mcdta -> m_unit_time > 20 * (m_link_vec[i] -> m_length / m_link_vec[i] -> m_ffs_car)){
      //     _tmp = 20 * m_link_vec[i] -> m_length / m_link_vec[i] -> m_ffs_car / m_mcdta -> m_unit_time;
      // }
      result_prt[i * l + t] = _tmp * m_mcdta -> m_unit_time;  // second
    }
  }
  return result;
}

py::array_t<double> Mcdta_Api::get_truck_link_tt(py::array_t<double>start_intervals)
{
  auto start_buf = start_intervals.request();
  if (start_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get_truck_link_tt, input dimension mismatch");
  }
  int l = start_buf.shape[0];
  int new_shape [2] = { (int) m_link_vec.size(), l}; 

  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  double *start_prt = (double *) start_buf.ptr;
  for (int t = 0; t < l; ++t){
    if (start_prt[t] > get_cur_loading_interval()){
      throw std::runtime_error("Error, Mcdta_Api::get_truck_link_tt, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec.size(); ++i){
      double _tmp = MNM_DTA_GRADIENT::get_travel_time_truck(m_link_vec[i], TFlt(start_prt[t]), m_mcdta -> m_unit_time, get_cur_loading_interval())();
      // if (_tmp * 5 > 20 * (m_link_vec[i] -> m_length / m_link_vec[i] -> m_ffs_truck)){
      //     _tmp = 20 * m_link_vec[i] -> m_length / m_link_vec[i] -> m_ffs_truck / m_mcdta -> m_unit_time;
      // }
      result_prt[i * l + t] = _tmp * m_mcdta -> m_unit_time;  // second
    }
  }
  return result;
}

py::array_t<double> Mcdta_Api::get_car_link_speed(py::array_t<double>start_intervals)
{
  auto start_buf = start_intervals.request();
  if (start_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get_car_link_speed, input dimension mismatch");
  }
  int l = start_buf.shape[0];
  int new_shape [2] = { (int) m_link_vec.size(), l};

  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  double *start_prt = (double *) start_buf.ptr;
  for (int t = 0; t < l; ++t){
    if (start_prt[t] > get_cur_loading_interval()){
      throw std::runtime_error("Error, Mcdta_Api::get_car_link_speed, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec.size(); ++i){  
      double _tt = MNM_DTA_GRADIENT::get_travel_time_car(m_link_vec[i], TFlt(start_prt[t]), m_mcdta -> m_unit_time, get_cur_loading_interval())() * m_mcdta -> m_unit_time; //seconds
      result_prt[i * l + t] = (m_link_vec[i] -> m_length) / _tt * 3600 / 1600; // mile per hour
    }
  }
  return result;
}

py::array_t<double> Mcdta_Api::get_truck_link_speed(py::array_t<double>start_intervals)
{
  auto start_buf = start_intervals.request();
  if (start_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get_truck_link_speed, input dimension mismatch");
  }
  int l = start_buf.shape[0];
  int new_shape [2] = { (int) m_link_vec.size(), l}; 

  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  double *start_prt = (double *) start_buf.ptr;
  for (int t = 0; t < l; ++t){
    if (start_prt[t] > get_cur_loading_interval()){
      throw std::runtime_error("Error, Mcdta_Api::get_truck_link_speed, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec.size(); ++i){
      double _tt = MNM_DTA_GRADIENT::get_travel_time_truck(m_link_vec[i], TFlt(start_prt[t]), m_mcdta -> m_unit_time, get_cur_loading_interval())() * m_mcdta -> m_unit_time; // seconds
      result_prt[i * l + t] = (m_link_vec[i] -> m_length) / _tt * 3600 / 1600; // mile per hour
    }
  }
  return result;
}

py::array_t<double> Mcdta_Api::get_link_car_inflow(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
{
  auto start_buf = start_intervals.request();
  auto end_buf = end_intervals.request();
  if (start_buf.ndim != 1 || end_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get_link_car_inflow, input dimension mismatch");
  }
  if (start_buf.shape[0] != end_buf.shape[0]){
    throw std::runtime_error("Error, Mcdta_Api::get_link_car_inflow, input length mismatch");
  }
  int l = start_buf.shape[0];
  int new_shape [2] = { (int) m_link_vec.size(), l};
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  int *start_prt = (int *) start_buf.ptr;
  int *end_prt = (int *) end_buf.ptr;
  for (int t = 0; t < l; ++t){
    if (end_prt[t] < start_prt[t]){
      throw std::runtime_error("Error, Mcdta_Api::get_link_car_inflow, end time smaller than start time");
    }
    if (end_prt[t] > get_cur_loading_interval()){
      throw std::runtime_error("Error, Mcdta_Api::get_link_car_inflow, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec.size(); ++i){
      result_prt[i * l + t] = MNM_DTA_GRADIENT::get_link_inflow_car(m_link_vec[i], TFlt(start_prt[t]), TFlt(end_prt[t]))();
      // printf("i %d, t %d, %f\n", i, t, result_prt[i * l + t]);
    }
  }
  return result;
}

py::array_t<double> Mcdta_Api::get_link_truck_inflow(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
{
  auto start_buf = start_intervals.request();
  auto end_buf = end_intervals.request();
  if (start_buf.ndim != 1 || end_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get_link_truck_inflow, input dimension mismatch");
  }
  if (start_buf.shape[0] != end_buf.shape[0]){
    throw std::runtime_error("Error, Mcdta_Api::get_link_truck_inflow, input length mismatch");
  }
  int l = start_buf.shape[0];
  int new_shape [2] = { (int) m_link_vec.size(), l};
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  int *start_prt = (int *) start_buf.ptr;
  int *end_prt = (int *) end_buf.ptr;
  for (int t = 0; t < l; ++t){
    if (end_prt[t] < start_prt[t]){
      throw std::runtime_error("Error, Mcdta_Api::get_link_truck_inflow, end time smaller than start time");
    }
    if (end_prt[t] > get_cur_loading_interval()){
      throw std::runtime_error("Error, Mcdta_Api::get_link_truck_inflow, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec.size(); ++i){
      result_prt[i * l + t] = MNM_DTA_GRADIENT::get_link_inflow_truck(m_link_vec[i], TFlt(start_prt[t]), TFlt(end_prt[t]))();
      // printf("i %d, t %d, %f\n", i, t, result_prt[i * l + t]);
    }
  }
  return result;
}

py::array_t<double> Mcdta_Api::get_link_rh_inflow(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
{
  auto start_buf = start_intervals.request();
  auto end_buf = end_intervals.request();
  if (start_buf.ndim != 1 || end_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get_link_rh_inflow, input dimension mismatch");
  }
  if (start_buf.shape[0] != end_buf.shape[0]){
    throw std::runtime_error("Error, Mcdta_Api::get_link_rh_inflow, input length mismatch");
  }
  int l = start_buf.shape[0];
  int new_shape [2] = { (int) m_link_vec.size(), l};
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  int *start_prt = (int *) start_buf.ptr;
  int *end_prt = (int *) end_buf.ptr;
  for (int t = 0; t < l; ++t){
    if (end_prt[t] < start_prt[t]){
      throw std::runtime_error("Error, Mcdta_Api::get_link_rh_inflow, end time smaller than start time");
    }
    if (end_prt[t] > get_cur_loading_interval()){
      throw std::runtime_error("Error, Mcdta_Api::get_link_rh_inflow, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec.size(); ++i){
      result_prt[i * l + t] = MNM_DTA_GRADIENT::get_link_inflow_rh(m_link_vec[i], TFlt(start_prt[t]), TFlt(end_prt[t]))();
      // printf("i %d, t %d, %f\n", i, t, result_prt[i * l + t]);
    }
  }
  return result;
}

py::array_t<double> Mcdta_Api::get_link_car_outflow(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
{
  auto start_buf = start_intervals.request();
  auto end_buf = end_intervals.request();
  if (start_buf.ndim != 1 || end_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get_link_car_outflow, input dimension mismatch");
  }
  if (start_buf.shape[0] != end_buf.shape[0]){
    throw std::runtime_error("Error, Mcdta_Api::get_link_car_outflow, input length mismatch");
  }
  int l = start_buf.shape[0];
  int new_shape [2] = { (int) m_link_vec.size(), l};
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  int *start_prt = (int *) start_buf.ptr;
  int *end_prt = (int *) end_buf.ptr;
  for (int t = 0; t < l; ++t){
    if (end_prt[t] < start_prt[t]){
      throw std::runtime_error("Error, Mcdta_Api::get_link_car_outflow, end time smaller than start time");
    }
    if (end_prt[t] > get_cur_loading_interval()){
      throw std::runtime_error("Error, Mcdta_Api::get_link_car_outflow, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec.size(); ++i){
      result_prt[i * l + t] = MNM_DTA_GRADIENT::get_link_outflow_car(m_link_vec[i], TFlt(start_prt[t]), TFlt(end_prt[t]))();
      // printf("i %d, t %d, %f\n", i, t, result_prt[i * l + t]);
    }
  }
  return result;
}

py::array_t<double> Mcdta_Api::get_link_truck_outflow(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
{
  auto start_buf = start_intervals.request();
  auto end_buf = end_intervals.request();
  if (start_buf.ndim != 1 || end_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get_link_truck_outflow, input dimension mismatch");
  }
  if (start_buf.shape[0] != end_buf.shape[0]){
    throw std::runtime_error("Error, Mcdta_Api::get_link_truck_outflow, input length mismatch");
  }
  int l = start_buf.shape[0];
  int new_shape [2] = { (int) m_link_vec.size(), l};
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  int *start_prt = (int *) start_buf.ptr;
  int *end_prt = (int *) end_buf.ptr;
  for (int t = 0; t < l; ++t){
    if (end_prt[t] < start_prt[t]){
      throw std::runtime_error("Error, Mcdta_Api::get_link_truck_outflow, end time smaller than start time");
    }
    if (end_prt[t] > get_cur_loading_interval()){
      throw std::runtime_error("Error, Mcdta_Api::get_link_truck_outflow, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec.size(); ++i){
      result_prt[i * l + t] = MNM_DTA_GRADIENT::get_link_outflow_truck(m_link_vec[i], TFlt(start_prt[t]), TFlt(end_prt[t]))();
      // printf("i %d, t %d, %f\n", i, t, result_prt[i * l + t]);
    }
  }
  return result;
}

int Mcdta_Api::register_paths(py::array_t<int> paths)
{
  if (m_path_vec.size() > 0){
    printf("Warning, Mcdta_Api::register_paths, path exists\n");
    m_path_vec.clear();
    m_path_set.clear();
  }

  auto paths_buf = paths.request();
  if (paths_buf.ndim != 1){
    throw std::runtime_error("Mcdta_Api::register_paths: Number of dimensions must be one");
  }

  int *paths_ptr = (int *) paths_buf.ptr; 
  TInt _path_ID;
  for (int i = 0; i < paths_buf.shape[0]; i++){
    _path_ID = TInt(paths_ptr[i]);
    // printf("registering path %d, %d\n", _path_ID(), (int)m_ID_path_mapping.size());
    if (m_ID_path_mapping.find(_path_ID) == m_ID_path_mapping.end()){
      throw std::runtime_error("Mcdta_Api::register_paths: No such path");
    }
    else {
      m_path_vec.push_back(m_ID_path_mapping[_path_ID]);
    }
  }
  m_path_set = std::set<MNM_Path*> (m_path_vec.begin(), m_path_vec.end());
  return 0;
}
// TODO jiachao
int Mcdta_Api::register_paths_cc(py::array_t<int> paths_cc)
{
  if (m_path_vec_truck.size() > 0){
    printf("Warning, Mcdta_Api::register_paths_cc, path exists\n");
    m_path_vec_truck.clear();
    m_path_set_truck.clear();
  }

  if (m_path_vec_rh.size() > 0){
    printf("Warning, Mcdta_Api::register_paths_rh, path exists\n");
    m_path_vec_rh.clear();
    m_path_set_rh.clear();
  } 
  
  auto paths_buf = paths_cc.request();
  if (paths_buf.ndim != 1){
    throw std::runtime_error("Mcdta_Api::register_paths_cc: Number of dimensions must be one");
  }

  int *paths_ptr = (int *) paths_buf.ptr; 
  TInt _path_ID;
  for (int i = 0; i < paths_buf.shape[0]; i++){
    _path_ID = TInt(paths_ptr[i]);
    // printf("registering path %d, %d\n", _path_ID(), (int)m_ID_path_mapping.size());
    if (m_ID_path_mapping_truck.find(_path_ID) == m_ID_path_mapping_truck.end()){
      throw std::runtime_error("Mcdta_Api::register_paths_cc: No such path");
    }
    else {
      m_path_vec_truck.push_back(m_ID_path_mapping_truck[_path_ID]);
    }

    if (m_ID_path_mapping_rh.find(_path_ID) == m_ID_path_mapping_rh.end()){
      throw std::runtime_error("Mcdta_Api::register_paths_rh: No such path");
    }
    else {
      m_path_vec_rh.push_back(m_ID_path_mapping_rh[_path_ID]);
    }
  }

  m_path_set_truck = std::set<MNM_Path*> (m_path_vec_truck.begin(), m_path_vec_truck.end());
  m_path_set_rh = std::set<MNM_Path*> (m_path_vec_rh.begin(), m_path_vec_rh.end());

  return 0;
}

py::array_t<double> Mcdta_Api::get_car_link_out_cc(int link_ID)
{
  MNM_Dlink_Multiclass *_link = (MNM_Dlink_Multiclass *) m_mcdta -> m_link_factory -> get_link(TInt(link_ID));
  // printf("link: %d\n", _link -> m_link_ID());
  if (_link -> m_N_out_car == nullptr){
    throw std::runtime_error("Error, Mcdta_Api::get_car_link_out_cc, cc not installed");
  }
  std::deque<std::pair<TFlt, TFlt>> _record = _link -> m_N_out_car -> m_recorder;
  int new_shape [2] = { (int) _record.size(), 2}; 
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  for (size_t i = 0; i< _record.size(); ++i){
    result_prt[i * 2] = _record[i].first();
    result_prt[i * 2 + 1] =  _record[i].second();
  }
  return result;
}

py::array_t<double> Mcdta_Api::get_car_link_in_cc(int link_ID)
{
  MNM_Dlink_Multiclass *_link = (MNM_Dlink_Multiclass *) m_mcdta -> m_link_factory -> get_link(TInt(link_ID));
  // printf("link: %d\n", _link -> m_link_ID());
  if (_link -> m_N_in_car == nullptr){
    throw std::runtime_error("Error, Mcdta_Api::get_car_link_in_cc, cc not installed");
  }
  std::deque<std::pair<TFlt, TFlt>> _record = _link -> m_N_in_car -> m_recorder;
  int new_shape [2] = { (int) _record.size(), 2}; 
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  for (size_t i=0; i< _record.size(); ++i){
    result_prt[i * 2 ] = _record[i].first();
    result_prt[i * 2 + 1 ] =  _record[i].second();
  }
  return result;
}

py::array_t<double> Mcdta_Api::get_truck_link_out_cc(int link_ID)
{
  MNM_Dlink_Multiclass *_link = (MNM_Dlink_Multiclass *) m_mcdta -> m_link_factory -> get_link(TInt(link_ID));
  printf("link: %d\n", _link -> m_link_ID());
  if (_link -> m_N_out_truck == nullptr){
    throw std::runtime_error("Error, Mcdta_Api::get_truck_link_out_cc, cc not installed");
  }
  std::deque<std::pair<TFlt, TFlt>> _record = _link -> m_N_out_truck -> m_recorder;
  int new_shape [2] = { (int) _record.size(), 2}; 
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  for (size_t i=0; i< _record.size(); ++i){
    result_prt[i * 2 ] = _record[i].first();
    result_prt[i * 2 + 1 ] =  _record[i].second();
  }
  return result;
}

py::array_t<double> Mcdta_Api::get_truck_link_in_cc(int link_ID)
{
  MNM_Dlink_Multiclass *_link = (MNM_Dlink_Multiclass *) m_mcdta -> m_link_factory -> get_link(TInt(link_ID));
  printf("link: %d\n", _link -> m_link_ID());
  if (_link -> m_N_in_truck == nullptr){
    throw std::runtime_error("Error, Mcdta_Api::get_truck_link_in_cc, cc not installed");
  }
  std::deque<std::pair<TFlt, TFlt>> _record = _link -> m_N_in_truck -> m_recorder;
  int new_shape [2] = { (int) _record.size(), 2}; 
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  for (size_t i=0; i< _record.size(); ++i){
    result_prt[i * 2 ] = _record[i].first();
    result_prt[i * 2 + 1 ] =  _record[i].second();
  }
  return result;
}

py::array_t<double> Mcdta_Api::get_enroute_and_queue_veh_stats_agg()
{
  int _tot_interval = get_cur_loading_interval();
  int new_shape[2] = {_tot_interval, 3};
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;

  if ((int) m_mcdta -> m_enroute_veh_num.size() != get_cur_loading_interval()){
    throw std::runtime_error("Error, Mcdta_Api::get_enroute_and_queue_veh_stats_agg, enroute vehicle missed for some intervals");
  }
  else if ((int) m_mcdta -> m_queue_veh_num.size() != get_cur_loading_interval()){
    throw std::runtime_error("Error, Mcdta_Api::get_enroute_and_queue_veh_stats_agg, queuing vehicle missed for some intervals");
  }
  else{
    for (int i = 0; i < _tot_interval; ++i){
      result_prt[i * 3] =  (m_mcdta -> m_enroute_veh_num[i]())/(m_mcdta -> m_flow_scalar);
      result_prt[i * 3 + 1] =  (m_mcdta -> m_queue_veh_num[i]())/(m_mcdta -> m_flow_scalar);
      result_prt[i * 3 + 2] =  (m_mcdta -> m_enroute_veh_num[i]() - m_mcdta -> m_queue_veh_num[i]())/(m_mcdta -> m_flow_scalar);
    }
  } 
  return result;
}

py::array_t<double> Mcdta_Api::get_queue_veh_each_link(py::array_t<int>useful_links, py::array_t<int>intervals)
{
  auto intervals_buf = intervals.request();
  if (intervals_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get_queue_veh_each_link, input (intervals) dimension mismatch");
  }
  auto links_buf = useful_links.request();
  if (links_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get_queue_veh_each_link, input (useful_links) dimension mismatch");
  }
  int num_intervals = intervals_buf.shape[0];
  int num_links = links_buf.shape[0];
  int new_shape[2] = {num_links, num_intervals};
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  double *intervals_prt = (double *) intervals_buf.ptr;
  double *links_prt = (double *) links_buf.ptr;
    
  for (int t = 0; t < num_intervals; ++t){
    if (intervals_prt[t] > get_cur_loading_interval()){
      throw std::runtime_error("Error, Mcdta_Api::get_queue_veh_each_link, too large interval number");
    }
    for (int i = 0; i < num_links; ++i){
      if (m_mcdta -> m_queue_veh_map.find(links_prt[i]) == m_mcdta -> m_queue_veh_map.end()){
        throw std::runtime_error("Error, Mcdta_Api::get_queue_veh_each_link, can't find link ID");
      }
      // not divided by flow_scalar in the original version
      result_prt[i * num_intervals + t] = (*(m_mcdta -> m_queue_veh_map[links_prt[i]]))[intervals_prt[t]] / m_mcdta -> m_flow_scalar;
    }
  }
  return result;
}

double Mcdta_Api::get_car_link_out_num(int link_ID, double time)
{
  MNM_Dlink_Multiclass *_link = (MNM_Dlink_Multiclass *) m_mcdta -> m_link_factory -> get_link(TInt(link_ID));
  // printf("%d\n", _link -> m_link_ID());
  if (_link -> m_N_out_car == nullptr){
    throw std::runtime_error("Error, Mcdta_Api::get_car_link_out_num, cc not installed");
  }
  // printf("1\n");
  TFlt result = _link -> m_N_out_car -> get_result(TFlt(time)) / m_mcdta -> m_flow_scalar;
  // printf("%lf\n", result());
  return result();
}

double Mcdta_Api::get_truck_link_out_num(int link_ID, double time)
{
  MNM_Dlink_Multiclass *_link = (MNM_Dlink_Multiclass *) m_mcdta -> m_link_factory -> get_link(TInt(link_ID));
  if (_link -> m_N_out_truck == nullptr){
    throw std::runtime_error("Error, Mcdta_Api::get_truck_link_out_num, cc not installed");
  }
  TFlt result = _link -> m_N_out_truck -> get_result(TFlt(time)) / m_mcdta -> m_flow_scalar;
  return result();
}


// py::array_t<double> Mcdta_Api::get_car_dar_matrix(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
// {
//   auto start_buf = start_intervals.request();
//   auto end_buf = end_intervals.request();
//   if (start_buf.ndim != 1 || end_buf.ndim != 1){
//     throw std::runtime_error("Error, Mcdta_Api::get_car_dar_matrix, input dimension mismatch");
//   }
//   if (start_buf.shape[0] != end_buf.shape[0]){
//     throw std::runtime_error("Error, Mcdta_Api::get_car_dar_matrix, input length mismatch");
//   }
//   int l = start_buf.shape[0];
//   int *start_prt = (int *) start_buf.ptr;
//   int *end_prt = (int *) end_buf.ptr;
//   std::vector<dar_record*> _record = std::vector<dar_record*>();
//   for (int t = 0; t < l; ++t){
//     // printf("Current processing time: %d\n", t);
//     if (end_prt[t] < start_prt[t]){
//         throw std::runtime_error("Error, Mcdta_Api::get_car_dar_matrix, end time smaller than start time");
//     }
//     if (end_prt[t] > get_cur_loading_interval()){
//         throw std::runtime_error("Error, Mcdta_Api::get_car_dar_matrix, loaded data not enough");
//     }
//     for (size_t i = 0; i<m_link_vec.size(); ++i){
//       MNM_DTA_GRADIENT::add_dar_records_car(_record, m_link_vec[i], m_path_set, TFlt(start_prt[t]), TFlt(end_prt[t]));
//     }
//   }
//   // _record.size() = num_timesteps x num_links x num_path x num_assign_timesteps
//   // path_ID, assign_time, link_ID, start_int, flow
//   int new_shape [2] = { (int) _record.size(), 5}; 
//   auto result = py::array_t<double>(new_shape);
//   auto result_buf = result.request();
//   double *result_prt = (double *) result_buf.ptr;
//   dar_record* tmp_record;
//   for (size_t i = 0; i < _record.size(); ++i){
//     tmp_record = _record[i];
//     result_prt[i * 5 + 0] = (double) tmp_record -> path_ID();
//     // the count of 1 min interval
//     result_prt[i * 5 + 1] = (double) tmp_record -> assign_int();
//     result_prt[i * 5 + 2] = (double) tmp_record -> link_ID();
//     // the count of unit time interval (5s)
//     result_prt[i * 5 + 3] = (double) tmp_record -> link_start_int();
//     result_prt[i * 5 + 4] = tmp_record -> flow();
//   }
//   for (size_t i = 0; i < _record.size(); ++i){
//     delete _record[i];
//   }
//   _record.clear();
//   return result;
// }

// py::array_t<double> Mcdta_Api::get_truck_dar_matrix(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
// {
//   auto start_buf = start_intervals.request();
//   auto end_buf = end_intervals.request();
//   if (start_buf.ndim != 1 || end_buf.ndim != 1){
//     throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_matrix, input dimension mismatch");
//   }
//   if (start_buf.shape[0] != end_buf.shape[0]){
//     throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_matrix, input length mismatch");
//   }
//   int l = start_buf.shape[0];
//   int *start_prt = (int *) start_buf.ptr;
//   int *end_prt = (int *) end_buf.ptr;
//   std::vector<dar_record*> _record = std::vector<dar_record*>();
//   for (int t = 0; t < l; ++t){
//     if (end_prt[t] < start_prt[t]){
//         throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_matrix, end time smaller than start time");
//     }
//     if (end_prt[t] > get_cur_loading_interval()){
//         throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_matrix, loaded data not enough");
//     }
//     for (size_t i = 0; i<m_link_vec.size(); ++i){
//         MNM_DTA_GRADIENT::add_dar_records_truck(_record, m_link_vec[i], m_path_set_truck, TFlt(start_prt[t]), TFlt(end_prt[t]));
//     }
//   }
//   // path_ID, assign_time, link_ID, start_int, flow
//   int new_shape [2] = { (int) _record.size(), 5}; 
//   auto result = py::array_t<double>(new_shape);
//   auto result_buf = result.request();
//   double *result_prt = (double *) result_buf.ptr;
//   dar_record* tmp_record;
//   for (size_t i = 0; i < _record.size(); ++i){
//     tmp_record = _record[i];
//     result_prt[i * 5 + 0] = (double) tmp_record -> path_ID();
//     // the count of 1 min interval
//     result_prt[i * 5 + 1] = (double) tmp_record -> assign_int();
//     result_prt[i * 5 + 2] = (double) tmp_record -> link_ID();
//     // the count of unit time interval (5s)
//     result_prt[i * 5 + 3] = (double) tmp_record -> link_start_int();
//     result_prt[i * 5 + 4] = tmp_record -> flow();
//   }
//   for (size_t i = 0; i < _record.size(); ++i){
//     delete _record[i];
//   }
//   _record.clear();
//   return result;
// }

// py::array_t<double> Mcdta_Api::get_rh_dar_matrix(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
// {
//   auto start_buf = start_intervals.request();
//   auto end_buf = end_intervals.request();
//   if (start_buf.ndim != 1 || end_buf.ndim != 1){
//     throw std::runtime_error("Error, Mcdta_Api::get_rh_dar_matrix, input dimension mismatch");
//   }
//   if (start_buf.shape[0] != end_buf.shape[0]){
//     throw std::runtime_error("Error, Mcdta_Api::get_rh_dar_matrix, input length mismatch");
//   }
//   int l = start_buf.shape[0];
//   int *start_prt = (int *) start_buf.ptr;
//   int *end_prt = (int *) end_buf.ptr;
//   std::vector<dar_record*> _record = std::vector<dar_record*>();
  
//   for (int t = 0; t < l; ++t){
//     // printf("Current processing time: %d\n", t);
//     if (end_prt[t] < start_prt[t]){
//         throw std::runtime_error("Error, Mcdta_Api::get_rh_dar_matrix, end time smaller than start time");
//     }
//     if (end_prt[t] > get_cur_loading_interval()){
//         throw std::runtime_error("Error, Mcdta_Api::get_rh_dar_matrix, loaded data not enough");
//     }
//     for (size_t i = 0; i<m_link_vec.size(); ++i){
//       MNM_DTA_GRADIENT::add_dar_records_rh(_record, m_link_vec[i], m_path_set_rh, TFlt(start_prt[t]), TFlt(end_prt[t]));
//     }
//   }
//   // _record.size() = num_timesteps x num_links x num_path x num_assign_timesteps
//   // path_ID, assign_time, link_ID, start_int, flow
//   int new_shape [2] = { (int) _record.size(), 5}; 
//   auto result = py::array_t<double>(new_shape);
//   auto result_buf = result.request();
//   double *result_prt = (double *) result_buf.ptr;
//   dar_record* tmp_record;
//   for (size_t i = 0; i < _record.size(); ++i){
//     tmp_record = _record[i];
//     result_prt[i * 5 + 0] = (double) tmp_record -> path_ID();
//     // the count of 1 min interval
//     result_prt[i * 5 + 1] = (double) tmp_record -> assign_int();
//     result_prt[i * 5 + 2] = (double) tmp_record -> link_ID();
//     // the count of unit time interval (5s)
//     result_prt[i * 5 + 3] = (double) tmp_record -> link_start_int();
//     result_prt[i * 5 + 4] = tmp_record -> flow();
//   }
//   for (size_t i = 0; i < _record.size(); ++i){
//     delete _record[i];
//   }
//   _record.clear();
//   return result;
// }

// TODO new added curb arrival and departure DAR
// py::array_t<double> Mcdta_Api::get_car_dar_arrival_matrix(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
// {
//   auto start_buf = start_intervals.request();
//   auto end_buf = end_intervals.request();
//   if (start_buf.ndim != 1 || end_buf.ndim != 1){
//     throw std::runtime_error("Error, Mcdta_Api::get_car_dar_arrival matrix sep, input dimension mismatch");
//   }
//   if (start_buf.shape[0] != end_buf.shape[0]){
//     throw std::runtime_error("Error, Mcdta_Api::get_car_dar_arrival matrix sep, input length mismatch");
//   }
//   int l = start_buf.shape[0];
//   int *start_prt = (int *) start_buf.ptr;
//   int *end_prt = (int *) end_buf.ptr;
//   std::vector<dar_record*> _record = std::vector<dar_record*>();

//   for (int t = 0; t < l; ++t){
//     // printf("Current processing time: %d\n", t);
//     if (end_prt[t] < start_prt[t]){
//         throw std::runtime_error("Error, Mcdta_Api::get_car_dar_arrival matrix sep, end time smaller than start time");
//     }
//     if (end_prt[t] > get_cur_loading_interval()){
//         throw std::runtime_error("Error, Mcdta_Api::get_car_dar_arrival matrix sep, loaded data not enough");
//     }
//     for (size_t i = 0; i < m_link_vec.size(); ++i){
//       MNM_DTA_GRADIENT::add_dar_records_curb_arrival_car(_record, m_link_vec[i], m_path_set, TFlt(start_prt[t]), TFlt(end_prt[t]));
//     }
//   }
//   // _record.size() = num_timesteps x num_links x num_path x num_assign_timesteps
//   // path_ID, assign_time, link_ID, start_int, flow
//   int new_shape [2] = { (int) _record.size(), 5}; 
//   auto result = py::array_t<double>(new_shape);
//   auto result_buf = result.request();
//   double *result_prt = (double *) result_buf.ptr;
//   dar_record* tmp_record;
//   for (size_t i = 0; i < _record.size(); ++i){
//     tmp_record = _record[i];
//     result_prt[i * 5 + 0] = (double) tmp_record -> path_ID();
//     // the count of 1 min interval
//     result_prt[i * 5 + 1] = (double) tmp_record -> assign_int();
//     result_prt[i * 5 + 2] = (double) tmp_record -> link_ID();
//     // the count of unit time interval (5s)
//     result_prt[i * 5 + 3] = (double) tmp_record -> link_start_int();
//     result_prt[i * 5 + 4] = tmp_record -> flow();
//   }
//   for (size_t i = 0; i < _record.size(); ++i){
//     delete _record[i];
//   }
//   _record.clear();
//   return result;
// }

// py::array_t<double> Mcdta_Api::get_car_dar_departure_matrix(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
// {
//   auto start_buf = start_intervals.request();
//   auto end_buf = end_intervals.request();
//   if (start_buf.ndim != 1 || end_buf.ndim != 1){
//     throw std::runtime_error("Error, Mcdta_Api::get_car_dar_departure matrix_sep, input dimension mismatch");
//   }
//   if (start_buf.shape[0] != end_buf.shape[0]){
//     throw std::runtime_error("Error, Mcdta_Api::get_car_dar_departure matrix_sep, input length mismatch");
//   }
//   int l = start_buf.shape[0];
//   int *start_prt = (int *) start_buf.ptr;
//   int *end_prt = (int *) end_buf.ptr;
//   std::vector<dar_record*> _record = std::vector<dar_record*>();

//   for (int t = 0; t < l; ++t){
//     // printf("Current processing time: %d\n", t);
//     if (end_prt[t] < start_prt[t]){
//         throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_departure matrix_sep, end time smaller than start time");
//     }
//     if (end_prt[t] > get_cur_loading_interval()){
//         throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_departure matrix_sep, loaded data not enough");
//     }
//     for (size_t i = 0; i < m_link_vec.size(); ++i){
//       MNM_DTA_GRADIENT::add_dar_records_curb_departure_car(_record, m_link_vec[i], m_path_set, TFlt(start_prt[t]), TFlt(end_prt[t]));
//     }
//   }
//   // _record.size() = num_timesteps x num_links x num_path x num_assign_timesteps
//   // path_ID, assign_time, link_ID, start_int, flow
//   int new_shape [2] = { (int) _record.size(), 5}; 
//   auto result = py::array_t<double>(new_shape);
//   auto result_buf = result.request();
//   double *result_prt = (double *) result_buf.ptr;
//   dar_record* tmp_record;
//   for (size_t i = 0; i < _record.size(); ++i){
//     tmp_record = _record[i];
//     result_prt[i * 5 + 0] = (double) tmp_record -> path_ID();
//     // the count of 1 min interval
//     result_prt[i * 5 + 1] = (double) tmp_record -> assign_int();
//     result_prt[i * 5 + 2] = (double) tmp_record -> link_ID();
//     // the count of unit time interval (5s)
//     result_prt[i * 5 + 3] = (double) tmp_record -> link_start_int();
//     result_prt[i * 5 + 4] = tmp_record -> flow();
//   }
//   for (size_t i = 0; i < _record.size(); ++i){
//     delete _record[i];
//   }
//   _record.clear();
//   return result;
// }

// py::array_t<double> Mcdta_Api::get_truck_dar_arrival_matrix(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
// {
//   auto start_buf = start_intervals.request();
//   auto end_buf = end_intervals.request();
//   if (start_buf.ndim != 1 || end_buf.ndim != 1){
//     throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_arrival_matrix, input dimension mismatch");
//   }
//   if (start_buf.shape[0] != end_buf.shape[0]){
//     throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_arrival_matrix, input length mismatch");
//   }
//   int l = start_buf.shape[0];
//   int *start_prt = (int *) start_buf.ptr;
//   int *end_prt = (int *) end_buf.ptr;
//   std::vector<dar_record*> _record = std::vector<dar_record*>();

//   for (int t = 0; t < l; ++t){
//     // printf("Current processing time: %d\n", t);
//     if (end_prt[t] < start_prt[t]){
//         throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_arrival_matrix, end time smaller than start time");
//     }
//     if (end_prt[t] > get_cur_loading_interval()){
//         throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_arrival_matrix, loaded data not enough");
//     }
//     for (size_t i = 0; i<m_link_vec.size(); ++i){
//       MNM_DTA_GRADIENT::add_dar_records_curb_arrival_truck(_record, m_link_vec[i], m_path_set_truck, TFlt(start_prt[t]), TFlt(end_prt[t]));
//     }
//   }
//   // _record.size() = num_timesteps x num_links x num_path x num_assign_timesteps
//   // path_ID, assign_time, link_ID, start_int, flow
//   int new_shape [2] = { (int) _record.size(), 5}; 
//   auto result = py::array_t<double>(new_shape);
//   auto result_buf = result.request();
//   double *result_prt = (double *) result_buf.ptr;
//   dar_record* tmp_record;
//   for (size_t i = 0; i < _record.size(); ++i){
//     tmp_record = _record[i];
//     result_prt[i * 5 + 0] = (double) tmp_record -> path_ID();
//     // the count of 1 min interval
//     result_prt[i * 5 + 1] = (double) tmp_record -> assign_int();
//     result_prt[i * 5 + 2] = (double) tmp_record -> link_ID();
//     // the count of unit time interval (5s)
//     result_prt[i * 5 + 3] = (double) tmp_record -> link_start_int();
//     result_prt[i * 5 + 4] = tmp_record -> flow();
//   }
//   for (size_t i = 0; i < _record.size(); ++i){
//     delete _record[i];
//   }
//   _record.clear();
//   return result;
// }

// py::array_t<double> Mcdta_Api::get_rh_dar_arrival_matrix(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
// {
//   auto start_buf = start_intervals.request();
//   auto end_buf = end_intervals.request();
//   if (start_buf.ndim != 1 || end_buf.ndim != 1){
//     throw std::runtime_error("Error, Mcdta_Api::get_rh_dar_arrival_matrix, input dimension mismatch");
//   }
//   if (start_buf.shape[0] != end_buf.shape[0]){
//     throw std::runtime_error("Error, Mcdta_Api::get_rh_dar_arrival_matrix, input length mismatch");
//   }
//   int l = start_buf.shape[0];
//   int *start_prt = (int *) start_buf.ptr;
//   int *end_prt = (int *) end_buf.ptr;
//   std::vector<dar_record*> _record = std::vector<dar_record*>();

//   for (int t = 0; t < l; ++t){
//     // printf("Current processing time: %d\n", t);
//     if (end_prt[t] < start_prt[t]){
//         throw std::runtime_error("Error, Mcdta_Api::get_rh_dar_arrival_matrix, end time smaller than start time");
//     }
//     if (end_prt[t] > get_cur_loading_interval()){
//         throw std::runtime_error("Error, Mcdta_Api::get_rh_dar_arrival_matrix, loaded data not enough");
//     }
//     for (size_t i = 0; i<m_link_vec.size(); ++i){
//       MNM_DTA_GRADIENT::add_dar_records_curb_arrival_rh(_record, m_link_vec[i], m_path_set_rh, TFlt(start_prt[t]), TFlt(end_prt[t]));
//     }
//   }
//   // _record.size() = num_timesteps x num_links x num_path x num_assign_timesteps
//   // path_ID, assign_time, link_ID, start_int, flow
//   int new_shape [2] = { (int) _record.size(), 5}; 
//   auto result = py::array_t<double>(new_shape);
//   auto result_buf = result.request();
//   double *result_prt = (double *) result_buf.ptr;
//   dar_record* tmp_record;
//   for (size_t i = 0; i < _record.size(); ++i){
//     tmp_record = _record[i];
//     result_prt[i * 5 + 0] = (double) tmp_record -> path_ID();
//     // the count of 1 min interval
//     result_prt[i * 5 + 1] = (double) tmp_record -> assign_int();
//     result_prt[i * 5 + 2] = (double) tmp_record -> link_ID();
//     // the count of unit time interval (5s)
//     result_prt[i * 5 + 3] = (double) tmp_record -> link_start_int();
//     result_prt[i * 5 + 4] = tmp_record -> flow();
//   }
//   for (size_t i = 0; i < _record.size(); ++i){
//     delete _record[i];
//   }
//   _record.clear();
//   return result;
// }

// py::array_t<double> Mcdta_Api::get_truck_dar_departure_matrix(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
// {
//   auto start_buf = start_intervals.request();
//   auto end_buf = end_intervals.request();
//   if (start_buf.ndim != 1 || end_buf.ndim != 1){
//     throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_departure_matrix, input dimension mismatch");
//   }
//   if (start_buf.shape[0] != end_buf.shape[0]){
//     throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_departure_matrix, input length mismatch");
//   }
//   int l = start_buf.shape[0];
//   int *start_prt = (int *) start_buf.ptr;
//   int *end_prt = (int *) end_buf.ptr;
//   std::vector<dar_record*> _record = std::vector<dar_record*>();

//   for (int t = 0; t < l; ++t){
//     // printf("Current processing time: %d\n", t);
//     if (end_prt[t] < start_prt[t]){
//         throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_departure_matrix, end time smaller than start time");
//     }
//     if (end_prt[t] > get_cur_loading_interval()){
//         throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_departure_matrix, loaded data not enough");
//     }
//     for (size_t i = 0; i<m_link_vec.size(); ++i){
//       MNM_DTA_GRADIENT::add_dar_records_curb_departure_truck(_record, m_link_vec[i], m_path_set_truck, TFlt(start_prt[t]), TFlt(end_prt[t]));
//     }
//   }
//   // _record.size() = num_timesteps x num_links x num_path x num_assign_timesteps
//   // path_ID, assign_time, link_ID, start_int, flow
//   int new_shape [2] = { (int) _record.size(), 5}; 
//   auto result = py::array_t<double>(new_shape);
//   auto result_buf = result.request();
//   double *result_prt = (double *) result_buf.ptr;
//   dar_record* tmp_record;
//   for (size_t i = 0; i < _record.size(); ++i){
//     tmp_record = _record[i];
//     result_prt[i * 5 + 0] = (double) tmp_record -> path_ID();
//     // the count of 1 min interval
//     result_prt[i * 5 + 1] = (double) tmp_record -> assign_int();
//     result_prt[i * 5 + 2] = (double) tmp_record -> link_ID();
//     // the count of unit time interval (5s)
//     result_prt[i * 5 + 3] = (double) tmp_record -> link_start_int();
//     result_prt[i * 5 + 4] = tmp_record -> flow();
//   }
//   for (size_t i = 0; i < _record.size(); ++i){
//     delete _record[i];
//   }
//   _record.clear();
//   return result;
// }

// py::array_t<double> Mcdta_Api::get_rh_dar_departure_matrix(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
// {
//   auto start_buf = start_intervals.request();
//   auto end_buf = end_intervals.request();
//   if (start_buf.ndim != 1 || end_buf.ndim != 1){
//     throw std::runtime_error("Error, Mcdta_Api::get_rh_dar_departure_matrix, input dimension mismatch");
//   }
//   if (start_buf.shape[0] != end_buf.shape[0]){
//     throw std::runtime_error("Error, Mcdta_Api::get_rh_dar_departure_matrix, input length mismatch");
//   }
//   int l = start_buf.shape[0];
//   int *start_prt = (int *) start_buf.ptr;
//   int *end_prt = (int *) end_buf.ptr;
//   std::vector<dar_record*> _record = std::vector<dar_record*>();

//   for (int t = 0; t < l; ++t){
//     // printf("Current processing time: %d\n", t);
//     if (end_prt[t] < start_prt[t]){
//         throw std::runtime_error("Error, Mcdta_Api::get_rh_dar_departure_matrix, end time smaller than start time");
//     }
//     if (end_prt[t] > get_cur_loading_interval()){
//         throw std::runtime_error("Error, Mcdta_Api::get_rh_dar_departure_matrix, loaded data not enough");
//     }
//     for (size_t i = 0; i<m_link_vec.size(); ++i){
//       MNM_DTA_GRADIENT::add_dar_records_curb_departure_rh(_record, m_link_vec[i], m_path_set_rh, TFlt(start_prt[t]), TFlt(end_prt[t]));
//     }
//   }
//   // _record.size() = num_timesteps x num_links x num_path x num_assign_timesteps
//   // path_ID, assign_time, link_ID, start_int, flow
//   int new_shape [2] = { (int) _record.size(), 5}; 
//   auto result = py::array_t<double>(new_shape);
//   auto result_buf = result.request();
//   double *result_prt = (double *) result_buf.ptr;
//   dar_record* tmp_record;
//   for (size_t i = 0; i < _record.size(); ++i){
//     tmp_record = _record[i];
//     result_prt[i * 5 + 0] = (double) tmp_record -> path_ID();
//     // the count of 1 min interval
//     result_prt[i * 5 + 1] = (double) tmp_record -> assign_int();
//     result_prt[i * 5 + 2] = (double) tmp_record -> link_ID();
//     // the count of unit time interval (5s)
//     result_prt[i * 5 + 3] = (double) tmp_record -> link_start_int();
//     result_prt[i * 5 + 4] = tmp_record -> flow();
//   }
//   for (size_t i = 0; i < _record.size(); ++i){
//     delete _record[i];
//   }
//   _record.clear();
//   return result;
// }

// new functions for separate count, TT and curb observations - jiachao 03/31/2025
py::array_t<double> Mcdta_Api::get_car_link_tt_sep(py::array_t<double>start_intervals)
{
  auto start_buf = start_intervals.request();
  if (start_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get_car_link_tt_sep, input dimension mismatch");
  }
  int l = start_buf.shape[0];
  int new_shape [2] = { (int) m_link_vec_tt.size(), l}; 

  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  double *start_prt = (double *) start_buf.ptr;
  for (int t = 0; t < l; ++t){
    if (start_prt[t] > get_cur_loading_interval()){
      throw std::runtime_error("Error, Mcdta_Api::get_car_link_tt_sep, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_tt.size(); ++i){
      MNM_Dlink_Multiclass_Curb* _link = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(m_link_vec_tt[i]);
      double _tmp = MNM_DTA_GRADIENT_CURB::get_travel_time_car(_link, TFlt(start_prt[t]), m_mcdta->m_unit_time, get_cur_loading_interval())();
      result_prt[i * l + t] = _tmp * m_mcdta -> m_unit_time;
    }
  }
  return result;
}

py::array_t<double> Mcdta_Api::get_truck_link_tt_sep(py::array_t<double>start_intervals)
{
  auto start_buf = start_intervals.request();
  if (start_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get_truck_link_tt_sep, input dimension mismatch");
  }
  int l = start_buf.shape[0];
  int new_shape [2] = { (int) m_link_vec_tt.size(), l}; 

  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  double *start_prt = (double *) start_buf.ptr;
  for (int t = 0; t < l; ++t){
    if (start_prt[t] > get_cur_loading_interval()){
      throw std::runtime_error("Error, Mcdta_Api::get_truck_link_tt_sep, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_tt.size(); ++i){
      MNM_Dlink_Multiclass_Curb* _link = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(m_link_vec_tt[i]);
      double _tmp = MNM_DTA_GRADIENT_CURB::get_travel_time_truck(_link, TFlt(start_prt[t]), m_mcdta -> m_unit_time, get_cur_loading_interval())();
      result_prt[i * l + t] = _tmp * m_mcdta -> m_unit_time;  // in unit of second (not interval)
    }
  }
  return result;
}

// count
py::array_t<double> Mcdta_Api::get_link_car_inflow_sep(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
{
  auto start_buf = start_intervals.request();
  auto end_buf = end_intervals.request();
  if (start_buf.ndim != 1 || end_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get link_car_inflow_sep, input dimension mismatch");
  }
  if (start_buf.shape[0] != end_buf.shape[0]){
    throw std::runtime_error("Error, Mcdta_Api::get link_car_inflow_sep, input length mismatch");
  }
  int l = start_buf.shape[0];
  int new_shape [2] = { (int) m_link_vec_count.size(), l};
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  int *start_prt = (int *) start_buf.ptr;
  int *end_prt = (int *) end_buf.ptr;
  for (int t = 0; t < l; ++t){
    if (end_prt[t] < start_prt[t]){
      throw std::runtime_error("Error, Mcdta_Api::get link_car_inflow_sep, end time smaller than start time");
    }
    if (end_prt[t] > get_cur_loading_interval()){
      throw std::runtime_error("Error, Mcdta_Api::get link_car_inflow_sep, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_count.size(); ++i){
      MNM_Dlink_Multiclass_Curb* _link = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(m_link_vec_count[i]);
      result_prt[i * l + t] = MNM_DTA_GRADIENT_CURB::get_link_inflow_car(_link, TFlt(start_prt[t]), TFlt(end_prt[t]))();
    }
  }
  return result;
}

py::array_t<double> Mcdta_Api::get_link_truck_inflow_sep(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
{
  auto start_buf = start_intervals.request();
  auto end_buf = end_intervals.request();
  if (start_buf.ndim != 1 || end_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get_link_truck_inflow_sep, input dimension mismatch");
  }
  if (start_buf.shape[0] != end_buf.shape[0]){
    throw std::runtime_error("Error, Mcdta_Api::get_link_truck_inflow_sep, input length mismatch");
  }
  int l = start_buf.shape[0];
  int new_shape [2] = { (int) m_link_vec_count.size(), l};
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  int *start_prt = (int *) start_buf.ptr;
  int *end_prt = (int *) end_buf.ptr;
  for (int t = 0; t < l; ++t){
    if (end_prt[t] < start_prt[t]){
      throw std::runtime_error("Error, Mcdta_Api::get_link_truck_inflow_sep, end time smaller than start time");
    }
    if (end_prt[t] > get_cur_loading_interval()){
      throw std::runtime_error("Error, Mcdta_Api::get_link_truck_inflow_sep, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_count.size(); ++i){
      MNM_Dlink_Multiclass_Curb* _link = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(m_link_vec_count[i]);
      result_prt[i * l + t] = MNM_DTA_GRADIENT_CURB::get_link_inflow_truck(_link, TFlt(start_prt[t]), TFlt(end_prt[t]))();
    }
  }
  return result;
}

py::array_t<double> Mcdta_Api::get_link_rh_inflow_sep(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
{
  auto start_buf = start_intervals.request();
  auto end_buf = end_intervals.request();
  if (start_buf.ndim != 1 || end_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get_link_rh_inflow_sep, input dimension mismatch");
  }
  if (start_buf.shape[0] != end_buf.shape[0]){
    throw std::runtime_error("Error, Mcdta_Api::get_link_rh_inflow_sep, input length mismatch");
  }
  int l = start_buf.shape[0];
  int new_shape [2] = { (int) m_link_vec_count.size(), l};
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  int *start_prt = (int *) start_buf.ptr;
  int *end_prt = (int *) end_buf.ptr;
  for (int t = 0; t < l; ++t){
    if (end_prt[t] < start_prt[t]){
      throw std::runtime_error("Error, Mcdta_Api::get_link_rh_inflow_sep, end time smaller than start time");
    }
    if (end_prt[t] > get_cur_loading_interval()){
      throw std::runtime_error("Error, Mcdta_Api::get_link_rh_inflow_sep, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_count.size(); ++i){
      MNM_Dlink_Multiclass_Curb* _link = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(m_link_vec_count[i]);
      result_prt[i * l + t] = MNM_DTA_GRADIENT_CURB::get_link_inflow_rh(_link, TFlt(start_prt[t]), TFlt(end_prt[t]))();
    }
  }
  return result;
}

py::array_t<double> Mcdta_Api::get_link_k_car_total_robust(py::array_t<int> timestamps)
{
  auto timestamp_buf = timestamps.request();
  if (timestamp_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get link k car total (including parking) robust, input dimension mismatch");
  }
  int l = timestamp_buf.shape[0];
  int new_shape [2] = { (int) m_link_vec_density.size(), l}; 

  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  int *timestamp_prt = (int *) timestamp_buf.ptr;

  for (int t = 0; t < l; ++t){
    if (timestamp_prt[t] > get_cur_loading_interval()){
      throw std::runtime_error("Error, Mcdta_Api::get link k car total (including parking) robust, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_density.size(); ++i){
      MNM_Dlink_Multiclass_Curb* _link = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(m_link_vec_density[i]);
      double _density = MNM_DTA_GRADIENT_CURB::get_link_density_car_robust(_link, TFlt(timestamp_prt[t]), m_mcdta -> m_current_loading_interval(), TInt(5)); // seconds
      double _density_rh = MNM_DTA_GRADIENT_CURB::get_link_density_rh_robust(_link, TFlt(timestamp_prt[t]), m_mcdta -> m_current_loading_interval(), TInt(5)); // seconds
      double _density_stop = MNM_DTA_GRADIENT_CURB::get_link_density_stop_rh_robust(_link, TFlt(timestamp_prt[t]), m_mcdta -> m_current_loading_interval(), TInt(5)); // seconds
      result_prt[i * l + t] = _density + _density_rh + _density_stop;
    }
  }
  return result;
}

py::array_t<double> Mcdta_Api::get_link_k_truck_total_robust(py::array_t<int> timestamps)
{
  auto timestamp_buf = timestamps.request();
  if (timestamp_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get link k truck total (including parking) robust, input dimension mismatch");
  }
  int l = timestamp_buf.shape[0];
  int new_shape [2] = { (int) m_link_vec_density.size(), l}; 

  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  int *timestamp_prt = (int *) timestamp_buf.ptr;

  for (int t = 0; t < l; ++t){
    if (timestamp_prt[t] > get_cur_loading_interval()){
      throw std::runtime_error("Error, Mcdta_Api::get link k truck total (including parking) robust, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_density.size(); ++i){
      MNM_Dlink_Multiclass_Curb* _link = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(m_link_vec_density[i]);
      double _density = MNM_DTA_GRADIENT_CURB::get_link_density_truck_robust(_link, TFlt(timestamp_prt[t]), m_mcdta -> m_current_loading_interval(), TInt(5)); // seconds
      double _density_stop = MNM_DTA_GRADIENT_CURB::get_link_density_stop_truck_robust(_link, TFlt(timestamp_prt[t]), m_mcdta -> m_current_loading_interval(), TInt(5)); // seconds
      result_prt[i * l + t] = _density + _density_stop;
    }
  }
  return result;
}

// seperate moving and parking density
py::array_t<double> Mcdta_Api::get_link_k_car_moving_robust(py::array_t<int>timestamps)
{
  auto timestamp_buf = timestamps.request();
  if (timestamp_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get link k car moving robust, input dimension mismatch");
  }
  int l = timestamp_buf.shape[0];
  int new_shape [2] = { (int) m_link_vec_density.size(), l}; 

  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  int *timestamp_prt = (int *) timestamp_buf.ptr;

  for (int t = 0; t < l; ++t){
    if (timestamp_prt[t] > get_cur_loading_interval()){
      throw std::runtime_error("Error, Mcdta_Api::get link k car moving robust, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_density.size(); ++i){
      MNM_Dlink_Multiclass_Curb* _link = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(m_link_vec_density[i]);
      double _density = MNM_DTA_GRADIENT_CURB::get_link_density_car_robust(_link, TFlt(timestamp_prt[t]), m_mcdta -> m_current_loading_interval(), TInt(5));
      double _density_rh = MNM_DTA_GRADIENT_CURB::get_link_density_rh_robust(_link, TFlt(timestamp_prt[t]), m_mcdta -> m_current_loading_interval(), TInt(5)); 
      result_prt[i * l + t] = _density + _density_rh;
    }
  }
  return result;
}
py::array_t<double> Mcdta_Api::get_link_k_truck_moving_robust(py::array_t<int>timestamps)
{
  auto timestamp_buf = timestamps.request();
  if (timestamp_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get link k truck moving robust, input dimension mismatch");
  }
  int l = timestamp_buf.shape[0];
  int new_shape [2] = { (int) m_link_vec_density.size(), l}; 

  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  int *timestamp_prt = (int *) timestamp_buf.ptr;

  for (int t = 0; t < l; ++t){
    if (timestamp_prt[t] > get_cur_loading_interval()){
      throw std::runtime_error("Error, Mcdta_Api::get link k truck moving robust, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_density.size(); ++i){
      MNM_Dlink_Multiclass_Curb* _link = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(m_link_vec_density[i]);
      double _density = MNM_DTA_GRADIENT_CURB::get_link_density_truck_robust(_link, TFlt(timestamp_prt[t]), m_mcdta -> m_current_loading_interval(), TInt(5));
      result_prt[i * l + t] = _density;
    }
  }
  return result;
}

py::array_t<double> Mcdta_Api::get_link_k_car_parking_robust(py::array_t<int>timestamps)
{
  auto timestamp_buf = timestamps.request();
  if (timestamp_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get link k car parking robust, input dimension mismatch");
  }
  int l = timestamp_buf.shape[0];
  int new_shape [2] = { (int) m_link_vec_density.size(), l}; 

  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  int *timestamp_prt = (int *) timestamp_buf.ptr;

  for (int t = 0; t < l; ++t){
    if (timestamp_prt[t] > get_cur_loading_interval()){
      throw std::runtime_error("Error, Mcdta_Api::get link k car parking robust, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_density.size(); ++i){
      MNM_Dlink_Multiclass_Curb* _link = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(m_link_vec_density[i]);
      double _density_stop = MNM_DTA_GRADIENT_CURB::get_link_density_stop_rh_robust(_link, TFlt(timestamp_prt[t]), m_mcdta -> m_current_loading_interval(), TInt(5)); 
      result_prt[i * l + t] = _density_stop;
    }
  }
  return result;
}
py::array_t<double> Mcdta_Api::get_link_k_truck_parking_robust(py::array_t<int>timestamps)
{
  auto timestamp_buf = timestamps.request();
  if (timestamp_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get link k truck parking robust, input dimension mismatch");
  }
  int l = timestamp_buf.shape[0];
  int new_shape [2] = { (int) m_link_vec_density.size(), l}; 

  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  int *timestamp_prt = (int *) timestamp_buf.ptr;

  for (int t = 0; t < l; ++t){
    if (timestamp_prt[t] > get_cur_loading_interval()){
      throw std::runtime_error("Error, Mcdta_Api::get link k truck parking robust, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_density.size(); ++i){
      MNM_Dlink_Multiclass_Curb* _link = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(m_link_vec_density[i]);
      double _density_stop = MNM_DTA_GRADIENT_CURB::get_link_density_stop_truck_robust(_link, TFlt(timestamp_prt[t]), m_mcdta -> m_current_loading_interval(), TInt(5)); 
      result_prt[i * l + t] = _density_stop;
    }
  }
  return result;
}


// curb
py::array_t<double> Mcdta_Api::get_link_truck_curb_inflow_sep(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
{
  auto start_buf = start_intervals.request();
  auto end_buf = end_intervals.request();
  if (start_buf.ndim != 1 || end_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get link truck curb inflow sep, input dimension mismatch");
  }
  if (start_buf.shape[0] != end_buf.shape[0]){
    throw std::runtime_error("Error, Mcdta_Api::get link truck curb inflow sep, input length mismatch");
  }
  int l = start_buf.shape[0];
  int new_shape [2] = { (int) m_link_vec_curb.size(), l};// 2-d
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  int *start_prt = (int *) start_buf.ptr;
  int *end_prt = (int *) end_buf.ptr;
  // l is the intervel numbers 
  for (int t = 0; t < l; ++t){
    if (end_prt[t] < start_prt[t]){
      throw std::runtime_error("Error, Mcdta_Api::get link truck curb inflow sep, end time smaller than start time");
    }
    if (end_prt[t] > get_cur_loading_interval()){
      throw std::runtime_error("Error, Mcdta_Api::get link truck curb inflow sep, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_curb.size(); ++i){
      MNM_Dlink_Multiclass_Curb* _link = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(m_link_vec_curb[i]);
      result_prt[i * l + t] = MNM_DTA_GRADIENT_CURB::get_curb_inflow_truck(_link, TFlt(start_prt[t]), TFlt(end_prt[t]))();
    }
  }
  return result;
}

py::array_t<double> Mcdta_Api::get_link_rh_curb_inflow_sep(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
{
  auto start_buf = start_intervals.request();
  auto end_buf = end_intervals.request();
  if (start_buf.ndim != 1 || end_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get link rh curb inflow sep, input dimension mismatch");
  }
  if (start_buf.shape[0] != end_buf.shape[0]){
    throw std::runtime_error("Error, Mcdta_Api::get link rh curb inflow sep, input length mismatch");
  }
  int l = start_buf.shape[0];
  int new_shape [2] = { (int) m_link_vec_curb.size(), l};
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  int *start_prt = (int *) start_buf.ptr;
  int *end_prt = (int *) end_buf.ptr;
  // l is the intervel numbers 
  for (int t = 0; t < l; ++t){
    if (end_prt[t] < start_prt[t]){
      throw std::runtime_error("Error, Mcdta_Api::get link rh curb inflow sep, end time smaller than start time");
    }
    if (end_prt[t] > get_cur_loading_interval()){
      throw std::runtime_error("Error, Mcdta_Api::get link rh curb inflow sep, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_curb.size(); ++i){
      MNM_Dlink_Multiclass_Curb* _link = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(m_link_vec_curb[i]);
      result_prt[i * l + t] = MNM_DTA_GRADIENT_CURB::get_curb_inflow_rh(_link, TFlt(start_prt[t]), TFlt(end_prt[t]))();
    }
  }
  return result;
}

py::array_t<double> Mcdta_Api::get_link_truck_curb_outflow_sep(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
{
  auto start_buf = start_intervals.request();
  auto end_buf = end_intervals.request();
  if (start_buf.ndim != 1 || end_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get link truck curb outflow sep, input dimension mismatch");
  }
  if (start_buf.shape[0] != end_buf.shape[0]){
    throw std::runtime_error("Error, Mcdta_Api::get link truck curb outflow sep, input length mismatch");
  }
  int l = start_buf.shape[0];
  int new_shape [2] = { (int) m_link_vec_curb.size(), l};
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  int *start_prt = (int *) start_buf.ptr;
  int *end_prt = (int *) end_buf.ptr;
  // l is the intervel numbers 
  for (int t = 0; t < l; ++t){
    if (end_prt[t] < start_prt[t]){
      throw std::runtime_error("Error, Mcdta_Api::get link truck curb outflow sep, end time smaller than start time");
    }
    if (end_prt[t] > get_cur_loading_interval()){
      throw std::runtime_error("Error, Mcdta_Api::get link truck curb outflow sep, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_curb.size(); ++i){
      MNM_Dlink_Multiclass_Curb* _link = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(m_link_vec_curb[i]);
      result_prt[i * l + t] = MNM_DTA_GRADIENT_CURB::get_curb_outflow_truck(_link, TFlt(start_prt[t]), TFlt(end_prt[t]))();
    }
  }
  return result;
}

py::array_t<double> Mcdta_Api::get_link_rh_curb_outflow_sep(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
{
  auto start_buf = start_intervals.request();
  auto end_buf = end_intervals.request();
  if (start_buf.ndim != 1 || end_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get link rh curb outflow sep, input dimension mismatch");
  }
  if (start_buf.shape[0] != end_buf.shape[0]){
    throw std::runtime_error("Error, Mcdta_Api::get link rh curb outflow sep, input length mismatch");
  }
  int l = start_buf.shape[0];
  int new_shape [2] = { (int) m_link_vec_curb.size(), l};// 2-d?
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  int *start_prt = (int *) start_buf.ptr;
  int *end_prt = (int *) end_buf.ptr;
  // l is the intervel numbers 
  for (int t = 0; t < l; ++t){
    if (end_prt[t] < start_prt[t]){
      throw std::runtime_error("Error, Mcdta_Api::get link rh curb outflow sep, end time smaller than start time");
    }
    if (end_prt[t] > get_cur_loading_interval()){
      throw std::runtime_error("Error, Mcdta_Api::get link rh curb outflow sep, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_curb.size(); ++i){
      MNM_Dlink_Multiclass_Curb* _link = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(m_link_vec_curb[i]);
      result_prt[i * l + t] = MNM_DTA_GRADIENT_CURB::get_curb_outflow_rh(_link, TFlt(start_prt[t]), TFlt(end_prt[t]))();
    }
  }
  return result;
}

// count DAR
py::array_t<double> Mcdta_Api::get_car_dar_matrix_sep(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
{
  auto start_buf = start_intervals.request();
  auto end_buf = end_intervals.request();
  if (start_buf.ndim != 1 || end_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get car_dar_matrix_sep, input dimension mismatch");
  }
  if (start_buf.shape[0] != end_buf.shape[0]){
    throw std::runtime_error("Error, Mcdta_Api::get car_dar_matrix_sep, input length mismatch");
  }

  int l = start_buf.shape[0];
  int *start_prt = (int *) start_buf.ptr;
  int *end_prt = (int *) end_buf.ptr;
  std::vector<dar_record*> _record = std::vector<dar_record*>();

  for (int t = 0; t < l; ++t){
    if (end_prt[t] < start_prt[t]){
      throw std::runtime_error("Error, Mcdta_Api::get car_dar_matrix_sep, end time smaller than start time");
    }
    if (end_prt[t] > get_cur_loading_interval()){
      throw std::runtime_error("Error, Mcdta_Api::get car_dar_matrix_sep, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_count.size(); ++i){
      MNM_Dlink_Multiclass_Curb* _link = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(m_link_vec_count[i]);
      MNM_DTA_GRADIENT_CURB::add_dar_records_car(_record, _link, m_path_set, TFlt(start_prt[t]), TFlt(end_prt[t]));
    }
  }

  int new_shape [2] = { (int) _record.size(), 5}; 
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  dar_record* tmp_record;
  for (size_t i = 0; i < _record.size(); ++i){
    tmp_record = _record[i];
    result_prt[i * 5 + 0] = (double) tmp_record -> path_ID();
    result_prt[i * 5 + 1] = (double) tmp_record -> assign_int();
    result_prt[i * 5 + 2] = (double) tmp_record -> link_ID();
    result_prt[i * 5 + 3] = (double) tmp_record -> link_start_int();
    result_prt[i * 5 + 4] = tmp_record -> flow();
  }
  for (size_t i = 0; i < _record.size(); ++i){
    delete _record[i];
  }
  _record.clear();
  return result;
}

py::array_t<double> Mcdta_Api::get_truck_dar_matrix_sep(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
{
  auto start_buf = start_intervals.request();
  auto end_buf = end_intervals.request();
  if (start_buf.ndim != 1 || end_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_matrix_sep, input dimension mismatch");
  }
  if (start_buf.shape[0] != end_buf.shape[0]){
    throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_matrix_sep, input length mismatch");
  }
  int l = start_buf.shape[0];
  int *start_prt = (int *) start_buf.ptr;
  int *end_prt = (int *) end_buf.ptr;
  std::vector<dar_record*> _record = std::vector<dar_record*>();

  for (int t = 0; t < l; ++t){
    if (end_prt[t] < start_prt[t]){
        throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_matrix_sep, end time smaller than start time");
    }
    if (end_prt[t] > get_cur_loading_interval()){
        throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_matrix_sep, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_count.size(); ++i){
        MNM_Dlink_Multiclass_Curb* _link = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(m_link_vec_count[i]);
        MNM_DTA_GRADIENT_CURB::add_dar_records_truck(_record, _link, m_path_set_truck, TFlt(start_prt[t]), TFlt(end_prt[t]));
    }
  }
  // path_ID, assign_time, link_ID, start_int, flow
  int new_shape [2] = { (int) _record.size(), 5}; 
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  dar_record* tmp_record;
  for (size_t i = 0; i < _record.size(); ++i){
    tmp_record = _record[i];
    result_prt[i * 5 + 0] = (double) tmp_record -> path_ID();
    // the count of 1 min interval
    result_prt[i * 5 + 1] = (double) tmp_record -> assign_int();
    result_prt[i * 5 + 2] = (double) tmp_record -> link_ID();
    // the count of unit time interval (5s)
    result_prt[i * 5 + 3] = (double) tmp_record -> link_start_int();
    result_prt[i * 5 + 4] = tmp_record -> flow();
  }
  for (size_t i = 0; i < _record.size(); ++i){
    delete _record[i];
  }
  _record.clear();
  return result;
}

py::array_t<double> Mcdta_Api::get_rh_dar_matrix_sep(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
{
  auto start_buf = start_intervals.request();
  auto end_buf = end_intervals.request();
  if (start_buf.ndim != 1 || end_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get rh_dar_matrix_sep, input dimension mismatch");
  }
  if (start_buf.shape[0] != end_buf.shape[0]){
    throw std::runtime_error("Error, Mcdta_Api::get rh_dar_matrix_sep, input length mismatch");
  }
  int l = start_buf.shape[0];
  int *start_prt = (int *) start_buf.ptr;
  int *end_prt = (int *) end_buf.ptr;
  std::vector<dar_record*> _record = std::vector<dar_record*>();
  
  for (int t = 0; t < l; ++t){
    // printf("Current processing time: %d\n", t);
    if (end_prt[t] < start_prt[t]){
        throw std::runtime_error("Error, Mcdta_Api::get rh_dar_matrix_sep, end time smaller than start time");
    }
    if (end_prt[t] > get_cur_loading_interval()){
        throw std::runtime_error("Error, Mcdta_Api::get rh_dar_matrix_sep, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_count.size(); ++i){
      MNM_Dlink_Multiclass_Curb* _link = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(m_link_vec_count[i]);
      MNM_DTA_GRADIENT_CURB::add_dar_records_rh(_record, _link, m_path_set_rh, TFlt(start_prt[t]), TFlt(end_prt[t]));
    }
  }
  // _record.size() = num_timesteps x num_links x num_path x num_assign_timesteps
  // path_ID, assign_time, link_ID, start_int, flow
  int new_shape [2] = { (int) _record.size(), 5}; 
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  dar_record* tmp_record;
  for (size_t i = 0; i < _record.size(); ++i){
    tmp_record = _record[i];
    result_prt[i * 5 + 0] = (double) tmp_record -> path_ID();
    // the count of 1 min interval
    result_prt[i * 5 + 1] = (double) tmp_record -> assign_int();
    result_prt[i * 5 + 2] = (double) tmp_record -> link_ID();
    // the count of unit time interval (5s)
    result_prt[i * 5 + 3] = (double) tmp_record -> link_start_int();
    result_prt[i * 5 + 4] = tmp_record -> flow();
  }
  for (size_t i = 0; i < _record.size(); ++i){
    delete _record[i];
  }
  _record.clear();
  return result;
}

// Fujitsu density DARs
py::array_t<double> Mcdta_Api::get_car_dar_k_in(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
{
  auto start_buf = start_intervals.request();
  auto end_buf = end_intervals.request();
  if (start_buf.ndim != 1 || end_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get_car_dar_in_matrix, input dimension mismatch");
  }
  if (start_buf.shape[0] != end_buf.shape[0]){
    throw std::runtime_error("Error, Mcdta_Api::get_car_dar_in_matrix, input length mismatch");
  }
  int l = start_buf.shape[0];
  int *start_prt = (int *) start_buf.ptr;
  int *end_prt = (int *) end_buf.ptr;
  std::vector<dar_record*> _record = std::vector<dar_record*>();
  for (int t = 0; t < l; ++t){
    if (end_prt[t] < start_prt[t]){
      throw std::runtime_error("Error, Mcdta_Api::get_car_dar_in_matrix, end time smaller than start time");
    }
    if (end_prt[t] > get_cur_loading_interval()){
      throw std::runtime_error("Error, Mcdta_Api::get_car_dar_in_matrix, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_density.size(); ++i){
      MNM_Dlink_Multiclass_Curb* _link = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(m_link_vec_density[i]);
      MNM_DTA_GRADIENT_CURB::add_dar_records_car(_record, _link, m_path_set, TFlt(start_prt[t]), TFlt(end_prt[t]));
    }
  }

  int new_shape [2] = { (int) _record.size(), 5}; 
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  dar_record* tmp_record;
  for (size_t i = 0; i < _record.size(); ++i){
    tmp_record = _record[i];
    result_prt[i * 5 + 0] = (double) tmp_record -> path_ID();
    // the count of 1 min interval
    result_prt[i * 5 + 1] = (double) tmp_record -> assign_int();
    result_prt[i * 5 + 2] = (double) tmp_record -> link_ID();
    // the count of unit time interval (5s)
    result_prt[i * 5 + 3] = (double) tmp_record -> link_start_int();
    result_prt[i * 5 + 4] = tmp_record -> flow();
  }
  for (size_t i = 0; i < _record.size(); ++i){
    delete _record[i];
  }
  _record.clear();
  return result;
}

py::array_t<double> Mcdta_Api::get_truck_dar_k_in(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
{
  auto start_buf = start_intervals.request();
  auto end_buf = end_intervals.request();
  if (start_buf.ndim != 1 || end_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_in_matrix, input dimension mismatch");
  }
  if (start_buf.shape[0] != end_buf.shape[0]){
    throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_in_matrix, input length mismatch");
  }
  int l = start_buf.shape[0];
  int *start_prt = (int *) start_buf.ptr;
  int *end_prt = (int *) end_buf.ptr;
  std::vector<dar_record*> _record = std::vector<dar_record*>();
  for (int t = 0; t < l; ++t){
    if (end_prt[t] < start_prt[t]){
      throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_in_matrix, end time smaller than start time");
    }
    if (end_prt[t] > get_cur_loading_interval()){
      throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_in_matrix, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_density.size(); ++i){
      MNM_Dlink_Multiclass_Curb* _link = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(m_link_vec_density[i]);
      MNM_DTA_GRADIENT_CURB::add_dar_records_truck(_record, _link, m_path_set_truck, TFlt(start_prt[t]), TFlt(end_prt[t]));
    }
  }
  // path_ID, assign_time, link_ID, start_int, flow
  int new_shape [2] = { (int) _record.size(), 5}; 
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  dar_record* tmp_record;
  for (size_t i = 0; i < _record.size(); ++i){
    tmp_record = _record[i];
    result_prt[i * 5 + 0] = (double) tmp_record -> path_ID();
    // the count of 1 min interval
    result_prt[i * 5 + 1] = (double) tmp_record -> assign_int();
    result_prt[i * 5 + 2] = (double) tmp_record -> link_ID();
    // the count of unit time interval (5s)
    result_prt[i * 5 + 3] = (double) tmp_record -> link_start_int();
    result_prt[i * 5 + 4] = tmp_record -> flow();
  }
  for (size_t i = 0; i < _record.size(); ++i){
    delete _record[i];
  }
  _record.clear();
  return result;
}

py::array_t<double> Mcdta_Api::get_rh_dar_k_in(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
{
  auto start_buf = start_intervals.request();
  auto end_buf = end_intervals.request();
  if (start_buf.ndim != 1 || end_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get_rh_dar_in_matrix, input dimension mismatch");
  }
  if (start_buf.shape[0] != end_buf.shape[0]){
    throw std::runtime_error("Error, Mcdta_Api::get_rh_dar_in_matrix, input length mismatch");
  }
  int l = start_buf.shape[0];
  int *start_prt = (int *) start_buf.ptr;
  int *end_prt = (int *) end_buf.ptr;
  std::vector<dar_record*> _record = std::vector<dar_record*>();
  for (int t = 0; t < l; ++t){
    if (end_prt[t] < start_prt[t]){
      throw std::runtime_error("Error, Mcdta_Api::get_rh_dar_in_matrix, end time smaller than start time");
    }
    if (end_prt[t] > get_cur_loading_interval()){
      throw std::runtime_error("Error, Mcdta_Api::get_rh_dar_in_matrix, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_density.size(); ++i){
      MNM_Dlink_Multiclass_Curb* _link = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(m_link_vec_density[i]);
      MNM_DTA_GRADIENT_CURB::add_dar_records_rh(_record, _link, m_path_set_rh, TFlt(start_prt[t]), TFlt(end_prt[t]));
    }
  }
  // path_ID, assign_time, link_ID, start_int, flow
  int new_shape [2] = { (int) _record.size(), 5}; 
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  dar_record* tmp_record;
  for (size_t i = 0; i < _record.size(); ++i){
    tmp_record = _record[i];
    result_prt[i * 5 + 0] = (double) tmp_record -> path_ID();
    // the count of 1 min interval
    result_prt[i * 5 + 1] = (double) tmp_record -> assign_int();
    result_prt[i * 5 + 2] = (double) tmp_record -> link_ID();
    // the count of unit time interval (5s)
    result_prt[i * 5 + 3] = (double) tmp_record -> link_start_int();
    result_prt[i * 5 + 4] = tmp_record -> flow();
  }
  for (size_t i = 0; i < _record.size(); ++i){
    delete _record[i];
  }
  _record.clear();
  return result;
}

py::array_t<double> Mcdta_Api::get_car_dar_k_out(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
{
  auto start_buf = start_intervals.request();
  auto end_buf = end_intervals.request();
  if (start_buf.ndim != 1 || end_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get_car_dar_out_matrix, input dimension mismatch");
  }
  if (start_buf.shape[0] != end_buf.shape[0]){
    throw std::runtime_error("Error, Mcdta_Api::get_car_dar_out_matrix, input length mismatch");
  }
  int l = start_buf.shape[0];
  int *start_prt = (int *) start_buf.ptr;
  int *end_prt = (int *) end_buf.ptr;
  std::vector<dar_record*> _record = std::vector<dar_record*>();
  for (int t = 0; t < l; ++t){
    if (end_prt[t] < start_prt[t]){
      throw std::runtime_error("Error, Mcdta_Api::get_car_dar_out_matrix, end time smaller than start time");
    }
    if (end_prt[t] > get_cur_loading_interval()){
      throw std::runtime_error("Error, Mcdta_Api::get_car_dar_out_matrix, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_density.size(); ++i){
      MNM_Dlink_Multiclass_Curb* _link = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(m_link_vec_density[i]);
      MNM_DTA_GRADIENT_CURB::add_dar_records_car_out(_record, _link, m_path_set, TFlt(start_prt[t]), TFlt(end_prt[t]));
    }
  }

  int new_shape [2] = { (int) _record.size(), 5}; 
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  dar_record* tmp_record;
  for (size_t i = 0; i < _record.size(); ++i){
    tmp_record = _record[i];
    result_prt[i * 5 + 0] = (double) tmp_record -> path_ID();
    // the count of 1 min interval
    result_prt[i * 5 + 1] = (double) tmp_record -> assign_int();
    result_prt[i * 5 + 2] = (double) tmp_record -> link_ID();
    // the count of unit time interval (5s)
    result_prt[i * 5 + 3] = (double) tmp_record -> link_start_int();
    result_prt[i * 5 + 4] = tmp_record -> flow();
  }
  for (size_t i = 0; i < _record.size(); ++i){
    delete _record[i];
  }
  _record.clear();
  return result;
}

py::array_t<double> Mcdta_Api::get_truck_dar_k_out(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
{
  auto start_buf = start_intervals.request();
  auto end_buf = end_intervals.request();
  if (start_buf.ndim != 1 || end_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_out_matrix, input dimension mismatch");
  }
  if (start_buf.shape[0] != end_buf.shape[0]){
    throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_out_matrix, input length mismatch");
  }
  int l = start_buf.shape[0];
  int *start_prt = (int *) start_buf.ptr;
  int *end_prt = (int *) end_buf.ptr;
  std::vector<dar_record*> _record = std::vector<dar_record*>();
  for (int t = 0; t < l; ++t){
    if (end_prt[t] < start_prt[t]){
      throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_out_matrix, end time smaller than start time");
    }
    if (end_prt[t] > get_cur_loading_interval()){
      throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_out_matrix, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_density.size(); ++i){
      MNM_Dlink_Multiclass_Curb* _link = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(m_link_vec_density[i]);
      MNM_DTA_GRADIENT_CURB::add_dar_records_truck_out(_record, _link, m_path_set_truck, TFlt(start_prt[t]), TFlt(end_prt[t]));
    }
  }
  // path_ID, assign_time, link_ID, start_int, flow
  int new_shape [2] = { (int) _record.size(), 5}; 
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  dar_record* tmp_record;
  for (size_t i = 0; i < _record.size(); ++i){
    tmp_record = _record[i];
    result_prt[i * 5 + 0] = (double) tmp_record -> path_ID();
    // the count of 1 min interval
    result_prt[i * 5 + 1] = (double) tmp_record -> assign_int();
    result_prt[i * 5 + 2] = (double) tmp_record -> link_ID();
    // the count of unit time interval (5s)
    result_prt[i * 5 + 3] = (double) tmp_record -> link_start_int();
    result_prt[i * 5 + 4] = tmp_record -> flow();
  }
  for (size_t i = 0; i < _record.size(); ++i){
    delete _record[i];
  }
  _record.clear();
  return result;
}

py::array_t<double> Mcdta_Api::get_rh_dar_k_out(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
{
  auto start_buf = start_intervals.request();
  auto end_buf = end_intervals.request();
  if (start_buf.ndim != 1 || end_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get_rh_dar_out_matrix, input dimension mismatch");
  }
  if (start_buf.shape[0] != end_buf.shape[0]){
    throw std::runtime_error("Error, Mcdta_Api::get_rh_dar_out_matrix, input length mismatch");
  }
  int l = start_buf.shape[0];
  int *start_prt = (int *) start_buf.ptr;
  int *end_prt = (int *) end_buf.ptr;
  std::vector<dar_record*> _record = std::vector<dar_record*>();
  for (int t = 0; t < l; ++t){
    if (end_prt[t] < start_prt[t]){
      throw std::runtime_error("Error, Mcdta_Api::get_rh_dar_out_matrix, end time smaller than start time");
    }
    if (end_prt[t] > get_cur_loading_interval()){
      throw std::runtime_error("Error, Mcdta_Api::get_rh_dar_out_matrix, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_density.size(); ++i){
      MNM_Dlink_Multiclass_Curb* _link = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(m_link_vec_density[i]);
      MNM_DTA_GRADIENT_CURB::add_dar_records_rh_out(_record, _link, m_path_set_rh, TFlt(start_prt[t]), TFlt(end_prt[t]));
    }
  }
  // path_ID, assign_time, link_ID, start_int, flow
  int new_shape [2] = { (int) _record.size(), 5}; 
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  dar_record* tmp_record;
  for (size_t i = 0; i < _record.size(); ++i){
    tmp_record = _record[i];
    result_prt[i * 5 + 0] = (double) tmp_record -> path_ID();
    // the count of 1 min interval
    result_prt[i * 5 + 1] = (double) tmp_record -> assign_int();
    result_prt[i * 5 + 2] = (double) tmp_record -> link_ID();
    // the count of unit time interval (5s)
    result_prt[i * 5 + 3] = (double) tmp_record -> link_start_int();
    result_prt[i * 5 + 4] = tmp_record -> flow();
  }
  for (size_t i = 0; i < _record.size(); ++i){
    delete _record[i];
  }
  _record.clear();
  return result;
}

// density for parking DAR
py::array_t<double> Mcdta_Api::get_truck_dar_k_in_parking(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
{
  auto start_buf = start_intervals.request();
  auto end_buf = end_intervals.request();
  if (start_buf.ndim != 1 || end_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_k in parking, input dimension mismatch");
  }
  if (start_buf.shape[0] != end_buf.shape[0]){
    throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_k in parking, input length mismatch");
  }
  int l = start_buf.shape[0];
  int *start_prt = (int *) start_buf.ptr;
  int *end_prt = (int *) end_buf.ptr;
  std::vector<dar_record*> _record = std::vector<dar_record*>();

  for (int t = 0; t < l; ++t){
    // printf("Current processing time: %d\n", t);
    if (end_prt[t] < start_prt[t]){
        throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_k in parking, end time smaller than start time");
    }
    if (end_prt[t] > get_cur_loading_interval()){
        throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_k in parking, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_density.size(); ++i){
      MNM_Dlink_Multiclass_Curb* _link = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(m_link_vec_density[i]);
      MNM_DTA_GRADIENT_CURB::add_dar_records_curb_arrival_truck(_record, _link, m_path_set_truck, TFlt(start_prt[t]), TFlt(end_prt[t]));
    }
  }

  int new_shape [2] = { (int) _record.size(), 5}; 
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  dar_record* tmp_record;
  for (size_t i = 0; i < _record.size(); ++i){
    tmp_record = _record[i];
    result_prt[i * 5 + 0] = (double) tmp_record -> path_ID();
    // the count of 1 min interval
    result_prt[i * 5 + 1] = (double) tmp_record -> assign_int();
    result_prt[i * 5 + 2] = (double) tmp_record -> link_ID();
    // the count of unit time interval (5s)
    result_prt[i * 5 + 3] = (double) tmp_record -> link_start_int();
    result_prt[i * 5 + 4] = tmp_record -> flow();
  }
  for (size_t i = 0; i < _record.size(); ++i){
    delete _record[i];
  }
  _record.clear();
  return result;
}

py::array_t<double> Mcdta_Api::get_truck_dar_k_out_parking(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
{
  auto start_buf = start_intervals.request();
  auto end_buf = end_intervals.request();
  if (start_buf.ndim != 1 || end_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_k out parking, input dimension mismatch");
  }
  if (start_buf.shape[0] != end_buf.shape[0]){
    throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_k out parking, input length mismatch");
  }
  int l = start_buf.shape[0];
  int *start_prt = (int *) start_buf.ptr;
  int *end_prt = (int *) end_buf.ptr;
  std::vector<dar_record*> _record = std::vector<dar_record*>();

  for (int t = 0; t < l; ++t){
    // printf("Current processing time: %d\n", t);
    if (end_prt[t] < start_prt[t]){
        throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_k out parking, end time smaller than start time");
    }
    if (end_prt[t] > get_cur_loading_interval()){
        throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_k out parking, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_density.size(); ++i){
      MNM_Dlink_Multiclass_Curb* _link = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(m_link_vec_density[i]);
      MNM_DTA_GRADIENT_CURB::add_dar_records_curb_departure_truck(_record, _link, m_path_set_truck, TFlt(start_prt[t]), TFlt(end_prt[t]));
    }
  }

  int new_shape [2] = { (int) _record.size(), 5}; 
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  dar_record* tmp_record;
  for (size_t i = 0; i < _record.size(); ++i){
    tmp_record = _record[i];
    result_prt[i * 5 + 0] = (double) tmp_record -> path_ID();
    // the count of 1 min interval
    result_prt[i * 5 + 1] = (double) tmp_record -> assign_int();
    result_prt[i * 5 + 2] = (double) tmp_record -> link_ID();
    // the count of unit time interval (5s)
    result_prt[i * 5 + 3] = (double) tmp_record -> link_start_int();
    result_prt[i * 5 + 4] = tmp_record -> flow();
  }
  for (size_t i = 0; i < _record.size(); ++i){
    delete _record[i];
  }
  _record.clear();
  return result;
}

py::array_t<double> Mcdta_Api::get_rh_dar_k_in_parking(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
{
  auto start_buf = start_intervals.request();
  auto end_buf = end_intervals.request();
  if (start_buf.ndim != 1 || end_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get_rh_dar_k in parking, input dimension mismatch");
  }
  if (start_buf.shape[0] != end_buf.shape[0]){
    throw std::runtime_error("Error, Mcdta_Api::get_rh_dar_k in parking, input length mismatch");
  }
  int l = start_buf.shape[0];
  int *start_prt = (int *) start_buf.ptr;
  int *end_prt = (int *) end_buf.ptr;
  std::vector<dar_record*> _record = std::vector<dar_record*>();

  for (int t = 0; t < l; ++t){
    // printf("Current processing time: %d\n", t);
    if (end_prt[t] < start_prt[t]){
        throw std::runtime_error("Error, Mcdta_Api::get_rh_dar_k in parking, end time smaller than start time");
    }
    if (end_prt[t] > get_cur_loading_interval()){
        throw std::runtime_error("Error, Mcdta_Api::get_rh_dar_k in parking, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_density.size(); ++i){
      MNM_Dlink_Multiclass_Curb* _link = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(m_link_vec_density[i]);
      MNM_DTA_GRADIENT_CURB::add_dar_records_curb_arrival_rh(_record, _link, m_path_set_rh, TFlt(start_prt[t]), TFlt(end_prt[t]));
    }
  }

  int new_shape [2] = { (int) _record.size(), 5}; 
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  dar_record* tmp_record;
  for (size_t i = 0; i < _record.size(); ++i){
    tmp_record = _record[i];
    result_prt[i * 5 + 0] = (double) tmp_record -> path_ID();
    // the count of 1 min interval
    result_prt[i * 5 + 1] = (double) tmp_record -> assign_int();
    result_prt[i * 5 + 2] = (double) tmp_record -> link_ID();
    // the count of unit time interval (5s)
    result_prt[i * 5 + 3] = (double) tmp_record -> link_start_int();
    result_prt[i * 5 + 4] = tmp_record -> flow();
  }
  for (size_t i = 0; i < _record.size(); ++i){
    delete _record[i];
  }
  _record.clear();
  return result;
}

py::array_t<double> Mcdta_Api::get_rh_dar_k_out_parking(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
{
  auto start_buf = start_intervals.request();
  auto end_buf = end_intervals.request();
  if (start_buf.ndim != 1 || end_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get_rh_dar_k out parking, input dimension mismatch");
  }
  if (start_buf.shape[0] != end_buf.shape[0]){
    throw std::runtime_error("Error, Mcdta_Api::get_rh_dar_k out parking, input length mismatch");
  }
  int l = start_buf.shape[0];
  int *start_prt = (int *) start_buf.ptr;
  int *end_prt = (int *) end_buf.ptr;
  std::vector<dar_record*> _record = std::vector<dar_record*>();

  for (int t = 0; t < l; ++t){
    // printf("Current processing time: %d\n", t);
    if (end_prt[t] < start_prt[t]){
        throw std::runtime_error("Error, Mcdta_Api::get_rh_dar_k out parking, end time smaller than start time");
    }
    if (end_prt[t] > get_cur_loading_interval()){
        throw std::runtime_error("Error, Mcdta_Api::get_rh_dar_k out parking, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_density.size(); ++i){
      MNM_Dlink_Multiclass_Curb* _link = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(m_link_vec_density[i]);
      MNM_DTA_GRADIENT_CURB::add_dar_records_curb_departure_rh(_record, _link, m_path_set_rh, TFlt(start_prt[t]), TFlt(end_prt[t]));
    }
  }

  int new_shape [2] = { (int) _record.size(), 5}; 
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  dar_record* tmp_record;
  for (size_t i = 0; i < _record.size(); ++i){
    tmp_record = _record[i];
    result_prt[i * 5 + 0] = (double) tmp_record -> path_ID();
    // the count of 1 min interval
    result_prt[i * 5 + 1] = (double) tmp_record -> assign_int();
    result_prt[i * 5 + 2] = (double) tmp_record -> link_ID();
    // the count of unit time interval (5s)
    result_prt[i * 5 + 3] = (double) tmp_record -> link_start_int();
    result_prt[i * 5 + 4] = tmp_record -> flow();
  }
  for (size_t i = 0; i < _record.size(); ++i){
    delete _record[i];
  }
  _record.clear();
  return result;
}

// TT DAR
py::array_t<double> Mcdta_Api::get_car_dar_matrix_tt(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
{
  auto start_buf = start_intervals.request();
  auto end_buf = end_intervals.request();
  if (start_buf.ndim != 1 || end_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get car_dar_matrix_tt, input dimension mismatch");
  }
  if (start_buf.shape[0] != end_buf.shape[0]){
    throw std::runtime_error("Error, Mcdta_Api::get car_dar_matrix_tt, input length mismatch");
  }

  int l = start_buf.shape[0];
  int *start_prt = (int *) start_buf.ptr;
  int *end_prt = (int *) end_buf.ptr;
  std::vector<dar_record*> _record = std::vector<dar_record*>();

  for (int t = 0; t < l; ++t){
    // printf("Current processing time: %d\n", t);
    if (end_prt[t] < start_prt[t]){
        throw std::runtime_error("Error, Mcdta_Api::get car_dar_matrix_tt, end time smaller than start time");
    }
    if (end_prt[t] > get_cur_loading_interval()){
        throw std::runtime_error("Error, Mcdta_Api::get car_dar_matrix_tt, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_tt.size(); ++i){
      MNM_Dlink_Multiclass_Curb* _link = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(m_link_vec_tt[i]);
      MNM_DTA_GRADIENT_CURB::add_dar_records_car(_record, _link, m_path_set, TFlt(start_prt[t]), TFlt(end_prt[t]));
    }
  }

  int new_shape [2] = { (int) _record.size(), 5}; 
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  dar_record* tmp_record;
  for (size_t i = 0; i < _record.size(); ++i){
    tmp_record = _record[i];
    result_prt[i * 5 + 0] = (double) tmp_record -> path_ID();
    // the count of 1 min interval
    result_prt[i * 5 + 1] = (double) tmp_record -> assign_int();
    result_prt[i * 5 + 2] = (double) tmp_record -> link_ID();
    // the count of unit time interval (5s)
    result_prt[i * 5 + 3] = (double) tmp_record -> link_start_int();
    result_prt[i * 5 + 4] = tmp_record -> flow();
  }
  for (size_t i = 0; i < _record.size(); ++i){
    delete _record[i];
  }
  _record.clear();
  return result;
}

py::array_t<double> Mcdta_Api::get_truck_dar_matrix_tt(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
{
  auto start_buf = start_intervals.request();
  auto end_buf = end_intervals.request();
  if (start_buf.ndim != 1 || end_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get truck_dar_matrix_tt, input dimension mismatch");
  }
  if (start_buf.shape[0] != end_buf.shape[0]){
    throw std::runtime_error("Error, Mcdta_Api::get truck_dar_matrix_tt, input length mismatch");
  }
  int l = start_buf.shape[0];
  int *start_prt = (int *) start_buf.ptr;
  int *end_prt = (int *) end_buf.ptr;
  std::vector<dar_record*> _record = std::vector<dar_record*>();

  for (int t = 0; t < l; ++t){
    if (end_prt[t] < start_prt[t]){
        throw std::runtime_error("Error, Mcdta_Api::get truck_dar_matrix_tt, end time smaller than start time");
    }
    if (end_prt[t] > get_cur_loading_interval()){
        throw std::runtime_error("Error, Mcdta_Api::get truck_dar_matrix_tt, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_tt.size(); ++i){
      MNM_Dlink_Multiclass_Curb* _link = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(m_link_vec_tt[i]);
      MNM_DTA_GRADIENT_CURB::add_dar_records_truck(_record, _link, m_path_set_truck, TFlt(start_prt[t]), TFlt(end_prt[t]));
    }
  }
  // path_ID, assign_time, link_ID, start_int, flow
  int new_shape [2] = { (int) _record.size(), 5}; 
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  dar_record* tmp_record;
  for (size_t i = 0; i < _record.size(); ++i){
    tmp_record = _record[i];
    result_prt[i * 5 + 0] = (double) tmp_record -> path_ID();
    // the count of 1 min interval
    result_prt[i * 5 + 1] = (double) tmp_record -> assign_int();
    result_prt[i * 5 + 2] = (double) tmp_record -> link_ID();
    // the count of unit time interval (5s)
    result_prt[i * 5 + 3] = (double) tmp_record -> link_start_int();
    result_prt[i * 5 + 4] = tmp_record -> flow();
  }
  for (size_t i = 0; i < _record.size(); ++i){
    delete _record[i];
  }
  _record.clear();
  return result;
}

py::array_t<double> Mcdta_Api::get_rh_dar_matrix_tt(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
{
  auto start_buf = start_intervals.request();
  auto end_buf = end_intervals.request();
  if (start_buf.ndim != 1 || end_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get_rh_dar_matrix_tt, input dimension mismatch");
  }
  if (start_buf.shape[0] != end_buf.shape[0]){
    throw std::runtime_error("Error, Mcdta_Api::get_rh_dar_matrix_tt, input length mismatch");
  }
  int l = start_buf.shape[0];
  int *start_prt = (int *) start_buf.ptr;
  int *end_prt = (int *) end_buf.ptr;
  std::vector<dar_record*> _record = std::vector<dar_record*>();
  
  for (int t = 0; t < l; ++t){
    // printf("Current processing time: %d\n", t);
    if (end_prt[t] < start_prt[t]){
        throw std::runtime_error("Error, Mcdta_Api::get_rh_dar_matrix_tt, end time smaller than start time");
    }
    if (end_prt[t] > get_cur_loading_interval()){
        throw std::runtime_error("Error, Mcdta_Api::get_rh_dar_matrix_tt, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_tt.size(); ++i){
      MNM_Dlink_Multiclass_Curb* _link = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(m_link_vec_tt[i]);
      MNM_DTA_GRADIENT_CURB::add_dar_records_rh(_record, _link, m_path_set_rh, TFlt(start_prt[t]), TFlt(end_prt[t]));
    }
  }

  int new_shape [2] = { (int) _record.size(), 5}; 
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  dar_record* tmp_record;
  for (size_t i = 0; i < _record.size(); ++i){
    tmp_record = _record[i];
    result_prt[i * 5 + 0] = (double) tmp_record -> path_ID();
    // the count of 1 min interval
    result_prt[i * 5 + 1] = (double) tmp_record -> assign_int();
    result_prt[i * 5 + 2] = (double) tmp_record -> link_ID();
    // the count of unit time interval (5s)
    result_prt[i * 5 + 3] = (double) tmp_record -> link_start_int();
    result_prt[i * 5 + 4] = tmp_record -> flow();
  }
  for (size_t i = 0; i < _record.size(); ++i){
    delete _record[i];
  }
  _record.clear();
  return result;
}

// curb DAR
py::array_t<double> Mcdta_Api::get_truck_dar_arrival_matrix_sep(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
{
  auto start_buf = start_intervals.request();
  auto end_buf = end_intervals.request();
  if (start_buf.ndim != 1 || end_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_arrival matrix sep, input dimension mismatch");
  }
  if (start_buf.shape[0] != end_buf.shape[0]){
    throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_arrival matrix sep, input length mismatch");
  }
  int l = start_buf.shape[0];
  int *start_prt = (int *) start_buf.ptr;
  int *end_prt = (int *) end_buf.ptr;
  std::vector<dar_record*> _record = std::vector<dar_record*>();

  for (int t = 0; t < l; ++t){
    // printf("Current processing time: %d\n", t);
    if (end_prt[t] < start_prt[t]){
        throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_arrival matrix sep, end time smaller than start time");
    }
    if (end_prt[t] > get_cur_loading_interval()){
        throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_arrival matrix sep, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_curb.size(); ++i){
      MNM_Dlink_Multiclass_Curb* _link = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(m_link_vec_curb[i]);
      MNM_DTA_GRADIENT_CURB::add_dar_records_curb_arrival_truck(_record, _link, m_path_set_truck, TFlt(start_prt[t]), TFlt(end_prt[t]));
    }
  }

  int new_shape [2] = { (int) _record.size(), 5}; 
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  dar_record* tmp_record;
  for (size_t i = 0; i < _record.size(); ++i){
    tmp_record = _record[i];
    result_prt[i * 5 + 0] = (double) tmp_record -> path_ID();
    // the count of 1 min interval
    result_prt[i * 5 + 1] = (double) tmp_record -> assign_int();
    result_prt[i * 5 + 2] = (double) tmp_record -> link_ID();
    // the count of unit time interval (5s)
    result_prt[i * 5 + 3] = (double) tmp_record -> link_start_int();
    result_prt[i * 5 + 4] = tmp_record -> flow();
  }
  for (size_t i = 0; i < _record.size(); ++i){
    delete _record[i];
  }
  _record.clear();
  return result;
}

py::array_t<double> Mcdta_Api::get_truck_dar_departure_matrix_sep(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
{
  auto start_buf = start_intervals.request();
  auto end_buf = end_intervals.request();
  if (start_buf.ndim != 1 || end_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_departure matrix_sep, input dimension mismatch");
  }
  if (start_buf.shape[0] != end_buf.shape[0]){
    throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_departure matrix_sep, input length mismatch");
  }
  int l = start_buf.shape[0];
  int *start_prt = (int *) start_buf.ptr;
  int *end_prt = (int *) end_buf.ptr;
  std::vector<dar_record*> _record = std::vector<dar_record*>();

  for (int t = 0; t < l; ++t){
    // printf("Current processing time: %d\n", t);
    if (end_prt[t] < start_prt[t]){
        throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_departure matrix_sep, end time smaller than start time");
    }
    if (end_prt[t] > get_cur_loading_interval()){
        throw std::runtime_error("Error, Mcdta_Api::get_truck_dar_departure matrix_sep, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_curb.size(); ++i){
      MNM_Dlink_Multiclass_Curb* _link = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(m_link_vec_curb[i]);
      MNM_DTA_GRADIENT_CURB::add_dar_records_curb_departure_truck(_record, _link, m_path_set_truck, TFlt(start_prt[t]), TFlt(end_prt[t]));
    }
  }

  int new_shape [2] = { (int) _record.size(), 5}; 
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  dar_record* tmp_record;
  for (size_t i = 0; i < _record.size(); ++i){
    tmp_record = _record[i];
    result_prt[i * 5 + 0] = (double) tmp_record -> path_ID();
    // the count of 1 min interval
    result_prt[i * 5 + 1] = (double) tmp_record -> assign_int();
    result_prt[i * 5 + 2] = (double) tmp_record -> link_ID();
    // the count of unit time interval (5s)
    result_prt[i * 5 + 3] = (double) tmp_record -> link_start_int();
    result_prt[i * 5 + 4] = tmp_record -> flow();
  }
  for (size_t i = 0; i < _record.size(); ++i){
    delete _record[i];
  }
  _record.clear();
  return result;
}

py::array_t<double> Mcdta_Api::get_rh_dar_arrival_matrix_sep(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
{
  auto start_buf = start_intervals.request();
  auto end_buf = end_intervals.request();
  if (start_buf.ndim != 1 || end_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get_rh_dar_arrival matrix_sep, input dimension mismatch");
  }
  if (start_buf.shape[0] != end_buf.shape[0]){
    throw std::runtime_error("Error, Mcdta_Api::get_rh_dar_arrival matrix_sep, input length mismatch");
  }
  int l = start_buf.shape[0];
  int *start_prt = (int *) start_buf.ptr;
  int *end_prt = (int *) end_buf.ptr;
  std::vector<dar_record*> _record = std::vector<dar_record*>();

  for (int t = 0; t < l; ++t){
    // printf("Current processing time: %d\n", t);
    if (end_prt[t] < start_prt[t]){
        throw std::runtime_error("Error, Mcdta_Api::get_rh_dar_arrival matrix_sep, end time smaller than start time");
    }
    if (end_prt[t] > get_cur_loading_interval()){
        throw std::runtime_error("Error, Mcdta_Api::get_rh_dar_arrival matrix_sep, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_curb.size(); ++i){
      MNM_Dlink_Multiclass_Curb* _link = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(m_link_vec_curb[i]);
      MNM_DTA_GRADIENT_CURB::add_dar_records_curb_arrival_rh(_record, _link, m_path_set_rh, TFlt(start_prt[t]), TFlt(end_prt[t]));
    }
  }

  int new_shape [2] = { (int) _record.size(), 5}; 
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  dar_record* tmp_record;
  for (size_t i = 0; i < _record.size(); ++i){
    tmp_record = _record[i];
    result_prt[i * 5 + 0] = (double) tmp_record -> path_ID();
    // the count of 1 min interval
    result_prt[i * 5 + 1] = (double) tmp_record -> assign_int();
    result_prt[i * 5 + 2] = (double) tmp_record -> link_ID();
    // the count of unit time interval (5s)
    result_prt[i * 5 + 3] = (double) tmp_record -> link_start_int();
    result_prt[i * 5 + 4] = tmp_record -> flow();
  }
  for (size_t i = 0; i < _record.size(); ++i){
    delete _record[i];
  }
  _record.clear();
  return result;
}

py::array_t<double> Mcdta_Api::get_rh_dar_departure_matrix_sep(py::array_t<int>start_intervals, py::array_t<int>end_intervals)
{
  auto start_buf = start_intervals.request();
  auto end_buf = end_intervals.request();
  if (start_buf.ndim != 1 || end_buf.ndim != 1){
    throw std::runtime_error("Error, Mcdta_Api::get_rh_dar_departure matrix_sep, input dimension mismatch");
  }
  if (start_buf.shape[0] != end_buf.shape[0]){
    throw std::runtime_error("Error, Mcdta_Api::get_rh_dar_departure matrix_sep, input length mismatch");
  }
  int l = start_buf.shape[0];
  int *start_prt = (int *) start_buf.ptr;
  int *end_prt = (int *) end_buf.ptr;
  std::vector<dar_record*> _record = std::vector<dar_record*>();

  for (int t = 0; t < l; ++t){
    // printf("Current processing time: %d\n", t);
    if (end_prt[t] < start_prt[t]){
        throw std::runtime_error("Error, Mcdta_Api::get_rh_dar_departure matrix_sep, end time smaller than start time");
    }
    if (end_prt[t] > get_cur_loading_interval()){
        throw std::runtime_error("Error, Mcdta_Api::get_rh_dar_departure matrix_sep, loaded data not enough");
    }
    for (size_t i = 0; i < m_link_vec_curb.size(); ++i){
      MNM_Dlink_Multiclass_Curb* _link = dynamic_cast<MNM_Dlink_Multiclass_Curb *>(m_link_vec_curb[i]);
      MNM_DTA_GRADIENT_CURB::add_dar_records_curb_departure_rh(_record, _link, m_path_set_rh, TFlt(start_prt[t]), TFlt(end_prt[t]));
    }
  }

  int new_shape [2] = { (int) _record.size(), 5}; 
  auto result = py::array_t<double>(new_shape);
  auto result_buf = result.request();
  double *result_prt = (double *) result_buf.ptr;
  dar_record* tmp_record;
  for (size_t i = 0; i < _record.size(); ++i){
    tmp_record = _record[i];
    result_prt[i * 5 + 0] = (double) tmp_record -> path_ID();
    // the count of 1 min interval
    result_prt[i * 5 + 1] = (double) tmp_record -> assign_int();
    result_prt[i * 5 + 2] = (double) tmp_record -> link_ID();
    // the count of unit time interval (5s)
    result_prt[i * 5 + 3] = (double) tmp_record -> link_start_int();
    result_prt[i * 5 + 4] = tmp_record -> flow();
  }
  for (size_t i = 0; i < _record.size(); ++i){
    delete _record[i];
  }
  _record.clear();
  return result;
}
// new added end 0606

// py::array_t<double> Mcdta_Api::get_car_ltg_matrix(py::array_t<int>start_intervals, int threshold_timestamp)
// {
//     // input: intervals in which the agents are released for each path, 1 min interval = 12 5-s intervals
//     // assume Mcdta_Api::build_link_cost_map() and Mcdta_Api::get_link_queue_dissipated_time() are invoked already
//     auto start_buf = start_intervals.request();
//     if (start_buf.ndim != 1){
//         throw std::runtime_error("Error, Mcdta_Api::get_car_ltg_matrix_driving, input dimension mismatch");
//     }
//     int l = start_buf.shape[0];
//     int *start_prt = (int *) start_buf.ptr;

//     // struct ltg_record {TInt path_ID; int assign_int; TInt link_ID; int link_start_int; TFlt gradient;}
//     std::vector<ltg_record*> _record = std::vector<ltg_record*>();
//     bool _flg; 
//     TFlt _fftt, _gradient;
//     int _t_arrival, _t_depart, _t_arrival_lift_up, _t_depart_lift_up, _t_depart_prime, _t_queue_dissipated_valid, _t_depart_lift_up_valid;
//     for (auto *_path : m_path_vec) { // path loop
//         // check if the path does not include any link in m_link_vec
//         _flg = false;
//         for (auto *_link : m_link_vec) {
//             if (_path -> is_link_in(_link -> m_link_ID)) {
//                 _flg = true;
//                 break;
//             }
//         }
//         if (!_flg) {
//             continue;
//         }

//         for (int t = 0; t < l; ++t){ // time loop for each start intervals of original perturbation
//             // printf("Current processing time: %d\n", t);
//             if (start_prt[t] >= get_cur_loading_interval()){
//                 throw std::runtime_error("Error, Mcdta_Api::get_car_ltg_matrix_driving, input start intervals exceeds the total loading intervals - 1");
//             } // the start int exceeds the last time int

//             // initialization for variables
//             _t_arrival = -1, _t_depart = -1, _t_arrival_lift_up = -1, _t_depart_lift_up = -1, _t_depart_prime = -1;
            
//             // trace one additional veh departing from origin of path at start_prt[t]
//             _t_depart = start_prt[t];
//             for (TInt _link_ID : _path -> m_link_vec) { // link loop for each _path from the start
//                 // arrival and departure time of original perturbation vehicle
//                 _t_arrival = _t_depart; // arrival is depart
//                 _t_depart = _t_arrival + MNM_Ults::round_up_time(m_link_tt_map[_link_ID][_t_arrival < get_cur_loading_interval() ? _t_arrival : get_cur_loading_interval() - 1]);
                
//                 // arrival time of the new perturbation vehicle
//                 // first link must be PQ model!!
//                 auto *_link = dynamic_cast<MNM_Dlink_Multiclass*>(m_mcdta -> m_link_factory -> get_link(_link_ID));
//                 if (dynamic_cast<MNM_Dlink_Pq_Multiclass*>(_link) != nullptr) {
//                     _t_arrival_lift_up = _t_arrival;  // for last pq, _t_arrival_lift_up >= get_cur_loading_interval()
//                 }
//                 else {
//                     IAssert(dynamic_cast<MNM_Dlink_Ctm_Multiclass*>(_link) != nullptr);
//                     IAssert(_t_depart_lift_up >= 0);  // from its upstream link what is this???
//                     _t_arrival_lift_up = _t_depart_lift_up;
//                 }
//                 if (_t_arrival_lift_up < _t_arrival) {
//                     printf("t_arrival_lift_up = %d and t_arrival = %d", int(_t_arrival_lift_up), int(_t_arrival));
//                     _t_arrival_lift_up = _t_arrival;
//                 }
//                 IAssert(_t_arrival_lift_up >= _t_arrival);

//                 // jiachao bug

//                 if (_link -> m_last_valid_time <= 0){
//                     printf("last valid time = %d\n", int(_link -> m_last_valid_time));
//                 }
//                 IAssert(_link -> m_last_valid_time > 0);

//                 if (_t_arrival_lift_up > int(round(_link -> m_last_valid_time - 1)) || _t_arrival_lift_up >= threshold_timestamp) {
//                     break;
//                 }

//                 // departure time of new perturbation vehicle
//                 _t_depart_prime = _t_arrival_lift_up + MNM_Ults::round_up_time(m_link_tt_map[_link_ID][_t_arrival_lift_up < get_cur_loading_interval() ? _t_arrival_lift_up : get_cur_loading_interval() - 1]);

//                 // arrival time of the NEXT new perturbation for the NEXT link
//                 _fftt = dynamic_cast<MNM_Dlink_Multiclass*>(m_mcdta -> m_link_factory -> get_link(_link_ID)) -> get_link_freeflow_tt_loading_car();
//                 // _fftt = dynamic_cast<MNM_Dlink_Ctm_Multiclass*>(m_mcdta -> m_link_factory -> get_link(_link_ID)) -> get_link_freeflow_tt_car() / m_mcdta -> m_unit_time;
//                 _t_depart_lift_up = m_queue_dissipated_time_car[_link_ID][_t_arrival_lift_up] + MNM_Ults::round_up_time(_fftt);
                
//                 // why adding this set of assertions??
//                 if (!m_link_congested_car[_link_ID][_t_arrival_lift_up]) {
//                     if (m_queue_dissipated_time_car[_link_ID][_t_arrival_lift_up] == _t_arrival_lift_up) {
//                         IAssert(_t_arrival_lift_up + MNM_Ults::round_up_time(_fftt) == _t_depart_lift_up);
//                     }
//                     else {
//                         // critical state where subgradient applies
//                         IAssert(m_queue_dissipated_time_car[_link_ID][_t_arrival_lift_up] > _t_arrival_lift_up);
//                         IAssert(_t_arrival_lift_up + MNM_Ults::round_up_time(_fftt) < _t_depart_lift_up);
//                     }
//                 }
//                 else {
//                     IAssert(m_queue_dissipated_time_car[_link_ID][_t_arrival_lift_up] > _t_arrival_lift_up);
//                     IAssert(_t_arrival_lift_up + MNM_Ults::round_up_time(_fftt) < _t_depart_lift_up);
//                 }

//                 if (_t_depart_lift_up < _t_depart_prime) {
//                     printf("Error, Mcdta_Api::get_car_ltg_matrix_driving, something is wrong");
//                     exit(-1);
//                     // _t_depart_lift_up can be equal to _t_depart_prime, when the arrival curve is horizontal
//                 }

//                 if (_t_depart_prime < get_cur_loading_interval() - 1 &&
//                     std::find_if(m_link_vec.begin(), m_link_vec.end(), 
//                         [&_link_ID](const MNM_Dlink_Multiclass *_l){return _l -> m_link_ID == _link_ID;}) != m_link_vec.end()) {
//                     if (m_link_congested_car[_link_ID][_t_arrival_lift_up] && _t_depart_lift_up > _t_depart_prime) {
//                         IAssert(_link -> m_last_valid_time > 0);
//                         if (m_queue_dissipated_time_car[_link_ID][_t_arrival_lift_up] <= int(round(_link -> m_last_valid_time - 1))) {
//                             _t_queue_dissipated_valid = m_queue_dissipated_time_car[_link_ID][_t_arrival_lift_up];
//                             _t_depart_lift_up_valid = _t_depart_lift_up;
//                         }
//                         else {
//                             _t_queue_dissipated_valid = int(round(_link -> m_last_valid_time));
//                             _t_depart_lift_up_valid = _t_queue_dissipated_valid - 1 + MNM_Ults::round_up_time(m_link_tt_map[_link_ID][_t_queue_dissipated_valid - 1 < get_cur_loading_interval() ? _t_queue_dissipated_valid - 1 : get_cur_loading_interval() - 1]);
//                         }

//                         IAssert(_t_depart_lift_up_valid <= get_cur_loading_interval() - 1);
//                         IAssert(_t_arrival_lift_up < _t_queue_dissipated_valid);
//                         if (_t_depart_prime > _t_depart_lift_up_valid) {
//                             std::cout << "\nError, Mcdta_Api::get_car_ltg_matrix" << "\n";
//                             std::cout << "interval: " << start_prt[t] << ", link: " << _link_ID << "\n";
//                             std::cout << "car in" << "\n";
//                             std::cout << _link -> m_N_in_car -> to_string() << "\n";
//                             std::cout << "car out" << "\n";
//                             std::cout << _link -> m_N_out_car -> to_string() << "\n\n";
//                             std::cout << "last valid time: " << _link -> m_last_valid_time << "\n";
//                             std::cout << "_t_arrival: " << _t_arrival << "\n";
//                             std::cout << "_t_depart: " << _t_depart << "\n";
//                             std::cout << "_t_arrival_lift_up: " << _t_arrival_lift_up << "\n";
//                             std::cout << "_t_depart_prime: " << _t_depart_prime << "\n";
//                             std::cout << "m_queue_dissipated_time_car[_link_ID][_t_arrival_lift_up]: " << m_queue_dissipated_time_car[_link_ID][_t_arrival_lift_up] << "\n";
//                             std::cout << "_t_queue_dissipated_valid: " << _t_queue_dissipated_valid << "\n";
//                             std::cout << "_t_depart_lift_up: " << _t_depart_lift_up << "\n";
//                             std::cout << "_t_depart_lift_up_valid: " << _t_depart_lift_up_valid << "\n";
//                             std::cout << "_fftt: " << _fftt << "\n";
//                             std::cout << "m_link_tt_map[_link_ID][_t_queue_dissipated_valid]: " << m_link_tt_map[_link_ID][_t_queue_dissipated_valid - 1 < get_cur_loading_interval() ? _t_queue_dissipated_valid - 1 : get_cur_loading_interval() - 1] << "\n";
//                             std::cout << "get_cur_loading_interval(): " << get_cur_loading_interval() << "\n";
//                             exit(-1);
//                         }
//                         if (_t_depart_prime < _t_depart_lift_up_valid) {

//                             _gradient = MNM_DTA_GRADIENT::get_departure_cc_slope_car(_link, TFlt(_t_depart_prime), TFlt(_t_depart_lift_up_valid + 1));
//                             if (_gradient > DBL_EPSILON) {
//                                 _gradient = m_mcdta -> m_unit_time / _gradient;  // seconds
//                                 for (int t_prime = _t_arrival_lift_up; t_prime < _t_queue_dissipated_valid; ++t_prime) {
//                                     MNM_DTA_GRADIENT::add_ltg_records_veh(_record, _link, _path, start_prt[t], t_prime, _gradient);
//                                 }
//                             }
//                         }  
//                     }
//                 }
//             } // end of link loop in _path
//         } // end of time loop
//     } // end of path loop

//     // _record.size() = num_timesteps x num_links x num_path x num_assign_timesteps
//     // path_ID, assign_time, link_ID, start_int, gradient
//     int new_shape [2] = { (int) _record.size(), 5};
//     auto result = py::array_t<double>(new_shape);
//     auto result_buf = result.request();
//     double *result_prt = (double *) result_buf.ptr;
//     ltg_record* tmp_record;
//     for (size_t i = 0; i < _record.size(); ++i){
//         tmp_record = _record[i];
//         result_prt[i * 5 + 0] = (double) tmp_record -> path_ID();
//         // the count of 1 min interval
//         result_prt[i * 5 + 1] = (double) tmp_record -> assign_int;
//         result_prt[i * 5 + 2] = (double) tmp_record -> link_ID();
//         // the count of unit time interval (5s)
//         result_prt[i * 5 + 3] = (double) tmp_record -> link_start_int;
//         result_prt[i * 5 + 4] = tmp_record -> gradient();
//         // printf("path ID: %f, departure assign interval (5 s): %f, link ID: %f, time interval (5 s): %f, gradient: %f\n",
//         //         result_prt[i * 5 + 0], result_prt[i * 5 + 1], result_prt[i * 5 + 2], result_prt[i * 5 + 3], result_prt[i * 5 + 4]);
//     }
//     for (size_t i = 0; i < _record.size(); ++i){
//         delete _record[i];
//     }
//     _record.clear();
//     return result;
// }

// py::array_t<double> Mcdta_Api::get_truck_ltg_matrix(py::array_t<int>start_intervals, int threshold_timestamp)
// {
//     // input: intervals in which the agents are released for each path, 1 min interval = 12 5-s intervals
//     // assume Mcdta_Api::build_link_cost_map() and Mcdta_Api::get_link_queue_dissipated_time() are invoked already
//     auto start_buf = start_intervals.request();
//     if (start_buf.ndim != 1){
//         throw std::runtime_error("Error, Mcdta_Api::get_truck_ltg_matrix, input dimension mismatch");
//     }
//     int l = start_buf.shape[0];
//     int *start_prt = (int *) start_buf.ptr;

//     std::vector<ltg_record*> _record = std::vector<ltg_record*>();
//     bool _flg; 
//     TFlt _fftt, _gradient;
//     int _t_arrival, _t_depart, _t_arrival_lift_up, _t_depart_lift_up, _t_depart_prime, _t_queue_dissipated_valid, _t_depart_lift_up_valid;
//     for (auto *_path : m_path_vec_truck) {
//         // check if the path does not include any link in m_link_vec
//         _flg = false;
//         for (auto *_link : m_link_vec) {
//             if (_path -> is_link_in(_link -> m_link_ID)) {
//                 _flg = true;
//                 break;
//             }
//         }
//         if (!_flg) {
//             continue;
//         }

//         for (int t = 0; t < l; ++t){
//             // printf("Current processing time: %d\n", t);
//             if (start_prt[t] >= get_cur_loading_interval()){
//                 throw std::runtime_error("Error, Mcdta_Api::get_truck_ltg_matrix, input start intervals exceeds the total loading intervals - 1");
//             }

//             _t_arrival = -1, _t_depart = -1, _t_arrival_lift_up = -1, _t_depart_lift_up = -1, _t_depart_prime = -1;
//             // trace one additional veh departing from origin of path at start_prt[t]
//             _t_depart = start_prt[t];
//             for (TInt _link_ID : _path -> m_link_vec) {
//                 // arrival and departure time of original perturbation vehicle
//                 _t_arrival = _t_depart;
//                 _t_depart = _t_arrival + MNM_Ults::round_up_time(m_link_tt_map_truck[_link_ID][_t_arrival < get_cur_loading_interval() ? _t_arrival : get_cur_loading_interval() - 1]);
                
//                 // arrival time of the new perturbation vehicle
//                 auto *_link = dynamic_cast<MNM_Dlink_Multiclass*>(m_mcdta -> m_link_factory -> get_link(_link_ID));
//                 if (dynamic_cast<MNM_Dlink_Pq_Multiclass*>(_link) != nullptr) {
//                     _t_arrival_lift_up = _t_arrival;  // for last pq, _t_arrival_lift_up >= get_cur_loading_interval()
//                 }
//                 else {
//                     IAssert(dynamic_cast<MNM_Dlink_Ctm_Multiclass*>(_link) != nullptr);
//                     IAssert(_t_depart_lift_up >= 0);  // from its upstream link
//                     _t_arrival_lift_up = _t_depart_lift_up;
//                 }
//                 IAssert(_t_arrival_lift_up >= _t_arrival);

//                 IAssert(_link -> m_last_valid_time > 0);
//                 if (_t_arrival_lift_up > int(round(_link -> m_last_valid_time - 1)) || _t_arrival_lift_up >= threshold_timestamp) {
//                     break;
//                 }

//                 // departure time of new perturbation vehicle
//                 _t_depart_prime = _t_arrival_lift_up + MNM_Ults::round_up_time(m_link_tt_map_truck[_link_ID][_t_arrival_lift_up < get_cur_loading_interval() ? _t_arrival_lift_up : get_cur_loading_interval() - 1]);

//                 // arrival time of the NEXT new perturbation for the NEXT link
//                 _fftt = dynamic_cast<MNM_Dlink_Multiclass*>(m_mcdta -> m_link_factory -> get_link(_link_ID)) -> get_link_freeflow_tt_loading_truck();
//                 // _fftt = dynamic_cast<MNM_Dlink_Ctm_Multiclass*>(m_mcdta -> m_link_factory -> get_link(_link_ID)) -> get_link_freeflow_tt_truck() / m_mcdta -> m_unit_time;
//                 _t_depart_lift_up = m_queue_dissipated_time_truck[_link_ID][_t_arrival_lift_up] + MNM_Ults::round_up_time(_fftt);
                    
//                 if (!m_link_congested_truck[_link_ID][_t_arrival_lift_up]) {
//                     if (m_queue_dissipated_time_truck[_link_ID][_t_arrival_lift_up] == _t_arrival_lift_up) {
//                         IAssert(_t_arrival_lift_up + MNM_Ults::round_up_time(_fftt) == _t_depart_lift_up);
//                     }
//                     else {
//                         // critical state where subgradient applies
//                         IAssert(m_queue_dissipated_time_truck[_link_ID][_t_arrival_lift_up] > _t_arrival_lift_up);
//                         IAssert(_t_arrival_lift_up + MNM_Ults::round_up_time(_fftt) < _t_depart_lift_up);
//                     }
//                 }
//                 else {
//                     IAssert(m_queue_dissipated_time_truck[_link_ID][_t_arrival_lift_up] > _t_arrival_lift_up);
//                     IAssert(_t_arrival_lift_up + MNM_Ults::round_up_time(_fftt) < _t_depart_lift_up);
//                 }

//                 if (_t_depart_lift_up < _t_depart_prime) {
//                     printf("Error, Mcdta_Api::get_truck_ltg_matrix, something is wrong");
//                     exit(-1);
//                     // _t_depart_lift_up can be equal to _t_depart_prime, when the arrival curve is horizontal
//                 }

//                 if (_t_depart_prime < get_cur_loading_interval() - 1 &&
//                     std::find_if(m_link_vec.begin(), m_link_vec.end(), 
//                                  [&_link_ID](const MNM_Dlink_Multiclass *_l){return _l -> m_link_ID == _link_ID;}) != m_link_vec.end()) {
//                     if (m_link_congested_truck[_link_ID][_t_arrival_lift_up] && _t_depart_lift_up > _t_depart_prime) {
//                         IAssert(_link -> m_last_valid_time > 0);
//                         if (m_queue_dissipated_time_truck[_link_ID][_t_arrival_lift_up] <= int(round(_link -> m_last_valid_time - 1))) {
//                             _t_queue_dissipated_valid = m_queue_dissipated_time_truck[_link_ID][_t_arrival_lift_up];
//                             _t_depart_lift_up_valid = _t_depart_lift_up;
//                         }
//                         else {
//                             _t_queue_dissipated_valid = int(round(_link -> m_last_valid_time));
//                             _t_depart_lift_up_valid = _t_queue_dissipated_valid - 1 + MNM_Ults::round_up_time(m_link_tt_map_truck[_link_ID][_t_queue_dissipated_valid - 1 < get_cur_loading_interval() ? _t_queue_dissipated_valid - 1 : get_cur_loading_interval() - 1]);
//                         }
//                         // jiachao added
//                         if (_t_depart_lift_up_valid > get_cur_loading_interval() - 1) {
//                             _t_depart_lift_up_valid = get_cur_loading_interval() - 1;
//                         }
//                         IAssert(_t_depart_lift_up_valid <= get_cur_loading_interval() - 1);
//                         IAssert(_t_arrival_lift_up < _t_queue_dissipated_valid);
//                         if (_t_depart_prime > _t_depart_lift_up_valid) {
//                             std::cout << "\nError, Mcdta_Api::get_truck_ltg_matrix" << "\n";
//                             std::cout << "interval: " << start_prt[t] << ", link: " << _link_ID << "\n";
//                             std::cout << "truck in" << "\n";
//                             std::cout << _link -> m_N_in_truck -> to_string() << "\n";
//                             std::cout << "truck out" << "\n";
//                             std::cout << _link -> m_N_out_truck -> to_string() << "\n\n";
//                             std::cout << "last valid time: " << _link -> m_last_valid_time << "\n";
//                             std::cout << "_t_arrival: " << _t_arrival << "\n";
//                             std::cout << "_t_depart: " << _t_depart << "\n";
//                             std::cout << "_t_arrival_lift_up: " << _t_arrival_lift_up << "\n";
//                             std::cout << "_t_depart_prime: " << _t_depart_prime << "\n";
//                             std::cout << "m_queue_dissipated_time_truck[_link_ID][_t_arrival_lift_up]: " << m_queue_dissipated_time_truck[_link_ID][_t_arrival_lift_up] << "\n";
//                             std::cout << "_t_queue_dissipated_valid: " << _t_queue_dissipated_valid << "\n";
//                             std::cout << "_t_depart_lift_up: " << _t_depart_lift_up << "\n";
//                             std::cout << "_t_depart_lift_up_valid: " << _t_depart_lift_up_valid << "\n";
//                             std::cout << "_fftt: " << _fftt << "\n";
//                             std::cout << "m_link_tt_map_truck[_link_ID][_t_queue_dissipated_valid]: " << m_link_tt_map_truck[_link_ID][_t_queue_dissipated_valid - 1 < get_cur_loading_interval() ? _t_queue_dissipated_valid - 1 : get_cur_loading_interval() - 1] << "\n";
//                             std::cout << "get_cur_loading_interval(): " << get_cur_loading_interval() << "\n";
//                             exit(-1);
//                         }
//                         if (_t_depart_prime < _t_depart_lift_up_valid) {
//                             _gradient = MNM_DTA_GRADIENT::get_departure_cc_slope_truck(_link, TFlt(_t_depart_prime), TFlt(_t_depart_lift_up_valid + 1));
//                             if (_gradient > DBL_EPSILON) {
//                                 _gradient = m_mcdta -> m_unit_time / _gradient;  // seconds
//                                 for (int t_prime = _t_arrival_lift_up; t_prime < _t_queue_dissipated_valid; ++t_prime) {
//                                     MNM_DTA_GRADIENT::add_ltg_records_veh(_record, _link, _path, start_prt[t], t_prime, _gradient);
//                                 }
//                             }
//                         }
//                     }
//                 }
//             }
//         }
//     }

//     // _record.size() = num_timesteps x num_links x num_path x num_assign_timesteps
//     // path_ID, assign_time, link_ID, start_int, gradient
//     int new_shape [2] = { (int) _record.size(), 5};
//     auto result = py::array_t<double>(new_shape);
//     auto result_buf = result.request();
//     double *result_prt = (double *) result_buf.ptr;
//     ltg_record* tmp_record;
//     for (size_t i = 0; i < _record.size(); ++i){
//         tmp_record = _record[i];
//         result_prt[i * 5 + 0] = (double) tmp_record -> path_ID();
//         // the count of 1 min interval
//         result_prt[i * 5 + 1] = (double) tmp_record -> assign_int;
//         result_prt[i * 5 + 2] = (double) tmp_record -> link_ID();
//         // the count of unit time interval (5s)
//         result_prt[i * 5 + 3] = (double) tmp_record -> link_start_int;
//         result_prt[i * 5 + 4] = tmp_record -> gradient();
//         // printf("path ID: %f, departure assign interval (5 s): %f, link ID: %f, time interval (5 s): %f, gradient: %f\n",
//         //         result_prt[i * 5 + 0], result_prt[i * 5 + 1], result_prt[i * 5 + 2], result_prt[i * 5 + 3], result_prt[i * 5 + 4]);
//     }
//     for (size_t i = 0; i < _record.size(); ++i){
//         delete _record[i];
//     }
//     _record.clear();
//     return result;
// }

// py::array_t<double> Mcdta_Api::get_rh_ltg_matrix(py::array_t<int>start_intervals, int threshold_timestamp)
// {
//     // input: intervals in which the agents are released for each path, 1 min interval = 12 5-s intervals
//     // assume Mcdta_Api::build_link_cost_map() and Mcdta_Api::get_link_queue_dissipated_time() are invoked already
//     auto start_buf = start_intervals.request();
//     if (start_buf.ndim != 1){
//         throw std::runtime_error("Error, Mcdta_Api::get_rh_ltg_matrix, input dimension mismatch");
//     }
//     int l = start_buf.shape[0];
//     int *start_prt = (int *) start_buf.ptr;

//     std::vector<ltg_record*> _record = std::vector<ltg_record*>();
//     bool _flg; 
//     TFlt _fftt, _gradient;
//     int _t_arrival, _t_depart, _t_arrival_lift_up, _t_depart_lift_up, _t_depart_prime, _t_queue_dissipated_valid, _t_depart_lift_up_valid;
//     for (auto *_path : m_path_vec_rh) {
//         // check if the path does not include any link in m_link_vec
//         _flg = false;
//         for (auto *_link : m_link_vec) {
//             if (_path -> is_link_in(_link -> m_link_ID)) {
//                 _flg = true;
//                 break;
//             }
//         }
//         if (!_flg) {
//             continue;
//         }

//         for (int t = 0; t < l; ++t){
//             // printf("Current processing time: %d\n", t);
//             if (start_prt[t] >= get_cur_loading_interval()){
//                 throw std::runtime_error("Error, Mcdta_Api::get_truck_ltg_matrix, input start intervals exceeds the total loading intervals - 1");
//             }

//             _t_arrival = -1, _t_depart = -1, _t_arrival_lift_up = -1, _t_depart_lift_up = -1, _t_depart_prime = -1;
//             // trace one additional veh departing from origin of path at start_prt[t]
//             _t_depart = start_prt[t];
//             for (TInt _link_ID : _path -> m_link_vec) {
//                 // arrival and departure time of original perturbation vehicle
//                 _t_arrival = _t_depart;
//                 _t_depart = _t_arrival + MNM_Ults::round_up_time(m_link_tt_map[_link_ID][_t_arrival < get_cur_loading_interval() ? _t_arrival : get_cur_loading_interval() - 1]);
                
//                 // arrival time of the new perturbation vehicle
//                 auto *_link = dynamic_cast<MNM_Dlink_Multiclass*>(m_mcdta -> m_link_factory -> get_link(_link_ID));
//                 if (dynamic_cast<MNM_Dlink_Pq_Multiclass*>(_link) != nullptr) {
//                     _t_arrival_lift_up = _t_arrival;  // for last pq, _t_arrival_lift_up >= get_cur_loading_interval()
//                 }
//                 else {
//                     IAssert(dynamic_cast<MNM_Dlink_Ctm_Multiclass*>(_link) != nullptr);
//                     IAssert(_t_depart_lift_up >= 0);  // from its upstream link
//                     _t_arrival_lift_up = _t_depart_lift_up;
//                 }
//                 IAssert(_t_arrival_lift_up >= _t_arrival);

//                 IAssert(_link -> m_last_valid_time > 0);
//                 if (_t_arrival_lift_up > int(round(_link -> m_last_valid_time - 1)) || _t_arrival_lift_up >= threshold_timestamp) {
//                     break;
//                 }

//                 // departure time of new perturbation vehicle
//                 _t_depart_prime = _t_arrival_lift_up + MNM_Ults::round_up_time(m_link_tt_map[_link_ID][_t_arrival_lift_up < get_cur_loading_interval() ? _t_arrival_lift_up : get_cur_loading_interval() - 1]);

//                 // arrival time of the NEXT new perturbation for the NEXT link
//                 _fftt = dynamic_cast<MNM_Dlink_Multiclass*>(m_mcdta -> m_link_factory -> get_link(_link_ID)) -> get_link_freeflow_tt_loading_truck();
//                 // _fftt = dynamic_cast<MNM_Dlink_Ctm_Multiclass*>(m_mcdta -> m_link_factory -> get_link(_link_ID)) -> get_link_freeflow_tt_truck() / m_mcdta -> m_unit_time;
//                 _t_depart_lift_up = m_queue_dissipated_time_car[_link_ID][_t_arrival_lift_up] + MNM_Ults::round_up_time(_fftt);
                    
//                 if (!m_link_congested_car[_link_ID][_t_arrival_lift_up]) {
//                     if (m_queue_dissipated_time_car[_link_ID][_t_arrival_lift_up] == _t_arrival_lift_up) {
//                         IAssert(_t_arrival_lift_up + MNM_Ults::round_up_time(_fftt) == _t_depart_lift_up);
//                     }
//                     else {
//                         // critical state where subgradient applies
//                         IAssert(m_queue_dissipated_time_car[_link_ID][_t_arrival_lift_up] > _t_arrival_lift_up);
//                         IAssert(_t_arrival_lift_up + MNM_Ults::round_up_time(_fftt) < _t_depart_lift_up);
//                     }
//                 }
//                 else {
//                     IAssert(m_queue_dissipated_time_car[_link_ID][_t_arrival_lift_up] > _t_arrival_lift_up);
//                     IAssert(_t_arrival_lift_up + MNM_Ults::round_up_time(_fftt) < _t_depart_lift_up);
//                 }

//                 if (_t_depart_lift_up < _t_depart_prime) {
//                     printf("Error, Mcdta_Api::get_truck_ltg_matrix, something is wrong");
//                     exit(-1);
//                     // _t_depart_lift_up can be equal to _t_depart_prime, when the arrival curve is horizontal
//                 }

//                 if (_t_depart_prime < get_cur_loading_interval() - 1 &&
//                     std::find_if(m_link_vec.begin(), m_link_vec.end(), 
//                                  [&_link_ID](const MNM_Dlink_Multiclass *_l){return _l -> m_link_ID == _link_ID;}) != m_link_vec.end()) {
//                     if (m_link_congested_car[_link_ID][_t_arrival_lift_up] && _t_depart_lift_up > _t_depart_prime) {
//                         IAssert(_link -> m_last_valid_time > 0);
//                         if (m_queue_dissipated_time_car[_link_ID][_t_arrival_lift_up] <= int(round(_link -> m_last_valid_time - 1))) {
//                             _t_queue_dissipated_valid = m_queue_dissipated_time_car[_link_ID][_t_arrival_lift_up];
//                             _t_depart_lift_up_valid = _t_depart_lift_up;
//                         }
//                         else {
//                             _t_queue_dissipated_valid = int(round(_link -> m_last_valid_time));
//                             _t_depart_lift_up_valid = _t_queue_dissipated_valid - 1 + MNM_Ults::round_up_time(m_link_tt_map[_link_ID][_t_queue_dissipated_valid - 1 < get_cur_loading_interval() ? _t_queue_dissipated_valid - 1 : get_cur_loading_interval() - 1]);
//                         }

//                         // bug !!!
//                         if (_t_depart_lift_up_valid > get_cur_loading_interval() - 1){
//                             // std::cout << "\nError, Mcdta_Api::get_rh_ltg_matrix: _t_depart_lift_up_valid > get_cur_loading_interval() - 1" << "\n";
//                             // std::cout << "interval: " << start_prt[t] << ", link: " << _link_ID << "\n";
//                             // std::cout << "_t_depart_lift_up_valid: " << _t_depart_lift_up_valid << "\n"; 
//                             _t_depart_lift_up_valid = get_cur_loading_interval() - 1;
//                         }
//                         IAssert(_t_depart_lift_up_valid <= get_cur_loading_interval() - 1); // bug here
//                         IAssert(_t_arrival_lift_up < _t_queue_dissipated_valid);
//                         if (_t_depart_prime > _t_depart_lift_up_valid) {
//                             std::cout << "\nError, Mcdta_Api::get_rh_ltg_matrix" << "\n";
//                             std::cout << "interval: " << start_prt[t] << ", link: " << _link_ID << "\n";
//                             std::cout << "truck in" << "\n";
//                             std::cout << _link -> m_N_in_car -> to_string() << "\n";
//                             std::cout << "truck out" << "\n";
//                             std::cout << _link -> m_N_out_car -> to_string() << "\n\n";
//                             std::cout << "last valid time: " << _link -> m_last_valid_time << "\n";
//                             std::cout << "_t_arrival: " << _t_arrival << "\n";
//                             std::cout << "_t_depart: " << _t_depart << "\n";
//                             std::cout << "_t_arrival_lift_up: " << _t_arrival_lift_up << "\n";
//                             std::cout << "_t_depart_prime: " << _t_depart_prime << "\n";
//                             std::cout << "m_queue_dissipated_time_car[_link_ID][_t_arrival_lift_up]: " << m_queue_dissipated_time_car[_link_ID][_t_arrival_lift_up] << "\n";
//                             std::cout << "_t_queue_dissipated_valid: " << _t_queue_dissipated_valid << "\n";
//                             std::cout << "_t_depart_lift_up: " << _t_depart_lift_up << "\n";
//                             std::cout << "_t_depart_lift_up_valid: " << _t_depart_lift_up_valid << "\n";
//                             std::cout << "_fftt: " << _fftt << "\n";
//                             std::cout << "m_link_tt_map[_link_ID][_t_queue_dissipated_valid]: " << m_link_tt_map[_link_ID][_t_queue_dissipated_valid - 1 < get_cur_loading_interval() ? _t_queue_dissipated_valid - 1 : get_cur_loading_interval() - 1] << "\n";
//                             std::cout << "get_cur_loading_interval(): " << get_cur_loading_interval() << "\n";
//                             exit(-1);
//                         }
//                         if (_t_depart_prime < _t_depart_lift_up_valid) {
//                             _gradient = MNM_DTA_GRADIENT::get_departure_cc_slope_car(_link, TFlt(_t_depart_prime), TFlt(_t_depart_lift_up_valid + 1));
//                             if (_gradient > DBL_EPSILON) {
//                                 _gradient = m_mcdta -> m_unit_time / _gradient;  // seconds
//                                 for (int t_prime = _t_arrival_lift_up; t_prime < _t_queue_dissipated_valid; ++t_prime) {
//                                     MNM_DTA_GRADIENT::add_ltg_records_veh(_record, _link, _path, start_prt[t], t_prime, _gradient);
//                                 }
//                             }
//                         }
//                     }
//                 }
//             }
//         }
//     }

//     // _record.size() = num_timesteps x num_links x num_path x num_assign_timesteps
//     // path_ID, assign_time, link_ID, start_int, gradient
//     int new_shape [2] = { (int) _record.size(), 5};
//     auto result = py::array_t<double>(new_shape);
//     auto result_buf = result.request();
//     double *result_prt = (double *) result_buf.ptr;
//     ltg_record* tmp_record;
//     for (size_t i = 0; i < _record.size(); ++i){
//         tmp_record = _record[i];
//         result_prt[i * 5 + 0] = (double) tmp_record -> path_ID();
//         // the count of 1 min interval
//         result_prt[i * 5 + 1] = (double) tmp_record -> assign_int;
//         result_prt[i * 5 + 2] = (double) tmp_record -> link_ID();
//         // the count of unit time interval (5s)
//         result_prt[i * 5 + 3] = (double) tmp_record -> link_start_int;
//         result_prt[i * 5 + 4] = tmp_record -> gradient();
//         // printf("path ID: %f, departure assign interval (5 s): %f, link ID: %f, time interval (5 s): %f, gradient: %f\n",
//         //         result_prt[i * 5 + 0], result_prt[i * 5 + 1], result_prt[i * 5 + 2], result_prt[i * 5 + 3], result_prt[i * 5 + 4]);
//     }
//     for (size_t i = 0; i < _record.size(); ++i){
//         delete _record[i];
//     }
//     _record.clear();
//     return result;
// }

py::array_t<double> Mcdta_Api::get_car_ltg_matrix_sep(py::array_t<int>start_intervals, int threshold_timestamp)
{
    // input: intervals in which the agents are released for each path, 1 min interval = 12 5-s intervals
    // assume Mcdta_Api::build_link_cost_map() and Mcdta_Api::get_link_queue_dissipated_time() are invoked already
    auto start_buf = start_intervals.request();
    if (start_buf.ndim != 1){
        throw std::runtime_error("Error, Mcdta_Api::get_car_ltg_matrix_driving, input dimension mismatch");
    }
    int l = start_buf.shape[0];
    int *start_prt = (int *) start_buf.ptr;

    // struct ltg_record {TInt path_ID; int assign_int; TInt link_ID; int link_start_int; TFlt gradient;}
    std::vector<ltg_record*> _record = std::vector<ltg_record*>();
    bool _flg; 
    TFlt _fftt, _gradient;
    int _t_arrival, _t_depart, _t_arrival_lift_up, _t_depart_lift_up, _t_depart_prime, _t_queue_dissipated_valid, _t_depart_lift_up_valid;
    for (auto *_path : m_path_vec) { // path loop
        // check if the path does not include any link in m_link_vec
        _flg = false;
        for (auto *_link : m_link_vec) {
            if (_path -> is_link_in(_link -> m_link_ID)) {
                _flg = true;
                break;
            }
        }
        if (!_flg) {
            continue;
        }

        for (int t = 0; t < l; ++t){ // time loop for each start intervals of original perturbation
            // printf("Current processing time: %d\n", t);
            if (start_prt[t] >= get_cur_loading_interval()){
                throw std::runtime_error("Error, Mcdta_Api::get_car_ltg_matrix_driving, input start intervals exceeds the total loading intervals - 1");
            } // the start int exceeds the last time int

            // initialization for variables
            _t_arrival = -1, _t_depart = -1, _t_arrival_lift_up = -1, _t_depart_lift_up = -1, _t_depart_prime = -1;
            
            // trace one additional veh departing from origin of path at start_prt[t]
            _t_depart = start_prt[t];
            for (TInt _link_ID : _path -> m_link_vec) { // link loop for each _path from the start
                // arrival and departure time of original perturbation vehicle
                _t_arrival = _t_depart; // arrival is depart
                _t_depart = _t_arrival + MNM_Ults::round_up_time(m_link_tt_map[_link_ID][_t_arrival < get_cur_loading_interval() ? _t_arrival : get_cur_loading_interval() - 1]);
                
                // arrival time of the new perturbation vehicle
                // first link must be PQ model!!
                auto *_link = dynamic_cast<MNM_Dlink_Multiclass*>(m_mcdta -> m_link_factory -> get_link(_link_ID));
                if (dynamic_cast<MNM_Dlink_Pq_Multiclass*>(_link) != nullptr) {
                    _t_arrival_lift_up = _t_arrival;  // for last pq, _t_arrival_lift_up >= get_cur_loading_interval()
                }
                else {
                    IAssert(dynamic_cast<MNM_Dlink_Ctm_Multiclass*>(_link) != nullptr);
                    IAssert(_t_depart_lift_up >= 0);  // from its upstream link what is this???
                    _t_arrival_lift_up = _t_depart_lift_up;
                }
                if (_t_arrival_lift_up < _t_arrival) {
                    printf("t_arrival_lift_up = %d and t_arrival = %d", int(_t_arrival_lift_up), int(_t_arrival));
                    _t_arrival_lift_up = _t_arrival;
                }
                IAssert(_t_arrival_lift_up >= _t_arrival);

                // jiachao bug

                if (_link -> m_last_valid_time <= 0){
                    printf("last valid time = %d\n", int(_link -> m_last_valid_time));
                }
                IAssert(_link -> m_last_valid_time > 0);

                if (_t_arrival_lift_up > int(round(_link -> m_last_valid_time - 1)) || _t_arrival_lift_up >= threshold_timestamp) {
                    break;
                }

                // departure time of new perturbation vehicle
                _t_depart_prime = _t_arrival_lift_up + MNM_Ults::round_up_time(m_link_tt_map[_link_ID][_t_arrival_lift_up < get_cur_loading_interval() ? _t_arrival_lift_up : get_cur_loading_interval() - 1]);

                // arrival time of the NEXT new perturbation for the NEXT link
                _fftt = dynamic_cast<MNM_Dlink_Multiclass*>(m_mcdta -> m_link_factory -> get_link(_link_ID)) -> get_link_freeflow_tt_loading_car();
                // _fftt = dynamic_cast<MNM_Dlink_Ctm_Multiclass*>(m_mcdta -> m_link_factory -> get_link(_link_ID)) -> get_link_freeflow_tt_car() / m_mcdta -> m_unit_time;
                _t_depart_lift_up = m_queue_dissipated_time_car[_link_ID][_t_arrival_lift_up] + MNM_Ults::round_up_time(_fftt);
                
                // why adding this set of assertions??
                if (!m_link_congested_car[_link_ID][_t_arrival_lift_up]) {
                    if (m_queue_dissipated_time_car[_link_ID][_t_arrival_lift_up] == _t_arrival_lift_up) {
                        IAssert(_t_arrival_lift_up + MNM_Ults::round_up_time(_fftt) == _t_depart_lift_up);
                    }
                    else {
                        // critical state where subgradient applies
                        IAssert(m_queue_dissipated_time_car[_link_ID][_t_arrival_lift_up] > _t_arrival_lift_up);
                        IAssert(_t_arrival_lift_up + MNM_Ults::round_up_time(_fftt) < _t_depart_lift_up);
                    }
                }
                else {
                    IAssert(m_queue_dissipated_time_car[_link_ID][_t_arrival_lift_up] > _t_arrival_lift_up);
                    IAssert(_t_arrival_lift_up + MNM_Ults::round_up_time(_fftt) < _t_depart_lift_up);
                }

                if (_t_depart_lift_up < _t_depart_prime) {
                    printf("Error, Mcdta_Api::get_car_ltg_matrix_driving, something is wrong");
                    exit(-1);
                    // _t_depart_lift_up can be equal to _t_depart_prime, when the arrival curve is horizontal
                }

                if (_t_depart_prime < get_cur_loading_interval() - 1 &&
                    std::find_if(m_link_vec.begin(), m_link_vec.end(), 
                        [&_link_ID](const MNM_Dlink_Multiclass *_l){return _l -> m_link_ID == _link_ID;}) != m_link_vec.end()) {
                    if (m_link_congested_car[_link_ID][_t_arrival_lift_up] && _t_depart_lift_up > _t_depart_prime) {
                        IAssert(_link -> m_last_valid_time > 0);
                        if (m_queue_dissipated_time_car[_link_ID][_t_arrival_lift_up] <= int(round(_link -> m_last_valid_time - 1))) {
                            _t_queue_dissipated_valid = m_queue_dissipated_time_car[_link_ID][_t_arrival_lift_up];
                            _t_depart_lift_up_valid = _t_depart_lift_up;
                        }
                        else {
                            _t_queue_dissipated_valid = int(round(_link -> m_last_valid_time));
                            _t_depart_lift_up_valid = _t_queue_dissipated_valid - 1 + MNM_Ults::round_up_time(m_link_tt_map[_link_ID][_t_queue_dissipated_valid - 1 < get_cur_loading_interval() ? _t_queue_dissipated_valid - 1 : get_cur_loading_interval() - 1]);
                        }

                        IAssert(_t_depart_lift_up_valid <= get_cur_loading_interval() - 1);
                        IAssert(_t_arrival_lift_up < _t_queue_dissipated_valid);
                        if (_t_depart_prime > _t_depart_lift_up_valid) {
                            std::cout << "\nError, Mcdta_Api::get_car_ltg_matrix" << "\n";
                            std::cout << "interval: " << start_prt[t] << ", link: " << _link_ID << "\n";
                            std::cout << "car in" << "\n";
                            std::cout << _link -> m_N_in_car -> to_string() << "\n";
                            std::cout << "car out" << "\n";
                            std::cout << _link -> m_N_out_car -> to_string() << "\n\n";
                            std::cout << "last valid time: " << _link -> m_last_valid_time << "\n";
                            std::cout << "_t_arrival: " << _t_arrival << "\n";
                            std::cout << "_t_depart: " << _t_depart << "\n";
                            std::cout << "_t_arrival_lift_up: " << _t_arrival_lift_up << "\n";
                            std::cout << "_t_depart_prime: " << _t_depart_prime << "\n";
                            std::cout << "m_queue_dissipated_time_car[_link_ID][_t_arrival_lift_up]: " << m_queue_dissipated_time_car[_link_ID][_t_arrival_lift_up] << "\n";
                            std::cout << "_t_queue_dissipated_valid: " << _t_queue_dissipated_valid << "\n";
                            std::cout << "_t_depart_lift_up: " << _t_depart_lift_up << "\n";
                            std::cout << "_t_depart_lift_up_valid: " << _t_depart_lift_up_valid << "\n";
                            std::cout << "_fftt: " << _fftt << "\n";
                            std::cout << "m_link_tt_map[_link_ID][_t_queue_dissipated_valid]: " << m_link_tt_map[_link_ID][_t_queue_dissipated_valid - 1 < get_cur_loading_interval() ? _t_queue_dissipated_valid - 1 : get_cur_loading_interval() - 1] << "\n";
                            std::cout << "get_cur_loading_interval(): " << get_cur_loading_interval() << "\n";
                            exit(-1);
                        }
                        if (_t_depart_prime < _t_depart_lift_up_valid) {

                            _gradient = MNM_DTA_GRADIENT::get_departure_cc_slope_car(_link, TFlt(_t_depart_prime), TFlt(_t_depart_lift_up_valid + 1));
                            if (_gradient > DBL_EPSILON) {
                                _gradient = m_mcdta -> m_unit_time / _gradient;  // seconds
                                for (int t_prime = _t_arrival_lift_up; t_prime < _t_queue_dissipated_valid; ++t_prime) {
                                    MNM_DTA_GRADIENT::add_ltg_records_veh(_record, _link, _path, start_prt[t], t_prime, _gradient);
                                }
                            }
                        }  
                    }
                }
            } // end of link loop in _path
        } // end of time loop
    } // end of path loop

    // _record.size() = num_timesteps x num_links x num_path x num_assign_timesteps
    // path_ID, assign_time, link_ID, start_int, gradient
    int new_shape [2] = { (int) _record.size(), 5};
    auto result = py::array_t<double>(new_shape);
    auto result_buf = result.request();
    double *result_prt = (double *) result_buf.ptr;
    ltg_record* tmp_record;
    for (size_t i = 0; i < _record.size(); ++i){
        tmp_record = _record[i];
        result_prt[i * 5 + 0] = (double) tmp_record -> path_ID();
        // the count of 1 min interval
        result_prt[i * 5 + 1] = (double) tmp_record -> assign_int;
        result_prt[i * 5 + 2] = (double) tmp_record -> link_ID();
        // the count of unit time interval (5s)
        result_prt[i * 5 + 3] = (double) tmp_record -> link_start_int;
        result_prt[i * 5 + 4] = tmp_record -> gradient();
        // printf("path ID: %f, departure assign interval (5 s): %f, link ID: %f, time interval (5 s): %f, gradient: %f\n",
        //         result_prt[i * 5 + 0], result_prt[i * 5 + 1], result_prt[i * 5 + 2], result_prt[i * 5 + 3], result_prt[i * 5 + 4]);
    }
    for (size_t i = 0; i < _record.size(); ++i){
        delete _record[i];
    }
    _record.clear();
    return result;
}

py::array_t<double> Mcdta_Api::get_truck_ltg_matrix_sep(py::array_t<int>start_intervals, int threshold_timestamp)
{
    // input: intervals in which the agents are released for each path, 1 min interval = 12 5-s intervals
    // assume Mcdta_Api::build_link_cost_map() and Mcdta_Api::get_link_queue_dissipated_time() are invoked already
    auto start_buf = start_intervals.request();
    if (start_buf.ndim != 1){
        throw std::runtime_error("Error, Mcdta_Api::get_truck_ltg_matrix, input dimension mismatch");
    }
    int l = start_buf.shape[0];
    int *start_prt = (int *) start_buf.ptr;

    std::vector<ltg_record*> _record = std::vector<ltg_record*>();
    bool _flg; 
    TFlt _fftt, _gradient;
    int _t_arrival, _t_depart, _t_arrival_lift_up, _t_depart_lift_up, _t_depart_prime, _t_queue_dissipated_valid, _t_depart_lift_up_valid;
    for (auto *_path : m_path_vec_truck) {
        // check if the path does not include any link in m_link_vec
        _flg = false;
        for (auto *_link : m_link_vec) {
            if (_path -> is_link_in(_link -> m_link_ID)) {
                _flg = true;
                break;
            }
        }
        if (!_flg) {
            continue;
        }

        for (int t = 0; t < l; ++t){
            // printf("Current processing time: %d\n", t);
            if (start_prt[t] >= get_cur_loading_interval()){
                throw std::runtime_error("Error, Mcdta_Api::get_truck_ltg_matrix, input start intervals exceeds the total loading intervals - 1");
            }

            _t_arrival = -1, _t_depart = -1, _t_arrival_lift_up = -1, _t_depart_lift_up = -1, _t_depart_prime = -1;
            // trace one additional veh departing from origin of path at start_prt[t]
            _t_depart = start_prt[t];
            for (TInt _link_ID : _path -> m_link_vec) {
                // arrival and departure time of original perturbation vehicle
                _t_arrival = _t_depart;
                _t_depart = _t_arrival + MNM_Ults::round_up_time(m_link_tt_map_truck[_link_ID][_t_arrival < get_cur_loading_interval() ? _t_arrival : get_cur_loading_interval() - 1]);
                
                // arrival time of the new perturbation vehicle
                auto *_link = dynamic_cast<MNM_Dlink_Multiclass*>(m_mcdta -> m_link_factory -> get_link(_link_ID));
                if (dynamic_cast<MNM_Dlink_Pq_Multiclass*>(_link) != nullptr) {
                    _t_arrival_lift_up = _t_arrival;  // for last pq, _t_arrival_lift_up >= get_cur_loading_interval()
                }
                else {
                    IAssert(dynamic_cast<MNM_Dlink_Ctm_Multiclass*>(_link) != nullptr);
                    IAssert(_t_depart_lift_up >= 0);  // from its upstream link
                    _t_arrival_lift_up = _t_depart_lift_up;
                }
                IAssert(_t_arrival_lift_up >= _t_arrival);

                IAssert(_link -> m_last_valid_time > 0);
                if (_t_arrival_lift_up > int(round(_link -> m_last_valid_time - 1)) || _t_arrival_lift_up >= threshold_timestamp) {
                    break;
                }

                // departure time of new perturbation vehicle
                _t_depart_prime = _t_arrival_lift_up + MNM_Ults::round_up_time(m_link_tt_map_truck[_link_ID][_t_arrival_lift_up < get_cur_loading_interval() ? _t_arrival_lift_up : get_cur_loading_interval() - 1]);

                // arrival time of the NEXT new perturbation for the NEXT link
                _fftt = dynamic_cast<MNM_Dlink_Multiclass*>(m_mcdta -> m_link_factory -> get_link(_link_ID)) -> get_link_freeflow_tt_loading_truck();
                // _fftt = dynamic_cast<MNM_Dlink_Ctm_Multiclass*>(m_mcdta -> m_link_factory -> get_link(_link_ID)) -> get_link_freeflow_tt_truck() / m_mcdta -> m_unit_time;
                _t_depart_lift_up = m_queue_dissipated_time_truck[_link_ID][_t_arrival_lift_up] + MNM_Ults::round_up_time(_fftt);
                    
                if (!m_link_congested_truck[_link_ID][_t_arrival_lift_up]) {
                    if (m_queue_dissipated_time_truck[_link_ID][_t_arrival_lift_up] == _t_arrival_lift_up) {
                        IAssert(_t_arrival_lift_up + MNM_Ults::round_up_time(_fftt) == _t_depart_lift_up);
                    }
                    else {
                        // critical state where subgradient applies
                        IAssert(m_queue_dissipated_time_truck[_link_ID][_t_arrival_lift_up] > _t_arrival_lift_up);
                        IAssert(_t_arrival_lift_up + MNM_Ults::round_up_time(_fftt) < _t_depart_lift_up);
                    }
                }
                else {
                    IAssert(m_queue_dissipated_time_truck[_link_ID][_t_arrival_lift_up] > _t_arrival_lift_up);
                    IAssert(_t_arrival_lift_up + MNM_Ults::round_up_time(_fftt) < _t_depart_lift_up);
                }

                if (_t_depart_lift_up < _t_depart_prime) {
                    printf("Error, Mcdta_Api::get_truck_ltg_matrix, something is wrong");
                    exit(-1);
                    // _t_depart_lift_up can be equal to _t_depart_prime, when the arrival curve is horizontal
                }

                if (_t_depart_prime < get_cur_loading_interval() - 1 &&
                    std::find_if(m_link_vec.begin(), m_link_vec.end(), 
                                 [&_link_ID](const MNM_Dlink_Multiclass *_l){return _l -> m_link_ID == _link_ID;}) != m_link_vec.end()) {
                    if (m_link_congested_truck[_link_ID][_t_arrival_lift_up] && _t_depart_lift_up > _t_depart_prime) {
                        IAssert(_link -> m_last_valid_time > 0);
                        if (m_queue_dissipated_time_truck[_link_ID][_t_arrival_lift_up] <= int(round(_link -> m_last_valid_time - 1))) {
                            _t_queue_dissipated_valid = m_queue_dissipated_time_truck[_link_ID][_t_arrival_lift_up];
                            _t_depart_lift_up_valid = _t_depart_lift_up;
                        }
                        else {
                            _t_queue_dissipated_valid = int(round(_link -> m_last_valid_time));
                            _t_depart_lift_up_valid = _t_queue_dissipated_valid - 1 + MNM_Ults::round_up_time(m_link_tt_map_truck[_link_ID][_t_queue_dissipated_valid - 1 < get_cur_loading_interval() ? _t_queue_dissipated_valid - 1 : get_cur_loading_interval() - 1]);
                        }
                        // jiachao added
                        if (_t_depart_lift_up_valid > get_cur_loading_interval() - 1) {
                            _t_depart_lift_up_valid = get_cur_loading_interval() - 1;
                        }
                        IAssert(_t_depart_lift_up_valid <= get_cur_loading_interval() - 1);
                        IAssert(_t_arrival_lift_up < _t_queue_dissipated_valid);
                        if (_t_depart_prime > _t_depart_lift_up_valid) {
                            std::cout << "\nError, Mcdta_Api::get_truck_ltg_matrix" << "\n";
                            std::cout << "interval: " << start_prt[t] << ", link: " << _link_ID << "\n";
                            std::cout << "truck in" << "\n";
                            std::cout << _link -> m_N_in_truck -> to_string() << "\n";
                            std::cout << "truck out" << "\n";
                            std::cout << _link -> m_N_out_truck -> to_string() << "\n\n";
                            std::cout << "last valid time: " << _link -> m_last_valid_time << "\n";
                            std::cout << "_t_arrival: " << _t_arrival << "\n";
                            std::cout << "_t_depart: " << _t_depart << "\n";
                            std::cout << "_t_arrival_lift_up: " << _t_arrival_lift_up << "\n";
                            std::cout << "_t_depart_prime: " << _t_depart_prime << "\n";
                            std::cout << "m_queue_dissipated_time_truck[_link_ID][_t_arrival_lift_up]: " << m_queue_dissipated_time_truck[_link_ID][_t_arrival_lift_up] << "\n";
                            std::cout << "_t_queue_dissipated_valid: " << _t_queue_dissipated_valid << "\n";
                            std::cout << "_t_depart_lift_up: " << _t_depart_lift_up << "\n";
                            std::cout << "_t_depart_lift_up_valid: " << _t_depart_lift_up_valid << "\n";
                            std::cout << "_fftt: " << _fftt << "\n";
                            std::cout << "m_link_tt_map_truck[_link_ID][_t_queue_dissipated_valid]: " << m_link_tt_map_truck[_link_ID][_t_queue_dissipated_valid - 1 < get_cur_loading_interval() ? _t_queue_dissipated_valid - 1 : get_cur_loading_interval() - 1] << "\n";
                            std::cout << "get_cur_loading_interval(): " << get_cur_loading_interval() << "\n";
                            exit(-1);
                        }
                        if (_t_depart_prime < _t_depart_lift_up_valid) {
                            _gradient = MNM_DTA_GRADIENT::get_departure_cc_slope_truck(_link, TFlt(_t_depart_prime), TFlt(_t_depart_lift_up_valid + 1));
                            if (_gradient > DBL_EPSILON) {
                                _gradient = m_mcdta -> m_unit_time / _gradient;  // seconds
                                for (int t_prime = _t_arrival_lift_up; t_prime < _t_queue_dissipated_valid; ++t_prime) {
                                    MNM_DTA_GRADIENT::add_ltg_records_veh(_record, _link, _path, start_prt[t], t_prime, _gradient);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // _record.size() = num_timesteps x num_links x num_path x num_assign_timesteps
    // path_ID, assign_time, link_ID, start_int, gradient
    int new_shape [2] = { (int) _record.size(), 5};
    auto result = py::array_t<double>(new_shape);
    auto result_buf = result.request();
    double *result_prt = (double *) result_buf.ptr;
    ltg_record* tmp_record;
    for (size_t i = 0; i < _record.size(); ++i){
        tmp_record = _record[i];
        result_prt[i * 5 + 0] = (double) tmp_record -> path_ID();
        // the count of 1 min interval
        result_prt[i * 5 + 1] = (double) tmp_record -> assign_int;
        result_prt[i * 5 + 2] = (double) tmp_record -> link_ID();
        // the count of unit time interval (5s)
        result_prt[i * 5 + 3] = (double) tmp_record -> link_start_int;
        result_prt[i * 5 + 4] = tmp_record -> gradient();
        // printf("path ID: %f, departure assign interval (5 s): %f, link ID: %f, time interval (5 s): %f, gradient: %f\n",
        //         result_prt[i * 5 + 0], result_prt[i * 5 + 1], result_prt[i * 5 + 2], result_prt[i * 5 + 3], result_prt[i * 5 + 4]);
    }
    for (size_t i = 0; i < _record.size(); ++i){
        delete _record[i];
    }
    _record.clear();
    return result;
}

py::array_t<double> Mcdta_Api::get_rh_ltg_matrix_sep(py::array_t<int>start_intervals, int threshold_timestamp)
{
    // input: intervals in which the agents are released for each path, 1 min interval = 12 5-s intervals
    // assume Mcdta_Api::build_link_cost_map() and Mcdta_Api::get_link_queue_dissipated_time() are invoked already
    auto start_buf = start_intervals.request();
    if (start_buf.ndim != 1){
        throw std::runtime_error("Error, Mcdta_Api::get_rh_ltg_matrix, input dimension mismatch");
    }
    int l = start_buf.shape[0];
    int *start_prt = (int *) start_buf.ptr;

    std::vector<ltg_record*> _record = std::vector<ltg_record*>();
    bool _flg; 
    TFlt _fftt, _gradient;
    int _t_arrival, _t_depart, _t_arrival_lift_up, _t_depart_lift_up, _t_depart_prime, _t_queue_dissipated_valid, _t_depart_lift_up_valid;
    for (auto *_path : m_path_vec_rh) {
        // check if the path does not include any link in m_link_vec
        _flg = false;
        for (auto *_link : m_link_vec) {
            if (_path -> is_link_in(_link -> m_link_ID)) {
                _flg = true;
                break;
            }
        }
        if (!_flg) {
            continue;
        }

        for (int t = 0; t < l; ++t){
            // printf("Current processing time: %d\n", t);
            if (start_prt[t] >= get_cur_loading_interval()){
                throw std::runtime_error("Error, Mcdta_Api::get_truck_ltg_matrix, input start intervals exceeds the total loading intervals - 1");
            }

            _t_arrival = -1, _t_depart = -1, _t_arrival_lift_up = -1, _t_depart_lift_up = -1, _t_depart_prime = -1;
            // trace one additional veh departing from origin of path at start_prt[t]
            _t_depart = start_prt[t];
            for (TInt _link_ID : _path -> m_link_vec) {
                // arrival and departure time of original perturbation vehicle
                _t_arrival = _t_depart;
                _t_depart = _t_arrival + MNM_Ults::round_up_time(m_link_tt_map[_link_ID][_t_arrival < get_cur_loading_interval() ? _t_arrival : get_cur_loading_interval() - 1]);
                
                // arrival time of the new perturbation vehicle
                auto *_link = dynamic_cast<MNM_Dlink_Multiclass*>(m_mcdta -> m_link_factory -> get_link(_link_ID));
                if (dynamic_cast<MNM_Dlink_Pq_Multiclass*>(_link) != nullptr) {
                    _t_arrival_lift_up = _t_arrival;  // for last pq, _t_arrival_lift_up >= get_cur_loading_interval()
                }
                else {
                    IAssert(dynamic_cast<MNM_Dlink_Ctm_Multiclass*>(_link) != nullptr);
                    IAssert(_t_depart_lift_up >= 0);  // from its upstream link
                    _t_arrival_lift_up = _t_depart_lift_up;
                }
                IAssert(_t_arrival_lift_up >= _t_arrival);

                IAssert(_link -> m_last_valid_time > 0);
                if (_t_arrival_lift_up > int(round(_link -> m_last_valid_time - 1)) || _t_arrival_lift_up >= threshold_timestamp) {
                    break;
                }

                // departure time of new perturbation vehicle
                _t_depart_prime = _t_arrival_lift_up + MNM_Ults::round_up_time(m_link_tt_map[_link_ID][_t_arrival_lift_up < get_cur_loading_interval() ? _t_arrival_lift_up : get_cur_loading_interval() - 1]);

                // arrival time of the NEXT new perturbation for the NEXT link
                _fftt = dynamic_cast<MNM_Dlink_Multiclass*>(m_mcdta -> m_link_factory -> get_link(_link_ID)) -> get_link_freeflow_tt_loading_truck();
                // _fftt = dynamic_cast<MNM_Dlink_Ctm_Multiclass*>(m_mcdta -> m_link_factory -> get_link(_link_ID)) -> get_link_freeflow_tt_truck() / m_mcdta -> m_unit_time;
                _t_depart_lift_up = m_queue_dissipated_time_car[_link_ID][_t_arrival_lift_up] + MNM_Ults::round_up_time(_fftt);
                    
                if (!m_link_congested_car[_link_ID][_t_arrival_lift_up]) {
                    if (m_queue_dissipated_time_car[_link_ID][_t_arrival_lift_up] == _t_arrival_lift_up) {
                        IAssert(_t_arrival_lift_up + MNM_Ults::round_up_time(_fftt) == _t_depart_lift_up);
                    }
                    else {
                        // critical state where subgradient applies
                        IAssert(m_queue_dissipated_time_car[_link_ID][_t_arrival_lift_up] > _t_arrival_lift_up);
                        IAssert(_t_arrival_lift_up + MNM_Ults::round_up_time(_fftt) < _t_depart_lift_up);
                    }
                }
                else {
                    IAssert(m_queue_dissipated_time_car[_link_ID][_t_arrival_lift_up] > _t_arrival_lift_up);
                    IAssert(_t_arrival_lift_up + MNM_Ults::round_up_time(_fftt) < _t_depart_lift_up);
                }

                if (_t_depart_lift_up < _t_depart_prime) {
                    printf("Error, Mcdta_Api::get_truck_ltg_matrix, something is wrong");
                    exit(-1);
                    // _t_depart_lift_up can be equal to _t_depart_prime, when the arrival curve is horizontal
                }

                if (_t_depart_prime < get_cur_loading_interval() - 1 &&
                    std::find_if(m_link_vec.begin(), m_link_vec.end(), 
                                 [&_link_ID](const MNM_Dlink_Multiclass *_l){return _l -> m_link_ID == _link_ID;}) != m_link_vec.end()) {
                    if (m_link_congested_car[_link_ID][_t_arrival_lift_up] && _t_depart_lift_up > _t_depart_prime) {
                        IAssert(_link -> m_last_valid_time > 0);
                        if (m_queue_dissipated_time_car[_link_ID][_t_arrival_lift_up] <= int(round(_link -> m_last_valid_time - 1))) {
                            _t_queue_dissipated_valid = m_queue_dissipated_time_car[_link_ID][_t_arrival_lift_up];
                            _t_depart_lift_up_valid = _t_depart_lift_up;
                        }
                        else {
                            _t_queue_dissipated_valid = int(round(_link -> m_last_valid_time));
                            _t_depart_lift_up_valid = _t_queue_dissipated_valid - 1 + MNM_Ults::round_up_time(m_link_tt_map[_link_ID][_t_queue_dissipated_valid - 1 < get_cur_loading_interval() ? _t_queue_dissipated_valid - 1 : get_cur_loading_interval() - 1]);
                        }

                        // bug !!!
                        if (_t_depart_lift_up_valid > get_cur_loading_interval() - 1){
                            // std::cout << "\nError, Mcdta_Api::get_rh_ltg_matrix: _t_depart_lift_up_valid > get_cur_loading_interval() - 1" << "\n";
                            // std::cout << "interval: " << start_prt[t] << ", link: " << _link_ID << "\n";
                            // std::cout << "_t_depart_lift_up_valid: " << _t_depart_lift_up_valid << "\n"; 
                            _t_depart_lift_up_valid = get_cur_loading_interval() - 1;
                        }
                        IAssert(_t_depart_lift_up_valid <= get_cur_loading_interval() - 1); // bug here
                        IAssert(_t_arrival_lift_up < _t_queue_dissipated_valid);
                        if (_t_depart_prime > _t_depart_lift_up_valid) {
                            std::cout << "\nError, Mcdta_Api::get_rh_ltg_matrix" << "\n";
                            std::cout << "interval: " << start_prt[t] << ", link: " << _link_ID << "\n";
                            std::cout << "truck in" << "\n";
                            std::cout << _link -> m_N_in_car -> to_string() << "\n";
                            std::cout << "truck out" << "\n";
                            std::cout << _link -> m_N_out_car -> to_string() << "\n\n";
                            std::cout << "last valid time: " << _link -> m_last_valid_time << "\n";
                            std::cout << "_t_arrival: " << _t_arrival << "\n";
                            std::cout << "_t_depart: " << _t_depart << "\n";
                            std::cout << "_t_arrival_lift_up: " << _t_arrival_lift_up << "\n";
                            std::cout << "_t_depart_prime: " << _t_depart_prime << "\n";
                            std::cout << "m_queue_dissipated_time_car[_link_ID][_t_arrival_lift_up]: " << m_queue_dissipated_time_car[_link_ID][_t_arrival_lift_up] << "\n";
                            std::cout << "_t_queue_dissipated_valid: " << _t_queue_dissipated_valid << "\n";
                            std::cout << "_t_depart_lift_up: " << _t_depart_lift_up << "\n";
                            std::cout << "_t_depart_lift_up_valid: " << _t_depart_lift_up_valid << "\n";
                            std::cout << "_fftt: " << _fftt << "\n";
                            std::cout << "m_link_tt_map[_link_ID][_t_queue_dissipated_valid]: " << m_link_tt_map[_link_ID][_t_queue_dissipated_valid - 1 < get_cur_loading_interval() ? _t_queue_dissipated_valid - 1 : get_cur_loading_interval() - 1] << "\n";
                            std::cout << "get_cur_loading_interval(): " << get_cur_loading_interval() << "\n";
                            exit(-1);
                        }
                        if (_t_depart_prime < _t_depart_lift_up_valid) {
                            _gradient = MNM_DTA_GRADIENT::get_departure_cc_slope_car(_link, TFlt(_t_depart_prime), TFlt(_t_depart_lift_up_valid + 1));
                            if (_gradient > DBL_EPSILON) {
                                _gradient = m_mcdta -> m_unit_time / _gradient;  // seconds
                                for (int t_prime = _t_arrival_lift_up; t_prime < _t_queue_dissipated_valid; ++t_prime) {
                                    MNM_DTA_GRADIENT::add_ltg_records_veh(_record, _link, _path, start_prt[t], t_prime, _gradient);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // _record.size() = num_timesteps x num_links x num_path x num_assign_timesteps
    // path_ID, assign_time, link_ID, start_int, gradient
    int new_shape [2] = { (int) _record.size(), 5};
    auto result = py::array_t<double>(new_shape);
    auto result_buf = result.request();
    double *result_prt = (double *) result_buf.ptr;
    ltg_record* tmp_record;
    for (size_t i = 0; i < _record.size(); ++i){
        tmp_record = _record[i];
        result_prt[i * 5 + 0] = (double) tmp_record -> path_ID();
        // the count of 1 min interval
        result_prt[i * 5 + 1] = (double) tmp_record -> assign_int;
        result_prt[i * 5 + 2] = (double) tmp_record -> link_ID();
        // the count of unit time interval (5s)
        result_prt[i * 5 + 3] = (double) tmp_record -> link_start_int;
        result_prt[i * 5 + 4] = tmp_record -> gradient();
        // printf("path ID: %f, departure assign interval (5 s): %f, link ID: %f, time interval (5 s): %f, gradient: %f\n",
        //         result_prt[i * 5 + 0], result_prt[i * 5 + 1], result_prt[i * 5 + 2], result_prt[i * 5 + 3], result_prt[i * 5 + 4]);
    }
    for (size_t i = 0; i < _record.size(); ++i){
        delete _record[i];
    }
    _record.clear();
    return result;
}

/**********************************************************************************************************
***********************************************************************************************************
                        Pybind11
***********************************************************************************************************
***********************************************************************************************************/

PYBIND11_MODULE(MNMAPI, m) {
    m.doc() = R"pbdoc(
        Pybind11 example plugin
        -----------------------

        .. currentmodule:: cmake_example

        .. autosummary::
           :toctree: _generate

           run_dta
    )pbdoc";

    m.def("run_dta", &run_dta, R"pbdoc(
        Run MAC-POSTS dta model

        Some other explanation about the add function.
    )pbdoc");



    py::class_<Test_Types> (m, "test_types")
            .def(py::init<>())
            .def("get_list", &Test_Types::get_list, "test conversion")
            .def("get_matrix", &Test_Types::get_matrix, "test conversion")
            .def("get_sparse_matrix", &Test_Types::get_sparse_matrix, "test conversion")
            .def("get_sparse_matrix2", &Test_Types::get_sparse_matrix2, "test conversion");
    // m.def("subtract", [](int i, int j) { return i - j; }, R"pbdoc(
    //     Subtract two numbers

    //     Some other explanation about the subtract function.
    // )pbdoc");

    py::class_<Dta_Api> (m, "dta_api")
            .def(py::init<>())
            .def("initialize", &Dta_Api::initialize)
            .def("run_whole", &Dta_Api::run_whole)
            .def("install_cc", &Dta_Api::install_cc)
            .def("install_cc_tree", &Dta_Api::install_cc_tree)
            .def("get_cur_loading_interval", &Dta_Api::get_cur_loading_interval)
            .def("register_links", &Dta_Api::register_links)
            .def("register_paths", &Dta_Api::register_paths)
            .def("get_link_tt", &Dta_Api::get_link_tt)
            .def("get_path_tt", &Dta_Api::get_path_tt)
            .def("get_link_inflow", &Dta_Api::get_link_inflow)
            .def("get_link_in_cc", &Dta_Api::get_link_in_cc)
            .def("get_link_out_cc", &Dta_Api::get_link_out_cc)
            .def("get_dar_matrix", &Dta_Api::get_dar_matrix)
            .def("get_complete_dar_matrix", &Dta_Api::get_complete_dar_matrix);

    py::class_<Mcdta_Api_Biclass> (m, "Mcdta_Api_Biclass")
            .def(py::init<>())
            .def("initialize_biclass_sep", &Mcdta_Api_Biclass::initialize_biclass_sep)
            .def("preloading", &Mcdta_Api_Biclass::preloading)
            .def("run_once_control", &Mcdta_Api_Biclass::run_once_control)
            .def("run_whole_control", &Mcdta_Api_Biclass::run_whole_control)
            .def("run_whole_control_false", &Mcdta_Api_Biclass::run_whole_control_false)

            .def("register_paths_car", &Mcdta_Api_Biclass::register_paths_car)
            .def("register_paths_truck", &Mcdta_Api_Biclass::register_paths_truck)
            .def("register_links_count", &Mcdta_Api_Biclass::register_links_count)
            .def("register_links_tt", &Mcdta_Api_Biclass::register_links_tt)
            .def("register_links_density", &Mcdta_Api_Biclass::register_links_density)
            .def("install_cc_separate_with_trees_density", &Mcdta_Api_Biclass::install_cc_separate_with_trees_density)
            .def("get_cur_loading_interval", &Mcdta_Api_Biclass::get_cur_loading_interval)

            .def("get_link_inflow_car", &Mcdta_Api_Biclass::get_link_inflow_car)
            .def("get_link_inflow_truck", &Mcdta_Api_Biclass::get_link_inflow_truck)

            .def("get_link_outflow_car", &Mcdta_Api_Biclass::get_link_outflow_car)
            .def("get_link_outflow_truck", &Mcdta_Api_Biclass::get_link_outflow_truck)

            .def("get_link_tt_car", &Mcdta_Api_Biclass::get_link_tt_car)
            .def("get_link_tt_truck", &Mcdta_Api_Biclass::get_link_tt_truck)

            .def("get_link_fftt_car", &Mcdta_Api_Biclass::get_link_fftt_car)
            .def("get_link_fftt_truck", &Mcdta_Api_Biclass::get_link_fftt_truck)

            .def("get_link_density_car", &Mcdta_Api_Biclass::get_link_density_car)
            .def("get_link_density_truck", &Mcdta_Api_Biclass::get_link_density_truck)

            .def("get_link_density_car_robust", &Mcdta_Api_Biclass::get_link_density_car_robust)
            .def("get_link_density_truck_robust", &Mcdta_Api_Biclass::get_link_density_truck_robust)

            .def("get_car_dar_matrix_count", &Mcdta_Api_Biclass::get_car_dar_matrix_count)
            .def("get_truck_dar_matrix_count", &Mcdta_Api_Biclass::get_truck_dar_matrix_count)

            .def("get_car_dar_matrix_tt", &Mcdta_Api_Biclass::get_car_dar_matrix_tt)
            .def("get_truck_dar_matrix_tt", &Mcdta_Api_Biclass::get_truck_dar_matrix_tt)

            .def("get_car_dar_matrix_density_in", &Mcdta_Api_Biclass::get_car_dar_matrix_density_in)
            .def("get_truck_dar_matrix_density_in", &Mcdta_Api_Biclass::get_truck_dar_matrix_density_in)
            
            .def("get_car_dar_matrix_density_out", &Mcdta_Api_Biclass::get_car_dar_matrix_density_out)
            .def("get_truck_dar_matrix_density_out", &Mcdta_Api_Biclass::get_truck_dar_matrix_density_out);


    py::class_<Mcdta_Api> (m, "Mcdta_Api")
            .def(py::init<>())
            .def("initialize", &Mcdta_Api::initialize)
            .def("initialize_curb", &Mcdta_Api::initialize_curb)
            .def("install_cc", &Mcdta_Api::install_cc)
            .def("install_cc_tree", &Mcdta_Api::install_cc_tree)
            .def("run_whole", &Mcdta_Api::run_whole)
            .def("run_once_control", &Mcdta_Api::run_once_control)
            .def("run_whole_control", &Mcdta_Api::run_whole_control)
            .def("run_whole_control_false", &Mcdta_Api::run_whole_control_false)

            .def("preloading", &Mcdta_Api::preloading)
            .def("run_whole_curb", &Mcdta_Api::run_whole_curb)
            .def("run_whole_curb_false", &Mcdta_Api::run_whole_curb_false)
            .def("register_links", &Mcdta_Api::register_links)
            .def("get_cur_loading_interval", &Mcdta_Api::get_cur_loading_interval)
            .def("get_travel_stats", &Mcdta_Api::get_travel_stats)
            .def("print_emission_stats", &Mcdta_Api::print_emission_stats)
            .def("print_simulation_results", &Mcdta_Api::print_simulation_results)
            .def("build_link_cost_map", &Mcdta_Api::build_link_cost_map)
            .def("get_link_queue_dissipated_time", &Mcdta_Api::get_link_queue_dissipated_time)
            .def("get_car_link_fftt", &Mcdta_Api::get_car_link_fftt)
            .def("get_truck_link_fftt", &Mcdta_Api::get_truck_link_fftt)
            .def("get_car_link_tt", &Mcdta_Api::get_car_link_tt)
            .def("get_car_link_tt_robust", &Mcdta_Api::get_car_link_tt_robust)
            .def("get_truck_link_tt", &Mcdta_Api::get_truck_link_tt)
            .def("get_car_link_speed", &Mcdta_Api::get_car_link_speed)
            .def("get_truck_link_speed", &Mcdta_Api::get_truck_link_speed)

            .def("get_link_car_inflow", &Mcdta_Api::get_link_car_inflow)
            .def("get_link_truck_inflow", &Mcdta_Api::get_link_truck_inflow)
            .def("get_link_rh_inflow", &Mcdta_Api::get_link_rh_inflow)
            .def("get_link_car_outflow", &Mcdta_Api::get_link_car_outflow)
            .def("get_link_truck_outflow", &Mcdta_Api::get_link_truck_outflow)

            .def("register_paths", &Mcdta_Api::register_paths)
            .def("register_paths_cc", &Mcdta_Api::register_paths_cc)
            .def("get_car_link_out_cc", &Mcdta_Api::get_car_link_out_cc)
            .def("get_car_link_in_cc", &Mcdta_Api::get_car_link_in_cc)
            .def("get_truck_link_out_cc", &Mcdta_Api::get_truck_link_out_cc)
            .def("get_truck_link_in_cc", &Mcdta_Api::get_truck_link_in_cc)            
            .def("get_enroute_and_queue_veh_stats_agg", &Mcdta_Api::get_enroute_and_queue_veh_stats_agg)
            .def("get_queue_veh_each_link", &Mcdta_Api::get_queue_veh_each_link)
            .def("get_car_link_out_num", &Mcdta_Api::get_car_link_out_num)
            .def("get_truck_link_out_num", &Mcdta_Api::get_truck_link_out_num)

            // .def("get_car_dar_matrix", &Mcdta_Api::get_car_dar_matrix)
            // .def("get_truck_dar_matrix", &Mcdta_Api::get_truck_dar_matrix)
            // .def("get_rh_dar_matrix", &Mcdta_Api::get_rh_dar_matrix)

            // .def("get_car_dar_arrival_matrix", &Mcdta_Api::get_car_dar_arrival_matrix)
            // .def("get_car_dar_departure_matrix", &Mcdta_Api::get_car_dar_departure_matrix)
            // .def("get_truck_dar_arrival_matrix", &Mcdta_Api::get_truck_dar_arrival_matrix)
            // .def("get_rh_dar_arrival_matrix", &Mcdta_Api::get_rh_dar_arrival_matrix)
            // .def("get_truck_dar_departure_matrix", &Mcdta_Api::get_truck_dar_departure_matrix)
            // .def("get_rh_dar_departure_matrix", &Mcdta_Api::get_rh_dar_departure_matrix)

            // For scenarios in McKees Rocks project:
            .def("get_waiting_time_at_intersections", &Mcdta_Api::get_waiting_time_at_intersections)
            .def("get_link_spillback", &Mcdta_Api::get_link_spillback)
            .def("get_path_tt_car", &Mcdta_Api::get_path_tt_car)
            .def("get_path_tt_truck", &Mcdta_Api::get_path_tt_truck)
            
            // for PGH curb DODE
            .def("initialize_curb_sep", &Mcdta_Api::initialize_curb_sep)
            .def("initialize_biclass_sep", &Mcdta_Api::initialize_biclass_sep)
            .def("register_paths_car", &Mcdta_Api::register_paths_car)
            .def("register_paths_truck", &Mcdta_Api::register_paths_truck)
            .def("register_paths_rh", &Mcdta_Api::register_paths_rh)

            .def("register_links_count", &Mcdta_Api::register_links_count)
            .def("register_links_tt", &Mcdta_Api::register_links_tt)
            .def("register_links_curb", &Mcdta_Api::register_links_curb)
            .def("register_links_density", &Mcdta_Api::register_links_density)
            
            .def("install_cc_separate", &Mcdta_Api::install_cc_separate)
            .def("install_cc_tree_separate", &Mcdta_Api::install_cc_tree_separate)
            .def("install_cc_separate_with_trees_curb", &Mcdta_Api::install_cc_separate_with_trees_curb)
            .def("install_cc_separate_with_trees_density", &Mcdta_Api::install_cc_separate_with_trees_density)
            .def("delete_mcdta", &Mcdta_Api::delete_mcdta)

            .def("get_car_link_tt_sep", &Mcdta_Api::get_car_link_tt_sep)
            .def("get_truck_link_tt_sep", &Mcdta_Api::get_truck_link_tt_sep)

            .def("get_link_car_inflow_sep", &Mcdta_Api::get_link_car_inflow_sep)
            .def("get_link_truck_inflow_sep", &Mcdta_Api::get_link_truck_inflow_sep)
            .def("get_link_rh_inflow_sep", &Mcdta_Api::get_link_rh_inflow_sep)

            // .def("get_link_density_car_sep", &Mcdta_Api::get_link_density_car_sep)
            // .def("get_link_density_truck_sep", &Mcdta_Api::get_link_density_truck_sep)
            
            .def("get_link_k_car_total_robust", &Mcdta_Api::get_link_k_car_total_robust)
            .def("get_link_k_truck_total_robust", &Mcdta_Api::get_link_k_truck_total_robust)

            .def("get_link_k_car_moving_robust", &Mcdta_Api::get_link_k_car_moving_robust)
            .def("get_link_k_truck_moving_robust", &Mcdta_Api::get_link_k_truck_moving_robust)
            .def("get_link_k_car_parking_robust", &Mcdta_Api::get_link_k_car_parking_robust)
            .def("get_link_k_truck_parking_robust", &Mcdta_Api::get_link_k_truck_parking_robust)

            .def("get_link_truck_curb_inflow_sep", &Mcdta_Api::get_link_truck_curb_inflow_sep)
            .def("get_link_rh_curb_inflow_sep", &Mcdta_Api::get_link_rh_curb_inflow_sep)
            .def("get_link_truck_curb_outflow_sep", &Mcdta_Api::get_link_truck_curb_outflow_sep)
            .def("get_link_rh_curb_outflow_sep", &Mcdta_Api::get_link_rh_curb_outflow_sep)

            .def("get_car_dar_matrix_sep", &Mcdta_Api::get_car_dar_matrix_sep)
            .def("get_truck_dar_matrix_sep", &Mcdta_Api::get_truck_dar_matrix_sep)
            .def("get_rh_dar_matrix_sep", &Mcdta_Api::get_rh_dar_matrix_sep)

            .def("get_car_dar_k_in", &Mcdta_Api::get_car_dar_k_in)
            .def("get_truck_dar_k_in", &Mcdta_Api::get_truck_dar_k_in)
            .def("get_rh_dar_k_in", &Mcdta_Api::get_rh_dar_k_in)
            .def("get_car_dar_k_out", &Mcdta_Api::get_car_dar_k_out)
            .def("get_truck_dar_k_out", &Mcdta_Api::get_truck_dar_k_out)
            .def("get_rh_dar_k_out", &Mcdta_Api::get_rh_dar_k_out)
            .def("get_truck_dar_k_in_parking", &Mcdta_Api::get_truck_dar_k_in_parking)
            .def("get_rh_dar_k_in_parking", &Mcdta_Api::get_rh_dar_k_in_parking)
            .def("get_truck_dar_k_out_parking", &Mcdta_Api::get_truck_dar_k_out_parking)
            .def("get_rh_dar_k_out_parking", &Mcdta_Api::get_rh_dar_k_out_parking)

            .def("get_car_dar_matrix_tt", &Mcdta_Api::get_car_dar_matrix_tt)
            .def("get_truck_dar_matrix_tt", &Mcdta_Api::get_truck_dar_matrix_tt)
            .def("get_rh_dar_matrix_tt", &Mcdta_Api::get_rh_dar_matrix_tt)

            .def("get_truck_dar_arrival_matrix_sep", &Mcdta_Api::get_truck_dar_arrival_matrix_sep)
            .def("get_truck_dar_departure_matrix_sep", &Mcdta_Api::get_truck_dar_departure_matrix_sep)
            .def("get_rh_dar_arrival_matrix_sep", &Mcdta_Api::get_rh_dar_arrival_matrix_sep)
            .def("get_rh_dar_departure_matrix_sep", &Mcdta_Api::get_rh_dar_departure_matrix_sep)            

            // .def("get_car_ltg_matrix", &Mcdta_Api::get_car_ltg_matrix)
            // .def("get_truck_ltg_matrix", &Mcdta_Api::get_truck_ltg_matrix)
            // .def("get_rh_ltg_matrix", &Mcdta_Api::get_rh_ltg_matrix)
            
            .def("get_car_ltg_matrix_sep", &Mcdta_Api::get_car_ltg_matrix_sep)
            .def("get_truck_ltg_matrix_sep", &Mcdta_Api::get_truck_ltg_matrix_sep)
            .def("get_rh_ltg_matrix_sep", &Mcdta_Api::get_rh_ltg_matrix_sep);

#ifdef VERSION_INFO
    m.attr("__version__") = VERSION_INFO;
#else
    m.attr("__version__") = "dev";
#endif
}
