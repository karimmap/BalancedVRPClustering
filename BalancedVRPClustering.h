#pragma once

#include <cstddef>
#include <vector>

#include "problem.pb.h"

#include <math.h> /* cos */

#include "InvalidMoveException.h"
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include<string.h>
#include <ctime>
#include <list>

#define PI 3.14159265

double Max = std::numeric_limits<double>::max();
struct item_with_limit_violation{
  const problem::Service & _service;
  double _diff_limit_violation;
  double _ratio_limit_violation;
  int _closest_cluster_index;
  int _closest_cluster_wo_violation_index;
  double _minimum_with_limit_violation;
  item_with_limit_violation();
  item_with_limit_violation(const problem::Service & service,double diff_limit_violation,double ratio_limit_violation,int closest_cluster_index,
  int closest_cluster_wo_violation_index,double minimum_with_limit_violation) : _service(service),_diff_limit_violation(diff_limit_violation),
  _ratio_limit_violation(ratio_limit_violation),_closest_cluster_index(closest_cluster_index),_closest_cluster_wo_violation_index(closest_cluster_wo_violation_index),
  _minimum_with_limit_violation(minimum_with_limit_violation){};
};

struct centroid_weights{
  std::vector<double> _limit;
  double _compatibility;
  centroid_weights(std::vector<double>limit, double compatibility) : _limit(limit),_compatibility(compatibility){};
};

std::vector< centroid_weights *> centroids_weights;

auto compatibleVehicle(problem::Problem& problem) -> void {
  for (int i = 0; i < problem.services_size(); ++i) {
    auto service = problem.mutable_services(i);
    for (int k = 0; k < problem.vehicles_size(); ++k) {
      bool compatible = true;
      auto vehicle    = problem.mutable_vehicles(k);
      for (int u = 0; u < service->quantities_size() && compatible; ++u) {
        auto quantity = service->quantities(u);
        auto capacity = vehicle->capacities(u);
        if (quantity > capacity.limit() && capacity.limit() != -1) {
          compatible = false;
        }
      }
      if (service->skills_size() - vehicle->skills_size() > 0) {
        compatible = false;
      }
      for (int s = 0; s < service->skills_size() && compatible; ++s) {
        for (int sv = 0; sv < vehicle->skills_size() && compatible; ++sv) {
          if (service->skills(s) != vehicle->skills(s)) {
            compatible = false;
          }
        }
      }
      if (compatible) {
        service->add_compatible_vehicle_indices(k);
      }

    }
  }
  // initialize load vector
  for(int v = 0; v < problem.vehicles_size(); v++){
    auto vehicle = problem.mutable_vehicles(v);
    for(int c = 0; c < vehicle->capacities_size(); ++c){
      vehicle->add_load(0.0);
    }
  }
}

auto Euclidean_distance(const problem::Location & loc_a, const problem::Location & loc_b)
    -> double {
  double delta_lat = loc_a.latitude() - loc_b.latitude();
  double delta_lon = (loc_a.longitude() - loc_b.longitude()) *
                     cos((loc_a.latitude() - loc_b.latitude()) * PI /
                         360.0); // Correct the length of a lon difference with cosine of
                                 // avereage latitude

  return 111321 *
         sqrt(pow(delta_lat, 2) +
              pow(delta_lon,
                  2)); // 111321 is the length of a degree (of lon and lat) in meters
}

auto Compute_distance_from_and_to_depot(problem::Problem problem) -> std::vector<double> {
  std::vector<double> duration_from_and_to_depot(problem.services_size());
  std::vector<double> time_matrix_from_depot(problem.services_size());
  std::vector<double> time_matrix_to_depot(problem.services_size());
  for (auto k : problem.vehicles()) {
    if (k.matrix_index() >= 0) {
      auto matrix = problem.matrices(k.matrix_index()).time();
      for (int i = 0; i < problem.services_size(); ++i) {
        auto service              = problem.services(i);
        time_matrix_from_depot[i] = double(matrix.at(service.matrix_index()));
        time_matrix_to_depot[i] = matrix.at(service.matrix_index() * sqrt(matrix.size()));
      }
    } else {
      for (int i = 0; i < problem.services_size(); ++i) {
        auto service = problem.services(i);
        time_matrix_from_depot[i] =
            Euclidean_distance(k.start_location(), service.location());
        time_matrix_to_depot[i] =
            Euclidean_distance(service.location(), k.start_location());
      }
    }
  }

  for (int i = 0; i < problem.services_size(); ++i) {
    duration_from_and_to_depot[i] = time_matrix_from_depot[i] + time_matrix_to_depot[i];
  }
  return duration_from_and_to_depot;
}

auto none(problem::Problem problem) -> bool {
  bool is_empty = true;
  for (auto i : problem.vehicles()) {
    if (!(i.capacities().empty())) {
      is_empty = false;
    }
    break;
  }
  return is_empty;
}

auto Compute_limits(int cut_index, problem::Problem& problem, double cut_ratio) -> void {
  double cumul_metric = 0.0;
  for (auto i : problem.services()) {
    cumul_metric += i.quantities(cut_index);
  }
  problem.set_cumulated_metric(cumul_metric);
  double total_work_time = 0.0;
  for (auto k : problem.vehicles()) {
    total_work_time += k.duration();
  }
  for (int i = 0; i < problem.vehicles_size(); ++i) {
    auto vehicle = problem.mutable_vehicles(i);
    if (cut_index > problem.unit_labels_size()) {
      vehicle->set_metric_limit(cut_ratio * vehicle->duration() / total_work_time);
    } else {
      vehicle->set_metric_limit(cut_ratio * problem.cumulated_metric() /
                                problem.vehicles_size());
    }
  }
}

auto flaying_distance(const problem::Location& loc_a, const problem::Location& loc_b)
    -> double {
  if (!(loc_a.latitude()) && !(loc_b.latitude()))
    return 0.0;
  if (abs(loc_a.latitude() - loc_b.latitude()) < 30 &&
      (std::max(loc_a.latitude(), loc_b.latitude()) +
       abs(loc_a.longitude() - loc_b.longitude())) < 100) {
    //       These limits ensures that relative error cannot be much greather than 2%
    //       For a distance like Bordeaux - Berlin, relative error between
    //       euclidean_distance and flying_distance is 0.1%.
    //       That is no need for trigonometric calculation.
    return Euclidean_distance(loc_a, loc_b);
  }
  double r             = 6378137; // Earth's radius in meters
  double deg2rad_lat_a = loc_a.latitude() * PI / 180;
  double deg2rad_lat_b = loc_b.latitude() * PI / 180;
  double deg2rad_lon_a = loc_a.longitude() * PI / 180;
  double deg2rad_lon_b = loc_b.longitude() * PI / 180;
  double lat_distance  = deg2rad_lat_b - deg2rad_lat_a;
  double lon_distance  = deg2rad_lon_b - deg2rad_lon_a;
  double intermediate  = pow(sin(lat_distance / 2), 2) + cos(deg2rad_lat_a) *
                                                            cos(deg2rad_lat_b) *
                                                            pow(sin(lon_distance / 2), 2);
  return r * 2 * atan2(sqrt(intermediate), sqrt(1 - intermediate));
}

auto check_if_projection_inside_the_line_segment(const problem::Location& point_coord,
                                                 const problem::Location& line_beg_coord,
                                                 const problem::Location& line_end_coord,
                                                 double margin) -> bool {
  /*
   * margin: if (0 > margin > 1), the percentage of the line segment that will be
   * considered as "outside" margin: if (margin < 0), the percentage that the "inside"
   * zone is extended [0, 1]: coordinates [lat, lon] or [lon, lat]
   */
  std::pair<double, double> line_direction;
  line_direction.first  = line_end_coord.latitude() - line_beg_coord.latitude();
  line_direction.second = line_end_coord.longitude() - line_beg_coord.longitude();

  std::pair<double, double> point_direction;
  point_direction.first  = point_coord.latitude() - line_beg_coord.latitude();
  point_direction.second = point_coord.longitude() - line_beg_coord.longitude();

  std::pair<double, double> projection_scaler_coord;
  projection_scaler_coord.first =
      line_direction.first * point_direction.first / pow(line_direction.first, 2);
  projection_scaler_coord.second =
      line_direction.second * point_direction.second / pow(line_direction.second, 2);
  /*
   * If projection_scaler
   * >1:        the projection is after the line segment end
   * <0:        it is before the line segment begin
   * otherwise: it is on the line segment define by begin_coord end end_coord
   */
  double projection_scaler =
      projection_scaler_coord.first + projection_scaler_coord.second;
  // If the following holds, it is inside the margin of the line segment
  if (projection_scaler >= 0.5 * margin && projection_scaler <= 1 - 0.5 * margin)
    return true;
  else return false;
}

auto item_nil(const problem::Problem& problem) -> bool {
  for (auto i : problem.services()) {
    if (i.quantities_size() == 0)
      return true;
  }
  return false;
}

auto Check_vehicle_location(const problem::Problem& problem) -> bool {
  for (auto i : problem.vehicles()) {
    if (i.start_location().latitude() || i.start_location().longitude())
      return false;
  }
  return true;
}

auto Check_service_location(const problem::Problem& problem) -> bool {
  for (auto i : problem.services()) {
    if (i.location().latitude() || i.location().longitude())
      return false;
  }
  return true;
}

auto Get_service_index(const problem::Service & service,  const problem::Problem & problem ) -> int {
  // std::string str1 =  service.name();
  // char * c = const_cast<char*>(str1.c_str());
  // for( int i = 0; i < problem.services_size(); ++i){
  //   std::string str2 = problem.services(i).name();
  //   char * cc = const_cast<char*>(str2.c_str());
  //   int result = strcmp(c, cc);
  //   if( result == 0){
  //     return i;
  //   }
  // }
  for(int i = 0; i < problem.services_size(); ++i){
    if(service.name() == problem.services(i).name()){
      return i;
    }
  }
}
auto Sum_services_visits(double l,const problem::Service service ) -> double{
  return l + service.visits();
}
/*
 *
 * ***************************Classe BalancedVRPClustering************************************
 *
 */
// class cluster{
//   std::vector<problem::Service *> _assigned_services;
//   problem::Vehicle * _vehicle;
//   std::vector<double> load;

// }
class BalancedVRPClustering {
public:
  problem::Problem &_problem;
  double const _cut_ratio;
  int const _cut_index;
  bool _apply_balancing;
  double _total_assigned_cut_load;
  double _percent_assigned_cut_load;
  bool _distance_matrix;
  double _rate_balance;
  std::vector<double> _limit_violation_coefficient;
  std::vector<item_with_limit_violation  *> _items_with_limit_violation;
  std::vector<int> _clusters_with_limit_violation;
  int _manage_empty_clusters_iterations;
  int _iteration;
  problem::Location  _old_centroids_lat_lon;
  std::vector<double> _last_n_average_diffs;
  int _limit_violation_count;
  double _expected_n_visits;
  std::vector<problem::Service * > _ser;

  BalancedVRPClustering();
  BalancedVRPClustering( problem::Problem  &problem, double const cut_ratio,
                        int const cut_index,bool apply_balancing,double total_assigned_cut_load,double percent_assigned_cut_load,
                         bool distance_matrix,double rate_balance);
  ~BalancedVRPClustering();
  auto build(problem::Problem &problem, double const cut_ratio, int const cut_index)
      -> void;
  auto populate_centroids(std::string const populate_methode,
                          int const number_of_cluster) -> void;
  auto exist(int v, const problem::Service service) -> bool;
  auto Check_if_initial_centroids_are_taken() -> bool;
  auto check_centroid_compatibility() -> bool;
  auto calc_initial_centroids() -> void;
  auto update_metrics(problem::Service * service, problem::Vehicle * vehicle) -> void;
  auto capacity_violation(const problem::Service &service, const problem::Vehicle &vehicle) -> bool;
  auto distance ( const problem::Service &service, const problem::Vehicle &vehicle) -> double;
  auto evaluate (const problem::Service & service) -> int;
  auto calculate_membership_clusters() -> void;
  auto manage_empty_clusters(std::string const populate_methode) -> void;
  auto eliminate_empty_clusters() -> void;
  auto centroids_converged_or_in_loop( int last_n_iterations) -> bool;
  auto move_limit_violating_dataitems() -> void;
};

BalancedVRPClustering::BalancedVRPClustering( problem::Problem  &problem,
                                             double const cut_ratio, int const cut_index,bool apply_balancing,double total_assigned_cut_load,double percent_assigned_cut_load,
                                              bool distance_matrix, double rate_balance):
    _problem(problem), _cut_ratio(cut_ratio), _cut_index(cut_index), _apply_balancing(apply_balancing),_total_assigned_cut_load(0.0),
     _percent_assigned_cut_load(1.0),_distance_matrix(false), _rate_balance(rate_balance),
     _manage_empty_clusters_iterations(0),
     _iteration(0),_old_centroids_lat_lon(_problem.services(0).location()),_limit_violation_count(0),_expected_n_visits(0.0) {

       _limit_violation_coefficient.resize(_problem.vehicles_size(),1.0);

       _clusters_with_limit_violation.resize(_problem.vehicles_size(),0);

       for(int i = 0; i < _problem.services_size(); ++i){
         _ser.push_back(problem.mutable_services(i));
       }

      // auto it = ser.begin() + 5;
      // std::rotate(it, it + 1, ser.end());
     }
BalancedVRPClustering::~BalancedVRPClustering() {
	for(auto item : _items_with_limit_violation)
	{
		delete item;
	}
}


enum Move_info{
  move_up,
  move_down,
  no_move
};
std::vector<Move_info *> Move;


auto BalancedVRPClustering::exist(int v, const problem::Service service) -> bool {
  for (int u = 0; u < service.compatible_vehicle_indices_size(); u++) {
    if (service.compatible_vehicle_indices(u) == v)
      return true;
  }
  return false;
}

auto BalancedVRPClustering::build(problem::Problem &problem, double const cut_ratio,
                                  int const cut_index) -> void {

  try {
    // return clean errors if unconsistent data
    if (item_nil(problem)) {
      throw InvalidMoveException("Error: \t unit should be provided for all item");
    } else if (Check_vehicle_location(problem)) {
      throw InvalidMoveException(
          "Location info (lattitude and longitude) should be provided for all vehicles");
    } else if (Check_service_location(problem)) {
      throw InvalidMoveException(
          "Location info (lattitude and longitude) should be provided for all items");
    }

    // if(problem.services(0).visits()  == 0){
    //   throw InvalidMoveException(
    //       " Error: \t Invalid visits initialisation");
    // }
    int max_iteration;
    if (problem.options().max_iteration() != 0)
      max_iteration = problem.options().max_iteration();
    else
      max_iteration = std::max(0.5 * problem.services_size(), 100.0);

    std::vector<double> distance_from_and_to_depot;
    if (cut_index > 3 || cut_index < 0)
      distance_from_and_to_depot =
          Compute_distance_from_and_to_depot(problem); // case: duration
    Compute_limits(cut_index, problem, cut_ratio);

    // Value
    for(int i = 0; i < problem.services_size(); ++i){
      auto service = problem.services(i);
      std::vector<double> limit(_problem.vehicles_size(), 1.0);
      double compatibility = 1.0;
      centroids_weights.push_back(new centroid_weights(limit, compatibility));
    }


    for(int i = 0; i < problem.services_size(); ++i){
      auto service = problem.mutable_services(i);
      if(service->visits() == 0){
        service->set_visits(1);
      }
    }
    _expected_n_visits = std::accumulate(problem.mutable_services()->begin(),problem.mutable_services()->end(),0.0,Sum_services_visits) / problem.vehicles_size();

    // *** Algo start ****//
    int iteration = 0;
    std::vector<int> i_like_to_move_it_move_it;
    int moved_up   = 0;
    int moved_down = 0;
    std::vector<double> cluster_with_capacity_violation;
    if (cut_index <= 3 && cut_index >= 0) {
      if (problem.cumulated_metric() == 0.0) {
        std::cout << "Disable balancing because there is no point" << std::endl;
      } else {
        if (cut_index < problem.unit_labels_size()) {
          std::sort(
              problem.mutable_services()->begin(), problem.mutable_services()->end(),
              [](const problem::Service& service1, const problem::Service& service2) {
                if (service1.quantities(0) > service2.quantities(0))
                  return true;
              });
        }
        std::random_shuffle(
            problem.mutable_services()->begin() + int(problem.services_size() * 0.1),
            problem.mutable_services()->begin() + int(problem.services_size() * 0.9));
      }
    }

  } catch (exception& e) {
    std::cout << e.what() << std::endl;
  }
}

auto BalancedVRPClustering::Check_if_initial_centroids_are_taken() -> bool {
  for(int i = 0; i < _problem.vehicles_size(); ++i){
    auto vehicle = _problem.vehicles(i);
    if( vehicle.matrix_index() == 0 && vehicle.initial_centroid().latitude() == 0.0 && vehicle.initial_centroid().longitude() == 0.0) return false;
  }
  return true;
}

auto BalancedVRPClustering::update_metrics( problem::Service * service, problem::Vehicle * vehicle) -> void{

  auto l = vehicle->mutable_load();
  for(int u = 0; u < service->quantities_size(); ++u){
    l->at(u) += service->quantities(u);
  }

  _total_assigned_cut_load += service->quantities(_cut_index);
  _percent_assigned_cut_load = _total_assigned_cut_load / _problem.cumulated_metric();
  bool cap = true;
  for( int v = 0; v < _problem.vehicles_size() && cap; v++  ){
    auto vehicle = _problem.vehicles(v);
    if(vehicle.capacities(_cut_index).limit() == 0.0){
      cap = false;
    }
  }
  if( !_apply_balancing && cap ){
    _apply_balancing = true;
  }
}

auto BalancedVRPClustering::capacity_violation(const problem::Service &service, const problem::Vehicle &vehicle) -> bool{
  for(int c = 0; c < vehicle.capacities_size(); ++c){
    if( vehicle.capacities(c).limit() != -1 && vehicle.load(c) + service.quantities(c) > vehicle.capacities(c).limit()) return true;
  }
  return false;
}

auto BalancedVRPClustering::distance ( const problem::Service &service, const problem::Vehicle &vehicle) -> double {
  double distance;
  if(_distance_matrix){
      auto matrix = _problem.matrices(vehicle.matrix_index()).distance();
      distance = double(matrix.at(vehicle.matrix_index() * sqrt(matrix.size()) + service.matrix_index()));
  }
  else{
    distance = flaying_distance(service.location(), vehicle.initial_centroid());
  }
  // balance between clusters computation
  double balance = 1.0;
  if(_apply_balancing){
    // At this stage of the clustering we would expect this limit to be met
    double expected_cut_limit = vehicle.metric_limit() * _percent_assigned_cut_load;
  // Compare "expected_cut_limit to the current cut_value
  //and penalize (or favorise) if cut_value/expected_cut_limit greater (or less) than 1.
    if(_percent_assigned_cut_load < 0.95){
    //First down-play the effect of balance (i.e., **power < 1)
    //After then make it more pronounced (i.e., **power > 1)
      balance = std::pow( (vehicle.capacities(_cut_index).limit() / expected_cut_limit), (2 + _rate_balance) * _percent_assigned_cut_load );
    }
    else {
      balance = vehicle.capacities(_cut_index).limit() / expected_cut_limit;
    }
  }
  if(_rate_balance) {
    return ((1.0 - _rate_balance) * distance + _rate_balance * distance * balance);
  }
  else {
    return distance * balance;
  }
}
auto BalancedVRPClustering::calc_initial_centroids() -> void {
  if(!Check_if_initial_centroids_are_taken()){
    populate_centroids("random",_problem.vehicles_size());
  }
  else{
    populate_centroids("indices",_problem.vehicles_size());
  }


}
auto BalancedVRPClustering::check_centroid_compatibility() -> bool {
  bool compatible = true;
  for( int v = 0; v < _problem.vehicles_size() && compatible; ++v){
    auto vehicle = _problem.vehicles(v);
    for( int i = 0; i < _problem.services_size(); ++i){ /// revoir
      auto service = _problem.services(i);
      if(vehicle.initial_centroid().matrix_index() == service.location().matrix_index() &&
       abs(vehicle.initial_centroid().latitude() - service.location().latitude()) < 1e-5 &&
       abs(vehicle.initial_centroid().longitude() - service.location().longitude()) < 1e-5 ){
         if(!exist( v,service)){
           compatible = false;
         }
       }
    }
  }
  if (compatible) {
    return true;
  }
  else {
    return false;
  }
}
auto BalancedVRPClustering::populate_centroids(std::string const populate_methode,
                                               int const number_of_cluster) -> void {
  /* Generate centroids based on remaining_skills available
   * Similarly with data_items, each centroid id defined by:
   * Latitude, Longitude
   * item_id
   * unit_fullfillment -> for each unit, quantity contained in corresponding cluster
   * Characterisits -> {v_id: skills: days: matrix_index}
   */
  try {
    if (_problem.vehicles_size() == 0) {
      throw InvalidMoveException("Error: \t No vehicles provided");
    }
    if (populate_methode == "random") {
      std::srand(std::time(nullptr)); // use current time as seed for random generator
      std::vector<int> disponible_services(_problem.services_size(), 0); // ALL indicies
      for (int i = 0; i < _problem.services_size(); ++i) {
        disponible_services[i] = i;
      }
      for (int v = 0; v < _problem.vehicles_size(); ++v) {
        // problem::Vehicle* vehicle = new problem::Vehicle(problem.vehicles(v));
        auto vehicle = _problem.mutable_vehicles(v);
        std::vector<int> compatible_services;
        // Find the items which are not already used, and specifically need the skill set
        // of this cluster
        for (auto i : disponible_services) {
          if (_problem.services(i).compatible_vehicle_indices_size() == 1 &&
              _problem.services(i).compatible_vehicle_indices(0) == v) {
            compatible_services.push_back(i);
          }
        }
        if (compatible_services.empty()) {
          // If there are no items which specifically needs these skills,
          // then find all the items that can be assigned to this cluster
          for (auto i : disponible_services) {
            if (exist(v, _problem.services(i))) {
              compatible_services.push_back(i);
            }
          }
        }

        int service_index = compatible_services[std::rand() % compatible_services.size()];
        disponible_services.erase(std::remove(disponible_services.begin(),
                                              disponible_services.end(), service_index),
                                  disponible_services.end());

        problem::Service* centroid = new problem::Service(_problem.services(service_index));
        vehicle->mutable_initial_centroid()->set_latitude(centroid->location().latitude());
        vehicle->mutable_initial_centroid()->set_longitude(centroid->location().longitude());
        vehicle->mutable_initial_centroid()->set_matrix_index(centroid->location().matrix_index());
      }
    }
    else if (populate_methode == "indices") {
      auto it = std::unique(
          _problem.mutable_vehicles()->begin(), _problem.mutable_vehicles()->end(),
          [](const problem::Vehicle& first, const problem::Vehicle& sec) {
            if (first.initial_centroid().matrix_index() == sec.initial_centroid().matrix_index())
              return true;
            else
              return false;
          });
      for( int i = 0; i < _problem.vehicles_size(); ++i){
        auto vehicle = _problem.vehicles(i);
        std::cout << " limitation vehicle " << vehicle.metric_limit() << std::endl;
      }
      if (it != _problem.mutable_vehicles()->end()) {
        throw InvalidMoveException(
            "Error: \t Same centroid_index provided several times");
      }
      if(!Check_if_initial_centroids_are_taken()){
        throw InvalidMoveException(
            "Error: \t Wrong number of initial centroids provided");
      }
      if(!check_centroid_compatibility()){
        throw InvalidMoveException(
            "Error: \t Centroid  is initialised with  incompatible service");
      }
    }

  } catch (exception& e) {
    std::cout << e.what() << std::endl;
  }
}

auto BalancedVRPClustering::evaluate (const problem::Service & service) -> int {
  std::vector<double> distances(_problem.vehicles_size(),0);
  for(int i = 0; i < _problem.vehicles_size(); i++){
    auto vehicle = _problem.vehicles(i);
    if(exist(i, service)){
      distances[i] = distance(service,vehicle);
    }
    else {
      distances[i] = pow(2,32);
    }
  }

  int closest_cluster_index = std::min_element(distances.begin(),distances.end()) - distances.begin(); // algorithm header
  if(!capacity_violation(service, _problem.vehicles(closest_cluster_index))){
    double minimum_without_limit_violation = pow(2,32); // Consider only compatible ones
    int closest_cluster_wo_violation_index = -1;

    for(int k = 0; k < _problem.vehicles_size(); ++k){
      if(distances[k] < minimum_without_limit_violation && !capacity_violation(service,_problem.vehicles(k))) {
        closest_cluster_wo_violation_index = k;
        minimum_without_limit_violation = distances[k];
      }
    }
    if(closest_cluster_wo_violation_index != -1){
      double minimum_with_limit_violation = *std::min_element(distances.begin(), distances.end());
      double diff = minimum_without_limit_violation - minimum_with_limit_violation;
      double ratio = minimum_without_limit_violation / minimum_with_limit_violation;
      if(minimum_with_limit_violation == 0.0){
        ratio = 1.0;
      }

      _items_with_limit_violation.push_back(new item_with_limit_violation(service,diff,ratio,closest_cluster_index,closest_cluster_wo_violation_index,minimum_with_limit_violation));

      _clusters_with_limit_violation[closest_cluster_index] = 1;


    }

  }
  return closest_cluster_index;
}

auto BalancedVRPClustering::eliminate_empty_clusters() -> void{
   for( int v = 0; v < _problem.vehicles_size(); ++v){
     auto vehicle = _problem.vehicles(v);
     if( vehicle.assigned_service_indices_size() == 0){
       std::cout << " \t The centroid (" << vehicle.initial_centroid().latitude() << ", " << vehicle.initial_centroid().longitude() << ") has no be selected. " << std::endl;
     }
   }
}

auto BalancedVRPClustering::manage_empty_clusters(std::string const populate_methode) -> void {
  _manage_empty_clusters_iterations +=1;
  if( populate_methode == "terminate"){
     // Do nothing to terminate with error. (The empty cluster will be assigned a nil centroid, and then calculating the distance from this centroid to another point will raise an exception.)
    return;
  }
  std::cout << " The gem logic isn't necessary in our implementation " << std::endl;

}

auto BalancedVRPClustering::centroids_converged_or_in_loop( int last_n_iterations) -> bool{
  // checks if there is a loop of size last_n_iterations

  if(_iteration == 0){
    // initialize vector stats vector
    return false;
  }
  // calculate total absolute centroid movment in meters
  double total_movement_meter = 0.0;
  for(int v = 0; v < _problem.vehicles_size(); ++v){
    auto vehicle = _problem.vehicles(v);
    if(vehicle.assigned_service_indices_size() != 0){
      total_movement_meter += Euclidean_distance(_old_centroids_lat_lon,vehicle.initial_centroid());
    }
  }
  std::cout << "Iteration: \t" << _iteration << " " << "total_centroid_movement: \t" << round(total_movement_meter) << "\t eucledian meters" << std::endl;
  _last_n_average_diffs.push_back(total_movement_meter); // add to the vector before convergence check in case other conditions are not satisfied
  // If converged we can stp
  if(_last_n_average_diffs[_last_n_average_diffs.size()] < _problem.vehicles_size() * 10){
    return true;
  }
  for(int i = 0; i < last_n_iterations; ++i){
    double last_n_iter_average_curr = std::accumulate(_last_n_average_diffs.begin() + 1, _last_n_average_diffs.begin() + i, 0.0);
    double last_n_iter_average_prev = std::accumulate(_last_n_average_diffs.begin() + (i + 1),_last_n_average_diffs.begin() + (2 * i), 0.0 );
    if( abs(last_n_iter_average_curr - last_n_iter_average_prev ) < 1e-5 ){
      return true;
    }
  }
  // Clean old stats
  _last_n_average_diffs.erase(_last_n_average_diffs.begin());
  return false;
}

// Instead lambda function to calculate the object vector mean
auto Sum_items_with_limit_violation_diff(double l,  const item_with_limit_violation  * y) -> double{
  return l + y->_diff_limit_violation;
}

auto Sum_items_with_limit_violation_ratio(double l,  const item_with_limit_violation  * y) -> double{
  return l + y->_ratio_limit_violation;
}

auto BalancedVRPClustering::move_limit_violating_dataitems() -> void{
  Move.resize(_problem.services_size());

  _limit_violation_count = _items_with_limit_violation.size();

  double f = std::accumulate(_items_with_limit_violation.begin(),_items_with_limit_violation.end(),0.0, Sum_items_with_limit_violation_diff);
  double mean_distance_diff = f / _items_with_limit_violation.size();
  double s = std::accumulate(_items_with_limit_violation.begin(),_items_with_limit_violation.end(),0.0, Sum_items_with_limit_violation_ratio);
  double mean_ratio = s / _items_with_limit_violation.size();

  int moved_up = 0;
  int moved_down = 0;
  /*
  # TODO: check if any other type of ordering might help
  # Since the last one that is moved up will appear at the very top and vice-versa for the down.
  # We need the most distant ones to closer to the top so that the cluster can move to that direction next iteration
  # but a random order might work better for the down ones since they are in the middle and the border is not a straight line
  # nothing vanilla order 9/34
  # @items_with_limit_violation.shuffle!  7/34 not much of help. it increases normal iteration count but decreases the loop time
  # @items_with_limit_violation.sort_by!{ |i| i[5] } 6/34 fails .. good
  # @items_with_limit_violation.sort_by!{ |i| -i[5] } 5/34 fails .. better
  # 0.33sort+ and 0.66sort-  #8/21 fails ... bad
  # 0.33shuffle and 0.66sort- #7/20 fails ... bad
  */
  std::sort(_items_with_limit_violation.begin(),_items_with_limit_violation.end(),
  [](const item_with_limit_violation * item1, const item_with_limit_violation * item2){
    if(item1->_minimum_with_limit_violation > item2->_minimum_with_limit_violation){
      return true;
    }
  });
  // for(int i = 0; i < _items_with_limit_violation.size(); i++){
  //   std::cout << " b " << _items_with_limit_violation[i]->_minimum_with_limit_violation << std::endl;
  // }
  while (!_items_with_limit_violation.empty()){
    const item_with_limit_violation * data = _items_with_limit_violation[_items_with_limit_violation.size() - 1];
    _items_with_limit_violation.pop_back();
    /*
    # TODO: check the effectiveness of the following stochastic condition.
    # Specifically, moving the "up" ones always would be better...?
    # But the downs are the really problematic ones so moving them would make sense too.
    # Tested some options but it looks alright as it is.. Needs more testing.

    # if the limt violation leads to more than 2-5 times distance increase, move it
    # otherwise, move it with a probability correlated to the detour it generates
    */

     double _rand = (double)rand() / ((double)RAND_MAX+1);
     if(data->_ratio_limit_violation > std::min(2 * mean_ratio, 5.0) || _rand < data->_diff_limit_violation / (3 * mean_distance_diff + 1e-10)) {
       problem::Location point = data[0]._service.location();
       problem::Location centroid_with_violation = _problem.vehicles(data->_closest_cluster_index).initial_centroid();
       problem::Location centroid_without_violation = _problem.vehicles(data->_closest_cluster_wo_violation_index).initial_centroid();

       problem::Location *centroid_without_violationn  = new problem::Location(centroid_with_violation);
       if(centroid_without_violationn != nullptr && check_if_projection_inside_the_line_segment(point, centroid_with_violation,centroid_without_violation,0.1)){
         moved_down += 1;
         int service_index_to_move = Get_service_index(data->_service, _problem );
         Move[service_index_to_move] = new Move_info(move_down);  // MOVE_DOWN
         int index = Get_service_index(data->_service, _problem );
         for(int w = 0; w < centroids_weights[index]->_limit.size(); ++w){
           centroids_weights[index]->_limit[w] = std::min( centroids_weights[index]->_limit[w] * 2.0, std::max(_expected_n_visits / 10 , 5.0)); // TODO: a better mechanism
         }
         centroids_weights[index]->_limit[data->_closest_cluster_index] = 1;
         // a thing no really understarnd
       }
       else {
         moved_up += 1;
         if(centroid_without_violationn != nullptr){
            int service_index_to_move = Get_service_index(data->_service, _problem );
            Move[service_index_to_move] = new Move_info(move_up);  // MOVE_DOWN

         }

       }

     }
  }
  if(_limit_violation_count > 0){
    std::cout << " \t Decisions taken due to capacity violation for " << _limit_violation_count << " items: " << moved_down << ", " << moved_up << " of them moved_up " << _limit_violation_count - moved_down - moved_up << " of them untouched " << std::endl;
    for(int i = 0; i < _clusters_with_limit_violation.size(); i++){
      if(_problem.services_size() <= 40){
        std::cout << " \t Clusters with limit violation (order) " <<  _clusters_with_limit_violation[i] ? " _ " : " "  ;
        std::cout << " " << i + 1 << std::endl;
      }

    }

  }

  // for(int i = 0; i < _items_with_limit_violation.size(); i++){
  //   std::cout << " a " << _items_with_limit_violation[i]->_minimum_with_limit_violation << std::endl;
  // }

}

auto BalancedVRPClustering::calculate_membership_clusters() -> void {
  for( int i = 0; i < _problem.services_size(); ++i){
    auto service = _problem.services(i);
    int cluster_index = evaluate(service);
    auto vehicle = _problem.mutable_vehicles(cluster_index);
    vehicle->add_assigned_service_indices(i);
    problem::Service * _service = new problem::Service (service);
    update_metrics(_service,vehicle);
  }
  // Isn't necessary to call manage_empty_clusters
}
