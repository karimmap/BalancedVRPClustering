#pragma once

#include <cstddef>
#include <vector>

#include "problem.pb.h"

#include <math.h> /* cos */

#include "InvalidMoveException.h"
#include <algorithm>
#include <cstdio>
#include <cstdlib>
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
struct cluster_with_limit_violation{
  std::vector<int> _x;
  cluster_with_limit_violation();
  cluster_with_limit_violation(std::vector<int> x) : _x(x){};

};


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

auto Euclidean_distance(const problem::Location& loc_a, const problem::Location& loc_b)
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
                                                 double margin) -> double {
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
    return projection_scaler;
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
/*
 *
 * ***************************Classe BalancedVRPClustering************************************
 *
 */

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
  std::vector<item_with_limit_violation const *> &_items_with_limit_violation;
  std::vector<cluster_with_limit_violation const *> &_clusters_with_limit_violation;
  int _manage_empty_clusters_iterations;

  BalancedVRPClustering();
  BalancedVRPClustering( problem::Problem  &problem, double const cut_ratio,
                        int const cut_index,bool apply_balancing,double total_assigned_cut_load,double percent_assigned_cut_load,
                         bool distance_matrix,double rate_balance, std::vector<double> limit_violation_coefficient,
                         std::vector<item_with_limit_violation const *> &items_with_limit_violation,
                         std::vector<cluster_with_limit_violation const *> &clusters_with_limit_violation, int manage_empty_clusters_iterations);
  ~BalancedVRPClustering();
  auto build(problem::Problem problem, double const cut_ratio, int const cut_index)
      -> void;
  auto populate_centroids(std::string const populate_methode, problem::Problem& problem,
                          int const number_of_cluster) -> void;
  auto exist(int v, const problem::Service service) -> bool;
  auto Check_if_initial_centroids_are_tacken(problem::Problem& problem) -> bool;
  auto check_centroid_compatibility(problem::Problem &problem) -> bool;
  auto calc_initial_centroids(problem::Problem &problem) -> void;
  auto update_metrics(problem::Service * service, problem::Vehicle * vehicle) -> void;
  auto capacity_violation(const problem::Service &service, const problem::Vehicle &vehicle) -> bool;
  auto distance (const problem::Problem &problem, const problem::Service &service, const problem::Vehicle &vehicle) -> double;
  auto evaluate (const problem::Service & service) -> int;
  auto calculate_membership_clusters(problem::Problem & problem) -> void;
  auto manage_empty_clusters(std::string const populate_methode) -> void;
};

BalancedVRPClustering::BalancedVRPClustering( problem::Problem  &problem,
                                             double const cut_ratio, int const cut_index,bool apply_balancing,double total_assigned_cut_load,double percent_assigned_cut_load,
                                              bool distance_matrix, double rate_balance, std::vector<double> limit_violation_coefficient,
                                              std::vector<item_with_limit_violation const *> &items_with_limit_violation,
                                              std::vector<cluster_with_limit_violation const *> &clusters_with_limit_violation, int manage_empty_clusters_iterations ) :
    _problem(problem), _cut_ratio(cut_ratio), _cut_index(cut_index), _apply_balancing(apply_balancing),_total_assigned_cut_load(total_assigned_cut_load),
     _percent_assigned_cut_load(percent_assigned_cut_load),_distance_matrix(distance_matrix), _rate_balance(rate_balance),_limit_violation_coefficient(limit_violation_coefficient),
     _items_with_limit_violation(items_with_limit_violation),_clusters_with_limit_violation(clusters_with_limit_violation),_manage_empty_clusters_iterations(manage_empty_clusters_iterations) {}
BalancedVRPClustering::~BalancedVRPClustering() {
	for(auto item : _items_with_limit_violation)
	{
		delete item;
	}
  for(auto item : _clusters_with_limit_violation)
  {
    delete item;
  }
}
auto BalancedVRPClustering::exist(int v, const problem::Service service) -> bool {
  for (int u = 0; u < service.compatible_vehicle_indices_size(); u++) {
    if (service.compatible_vehicle_indices(u) == v)
      return true;
  }
  return false;
}

auto BalancedVRPClustering::build(problem::Problem problem, double const cut_ratio,
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

auto BalancedVRPClustering::Check_if_initial_centroids_are_tacken(problem::Problem& problem) -> bool {
  for(int i = 0; i < problem.vehicles_size(); ++i){
    auto vehicle = problem.vehicles(i);
    if( vehicle.matrix_index() == 0 && vehicle.centroid().latitude() == 0.0 && vehicle.centroid().longitude() == 0.0) return false;
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

auto BalancedVRPClustering::distance (const problem::Problem &problem, const problem::Service &service, const problem::Vehicle &vehicle) -> double {
  double distance;
  if(_distance_matrix){
      auto matrix = problem.matrices(vehicle.matrix_index()).distance();
      distance = double(matrix.at(vehicle.matrix_index() * sqrt(matrix.size()) + service.matrix_index()));
  }
  else{
    distance = flaying_distance(service.location(), vehicle.centroid());
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
auto BalancedVRPClustering::calc_initial_centroids(problem::Problem &problem) -> void {
  if(!Check_if_initial_centroids_are_tacken(problem)){
    populate_centroids("random",problem,problem.vehicles_size());
  }
  else{
    populate_centroids("indices",problem,problem.vehicles_size());
  }


}
auto BalancedVRPClustering::check_centroid_compatibility(problem::Problem &problem) -> bool {
  bool compatible = true;
  for( int v = 0; v < problem.vehicles_size() && compatible; ++v){
    auto vehicle = problem.vehicles(v);
    for( int i = 0; i < problem.services_size(); ++i){
      auto service = problem.services(i);
      if(vehicle.centroid().matrix_index() == service.location().matrix_index() &&
       abs(vehicle.centroid().latitude() - service.location().latitude()) < 1e-5 &&
       abs(vehicle.centroid().longitude() - service.location().longitude()) < 1e-5 ){
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
                                               problem::Problem& problem,
                                               int const number_of_cluster) -> void {
  /* Generate centroids based on remaining_skills available
   * Similarly with data_items, each centroid id defined by:
   * Latitude, Longitude
   * item_id
   * unit_fullfillment -> for each unit, quantity contained in corresponding cluster
   * Characterisits -> {v_id: skills: days: matrix_index}
   */
  try {
    if (problem.vehicles_size() == 0) {
      throw InvalidMoveException("Error: \t No vehicles provided");
    }
    if (populate_methode == "random") {
      std::srand(std::time(nullptr)); // use current time as seed for random generator
      std::vector<int> disponible_services(problem.services_size(), 0); // ALL indicies
      for (int i = 0; i < problem.services_size(); ++i) {
        disponible_services[i] = i;
      }
      for (int v = 0; v < problem.vehicles_size(); ++v) {
        // problem::Vehicle* vehicle = new problem::Vehicle(problem.vehicles(v));
        auto vehicle = problem.mutable_vehicles(v);
        std::vector<int> compatible_services;
        // Find the items which are not already used, and specifically need the skill set
        // of this cluster
        for (auto i : disponible_services) {
          if (problem.services(i).compatible_vehicle_indices_size() == 1 &&
              problem.services(i).compatible_vehicle_indices(0) == v) {
            compatible_services.push_back(i);
          }
        }
        if (compatible_services.empty()) {
          // If there are no items which specifically needs these skills,
          // then find all the items that can be assigned to this cluster
          for (auto i : disponible_services) {
            if (exist(v, problem.services(i))) {
              compatible_services.push_back(i);
            }
          }
        }

        int service_index = compatible_services[std::rand() % compatible_services.size()];
        disponible_services.erase(std::remove(disponible_services.begin(),
                                              disponible_services.end(), service_index),
                                  disponible_services.end());

        problem::Service* centroid =
            new problem::Service(problem.services(service_index));
        vehicle->mutable_centroid()->set_latitude(centroid->location().latitude());
        vehicle->mutable_centroid()->set_longitude(centroid->location().longitude());
        vehicle->mutable_centroid()->set_matrix_index(
            centroid->location().matrix_index());
      }
    }
    if (populate_methode == "indices") {
      auto it = std::unique(
          problem.mutable_vehicles()->begin(), problem.mutable_vehicles()->end(),
          [](const problem::Vehicle& first, const problem::Vehicle& sec) {
            if (first.centroid().matrix_index() == sec.centroid().matrix_index())
              return true;
            else
              return false;
          });
      for( int i = 0; i < problem.vehicles_size(); ++i){
        auto vehicle = problem.vehicles(i);
        std::cout << " limitation vehicle " << vehicle.metric_limit() << std::endl;
      }
      if (it != problem.mutable_vehicles()->end()) {
        throw InvalidMoveException(
            "Error: \t Same centroid_index provided several times");
      }
      if(!Check_if_initial_centroids_are_tacken(problem)){
        throw InvalidMoveException(
            "Error: \t Wrong number of initial centroids provided");
      }
      if(!check_centroid_compatibility(problem)){
        throw InvalidMoveException(
            "Error: \t Centroid  is initialised with  incompatible services");
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
      distances[i] = distance(_problem,service,vehicle);
    }
    else {
      distances[i] = pow(2,32);
    }
  }

  int closest_cluster_index = std::min_element(distances.begin(),distances.end()) - distances.begin(); // algorithm header
  if(capacity_violation(service, _problem.vehicles(closest_cluster_index))){
    double minimum_without_limit_violation = pow(2,32); // onsider only compatible ones
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
      std::vector<int> vec = _clusters_with_limit_violation[closest_cluster_index]->_x;
      vec.push_back(closest_cluster_wo_violation_index);
      _clusters_with_limit_violation[closest_cluster_index] = new cluster_with_limit_violation(vec);
      closest_cluster_index = closest_cluster_wo_violation_index;
    }
  }
  std::cout << " name " << service.name() << " closes_cluster_index " << closest_cluster_index << std::endl;
  return closest_cluster_index;

}

auto BalancedVRPClustering::manage_empty_clusters(std::string const populate_methode) -> void {
  _manage_empty_clusters_iterations +=1;
  if( populate_methode == "terminate"){
     // Do nothing to terminate with error. (The empty cluster will be assigned a nil centroid, and then calculating the distance from this centroid to another point will raise an exception.)
  }

}

auto BalancedVRPClustering::calculate_membership_clusters(problem::Problem & problem) -> void {
  for( int i = 0; i < problem.services_size(); ++i){
    auto service = problem.services(i);
    int cluster_index = evaluate(service);
    auto vehicle = problem.mutable_vehicles(cluster_index);
    vehicle->add_assigned_service_indices(i);
    problem::Service * _service = new problem::Service (service);
    update_metrics(_service,vehicle);
  }
}
