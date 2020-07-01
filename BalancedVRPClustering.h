#pragma once

#include<cstddef>
#include<vector>

#include "problem.pb.h"

#include <math.h>       /* cos */

#include "InvalidMoveException.h"
#include<list>

#define PI 3.14159265

double Max = std::numeric_limits<double>::max();

auto compatibleVehicle(problem::Problem& problem) -> void
{
  for (int i = 0; i < problem.services_size(); ++i) {
    auto service = problem.mutable_services(i);
    for (int k = 0; k < problem.vehicles_size(); ++k) {
      auto vehicle_add = problem.mutable_vehicles(k);
      bool compatible = true;
      auto vehicle = problem.vehicles(k);
      for (int u = 0; u < service->quantities_size() && compatible; ++u) {
            auto quantity = service->quantities(u);
            auto capacity = vehicle.capacities(u);
            if (quantity > capacity.limit() && capacity.limit() != -1 ) {
                compatible = false;
            }
       }
      if(service->skills_size() - vehicle.skills_size() > 0){
        compatible = false;
      }
        for(int s = 0; s < service->skills_size() && compatible; ++s) {
          for(int sv = 0; sv < vehicle.skills_size() && compatible; ++sv){
              if (service->skills(s) != vehicle.skills(s)){
                compatible = false;
              }
           }
        }

      if (compatible) {
        service->add_compatible_vehicle_indices(k);
      }
    }
  }
}

auto Euclidean_distance(const problem::Location &loc_a,const problem::Location &loc_b) -> double
{
   double delta_lat = loc_a.latitude() - loc_b.latitude();
   double delta_lon = (loc_a.longitude() - loc_b.longitude()) * cos((loc_a.latitude() - loc_b.latitude()) * PI / 360.0); //Correct the length of a lon difference with cosine of avereage latitude

   return 111321 * sqrt(pow(delta_lat,2) + pow(delta_lon,2)); //111321 is the length of a degree (of lon and lat) in meters
}

auto Compute_distance_from_and_to_depot(problem::Problem problem) -> std::vector<double>
{
  std::vector<double> duration_from_and_to_depot(problem.services_size());
  std::vector<double> time_matrix_from_depot(problem.services_size());
  std::vector<double> time_matrix_to_depot(problem.services_size());
  for(auto k:problem.vehicles()){
      if( k.matrix_index() >= 0){
         auto matrix = problem.matrices(k.matrix_index()).time();
         for(int i = 0; i < problem.services_size(); ++i){
            auto service = problem.services(i);
            time_matrix_from_depot[i] = double(matrix.at(service.matrix_index()));
            time_matrix_to_depot[i] = matrix.at(service.matrix_index() * sqrt(matrix.size()));
         }
      }
      else {
         for(int i = 0; i < problem.services_size(); ++i){
           auto service = problem.services(i);
            time_matrix_from_depot[i] = Euclidean_distance(k.start_location(),service.location());
            time_matrix_to_depot[i] = Euclidean_distance(service.location(),k.start_location());
         }
      }
  }

  for(int i = 0; i < problem.services_size(); ++i){
      duration_from_and_to_depot[i] = time_matrix_from_depot[i] + time_matrix_to_depot[i];
  }
  return duration_from_and_to_depot;
}

auto none(problem::Problem problem) -> bool
{
    bool is_empty = true;
    for( auto i:problem.vehicles()){
        if(!(i.capacities().empty())) {
            is_empty = false;
        }
        break;
    }
    return is_empty;
}

auto Compute_limits(int cut_index,problem::Problem &problem, double cut_ratio,std::vector<double> &cumulated_metrics ) -> void
{
    for( int u = 0; u < problem.services(0).unit_labels_size();++u){
        auto add = problem.mutable_unit_labels();
        problem.add_unit_labels(problem.services(0).unit_labels(u));
    }

    cumulated_metrics.resize(problem.unit_labels_size());

    for(int u = 0; u < problem.services(0).quantities_size(); ++u){
        for(auto i:problem.services()){
            cumulated_metrics[u] +=  i.quantities(u);
        }
    }

    double total_work_time = 0.0;
    for(auto k: problem.vehicles()){
        total_work_time += k.duration();
    }
    for(int i = 0; i < problem.vehicles_size(); ++i){
      auto vehicle = problem.mutable_vehicles(i);
      if( cut_index > problem.unit_labels_size()){
        vehicle->set_metric_limit(cut_ratio * vehicle->duration() / total_work_time);
      }
      else {
        vehicle->set_metric_limit(cut_ratio * cumulated_metrics[cut_index] / problem.vehicles_size());
    }   }
}

auto flaying_distance(const problem::Location &loc_a,const problem::Location &loc_b) -> double
{
  if(!(loc_a.latitude())  && !(loc_b.latitude()) ) return 0.0;
  if(abs(loc_a.latitude() - loc_b.latitude()) < 30 && (std::max(loc_a.latitude(),loc_b.latitude()) +abs(loc_a.longitude() - loc_b.longitude())) < 100) {
  //       These limits ensures that relative error cannot be much greather than 2%
  //       For a distance like Bordeaux - Berlin, relative error between
  //       euclidean_distance and flying_distance is 0.1%.
  //       That is no need for trigonometric calculation.
  return Euclidean_distance(loc_a,loc_b);
  }
  double r = 6378137; // Earth's radius in meters
  double deg2rad_lat_a = loc_a.latitude() * PI / 180;
  double deg2rad_lat_b = loc_b.latitude() * PI / 180;
  double deg2rad_lon_a = loc_a.longitude() * PI / 180;
  double deg2rad_lon_b = loc_b.longitude() * PI / 180;
  double lat_distance = deg2rad_lat_b - deg2rad_lat_a;
  double lon_distance = deg2rad_lon_b - deg2rad_lon_a;
  double intermediate = pow(sin(lat_distance / 2),2) + cos(deg2rad_lat_a) * cos(deg2rad_lat_b) * pow(sin(lon_distance / 2),2);
  return r * 2 * atan2 (sqrt(intermediate),sqrt( 1 - intermediate));
}

auto check_if_projection_inside_the_line_segment(const problem::Location &point_coord,const problem::Location &line_beg_coord,const problem::Location &line_end_coord, double margin ) -> double
{
  /*
   * margin: if (0 > margin > 1), the percentage of the line segment that will be considered as "outside"
   * margin: if (margin < 0), the percentage that the "inside" zone is extended
   * [0, 1]: coordinates [lat, lon] or [lon, lat]
   */
   std::pair<double,double> line_direction;
   line_direction.first = line_end_coord.latitude() - line_beg_coord.latitude();
   line_direction.second = line_end_coord.longitude() - line_beg_coord.longitude();

   std::pair<double,double> point_direction;
   point_direction.first = point_coord.latitude() - line_beg_coord.latitude();
   point_direction.second = point_coord.longitude() - line_beg_coord.longitude();

   std::pair<double,double> projection_scaler_coord;
   projection_scaler_coord.first = line_direction.first * point_direction.first / pow(line_direction.first,2);
   projection_scaler_coord.second = line_direction.second * point_direction.second / pow(line_direction.second,2);
   /*
   * If projection_scaler
   * >1:        the projection is after the line segment end
   * <0:        it is before the line segment begin
   * otherwise: it is on the line segment define by begin_coord end end_coord
   */
   double projection_scaler = projection_scaler_coord.first + projection_scaler_coord.second;
   // If the following holds, it is inside the margin of the line segment
   if(projection_scaler >= 0.5 * margin && projection_scaler <= 1 - 0.5 * margin) return projection_scaler;


}

auto item_nil(const problem::Problem &problem) -> bool
{
  for(auto i:problem.services()){
    if(i.quantities_size() == 0) return true;
  }
  return false;
}

auto Check_vehicle_location(const problem::Problem &problem) -> bool
{
  for(auto i:problem.vehicles()){
    if(i.start_location().latitude()  || i.start_location().longitude() ) return false;
  }
  return true;
}

auto Check_service_location(const problem::Problem &problem) -> bool
{
  for(auto i:problem.services()){
    if(i.location().latitude() || i.location().longitude() ) return false;
  }
  return true;
}
/*
 *
 * ***************************Classe BalancedVRPClustering************************************
 *
 */

class BalancedVRPClustering
{
  public:
    problem::Problem const _problem;
    double const _cut_ratio;
    int const _cut_index;

    BalancedVRPClustering();
    BalancedVRPClustering(problem::Problem const problem, double const cut_ratio, int const cut_index);
    auto build(problem::Problem  problem, double const cut_ratio, int const cut_index) -> void;
    auto populate_centroids(std::string const populate_methode,problem::Problem const problem, int const number_of_cluster) -> void;
};

BalancedVRPClustering::BalancedVRPClustering(problem::Problem const problem, double const cut_ratio, int const cut_index):_problem(problem),_cut_ratio(cut_ratio),_cut_index(cut_index){}

auto BalancedVRPClustering::build(problem::Problem  problem, double const cut_ratio, int const cut_index) -> void
{
 try{
   //return clean errors if unconsistent data
  if(item_nil(problem)){
    throw InvalidMoveException("Error: \t unit should be provided for all item");
  }
  else if(Check_vehicle_location(problem) ){
    throw InvalidMoveException("Location info (lattitude and longitude) should be provided for all vehicles");
  }
  else if(Check_service_location(problem)){
    throw InvalidMoveException("Location info (lattitude and longitude) should be provided for all items");
  }

  int max_iteration;
  if(problem.options().max_iteration() != 0) max_iteration = problem.options().max_iteration();
  else max_iteration = std::max(0.5 * problem.services_size(), 100.0);

  std::vector<double> distance_from_and_to_depot;
  if( cut_index > 3 || cut_index < 0) distance_from_and_to_depot = Compute_distance_from_and_to_depot(problem); // case: duration
  std::vector<double> cumulated_metrics;
  Compute_limits(cut_index,problem, cut_ratio,cumulated_metrics);

  // *** Algo start ****//
  int iteration = 0;
  std::vector<int> i_like_to_move_it_move_it;
  int moved_up = 0;
  int moved_down = 0;
  std::vector<double>cluster_with_capacity_violation;
  if(cut_index <= 3 && cut_index >= 0){
    if(cumulated_metrics[cut_index] == 0.0){
      std::cout << "Disable balancing because there is no point" << std::endl;
    }
    else{
      if( cut_index < problem.unit_labels_size()){
        std::sort(problem.mutable_services()->begin(), problem.mutable_services()->end(), [ ](const problem::Service & service1 , const problem::Service & service2){
          if( service1.quantities(0) > service2.quantities(0))
           return true;
        });
      }
        std::random_shuffle(problem.mutable_services()->begin() + int(problem.services_size() * 0.1), problem.mutable_services()->begin() + int(problem.services_size() * 0.9));
    }
  }

 }catch(exception &e){
   std::cout << e.what() << std::endl;
 }
}

auto BalancedVRPClustering::populate_centroids(std::string const populate_methode,problem::Problem const problem, int const number_of_cluster) -> void
{
  /* Generate centroids based on remaining_skills available
   * Similarly with data_items, each centroid id defined by:
   * Latitude, Longitude
   * item_id
   * unit_fullfillment -> for each unit, quantity contained in corresponding cluster
   * Characterisits -> {v_id: skills: days: matrix_index}
   */
  try{
   if(problem.vehicles_size() == 0){
      throw InvalidMoveException("Error: \t No vehicles provided");
   }
   if(populate_methode == "rondom"){
     for(int v = 0; v < problem.vehicles_size(); ++v){
       auto vehicle = problem.vehicles(v);
       
     }
   }







   }catch(exception &e){
   std::cout << e.what() << std::endl;
   }

}

// def populate_centroids(populate_method, number_of_clusters = @number_of_clusters)
//         # Generate centroids based on remaining_skills available
//         # Similarly with data_items, each centroid is defined by :
//         #    index 0 : latitude
//         #    index 1 : longitude
//         #    index 2 : item_id
//         #    index 3 : unit_fullfillment -> for each unit, quantity contained in corresponding cluster
//         #    index 4 : characterisits -> { v_id: sticky_vehicle_ids, skills: skills, days: day_skills, matrix_index: matrix_index }
//         raise ArgumentError, 'No vehicles_infos provided' if @remaining_skills.nil?
//         raise ArgumentError, 'No vehicles provided' if @remaining_skills.nil?
