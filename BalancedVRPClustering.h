#pragma once

#include<cstddef>
#include<vector>

#include "problem.pb.h"

#include <math.h>       /* cos */

#define PI 3.14159265

double Max = std::numeric_limits<double>::max();
struct Strict_limit
{
  std::vector<double> limit;
  Strict_limit();
  Strict_limit(std::vector<double> _limit):limit(_limit){};
};

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
       vehicle_add -> set_id(k);
    }
    service -> set_id(i);
  }
}
auto Euclidean_distance(const problem::Location &loc_a,const problem::Location &loc_b) -> double
{
   double delta_lat = loc_a.latitude() - loc_b.latitude();
   double delta_lon = (loc_a.longitude() - loc_b.longitude()) * cos((loc_a.latitude() - loc_b.latitude()) * PI / 360.0); //Correct the length of a lon difference with cosine of avereage latitude

   return 111321 * sqrt(pow(delta_lat,2) + pow(delta_lon,2)); //111321 is the length of a degree (of lon and lat) in meters
}

auto Compute_distance_from_and_to_depot(problem::Problem& problem) -> std::vector<double>
{
  std::vector<double> duration_from_and_to_depot(problem.services_size());
  std::vector<double> time_matrix_from_depot(problem.services_size());
  std::vector<double> time_matrix_to_depot(problem.services_size());
  for(auto k:problem.vehicles()){
      if( k.matrix_index() >= 0){
         auto matrix = problem.matrices(k.matrix_index()).time();
         for(auto i : problem.services()){
            time_matrix_from_depot[i.id()] = double(matrix.at(i.matrix_index()));
            time_matrix_to_depot[i.id()] = matrix.at(i.matrix_index() * sqrt(matrix.size()));
         }
      }
      else {
         for(auto i:problem.services()){
            time_matrix_from_depot[i.id()] = Euclidean_distance(k.start_location(),i.location());
            time_matrix_to_depot[i.id()] = Euclidean_distance(i.location(),k.start_location());
         }
      }
  }

  for(auto i:problem.services()){
      duration_from_and_to_depot[i.id()] = time_matrix_from_depot[i.id()] + time_matrix_to_depot[i.id()];
  }
  return duration_from_and_to_depot;
}

auto none(problem::Problem& problem) -> bool
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
auto get_metric_index(problem::Problem& problem, std::string cut_symbol) -> int
{
  for(int i = 0; i < problem.unit_labels_size(); ++i){
    if(cut_symbol == problem.unit_labels(i)){
      return i;
      break;
    }
  }
  return -1;
}
auto Compute_limits(std::string cut_symbol,problem::Problem& problem, double cut_ratio,std::vector<Strict_limit* > &strict_limits,std::vector<double> &metric_limits ) -> void
{
    for( int u = 0; u < problem.services(0).unit_labels_size();++u){
        auto add = problem.mutable_unit_labels();
        problem.add_unit_labels(problem.services(0).unit_labels(u));
    }

    std::vector<double> cumulated_metrics(problem.unit_labels_size(),0);

    for(int u = 0; u < problem.services(0).quantities_size(); ++u){
        for(auto i:problem.services()){
            cumulated_metrics[u] +=  i.quantities(u);
        }
    }
    strict_limits.resize(problem.vehicles_size());
    if(none(problem)) std::cout << " no vehicle has capacities :-> \t strict_limits is empty " << std::endl;
    else{
        for(auto i:problem.vehicles()){
          std::vector<double> temp;
            for(int u = 0; u < cumulated_metrics.size(); u++){
              double val = i.capacities(u).limit();
              if (val != -1) temp.push_back(val);
              else {
                temp.push_back(Max);
              }
            }
            strict_limits[i.id()] = new Strict_limit(temp);
        }
    }

    double total_work_time = 0.0;
    for(auto k: problem.vehicles()){
        total_work_time += k.duration();
    }
    metric_limits.resize(problem.vehicles_size());
    if(total_work_time > 0.0){
      for(auto i:problem.vehicles()){
        int index = get_metric_index(problem,cut_symbol);
        if(index != -1) metric_limits[i.id()] = cut_ratio * (cumulated_metrics[index] * i.duration()) / total_work_time;
      }
    }
    else{
      for(auto i:problem.vehicles()){
        int index = get_metric_index(problem,cut_symbol);
        if(index != -1) metric_limits[i.id()] = cut_ratio * (cumulated_metrics[index]) / problem.vehicles_size();
      }
    }

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

