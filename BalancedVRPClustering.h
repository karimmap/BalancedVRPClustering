#pragma once

#include<cstddef>
#include<vector>

#include "problem.pb.h"

#include <math.h>       /* cos */

#define PI 3.14159265

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

auto Compute_limits(problem::Problem& problem, double cut_ratio) -> void
{
    for( int u = 0; u < problem.services(0).unit_labels_size();++u){
        problem.add_unit_labels(problem.services(0).unit_labels(u));
    }


    for(int i = 0; i < problem.unit_labels_size(); ++i){
        std::cout << problem.unit_labels(i) << std::endl;
    }
}
//  def compute_limits(cut_symbol, cut_ratio, vehicles_infos, data_items, entity = :vehicle)
//     cumulated_metrics = Hash.new(0)

//     (@unit_symbols || [cut_symbol]).each{ |unit|
//       cumulated_metrics[unit] = data_items.collect{ |item| item[3][unit] || 0 }.reduce(&:+)
//     }

//     strict_limits = if vehicles_infos.none?{ |v_i| v_i[:capacities] }
//       []
//     else
//       vehicles_infos.collect{ |cluster|
//         s_l = { duration: cluster[:total_work_time], visits: cumulated_metrics[:visits] }
//         cumulated_metrics.each{ |unit, _total_metric|
//           s_l[unit] = ((cluster[:capacities].has_key? unit) ? cluster[:capacities][unit] : 0)
//         }
//         s_l
//       }
//     end

//     total_work_time = vehicles_infos.map{ |cluster| cluster[:total_work_time] }.reduce(&:+).to_f
//     metric_limits = if entity == :vehicle && total_work_time.positive?
//       vehicles_infos.collect{ |cluster|
//         { limit: cut_ratio * (cumulated_metrics[cut_symbol].to_f * (cluster[:total_work_time] / total_work_time)) }
//       }
//     else
//       { limit: cut_ratio * (cumulated_metrics[cut_symbol] / vehicles_infos.size) }
//     end

//     [strict_limits, metric_limits]
//   end
// end
