
#include <bits/stdc++.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "concave.h"

#include "problem.pb.h"

#include "ext-lib/json.h"

#include "BalancedVRPClustering.h"

using json = nlohmann::json;

using namespace std;
namespace operations_research {

void printProblem(problem::Problem& problem) {
  for (auto matrix : problem.matrices())
    cout << matrix.ShortDebugString() << endl;

  for (auto vehicle : problem.vehicles())
    cout << vehicle.DebugString() << endl;

  for (auto service : problem.services())
    cout << service.ShortDebugString() << endl;

  for (auto matrix : problem.matrices()) {
    cout << " time.size() " << matrix.time_size() << endl;
  }
}
void readColor(vector<string>& color, string const& filePath) {
  cout << "<read> Read the data " << endl;
  ifstream f(filePath.c_str());
  if (!f.good()) {
    cerr << "<read> Error: Could not open file " << filePath << "." << endl;
    exit(EXIT_FAILURE);
  }
  color.resize(20);
  for (int i = 0; i < 20; ++i) {
    f >> color[i];
  }
  f.close();
}

void run() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  const string& filename = "instance-13.bin";
  problem::Problem problem;
  {
    fstream input(filename, ios::in | ios::binary);
    if (!problem.ParseFromIstream(&input)) {
      cout << "Failed to parse pbf." << endl;
    }
  }

  compatibleVehicle(problem);
  vector<double> duration_from_and_to_depot = Compute_distance_from_and_to_depot(problem);
  // for(auto i:problem.services())
  // {
  //   cout << duration_from_and_to_depot[i.id()] << endl;
  // }
  for (auto service : problem.services())
    cout << service.ShortDebugString() << endl;
  cout << endl;
  cout << endl;
  for (auto service : problem.vehicles())
    cout << service.ShortDebugString() << endl;
  double cut_ratio = 0.9;
  double dist =
      Euclidean_distance(problem.services(0).location(), problem.services(1).location());
  double dist1 =
      flaying_distance(problem.services(0).location(), problem.services(1).location());
  cout << " dist " << dist << " " << dist1 << endl;
  vector<double> cumulated_metrics;
  Compute_limits(0, problem,0.9);
  for (int i = 0; i < cumulated_metrics.size(); ++i)
    cout << " cumulated metric " << cumulated_metrics[i] << endl;
  for (auto i : problem.vehicles()) {
    cout << " i limit " << i.metric_limit() << endl;
  }
  double projection_scaler = check_if_projection_inside_the_line_segment(
      problem.services(0).location(), problem.services(1).location(),
      problem.services(5).location(), 0.8);
  cout << " scaler " << projection_scaler << endl;
  vector<double> limit_violation_coefficient(problem.vehicles_size(),1.0);
  vector<item_with_limit_violation *> items_with_limit_violation;

  BalancedVRPClustering pro(problem, 0.9, 0,false,0.0,1.0,false,0);
  pro.build(problem, 0.9, 0);
  std::string populate_methode;
  pro.populate_centroids("random", 2);
  pro.calc_initial_centroids();
  // problem::Vehicle  * vehicle = problem.mutable_vehicles(0);
  // problem::Service * service = problem.mutable_services(12);
  // pro.update_metrics(service,vehicle);
  for(int i = 0; i < problem.vehicles_size(); ++i){
    auto vehicle = problem.vehicles(i);
    cout << " lat =" << vehicle.initial_centroid().latitude() << " lon =" << vehicle.initial_centroid().longitude() << "mat_index " << vehicle.initial_centroid().matrix_index() << endl;
  }
  // //cout << " check here " << total_assigned_cut_load << " " << percent_assigned_cut_load << endl;
  // // cout << " check capacity violation " << pro.capactity_violation(problem.services(0), 1) << endl;
  // //cout <<" load " << vehicle.load(0) << " " << pro._total_assigned_cut_load  << " " <<  pro._percent_assigned_cut_load<<  endl;
  // auto veh = problem.vehicles(0);
  // for(int i = 0; i < veh.load_size(); ++i){
  //   cout << " load \t " << veh.load(i) << endl;
  // }
  // cout << " capacity violation ->  " << pro.capacity_violation(problem.services(0),vehicle) << endl;
  cout << " here " << pro.distance ( problem.services(0), problem.vehicles(0)) << endl;
  cout << " evaluate "  <<endl;
  cout << " closest cluster index " << pro.evaluate (problem.services(0)) << endl;
  pro.calculate_membership_clusters();
  for(int v = 0; v < problem.vehicles_size(); ++v){
    auto vehicle = problem.vehicles(v);
    for(int i = 0; i < vehicle.assigned_service_indices_size(); ++i){
      cout << vehicle.assigned_service_indices(i) << " ";
    }
    cout << endl;
  }
  cout << "capacity violation " << pro.capacity_violation(problem.services(0), problem.vehicles(0)) << " " <<  problem.vehicles(0).load(0) << endl;
  pro.manage_empty_clusters("terminate");
  for(int v = 0; v < pro._problem.vehicles_size(); v++){
    auto vehicle = pro._problem.vehicles(v);
    cout << vehicle.initial_centroid().latitude() << " " << vehicle.initial_centroid().longitude() << " " << vehicle.initial_centroid().matrix_index() << endl;
  }
    for(int v = 0; v < problem.vehicles_size(); v++){
    auto vehicle = problem.vehicles(v);
    cout << vehicle.initial_centroid().latitude() << " " << vehicle.initial_centroid().longitude() << " " << vehicle.initial_centroid().matrix_index() << endl;
  }
  // pro.eliminate_empty_clusters();
  // pro.manage_empty_clusters("terminate");
  // bool  e = pro.centroids_converged_or_in_loop( 50);
  cout << " size " << pro._items_with_limit_violation.size() << std::endl;
  pro.move_limit_violating_dataitems();
}
} // namespace operations_research

int main(int argc, char* argv[]) {
  operations_research::run();

  return EXIT_SUCCESS;
}