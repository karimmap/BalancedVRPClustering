
#include <string>
#include <iostream>
#include <fstream>
#include<sstream>
#include <bits/stdc++.h>


#include "concave.h"

#include "problem.pb.h"

#include "ext-lib/json.h"

#include"BalancedVRPClustering.h"

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


      for( auto matrix: problem.matrices()){
         cout << " time.size() "<<matrix.time_size()<<endl;
      }

    }
  void readColor( vector < string > &color, string const & filePath ){
   cout << "<read> Read the data " << endl;
   ifstream f(filePath.c_str());
   if (!f.good()){
     cerr << "<read> Error: Could not open file " << filePath << "." << endl;
     exit(EXIT_FAILURE);
   }
   color.resize(20);
   for( int i = 0; i < 20; ++i){
     f >> color[i];
   }
   f.close();
  }

  void run()
  {
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
    cout << endl; cout << endl;
    for (auto service : problem.vehicles())
      cout << service.ShortDebugString() << endl;
    double cut_ratio = 0.9;
    double dist = Euclidean_distance(problem.services(0).location(), problem.services(1).location());
    double dist1 = flaying_distance(problem.services(0).location(), problem.services(1).location());
    cout << " dist " << dist << " " << dist1 << endl;
    vector<double> cumulated_metrics;
    Compute_limits(0,problem,cut_ratio,cumulated_metrics);
    for(int i = 0; i < cumulated_metrics.size(); ++i) cout << " cumulated metric " << cumulated_metrics[i] << endl;
    for(auto i:problem.vehicles()){
      cout << " i limit "<< i.metric_limit() << endl;
    }
    double projection_scaler = check_if_projection_inside_the_line_segment(problem.services(0).location(),problem.services(1).location(),problem.services(5).location(), 0.8 );
    cout << " scaler " << projection_scaler << endl;
    BalancedVRPClustering pro(problem,cut_ratio,0);
    pro.build(problem,cut_ratio,0);
    std::string  populate_methode;
    pro.populate_centroids(populate_methode, problem,2);
  }
} // namespace operations_research

int main(int argc, char* argv[])
{
  operations_research::run();

  return EXIT_SUCCESS;

}