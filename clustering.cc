
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

      cout << "******************************************* vehicles capacity ****************************"<< endl;
      for (auto vehicle : problem.vehicles()) {
         for (auto capacity : vehicle.capacities()) {
            cout << " V[" << vehicle.id() << "]=" << capacity.limit() << " | ";
          }
      cout << endl;
      }
      for( auto matrix: problem.matrices()){
         cout << " time.size() "<<matrix.time_size()<<endl;
      }
      cout << " ********************************** a vector of quantities for each sevice *************************************************"<< endl;
      for (auto service : problem.services()) {
         for (int i = 0; i < service.quantities_size(); ++i){
            cout << " D[" << i << "]=" << service.quantities(i) << " | ";
            cout << endl;
         }
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
    // for (auto service : problem.services())
    //   cout << service.ShortDebugString() << endl;
    // cout << endl; cout << endl;
    // for (auto service : problem.vehicles())
    //   cout << service.ShortDebugString() << endl;
    double cut_ratio = 0.9;
    double dist = Euclidean_distance(problem.services(0).location(), problem.services(1).location());
    double dist1 = flaying_distance(problem.services(0).location(), problem.services(1).location());
    cout << " dist " << dist << " " << dist1 << endl;
    std::vector<Strict_limit* > strict_limits ; std::vector<double> metric_limits;
    Compute_limits("kg",problem,cut_ratio,strict_limits,metric_limits);
    // for(int i = 0; i < strict_limits.size(); ++i){
    //   for(int j = 0; j < strict_limits[i] -> limit.size(); ++j){
    //     cout << "  "<< strict_limits[i] -> limit[j];
    //   } cout << endl;
    //   cout << " metric " << metric_limits[i] << endl;
    // }
  }
} // namespace operations_research

int main(int argc, char* argv[])
{
  operations_research::run();

  return EXIT_SUCCESS;

}