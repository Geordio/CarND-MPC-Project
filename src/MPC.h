#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

#include <vector>
#include <string>
#include <fstream>
#include <iostream>
using namespace std;


using namespace std;

//
//string debugFilename_;
//ofstream debugFile_;
//int count;

///*
//* Sets up a csv file for debug information
//*/
//void SetupCsv();

class MPC {
 public:
  MPC();


  virtual ~MPC();



  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);


};

#endif /* MPC_H */
