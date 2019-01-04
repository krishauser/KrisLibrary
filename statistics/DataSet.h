#ifndef STAT_DATA_SET_H
#define STAT_DATA_SET_H

#include "statistics.h"
#include <string>

namespace Statistics {

/** @addtogroup Statistics
 * @brief A set of vector-valued observations.
 *
 * Internally represented as a Matrix.
 */
class DataSet
{
 public:
  inline Real& operator()(int i,int j) { return data(i,j); }
  inline const Real& operator()(int i,int j) const { return data(i,j); }

  ///outputs the observations into the vector<Vector> form.
  ///Note: (Vector's are references!)
  void GetObservations(std::vector<Vector>& output);
  void SetObservations(const std::vector<Vector>& input);

  ///sets the ith observation to the input vector
  void SetObservation(int i,std::vector<Real>& in);
  void SetObservation(int i,Vector& in);

  ///outputs the ith observation into the output vector
  void GetObservation(int i,std::vector<Real>& out);
  void GetObservation(int i,Vector& out);
  
  ///sets the jth element of data to the input vector
  void SetElement(int j,std::vector<Real>& in);
  void SetElement(int j,Vector& in);

  ///outputs the jth element of data into the output vector
  void GetElement(int j,std::vector<Real>& out);
  void GetElement(int j,Vector& out);

  std::vector<std::string> labels;
  Matrix data;
};

}

#endif





