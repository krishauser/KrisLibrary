#ifndef STATISTICS_DATABASE_H
#define STATISTICS_DATABASE_H

#include <KrisLibrary/utils/NamedTree.h>
#include "DistributionCollector.h"

namespace Statistics {

/** @ingroup Statistics
 * @brief A stat collector that may maintain data points as well as general
 * statistics.
 * 
 * To retain a data point, call record() rather than collect()
 */
struct SuperStatCollector : public DistributionCollectorND
{
  void record(const Vector& x) { collect(x); data.push_back(x); }
  void record(Real x1) { Vector x(1); x(0)=x1; record(x); }
  void record(Real x1,Real x2) { Vector x(2); x(0)=x1; x(1)=x2; record(x); }
  void record(Real x1,Real x2,Real x3) { Vector x(3); x(0)=x1; x(1)=x2; x(2)=x3; record(x); }

  vector<Vector> data;
};

struct StatDatabase : public NamedTree<SuperStatCollector>
{
  void collect(const std::string& name,const Vector& x) { Insert(name).collect(x); }
  void collect(const std::string& name,Real x1) { Insert(name).collect(x1); }
  void collect(const std::string& name,Real x1,Real x2) { Insert(name).collect(x1,x2); }
  void collect(const std::string& name,Real x1,Real x2,Real x3) { Insert(name).collect(x1,x2,x3); }

  void collect(const std::string& s1,const std::string& s2,const Vector& x) { Insert(s1).Insert(s2).collect(x); }
  void collect(const std::string& s1,const std::string& s2,Real x1) { Insert(s1).Insert(s2).collect(x1); }
  void collect(const std::string& s1,const std::string& s2,Real x1,Real x2) { Insert(s1).Insert(s2).collect(x1,x2); }
  void collect(const std::string& s1,const std::string& s2,Real x1,Real x2,Real x3) { Insert(s1).Insert(s2).collect(x1,x2,x3); }

  void collect(const std::string& s1,const std::string& s2,const std::string& s3,const Vector& x) { Insert(s1).Insert(s2).Insert(s3).collect(x); }
  void collect(const std::string& s1,const std::string& s2,const std::string& s3,Real x1) { Insert(s1).Insert(s2).Insert(s3).collect(x1); }
  void collect(const std::string& s1,const std::string& s2,const std::string& s3,Real x1,Real x2) { Insert(s1).Insert(s2).Insert(s3).collect(x1,x2); }
  void collect(const std::string& s1,const std::string& s2,const std::string& s3,Real x1,Real x2,Real x3) { Insert(s1).Insert(s2).Insert(s3).collect(x1,x2,x3); }

  void record(const std::string& name,const Vector& x) { Insert(name).record(x); }
  void record(const std::string& name,Real x1) { Insert(name).record(x1); }
  void record(const std::string& name,Real x1,Real x2) { Insert(name).record(x1,x2); }
  void record(const std::string& name,Real x1,Real x2,Real x3) { Insert(name).record(x1,x2,x3); }

  void record(const std::string& s1,const std::string& s2,const Vector& x) { Insert(s1).Insert(s2).record(x); }
  void record(const std::string& s1,const std::string& s2,Real x1) { Insert(s1).Insert(s2).record(x1); }
  void record(const std::string& s1,const std::string& s2,Real x1,Real x2) { Insert(s1).Insert(s2).record(x1,x2); }
  void record(const std::string& s1,const std::string& s2,Real x1,Real x2,Real x3) { Insert(s1).Insert(s2).record(x1,x2,x3); }


  void record(const std::string& s1,const std::string& s2, const std::string& s3,const Vector& x) { Insert(s1).Insert(s2).Insert(s3).record(x); }
  void record(const std::string& s1,const std::string& s2, const std::string& s3,Real x1) { Insert(s1).Insert(s2).Insert(s3).record(x1); }
  void record(const std::string& s1,const std::string& s2, const std::string& s3,Real x1,Real x2) { Insert(s1).Insert(s2).Insert(s3).record(x1,x2); }
  void record(const std::string& s1,const std::string& s2, const std::string& s3,Real x1,Real x2,Real x3) { Insert(s1).Insert(s2).Insert(s3).record(x1,x2,x3); }
};

#endif
