#ifndef UTILS_STAT_COLLECTOR_H
#define UTILS_STAT_COLLECTOR_H

#include <KrisLibrary/Logger.h>
#include <KrisLibrary/utils.h>
#include <math.h>
#include <ostream>
#include <map>
#include <vector>
#include <string>

/** @ingroup Utils
 * @brief Collects statistics (min,max,mean,stddev,etc) on
 * floating-point data.
 *
 * Collects datapoints incrementally and keeps statistics in a constant-sized
 * structure.
 *
 * There is a risk of numerical overflow and precision errors in the stddev
 * and variance methods, when collecting many large values.
 *
 * Print prints a concise, human-readable representation.  Load and Save can
 * be used for loading/saving to disk.
 *
 * See also optimization/DistributionCollector.h
 */
struct StatCollector
{
  StatCollector();  
  inline void operator << (double x) { collect(x); }
  void collect(double x);
  void weightedCollect(double x,double weight);

  int number() const { return (int)n; }
  double minimum() const { return xmin; }
  double maximum() const { return xmax; }
  double average() const { return sum/n; }
  double variance() const {
    double avg=average();
	return Max(sumsquared/n - avg*avg,0.0);  //may have numerical errors...
  }
  double stddev() const { return sqrt(variance()); }
  void clear();
  
  bool Load(std::istream& in);
  bool Save(std::ostream& out) const;
  void Print(std::ostream& out) const;

  double n;
  double xmin,xmax;
  double sum,sumsquared;
};

/** @ingroup Utils
 * @brief A heirarchical database of statistical data (counts and
 * StatCollectors). Useful for performance testing.
 */
struct StatDatabase
{
  struct Data
  {
    Data() : count(0) {}

    int count;
    StatCollector value;
    std::map<std::string,Data> children;
  };

  StatDatabase();
  void Clear();
  void Increment(const std::string& name,int inc=1);
  void AddValue(const std::string& name,double val);
  int GetCount(const std::string& name) const;
  const StatCollector& GetValue(const std::string& name) const;

  Data& AddData(const std::string& name);
  const Data* GetData(const std::string& name) const;

  bool Load(std::istream& in);
  bool Save(std::ostream& out) const;
  void Print(std::ostream& out) const;

  //convenience functions
  void Increment(const std::string& name,const std::string& child,int inc=1) { Increment(Concat(name,child),inc); }
  void Increment(const std::string& name,const std::string& child1,const std::string& child2,int inc=1) { Increment(Concat(name,child1,child2),inc); }
  void Increment(const std::string& n,const std::string& c1,const std::string& c2,const std::string& c3,int inc=1) { Increment(Concat(n,c1,c2,c3),inc); }
  void AddValue(const std::string& name,const std::string& child,double val) { AddValue(Concat(name,child),val); }
  void AddValue(const std::string& name,const std::string& child1,const std::string& child2,double val) { AddValue(Concat(name,child1,child2),val); }
  void AddValue(const std::string& n,const std::string& c1,const std::string& c2,const std::string& c3,double val) { AddValue(Concat(n,c1,c2,c3),val); }


  //helpers
  void NameToPath(const std::string& name,std::vector<std::string>& path) const;
  std::string Concat(const std::string& s1,const std::string& s2) const;
  std::string Concat(const std::string& s1,const std::string& s2,const std::string& s3) const;
  std::string Concat(const std::string& s1,const std::string& s2,const std::string& s3,const std::string& s4) const;
  std::string Concat(const std::vector<std::string>& s) const;

  unsigned char delim;
  Data root;
};

#endif
