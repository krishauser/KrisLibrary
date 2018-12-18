#ifndef ROBOTICS_MULTI_MODAL_CSPACE_H
#define ROBOTICS_MULTI_MODAL_CSPACE_H

#include "CSpace.h"
#include <KrisLibrary/graph/UndirectedGraph.h>
#include <KrisLibrary/errors.h>
#include <vector>

/** @ingroup MotionPlanning
 * @brief Multi-modal configuration space base class.
 */
template<class Mode>
class MultiModalCSpace
{
 public:
  virtual ~MultiModalCSpace() {}
  //must be overloaded
  virtual bool IsValid(const Mode& m) { return true; }
  virtual CSpace* GetModeCSpace(const Mode& m) { return NULL; }
  virtual CSpace* GetTransitionCSpace(const Mode& m1,const Mode& m2) { return NULL; }

  //getting modes
  virtual bool CanEnum() const { return false; }
  virtual bool CanSample() const { return false; }
  virtual void Enum(std::vector<Mode>& modes) { FatalError("MultiModalCSpace: Cannot enumerate all modes"); }
  virtual void Sample(std::vector<Mode>& modes) { FatalError("MultiModalCSpace: Cannot sample modes"); }

  //getting/testing adjacencies
  virtual bool CanEnumAdjacent() const { return false; }
  virtual bool CanSampleAdjacent() const { return false; }
  virtual bool CanTestAdjacent() const { return false; }
  virtual void EnumAdjacent(const Mode& m,std::vector<Mode>& modes) { FatalError("MultiModalCSpace: Cannot enumerate adjacent modes"); }
  virtual void SampleAdjacent(const Mode& m,std::vector<Mode>& adj) { FatalError("MultiModalCSpace: Cannot sample adjacent modes"); }
  virtual bool TestAdjacent(const Mode& m1,const Mode& m2) { FatalError("MultiModalCSpace: Cannot test mode adjacency"); return false; }
};

class ExplicitMMCSpace : public MultiModalCSpace<int>
{
 public:
  typedef int Mode;
  typedef Graph::UndirectedGraph<CSpace*,CSpace*> ModeGraph;

  virtual ~ExplicitMMCSpace() {}
  void DeleteAll();

  //must be overloaded
  virtual bool IsValid(const Mode& m) { return m >= 0 && m < modeGraph.NumNodes(); }
  virtual CSpace* GetModeCSpace(const Mode& m) { return modeGraph.nodes[m]; }
  virtual CSpace* GetTransitionCSpace(const Mode& m1,const Mode& m2) { return *modeGraph.FindEdge(m1,m2); }

  //getting modes
  virtual bool CanEnum() const { return true; }
  virtual bool CanSample() const { return true; }
  virtual void Enum(std::vector<Mode>& modes) { modes.resize(modeGraph.nodes.size()); for(size_t i=0;i<modeGraph.nodes.size();i++) modes[i]=i; }
  virtual void Sample(std::vector<Mode>& modes) { Enum(modes); }

  //getting/testing adjacencies
  virtual bool CanEnumAdjacent() const { return true; }
  virtual bool CanSampleAdjacent() const { return true; }
  virtual bool CanTestAdjacent() const { return true; }
  virtual void EnumAdjacent(const Mode& m,std::vector<Mode>& modes) {
    ModeGraph::Iterator e;
    modes.resize(0);
    for(modeGraph.Begin(m,e);!e.end();++e) modes.push_back(e.target());
  }
  virtual void SampleAdjacent(const Mode& m,std::vector<Mode>& adj) { EnumAdjacent(m,adj); }
  virtual bool TestAdjacent(const Mode& m1,const Mode& m2) { return modeGraph.HasEdge(m1,m2); }

  //must fill out this graph beforehand
  ModeGraph modeGraph;
};


#endif
