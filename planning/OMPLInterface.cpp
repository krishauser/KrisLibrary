#include "OMPLInterface.h"
#include <planning/EdgePlanner.h>
#include <math/random.h>

#if HAVE_OMPL

#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/goals/GoalState.h>
namespace og = ompl::geometric;

///helper: convert a Config to an OMPL state
ob::State * ToOMPL(const ob::StateSpace* si_,const Config& q)
{
  ob::State* omplstate = si_->allocState();
  vector<Real> vq = q;
  si_->copyFromReals(omplstate,vq);
  return omplstate;
}

///helper: convert an OMPL state to a Config
Config FromOMPL(const ob::StateSpace* si_,const ob::State* s)
{
  vector<Real> vq;
  si_->copyToReals(vq,s);
  return Config(vq);
}


///helper: convert a Config to an OMPL state
ob::State * ToOMPL(const ob::SpaceInformation* si_,const Config& q)
{
  ob::State* omplstate = si_->allocState();
  vector<Real> vq = q;
  si_->getStateSpace()->copyFromReals(omplstate,vq);
  return omplstate;
}

///helper: convert an OMPL state to a Config
Config FromOMPL(const ob::SpaceInformation* si_,const ob::State* s)
{
  vector<Real> vq;
  si_->getStateSpace()->copyToReals(vq,s);
  return Config(vq);
}


///helper: convert a Config to an OMPL state
ob::State * ToOMPL(const ob::SpaceInformationPtr& si_,const Config& q)
{
  ob::State* omplstate = si_->allocState();
  vector<Real> vq = q;
  si_->getStateSpace()->copyFromReals(omplstate,vq);
  return omplstate;
}

///helper: convert an OMPL state to a Config
Config FromOMPL(const ob::SpaceInformationPtr& si_,const ob::State* s)
{
  vector<Real> vq;
  si_->getStateSpace()->copyToReals(vq,s);
  return Config(vq);
}


og::PathGeometric* ToOMPL(const ob::SpaceInformationPtr& si,const MilestonePath& path)
{
  og::PathGeometric* res = new og::PathGeometric(si);
  for(int i=0;i<path.NumMilestones();i++) {
    ob::State* s=ToOMPL(si,path.GetMilestone(i));
    res->append(s);
    si->freeState(s);
  }
  return res;
}



OMPLCSpace::OMPLCSpace( const ob::SpaceInformationPtr& si)
{
  nD = si->getStateDimension();
  ob::StateSpacePtr space = si->getStateSpace();
  ob::RealVectorStateSpace* vs = dynamic_cast<ob::RealVectorStateSpace*>(&*space);
  if(vs) {
    qMin.copy(vs->getBounds().low);
    qMax.copy(vs->getBounds().high);
  }
  else {
    qMin.resize(nD);
    qMax.resize(nD);
    qMin.set(0.0);
    qMax.set(1.0);
  }
  resolution = si->getStateValidityCheckingResolution();
  si_ = si;
}

void OMPLCSpace::SetSpaceBounds(const Config& qmin, const Config& qmax) {
	qMin = qmin;
	qMax = qmax;
}

void OMPLCSpace::SetSpaceBounds(const double& qmin, const double& qmax) {
	qMin.set(qmin);
	qMax.set(qmax);
}

void OMPLCSpace::Sample(Config& x) {
	x.resize(nD);
	for(int i = 0; i < nD; i++)
		x[i] = Rand(qMin[i], qMax[i]);
}

ob::State * OMPLCSpace::ToOMPL(const Config& q) const
{
  return ::ToOMPL(si_,q);
}

Config OMPLCSpace::FromOMPL(const ob::State * s) const
{
  return ::FromOMPL(si_,s);
}

bool OMPLCSpace::IsFeasible(const Config& x) {
  //ob::RealVectorStateSpace* rspace = (si_->getStateSpace())->as<ob::RealVectorStateSpace>();
  //assert(rspace != NULL);
  ob::State *omplstate = ToOMPL(x);
  bool res = si_->getStateValidityChecker()->isValid(omplstate);
  si_->freeState(omplstate);
  return res;
}

EdgePlanner* OMPLCSpace::LocalPlanner(const Config& x, const Config& y)
{
  return new StraightLineEpsilonPlanner(this, x, y, resolution);
}


KrisLibraryOMPLPlanner::KrisLibraryOMPLPlanner(const ob::SpaceInformationPtr &si,const MotionPlannerFactory& _factory)
  :ob::Planner(si,_factory.type.c_str()),factory(_factory)
{}

void KrisLibraryOMPLPlanner::clear()
{
  planner = NULL;
  ob::Planner::clear();
}

void KrisLibraryOMPLPlanner::setup()
{
  ob::ProblemDefinitionPtr pdef = this->getProblemDefinition();
  if(pdef == NULL) {
    fprintf(stderr,"KrisLibraryOMPLPlanner::setup(): problem definition not set\n");
    return;
  }
  cspace = new OMPLCSpace(pdef->getSpaceInformation());
  MotionPlanningProblem problem;
  problem.space = cspace;
  Assert(pdef->getStartStateCount() == 1);
  problem.qstart = cspace->FromOMPL(pdef->getStartState(0));
  const ob::GoalState* goalstate = pdef->getGoal()->as<ob::GoalState>();
  if(goalstate)
    problem.qgoal = cspace->FromOMPL(goalstate->getState());
  else {
    FatalError("KrisLibraryOMPLPlanner: TODO: create a goalSet CSpace from an OMPL goal set");
  }
  //TODO: objective functions?
  planner = factory.Create(problem);
  ob::Planner::setup();
}

void KrisLibraryOMPLPlanner::getPlannerData (ob::PlannerData &data) const
{
  if(!planner) return;
  RoadmapPlanner roadmap(const_cast<CSpace*>((const CSpace*)cspace));
  MotionPlannerInterface* iplanner = const_cast<MotionPlannerInterface*>((const MotionPlannerInterface*)planner);
  iplanner->GetRoadmap(roadmap);
  for(size_t i=0;i<roadmap.roadmap.nodes.size();i++) {
    unsigned int id = data.addVertex(ob::PlannerDataVertex(cspace->ToOMPL(roadmap.roadmap.nodes[i]),(int)i));
    Assert(id == i);
  }
  for(size_t i=0;i<roadmap.roadmap.nodes.size();i++) {
    RoadmapPlanner::Roadmap::Iterator e;
    for(roadmap.roadmap.Begin(i,e);!e.end();e++)
      data.addEdge(e.source(),e.target());
  }
}

ob::PlannerStatus KrisLibraryOMPLPlanner::solve (const ob::PlannerTerminationCondition &ptc)
{
  while(!ptc()) {
    if(planner->IsSolved()) {
      //convert solution to OMPL solution
      MilestonePath path;
      planner->GetSolution(path);
      ob::PathPtr pptr(ToOMPL(si_,path));
      this->getProblemDefinition()->addSolutionPath(pptr);
      return ob::PlannerStatus(true,false);
    }
    planner->PlanMore();  
  }
  if(planner->IsSolved()) {
    //convert solution to OMPL solution
    MilestonePath path;
    planner->GetSolution(path);
    ob::PathPtr pptr(ToOMPL(si_,path));
    this->getProblemDefinition()->addSolutionPath(pptr);
    return ob::PlannerStatus(true,false);
  }
  return ob::PlannerStatus(false,false);
}



CSpaceOMPLSpaceInformation::CSpaceOMPLSpaceInformation(CSpace* _space)
  :ob::SpaceInformation(ob::StateSpacePtr(new CSpaceOMPLStateSpace(_space,this))),cspace(_space)
{
  setStateValidityChecker(ob::StateValidityCheckerPtr(new CSpaceOMPLValidityChecker(this)));
  setMotionValidator(ob::MotionValidatorPtr(new CSpaceOMPLMotionValidator(this)));

  setup();
}

ob::State * CSpaceOMPLSpaceInformation::ToOMPL(const Config& q) const
{
  return ::ToOMPL(this,q);
}

Config CSpaceOMPLSpaceInformation::FromOMPL(const ob::State * s) const
{
  return ::FromOMPL(this,s);
}


int NumDimensions(CSpace* s)
{
  Vector x;
  s->Sample(x);
  return x.n;
}

CSpaceOMPLStateSpace::CSpaceOMPLStateSpace(CSpace* _space,CSpaceOMPLSpaceInformation* _si)
  :ob::RealVectorStateSpace(NumDimensions(_space)),space(_space),si(_si)
{
  PropertyMap props;
  space->Properties(props);
  vector<double> minimum,maximum;
  if(props.getArray("minimum",minimum) && props.getArray("maximum",maximum)) {
  }
  else {
    minimum.resize(0);
    maximum.resize(0);
  }
}

unsigned int CSpaceOMPLStateSpace::getDimension (void) const
{
  return NumDimensions(space);
}

double CSpaceOMPLStateSpace::getMaximumExtent (void) const
{
  if(!minimum.empty()) {
    Real extent = 0;
    for(size_t i=0;i<minimum.size();i++)
      extent = Max(extent,Abs(minimum[i]));
    for(size_t i=0;i<maximum.size();i++)
      extent = Max(extent,Abs(maximum[i]));
    return extent;
  }
  return Inf;
}

void CSpaceOMPLStateSpace::enforceBounds (ob::State *state) const
{
  if(!minimum.empty()) {
    vector<Real> q;
    this->copyToReals(q,state);
    if(q.size() != minimum.size()) {
      fprintf(stderr,"CSpaceOMPLStateSpace::enforceBounds: incorrect size of configuration?\n");
      return;
    }
    for(size_t i=0;i<q.size();i++)
      q[i] = Clamp(q[i],minimum[i],maximum[i]);
    this->copyFromReals(state,q);
  }
}

bool CSpaceOMPLStateSpace::satisfiesBounds (const ob::State *state) const
{
  if(!minimum.empty()) {
    vector<Real> q;
    this->copyToReals(q,state);
    if(q.size() != minimum.size()) {
      fprintf(stderr,"CSpaceOMPLStateSpace::satisfiesBounds: incorrect size of configuration?\n");
      return true;
    }
    for(size_t i=0;i<q.size();i++)
      if(q[i] != Clamp(q[i],minimum[i],maximum[i])) return false;
  }
  return true;
}

double CSpaceOMPLStateSpace::distance (const ob::State *state1, const ob::State *state2) const
{
  return space->Distance(FromOMPL(this,state1),FromOMPL(this,state2));
}

void CSpaceOMPLStateSpace::interpolate (const ob::State *from, const ob::State *to, const double t, ob::State *state) const
{
  Vector x;
  space->Interpolate(FromOMPL(this,from),FromOMPL(this,to),t,x);
  this->copyFromReals(state,x);
}

ob::StateSamplerPtr CSpaceOMPLStateSpace::allocDefaultStateSampler (void) const
{
  return ob::StateSamplerPtr(new CSpaceOMPLStateSampler(this));
}


CSpaceOMPLStateSampler::CSpaceOMPLStateSampler(const CSpaceOMPLStateSpace* _space)
  :ob::StateSampler(_space),space(_space->space)
{}

void CSpaceOMPLStateSampler::sampleUniform (ob::State *state)
{
  Vector x;
  space->Sample(x);
  space_->copyFromReals(state,x);
}

void CSpaceOMPLStateSampler::sampleUniformNear (ob::State *state, const ob::State *near, const double distance)
{
  Config c = FromOMPL(space_,near);
  Vector x;
  space->SampleNeighborhood(c,distance,x);
  space_->copyFromReals(state,x);
}

void CSpaceOMPLStateSampler::sampleGaussian (ob::State *state, const ob::State *mean, const double stdDev)
{
  sampleUniformNear(state,mean,stdDev*2);
}

CSpaceOMPLValidityChecker::CSpaceOMPLValidityChecker(CSpaceOMPLSpaceInformation* _space)
  :ob::StateValidityChecker(_space),space(_space->cspace)
{
  specs_.clearanceComputationType = ob::StateValidityCheckerSpecs::NONE;
  specs_.hasValidDirectionComputation = false;
}

bool CSpaceOMPLValidityChecker::isValid(const ob::State* state) const
{
  return space->IsFeasible(FromOMPL(si_,state));
}

CSpaceOMPLMotionValidator::CSpaceOMPLMotionValidator(CSpaceOMPLSpaceInformation* _space)
  :ob::MotionValidator(_space),space(_space->cspace)
{
}

bool CSpaceOMPLMotionValidator::checkMotion (const ob::State *s1, const ob::State *s2) const
{
  EdgePlanner* e=space->LocalPlanner(FromOMPL(si_,s1),FromOMPL(si_,s2));
  if(e->IsVisible()) {
    delete e;
    return true;
  }
  return false;
}

bool CSpaceOMPLMotionValidator::checkMotion (const ob::State *s1, const ob::State *s2, std::pair< ob::State *, double > &lastValid) const
{
  EdgePlanner* e=space->LocalPlanner(FromOMPL(si_,s1),FromOMPL(si_,s2));
  if(e) {
    delete e;
    return true;
  }
  lastValid.first = si_->cloneState(s1);
  lastValid.second = 0;
  return false;
}


#endif // HAVE_OMPL
