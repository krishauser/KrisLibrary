#include "OMPLInterface.h"
#include <KrisLibrary/planning/EdgePlanner.h>
#include <KrisLibrary/utils/stringutils.h>
#include <math/random.h>

#if HAVE_OMPL

#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/OptimizationObjective.h>
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
  space->setup();
  resolution = space->getLongestValidSegmentLength();
  sampler_ = si->allocStateSampler();
  si_ = si;
  stemp1 = si->allocState();
  stemp2 = si->allocState();
  stemp3 = si->allocState();

  /* debugging...
  for(int i=0;i<100000;i++) {
    sampler_->sampleUniform(stemp1);
    sampler_->sampleUniform(stemp2);
    bool visible = si_->checkMotion(stemp1,stemp2);
    Config a,b;
    FromOMPL(stemp1,a);
    FromOMPL(stemp2,b);
    EdgePlanner* e= LocalPlanner(a,b);
    assert(visible == e->IsVisible());
    delete e;
  }
  */
}

OMPLCSpace::~OMPLCSpace()
{
  si_->freeState(stemp1);
  si_->freeState(stemp2);
  si_->freeState(stemp3);
}


void OMPLCSpace::Properties(PropertyMap& props) const
{
  props.set("intrinsicDimension",si_->getStateDimension());
  props.setArray("minimum",vector<Real>(qMin));
  props.setArray("maximum",vector<Real>(qMax));
  props.set("diameter",si_->getStateSpace()->getMaximumExtent());
}


void OMPLCSpace::SetSpaceBounds(const Config& qmin, const Config& qmax) {
	qMin = qmin;
	qMax = qmax;
}

void OMPLCSpace::SetSpaceBounds(const double& qmin, const double& qmax) {
	qMin.set(qmin);
	qMax.set(qmax);
}

void OMPLCSpace::Sample(Config& x)
{
  if(sampler_) {
    sampler_->sampleUniform(stemp1);
    FromOMPL(stemp1,x);
  }
  else {
  	x.resize(nD);
  	for(int i = 0; i < nD; i++)
  		x[i] = Rand(qMin[i], qMax[i]);
  }
}

void OMPLCSpace::SampleNeighborhood(const Config& c,Real r,Config& x)
{
  if(sampler_) {
    ToOMPL(c,stemp1);
    sampler_->sampleUniformNear(stemp2,stemp1,r);
    FromOMPL(stemp2,x);
  }
  else {
    CSpace::SampleNeighborhood(c,r,x);
  }
}


void OMPLCSpace::ToOMPL(const Config& q,ob::State * s)
{
  vtemp.resize(q.n);
  for(int i=0;i<q.n;i++) vtemp[i] = q[i];
  si_->getStateSpace()->copyFromReals(s,vtemp);  
}


ob::State * OMPLCSpace::ToOMPL(const Config& q)
{
  ob::State* res = si_->getStateSpace()->allocState();
  ToOMPL(q,res);
  return res;
}

Config OMPLCSpace::FromOMPL(const ob::State * s)
{
  Config res;
  FromOMPL(s,res);
  return res;
}

void OMPLCSpace::FromOMPL(const ob::State * s,Config& q)
{
  si_->getStateSpace()->copyToReals(vtemp,s);
  q.resize(vtemp.size());
  for(size_t i=0;i<vtemp.size();i++) q[i] = vtemp[i];
}

bool OMPLCSpace::IsFeasible(const Config& x) {
  //ob::RealVectorStateSpace* rspace = (si_->getStateSpace())->as<ob::RealVectorStateSpace>();
  //assert(rspace != NULL);
  ToOMPL(x,stemp1);
  return si_->getStateValidityChecker()->isValid(stemp1);
}

double OMPLCSpace::Distance(const Config& a,const Config& b)
{
  ToOMPL(a,stemp1);
  ToOMPL(b,stemp2);
  return si_->getStateSpace()->distance(stemp1,stemp2);
}

void OMPLCSpace::Interpolate(const Config& a,const Config& b,Real u,Config& x)
{
  ToOMPL(a,stemp1);
  ToOMPL(b,stemp2);
  si_->getStateSpace()->interpolate(stemp1,stemp2,u,stemp3);
  FromOMPL(stemp3,x);
}

class OMPLEdgePlanner : public StraightLineEpsilonPlanner
{
public:
  OMPLEdgePlanner(OMPLCSpace* space,const Config& x,const Config& y,Real resolution)
  :StraightLineEpsilonPlanner(space,x,y,resolution),omplspace(space)
  {
  }
  virtual bool IsVisible() {
    omplspace->ToOMPL(this->a,omplspace->stemp1);
    omplspace->ToOMPL(this->b,omplspace->stemp2);
    this->foundInfeasible = !omplspace->si_->checkMotion(omplspace->stemp1,omplspace->stemp2);
    this->dist = 0;
    return !this->foundInfeasible;
  }
  virtual EdgePlanner* Copy() const { return new OMPLEdgePlanner(omplspace,a,b,epsilon); }
  virtual EdgePlanner* ReverseCopy() { return new OMPLEdgePlanner(omplspace,b,a,epsilon); }
  OMPLCSpace* omplspace;
};

EdgePlanner* OMPLCSpace::LocalPlanner(const Config& x, const Config& y)
{
  return new OMPLEdgePlanner(this,x,y,resolution);
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
  if(!planner) {
    fprintf(stderr,"KrisLibraryOMPLPlanner::setup(): there was a problem creating the planner!\n");
    return;
  }
  if(planner->IsOptimizing()) {
    specs_.optimizingPaths = true;
    specs_.approximateSolutions = true;
    specs_.canReportIntermediateSolutions = true;
  }
  if(!ob::Planner::isSetup()) 
    ob::Planner::setup();

}

void KrisLibraryOMPLPlanner::getPlannerData (ob::PlannerData &data) const
{
  if(!planner) return;
  OMPLCSpace* nc_cspace = const_cast<OMPLCSpace*>((const OMPLCSpace*)cspace);
  RoadmapPlanner roadmap(nc_cspace);
  MotionPlannerInterface* iplanner = const_cast<MotionPlannerInterface*>((const MotionPlannerInterface*)planner);
  iplanner->GetRoadmap(roadmap);
  for(size_t i=0;i<roadmap.roadmap.nodes.size();i++) {
    unsigned int id = data.addVertex(ob::PlannerDataVertex(nc_cspace->ToOMPL(roadmap.roadmap.nodes[i]),(int)i));
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
  if(!planner) {
    //may have had a previous clear() call
    //fprintf(stderr,"KrisLibraryOMPLPlanner::solve(): Warning, setup() not called yet\n");
    setup();
    if(!planner) 
      return ob::PlannerStatus(ob::PlannerStatus::UNKNOWN);
  }
  ob::ProblemDefinitionPtr pdef = this->getProblemDefinition();
  //how much to plan?
  int increment = (StartsWith(factory.type.c_str(),"fmm") ? 1 : 50);
  Real oldBest = Inf;
  bool optimizing = planner->IsOptimizing();
  double desiredCost = 0;
  if(optimizing) {
    if(pdef->getOptimizationObjective() != NULL)
      desiredCost = pdef->getOptimizationObjective()->getCostThreshold().value();
    else 
      OMPL_INFORM("%s: No optimization objective specified. Defaulting to optimizing path length for the allowed planning time.", getName().c_str());
  }
  while(!ptc()) {
    if(planner->IsSolved()) {
      //convert solution to OMPL solution
      MilestonePath path;
      planner->GetSolution(path);
      if(optimizing) {
        //optimizing
        Real cost = path.Length();
        if(cost < oldBest) {
          oldBest = cost;
          if(cost < desiredCost) {
            ob::PathPtr pptr(ToOMPL(si_,path));
            this->getProblemDefinition()->addSolutionPath(pptr);
            return ob::PlannerStatus(true,false);
          }
        }
      }
      else {
        //non-optimizing
        ob::PathPtr pptr(ToOMPL(si_,path));
        this->getProblemDefinition()->addSolutionPath(pptr);
        return ob::PlannerStatus(true,false);        
      }
    }

    planner->PlanMore(increment); 
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
  if(props.getArray("minimum",minimum) && props.getArray("maximum",maximum)) {
  }
  else {
    minimum.resize(0);
    maximum.resize(0);
  }
  ob::RealVectorBounds bounds(minimum.size());
  bounds.low = minimum;
  bounds.high = maximum;
  setBounds(bounds);
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
