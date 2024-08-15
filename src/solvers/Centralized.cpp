#include "Centralized.h"

CentralizedSolver::
CentralizedSolver(Problem* _problem, std::string _mapf, std::string _lowlevel,
                  std::string _costMetric, bool _debug){
    m_problem = _problem;
    m_debug = _debug;
    m_costMetric = _costMetric;
    m_heuristic = "random";

    CreatePathfinder(_lowlevel);
    CreateMAPF(_mapf);
}

CentralizedSolver::
CentralizedSolver(Problem* _problem, std::string _mapf, std::string _lowlevel,
                  std::string _costMetric, std::string _heuristic, bool _debug){
    m_problem = _problem;
    m_debug = _debug;
    m_costMetric = _costMetric;
    m_heuristic = _heuristic;

    CreatePathfinder(_lowlevel);
    CreateMAPF(_mapf);
}

void
CentralizedSolver::
CreatePathfinder(std::string _lowlevel) {
    // Create pathfinder object
    if (_lowlevel == "AStar"){
        m_lowlevel = new AStar(m_problem->GetGrid(), m_debug);
    } else if (_lowlevel == "Dijkstra") {
        m_lowlevel = new Dijkstra(m_problem->GetGrid(), m_debug);
    }
}

void
CentralizedSolver::
CreateMAPF(std::string _mapf) {
    // Create low-level search functor
    LowLevelFunction llSolve = [this](const Coord& _start, const Coord& _goal,
                                const std::set<MotionConstraint>& _constraints,
                                size_t _minend) {
        return this->m_lowlevel->Solve(_start, _goal, _constraints, _minend);
    };

    // Create multi-agent pathfinding object
    if (_mapf == "CBS" || _mapf == "cbs") {
        m_mapf = new CBS(llSolve, m_costMetric, m_debug);
    } else if (_mapf == "PBS" || _mapf == "pbs") {
        m_mapf = new PBS(llSolve, m_costMetric, m_debug);
    } else if (_mapf == "HCCBS" || _mapf == "hccbs") {
        m_mapf = new HCCBS(llSolve, m_costMetric, m_heuristic, m_debug);
    }
}


std::tuple<bool, size_t, CentralizedSolver::SolutionMap, double>
CentralizedSolver::
Solve (size_t _problemSize){
    // clear and set internal variables
    m_tasks.clear();
    m_tasks = m_problem->GetTaskProblem(_problemSize);
    m_agents.clear();
    for (const auto& kv : m_tasks) {
        m_agents.push_back(kv.first);
    }

    // clear search variables and delete any existing solutions
    m_mapf->Reset();
    if (m_solution != nullptr)
        delete m_solution;

    // solve and create solution object
    m_solution =  new MultiPathSolution(m_mapf->Solve(m_tasks, m_agents));

    // get runtime of MAPF algorithm
    m_runtime = m_mapf->GetRuntime();

    // return success, solution cost, solution, and runtime
    return {m_solution->GetSuccess(), m_solution->GetSolutionCost(), m_solution->GetSolution(), m_runtime};
}

std::pair<size_t, size_t>
CentralizedSolver::
GetWorkAnalysis() const{
    // returns search space size and state space size
    return {m_solution->GetSearchSize(), m_solution->GetSpaceSize()};
}
