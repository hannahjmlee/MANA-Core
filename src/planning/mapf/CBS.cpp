#include "CBS.h"

void
CBS::
Reset() {
    m_tasks.clear(); 
    m_agents.clear(); 

    m_exploredCount = 0; 
    m_totalCount = 0; 

    if (m_solution != nullptr) {
        delete m_solution; 
        m_solution = nullptr; 
    }
}

void
CBS::
Reset(const std::vector<size_t>& _agents) {
    m_agents.clear(); 
    m_agents = _agents; 

    m_exploredCount = 0; 
    m_totalCount = 0; 

    if (m_solution != nullptr) {
        delete m_solution; 
        m_solution = nullptr; 
    }
}

void
CBS::
Reset(const std::map<size_t, MotionTask>& _tasks, const std::vector<size_t>& _agents) {
    m_tasks.clear(); 
    m_agents.clear(); 

    m_tasks = _tasks; 
    m_agents = _agents; 

    m_exploredCount = 0; 
    m_totalCount = 0; 

    if (m_solution != nullptr) {
        delete m_solution; 
        m_solution = nullptr; 
    }
}


MultiPathSolution*
CBS:: 
Solve(const std::map<size_t, MotionTask>& _tasks, const std::vector<size_t>& _agents){
    Reset(_tasks, _agents); 

    CTInitialize<MotionConstraint> initialize = [this]() {
        auto root = this->Initialize(this->m_agents); 
        this->m_totalCount = root.size(); 
        return root; 
    }; 

    CTValidate<MotionConstraint> validate = [this](Node& _node) {
        this->m_exploredCount++; 
        return this->Validate(_node, this->m_agents); 
    };

    CTSplit<MotionConstraint> split = [this](const Node& _parent, const std::vector<FullConstraint>& _constraints){
        auto children = this->Split(_parent, _constraints); 
        this->m_totalCount += children.size(); 
        return children; 
    };

    auto start = std::chrono::high_resolution_clock::now();
    Node solutionNode = CTSolve(initialize, validate, split, m_root); 
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> time = end - start; 

    m_runtime = time.count(); 
    m_solution = new MultiPathSolution(solutionNode.m_paths, solutionNode.m_cost, m_exploredCount, m_totalCount); 

    return m_solution;
}


CBS::Node
CBS::
Solve(const std::vector<size_t>& _agents, ConstraintTree* _tree){
    Reset(_agents); 
    m_totalCount = _tree->size(); 

    CTValidate<MotionConstraint> validate = [this, _agents](Node& _node) {
        this->m_exploredCount++; 
        _node.Expand(this->m_nodeReference); // Expands node as necessary if it's using a condensed representation
        return this->Validate(_node, _agents); 
    };

    CTSplit<MotionConstraint> split = [this](const Node& _parent, const std::vector<FullConstraint>& _constraints){
        auto children = this->Split(_parent, _constraints); 
        this->m_totalCount += children.size(); 
        return children; 
    };

    auto start = std::chrono::high_resolution_clock::now();
    Node solutionNode = CTSolve(validate, split, _tree); 
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> time = end - start; 

    m_runtime = time.count(); 

    return solutionNode; 
}

CBS::Node
CBS:: 
Solve(const std::vector<size_t>& _agents, const NodeReference* _nodeReference, ConstraintTree* _tree, size_t& _exploredCount) const {
    // unlike the other solves, this solve function does not modify the CBS class and is meant to be used for multithreading purposes.
    
    CTValidate<MotionConstraint> validate = [this, _agents, _nodeReference, &_exploredCount](Node& _node) {
        _exploredCount += 1; 
        _node.Expand(_nodeReference);  // Expands node as necessary if it's using a condensed representation
        return this->Validate(_node, _agents); 
    };

    CTSplit<MotionConstraint> split = [this](const Node& _parent, const std::vector<FullConstraint>& _constraints){
        auto children = this->Split(_parent, _constraints);
        return children; 
    };

    auto start = std::chrono::high_resolution_clock::now();
    Node solutionNode = CTSolve(validate, split, _tree); 
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> time = end - start; 
    return solutionNode; 
}

std::vector<CBS::Node>
CBS::
Initialize(const std::vector<size_t>& _agents) {
    // if no root is given, create a default root node
    if (m_root.size() == 0) {
        Node rootNode; 
        for (auto agent : _agents) {
            Coord start = m_tasks[agent].first; 
            Coord goal = m_tasks[agent].second; 
            std::set<MotionConstraint> empty; 

            auto [success, path] = m_lowlevel(start, goal, empty, 0); 
            if (!success) {
                throw std::runtime_error("This problem is unsolvable"); 
            }
            rootNode.m_paths[agent] = path; 
            rootNode.m_constraints[agent] = empty; 
        }
        rootNode.m_cost = this->Cost(rootNode.m_paths); 
        m_root.push_back(rootNode); 
    }
    
    return m_root; 
}

std::vector<CBS::FullConstraint> 
CBS::
Validate(const Node& _node, const std::vector<size_t>& _agents) const{
    // calculate the last timestep that needs to be checked
    size_t maxTime = 0; 
    for (auto agent : _agents) {
        maxTime  = std::max(maxTime, _node.m_paths.at(agent).size()); 
    }
    
    std::vector<std::pair<size_t, MotionConstraint>> constraints; 
    // iterates through each timestep
    for (size_t t = 0; t < maxTime - 1; t++) {
        // pairwise comparison of agents' positions at the current timestep
        for (size_t i = 0; i < _agents.size(); i++) {
            size_t agentA = _agents[i]; 
            const auto& pathA = _node.m_paths.at(agentA); 

            size_t t1A = std::min(t, pathA.size() - 1); 
            size_t t2A = std::min(t + 1, pathA.size() - 1); 

            const Coord& positionA = pathA.at(t1A); 
            const Coord& nextA = pathA.at(t2A); 

            for (size_t j = i + 1; j < _agents.size(); j++){
                size_t agentB = _agents[j]; 
                const auto& pathB = _node.m_paths.at(agentB); 

                size_t t1B = std::min(t, pathB.size() - 1); 
                size_t t2B = std::min(t + 1, pathB.size() - 1); 

                const Coord& posB = pathB.at(t1B); 
                const Coord& nextB = pathB.at(t2B); 

                // find collision using traditional grid collision check
                auto [collision, positions, times] = GridCollisionCheck(positionA, nextA, posB, nextB, t); 
                if (!collision) 
                    continue; // if no collision is found, continue to the next collision check
                
                // create and return constraints based on the found collision
                FullConstraint constraintOne = {agentA, MotionConstraint(positions, times)}; 
                FullConstraint constraintTwo = {agentB, MotionConstraint(positions, times)}; 

                constraints.push_back(constraintOne); 
                constraints.push_back(constraintTwo); 

                if (m_debug) {
                    std::cout << "Collision found between Agent " << agentA << " and " << agentB << "." << std::endl; 
                    std::cout << "Collision: {" << positions.first.first << ", " << positions.first.second << "}, {" << 
                                 positions.second.first << ", " << positions.second.second << "}" << 
                                 " from time [" << times.first << ", " << times.second << "]" << std::endl; 
                    std::cout << "\tConstraint 1: {" << constraintOne.first << ", " << constraintOne.second << "}" << std::endl; 
                    std::cout << "\tConstraint 1: {" << constraintTwo.first << ", " << constraintTwo.second << "}" << std::endl; 
                }
                
                return constraints; 
            }
        }
    }

    return constraints; 
}


std::vector<CBS::Node> 
CBS::
Split(const Node& _parent, const std::vector<FullConstraint>& _constraints) const{
    auto agentA = _constraints.at(0).first;
    auto constraintA = _constraints.at(0).second; 
    auto agentB = _constraints.at(1).first;
    auto constraintB = _constraints.at(1).second; 

    
    std::vector<Node> children; 

    // safety check - make sure low level pathfinder is adhering to constraints by checking 
    // if any duplicate constraints have been added. 
    if (_parent.m_constraints.at(agentA).find(constraintA) != _parent.m_constraints.at(agentA).end()) {
        throw std::runtime_error("Duplicate constraint added to child A"); 
    }

    // try to create the first child node - if the low level pathfinder can't find a solution, 
    // the child is not created
    auto childA(_parent); 
    childA.m_constraints[agentA].insert(constraintA);
    size_t minEnd = FindMinimumEndTime(agentA, childA.m_constraints[agentA]); 
    auto [success, path] = m_lowlevel(m_tasks.at(agentA).first, m_tasks.at(agentA).second,
                                      childA.m_constraints[agentA], minEnd); 

    if (success) {
        childA.m_paths[agentA] = path; 
        childA.m_cost = this->Cost(childA.m_paths); 
        children.push_back(childA); 
    }

    // same safety check as above, but for the second agent
    if (_parent.m_constraints.at(agentB).find(constraintB) != _parent.m_constraints.at(agentB).end()) {
        throw std::runtime_error("Duplicate constraint added to child B"); 
    }

    // try to create the second child node
    auto childB(_parent); 
    childB.m_constraints[agentB].insert(constraintB); 
    minEnd = FindMinimumEndTime(agentB, childB.m_constraints[agentB]); 
    auto [successB, pathB] = m_lowlevel(m_tasks.at(agentB).first, m_tasks.at(agentB).second, 
                                      childB.m_constraints[agentB], minEnd); 
    if(successB) {
        childB.m_paths[agentB] = pathB; 
        childB.m_cost = this->Cost(childB.m_paths); 
        children.push_back(childB); 
    }

    return children; 
}


size_t
CBS:: 
FindMinimumEndTime(size_t _agent, const std::set<MotionConstraint>& _constraints) const{
    // finds the minimum end time that is passed to the low level pathfinder based on the applied constraints
    size_t minTime = 0; 
    const Coord& goal = m_tasks.at(_agent).second; 

    for (const auto& constraint : _constraints) {
        if (constraint.pos2 == goal) 
            minTime = std::max(minTime, constraint.t2); 
    }

    return minTime; 
}

void
CBS::
SetRoot(const std::vector<Node>& _root) {
    m_root = _root; 
}


void 
CBS:: 
SetTasks(const std::map<size_t, MotionTask> _tasks){
    m_tasks = _tasks; 
}


size_t 
CBS:: 
GetExploredCount() const{
    return m_exploredCount;
}

void 
CBS:: 
SetNodeReference(NodeReference* _nodeReference){
    m_nodeReference = _nodeReference; 
}