#include "PBS.h" 

void
PBS::
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
PBS::
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
PBS:: 
Solve(const std::map<size_t, MotionTask>& _tasks, const std::vector<size_t>& _agents){
    Reset(_tasks, _agents); 

    CTInitialize<PriorityConstraint> initialize = [this]() {
        return this->Initialize(); 
    }; 

    CTValidate<PriorityConstraint> validate = [this](Node& _node) {
        this->m_exploredCount++; 
        return this->Validate(_node); 
    };

    CTSplit<PriorityConstraint> split = [this](const Node& _parent, const std::vector<FullConstraint>& _constraints){
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


std::vector<PBS::Node>
PBS::
Initialize() {
    // if no root is given, create a default root node
    if (m_root.size() == 0) {
        Node rootNode; 
        for (auto agent : m_agents) {
            Coord start = m_tasks[agent].first; 
            Coord goal = m_tasks[agent].second; 
            std::set<PriorityConstraint> empty;
            rootNode.m_constraints[agent] = empty;  

            auto [success, path] = LowLevel(agent, rootNode); 
            if (!success) {
                throw std::runtime_error("This problem is unsolvable"); 
            }
            rootNode.m_paths[agent] = path; 
        }
        rootNode.m_cost = this->Cost(rootNode.m_paths); 
        m_root.push_back(rootNode); 
    }

    m_totalCount = m_root.size(); 
    return m_root;
}

std::pair<bool, std::vector<PBS::Coord>> 
PBS:: 
LowLevel(size_t _agent, const Node& _node){
    Coord start = m_tasks[_agent].first; 
    Coord goal = m_tasks[_agent].second; 

    // Get priority constraints and expand into motion constraints
    std::set<MotionConstraint> extractedConstraints; 
    for (size_t constraintAgent : _node.m_constraints.at(_agent)) {
        const auto& path = _node.m_paths.at(constraintAgent); 
        for (size_t i = 0; i < path.size() - 1; i++) {
            extractedConstraints.insert(MotionConstraint(path[i], path[i], i, i)); 
            extractedConstraints.insert(MotionConstraint(path[i], path[i + 1], i, i + 1)); 
        }
        size_t lastIndex = path.size() - 1; 
        MotionConstraint lastConstraint(path[lastIndex], path[lastIndex], lastIndex, std::numeric_limits<size_t>::max()); 
        if (extractedConstraints.find(lastConstraint) != extractedConstraints.end())
            extractedConstraints.erase(lastConstraint); 
        extractedConstraints.insert(lastConstraint);
    }

    // Find minimum end time from the last applied motion constraint
    size_t endTime = FindMinimumEndTime(goal, extractedConstraints); 
    if (m_debug) {
        std::cout << "Finding Path for Agent " << _agent << std::endl; 
    }
    
    // Pass the expanded motion constraints and end time to low level search
    return m_lowlevel(start, goal, extractedConstraints, endTime); 
}



std::vector<PBS::FullConstraint> 
PBS::
Validate(const Node& _node) {
    // calculate the last timestep that needs to be checked
    size_t maxTime = 0; 
    for (auto kv : _node.m_paths) {
        maxTime = std::max(maxTime, kv.second.size()); 
    }

    if (m_debug) {
        for (auto kv : _node.m_constraints) {
            if (kv.second.size() > 0) {
                std::cout << "Agent " << kv.first << ": [ "; 
                for (auto constraint : kv.second) {
                    std::cout << constraint << " "; 
                }
                std::cout << "]" << std::endl; 
            }
        }
    }
    
    std::vector<std::pair<size_t, PriorityConstraint>> constraints; 
    // iterates through each timestep
    for (size_t t = 0; t < maxTime - 1; t++) {
        // pairwise comparison of agents' positions at the current timestep
        for (size_t i = 0; i < m_agents.size(); i++) {
            size_t agentA = m_agents[i]; 
            const auto& pathA = _node.m_paths.at(agentA); 

            size_t t1A = std::min(t, pathA.size() - 1); 
            size_t t2A = std::min(t + 1, pathA.size() - 1); 

            const Coord& positionA = pathA.at(t1A); 
            const Coord& nextA = pathA.at(t2A); 

            for (size_t j = i + 1; j < m_agents.size(); j++){
                size_t agentB = m_agents[j]; 
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
                FullConstraint constraintOne = {agentA, agentB}; 
                FullConstraint constraintTwo = {agentB, agentA};
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


std::vector<PBS::Node> 
PBS::
Split(const Node& _parent, const std::vector<FullConstraint>& _constraints){
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

    // create child with new priority constraint
    auto childA(_parent); 
    childA.m_constraints[agentA].insert(constraintA);

    // check the validity of the child node
    auto validityCheck = CheckConstraintValidity(agentA, childA.m_constraints); 
    if (validityCheck.first) { // if no constraint cycle exists, continue to replan child
        bool add = true; 
        for (auto agent : validityCheck.second) {
            auto pathReturn = LowLevel(agent, childA); 
            if (pathReturn.first) { // if a path is found, populate child
                childA.m_paths[agent] = pathReturn.second; 
            } else { // if a path cannot be found, don't add the child node
                add = false; 
                break; 
            }
        }
        if (add) {
            childA.m_cost = this->Cost(childA.m_paths); 
            children.push_back(childA); 

            // This safety check is redundant and for deep debugging purposes. It allows you to catch low level search 
            // errors when they occur as opposed to when they are next validated.             
            if (m_debug && validityCheck.first && !PathValidate(childA, agentA)) {
                std::cout << "Low-level search not adhering to priority constraint error: " << std::endl; 
                auto debugChild(_parent); 
                debugChild.m_constraints[agentA].insert(constraintA); 
                auto validityCheck = CheckConstraintValidity(agentA, debugChild.m_constraints);

                std::cout << "Replanning Agents: ["; 
                for (auto agent : validityCheck.second) 
                    std::cout << agent <<", "; 
                std::cout << "]" << std::endl; 

                for (auto agent : validityCheck.second) {
                    if (agent == agentA) 
                        std::cout << "Check agents: " << agent << " and " << agentA << std::endl; 

                    auto pathReturn = LowLevel(agent, debugChild); 
                    if (pathReturn.first) {
                        debugChild.m_paths[agent] = pathReturn.second; 
                    }
                }
            }
        }
    }


    if (_parent.m_constraints.at(agentB).find(constraintB) != _parent.m_constraints.at(agentB).end()) {
        throw std::runtime_error("Duplicate constraint added to child B"); 
    }

    // create child with new priority constraint
    auto childB(_parent); 
    childB.m_constraints[agentB].insert(constraintB); 

    // check the validity of the child node
    validityCheck = CheckConstraintValidity(agentB, childB.m_constraints); 
    if (validityCheck.first) { // if no constraint cycle exists, continue to replan child
        bool add = true; 
        for (auto agent : validityCheck.second) {
            auto pathReturn = LowLevel(agent, childB); 
            if (pathReturn.first) { // if a path is found, populate child
                childB.m_paths[agent] = pathReturn.second; 
            } else { // if a path cannot be found, don't add the child node
                add = false; 
                break;
            }
        }
        if (add) {
            childB.m_cost = this->Cost(childB.m_paths); 
            children.push_back(childB); 

            // This safety check is redundant and for deep debugging purposes. It allows you to catch low level search 
            // errors when they occur as opposed to when they are next validated.    
            if (m_debug && validityCheck.first && !PathValidate(childB, agentB)) {
                std::cout << "Low-level search not adhering to priority constraint error: " << std::endl; 
                auto debugChildB(_parent); 
                debugChildB.m_constraints[agentB].insert(constraintB); 
                auto validityCheck = CheckConstraintValidity(agentB, debugChildB.m_constraints);

                std::cout << "Replanning Agents: ["; 
                for (auto agent : validityCheck.second) 
                    std::cout << agent <<", "; 
                std::cout << "]" << std::endl; 

                for (auto agent : validityCheck.second) {
                    if (agent == agentB) 
                        std::cout << "Check agents: " << agent << " and " << agentA << std::endl; 

                    auto pathReturn = LowLevel(agent, debugChildB); 
                    if (pathReturn.first) {
                        debugChildB.m_paths[agent] = pathReturn.second; 
                    }
                }
            }
        }
    }

    return children; 
}

size_t
PBS:: 
FindMinimumEndTime(const Coord& _goal, const std::set<MotionConstraint>& _constraints) {
    // finds the minimum end time that is passed to the low level pathfinder based on the 
    // applied constraints that affect the agent's goal position
    size_t minTime = 0; 

    for (const auto& constraint : _constraints) {
        if (constraint.pos2 == _goal) {
            minTime = std::max(minTime, constraint.t2); 
        }
    }

    return minTime; 
}

std::pair<bool, std::vector<size_t>> 
PBS::
CheckConstraintValidity(size_t _agent, const ConstraintMap& _constraintMap){
    std::set<size_t> checkedParents; 
    std::vector<size_t> parents(_constraintMap.at(_agent).begin(), _constraintMap.at(_agent).end()); 
    size_t current_idx = 0; 

    // check if adding the new constraint results in a cycle 
    while (current_idx != parents.size()) {
        size_t parent = parents[current_idx]; 
        if (checkedParents.find(parent) == checkedParents.end()) {
            for (size_t constraint : _constraintMap.at(parent)) {
                if (constraint == _agent) {
                    return {false, {}}; 
                }
                parents.push_back(constraint); 
            }
            checkedParents.insert(parent); 
        }
        current_idx++; 
    }

    // find all children that are impacted by the new constraint. 
    std::set<size_t> replanAgents; 
    std::vector<size_t> replanList = {_agent}; 
    current_idx = 0; 

    while (current_idx != replanList.size()) {
        size_t current = replanList[current_idx]; 

        if (replanAgents.find(current) == replanAgents.end()) {
            for (const auto& [constraint, constraints] : _constraintMap) {
                if (constraints.find(current) != constraints.end()) {
                    replanList.push_back(constraint); 
                    if(replanAgents.find(constraint) != replanAgents.end())
                        replanAgents.erase(constraint); 
                }
            }
            replanAgents.insert(current); 
        }
        current_idx++; 
    }

    // get rid of redundant copies 
    replanAgents.clear(); 
    std::vector<size_t> finalList;        

    for (size_t i = 0; i < replanList.size(); i++) {
        size_t agent = replanList[replanList.size() - 1 - i]; 
        if (replanAgents.find(agent) == replanAgents.end()) {
            replanAgents.insert(agent); 
            finalList.push_back(agent); 
        }
    }
    std::reverse(finalList.begin(), finalList.end()); 

    if (m_debug) {
        std::cout << "Constraint Map" << std::endl; 
        for (auto kv : _constraintMap) {
            std::cout << "\t" << kv.first << ": "; 
            for (auto a : kv.second) {
                std::cout << a << " "; 
            }
            std::cout << std::endl; 
        }
        std::cout << "Replan List: ["; 
        for (size_t agent : finalList) 
            std::cout << agent << " ";
        std::cout << "]" << std::endl; 
    }
    return {true, finalList}; 
}

void
PBS::
SetRoot(const std::vector<Node>& _root) {
    m_root = _root; 
}

bool 
PBS:: 
PathValidate(const Node& _node, size_t _agent) {
    // Redundant Validation code - check validity of a single agent

    // get agents that have a priority constraint with _agent
    std::vector<size_t> agents; 
    for (size_t a : _node.m_constraints.at(_agent)){
        agents.push_back(a); 
    }

    // calculate maximum end time of all agents
    size_t maxTime = _node.m_paths.at(_agent).size(); 
    for(size_t a : agents) {
        maxTime = std::max(maxTime, _node.m_paths.at(a).size());
    }


    // iterate through each timestep
    for (size_t t = 0; t < maxTime - 1; t++) {
        const auto& pathB = _node.m_paths.at(_agent); 

        size_t t1B = std::min(t, pathB.size() - 1); 
        size_t t2B = std::min(t + 1, pathB.size() - 1); 

        const Coord& posB = pathB.at(t1B); 
        const Coord& nextB = pathB.at(t2B); 
        
        // iterate through each priority agent
        for (size_t i = 0; i < agents.size(); i++) {
            size_t agentA = agents[i]; 
            const auto& pathA = _node.m_paths.at(agentA); 

            size_t t1A = std::min(t, pathA.size() - 1); 
            size_t t2A = std::min(t + 1, pathA.size() - 1); 

            const Coord& positionA = pathA.at(t1A); 
            const Coord& nextA = pathA.at(t2A); 

            // check for illegal collision between priority constraint agents and _agent
            auto [collision, positions, times] = GridCollisionCheck(positionA, nextA, posB, nextB, t); 
            if (!collision) 
                continue; 
            
            std::cout << "Collision: " << _agent <<" " << agentA << std::endl;
            std::cout << times.first << " " << times.second << std::endl;
            std::cout <<"(" << positions.first.first << ", " << positions.first.second << ")" << std::endl;
            std::cout <<"(" << positions.second.first << ", " << positions.second.second << ")" << std::endl; 

            return false; // if illegal collision found, return that the pathfinding failed
        }
    }
    return true;  // if no collisions are found, return that the pathfinding succeeded 
}