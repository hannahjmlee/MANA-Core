#include "HCCBS.h" 

void
HCCBS::
Reset() {
    m_tasks.clear(); 
    m_agents.clear(); 

    m_exploredCount = 0; 
    m_totalCount = 0; 

    if (m_solution != nullptr) {
        delete m_solution; 
        m_solution = nullptr; 
    }

    for (auto kv : m_treeReference) {
        if (kv.second != nullptr) {
            delete kv.second;  
        }
    }
    m_treeReference.clear(); 

    for (auto kv : m_nodeReference) {
        if (kv.second != nullptr) {
            for (auto sv : *kv.second) {
                if (sv.second != nullptr)
                    delete sv.second; 
            }

            delete kv.second; 
        }
    }
    m_nodeReference.clear(); 
}

void
HCCBS::
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

    for (auto kv : m_treeReference) {
        if (kv.second != nullptr) {
            delete kv.second;  
        }
    }
    m_treeReference.clear(); 

    
    m_cbs = CBS(m_lowlevel, m_cost, m_debug); 
    m_cbs.SetTasks(m_tasks); 
}

MultiPathSolution*
HCCBS:: 
Solve(const std::map<size_t, MotionTask>& _tasks, const std::vector<size_t>& _agents){
    Reset(_tasks, _agents);     

    HCInitialize<Subproblem> initialize = [this](Subproblem _fullProblem) {
        return this->InitializeLayer(_fullProblem); 
    }; 

    HCOrganize<Subproblem> organize = [this](const Layer& _layer) {
        // std::cout << std::endl; 
        return this->OrganizeLayer(_layer); 
    };

    HCGetSubproblem<Subproblem> getSubproblem = [this](const Layer& _layer, size_t index) {
        return this->GetSubproblems(_layer, index); 
    }; 

    HCMerge<Subproblem> merge = [this](const Layer& _subproblems) {
        auto start = std::chrono::high_resolution_clock::now();
        auto solution = this->CrossTrees(_subproblems[0], _subproblems[1]); 
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> time = end - start; 
        this->m_crossTime += time.count(); 
        return solution; 
    }; 

    HCSolve<Subproblem> solve = [this](Subproblem _fullProblem) {
        m_cbs.SetNodeReference(this->m_nodeReference[_fullProblem]); 
        Node solution = this->m_cbs.Solve(_fullProblem, this->m_treeReference[_fullProblem]);

        if (solution.m_paths.size() == 0) 
            return false; 

        m_solutions[_fullProblem] = std::move(solution); 
        this->m_exploredCount += this->m_cbs.GetExploredCount(); 
        return true; 
    }; 
    
    auto start = std::chrono::high_resolution_clock::now();
    Subproblem fullProblem = HierarchicalSolve(m_agents, initialize, organize, getSubproblem, merge, solve); 
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> time = end - start; 
    m_runtime = time.count(); 

    Node solutionNode = m_treeReference[fullProblem]->top();
    
    // total node count = total explored nodes + size of last tree - 1
    // offset by 1 because the solution node (which was explored) is added back to the CT
    m_totalCount = m_exploredCount + m_treeReference[fullProblem]->size() - 1; 
    m_solution = new MultiPathSolution(solutionNode.m_paths, solutionNode.m_cost, m_exploredCount, m_totalCount); 

    return m_solution;
}

HCCBS::Layer 
HCCBS::
InitializeLayer(Subproblem _fullProblem){
    Layer layer; 
    // creates a layer of subproblems consisting of CTs that only hold a single root node 
    // single root node consists of the optimal path for a single agent
    std::set<MotionConstraint> empty; 
    for (const size_t& agent : _fullProblem) {
        ConstraintTree* ct = new ConstraintTree(); 
        Node singleNode; 

        auto [success, path] = m_lowlevel(m_tasks[agent].first, m_tasks[agent].second, empty, 0); 
        if (!success) {
            throw std::runtime_error("This problem is unsolvable"); 
        }

        singleNode.m_paths[agent] = path;
        singleNode.m_constraints[agent] = empty; 
        singleNode.m_cost = this->Cost(singleNode.m_paths); 
        ct->push(singleNode); 

        Subproblem singleAgent = {agent}; 
        m_treeReference[singleAgent] = ct; 
        layer.push_back(singleAgent); 

        m_solutions[singleAgent] = singleNode; 
    }
    return layer; 
}

HCCBS::Layer
HCCBS::
OrganizeLayer(const Layer& _layer){
    Layer organized; 

    if (m_heuristic == "conflict" || m_heuristic == "cardinal") {
        // Counts the motion conflicts or cardinal conflicts 
        auto conflictMap = CountMotionConflicts(_layer, m_heuristic == "cardinal"); 
        if (m_debug) {
            std::cout << "Conflict Count:" << std::endl; 
            std::cout << "\t"; 
            for (auto kv : conflictMap) {
                std::cout << kv.second << " "; 
            }
            std::cout << std::endl; 
        }
        // Organize agents based on the conflict counts
        OrganizeAgents(_layer, organized, conflictMap); 
    } else {
        // randomly organizes the layer
        organized = _layer; 
        std::default_random_engine rng(0); 
        std::shuffle(organized.begin(), organized.end(), rng); 
    }
    return organized; 
}

std::map<std::pair<HCCBS::Subproblem, HCCBS::Subproblem>, size_t> 
HCCBS::
CountMotionConflicts(const Layer& _layer, bool _cardinal){
    std::map<size_t, std::vector<Coord>> pathMap; 
    std::map<std::pair<size_t, size_t>, size_t> conflictCounter; 
    std::map<size_t, Subproblem> subproblemMapper; 
    std::vector<size_t> allAgents; 

    // calculates last timestep to check and populates all structures with path information
    size_t maxTime = 0; 
    for (size_t i = 0; i < _layer.size(); i++) {
        Subproblem agents = _layer.at(i); 
        Node solution = m_solutions[agents]; 
        if (solution.m_nodeRef.size() > 0) {
            solution.Expand(m_nodeReference[agents]);
        }

        for (size_t agent : agents) {
            allAgents.push_back(agent); 
            subproblemMapper.emplace(agent, agents); 
            pathMap.emplace(agent, solution.m_paths[agent]); 
            maxTime = std::max(maxTime, solution.m_paths[agent].size()); 
        }
    }

    // iterates through each timestep 
    for (size_t t = 0; t < maxTime - 1; t++) {
        // pairwise comparison of agents
        for (size_t i = 0; i < allAgents.size(); i++) {
            size_t agentA = m_agents[i]; 
            const auto& pathA = pathMap[agentA]; 

            size_t t1A = std::min(t, pathA.size() - 1); 
            size_t t2A = std::min(t + 1, pathA.size() - 1); 

            const Coord& posA = pathA.at(t1A); 
            const Coord& nextA = pathA.at(t2A); 
            for (size_t j = i + 1; j < allAgents.size(); j++) {
                size_t agentB = m_agents[j]; 

                if (subproblemMapper[agentA] == subproblemMapper[agentB]) 
                    continue; 

                const auto& pathB = pathMap.at(agentB); 

                size_t t1B = std::min(t, pathB.size() - 1); 
                size_t t2B = std::min(t + 1, pathB.size() - 1); 

                const Coord& posB = pathB.at(t1B); 
                const Coord& nextB = pathB.at(t2B);

                // checks for collisions using a traditional grid collision check 
                auto [collision, positions, times] = GridCollisionCheck(posA, nextA, posB, nextB, t); 
                if (!collision) 
                    continue;

                // if cardinal, check cardinality 
                if (_cardinal) {
                    MotionConstraint newConstraint = MotionConstraint(positions, times); 

                    Node nodeOne = m_solutions[subproblemMapper[agentA]]; 
                    Node nodeTwo = m_solutions[subproblemMapper[agentB]]; 

                    std::set<MotionConstraint> constraintsA = nodeOne.m_constraints.at(agentA); 
                    constraintsA.insert(newConstraint); 
                    std::set<MotionConstraint> constraintsB = nodeTwo.m_constraints.at(agentB); 
                    constraintsB.insert(newConstraint); 

                    auto [successA, pathA] = m_lowlevel(m_tasks[agentA].first, m_tasks[agentA].second, 
                                                 constraintsA, FindMinimumEndTime(agentA, constraintsA)); 
                    auto [successB, pathB] = m_lowlevel(m_tasks[agentB].first, m_tasks[agentB].second, 
                                                 constraintsB, FindMinimumEndTime(agentB, constraintsB)); 
        
                    size_t cardinality = 0; 
                    // if path cost does not increase for either agent, it is not cardinal (0)
                    // if path cost increases for a single agent, it is semi-cardinal (1)
                    // if path cost increases for both agents, it is cardinal (2)
                    if (this->m_cost == "soc") {
                        if (pathA.size() > nodeOne.m_paths.at(agentA).size())
                            cardinality++; 
                        if (pathB.size() > nodeTwo.m_paths.at(agentB).size())
                            cardinality++; 
                    } else {
                        size_t maxCost = std::max(nodeOne.m_cost, nodeTwo.m_cost); 
                        if (pathA.size() > maxCost)
                            cardinality++; 
                        if (pathB.size() > maxCost) 
                            cardinality++; 
                    }

                    // still want to consider non-cardinal conflicts, so we offset by 1
                    conflictCounter[{agentA, agentB}] += cardinality + 1; 
                } else {
                    // when using "conflict" heuristic, simply increment with each found conflict
                    conflictCounter[{agentA, agentB}] += 1; 
                }
            }
        }
    }

    // merge individual agent conflict counts into subproblem conflict counts
    std::map<std::pair<Subproblem, Subproblem>, size_t> mergedCounter; 
    for (const auto& kv : conflictCounter) {
        const Subproblem& subOne = subproblemMapper[kv.first.first]; 
        const Subproblem& subTwo = subproblemMapper[kv.first.second]; 

        mergedCounter[{subOne, subTwo}] += kv.second; 
    }
    
    return mergedCounter; 
}


HCCBS::Layer 
HCCBS:: 
GetSubproblems(const Layer& _layer, size_t _index) {
    // HCCBS returns two subproblems in a given layer at a time
    Layer layer; 
    layer.push_back(_layer[_index]); 
    if (_index + 1 < _layer.size())
        layer.push_back(_layer[_index + 1]); 
    
    return layer; 
}

void 
HCCBS::
OrganizeAgents(const Layer& _layer, Layer& _organized, 
               std::map<std::pair<Subproblem, Subproblem>, size_t>& _conflictCounter){

    // sorts conflictCounter map from largest number of conflicts to smallest number of conflicts
    auto comparator = [] (const auto& _lhs, const auto& _rhs) {
        return _lhs.second > _rhs.second; 
    };
    std::vector<std::pair<std::pair<Subproblem, Subproblem>, size_t>> mapVector(_conflictCounter.begin(), _conflictCounter.end());
    std::sort(mapVector.begin(), mapVector.end(), comparator); 

    std::set<Subproblem> usedSubproblems; 
    size_t index = 0; 
    // greedily grab the subproblems with the most number of conflicts and repopulate the layer in that order
    for (size_t index = 0; index < mapVector.size(); index++) {
        if (_layer.size() - _organized.size() <= 1) 
            break; 

        auto bestMember = std::max_element(_conflictCounter.begin(), _conflictCounter.end(), comparator); 
        const Subproblem& groupOne = mapVector[index].first.first; 
        const Subproblem& groupTwo = mapVector[index].first.second; 

        if ((usedSubproblems.find(groupOne) != usedSubproblems.end()) || 
            (usedSubproblems.find(groupTwo) != usedSubproblems.end())) 
            continue; 
        
        _organized.push_back(groupOne); 
        _organized.push_back(groupTwo); 

        usedSubproblems.insert(groupOne); 
        usedSubproblems.insert(groupTwo); 
    }

    // if we're missing subproblems from our organized layer
    if (_layer.size() - _organized.size() >= 1) {
        // populate all all subproblems list to figure out which subproblems we're missing
        std::set<Subproblem> allSubproblems; 
        for (size_t i = 0; i < _layer.size(); i++) {
            allSubproblems.insert(_layer.at(i)); 
        }

        // if there are subproblems with 0 conflicts that were ommitted from the conflict counter map, 
        // or leftover subproblems tack them onto the end of the layer again
        std::set<Subproblem> setDiff; 
        std::set_difference(allSubproblems.begin(), allSubproblems.end(), 
                            usedSubproblems.begin(), usedSubproblems.end(), 
                            std::inserter(setDiff, setDiff.end())); 
        for (const auto& subproblem : setDiff) 
            _organized.push_back(subproblem); 
    }
}

HCCBS::Subproblem 
HCCBS::
CrossTrees(const Subproblem& _groupOne, const Subproblem& _groupTwo){
    // cross trees from each subproblem and populate the node reference structures

    ConstraintTree* treeOne = m_treeReference[_groupOne]; 
    std::vector<Node>* vectorOne = new std::vector<Node>(); 
    vectorOne->reserve(treeOne->size()); 

    ConstraintTree* treeTwo = m_treeReference[_groupTwo]; 
    std::vector<Node>* vectorTwo = new std::vector<Node>(); 
    vectorTwo->reserve(treeTwo->size()); 

    while (!treeOne->empty()) {
        vectorOne->push_back(std::move(treeOne->top())); 
        treeOne->pop(); 
    }

    while (!treeTwo->empty()) {
        vectorTwo->push_back(std::move(treeTwo->top())); 
        treeTwo->pop(); 
    }

    // create new merged subproblem consisting of the agents from each subproblme
    Subproblem merged; 
    merged.reserve(_groupOne.size() + _groupTwo.size()); 
    merged.insert(merged.end(), _groupOne.begin(), _groupOne.end()); 
    merged.insert(merged.end(), _groupTwo.begin(), _groupTwo.end()); 


    // create a new node reference structure for the merged subproblem
    // maps subproblem to a map of ct tree reference index to a vector of nodes
    m_nodeReference[merged] = new NodeReference();
    size_t refIndexOne = m_nodeReferenceIndex++; 
    size_t refIndexTwo = m_nodeReferenceIndex++; 
    m_nodeReference[merged]->emplace(refIndexOne, vectorOne); 
    m_nodeReference[merged]->emplace(refIndexTwo, vectorTwo); 

    // if a subproblem has node references, copy them to the new merged problem and clean up memory
    if (m_nodeReference.find(_groupOne) != m_nodeReference.end()) {
        for (auto& kv : *(m_nodeReference[_groupOne])) {
            m_nodeReference[merged]->emplace(kv.first, kv.second); 
        }
        delete m_nodeReference[_groupOne]; 
    }
    // do the same with the second subproblem
    if (m_nodeReference.find(_groupTwo) != m_nodeReference.end()) {
        for (auto& kv : *(m_nodeReference[_groupTwo])) {
            m_nodeReference[merged]->emplace(kv.first, kv.second); 
        }
        delete m_nodeReference[_groupTwo]; 
    }
    
    // create a new constraint tree
    ConstraintTree* newTree = new ConstraintTree(); 

    // iterate through each pairwise combination of nodes 
    for (size_t i = 0; i < vectorOne->size(); i++) {
        const Node& nodeOne = vectorOne->at(i); 
        for (size_t j = 0; j < vectorTwo->size(); j++) {
            const Node& nodeTwo = vectorTwo->at(j); 

            // create a new node that holds indices that point to the node references
            // allows for lazy expansion of nodes at search time to reduce copy/construction time
            Node newNode;

            if (nodeOne.m_nodeRef.size() > 0) { // if the node hasn't been expanded
                for (const auto& ref : nodeOne.m_nodeRef) {
                    newNode.m_nodeRef.push_back(ref);  // copy over any node references from within the node
                }
            } else { // if the node has been expanded
                newNode.m_nodeRef.push_back({refIndexOne, i}); // create a new node reference 
            }

            if (nodeTwo.m_nodeRef.size() > 0) {
                for (const auto& ref : nodeTwo.m_nodeRef) {
                    newNode.m_nodeRef.push_back(ref); 
                }
            } else {
                newNode.m_nodeRef.push_back({refIndexTwo, j}); 
            }

            // calculate cost of the node
            if (m_cost == "soc") {
                newNode.m_cost = nodeOne.m_cost + nodeTwo.m_cost; 
            } else {
                newNode.m_cost = std::max(nodeOne.m_cost, nodeTwo.m_cost); 
            }

            newTree->push(std::move(newNode)); // add node to CT 
        }
    }


    // memory clean up for each CT whose contents have been moved to a node reference
    delete treeOne; 
    m_treeReference[_groupOne] = nullptr; 
    delete treeTwo; 
    m_treeReference[_groupTwo] = nullptr; 

    // add new tree to the tree references
    m_treeReference[merged] = newTree; 
    return merged; 
}


size_t
HCCBS:: 
FindMinimumEndTime(size_t _agent, const std::set<MotionConstraint>& _constraints) {
    // finds the minimum end time that is passed to the low level pathfinder based on the applied constraints
    size_t minTime = 0; 
    const Coord& goal = m_tasks[_agent].second; 

    for (const auto& constraint : _constraints) {
        if (constraint.pos2 == goal) 
            minTime = std::max(minTime, constraint.t2); 
    }

    return minTime; 
}