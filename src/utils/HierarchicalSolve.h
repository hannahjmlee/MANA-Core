#ifndef HIERARCHICAL_SOLVE_H
#define HIERARCHICAL_SOLVE_H

#include <functional> 
#include <map> 
#include <omp.h> 
#include <vector> 

// Alias for a function that initializes subproblems from a full problem
template<typename Subproblem> 
using HCInitialize = std::function<
                                   std::vector<Subproblem>
                                   (Subproblem _fullProblem)
                                   >; 

// Alias for a function that organizes a set of subproblems
template<typename Subproblem> 
using HCOrganize = std::function<
                                 std::vector<Subproblem>
                                 (const std::vector<Subproblem>& _subproblems)
                                 >; 

// Alias for a function that retrieves subproblems from a layer of subproblems
template<typename Subproblem> 
using HCGetSubproblem = std::function<
                                      std::vector<Subproblem>
                                      (const std::vector<Subproblem>& _subproblems, size_t _index)
                                      >; 

// Alias for a function that merges multiple subproblems into a single subproblem
template<typename Subproblem> 
using HCMerge = std::function<
                              Subproblem
                              (const std::vector<Subproblem>& _subproblems)
                              >;
                              
// Alias for a function that solves a subproblem
template<typename Subproblem> 
using HCSolve = std::function<
                              bool
                              (Subproblem _fullProblem)
                              >; 


// HierarchicalSolve function that solves a full problem using hierarchical decomposition
template<typename Subproblem> 
Subproblem
HierarchicalSolve(Subproblem _fullProblem, 
                  HCInitialize<Subproblem>& _initialize, 
                  HCOrganize<Subproblem>& _organize, 
                  HCGetSubproblem<Subproblem>& _getProblem, 
                  HCMerge<Subproblem>& _merge, 
                  HCSolve<Subproblem>& _solve) {
    
    using Layer = std::vector<Subproblem>; 
    Subproblem solutionProblem; 

    // Initialize first layer of subproblems - should consist of all initial problems
    Layer layer = _initialize(_fullProblem); 
    if (layer.size() != _fullProblem.size()) {
        return solutionProblem; 
    }

    do { 
        Layer organizedLayer = _organize(layer); // organize the layer
        layer.clear(); 

        size_t i = 0; 
        while (i < organizedLayer.size()) {
            // get the current set of subproblems
            std::vector<Subproblem> currentSubproblems = _getProblem(organizedLayer, i); 
            i += currentSubproblems.size(); 

            if (currentSubproblems.size() == 1) {
                layer.push_back(currentSubproblems[0]); 
                continue; 
            }

            // create a new merged subproblem
            Subproblem newProblem = _merge(currentSubproblems); 
            if (!_solve(newProblem))
                return solutionProblem; 
                
            // add merged subproblem to layer
            layer.push_back(newProblem); 
        }

    } while (layer.size() != 1);  // repeat until only a single problem remains
                    
    // get and return solution problem
    solutionProblem = layer[0];
    return solutionProblem;                     
}

#endif
