#ifndef CT_NODE_H
#define CT_NODE_H

#include <cstdlib>
#include <iostream>
#include <map>
#include <set>
#include <utility>
#include <vector>

template <typename ConstraintType>
class CTNode{

    public:

        using Coord            = std::pair<size_t, size_t>;                     // alias for a coordinate pair (x, y)
        using FullConstraint   = std::pair<size_t, ConstraintType>;             // alias for a full constraint consisting of an agent ID and a constraint type
        using PathMap          = std::map<size_t, std::vector<Coord>>;          // alias for a map that tracks agent index to its solution path
        using ConstraintMap    = std::map<size_t, std::set<ConstraintType>>;    // alias for a map that tracks agents and their applied constraints
        using NodeReference    = std::vector<std::pair<size_t, size_t>>;        // alias for a vector of pairs representing node references by tree and node index

    public: 

        PathMap m_paths;                // map storing the paths for each agent
        ConstraintMap m_constraints;    // map storing the constraints for each agent
        NodeReference m_nodeRef;        // vector storing references to other nodes
        size_t m_cost;                  // cost associated with the node

    public: 
    
        // Default constructor
        CTNode(){};

        // Copy constructor
        CTNode(const CTNode& _other) : 
            m_paths(_other.m_paths), 
            m_constraints(_other.m_constraints), 
            m_nodeRef(_other.m_nodeRef),
            m_cost(_other.m_cost) {}; 

        // Move constructor
        CTNode(CTNode&& _other) noexcept: 
            m_paths(std::move(_other.m_paths)), 
            m_constraints(std::move(_other.m_constraints)), 
            m_nodeRef(std::move(_other.m_nodeRef)),
            m_cost(_other.m_cost) {}; 

        // Move assignment operator
        CTNode& operator=(CTNode&& _other) noexcept {
            if (this != &_other) {
                m_paths = std::move(_other.m_paths);
                m_constraints = std::move(_other.m_constraints);
                m_nodeRef = std::move(_other.m_nodeRef); 
                m_cost = _other.m_cost;
            }
            return *this;
        };

        // Constructor combining two CTNode instances
        CTNode(const CTNode& _one, const CTNode& _two) {
            m_paths = _one.m_paths; 
            for (const auto& kv : _two.m_paths) {
                m_paths[kv.first].insert(m_paths[kv.first].end(), kv.second.begin(), kv.second.end()); 
            }

            m_constraints = _one.m_constraints; 
            for (const auto& kv : _two.m_constraints) {
                m_constraints[kv.first].insert(kv.second.begin(), kv.second.end()); 
            }

            m_cost = _one.m_cost + _two.m_cost; 
        };

        // Constructor with paths, constraints, and cost
        CTNode(PathMap _paths, ConstraintMap _constraints, size_t _cost)
            : m_paths(_paths), m_constraints(_constraints), m_cost(_cost) {};

        // Copy assignment operator
        CTNode& operator=(const CTNode&) = default;

        // Destructor
        ~CTNode() = default;

        // Greater-than operator for comparing node costs
        bool operator> (const CTNode& _other) const {
            return m_cost > _other.m_cost;
        };

        // Less-than operator for comparing node costs
        bool operator< (const CTNode& _other) const {
            return m_cost < _other.m_cost;
        };

        // Method to count the total number of constraints in the node
        size_t NumConstraints() const {
            size_t count = 0;
            for (const auto& kv : m_constraints) {
                count += kv.second.size();
            }
            return count;
        };

        // Method to expand the condensed representation node by incorporating references from other nodes
        void Expand(const std::map<size_t, std::vector<CTNode<ConstraintType>>*>* _references) {
            if (m_nodeRef.empty())
                return; 

            std::set<ConstraintType> empty; 
            
            for (auto nv : m_nodeRef) {
                size_t tree = nv.first; 
                size_t nodeIndex = nv.second; 

                const CTNode<ConstraintType>& node = _references->at(tree)->at(nodeIndex); 
                for (const auto kv : node.m_paths) {
                    m_paths.emplace(kv.first, kv.second); 
                    m_constraints.emplace(kv.first, empty); 
                }
                for (const auto kv : node.m_constraints) {
                    m_constraints.emplace(kv.first, kv.second); 
                    m_constraints[kv.first] = kv.second; 
                }
            }

            m_nodeRef.clear(); 
            return; 
        }
};

#endif