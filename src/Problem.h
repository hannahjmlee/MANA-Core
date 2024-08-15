#ifndef PROBLEM_H
#define PROBLEM_H

#include <array> 
#include <fstream> 
#include <iostream>
#include <map>
#include <sstream> 
#include <stdexcept>
#include <string>
#include <utility> 
#include <vector> 

class Problem {

    public: 
    
        using Coord       = std::pair<size_t, size_t>;  // Alias for a coordinate pair (x, y)
        using MotionTask  = std::pair<Coord, Coord>;    // Alias for a motion task consisting of a start and goal coordinate

    public:

        size_t                         m_scenario;          // Scenario number being used in the problem
        
    private:

        std::string                    m_mapName;           // Name of the map being used in the problem
        size_t                         m_numRows;           // Number of rows in the grid
        size_t                         m_numCols;           // Number of columns in the grid
        std::vector<std::vector<bool>> m_grid;              // Grid representation of the map (true for passable, false for obstacle)
        std::vector<MotionTask>        m_queries;           // List of motion tasks (start and goal coordinates)
        std::string                    m_name = "Problem";  // Name of the Problem instance

    public: 

        // Default constructor
        Problem() {}; 

        // Constructor that loads problem data from a file
        Problem(const std::string& filename);

        // Constructor that loads problem data from a file with a specific scenario
        Problem(const std::string& filename, const size_t scenario);
        
        // Method to load a specific scenario from the file
        void LoadScenario(const size_t scenario);

        // Method to get the grid
        std::vector<std::vector<bool>> GetGrid() const;
        
        // Method to get the list of motion tasks
        std::vector<MotionTask> GetTasks() const;

        // Method to get a map of motion tasks for a given problem size
        std::map<size_t, MotionTask> GetTaskProblem(size_t _size) const;
        
        // Method to get the name of the Problem instance
        std::string GetName() const {
            return m_name; 
        };

    private: 
    
        // Method to read lines from a file, skipping a specified number of lines
        std::vector<std::string> GetLines(const std::string& filename, const size_t skip_lines);

        // Method to create a boolean grid from the lines read from the file
        std::vector<std::vector<bool>> CreateBooleanGrid(const std::vector<std::string>& lines);
};


#endif
