#include "Problem.h"

Problem::
Problem(const std::string& filename) {
    m_mapName = filename;

    // load just the map and set internal variables
    std::string mapFile = "./benchmarks/maps/" + filename + ".map";
    std::vector<std::string> mapLines = GetLines(mapFile, 4);
    m_grid = CreateBooleanGrid(mapLines);

    m_numRows = m_grid.size(); // y bounds
    m_numCols = m_grid[0].size(); // x bounds
    
    GetVertices(); 
}

Problem::
Problem(const std::string& filename, const size_t scenario, const size_t resolution) {
    m_mapName = filename;
    m_scenario = scenario;
    m_resolution = resolution;

    // load the map and set internal variables
    std::string mapFile = "./benchmarks/maps/" + filename + ".map";
    std::vector<std::string> mapLines = GetLines(mapFile, 4);

    m_grid = CreateBooleanGrid(mapLines);

    m_numRows = m_grid.size(); // y bounds
    m_numCols = m_grid[0].size(); // x bounds

    // load scenario
    LoadScenario(scenario);
    GetVertices(); 
}

std::vector<std::vector<bool>>
Problem::
GetGrid() const{
    return m_grid;
}

std::vector<Problem::MotionTask>
Problem::
GetTasks() const{
    return m_queries;
}


std::map<size_t, Problem::MotionTask>
Problem::
GetTaskProblem(size_t _size) const{
    if (_size > m_queries.size()) {
        throw std::runtime_error("Size exceeded number of possible queries.");
    }

    // create a task map that maps a task index to a motion task query
    std::map<size_t, MotionTask> taskMap;
    for (size_t i = 0; i < _size; i++) {
        taskMap[i] = m_queries[i];
    }
    return taskMap;
}

std::vector<std::string>
Problem::
GetLines(const std::string& filename, const size_t skip_lines) {
    std::vector<std::string> lines;

    // open input file
    std::ifstream inputFile(filename);
    std::string line;
    size_t count = 0;

    if (inputFile.is_open()) {
        while (std::getline(inputFile, line)) {
            count ++;
            // skip the provided number of lines
            if (count <= skip_lines)
                continue;

            // populate the set of lines
            lines.push_back(line);
        }
        inputFile.close();
    } else {
        std::cerr << "Unable to open file: " << filename << std::endl;
    }
    return lines;
}

std::vector<std::vector<bool>>
Problem::
CreateBooleanGrid(const std::vector<std::string>& lines) {
    // modify these obstacle space and free space characters as necessary to fit your input files
    const std::string obst_chars = "T@O";
    const std::string free_chars = ".G";

    size_t rows = lines.size();
    size_t cols = lines.empty() ? 0 : lines[0].size();

    // create a boolean grid representing free space (true) and obstacle space (false)
    std::vector<std::vector<bool>> bool_grid;

    for (size_t i = 0; i < rows; i++) {
        std::vector<bool> bool_line;
        for (size_t j = 0; j < cols; j++) {
            char ch = lines[i][j];

            if (obst_chars.find(ch) != std::string::npos) {
                bool_line.push_back(false); // obstacles are false
            } else {
                bool_line.push_back(true); // free space is true
            }
        }
        bool_grid.push_back(bool_line);
    }

    m_grid = bool_grid; 
    return m_grid; 
}

void
Problem::
LoadScenario(const size_t scenario) {
    // get scenario and load file
    m_scenario = scenario;
    std::string scenarioFile = "./benchmarks/scenarios/" + m_mapName + "-random-"
        + std::to_string(scenario) + ".scen";
    std::vector<std::string> scenarioLines = GetLines(scenarioFile, 1);

    // save all motion task queries represented using a pair of coordinates for the
    // start and goal of the task
    m_queries.clear();
    for (const auto& line : scenarioLines) {
        std::istringstream iss(line);
        std::vector<std::string> splitLine;
        std::string word;
        while (iss >> word) {
            splitLine.push_back(word);
        }

        Coord start = {std::stod(splitLine[4]), std::stod(splitLine[5])};
        Coord goal = {std::stod(splitLine[6]), std::stod(splitLine[7])};

        m_queries.push_back({start, goal});
    }

    return;
}

std::pair<double, double>
Problem::
GetUpperBounds() const{
    return {m_numCols - 1, m_numRows - 1};
}


std::set<std::pair<double, double>>
Problem::
GetVertices() {
    if (m_vertices.size() > 0)
        return m_vertices;

    if (m_resolution == 1) {
        for (size_t x = 0; x < m_numCols; x++) {
            for (size_t y = 0; y < m_numRows; y++) {
                if (!m_grid[y][x])
                    continue;
                m_vertices.insert({x, y});
            }
        }

    } else {
        double step = 1.0 / m_resolution;
        for (size_t x = 0; x < m_numCols; x++) {
            for (size_t y = 0; y < m_numRows; y++) {
                if (!m_grid[y][x]) {
                    continue;
                }

                m_vertices.insert({x, y});

                if (IsSquare(x, y)) { // if there is a square of valid vertices, fill all intermediates
                    for (double interX = x; interX <= x + 1; interX += step) {
                        for (double interY = y; interY <= y + 1; interY += step) {
                            m_vertices.insert({interX, interY});
                        }
                    }
                } else { // if there is not a square of valid vertices, fill connected edges with intermediates
                    if (x + 1 <= m_numCols - 1 && m_grid[y][x + 1]) {  // right edge exists
                        for (double interX = x; interX <= x + 1; interX += step) {
                            m_vertices.insert({interX, y});
                        }
                    }
                    if (y + 1 <= m_numRows - 1 && m_grid[y + 1][x]) { // down edge exists
                        for (double interY = y; interY <= y + 1; interY += step) {
                            m_vertices.insert({x, interY});
                        }
                    }
                }
            }
        }

    }
    return m_vertices;
}


bool
Problem::
IsSquare(size_t _x, size_t _y) const{
    if (_x + 1 > m_numCols - 1 || _y + 1 > m_numRows - 1) 
        return false; 
    return (m_grid[_y][_x + 1] && m_grid[_y + 1][_x] && m_grid[_y + 1][_x + 1]);
}

double
Problem::
GetStepSize() const{
    return 1.0 / m_resolution;
}
