#include "Problem.h"
#include "utils/MotionConstraint.h"

#include <iostream>
#include <limits>
#include <set>
#include <cassert>

void ReadBenchmarkFile(std::string _filename, size_t _scenario) {
  // prints out benchmark files to check if it is properly being read in 
  // also grabs first motion task from a scenario and plots it on the map as S and G

  Problem problemInstance = Problem(_filename, _scenario);
  auto grid = problemInstance.GetGrid();
  std::cout << "Testing Read of Benchmark File" << std::endl;
  std::cout << "\tMotion Task 1:";
  auto task = problemInstance.GetTasks()[10];
  auto start = task.first;
  auto goal = task.second;
  std::cout << "(" << start.first << ", " << start.second << ") --> ";
  std::cout << "(" << goal.first << ", " << goal.second << ")" << std::endl << std::endl;

  // populate environment
  std::vector<std::vector<char>> printGrid;
  for (auto line : grid) {
    std::vector<char> singleLine;
    for (bool isFree : line) {
      if (isFree)
        singleLine.push_back('.');
      else
        singleLine.push_back('#');
    }
    printGrid.push_back(singleLine);
  }
  // mark start and goal
  printGrid[start.second][start.first] = 'S';
  printGrid[goal.second][goal.first] = 'G';

  // print grid for verification
  for (auto line : printGrid) {
    std::cout << "\t\t";
    for (auto c : line) {
      std::cout << c;
    }
    std::cout << std::endl;
  }

  std::cout << std::endl;
  return;
}


void TestMotionConstraintLogic() {
    // TODO:: These constraint tests do not comprehensively check everything necessary. Needs to be udpated

    // MotionConstraint b({11, 21}, {11, 21}, 8, std::numeric_limits<size_t>::max());
    // MotionConstraint a({11, 21}, {11, 21}, 26, 26);

    // // MotionConstraint a({39, 18}, {39, 18}, 68, 68);
    // // MotionConstraint b({39, 18}, {39, 19}, 68, 69);
    // // MotionConstraint c({39, 19}, {39, 19}, 69, 69);

    // std::set<MotionConstraint> debugSet;
    // debugSet.insert(b);

    // std::cout << (debugSet.find(a) == debugSet.end()) << std::endl;
    // std::cout << (debugSet.find(b) != debugSet.end()) << std::endl;
    // std::cout << (a < b) << std::endl; 
    // std::cout << (b < a) << std::endl; 
    // std::cout << (a < b) << std::endl; 
    // std::cout << (b < a) << std::endl; 
    // std::cout << (a < b) << std::endl; 
    // std::cout << (b < a) << std::endl; 
    // // std::cout << (debugSet.find(c) != debugSet.end()) << std::endl;

    std::cout << "Running Motion Constraint Class Tests..." << std::endl;
    // Vertex Tests
    MotionConstraint vb0({1, 1}, {1, 1}, 5, 5);
    MotionConstraint vc1({1, 1}, {1, 1}, 4, 4);
    MotionConstraint vc2({1, 2}, {1, 2}, 5, 5);

    std::set<MotionConstraint> vertexSet;
    vertexSet.insert(vb0);

    std::cout << "\tChecking vertex tests: ";
    size_t passed = 0;
    size_t passable = 3;
    assert(vertexSet.find(vb0) != vertexSet.end());
    passed++;
    assert(vertexSet.find(vc1) == vertexSet.end());
    passed++;
    assert(vertexSet.find(vc2) == vertexSet.end());
    passed++;
    std::cout << "[" << passed << "/" << passable << "]" << std::endl;


    // Edge Tests
    MotionConstraint eb0({1, 2}, {1, 3}, 5, 6);
    MotionConstraint ec1({1, 3}, {1, 2}, 5, 6); // true
    MotionConstraint ec2({1, 2}, {1, 3}, 4, 5); // false
    MotionConstraint ec3({1, 2}, {1, 3}, 6, 7); // false
    MotionConstraint ec4({5, 6}, {6, 7}, 5, 6); // false
    MotionConstraint ec5({5, 6}, {6, 7}, 1, 2); // false

    std::set<MotionConstraint> edgeSet;
    edgeSet.insert(eb0);

    std::cout << "\tChecking edge tests: ";
    passed = 0;
    passable = 6;
    assert(edgeSet.find(eb0) != edgeSet.end());
    passed++;
    assert(edgeSet.find(ec1) != edgeSet.end());
    passed++;
    assert(edgeSet.find(ec2) == edgeSet.end());
    passed++;
    assert(edgeSet.find(ec3) == edgeSet.end());
    passed++;
    assert(edgeSet.find(ec4) == edgeSet.end());
    passed++;
    assert(edgeSet.find(ec5) == edgeSet.end());
    passed++;
    std::cout << "[" << passed << "/" << passable << "]" << std::endl;


    // Infinite Range Tests
    MotionConstraint ib0({1, 1}, {1, 1}, 6, std::numeric_limits<size_t>::max());
    MotionConstraint ic1({1, 1}, {1, 1}, 6, 6); // true
    MotionConstraint ic2({1, 1}, {1, 1}, 7, 8); // true
    MotionConstraint ic3({1, 1}, {1, 1}, 9, std::numeric_limits<size_t>::max()); // true
    MotionConstraint ic4({1, 1}, {1, 1}, 1, 1); // false
    MotionConstraint ic5({1, 1}, {1, 2}, 3, 4); // false
    MotionConstraint ic6({1, 1}, {1, 2}, 9, 10); // false

    std::set<MotionConstraint> infiniteSet;
    infiniteSet.insert(ib0);

    std::cout << "\tChecking infinite range tests: ";
    passed = 0;
    passable = 7;
    infiniteSet.find(ic1);
    assert(infiniteSet.find(ib0) != infiniteSet.end());
    passed++;
    assert(infiniteSet.find(ic1) != infiniteSet.end());
    passed++;
    assert(infiniteSet.find(ic2) != infiniteSet.end());
    passed++;
    assert(infiniteSet.find(ic3) != infiniteSet.end());
    passed++;
    assert(infiniteSet.find(ic4) == infiniteSet.end());
    passed++;
    assert(infiniteSet.find(ic5) == infiniteSet.end());
    passed++;
    assert(infiniteSet.find(ic6) == infiniteSet.end());
    passed++;
    std::cout << "[" << passed << "/" << passable << "]" << std::endl;
    std::cout << std::endl;
}


