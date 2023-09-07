#ifndef TICTOC_CLASS_H
#define TICTOC_CLASS_H
#include <chrono>
#include <iostream>
#include <iomanip>
#include <unordered_map>
#include <vector>

using namespace std;
using namespace std::chrono;

inline unordered_map<string, high_resolution_clock::time_point> startsMap;
inline unordered_map<string, size_t> durationMap; 

inline void tic(string MissionName)
{
    startsMap[MissionName] = high_resolution_clock::now();
}

inline void toc(string MissionName)
{
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - startsMap[MissionName]);

    if (durationMap.find(MissionName) == durationMap.end())
        durationMap[MissionName] = 0;

    durationMap[MissionName] += duration.count();
}

inline bool value_comparator(const std::pair<string, size_t>& a, const std::pair<string, size_t>& b) {
    return a.second > b.second;
}

inline void printAllTimes()
{
    vector<pair<string, size_t>> durations(durationMap.begin(), durationMap.end());
    cout << endl << "Profiler: " << endl;

    std::sort(durations.begin(), durations.end(), value_comparator);

    for (auto& i : durations)
    {
        cout << setw(30) << left << i.first;
        cout << "-> " << setw(7) << right << i.second / 1000;
        cout << " milliseconds" << endl;
    }
}
 
#endif