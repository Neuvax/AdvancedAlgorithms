/**
 * @file main.cpp
 * @author Armando Terrazas Gómez -- A01640924
 * @author Jorge German Wolburg Trujillo -- A01640826
 * @brief Program that solves a maze using backtracking and branch and bound
 * @version 1.0
 * @date 23-08-2023
 */
// #pragma GCC optimize("Ofast", "unroll-loops", "no-stack-protector", "fast-math")
// #pragma GCC target("avx,avx2,fma")
#include <bits/stdc++.h>

using namespace std;

typedef long double ld;
typedef long long lli;
typedef pair<lli, lli> ii;
typedef vector<lli> vi;

#define f first
#define s second
#define endl '\n'
#define pb push_back
#define sz(s) lli(s.size())
#define all(s) begin(s), end(s)
#define deb(x) cout << #x ": " << (x) << endl
#define print(x) cout << (x) << endl
#define fore(i, a, b) for (lli i = (a), BS = (b); i < BS; ++i)
#define _                         \
    ios_base::sync_with_stdio(0); \
    cin.tie(0);                   \
    cout.tie(0);

/**
 * @brief Solves a maze using backtracking
 *
 * This function takes a 2D vector representing a maze and the starting position (0, 0),
 * and uses a backtracking algorithm to find a path from the top-left corner to the
 * bottom-right corner of the maze. The function marks each visited cell with a 2.
 *
 * @param maze The maze to solve
 * @param m The row index of the current position in the maze
 * @param n The column index of the current position in the maze
 * @return true if a path was found, false otherwise
 */
bool solveBackTrack(vector<vi> &maze, lli m, lli n, char comingFrom = '\0') {
    // Check if I'm out of bounds or if I'm in a wall and if I'm not 0 or 2
    if (m < maze.size() && n < maze[0].size() && (maze[m][n] == 0 || maze[m][n] == 2)) return false;  // Don't go there

    // Check if I'm in the bottom-right corner
    if (m + 1 == maze.size() && n + 1 == maze[0].size()) {
        maze[m][n] = 2;
        return true;
    }

    bool bottom = false, right = false, up = false;
    // If I can move to the bottom, I'll explore that path
    if (m + 1 < maze.size()) {
        bottom = solveBackTrack(maze, m + 1, n);
    }

    // If I can move to the right, I'll explore that path
    if (n + 1 < maze[0].size()) {
        right = solveBackTrack(maze, m, n + 1, 'r');
    }

    // If any of the two paths is true then there is path
    if (bottom || right) {
        maze[m][n] = 2;
        return true;
    }

    // If there are no paths downwards or to the right, then check upwards
    if (m - 1 >= 0 && comingFrom == 'r') {
        up = solveBackTrack(maze, m - 1, n, 'r');
    }

    // If there is a path upwards, mark the current as 2
    if (up) {
        maze[m][n] = 2;
        return true;
    }

    // If I can't move to the right, bottom or up, I'll mark the current cell as not a possible path
    maze[m][n] = 0;
    return false;
}

/**
 * @brief Solves a maze using Branch and Bound
 *
 * This function takes a 2D vector representing a maze and the starting position (0, 0),
 * and uses a Branch and Bound algorithm to find a path from the top-left corner to the
 * bottom-right corner of the maze. The function marks each visited cell with a 2.
 *
 * @param maze The maze to solve
 * @param m The row index of the current position in the maze
 * @param n The column index of the current position in the maze
 * @return true if a path was found, false otherwise
 */
bool solveBranchBound(vector<vi> &maze, lli m, lli n) {
    // Create a priority queue to store the cells to explore
    priority_queue<pair<lli, pair<ii, vector<ii>>>> cells_to_explore;

    // Initialize with the starting cell (m, n) and an empty path
    cells_to_explore.push({0, {{m, n}, {}}});

    while (!cells_to_explore.empty()) {
        // Get the current cell and its path
        auto current = cells_to_explore.top();
        cells_to_explore.pop();
        ii cell = current.second.first;
        m = cell.first;
        n = cell.second;
        vector<ii> path = current.second.second;

        // If we have reached the bottom-right corner, mark the path to the goal and return true
        if (m == maze.size() - 1 && n == maze[0].size() - 1) {
            // Mark the path to the goal cell as visited
            for (lli i = path.size() - 1; i >= 0; --i) {
                maze[path[i].first][path[i].second] = 2;
            }
            maze[m][n] = 2;  // Mark the goal cell as visited
            return true;
        }

        vi dx = {1, 0, -1, 0};
        vi dy = {0, 1, 0, -1};

        for (lli i = 0; i < 4; ++i) {  // Change the loop to explore all possible moves
            lli new_m = m + dx[i];
            lli new_n = n + dy[i];

            // Check if the new cell is valid and hasn't been visited yet
            if (new_m >= 0 && new_m < maze.size() && new_n >= 0 && new_n < maze[0].size() && maze[new_m][new_n] == 1) {
                // Calculate the estimated distance to the goal cell using the Manhattan distance heuristic
                lli distance = llabs(new_m - (maze.size() - 1)) + llabs(new_n - (maze[0].size() - 1));

                // Push the new cell and its path into the priority queue with the estimated distance as the priority
                vector<ii> new_path = path;
                new_path.push_back({m, n});
                cells_to_explore.push({-distance, {{new_m, new_n}, new_path}});
                maze[new_m][new_n] = 0;  // Mark the new cell as visited
            }
        }
    }

    // If the priority queue becomes empty and we haven't reached the destination, return false
    return false;
}

int main() { _
    // freopen("input.txt", "r", stdin);
    // freopen("output.txt", "w", stdout);
    lli m,
    n;
    cin >> m >> n;
    vector<vi> maze(m, vi(n));
    fore(i, 0, m) {
        fore(j, 0, n) {
            lli a;
            cin >> a;
            maze[i][j] = a;
        }
    }

    vector<vi> maze2 = maze;

    if (solveBackTrack(maze, 0, 0)) {
        fore(i, 0, m) {
            fore(j, 0, n) {
                cout << (maze[i][j] == 2 ? 1 : 0) << " ";
            }
            cout << endl;
        }
    } else {
        cout << "No solution found using Branch and Bound." << endl;
    }

    cout << endl;  // Space for proper output requirements

    // Solve using Branch and Bound
    if (solveBranchBound(maze2, 0, 0)) {
        // Print the Branch and Bound solution
        fore(i, 0, m) {
            fore(j, 0, n) {
                cout << (maze2[i][j] == 2 ? 1 : 0) << " ";
            }
            cout << endl;
        }
    } else {
        cout << "No solution found using Branch and Bound." << endl;
    }

    return 0;
}

// Template obtained from Leones' team for ICPC
// Support in logic obtained from GeekForGeeks

// g++-13 -std=c++20 main.cpp && ./a.out < input.txt > output.txt