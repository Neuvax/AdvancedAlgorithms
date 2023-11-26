/**
 * @file main.cpp
 * @author Jorge Germán Wolburg Trujillo -- A01640826
 * @author Armando Terrazas Gómez -- A01640924
 * @brief
 * @date 09-09-2023
 */
#include <bits/stdc++.h>

using namespace std;

typedef long double ld;
typedef long long lli;
typedef pair<lli, lli> ii;
typedef vector<int> vi;

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
 * @brief Performs a BFS on a graph.
 *
 * This function performs a BFS represented by a residual matrix. It starts from a source node and ends at a sink node.
 * It is used to find an augmenting path in the graph. If such a path is found, the function returns true, otherwise it returns false.
 *
 * @param s The source node from where the BFS needs to start.
 * @param t The sink node where the BFS needs to end.
 * @param residual The residual graph where the BFS needs to be performed. It is a 2D vector where residual[i][j] represents the capacity of the edge from node i to node j.
 * @param parent A vector to store the parent of each node. This is used to construct the augmenting path if it exists.
 * @return Returns true if there is a path from source to sink in the residual graph. Otherwise, it returns false.
 * 
 * @note Complexity: O(V^2) since we are using an adjacency matrix to represent the graph.
 */
bool bfs(int s, int t, vector<vi> &residual, vi &parent) {
    vector<bool> visited(sz(residual), false);

    queue<int> q;
    q.push(s);
    visited[s] = true;
    parent[s] = -1;

    while (!q.empty()) {
        int u = q.front();
        q.pop();

        fore(v, 0, sz(residual)) {
            if (!visited[v] && residual[u][v] > 0) {
                if (v == t) {  // If we reach sink return true
                    parent[v] = u;
                    return true;
                }
                q.push(v);
                parent[v] = u;
                visited[v] = true;
            }
        }
    }

    return false; // If we don't reach sink return false
}

/**
 * @brief Implements the Edmonds-Karp algorithm for finding the maximum flow in a flow network.
 *
 * This function implements the Edmonds-Karp algorithm, which is an implementation of the Ford-Fulkerson method for computing the maximum flow in a flow network.
 * The algorithm uses breadth-first search to find augmenting paths and computes the maximum flow by summing up the flow in each path.
 *
 * @param residual The flow network where the maximum flow needs to be computed. It is a 2D vector where graph[i][j] represents the capacity of the edge from node i to node j.
 * @param s The source node in the flow network.
 * @param t The sink node in the flow network.
 * @return Returns the maximum flow from source to sink in the given flow network.
 * 
 * @note Complexity:O(E*V^3) , where V is the number of vertices and E is the number of edges in the flow network, since we are using an adjacency matrix to represent the graph.
 */
int edmondsKarp(vector<vi> &residual, int s, int t) {
    vi parent(sz(residual));
    int maxFlow = 0;

    while (bfs(s, t, residual, parent)) {
        int pathFlow = numeric_limits<int>::max();
        for (int v = t; v != s; v = parent[v]) {
            int u = parent[v];
            pathFlow = min(pathFlow, residual[u][v]);
        }

        for (int v = t; v != s; v = parent[v]) {
            int u = parent[v];
            residual[u][v] -= pathFlow;
            residual[v][u] += pathFlow;
        }

        maxFlow += pathFlow;
    }

    return maxFlow;
}

int main() { _
    // freopen("input.txt", "r", stdin);
    // freopen("output.txt", "w", stdout);
    int n;
    cin >> n;

    // 1. Kruskal

    // 2. TST

    // 3. Edmonds-Karp
    vector<vi> graph(n, vi(n));
    fore(u, 0, n) {
        fore(v, 0, n) {
            cin >> graph[u][v];
        }
    }

    cout << edmondsKarp(graph, 0, sz(graph) - 1) << endl;

    //4. Voronoi Diagram


    return 0;
}

// g++-13 -std=c++20 main.cpp && ./a.out < input.txt > output.txt