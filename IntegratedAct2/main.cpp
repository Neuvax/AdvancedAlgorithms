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
 * @struct DSU
 * @brief The Disjoint Set Union structure is used in Kruskal's algorithm to manage disjoint sets efficiently.
 */
struct DSU {
    vector<int> parent, rank;

    DSU(int n) : parent(n), rank(n, 0) {
        iota(parent.begin(), parent.end(), 0); // Initialize parent[i] = i
    }

    int find(int x) {
        if (x != parent[x]) parent[x] = find(parent[x]);
        return parent[x];
    }

    void unite(int x, int y) {
        x = find(x), y = find(y);
        if (rank[x] < rank[y]) swap(x, y);
        if (x != y) {
            if (rank[x] == rank[y]) rank[x]++;
            parent[y] = x;
        }
    }
};

/**
 * @brief Implements Kruskal's algorithm to find the Minimum Spanning Tree (MST) in a graph.
 * @param edges List of all edges in the graph, each with a weight and the nodes it connects.
 * @param n The number of nodes in the graph.
 * @return Returns a list of edges that form the MST.
 */
vector<pair<int, pair<int, int>>> kruskal(vector<pair<int, pair<int, int>>> &edges, int n) {
    sort(edges.begin(), edges.end()); // Sort edges based on weight
    DSU dsu(n); 
    vector<pair<int, pair<int, int>>> mst; // To store the edges of the MST

    for (auto &edge : edges) {
        int weight = edge.first;
        int u = edge.second.first;
        int v = edge.second.second;
        if (dsu.find(u) != dsu.find(v)) {
            dsu.unite(u, v); // Join the sets
            mst.push_back(edge); // Include this edge in MST
        }
    }
    return mst;
}

/**
 * @brief Solves the Traveling Salesman Problem (TSP) using dynamic programming.
 * @param start The starting node for the TSP.
 * @param dist The distance matrix representing the graph.
 * @return Returns the minimum cost to visit all nodes and return to the starting node.
 */
int tsp(int start, const vector<vector<int>>& dist) {
    int n = dist.size();
    vector<vector<int>> dp(1 << n, vector<int>(n, 1e9));

    dp[1 << start][start] = 0;

    for (int mask = 0; mask < (1 << n); mask++) {
        for (int i = 0; i < n; i++) {
            if (!(mask & (1 << i))) continue; // Continue if 'i' is not in the subset
            for (int j = 0; j < n; j++) {
                if (mask & (1 << j) || dist[i][j] == INT_MAX) continue; // Continue if 'j' is already in the subset or no direct path
                int nextMask = mask | (1 << j);
                dp[nextMask][j] = min(dp[nextMask][j], dp[mask][i] + dist[i][j]);
            }
        }
    }

    // Return the minimum cost to complete the cycle
    int ans = INT_MAX;
    for (int i = 0; i < n; i++) {
        if (dp[(1 << n) - 1][i] != INT_MAX) {
            ans = min(ans, dp[(1 << n) - 1][i] + dist[i][start]);
        }
    }

    return ans;
}

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
 * @param graph The flow network where the maximum flow needs to be computed. It is a 2D vector where graph[i][j] represents the capacity of the edge from node i to node j.
 * @param s The source node in the flow network.
 * @param t The sink node in the flow network.
 * @return Returns the maximum flow from source to sink in the given flow network.
 */
int edmondsKarp(vector<vi> &graph, int s, int t) {
    vector<vi> residual(sz(graph), vi(sz(graph), 0));

    // Copy the original graph to residual
    fore(u, 0, sz(graph)) {
        fore(v, 0, sz(graph)) {
            residual[u][v] = graph[u][v];
        }
    }

    vi parent(sz(graph));
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


    int n;
    cin >> n;

    // 1. Kruskal
    // Read distance matrix for MST and TSP
    vector<vector<int>> distanceMatrix(n, vector<int>(n));
    vector<pair<int, pair<int, int>>> edges; // For Kruskal's algorithm
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            cin >> distanceMatrix[i][j];
            if (i < j && distanceMatrix[i][j] > 0) {
                edges.push_back({distanceMatrix[i][j], {i, j}});
            }
        }
    }

    // 1. Kruskal's algorithm for MST
    auto mst = kruskal(edges, n);
    cout << "Minimum Spanning Tree (MST):" << endl;
    for (const auto& edge : mst) {
        cout << "Edge from " << edge.second.first << " to " << edge.second.second << " with weight " << edge.first << endl;
    }

    // 2. TSP for shortest mail delivery route
    int shortest_route = tsp(0, distanceMatrix); // Assuming starting from neighborhood 0
    cout << "Shortest route for mail delivery: " << shortest_route << endl;

    // 3. Edmonds-Karp
    vector<vi> graph(n, vi(n));
    for (int u = 0; u < n; u++) {
        for (int v = 0; v < n; v++) {
            cin >> graph[u][v];
        }
    }

    cout << "Maximum Information Flow Value: " << edmondsKarp(graph, 0, n - 1) << endl;

    // 4. Voronoi Diagram

    return 0;
}

// g++-13 -std=c++20 main.cpp && ./a.out < input.txt > output.txt