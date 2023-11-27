/**
 * @file main.cpp
 * @author Jorge Germán Wolburg Trujillo -- A01640826
 * @author Armando Terrazas Gómez -- A01640924
 * @brief This file contains the implementation of the Integrated Activity 2.
 * @date 28-11-2023
 */
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>

#include <algorithm>
#include <iostream>
#include <limits>
#include <numeric>
#include <queue>
#include <tuple>
#include <vector>

using namespace std;

typedef long double ld;
typedef long long lli;
typedef pair<lli, lli> ii;
typedef vector<int> vi;
typedef tuple<int, int, int> Edge;  // destination, capacity, reverse edge index
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Triangulation_vertex_base_2<K> Vb;
typedef CGAL::Triangulation_face_base_with_info_2<vector<K::Point_2>, K> Fb;
typedef CGAL::Triangulation_data_structure_2<Vb,Fb> Tds;
typedef CGAL::Delaunay_triangulation_2<K, Tds> Delaunay;
typedef K::Point_2 Point;

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
 * Time Complexity: 
 *   - find: Nearly constant time, O(log n) in the worst case.
 *   - unite: Nearly constant time, O(log n) in the worst case.
 * Space Complexity: O(n) for storing parent and rank arrays.
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
 * Time Complexity: O(E log E), where E is the number of edges.
 *   - O(E log E) for sorting the edges.
 *   - The union-find operations have a negligible impact on the overall time complexity.
 * Space Complexity: O(E) for storing the edges of the MST and O(n) for DSU.
 */
vector<pair<int, pair<int, int>>> kruskal(vector<pair<int, pair<int, int>>> &edges, int n) {
    sort(all(edges)); // Sort edges based on weight
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
 * Time Complexity: O(n^2 * 2^n)
 *   - There are 2^n subsets of nodes, and for each subset, we consider n nodes to find the minimum path.
 * Space Complexity: O(n * 2^n)
 *   - The dp table has dimensions 2^n by n, storing the minimum cost for each subset and each ending node.
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
 * @brief Performs a Breadth-First Search (BFS) on a residual graph.
 * 
 * This function performs a BFS to find an augmenting path in the residual graph from the source node (s) to the sink node (t). 
 * It updates the 'parent' vector with the path information. If an augmenting path is found, the function returns true.
 * 
 * @param s The source node in the residual graph.
 * @param t The sink node in the residual graph.
 * @param residual The residual graph, represented as a vector of vectors of Edge structures. 
 *                 Each Edge structure contains the destination node, the capacity of the edge, and the reverse edge index.
 * @param parent A vector to store the parent node and edge index for each node. Used to reconstruct the augmenting path.
 * @return Returns true if an augmenting path from source to sink is found. Otherwise, returns false.
 * 
 * @note Time Complexity: O(V + E), where V is the number of vertices and E is the number of edges in the graph.
 */
bool bfs(int s, int t, vector<vector<Edge>> &residual, vector<pair<int, int>> &parent) {
    vector<bool> visited(residual.size(), false);
    queue<int> q;
    q.push(s);
    visited[s] = true;
    parent[s] = {-1, -1};

    while (!q.empty()) {
        int u = q.front();
        q.pop();

        for (int i = 0; i < residual[u].size(); ++i) {
            auto [v, capacity, b] = residual[u][i];
            if (!visited[v] && capacity > 0) {
                parent[v] = {u, i};
                if (v == t) return true;
                q.push(v);
                visited[v] = true;
            }
        }
    }
    return false;
}

/**
 * @brief Implements the Edmonds-Karp algorithm to find the maximum flow in a flow network.
 * 
 * This function calculates the maximum flow from the source node (s) to the sink node (t) in a flow network using the Edmonds-Karp algorithm.
 * It iteratively finds augmenting paths using BFS and updates the flow accordingly until no more augmenting paths are found.
 * 
 * @param residual The flow network represented as a vector of vectors of Edge structures.
 *                 Each Edge structure contains the destination node, the capacity of the edge, and the reverse edge index.
 * @param s The source node in the flow network.
 * @param t The sink node in the flow network.
 * @return Returns the maximum flow from source to sink in the given flow network.
 * 
 * @note Time Complexity: O(V * E^2), where V is the number of vertices and E is the number of edges in the flow network.
 */
int edmondsKarp(vector<vector<Edge>> &residual, int s, int t) {
    vector<pair<int, int>> parent(residual.size());
    int maxFlow = 0;

    while (bfs(s, t, residual, parent)) {
        int pathFlow = numeric_limits<int>::max();
        for (int v = t; v != s; v = parent[v].first) {
            int u = parent[v].first;
            int edgeIndex = parent[v].second;
            auto [aa, capacity, bb] = residual[u][edgeIndex];
            pathFlow = min(pathFlow, capacity);
        }

        for (int v = t; v != s; v = parent[v].first) {
            int u = parent[v].first;
            int edgeIndex = parent[v].second;
            auto [aa, bb, revIndex] = residual[u][edgeIndex];

            get<1>(residual[u][edgeIndex]) -= pathFlow;  // Reduce capacity of forward edge
            get<1>(residual[v][revIndex]) += pathFlow;   // Increase capacity of reverse edge
        }

        maxFlow += pathFlow;
    }

    return maxFlow;
}

/**
 * @brief Converts an adjacency matrix to an adjacency list.
 *
 * This function takes an adjacency matrix as input and converts it to an adjacency list. 
 * It also adds a reverse edge with zero capacity for each edge in the graph.
 *
 * @param matrix The adjacency matrix to be converted. It is a 2D vector where matrix[i][j] represents the capacity of the edge from node i to node j.
 * @return Returns the adjacency list representation of the graph. It is a vector of vectors where each inner vector represents the edges from a node. Each edge is represented by an Edge object, which contains the destination node, the capacity of the edge, and the index of the reverse edge in the destination node's adjacency list.
 *
 * @note The time complexity for this function is O(V^2), where V is the number of vertices in the graph.
 */
vector<vector<Edge>> convertToAdjList(const vector<vi> &matrix) {
    int n = matrix.size();
    vector<vector<Edge>> adjList(n);

    for (int u = 0; u < n; ++u) {
        for (int v = 0; v < n; ++v) {
            if (matrix[u][v] > 0) {
                adjList[u].emplace_back(v, matrix[u][v], adjList[v].size());
                // Add a reverse edge with zero capacity
                adjList[v].emplace_back(u, 0, adjList[u].size() - 1);
            }
        }
    }

    return adjList;
}

/**
 * @brief Extracts and outputs the vertices of Voronoi cells from a Delaunay triangulation.
 *
 * This function iterates over all the faces of a Delaunay triangulation and calculates the circumcenter
 * of each triangle. These circumcenters serve as the vertices of the corresponding Voronoi cells.
 * The function skips over infinite faces to avoid unbounded cells. Each circumcenter is outputted as a vertex.
 *
 * @param dt A reference to the Delaunay triangulation from which Voronoi cells are to be extracted.
 * 
 * @note The time complexity of constructing the Delaunay Triangulation is O(n log n) for n points.
 *       The time complexity of this function is O(n), where n is the number of faces in the Delaunay triangulation.
 *       This is because the function iterates over all faces and performs a constant-time operation (circumcenter calculation)
 *       for each face.
 */
void extract_voronoi_cells(Delaunay& dt) {
    for (auto fi = dt.all_faces_begin(); fi != dt.all_faces_end(); ++fi) {
        if (dt.is_infinite(fi)) {
            continue; // Skip infinite faces
        }

        // Get the circumcenter of the Delaunay triangle
        Point circumcenter = dt.circumcenter(fi);

        // Output the circumcenter as a single point of the Voronoi cell
        cout << "Voronoi Cell Vertex: ";
        cout << "(" << circumcenter.x() << ", " << circumcenter.y() << ")" << endl;
    }
}

int main() { _
    int n;
    cin >> n;

    // 1. Kruskal
    // Read distance matrix for MST and TSP
    vector<vi> distanceMatrix(n, vi(n));
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

    vector<vector<Edge>> adjList = convertToAdjList(graph);

    cout << "Maximum Information Flow Value: " << edmondsKarp(adjList, 0, sz(adjList) - 1) << endl;

    // 4. Voronoi Diagram
    vector<Point> points(n);
    for (int i = 0; i < n; i++) {
        string s; //(x, y)
        cin >> s;
        // Create substring without parentheses and split by comma
        string x = s.substr(1, s.find(',') - 1);
        string y = s.substr(s.find(',') + 1, s.size() - 2);
        points[i] = Point(stold(x), stold(y));
    }

    Delaunay dt;
    dt.insert(points.begin(), points.end());

    extract_voronoi_cells(dt);

    return 0;
}

// g++-13 -std=c++20 main.cpp && ./a.out < input.txt > output.txt
// /opt/homebrew/bin/cgal_create_CMakeLists -s main && cmake -DCMAKE_BUILD_TYPE=Release . && make && ./main < input.txt > output.txt