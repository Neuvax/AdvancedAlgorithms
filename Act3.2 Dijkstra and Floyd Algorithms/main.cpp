/**
 * @file main.cpp
 * @author Jorge Germán Wolburg Trujillo -- A01640826
 * @author Armando Terrazas Gómez -- A01640924
 * @brief  Prints the shortest path from a source node to a destination node in a graph comparing
 *         Dijsktra's and Floyd's algorithms.
 * @version 0.1
 * @date 30-09-2023
 */
// #pragma GCC optimize("Ofast", "unroll-loops", "no-stack-protector", "fast-math")
// #pragma GCC target("avx,avx2,fma")
#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>

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
 * @brief Returns the shortest path from a source node to a destination node in a graph using
 * 
 * @param adjL vector<vector<ii>> of the adjacency list of the graph.
 * @param source lli of the source node.
 * @return vector<lli> of the shortest path from the source node to the destination node.
 * 
 * @complexity O(E*log(V)), where E is the number of edges and V is the number of vertices and auxiliar space O(V).
 */
vector<ld> dijkstraAdjL(vector<vector<pair<lli, ld>>> &adjL, lli source) {
    vector<ld> dist(sz(adjL), 1e18);
    dist[source] = 0;
    priority_queue<pair<lli, ld>, vector<pair<lli, ld>>, greater<pair<lli, ld>>> pq;
    pq.push({0, source});
    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();
        if (d > dist[u]) continue;
        for (auto &v : adjL[u]) {
            ld weight = dist[u] + v.s;
            if (weight < dist[v.f]) {
                dist[v.f] = weight;
                pq.push({dist[v.f], v.f});
            }
        }
    }
    return dist;
}


int main() { _
    //freopen("input.txt", "r", stdin);
    //freopen("output.txt", "w", stdout);
    lli N, E;
    cin >> N >> E;
    //Adjacency list
    vector<vector<pair<lli, ld>>> adjL(N);
    //Adjacency Matrix
    vector<vector<ld>> adjM(N, vector<ld>(N, 1e18*1.0));
    //Read edges
    fore(i, 0, E) {
        lli u, v;
        ld w;
        cin >> u >> v >> w;
        adjL[u].pb({v, w});
        adjM[u][v] = w;
    }

    //Dijkstra's algorithm
    for (lli i = 0; i < N; i++) {
        vector<ld> dist = dijkstraAdjL(adjL, i);
        for (lli j = 0; j < N; j++) {
            cout << "node " << i << " to node " << j << ": " << (dist[j] == 1e18 ? -1 : dist[j]) << endl;
        }
        cout << endl;
    }

    return 0;
}

// g++-13 -std=c++20 main.cpp && ./a.out < input.txt > output.txt   