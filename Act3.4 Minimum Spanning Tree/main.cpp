/**
 * @file main.cpp
 * @author Jorge Germán Wolburg Trujillo -- A01640826
 * @author Armando Terrazas Gómez -- A01640924 
 * @brief Minimum Spanning Tree (MST) using Kruskal's algorithm
 * @version 0.1
 * @date 14-10-2023
*/
//#pragma GCC optimize("Ofast", "unroll-loops", "no-stack-protector", "fast-math")
//#pragma GCC target("avx,avx2,fma")
#include <bits/stdc++.h>

using namespace std;

typedef long long lli;
typedef pair<lli, pair<lli, lli>> edge;

#define f first
#define s second
#define pb push_back
#define _ ios_base::sync_with_stdio(0); cin.tie(0); cout.tie(0);

/**
 * @class DSU
 * @brief Data Structure for Disjoint Set Union operations.
 */
class DSU {
private:
    vector<lli> parent, rank;
public:
    DSU(lli n) {
        rank.resize(n, 0);
        parent.resize(n);
        for(lli i = 0; i < n; ++i) {
            parent[i] = i;
        }
    }

    lli find(lli x) {
        if(parent[x] != x) {
            parent[x] = find(parent[x]);
        }
        return parent[x];
    }
    
    /**
     * @brief Merges two sets together.
     * @param x First node.
     * @param y Second node.
     */
    void unionSet(lli x, lli y) {
        lli rootX = find(x);
        lli rootY = find(y);

        if(rootX != rootY) {
            if(rank[rootX] < rank[rootY]) {
                parent[rootX] = rootY;
            } else if(rank[rootX] > rank[rootY]) {
                parent[rootY] = rootX;
            } else {
                parent[rootY] = rootX;
                rank[rootX]++;
            }
        }
    }
};

int main() { _
    //freopen("input.txt", "r", stdin);
    //freopen("output.txt", "w", stdout);

    lli n, m, a, b, w, mstWeight = 0;
    cin >> n >> m;

    vector<edge> edges;
    lli minVertex = INT_MAX;  // to detect the starting vertex (either 0 or 1)
    for(lli i = 0; i < m; i++) {
        cin >> a >> b >> w;
        edges.pb({w, {a, b}});
        minVertex = min(minVertex, min(a, b));
    }

    sort(edges.begin(), edges.end());

    DSU dsu(n + 1);

   for(auto e : edges) {
    lli u = e.s.f, v = e.s.s;
    if(minVertex == 1) {
        u -= 1;
        v -= 1;
    }
    if(dsu.find(u) != dsu.find(v)) {
        mstWeight += e.f;
        dsu.unionSet(u, v);
    }
}

    cout << mstWeight << endl;

    return 0;
}