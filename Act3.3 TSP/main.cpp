/**
 * @file main.cpp
 * @author Jorge Germán Wolburg Trujillo -- A01640826
 * @author Armando Terrazas Gómez -- A01640924 
 * @brief The Salesman Problem solution using Nearest Neighbor
 * @version 0.1
 * @date 05-10-2023
*/
//#pragma GCC optimize("Ofast", "unroll-loops", "no-stack-protector", "fast-math")
//#pragma GCC target("avx,avx2,fma")
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
#define deb(x) cout << #x": " << (x) << endl
#define print(x) cout << (x) << endl
#define fore(i, a, b) for(lli i = (a), BS = (b); i < BS; ++i)
#define _ ios_base::sync_with_stdio(0); cin.tie(0); cout.tie(0);

/**
 * @brief City class
 * 
 */
class City {
    private:
        lli id;
        lli x;
        lli y;
    public:
        /**
         * @brief Construct a new City object
         * 
         * @param id 
         * @param x 
         * @param y 
         */
        City(lli id = -1, lli x = -1, lli y = -1) {
            this->id = id;
            this->x = x;
            this->y = y;
        }

        /**
         * @brief Get the Distance To city
         * 
         * @param c City to calculate distance
         * @return lli distance between cities 
         */
        ld getDistanceTo(const City &c) {
            //Calculate square distance between cities
            lli d1 = (this->x - c.x) * (this->x - c.x) + (this->y - c.y) * (this->y - c.y);
            return sqrt(d1);
        }
};

/**
 * @brief Nearest Neighbor algorithm
 * 
 * @param currentCity to calculate nearest neighbor  
 * @param cities vector of cities 
 * @param visitedCities vector of visited cities 
 * @return pair<lli, ld> nearest neighbor and distance 
 * @note Complexity: O(n), where n is the number of cities. Space O(n)
 */
pair<lli, ld> nearestNeighbor(City currentCity, vector<City> &cities, vector<bool> &visitedCities) {
    ld minDistance = numeric_limits<ld>::max();
    lli nearestCity = -1;
    fore(i, 0, sz(cities)) {
        if (visitedCities[i]) continue;
        ld distance = currentCity.getDistanceTo(cities[i]);
        if (distance < minDistance) {
            minDistance = distance;
            nearestCity = i;
        }
    }
    return make_pair(nearestCity, minDistance);
}

int main() { _
    //freopen("input.txt", "r", stdin);
    //freopen("output.txt", "w", stdout);
    lli n;
    cin >> n;
    vector<City> cities(n);
    fore(i, 0, n) {
        lli id, x, y;
        cin >> id >> x >> y;
        cities[i] = City(i, x, y);
    }

    vector<bool> visitedCities(n, false);
    ld totalDistance = 0;
    lli currentCity = 0;
    visitedCities[currentCity] = true;
    fore(i, 0, n - 1) {
        pair<lli, ld> nearestCity = nearestNeighbor(cities[currentCity], cities, visitedCities);
        currentCity = nearestCity.f;
        totalDistance += nearestCity.s;
        visitedCities[currentCity] = true;
    }

    totalDistance += cities[currentCity].getDistanceTo(cities[0]);
    cout << fixed << setprecision(8) << totalDistance << endl;

    return 0;
}

//g++-13 -std=c++20 main.cpp && ./a.out < input.txt > output.txt