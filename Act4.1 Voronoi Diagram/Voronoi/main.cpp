/**
 * @file main.cpp
 * @author Jorge Germán Wolburg Trujillo -- A01640826
 * @author Armando Terrazas Gómez -- A01640924
 * @brief Voronoi Diagram to find the nearest toilet in Paris
 * @version 1.0
 * @date 07-11-2023
 */
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Delaunay_triangulation_2<K> Delaunay;
typedef K::Point_2 Point;

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
 * @brief Structure representing a toilet in the csv
 *
 */
struct Toilet {
        string line;
        string station;
        string accessible;
        string tariff;
        string passNavigoAccess;
        string pushButtonAccess;
        string inControlledArea;
        string outControlledAreaStation;
        string outControlledAreaPublicWay;
        string pmrAccessibility;  // Accessibilite PMR
        string location;          // Localisation
        double latitude;
        double longitude;
        string manager;  // Gestionnaire
};

/**
 * @brief Reads the csv file and returns a vector of toilets
 *
 * @param filename string with the name of the csv file
 * @return vector<Toilet> vector of toilets
 *
 * @complexity O(n)
 */
vector<Toilet> read_csv(const string& filename) {
    vector<Toilet> toilets;
    ifstream file(filename);
    string line;
    // Skip the header
    getline(file, line);

    while (getline(file, line)) {
        istringstream s(line);
        string field;
        Toilet toilet;

        getline(s, toilet.line, ';');
        getline(s, toilet.station, ';');
        getline(s, toilet.accessible, ';');
        getline(s, toilet.tariff, ';');
        getline(s, toilet.passNavigoAccess, ';');
        getline(s, toilet.pushButtonAccess, ';');
        getline(s, toilet.inControlledArea, ';');
        getline(s, toilet.outControlledAreaStation, ';');
        getline(s, toilet.outControlledAreaPublicWay, ';');
        getline(s, toilet.pmrAccessibility, ';');
        getline(s, toilet.location, ';');

        // Parse coordinates
        getline(s, field, ';');
        istringstream coord_stream(field);
        string lat, lng;
        getline(coord_stream, lat, ',');
        getline(coord_stream, lng, ',');
        toilet.latitude = stod(lat);
        toilet.longitude = stod(lng);

        getline(s, toilet.manager, ';');

        toilets.push_back(toilet);
    }
    return toilets;
}

int main() { _
    string csv_file = "toilets.csv";
    vector<Toilet> toilets = read_csv(csv_file);

    // Construct Voronoi Diagram
    vector<Point> points;
    for (const auto& toilet : toilets) {
        points.push_back(Point(toilet.longitude, toilet.latitude));
    }

    Delaunay dt;
    dt.insert(points.begin(), points.end());

    Point query(2.3522 /* longitude of Paris */, 48.8566 /* latitude of Paris */);
    auto nearest = dt.nearest_vertex(query);
    cout << "The nearest toilet is at: " << nearest->point().x() << ", " << nearest->point().y() << endl;

    return 0;
}

// g++-13 -std=c++20 main.cpp && ./a.out < input.txt > output.txt