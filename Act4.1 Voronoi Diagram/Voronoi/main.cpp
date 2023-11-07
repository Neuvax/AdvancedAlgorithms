/**
 * @file main.cpp
 * @author Jorge Germán Wolburg Trujillo -- A01640826
 * @author Armando Terrazas Gómez -- A01640924 
 * @brief Voronoi Diagram to find the nearest toilet in Paris
 * @version 1.0
 * @date 07-11-2023
*/
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string> // This must be defined before including CGAL draw headers
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Delaunay_triangulation_2<K> Delaunay;
typedef K::Point_2 Point;

/**
 * @brief Structure representing a toilet in the csv
 * 
 */
struct Toilet {
    std::string line;
    std::string station;
    std::string accessible;
    std::string tariff;
    std::string passNavigoAccess;
    std::string pushButtonAccess;
    std::string inControlledArea;
    std::string outControlledAreaStation;
    std::string outControlledAreaPublicWay;
    std::string pmrAccessibility; // Accessibilite PMR
    std::string location; // Localisation
    double latitude;
    double longitude;
    std::string manager; // Gestionnaire
};

/**
 * @brief Reads the csv file and returns a vector of toilets
 * 
 * @param filename string with the name of the csv file
 * @return std::vector<Toilet> vector of toilets 
 * 
 * @complexity O(n)
 */
std::vector<Toilet> read_csv(const std::string& filename) {
    std::vector<Toilet> toilets;
    std::ifstream file(filename);
    std::string line;
    // Skip the header
    std::getline(file, line);

    while (std::getline(file, line)) {
        std::istringstream s(line);
        std::string field;
        Toilet toilet;

        std::getline(s, toilet.line, ';');
        std::getline(s, toilet.station, ';');
        std::getline(s, toilet.accessible, ';');
        std::getline(s, toilet.tariff, ';');
        std::getline(s, toilet.passNavigoAccess, ';');
        std::getline(s, toilet.pushButtonAccess, ';');
        std::getline(s, toilet.inControlledArea, ';');
        std::getline(s, toilet.outControlledAreaStation, ';');
        std::getline(s, toilet.outControlledAreaPublicWay, ';');
        std::getline(s, toilet.pmrAccessibility, ';');
        std::getline(s, toilet.location, ';');

        // Parse coordinates
        std::getline(s, field, ';');
        std::istringstream coord_stream(field);
        std::string lat, lng;
        std::getline(coord_stream, lat, ',');
        std::getline(coord_stream, lng, ',');
        toilet.latitude = std::stod(lat);
        toilet.longitude = std::stod(lng);

        std::getline(s, toilet.manager, ';');

        toilets.push_back(toilet);
    }
    return toilets;
}

int main(int argc, char* argv[]) {
    std::string csv_file = "toilets.csv";
    std::vector<Toilet> toilets = read_csv(csv_file);
    
    // Construct Voronoi Diagram
    std::vector<Point> points;
    for (const auto& toilet : toilets) {
        points.push_back(Point(toilet.longitude, toilet.latitude));
    }

    Delaunay dt;
    dt.insert(points.begin(), points.end());

    Point query(2.3522 /* longitude of Paris */, 48.8566 /* latitude of Paris */);
    auto nearest = dt.nearest_vertex(query);
    std::cout << "The nearest toilet is at: "
              << nearest->point().x() << ", " << nearest->point().y() << std::endl;

    return 0;
}