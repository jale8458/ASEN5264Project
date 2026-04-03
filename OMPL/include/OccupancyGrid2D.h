#ifndef OCCUPANCY_GRID_2D_H
#define OCCUPANCY_GRID_2D_H

#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <map>
#include <Eigen/Dense>

class OccupancyGrid2D
{
public:
    using Polygon = std::vector<Eigen::Vector2d>;

    // Load polygons from CSV file (grouped by obstacle ID)
    static std::vector<Polygon> loadPolygonsFromCSV(const std::string& filepath)
    {
        std::ifstream file(filepath);
        if (!file.is_open()) {
            std::cerr << "Failed to open file: " << filepath << std::endl;
            return {};
        }

        std::string line;
        std::map<int, Polygon> polygonsByID;

        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string token;

            int obs_id;
            double x, y;

            std::getline(ss, token, ',');
            obs_id = std::stoi(token);

            std::getline(ss, token, ',');
            x = std::stod(token);

            std::getline(ss, token, ',');
            y = std::stod(token);

            polygonsByID[obs_id].emplace_back(x, y);
        }

        std::vector<Polygon> polygons;
        for (auto& [id, polygon] : polygonsByID) {
            polygons.push_back(std::move(polygon));
        }
        return polygons;
    }

    // Convert Obs2D (vector<vector<Eigen::Vector2d>>) to vector<Polygon>
    template<typename Obs2D>
    static std::vector<Polygon> convertObs2DToPolygons(const std::vector<Obs2D>& obstacles)
    {
        std::vector<Polygon> polygons;
        polygons.reserve(obstacles.size());
        
        for (const auto& obs : obstacles) {
            Polygon poly;
            poly.reserve(obs.size());
            for (const auto& vertex : obs) {
                poly.push_back(vertex);
            }
            polygons.push_back(poly);
        }
        
        return polygons;
    }

    // Ray casting point in polygon test
    static bool pointInPolygon(const Eigen::Vector2d& point, const Polygon& polygon)
    {
        bool inside = false;
        int n = polygon.size();

        for (int i = 0, j = n - 1; i < n; j = i++) {
            const auto& pi = polygon[i];
            const auto& pj = polygon[j];

            if (((pi.y() > point.y()) != (pj.y() > point.y())) &&
                (point.x() < (pj.x() - pi.x()) * (point.y() - pi.y()) / (pj.y() - pi.y()) + pi.x())) {
                inside = !inside;
            }
        }
        return inside;
    }

    // Create occupancy grid (1=free, 0=occupied)
    static std::vector<int> createOccupancyGrid(
        const std::vector<Polygon>& polygons,
        const Eigen::Vector2d& xBounds,
        const Eigen::Vector2d& yBounds,
        int gridSize)
    {
        std::vector<int> grid(gridSize * gridSize, 1);  // init free

        // Compute cell edges
        std::vector<double> xEdges(gridSize + 1), yEdges(gridSize + 1);
        for (int i = 0; i <= gridSize; ++i) {
            xEdges[i] = xBounds.x() + i * (xBounds.y() - xBounds.x()) / gridSize;
            yEdges[i] = yBounds.x() + i * (yBounds.y() - yBounds.x()) / gridSize;
        }

        for (int r = 0; r < gridSize; ++r) {
            for (int c = 0; c < gridSize; ++c) {
                // Test points: corners + center
                std::vector<Eigen::Vector2d> testPoints = {
                    {xEdges[c], yEdges[r]},               // bottom-left
                    {xEdges[c+1], yEdges[r]},             // bottom-right
                    {xEdges[c], yEdges[r+1]},             // top-left
                    {xEdges[c+1], yEdges[r+1]},           // top-right
                    {0.5 * (xEdges[c] + xEdges[c+1]), 0.5 * (yEdges[r] + yEdges[r+1])} // center
                };

                // If any test point inside any polygon, mark occupied
                bool occupied = false;
                for (const auto& pt : testPoints) {
                    for (const auto& poly : polygons) {
                        if (pointInPolygon(pt, poly)) {
                            occupied = true;
                            break;
                        }
                    }
                    if (occupied) break;
                }

                grid[r * gridSize + c] = occupied ? 0 : 1;
            }
        }
        return grid;
    }

    // Convenience function: create occupancy grid directly from Obs2D obstacles
    template<typename Obs2D>
    static std::vector<int> createOccupancyGridFromObs2D(
        const std::vector<Obs2D>& obstacles,
        const Eigen::Vector2d& xBounds,
        const Eigen::Vector2d& yBounds,
        int gridSize)
    {
        auto polygons = convertObs2DToPolygons(obstacles);
        return createOccupancyGrid(polygons, xBounds, yBounds, gridSize);
    }
};

#endif // OCCUPANCY_GRID_2D_H