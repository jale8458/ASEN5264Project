#include "RandomObsGen.h"

namespace ObsGenerator2D {

    static std::mt19937 rng(std::random_device{}());

    // Helper functions

    static Eigen::Vector2d randomPoint(const Eigen::Vector2d& minB,
                                       const Eigen::Vector2d& maxB)
    {
        std::uniform_real_distribution<double> dx(minB.x(), maxB.x());
        std::uniform_real_distribution<double> dy(minB.y(), maxB.y());
        return Eigen::Vector2d(dx(rng), dy(rng));
    }

    static Obs2D randomPolygon(const Eigen::Vector2d& center,
                               double scale,
                               int vertexCount)
    {
        Obs2D poly;
        poly.reserve(vertexCount);

        std::vector<double> angles(vertexCount);
        for (int i = 0; i < vertexCount; ++i)
            angles[i] = std::uniform_real_distribution<double>(0.0, 2.0*M_PI)(rng);

        std::sort(angles.begin(), angles.end());

        for (int i = 0; i < vertexCount; ++i)
        {
            double r = std::uniform_real_distribution<double>(0.5*scale, scale)(rng);
            poly.push_back(center + Eigen::Vector2d(r*std::cos(angles[i]),
                                                    r*std::sin(angles[i])));
        }

        return poly;
    }

    static Obs2D randomRectangle(const Eigen::Vector2d& center, double width, double height)
    {
        Obs2D rect(4);

        double hw = width / 2.0;
        double hh = height / 2.0;

        // Define vertices in order (clockwise or counter-clockwise)
        rect[0] = center + Eigen::Vector2d(-hw, -hh);
        rect[1] = center + Eigen::Vector2d(hw, -hh);
        rect[2] = center + Eigen::Vector2d(hw, hh);
        rect[3] = center + Eigen::Vector2d(-hw, hh);

        return rect;
    }

    static Obs2D randomSquare(const Eigen::Vector2d& center, double scale)
    {
        return randomRectangle(center, scale, scale);
    }


    // 2D line segment intersection
    static bool segmentsIntersect(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& q1, const Eigen::Vector2d& q2, const double clearance) {
        // Helper lambda funcs
        auto cross = [](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
            return a.x() * b.y() - a.y() * b.x();
        };
        auto clamp = [](double x, double a, double b) {
            return std::max(a, std::min(x, b));
        };
        auto sqdist_point_segment = [&](const Eigen::Vector2d& a, const Eigen::Vector2d& b, const Eigen::Vector2d& p)
        {
            Eigen::Vector2d ab = b - a;
            double t = (p - a).dot(ab) / ab.squaredNorm();
            t = clamp(t, 0.0, 1.0);
            Eigen::Vector2d proj = a + t * ab;
            return (p - proj).squaredNorm();
        };

        double r2 = clearance * clearance;

        // 1. Check exact segment intersection (no clearance)
        {
            Eigen::Vector2d r = p2 - p1;
            Eigen::Vector2d s = q2 - q1;

            double rxs = cross(r, s);
            double q_pxr = cross(q1 - p1, r);

            if (std::abs(rxs) < 1e-12) {
                // Collinear
                if (std::abs(q_pxr) < 1e-12) {
                    double rdotr = r.dot(r);
                    double t0 = (q1 - p1).dot(r) / rdotr;
                    double t1 = (q2 - p1).dot(r) / rdotr;
                    if (t0 > t1) std::swap(t0, t1);
                    if (!(t1 < 0 || t0 > 1))
                        return true; // exact overlap
                }
            } else {
                double t = cross((q1 - p1), s) / rxs;
                double u = cross((q1 - p1), r) / rxs;
                if (t >= 0 && t <= 1 && u >= 0 && u <= 1)
                    return true;
            }
        }

        // 2. Check capsule intersection (clearance)
        //    distance from endpoints of one segment to the other segment
        if (sqdist_point_segment(p1, p2, q1) <= r2) return true;
        if (sqdist_point_segment(p1, p2, q2) <= r2) return true;
        if (sqdist_point_segment(q1, q2, p1) <= r2) return true;
        if (sqdist_point_segment(q1, q2, p2) <= r2) return true;

        return false;
    }

    static bool polygonIntersectsPath(const Obs2D& poly, const std::vector<Eigen::VectorXd>& path, double clearance) {
        size_t n = poly.size();
        for (size_t i = 0; i < n; ++i)
        {
            Eigen::Vector2d a = poly[i];
            Eigen::Vector2d b = poly[(i+1)%n];
            for (size_t j = 0; j + 1 < path.size(); ++j)
            {
                Eigen::Vector2d p = path[j].head<2>();
                Eigen::Vector2d q = path[j+1].head<2>();
                if (segmentsIntersect(a, b, p, q, clearance))
                    return true;
            }
        }
        return false;
    }

    // RANDOM OBSTACLE GENERATION
    std::vector<Eigen::VectorXd> generateRandomPath(const std::tuple<Eigen::VectorXd, Eigen::VectorXd>& endpoints, const std::tuple<Eigen::VectorXd, Eigen::VectorXd>& bounds, const int numNodes) {
        const Eigen::VectorXd& q_start = std::get<0>(endpoints);
        const Eigen::VectorXd& q_goal  = std::get<1>(endpoints);

        const Eigen::VectorXd& bLow  = std::get<0>(bounds);
        const Eigen::VectorXd& bHigh = std::get<1>(bounds);

        const int dim = q_start.size();

        std::vector<Eigen::VectorXd> path;
        path.reserve(numNodes + 2);
        path.push_back(q_start);

        // build intermediate nodes
        for (int i = 1; i <= numNodes; ++i) {
            double t = double(i) / (numNodes + 1);

            // linear interpolation
            Eigen::VectorXd q = (1.0 - t) * q_start + t * q_goal;

            // random deviation orthogonal to the interpolation direction
            Eigen::VectorXd jitter(dim);
            for (int d = 0; d < dim; ++d) {
                std::uniform_real_distribution<double> dist(
                    -0.3 * (bHigh[d] - bLow[d]), 
                    +0.3 * (bHigh[d] - bLow[d])
                );
                jitter[d] = dist(rng);
            }

            q += jitter;

            // clamp to bounds
            for (int d = 0; d < dim; ++d)
                q[d] = std::max(bLow[d], std::min(q[d], bHigh[d]));

            path.push_back(q);
        }

        path.push_back(q_goal);
        return path;
    }


    std::vector<Obs2D> generateRandomObstacles(const std::tuple<Eigen::VectorXd, 
                                                Eigen::VectorXd>& endpoints, 
                                                const std::tuple<Eigen::VectorXd, 
                                                Eigen::VectorXd>& bounds, 
                                                const int numObstacles, 
                                                const int vertexCount, 
                                                const double scale, 
                                                const double clearance,
                                                ObsGenerator2D::ObstacleShape shape) {
        const Eigen::VectorXd& boundsLow = std::get<0>(bounds);
        const Eigen::VectorXd& boundsHigh = std::get<1>(bounds);

        const std::vector<Eigen::VectorXd> path = ObsGenerator2D::generateRandomPath(endpoints, bounds);
        
        std::vector<Obs2D> obstacles;
        obstacles.reserve(numObstacles);

        Eigen::Vector2d minB = boundsLow.head<2>();
        Eigen::Vector2d maxB = boundsHigh.head<2>();

       while ((int)obstacles.size() < numObstacles)
        {
            Eigen::Vector2d center = randomPoint(minB, maxB);
            Obs2D poly;

            switch (shape)
            {
                case ObstacleShape::Polygon:
                    poly = randomPolygon(center, scale, vertexCount);
                    break;
                case ObstacleShape::Rectangle:
                    {
                        std::uniform_real_distribution<double> dist(0.5 * scale, scale);
                        double width = dist(rng);
                        double height = dist(rng);
                        poly = randomRectangle(center, width, height);
                    }
                    break;
                case ObstacleShape::Square:
                    poly = randomSquare(center, scale);
                    break;
            }

            if (!polygonIntersectsPath(poly, path, clearance))
                obstacles.push_back(poly);
        }

        return obstacles;
    }

}