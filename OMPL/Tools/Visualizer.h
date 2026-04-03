#pragma once

#include "PyCaller.h"
#include "CollisionCheckers.h"
#include <iostream>

#include <ompl/geometric/PathGeometric.h>

class Visualizer {
    public:
        static void Plot2D(const Eigen::VectorXd& q_init, const Eigen::VectorXd& q_goal, ompl::geometric::PathGeometric* path, const std::vector<Obs2D> obstacles);
        static void Plot2D(const ompl::base::ScopedState<>& q_init, ompl::geometric::PathGeometric* path, const std::vector<Obs2D> obstacles);
        static void Plot2D(const ompl::base::ScopedState<>& q_init, const ompl::base::ScopedState<>& q_goal, ompl::geometric::PathGeometric* path, const std::vector<Obs2D> obstacles);
        static void Plot2D(ompl::geometric::PathGeometric* path, const std::vector<Obs2D> obstacles, const double carLength, const double carWidth);
        static void Animate2D(ompl::geometric::PathGeometric* path, const std::vector<Obs2D> obstacles, const double carLength, const double carWidth, const bool saveAnimation = false);
        static void boxplot(const std::vector<std::vector<double>>& data, const std::vector<std::string>& label, const std::string& title, const std::string& xlabel,const std::string& ylabel);
        static void boxplot(const std::vector<std::vector<int>>& data, const std::vector<std::string>& label, const std::string& title, const std::string& xlabel,const std::string& ylabel);
        static void boxplot(const std::vector<std::vector<double>>& data);
        static void barGraph(const std::vector<double>& data, const std::vector<std::string>& labels, const std::string& title, const std::string& xlabel,const std::string& ylabel);
        static void barGraph(const std::vector<double>& data);
        static void showFigures();
    protected:
        static const ompl::base::RealVectorBounds& getBounds(ompl::geometric::PathGeometric* path);
};