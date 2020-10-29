#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (auto neighbor : current_node->neighbors) {
        neighbor->parent = current_node;
        neighbor->h_value = CalculateHValue(neighbor);
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
        neighbor->visited = true;
        open_list.push_back(neighbor);
    }
}


RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), [](RouteModel::Node *n1, RouteModel::Node *n2) {
        auto f1 = n1->g_value + n1->h_value;
        auto f2 = n2->g_value + n2->h_value;
        return f1 > f2;
    });
    auto next_node = open_list.back();
    open_list.pop_back();
    return next_node;
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while (current_node) {
        path_found.push_back(*current_node);
        if (current_node->parent) {
            distance += current_node->distance(*current_node->parent);
        }
        current_node = current_node->parent;
    }
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    open_list.push_back(start_node);
    start_node->visited = true;

    while (auto next_node = NextNode()) {
        current_node = next_node;
        if (current_node == end_node) {
            break;
        }
        AddNeighbors(current_node);
    }

    m_Model.path = ConstructFinalPath(current_node);
}
