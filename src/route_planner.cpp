#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) 
{
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    start_node = &(m_Model.FindClosestNode(start_x, start_y));
    end_node = &(m_Model.FindClosestNode(end_x, end_y));

}


// 3: Implement the CalculateHValue method.
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) 
{
    return node->distance(*end_node);
}


// 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) 
{
    current_node->FindNeighbors();
    
    for(RouteModel::Node* &node : current_node->neighbors)
    {
        node->h_value = CalculateHValue(node);
        node->g_value = current_node->g_value + current_node->distance(*node);
        node->parent = current_node;
        node->visited = true;
        open_list.emplace_back(node);
    }

}


// 5: Complete the NextNode method to sort the open list and return the next node.
RouteModel::Node *RoutePlanner::NextNode() 
{
    std::sort(open_list.begin(), open_list.end(),
    [](RouteModel::Node* a, RouteModel::Node* b){ return (a->h_value + a->g_value) < (b->h_value + b->g_value); });

    RouteModel::Node *lowest_sum = open_list[0];

    open_list.erase(open_list.begin());

    return lowest_sum;
}


// 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) 
{
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    std::vector<RouteModel::Node> path;

    // TODO: Implement your solution here.
    RouteModel::Node *node = current_node;
    path.emplace_back(*node);

    //while(node->g_value != 0)
    while(node->distance(*start_node) != 0)
    { 
        distance += node->distance(*node->parent);  
        node = node->parent;
        path.emplace_back(*node);
    }
    
    std::reverse(path.begin(), path.end());
    path_found = path;

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// 7: Write the A* Search algorithm here.
void RoutePlanner::AStarSearch() 
{
    RouteModel::Node *current_node = nullptr;

    start_node->h_value = CalculateHValue(start_node);
    current_node = start_node;
    current_node->visited = true;
    float remaining_dist = 1;

    while(remaining_dist != 0)
    {   

        AddNeighbors(current_node);
        current_node = NextNode();
        remaining_dist = current_node->distance(*end_node);
    }

    m_Model.path = ConstructFinalPath(current_node);
    open_list.clear();
}