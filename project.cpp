#include <SFML/Graphics.hpp>
#include <bits/stdc++.h>
#include <iostream>
#include <vector>
#include <queue>
#include <set>
#include <string>
#include <cmath>
#include <algorithm>
#include <climits>

// Constants for visualization
const int WINDOW_WIDTH = 1920;
const int WINDOW_HEIGHT = 1080;
const float NODE_RADIUS = 10.0f;
const float CLICK_RADIUS = 10.0f;
const float ANIMATION_SPEED = 0.5f;

struct Node
{
    std::string name;
    sf::Vector2f position;

    Node(const std::string &n, float x, float y) : name(n), position(x, y) {}
};

struct Edge
{
    int from;
    int to;
    int weight;

    Edge(int f, int t, int w) : from(f), to(t), weight(w) {}

    bool operator<(const Edge &other) const
    {
        return weight < other.weight;
    }
};
// Add these new structures after the existing Edge struct
struct WaypointWithPriority
{
    int nodeIndex;
    int priority; // Lower number means higher priority (1 is highest)

    WaypointWithPriority(int node, int p) : nodeIndex(node), priority(p) {}

    bool operator<(const WaypointWithPriority &other) const
    {
        return priority < other.priority;
    }
};

struct PriorityOrder
{
    std::vector<WaypointWithPriority> waypoints;
    bool isComplete;

    PriorityOrder() : isComplete(false) {}
};

class PriorityDeliveryVisualizer
{
private:
    sf::RenderWindow window;
    std::vector<Node> nodes;
    std::vector<Edge> edges;
    sf::Font font;

    std::vector<PriorityOrder> orders;
    std::queue<PriorityOrder> activeOrders;
    std::vector<int> currentPath;

    bool animating;
    sf::Clock animationClock;
    float animationProgress;
    int currentSegment;
    sf::CircleShape deliveryMarker;
    int nextPriority; // Track priority for new waypoints

    // Reuse the existing distance calculation methods from DeliveryRouteVisualizer
    std::vector<std::vector<int>> calculateDistanceMatrix()
    {
        int n = nodes.size();
        std::vector<std::vector<int>> dist(n, std::vector<int>(n, INT_MAX));

        for (int i = 0; i < n; i++)
        {
            auto [distances, _] = dijkstra(i);
            for (int j = 0; j < n; j++)
            {
                dist[i][j] = distances[j];
            }
        }
        return dist;
    }
    std::vector<int> reconstructPath(int start, int end, const std::vector<int> &previous) const
    {
        std::vector<int> path;
        for (int at = end; at != -1; at = previous[at])
        {
            path.push_back(at);
        }
        std::reverse(path.begin(), path.end());
        return path;
    }

    std::pair<std::vector<int>, std::vector<int>> dijkstra(int start) const
    {
        std::vector<int> distances(nodes.size(), INT_MAX);
        std::vector<int> previous(nodes.size(), -1);
        std::set<std::pair<int, int>> queue;

        distances[start] = 0;
        queue.insert({0, start});

        while (!queue.empty())
        {
            int current = queue.begin()->second;
            queue.erase(queue.begin());

            for (const auto &edge : edges)
            {
                if (edge.from == current || edge.to == current)
                {
                    int neighbor = (edge.from == current) ? edge.to : edge.from;
                    int newDist = distances[current] + edge.weight;

                    if (newDist < distances[neighbor])
                    {
                        queue.erase({distances[neighbor], neighbor});
                        distances[neighbor] = newDist;
                        previous[neighbor] = current;
                        queue.insert({newDist, neighbor});
                    }
                }
            }
        }
        return {distances, previous};
    }
    std::vector<int> getDetailedPath(const std::vector<int> &highLevelPath)
    {
        std::vector<int> detailedPath;
        if (highLevelPath.size() < 2)
            return highLevelPath;

        for (size_t i = 0; i < highLevelPath.size() - 1; i++)
        {
            auto [_, previous] = dijkstra(highLevelPath[i]);
            auto subPath = reconstructPath(highLevelPath[i], highLevelPath[i + 1], previous);

            if (i > 0)
                subPath.erase(subPath.begin()); // Remove duplicate node except for first segment
            detailedPath.insert(detailedPath.end(), subPath.begin(), subPath.end());
        }
        return detailedPath;
    }

    std::vector<int> findPriorityBasedPath(const std::vector<WaypointWithPriority> &waypoints)
    {
        if (waypoints.empty())
            return {};

        // Sort waypoints by priority
        std::vector<WaypointWithPriority> sortedWaypoints = waypoints;
        std::sort(sortedWaypoints.begin(), sortedWaypoints.end());

        std::vector<int> highLevelPath;
        int currentNode = sortedWaypoints[0].nodeIndex;
        highLevelPath.push_back(currentNode);

        // Process each priority level
        int currentPriority = sortedWaypoints[0].priority;
        std::vector<int> samePriorityNodes;

        for (size_t i = 0; i < sortedWaypoints.size(); i++)
        {
            if (sortedWaypoints[i].priority == currentPriority)
            {
                if (sortedWaypoints[i].nodeIndex != currentNode)
                {
                    samePriorityNodes.push_back(sortedWaypoints[i].nodeIndex);
                }
            }
            else
            {
                // Process nodes of the current priority level
                if (!samePriorityNodes.empty())
                {
                    auto subPath = findShortestPathThroughNodes(currentNode, samePriorityNodes);
                    if (!subPath.empty())
                    {
                        subPath.erase(subPath.begin()); // Remove duplicate node
                        highLevelPath.insert(highLevelPath.end(), subPath.begin(), subPath.end());
                        currentNode = highLevelPath.back();
                    }
                }

                // Start new priority level
                currentPriority = sortedWaypoints[i].priority;
                samePriorityNodes.clear();
                samePriorityNodes.push_back(sortedWaypoints[i].nodeIndex);
            }
        }

        // Process the last group of same-priority nodes
        if (!samePriorityNodes.empty())
        {
            auto subPath = findShortestPathThroughNodes(currentNode, samePriorityNodes);
            if (!subPath.empty())
            {
                subPath.erase(subPath.begin());
                highLevelPath.insert(highLevelPath.end(), subPath.begin(), subPath.end());
            }
        }

        // Convert high-level path to detailed edge-by-edge path
        return getDetailedPath(highLevelPath);
    }
    void drawEdges()
    {
        for (const auto &edge : edges)
        {
            sf::Vertex line[] = {
                sf::Vertex(nodes[edge.from].position, sf::Color::Blue),
                sf::Vertex(nodes[edge.to].position, sf::Color::Blue)};

            if (animating && currentSegment < static_cast<int>(currentPath.size()) - 1)
            {
                if ((edge.from == currentPath[currentSegment] && edge.to == currentPath[currentSegment + 1]) ||
                    (edge.to == currentPath[currentSegment] && edge.from == currentPath[currentSegment + 1]))
                {
                    line[0].color = sf::Color::Red;
                    line[1].color = sf::Color::Red;
                }
            }

            window.draw(line, 2, sf::Lines);

            sf::Text weightText;
            weightText.setFont(font);
            weightText.setString(std::to_string(edge.weight));
            weightText.setCharacterSize(14);
            weightText.setFillColor(sf::Color::White);

            sf::Vector2f midpoint = (nodes[edge.from].position + nodes[edge.to].position) / 2.f;
            weightText.setPosition(midpoint);

            sf::FloatRect textBounds = weightText.getLocalBounds();
            weightText.setOrigin(textBounds.width / 2, textBounds.height / 2);

            window.draw(weightText);
        }
    }
    void updateAnimation()
    {
        if (animating)
        {
            float elapsed = animationClock.getElapsedTime().asSeconds();
            animationProgress += elapsed * ANIMATION_SPEED;
            animationClock.restart();

            if (animationProgress >= 1.0f)
            {
                animationProgress = 0.0f;
                currentSegment++;
                if (currentSegment >= static_cast<int>(currentPath.size()) - 1)
                {
                    animating = false;
                    if (!activeOrders.empty())
                    {
                        activeOrders.pop();
                    }
                }
            }
        }
        else
        {
            if (!orders.empty() && orders.front().isComplete)
            {
                activeOrders.push(orders.front());
                orders.erase(orders.begin());
            }

            if (!animating && !activeOrders.empty())
            {
                const auto &order = activeOrders.front();
                currentPath = findPriorityBasedPath(order.waypoints);

                if (!currentPath.empty())
                {
                    animating = true;
                    animationProgress = 0.0f;
                    currentSegment = 0;
                    animationClock.restart();
                }
                else
                {
                    std::cerr << "Error: No valid path found through waypoints!" << std::endl;
                    activeOrders.pop();
                }
            }
        }
    }
    void drawDeliveryMarker()
    {
        if (animating && currentSegment < static_cast<int>(currentPath.size()) - 1)
        {
            sf::Vector2f startPos = nodes[currentPath[currentSegment]].position;
            sf::Vector2f endPos = nodes[currentPath[currentSegment + 1]].position;
            sf::Vector2f currentPos = startPos + (endPos - startPos) * animationProgress;

            deliveryMarker.setPosition(currentPos - sf::Vector2f(deliveryMarker.getRadius(), deliveryMarker.getRadius()));
            window.draw(deliveryMarker);
        }
    }
    std::vector<int> findShortestPathThroughNodes(int start, const std::vector<int> &nodes)
    {
        if (nodes.empty())
            return {start};

        auto distMatrix = calculateDistanceMatrix();
        std::vector<int> bestPath;
        int shortestDist = INT_MAX;

        // Try all permutations for optimal path through same-priority nodes
        std::vector<int> indices(nodes.size());
        std::iota(indices.begin(), indices.end(), 0);

        do
        {
            int totalDist = 0;
            int current = start;
            bool valid = true;
            std::vector<int> currentPath = {start};

            for (int idx : indices)
            {
                int next = nodes[idx];
                if (distMatrix[current][next] == INT_MAX)
                {
                    valid = false;
                    break;
                }
                totalDist += distMatrix[current][next];
                current = next;
                currentPath.push_back(next);
            }

            if (valid && totalDist < shortestDist)
            {
                shortestDist = totalDist;
                bestPath = currentPath;
            }
        } while (std::next_permutation(indices.begin(), indices.end()));

        return bestPath;
    }

    void handleInput(const sf::Event &event)
    {
        if (event.type == sf::Event::MouseButtonPressed)
        {
            if (event.mouseButton.button == sf::Mouse::Left)
            {
                handleMouseClick(sf::Vector2f(event.mouseButton.x, event.mouseButton.y));
            }
            else if (event.mouseButton.button == sf::Mouse::Right)
            {
                if (!orders.empty() && !orders.back().isComplete)
                {
                    orders.back().isComplete = true;
                    nextPriority = 1; // Reset priority for next order
                }
            }
        }
    }

    void handleMouseClick(const sf::Vector2f &mousePos)
    {
        for (size_t i = 0; i < nodes.size(); ++i)
        {
            float dx = nodes[i].position.x - mousePos.x;
            float dy = nodes[i].position.y - mousePos.y;
            if (std::sqrt(dx * dx + dy * dy) < CLICK_RADIUS)
            {
                if (orders.empty() || orders.back().isComplete)
                {
                    PriorityOrder newOrder;
                    newOrder.waypoints.push_back(WaypointWithPriority(i, nextPriority++));
                    orders.push_back(newOrder);
                }
                else
                {
                    auto &currentOrder = orders.back();
                    bool exists = false;
                    for (const auto &wp : currentOrder.waypoints)
                    {
                        if (wp.nodeIndex == static_cast<int>(i))
                        {
                            exists = true;
                            break;
                        }
                    }
                    if (!exists)
                    {
                        currentOrder.waypoints.push_back(WaypointWithPriority(i, nextPriority++));
                    }
                }
                break;
            }
        }
    }
    // Similar drawing methods as DeliveryRouteVisualizer, but modified for priority visualization
    void drawNodes()
    {
        for (size_t i = 0; i < nodes.size(); ++i)
        {
            sf::CircleShape circle(NODE_RADIUS);
            circle.setPosition(nodes[i].position - sf::Vector2f(NODE_RADIUS, NODE_RADIUS));

            // Default color
            sf::Color nodeColor = sf::Color::Green;

            // Check for nodes in the active order
            if (!activeOrders.empty())
            {
                const auto &currentOrder = activeOrders.front();
                size_t waypointIndex = 0;
                for (const auto &waypoint : currentOrder.waypoints)
                {
                    if (static_cast<int>(i) == waypoint.nodeIndex)
                    {
                        if (waypointIndex == 0)
                        {
                            nodeColor = sf::Color::Yellow;
                        }
                        else if (waypointIndex == currentOrder.waypoints.size() - 1)
                        {
                            nodeColor = sf::Color::Magenta;
                        }
                        else
                        {
                            nodeColor = sf::Color::Cyan;
                        }

                        // Draw priority number
                        sf::Text priorityText;
                        priorityText.setFont(font);
                        priorityText.setString(std::to_string(waypoint.priority));
                        priorityText.setCharacterSize(14);
                        priorityText.setFillColor(sf::Color::White);

                        // Center the priority number on the node
                        sf::FloatRect textBounds = priorityText.getLocalBounds();
                        priorityText.setOrigin(textBounds.width / 2, textBounds.height / 2);
                        priorityText.setPosition(nodes[i].position);

                        window.draw(priorityText);
                        break;
                    }
                    waypointIndex++;
                }
            }
            // Check for nodes in pending orders (not yet active but complete)
            else if (!orders.empty())
            {
                const auto &currentOrder = orders.back();
                if (!currentOrder.isComplete)
                {
                    size_t waypointIndex = 0;
                    for (const auto &waypoint : currentOrder.waypoints)
                    {
                        if (static_cast<int>(i) == waypoint.nodeIndex)
                        {
                            if (waypointIndex == 0)
                            {
                                nodeColor = sf::Color::Yellow;
                            }
                            else if (waypointIndex == currentOrder.waypoints.size() - 1)
                            {
                                nodeColor = sf::Color::Magenta;
                            }
                            else
                            {
                                nodeColor = sf::Color::Cyan;
                            }

                            sf::Text priorityText;
                            priorityText.setFont(font);
                            priorityText.setString(std::to_string(waypoint.priority));
                            priorityText.setCharacterSize(14);
                            priorityText.setFillColor(sf::Color::White);

                            sf::FloatRect textBounds = priorityText.getLocalBounds();
                            priorityText.setOrigin(textBounds.width / 2, textBounds.height / 2);
                            priorityText.setPosition(nodes[i].position);

                            window.draw(priorityText);
                            break;
                        }
                        waypointIndex++;
                    }
                }
            }

            circle.setFillColor(nodeColor);
            window.draw(circle);

            // Draw node name
            sf::Text nodeText;
            nodeText.setFont(font);
            nodeText.setString(nodes[i].name);
            nodeText.setCharacterSize(16);
            nodeText.setFillColor(sf::Color::White);

            sf::FloatRect textBounds = nodeText.getLocalBounds();
            nodeText.setOrigin(textBounds.width / 2, textBounds.height + NODE_RADIUS);
            nodeText.setPosition(nodes[i].position);

            window.draw(nodeText);
        }
    }

    sf::Color hsvToRgb(float hue, float saturation, float value)
    {
        float c = value * saturation;
        float x = c * (1 - std::abs(std::fmod(hue / 60.0f, 2.0f) - 1));
        float m = value - c;

        float r = 0, g = 0, b = 0;

        if (hue >= 0 && hue < 60)
        {
            r = c;
            g = x;
            b = 0;
        }
        else if (hue >= 60 && hue < 120)
        {
            r = x;
            g = c;
            b = 0;
        }
        else if (hue >= 120 && hue < 180)
        {
            r = 0;
            g = c;
            b = x;
        }
        else if (hue >= 180 && hue < 240)
        {
            r = 0;
            g = x;
            b = c;
        }
        else if (hue >= 240 && hue < 300)
        {
            r = x;
            g = 0;
            b = c;
        }
        else
        {
            r = c;
            g = 0;
            b = x;
        }

        return sf::Color(
            static_cast<sf::Uint8>((r + m) * 255),
            static_cast<sf::Uint8>((g + m) * 255),
            static_cast<sf::Uint8>((b + m) * 255));
    }

public:
    PriorityDeliveryVisualizer(const std::vector<Node> &n, const std::vector<Edge> &e) : window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Priority-Based Delivery Route Visualization"),
                                                                                         nodes(n),
                                                                                         edges(e),
                                                                                         animating(false),
                                                                                         animationProgress(0.0f),
                                                                                         currentSegment(0),
                                                                                         nextPriority(1)
    {

        window.setFramerateLimit(60);

        if (!font.loadFromFile("arial.ttf"))
        {
            throw std::runtime_error("Error loading font. Ensure arial.ttf is in the same directory.");
        }

        deliveryMarker.setRadius(8);
        deliveryMarker.setFillColor(sf::Color::Red);
    }

    void run()
    {
        while (window.isOpen())
        {
            sf::Event event;
            while (window.pollEvent(event))
            {
                if (event.type == sf::Event::Closed)
                {
                    window.close();
                }
                else
                {
                    handleInput(event);
                }
            }

            updateAnimation();

            window.clear(sf::Color::Black);
            drawEdges();
            drawNodes();
            drawDeliveryMarker();
            window.display();
        }
    }
};

// DisjointSet class for Kruskal's algorithm
class DisjointSet
{
private:
    std::vector<int> parent;
    std::vector<int> rank;

public:
    DisjointSet(int n)
    {
        parent.resize(n);
        rank.resize(n, 0);
        for (int i = 0; i < n; i++)
        {
            parent[i] = i;
        }
    }

    int find(int x)
    {
        if (parent[x] != x)
        {
            parent[x] = find(parent[x]);
        }
        return parent[x];
    }

    void unite(int x, int y)
    {
        int px = find(x);
        int py = find(y);

        if (rank[px] < rank[py])
        {
            parent[px] = py;
        }
        else if (rank[px] > rank[py])
        {
            parent[py] = px;
        }
        else
        {
            parent[py] = px;
            rank[px]++;
        }
    }
};

// MST Visualizer class
class MSTVisualizer
{
private:
    sf::RenderWindow window;
    std::vector<Node> nodes;
    std::vector<Edge> edges;
    std::vector<Edge> mstEdges;
    sf::Font font;

    void calculateMST()
    {
        // Sort edges by weight
        std::vector<Edge> sortedEdges = edges;
        std::sort(sortedEdges.begin(), sortedEdges.end());

        // Initialize disjoint set
        DisjointSet ds(nodes.size());
        mstEdges.clear();

        // Kruskal's algorithm
        for (const Edge &edge : sortedEdges)
        {
            if (ds.find(edge.from) != ds.find(edge.to))
            {
                mstEdges.push_back(edge);
                ds.unite(edge.from, edge.to);
            }
        }
    }

    void drawEdges()
    {
        for (const auto &edge : mstEdges)
        {
            sf::Vertex line[] = {
                sf::Vertex(nodes[edge.from].position, sf::Color::Blue),
                sf::Vertex(nodes[edge.to].position, sf::Color::Blue)};
            window.draw(line, 2, sf::Lines);

            // Draw edge weights
            sf::Text weightText;
            weightText.setFont(font);
            weightText.setString(std::to_string(edge.weight));
            weightText.setCharacterSize(14);
            weightText.setFillColor(sf::Color::White);

            sf::Vector2f midpoint = (nodes[edge.from].position + nodes[edge.to].position) / 2.f;
            weightText.setPosition(midpoint);

            sf::FloatRect textBounds = weightText.getLocalBounds();
            weightText.setOrigin(textBounds.width / 2, textBounds.height / 2);

            window.draw(weightText);
        }
    }

    void drawNodes()
    {
        for (size_t i = 0; i < nodes.size(); ++i)
        {
            sf::CircleShape circle(NODE_RADIUS);
            circle.setPosition(nodes[i].position - sf::Vector2f(NODE_RADIUS, NODE_RADIUS));
            circle.setFillColor(sf::Color::Green);
            window.draw(circle);

            // Draw node names
            sf::Text nodeText;
            nodeText.setFont(font);
            nodeText.setString(nodes[i].name);
            nodeText.setCharacterSize(16);
            nodeText.setFillColor(sf::Color::White);

            sf::FloatRect textBounds = nodeText.getLocalBounds();
            nodeText.setOrigin(textBounds.width / 2, textBounds.height + NODE_RADIUS);
            nodeText.setPosition(nodes[i].position);

            window.draw(nodeText);
        }
    }

public:
    MSTVisualizer(const std::vector<Node> &n, const std::vector<Edge> &e) : window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Minimum Spanning Tree Visualization"),
                                                                            nodes(n),
                                                                            edges(e)
    {

        window.setFramerateLimit(60);

        if (!font.loadFromFile("arial.ttf"))
        {
            throw std::runtime_error("Error loading font. Ensure arial.ttf is in the same directory.");
        }

        calculateMST();
    }

    void run()
    {
        while (window.isOpen())
        {
            sf::Event event;
            while (window.pollEvent(event))
            {
                if (event.type == sf::Event::Closed)
                {
                    window.close();
                }
            }

            window.clear(sf::Color::Black);
            drawEdges();
            drawNodes();
            window.display();
        }
    }
};

class DeliveryRouteVisualizer
{
private:
    sf::RenderWindow window;
    std::vector<Node> nodes;
    std::vector<Edge> edges;
    sf::Font font;

    struct Order
    {
        std::vector<int> waypoints; // Points to visit in any order
        bool isComplete;

        Order() : isComplete(false) {}
    };

    std::vector<Order> orders;
    std::queue<Order> activeOrders;
    std::vector<int> currentPath;

    bool animating;
    sf::Clock animationClock;
    float animationProgress;
    int currentSegment;
    sf::CircleShape deliveryMarker;

    std::vector<std::vector<int>> calculateDistanceMatrix()
    {
        int n = nodes.size();
        std::vector<std::vector<int>> dist(n, std::vector<int>(n, INT_MAX));

        for (int i = 0; i < n; i++)
        {
            auto [distances, _] = dijkstra(i);
            for (int j = 0; j < n; j++)
            {
                dist[i][j] = distances[j];
            }
        }
        return dist;
    }

    std::vector<int> findOptimalWaypointOrder(const std::vector<int> &waypoints)
    {
        if (waypoints.size() <= 2)
            return waypoints;

        auto distMatrix = calculateDistanceMatrix();
        std::vector<int> optimalOrder;
        std::vector<bool> visited(waypoints.size(), false);

        optimalOrder.push_back(waypoints[0]);
        visited[0] = true;

        while (optimalOrder.size() < waypoints.size())
        {
            int currentNode = optimalOrder.back();
            int nearestDist = INT_MAX;
            int nearestIdx = -1;

            for (size_t i = 0; i < waypoints.size(); i++)
            {
                if (!visited[i])
                {
                    int dist = distMatrix[currentNode][waypoints[i]];
                    if (dist < nearestDist)
                    {
                        nearestDist = dist;
                        nearestIdx = i;
                    }
                }
            }

            if (nearestIdx != -1)
            {
                optimalOrder.push_back(waypoints[nearestIdx]);
                visited[nearestIdx] = true;
            }
        }

        return optimalOrder;
    }

    std::pair<std::vector<int>, std::vector<int>> dijkstra(int start) const
    {
        std::vector<int> distances(nodes.size(), INT_MAX);
        std::vector<int> previous(nodes.size(), -1);
        std::set<std::pair<int, int>> queue;

        distances[start] = 0;
        queue.insert({0, start});

        while (!queue.empty())
        {
            int current = queue.begin()->second;
            queue.erase(queue.begin());

            for (const auto &edge : edges)
            {
                if (edge.from == current || edge.to == current)
                {
                    int neighbor = (edge.from == current) ? edge.to : edge.from;
                    int newDist = distances[current] + edge.weight;

                    if (newDist < distances[neighbor])
                    {
                        queue.erase({distances[neighbor], neighbor});
                        distances[neighbor] = newDist;
                        previous[neighbor] = current;
                        queue.insert({newDist, neighbor});
                    }
                }
            }
        }
        return {distances, previous};
    }

    std::vector<int> findMultiPointPath(const std::vector<int> &waypoints)
    {
        if (waypoints.size() < 2)
            return {};

        std::vector<int> optimizedWaypoints = findOptimalWaypointOrder(waypoints);
        std::vector<int> completePath;

        for (size_t i = 0; i < optimizedWaypoints.size() - 1; i++)
        {
            auto [distances, previous] = dijkstra(optimizedWaypoints[i]);
            std::vector<int> segment = reconstructPath(optimizedWaypoints[i], optimizedWaypoints[i + 1], previous);

            if (segment.empty())
                return {};

            if (!completePath.empty())
                segment.erase(segment.begin());
            completePath.insert(completePath.end(), segment.begin(), segment.end());
        }

        return completePath;
    }

    std::vector<int> reconstructPath(int start, int end, const std::vector<int> &previous) const
    {
        if (previous[end] == -1 && start != end)
        {
            return std::vector<int>{};
        }

        std::vector<int> path;
        for (int at = end; at != -1; at = previous[at])
        {
            path.push_back(at);
        }
        std::reverse(path.begin(), path.end());
        return path;
    }

    void handleInput(const sf::Event &event)
    {
        if (event.type == sf::Event::MouseButtonPressed)
        {
            if (event.mouseButton.button == sf::Mouse::Left)
            {
                handleMouseClick(sf::Vector2f(event.mouseButton.x, event.mouseButton.y));
            }
            else if (event.mouseButton.button == sf::Mouse::Right)
            {
                if (!orders.empty() && !orders.back().isComplete)
                {
                    orders.back().isComplete = true;
                }
            }
        }
    }

    void handleMouseClick(const sf::Vector2f &mousePos)
    {
        for (size_t i = 0; i < nodes.size(); ++i)
        {
            float dx = nodes[i].position.x - mousePos.x;
            float dy = nodes[i].position.y - mousePos.y;
            if (std::sqrt(dx * dx + dy * dy) < CLICK_RADIUS)
            {
                if (orders.empty() || orders.back().isComplete)
                {
                    Order newOrder;
                    newOrder.waypoints.push_back(static_cast<int>(i));
                    orders.push_back(newOrder);
                }
                else
                {
                    auto &currentOrder = orders.back();
                    if (!currentOrder.waypoints.empty() &&
                        currentOrder.waypoints.back() == static_cast<int>(i))
                    {
                        return;
                    }
                    currentOrder.waypoints.push_back(static_cast<int>(i));
                }
                break;
            }
        }
    }

    void updateAnimation()
    {
        if (animating)
        {
            float elapsed = animationClock.getElapsedTime().asSeconds();
            animationProgress += elapsed * ANIMATION_SPEED;
            animationClock.restart();

            if (animationProgress >= 1.0f)
            {
                animationProgress = 0.0f;
                currentSegment++;
                if (currentSegment >= static_cast<int>(currentPath.size()) - 1)
                {
                    animating = false;
                    if (!activeOrders.empty())
                    {
                        activeOrders.pop();
                    }
                }
            }
        }
        else
        {
            processNextOrder();
        }
    }

    void processNextOrder()
    {
        if (!orders.empty() && orders.front().isComplete)
        {
            activeOrders.push(orders.front());
            orders.erase(orders.begin());
        }

        if (!animating && !activeOrders.empty())
        {
            const Order &order = activeOrders.front();
            currentPath = findMultiPointPath(order.waypoints);

            if (!currentPath.empty())
            {
                animating = true;
                animationProgress = 0.0f;
                currentSegment = 0;
                animationClock.restart();
            }
            else
            {
                std::cerr << "Error: No valid path found through waypoints!" << std::endl;
                activeOrders.pop();
            }
        }
    }

    void drawEdges()
    {
        for (const auto &edge : edges)
        {
            sf::Vertex line[] = {
                sf::Vertex(nodes[edge.from].position, sf::Color::Blue),
                sf::Vertex(nodes[edge.to].position, sf::Color::Blue)};

            if (animating && currentSegment < static_cast<int>(currentPath.size()) - 1)
            {
                if ((edge.from == currentPath[currentSegment] && edge.to == currentPath[currentSegment + 1]) ||
                    (edge.to == currentPath[currentSegment] && edge.from == currentPath[currentSegment + 1]))
                {
                    line[0].color = sf::Color::Red;
                    line[1].color = sf::Color::Red;
                }
            }

            window.draw(line, 2, sf::Lines);

            sf::Text weightText;
            weightText.setFont(font);
            weightText.setString(std::to_string(edge.weight));
            weightText.setCharacterSize(14);
            weightText.setFillColor(sf::Color::White);

            sf::Vector2f midpoint = (nodes[edge.from].position + nodes[edge.to].position) / 2.f;
            weightText.setPosition(midpoint);

            sf::FloatRect textBounds = weightText.getLocalBounds();
            weightText.setOrigin(textBounds.width / 2, textBounds.height / 2);

            window.draw(weightText);
        }
    }

    void drawNodes()
    {
        for (size_t i = 0; i < nodes.size(); ++i)
        {
            sf::CircleShape circle(NODE_RADIUS);
            circle.setPosition(nodes[i].position - sf::Vector2f(NODE_RADIUS, NODE_RADIUS));
            circle.setFillColor(sf::Color::Green);

            if (!orders.empty() && !orders.back().isComplete)
            {
                const auto &currentOrder = orders.back();
                size_t waypointIndex = 0;
                for (int waypoint : currentOrder.waypoints)
                {
                    if (static_cast<int>(i) == waypoint)
                    {
                        if (waypointIndex == 0)
                        {
                            circle.setFillColor(sf::Color::Yellow);
                        }
                        else if (waypointIndex == currentOrder.waypoints.size() - 1)
                        {
                            circle.setFillColor(sf::Color::Magenta);
                        }
                        else
                        {
                            circle.setFillColor(sf::Color::Cyan);
                        }
                    }
                    waypointIndex++;
                }
            }

            if (!activeOrders.empty())
            {
                const auto &activeOrder = activeOrders.front();
                size_t waypointIndex = 0;
                for (int waypoint : activeOrder.waypoints)
                {
                    if (static_cast<int>(i) == waypoint)
                    {
                        if (waypointIndex == 0)
                        {
                            circle.setFillColor(sf::Color::Yellow);
                        }
                        else if (waypointIndex == activeOrder.waypoints.size() - 1)
                        {
                            circle.setFillColor(sf::Color::Magenta);
                        }
                        else
                        {
                            circle.setFillColor(sf::Color::Cyan);
                        }
                    }
                    waypointIndex++;
                }
            }

            window.draw(circle);

            sf::Text nodeText;
            nodeText.setFont(font);
            nodeText.setString(nodes[i].name);
            nodeText.setCharacterSize(16);
            nodeText.setFillColor(sf::Color::White);

            sf::FloatRect textBounds = nodeText.getLocalBounds();
            nodeText.setOrigin(textBounds.width / 2, textBounds.height + NODE_RADIUS);
            nodeText.setPosition(nodes[i].position);

            window.draw(nodeText);
        }
    }

    void drawDeliveryMarker()
    {
        if (animating && currentSegment < static_cast<int>(currentPath.size()) - 1)
        {
            sf::Vector2f startPos = nodes[currentPath[currentSegment]].position;
            sf::Vector2f endPos = nodes[currentPath[currentSegment + 1]].position;
            sf::Vector2f currentPos = startPos + (endPos - startPos) * animationProgress;

            deliveryMarker.setPosition(currentPos - sf::Vector2f(deliveryMarker.getRadius(), deliveryMarker.getRadius()));
            window.draw(deliveryMarker);
        }
    }

public:
    DeliveryRouteVisualizer() : window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Optimized Multi-Waypoint Delivery Route Visualization"),
                                animating(false),
                                animationProgress(0.0f),
                                currentSegment(0)
    {

        window.setFramerateLimit(60);

        if (!font.loadFromFile("arial.ttf"))
        {
            throw std::runtime_error("Error loading font. Ensure arial.ttf is in the same directory.");
        }

        deliveryMarker.setRadius(8);
        deliveryMarker.setFillColor(sf::Color::Red);

        // Initialize nodes
        nodes = {
            Node("IndraPuram", 250, 200),
            Node("Daily Needs", 150, 350),
            Node("Lane 1", 500, 200),
            Node("House 1", 500, 350),
            Node("House 3", 500, 450),
            Node("House 5", 500, 550),
            Node("Lane 2", 650, 200),
            Node("House 2", 650, 275),
            Node("House 4", 650, 385),
            Node("House 7", 650, 495),
            Node("Noida", 1350, 125),
            Node("House 8", 1250, 295),
            Node("House 9", 1250, 395),
            Node("House 10", 1250, 495),
            Node("House 11", 1100, 340),
            Node("House 12", 1100, 440),
            Node("House 13", 1100, 540),
            Node("House 14", 1550, 295),
            Node("House 15", 1550, 400),
            Node("House 16", 1550, 500),
            Node("House 17", 1550, 650),
            Node("Store 2", 1450,750),
            Node("WareHouse", 900, 700),
            Node("Distributor", 450, 720)};

        // Initialize edges with weights
        edges = {
            Edge(0, 1, 10),
            Edge(0, 2, 20),
            Edge(2, 3, 7),
            Edge(3, 4, 7),
            Edge(4, 5, 7),
            Edge(2, 6, 10),
            Edge(6, 7, 7),
            Edge(7, 8, 7),
            Edge(8, 9, 7),
            Edge(6, 10, 50),
            Edge(10, 11, 14),
            Edge(10, 21, 30),
            Edge(10, 17, 20),
            Edge(11, 12, 5),
            Edge(12, 13, 5),
            Edge(11, 14, 10),
            Edge(14, 15, 5),
            Edge(15, 16, 5),
            Edge(17, 18, 8),
            Edge(18, 19, 8),
            Edge(19, 20, 8),
            Edge(21, 22, 30),
            Edge(1, 23, 30),
            Edge(23, 22, 25),
            Edge(5, 9, 12),
            Edge(13, 16, 12),
            Edge(9,22,14),
            Edge(14,22,20)};
    }

    void run()
    {
        while (window.isOpen())
        {
            sf::Event event;
            while (window.pollEvent(event))
            {
                if (event.type == sf::Event::Closed)
                {
                    window.close();
                }
                else
                {
                    handleInput(event);
                }
            }

            updateAnimation();

            window.clear(sf::Color::Black);
            drawEdges();
            drawNodes();
            drawDeliveryMarker();
            window.display();
        }
    }
};
// Main Function
int main()
{
    try
    {
        std::vector<Node> nodes = {
            Node("IndraPuram", 250, 200),
            Node("Daily Needs", 150, 350),
            Node("Lane 1", 500, 200),
            Node("House 1", 500, 350),
            Node("House 3", 500, 450),
            Node("House 5", 500, 550),
            Node("Lane 2", 650, 200),
            Node("House 2", 650, 275),
            Node("House 4", 650, 385),
            Node("House 7", 650, 495),
            Node("Noida", 1350, 125),
            Node("House 8", 1250, 295),
            Node("House 9", 1250, 395),
            Node("House 10", 1250, 495),
            Node("House 11", 1100, 340),
            Node("House 12", 1100, 440),
            Node("House 13", 1100, 540),
            Node("House 14", 1550, 295),
            Node("House 15", 1550, 400),
            Node("House 16", 1550, 500),
            Node("House 17", 1550, 650),
            Node("Store 2", 1450, 750),
            Node("WareHouse", 900, 700),
            Node("Distributor",450, 720)};

        std::vector<Edge> edges = {
            Edge(0, 1, 10),
            Edge(0, 2, 20),
            Edge(2, 3, 7),
            Edge(3, 4, 7),
            Edge(4, 5, 7),
            Edge(2, 6, 10),
            Edge(6, 7, 7),
            Edge(7, 8, 7),
            Edge(8, 9, 7),
            Edge(6, 10, 50),
            Edge(10, 11, 14),
            Edge(10, 21, 30),
            Edge(10, 17, 20),
            Edge(11, 12, 5),
            Edge(12, 13, 5),
            Edge(11, 14, 10),
            Edge(14, 15, 5),
            Edge(15, 16, 5),
            Edge(17, 18, 8),
            Edge(18, 19, 8),
            Edge(19, 20, 8),
            Edge(21, 22, 30),
            Edge(1, 23, 30),
            Edge(23, 22, 25),
            Edge(5, 9, 12),
            Edge(13, 16, 12),
            Edge(9,22,14),
            Edge(14,22,20)};

        int choice;
        // In main(), modify the choice menu:
        std::cout << "Choose visualization type:\n";
        std::cout << "1. Delivery Route Visualization\n";
        std::cout << "2. Minimum Spanning Tree\n";
        std::cout << "3. Priority-Based Delivery Route\n";
        std::cout << "Enter your choice (1, 2, or 3): ";
        std::cin >> choice;

        switch (choice)
        {
        case 1:
        {
            DeliveryRouteVisualizer deliveryVisualizer;
            deliveryVisualizer.run();
            break;
        }
        case 2:
        {
            MSTVisualizer mstVisualizer(nodes, edges);
            mstVisualizer.run();
            break;
        }
        case 3:
        {
            PriorityDeliveryVisualizer priorityVisualizer(nodes, edges);
            priorityVisualizer.run();
            break;
        }
        default:
            std::cout << "Invalid choice. Please select 1, 2, or 3.\n";
            return 1;
        }
    }
    catch (const std::exception &ex)
    {
        std::cerr << "Error: " << ex.what() << std::endl;
        return 1;
    }
    return 0;
}