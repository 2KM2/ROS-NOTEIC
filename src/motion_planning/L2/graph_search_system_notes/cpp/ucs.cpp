#include <iostream>
#include <vector>
#include <queue>
#include <limits>

using namespace std;

using Edge = pair<int, int>; // {neighbor, weight}

int main()
{
    vector<vector<Edge>> graph(5);

    graph[0] = {{1, 2}, {2, 5}};
    graph[1] = {{2, 1}, {3, 4}};
    graph[2] = {{3, 1}, {4, 7}};
    graph[3] = {{4, 3}};

    int start = 0;
    int goal = 4;

    const int INF = numeric_limits<int>::max();

    vector<int> dist(graph.size(), INF);
    vector<int> parent(graph.size(), -1);

    priority_queue<
        pair<int, int>,
        vector<pair<int, int>>,
        greater<pair<int, int>>
    > pq;

    dist[start] = 0;
    pq.push({0, start});

    while (!pq.empty())
    {
        auto [cost, current] = pq.top();
        pq.pop();

        if (cost != dist[current])
            continue;

        if (current == goal)
            break;

        for (auto [neighbor, weight] : graph[current])
        {
            int newCost = cost + weight;

            if (newCost < dist[neighbor])
            {
                dist[neighbor] = newCost;
                parent[neighbor] = current;

                pq.push({newCost, neighbor});
            }
        }
    }

    cout << "Minimum cost = " << dist[goal] << "\n";

    vector<int> path;

    for (int node = goal; node != -1; node = parent[node])
        path.push_back(node);

    cout << "Path: ";

    for (auto it = path.rbegin(); it != path.rend(); ++it)
        cout << *it << " ";

    cout << "\n";

    return 0;
}
