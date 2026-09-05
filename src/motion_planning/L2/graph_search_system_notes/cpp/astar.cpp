#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>

using namespace std;

using Edge = pair<int, int>; // {neighbor, weight}

int main()
{
    vector<vector<Edge>> graph(6);

    graph[0] = {{1, 2}, {2, 4}};
    graph[1] = {{3, 5}, {4, 2}};
    graph[2] = {{4, 1}};
    graph[3] = {{5, 2}};
    graph[4] = {{5, 3}};

    // 对目标节点 5 的启发值
    vector<int> h = {
        6, // node 0
        4, // node 1
        3, // node 2
        2, // node 3
        1, // node 4
        0  // node 5
    };

    int start = 0;
    int goal = 5;

    const int INF = numeric_limits<int>::max();

    vector<int> g(graph.size(), INF);
    vector<int> parent(graph.size(), -1);

    // {f(n), node}
    priority_queue<
        pair<int, int>,
        vector<pair<int, int>>,
        greater<pair<int, int>>
    > open;

    g[start] = 0;
    open.push({g[start] + h[start], start});

    while (!open.empty())
    {
        auto [f, current] = open.top();
        open.pop();

        if (f != g[current] + h[current])
            continue;

        if (current == goal)
            break;

        for (auto [neighbor, weight] : graph[current])
        {
            int newG = g[current] + weight;

            if (newG < g[neighbor])
            {
                g[neighbor] = newG;
                parent[neighbor] = current;

                int newF = newG + h[neighbor];

                open.push({newF, neighbor});
            }
        }
    }

    cout << "Cost = " << g[goal] << "\n";

    vector<int> path;

    for (int node = goal; node != -1; node = parent[node])
        path.push_back(node);

    reverse(path.begin(), path.end());

    cout << "Path: ";

    for (int node : path)
        cout << node << " ";

    cout << "\n";

    return 0;
}
