#include <iostream>
#include <vector>
#include <queue>

using namespace std;

int main()
{
    vector<vector<int>> graph = {
        {1, 2},
        {3, 4},
        {4},
        {5},
        {5},
        {}
    };

    // 假设目标节点为 5
    vector<int> h = {
        7, // node 0
        5, // node 1
        3, // node 2
        4, // node 3
        1, // node 4
        0  // node 5
    };

    int start = 0;
    int goal = 5;

    vector<bool> visited(graph.size(), false);

    priority_queue<
        pair<int, int>,
        vector<pair<int, int>>,
        greater<pair<int, int>>
    > pq;

    pq.push({h[start], start});

    while (!pq.empty())
    {
        auto [score, current] = pq.top();
        pq.pop();

        if (visited[current])
            continue;

        visited[current] = true;

        cout << "Visit node "
             << current
             << ", h = "
             << score
             << "\n";

        if (current == goal)
        {
            cout << "Goal found!\n";
            break;
        }

        for (int neighbor : graph[current])
        {
            if (!visited[neighbor])
            {
                pq.push({h[neighbor], neighbor});
            }
        }
    }

    return 0;
}
