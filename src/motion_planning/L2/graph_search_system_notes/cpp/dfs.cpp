#include <iostream>
#include <vector>
#include <stack>

using namespace std;

void dfs(const vector<vector<int>>& graph, int start)
{
    int n = graph.size();
    vector<bool> visited(n, false);
    stack<int> st;

    st.push(start);

    while (!st.empty())
    {
        int current = st.top();
        st.pop();

        if (visited[current])
            continue;

        visited[current] = true;

        cout << current << " ";

        // 逆序压栈可让输出更接近递归版的左到右访问
        for (auto it = graph[current].rbegin();
             it != graph[current].rend();
             ++it)
        {
            if (!visited[*it])
            {
                st.push(*it);
            }
        }
    }
}

int main()
{
    vector<vector<int>> graph = {
        {1, 2},
        {0, 3, 4},
        {0, 5},
        {1},
        {1},
        {2}
    };

    dfs(graph, 0);

    return 0;
}
