#include <iostream>
#include <vector>
#include <stack>
#include <queue>

using namespace std;

const char nodes[] = { 'A', 'B', 'C', 'D', 'E', 'F', 'G' };

enum TypeOfTraversal
{
    dfsRecursive,
    dfsIterative,
    bfs
};

class Graph
{
    private:
        vector<int>* adjacencyList;
        int numberOfNodes;
    private:
        void DFSRecursive(int node, bool* visited);
        void DFSIterative(int node, bool* visited);
        int* BFS(int node, bool* visited);
    public:
        Graph(int numberOfNodes);
        void addNode(int node, int adjacentNode);
        void traverse(int nodeStart, TypeOfTraversal type);
        vector<int> getNodes(int node);
        vector<int> findShortestPath(int start, int end);
};

Graph::Graph(int numberOfNodes)
{
    this->numberOfNodes = numberOfNodes;
    adjacencyList = new vector<int>[numberOfNodes];
}

void Graph::addNode(int node, int adjacentNode)
{
    adjacencyList[node].push_back(adjacentNode);
}

vector<int> Graph::getNodes(int node)
{
    return adjacencyList[node];
}

void Graph::DFSRecursive(int node, bool *visited)
{
    visited[node] = true;

    cout << nodes[node] << " ";

    vector<int> nodes_adj = adjacencyList[node];

    for (int i = 0; i < nodes_adj.size(); i++)
    {
        if (!visited[nodes_adj.at(i)]) 
        {
            DFSRecursive(nodes_adj.at(i), visited);
        }
    }
}

void Graph::DFSIterative(int node, bool* visited)
{
    stack<int> stack;

    stack.push(node);

    visited[node] = true;

    while (!stack.empty())
    {
        int n = stack.top();

        cout << nodes[n] << " ";

        stack.pop();

        vector<int> nodes_adj = adjacencyList[n];

        for (int i = 0; i < nodes_adj.size(); i++)
        {
            if (!visited[nodes_adj[i]])
            {
                stack.push(nodes_adj[i]);

                visited[nodes_adj[i]] = true;
            }
        }
    }
}

int* Graph::BFS(int node, bool* visited) 
{
    queue<int> q;

    q.push(node);

    visited[node] = true;

    int* nodesPrevious = new int[numberOfNodes];

    for (int i = 0; i < numberOfNodes; i++) 
    {
        nodesPrevious[i] = -1;
    }

    while (!q.empty())
    {
        int n = q.front();

        cout << nodes[n] << " ";

        q.pop();

        vector<int> nodes_adj = adjacencyList[n];

        for (int i = 0; i < nodes_adj.size(); i++)
        {
            if (!visited[nodes_adj[i]])
            {
                q.push(nodes_adj[i]);

                visited[nodes_adj[i]] = true;

                nodesPrevious[nodes_adj[i]] = n;
            }
        }
    }

    cout << endl;

    return nodesPrevious;
}

vector<int> Graph::findShortestPath(int start, int end)
{
    bool* visited = new bool[numberOfNodes];

    for (int i = 0; i < numberOfNodes; i++)
    {
        visited[i] = false;
    }

    int* nodesPrev = BFS(start, visited);

    vector<int> path;

    for (int i = end; i != -1; i = nodesPrev[i]) 
    {
        path.push_back(i);
    }

    delete[] visited;
    delete[] nodesPrev;

    return path;
}

void Graph::traverse(int node, TypeOfTraversal type)
{
    bool* visited = new bool[numberOfNodes];

    for (int i = 0; i < numberOfNodes; i++)
    {
        visited[i] = false;
    }

    switch (type)
    {
        case dfsRecursive:
            DFSRecursive(node, visited);
            break;
        case dfsIterative:
            DFSIterative(node, visited);
            break;
        case bfs:
            BFS(node, visited);
            break;
        default:
            cout << "No such traversal type" << endl;
            break;
    }

    cout << endl;

    delete[] visited;
}

int main()
{
    // 0 A -> [C, F]
    // 1 B -> [C, E]
    // 2 C -> [A, B, E, D]
    // 3 D -> [C, F]
    // 4 E -> [B, F]
    // 5 F -> [A, D, E, G]
    // 6 G -> [F]

    Graph graph(7);

    graph.addNode(0, 2);
    graph.addNode(0, 5);
    graph.addNode(1, 2);
    graph.addNode(1, 4);
    graph.addNode(2, 0);
    graph.addNode(2, 1);
    graph.addNode(2, 4);
    graph.addNode(2, 3);
    graph.addNode(3, 2);
    graph.addNode(3, 5);
    graph.addNode(4, 1);
    graph.addNode(4, 5);
    graph.addNode(5, 0);
    graph.addNode(5, 3);
    graph.addNode(5, 4);
    graph.addNode(5, 6);
    graph.addNode(6, 5);

    for (int i = 0; i < 7; i++)
    {
        cout << nodes[i] << " -> ";

        for (int j = 0; j < graph.getNodes(i).size(); j++)
        {
            cout << nodes[graph.getNodes(i).at(j)] << " ";
        }

        cout << endl;
    }

    cout << "DFS recursive traversal:" << endl;
    graph.traverse(0, dfsRecursive);

    cout << "DFS iterative traversal:" << endl;
    graph.traverse(0, dfsIterative);

    cout << "BFS:" << endl;
    graph.traverse(0, bfs);

    int start = 1;
    int end = 5;

    vector<int> path = graph.findShortestPath(start, end);

    cout << "Shortest path from " << nodes[start] << " to " << nodes[end] << endl;

    for (auto ir = path.rbegin(); ir != path.rend(); ++ir)
    {
        cout << nodes[*ir] << " ";
    }
}

