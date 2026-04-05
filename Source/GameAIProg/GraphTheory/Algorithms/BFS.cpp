#include "BFS.h"

#include <map>
#include <queue>
#include <algorithm>

#include "Shared/Graph/Graph.h"

using namespace GameAI;

BFS::BFS(Graph* const pGraph)
    : pGraph(pGraph)
{
}

std::vector<Node*> BFS::FindPath(Node* const pStartNode, Node* const pDestinationNode) const
{
    std::vector<Node*> path;
    std::queue<Node*>  openList{};
    std::vector<Node*> closedList{};
    std::map<Node*, Node*> parentMap{};

    openList.push(pStartNode);
    parentMap[pStartNode] = nullptr;

    Node* currentNode = nullptr;

    while (!openList.empty())
    {
        currentNode = openList.front();
        openList.pop();

        if (currentNode == pDestinationNode)
            break;

        std::vector<Connection*> connections = pGraph->FindConnectionsFrom(currentNode->GetId());

        for (Connection* pConnection : connections)
        {
            Node* pNeighbor = pGraph->GetNode(pConnection->GetToId()).get();

            if (std::find(closedList.begin(), closedList.end(), pNeighbor) != closedList.end())
                continue;

            if (parentMap.count(pNeighbor) > 0)
                continue;

            openList.push(pNeighbor);
            parentMap[pNeighbor] = currentNode;
        }

        closedList.push_back(currentNode);
    }

    if (currentNode == pDestinationNode)
    {
        Node* pNode = pDestinationNode;
        while (pNode != nullptr)
        {
            path.push_back(pNode);
            pNode = parentMap[pNode];
        }
        std::reverse(path.begin(), path.end());
    }

    return path;
}
