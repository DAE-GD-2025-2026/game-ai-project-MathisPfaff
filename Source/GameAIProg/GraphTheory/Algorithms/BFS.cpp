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

// TODO Breath First Search Algorithm searches for a path from the startNode to the destinationNode
std::vector<Node*> BFS::FindPath(Node* const pStartNode, Node* const pDestinationNode) const
{
    std::vector<Node*> path;
    std::queue<Node*>  openList{};
    std::vector<Node*> closedList{};
    std::map<Node*, Node*> parentMap{};

    // Step 1: Add startNode to openList
    openList.push(pStartNode);
    parentMap[pStartNode] = nullptr; // start has no parent

    Node* currentNode = nullptr;

    // Step 2: While openList is not empty
    while (!openList.empty())
    {
        // 2A: Get next node (FIFO)
        currentNode = openList.front();
        openList.pop();

        // 2B: Check if destination reached
        if (currentNode == pDestinationNode)
            break;

        // 2C: Get all connections
        std::vector<Connection*> connections = pGraph->FindConnectionsFrom(currentNode->GetId());

        for (Connection* pConnection : connections)
        {
            Node* pNeighbor = pGraph->GetNode(pConnection->GetToId()).get();

            // D. Already visited?
            if (std::find(closedList.begin(), closedList.end(), pNeighbor) != closedList.end())
                continue;

            // E. Already queued?
            if (parentMap.count(pNeighbor) > 0)
                continue;

            // F. Add to openList and record parent
            openList.push(pNeighbor);
            parentMap[pNeighbor] = currentNode;
        }

        // G. Mark current as visited
        closedList.push_back(currentNode);
    }

    // Step 3: Reconstruct path from destination back to start using parentMap
    if (currentNode == pDestinationNode)
    {
        Node* pNode = pDestinationNode;
        while (pNode != nullptr)
        {
            path.push_back(pNode);
            pNode = parentMap[pNode]; // walk to parent (nullptr stops at start)
        }
        std::reverse(path.begin(), path.end()); // built backwards → flip to start→goal
    }

    return path;
}
