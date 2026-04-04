#include "AStar.h"
#include <algorithm>

using namespace GameAI;

AStar::AStar(Graph* const pGraph, HeuristicFunctions::Heuristic hFunction)
    : pGraph(pGraph)
    , HeuristicFunction(hFunction)
{
}

std::vector<Node*> AStar::FindPath(Node* const pStartNode, Node* const pGoalNode)
{
    std::vector<Node*> path{};
    std::vector<NodeRecord> openList{};
    std::vector<NodeRecord> closedList{};

    // Step 1: Create startRecord and add to openList
    NodeRecord startRecord{ pStartNode, nullptr, 0.f, 
        GetHeuristicCost(pStartNode, pGoalNode) };
    
    openList.push_back(startRecord);

    NodeRecord currentRecord{};

    // Step 2: While openList is not empty
    while (!openList.empty())
    {
        // 2A: Get record with lowest F-score
        auto lowestIt = std::min_element(openList.begin(), openList.end());
        currentRecord = *lowestIt;

        // 2B: Check if goal reached
        if (currentRecord.pNode == pGoalNode)
            break;

        // 2C: Get all connections of current node
        std::vector<Connection*> connections = pGraph->FindConnectionsFrom(currentRecord.pNode->GetId());

        for (Connection* pConnection : connections)
        {
            Node* pNeighbor = pGraph->GetNode(pConnection->GetToId()).get();
            float gCost = currentRecord.costSoFar + pConnection->GetWeight();

            // D. Check closedList
            auto closedIt = std::find_if(closedList.begin(), closedList.end(),
                [pNeighbor](const NodeRecord& r) { return r.pNode == pNeighbor; });
            if (closedIt != closedList.end())
            {
                if (closedIt->costSoFar <= gCost)
                    continue;
                closedList.erase(closedIt);
            }
            else
            {
                // E. Check openList
                auto openIt = std::find_if(openList.begin(), openList.end(),
                    [pNeighbor](const NodeRecord& r) { return r.pNode == pNeighbor; });
                if (openIt != openList.end())
                {
                    if (openIt->costSoFar <= gCost)
                        continue;
                    // F. Remove expensive
                    openList.erase(openIt);
                }
            }

            // F. Create new NodeRecord and add to openList
            NodeRecord newRecord{};
            newRecord.pNode = pNeighbor;
            newRecord.pConnection = pConnection;
            newRecord.costSoFar = gCost;
            newRecord.estimatedTotalCost = gCost + GetHeuristicCost(pNeighbor, pGoalNode);
            openList.push_back(newRecord);
        }

        // G. Remove current from openList, add to closedList
        openList.erase(std::find(openList.begin(), openList.end(), currentRecord));
        closedList.push_back(currentRecord);
    }

    // Step 3: Reconstruct path from last connection back to start node
    if (currentRecord.pNode == pGoalNode)
    {
        while (currentRecord.pNode != pStartNode)
        {
            path.push_back(currentRecord.pNode);

            // Follow the connection backwards to the parent node
            Node* pFromNode = pGraph->GetNode(currentRecord.pConnection->GetFromId()).get();

            // Find the parent's NodeRecord in the closedList
            auto it = std::find_if(closedList.begin(), closedList.end(),
                [pFromNode](const NodeRecord& r) { return r.pNode == pFromNode; });
            currentRecord = *it;
        }
        path.push_back(pStartNode);
        std::reverse(path.begin(), path.end()); // built backwards → flip to start→goal
    }

    return path;
}

float AStar::GetHeuristicCost(Node* const pStartNode, Node* const pEndNode) const
{
    FVector2D toDestination = pGraph->GetNode(pEndNode->GetId())->GetPosition()
                            - pGraph->GetNode(pStartNode->GetId())->GetPosition();
    return HeuristicFunction(abs(toDestination.X), abs(toDestination.Y));
}
