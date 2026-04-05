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

    NodeRecord startRecord{ pStartNode, nullptr, 0.f, 
        GetHeuristicCost(pStartNode, pGoalNode) };
    
    openList.push_back(startRecord);

    NodeRecord currentRecord{};

    while (!openList.empty())
    {
        auto lowestIt = std::min_element(openList.begin(), openList.end());
        currentRecord = *lowestIt;

        if (currentRecord.pNode == pGoalNode)
            break;

        std::vector<Connection*> connections = pGraph->FindConnectionsFrom(currentRecord.pNode->GetId());

        for (Connection* pConnection : connections)
        {
            Node* pNeighbor = pGraph->GetNode(pConnection->GetToId()).get();
            float gCost = currentRecord.costSoFar + pConnection->GetWeight();

            // Closed nodes are re-opened only if there is a cheaper route to them
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
                // Already queued with a cheaper or equal cost, so no re-add
                auto openIt = std::find_if(openList.begin(), openList.end(),
                    [pNeighbor](const NodeRecord& r) { return r.pNode == pNeighbor; });
                if (openIt != openList.end())
                {
                    if (openIt->costSoFar <= gCost)
                        continue;
                    openList.erase(openIt);
                }
            }

            NodeRecord newRecord{};
            newRecord.pNode = pNeighbor;
            newRecord.pConnection = pConnection;
            newRecord.costSoFar = gCost;
            newRecord.estimatedTotalCost = gCost + GetHeuristicCost(pNeighbor, pGoalNode);
            openList.push_back(newRecord);
        }

        openList.erase(std::find(openList.begin(), openList.end(), currentRecord));
        closedList.push_back(currentRecord);
    }

    if (currentRecord.pNode == pGoalNode)
    {
        // Reconstruct path through going through connections backwards from goal to start
        while (currentRecord.pNode != pStartNode)
        {
            path.push_back(currentRecord.pNode);

            Node* pFromNode = pGraph->GetNode(currentRecord.pConnection->GetFromId()).get();

            auto it = std::find_if(closedList.begin(), closedList.end(),
                [pFromNode](const NodeRecord& r) { return r.pNode == pFromNode; });
            currentRecord = *it;
        }
        path.push_back(pStartNode);
        std::reverse(path.begin(), path.end());
    }

    return path;
}

float AStar::GetHeuristicCost(Node* const pStartNode, Node* const pEndNode) const
{
    FVector2D toDestination = pGraph->GetNode(pEndNode->GetId())->GetPosition()
                            - pGraph->GetNode(pStartNode->GetId())->GetPosition();
    return HeuristicFunction(abs(toDestination.X), abs(toDestination.Y));
}
