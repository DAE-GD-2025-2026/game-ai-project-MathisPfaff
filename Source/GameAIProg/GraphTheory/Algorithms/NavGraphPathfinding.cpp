#include "NavGraphPathfinding.h"

#include "AStar.h"
#include "Heuristics.h"
#include "PathSmoothing.h"
#include "VectorTypes.h"
#include "Shared/Graph/NavGraph/NavGraph.h"
#include "Shared/Graph/NavGraph/NavGraphNode.h"

using namespace GameAI;

std::vector<FVector2D> NavMeshPathfinding::FindPath(const FVector2D& startPos, const FVector2D& endPos,
	NavGraph* const pNavGraph, std::vector<FVector2D>& debugNodePositions, std::vector<NavLine>& debugPortals) 
{
	std::vector<FVector2D> finalPath{};

	const TriPolygon* pNavPoly = pNavGraph->GetNavPolygon();

	const TriPolygon::Triangle* startTriangle = pNavPoly->GetTriangleAtPosition(startPos, true);
	const TriPolygon::Triangle* endTriangle   = pNavPoly->GetTriangleAtPosition(endPos,   true);

	if (!startTriangle || !endTriangle)
		return finalPath;

	if (*startTriangle == *endTriangle)
	{
		finalPath.push_back(startPos);
		finalPath.push_back(endPos);
		return finalPath;
	}

	std::unique_ptr<NavGraph> clonedGraph = pNavGraph->Clone();

	int startNodeId = clonedGraph->AddNode(std::make_unique<NavGraphNode>(startPos, -1));

	for (const TriPolygon::Edge& edge : startTriangle->GetEdges())
	{
		int edgeIdx    = pNavPoly->FindEdgeIndex(edge).value_or(-1);
		int portalNode = clonedGraph->GetNodeIdFromEdgeIndex(edgeIdx);
		if (portalNode != Graphs::InvalidNodeId)
		{
			FVector2D portalPos = clonedGraph->GetNode(portalNode)->GetPosition();
			float dist = FVector2D::Distance(startPos, portalPos);
			auto conn = std::make_unique<Connection>(startNodeId, portalNode);
			conn->SetWeight(dist);
			clonedGraph->AddConnection(std::move(conn));
		}
	}

	int endNodeId = clonedGraph->AddNode(std::make_unique<NavGraphNode>(endPos, -1));

	for (const TriPolygon::Edge& edge : endTriangle->GetEdges())
	{
		int edgeIdx    = pNavPoly->FindEdgeIndex(edge).value_or(-1);
		int portalNode = clonedGraph->GetNodeIdFromEdgeIndex(edgeIdx);
		if (portalNode != Graphs::InvalidNodeId)
		{
			FVector2D portalPos = clonedGraph->GetNode(portalNode)->GetPosition();
			float dist = FVector2D::Distance(endPos, portalPos);
			auto conn = std::make_unique<Connection>(endNodeId, portalNode);
			conn->SetWeight(dist);
			clonedGraph->AddConnection(std::move(conn));
		}
	}

	AStar aStar{ clonedGraph.get(), HeuristicFunctions::Euclidean };

	Node* pStartNode = clonedGraph->GetNode(startNodeId).get();
	Node* pEndNode   = clonedGraph->GetNode(endNodeId).get();

	std::vector<Node*> nodePath = aStar.FindPath(pStartNode, pEndNode);

	if (nodePath.empty())
		return finalPath;

	for (Node* pNode : nodePath)
		debugNodePositions.push_back(pNode->GetPosition());

	debugPortals = SSFA::FindPortals(nodePath, *pNavGraph->GetNavPolygon());

	finalPath = SSFA::OptimizePortals(debugPortals, *pNavGraph->GetNavPolygon());

	return finalPath;
}

std::vector<FVector2D> NavMeshPathfinding::FindPath(const FVector2D& startPos, const FVector2D& endPos, NavGraph* const pNavGraph)
{
	std::vector<FVector2D> debugNodePositions{};
	std::vector<NavLine> debugPortals{};
	return FindPath(startPos, endPos, pNavGraph, debugNodePositions, debugPortals);
}
