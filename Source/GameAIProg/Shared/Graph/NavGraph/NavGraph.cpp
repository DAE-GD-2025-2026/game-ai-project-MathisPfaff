#include "NavGraph.h"

#include "NavGraphNode.h"

GameAI::NavGraph::NavGraph(std::unique_ptr<TriPolygon> && NavPoly)
	: Graph{false}
	, pNavPoly{std::move(NavPoly)}
{
	CreateNavigationGraph();
}

GameAI::NavGraph::NavGraph(const NavGraph& Other)
	: Graph(false)
{
	Nodes.reserve(Other.Nodes.size());
	for (std::unique_ptr<Node> const & OtherNode : Other.Nodes)
	{
		Nodes.push_back(std::make_unique<NavGraphNode>(*dynamic_cast<NavGraphNode*>(OtherNode.get())));
	}
        
	Connections.reserve(Other.Connections.size());
	for (std::unique_ptr<Connection> const & OtherConnection : Other.Connections)
	{
		Connections.push_back(std::make_unique<Connection>(*OtherConnection.get()));
	}
}

std::unique_ptr<GameAI::NavGraph> GameAI::NavGraph::Clone() const
{
	return std::make_unique<NavGraph>(*this);
}

int GameAI::NavGraph::GetNodeIdFromEdgeIndex(int EdgeIdx) const
{
	if (EdgeIdx >= 0)
	{
		for (auto const & pNode : Nodes)
		{
			if (reinterpret_cast<NavGraphNode*>(pNode.get())->GetEdgeIdx() == EdgeIdx)
			{
				return pNode->GetId();
			}
		}
	}
	
	return Graphs::InvalidNodeId;
}

void GameAI::NavGraph::CreateNavigationGraph()
{
    const std::vector<TriPolygon::Edge>&     edges     = pNavPoly->GetEdges();
    const std::vector<TriPolygon::Triangle>& triangles = pNavPoly->GetTriangles();
	
    for (int edgeIdx = 0; edgeIdx < static_cast<int>(edges.size()); ++edgeIdx)
    {
        const TriPolygon::Edge& edge = edges[edgeIdx];

        int triCount = 0;
        for (const TriPolygon::Triangle& tri : triangles)
        {
            if (tri.HasEdge(edge))
                ++triCount;
        }

        if (triCount >= 2)
        {
            FVector p1 = edge.GetP1(*pNavPoly);
            FVector p2 = edge.GetP2(*pNavPoly);
            FVector2D midpoint{ (p1.X + p2.X) * 0.5f, (p1.Y + p2.Y) * 0.5f };

            AddNode(std::make_unique<NavGraphNode>(midpoint, edgeIdx));
        }
    }

    for (const TriPolygon::Triangle& tri : triangles)
    {
        std::array<TriPolygon::Edge, 3> triEdges = tri.GetEdges();

        std::vector<int> nodeIds;
        for (const TriPolygon::Edge& triEdge : triEdges)
        {
            int edgeIdx = pNavPoly->FindEdgeIndex(triEdge).value_or(-1);
            int nodeId  = GetNodeIdFromEdgeIndex(edgeIdx);
            if (nodeId != Graphs::InvalidNodeId)
                nodeIds.push_back(nodeId);
        }

        if (nodeIds.size() == 2)
        {
            AddConnection(nodeIds[0], nodeIds[1]);
        }
    	
        else if (nodeIds.size() == 3)
        {
            AddConnection(nodeIds[0], nodeIds[1]);
            AddConnection(nodeIds[1], nodeIds[2]);
            AddConnection(nodeIds[0], nodeIds[2]);
        }
    }

    SetConnectionCostsToDistances();
}

