#pragma once
#include <stack>
#include "Shared/Graph/Graph.h"

namespace GameAI
{
	enum class Eulerianity
	{
		notEulerian,
		semiEulerian,
		eulerian,
	};

	class EulerianPath final
	{
	public:
		EulerianPath(Graph* const pGraph);

		Eulerianity IsEulerian() const;
		std::vector<Node*> FindPath(Eulerianity& eulerianity) const;

	private:
		void VisitAllNodesDFS(const std::vector<Node*>& pNodes, std::vector<bool>& visited, int startIndex) const;
		bool IsConnected() const;

		Graph* m_pGraph;
	};

	inline EulerianPath::EulerianPath(Graph* const pGraph)
		: m_pGraph(pGraph)
	{
	}

	inline Eulerianity EulerianPath::IsEulerian() const
	{
		if (!IsConnected())
			return Eulerianity::notEulerian;

		std::vector<Node*> Nodes = m_pGraph->GetActiveNodes();
		int oddDegreeCount = 0;
		for (Node* node : Nodes)
		{
			int degree = static_cast<int>(m_pGraph->FindConnectionsFrom(node->GetId()).size());
			if (degree % 2 != 0)
				++oddDegreeCount;
		}

		if (oddDegreeCount > 2)
			return Eulerianity::notEulerian;

		if (oddDegreeCount == 2)
			return Eulerianity::semiEulerian;

		return Eulerianity::eulerian;
	}

	inline std::vector<Node*> EulerianPath::FindPath(Eulerianity& eulerianity) const
	{
		Graph graphCopy = m_pGraph->Clone();
    	std::vector<Node*> Path = {};
    	std::vector<Node*> Nodes = graphCopy.GetActiveNodes();
    	int currentNodeId{ Graphs::InvalidNodeId };
		
    	eulerianity = IsEulerian();
		
    	if (eulerianity == Eulerianity::notEulerian)
    	    return Path;
		
    	if (eulerianity == Eulerianity::eulerian)
    	{
    	    if (!Nodes.empty())
    	        currentNodeId = Nodes[0]->GetId();
    	}
    	else
    	{
    	    for (Node* node : Nodes)
    	    {
    	        int degree = static_cast<int>(graphCopy.FindConnectionsFrom(node->GetId()).size());
    	        if (degree % 2 != 0)
    	        {
    	            currentNodeId = node->GetId();
    	            break;
    	        }
    	    }
    	}
		
    	if (currentNodeId == Graphs::InvalidNodeId)
    	    return Path;
		
    	std::stack<int> nodeStack;
		
    	while (!graphCopy.FindConnectionsFrom(currentNodeId).empty() || !nodeStack.empty())
    	{
    	    auto connections = graphCopy.FindConnectionsFrom(currentNodeId);
		
    	    if (!connections.empty())
    	    {
    	        nodeStack.push(currentNodeId);
		
    	        int neighborId = connections[0]->GetToId();
		
    	        graphCopy.RemoveConnection(currentNodeId, neighborId);
		
    	        currentNodeId = neighborId;
    	    }
    	    else
    	    {
    	        Path.push_back(m_pGraph->GetNode(currentNodeId).get());
		
    	        currentNodeId = nodeStack.top();
    	        nodeStack.pop();
    	    }
    	}
		
    	Path.push_back(m_pGraph->GetNode(currentNodeId).get());
		
    	std::reverse(Path.begin(), Path.end());
    	return Path;
	}

	inline void EulerianPath::VisitAllNodesDFS(const std::vector<Node*>& Nodes, std::vector<bool>& visited, int startIndex ) const
	{
		visited[startIndex] = true;

		std::vector<Connection*> connections = m_pGraph->FindConnectionsFrom(Nodes[startIndex]->GetId());

		for (Connection* connection : connections)
		{
			int toId = connection->GetToId();
			for (int i = 0; i < static_cast<int>(Nodes.size()); ++i)
			{
				if (Nodes[i]->GetId() == toId && !visited[i])
				{
					VisitAllNodesDFS(Nodes, visited, i);
				}
			}
		}
	}

	inline bool EulerianPath::IsConnected() const
	{
		std::vector<Node*> Nodes = m_pGraph->GetActiveNodes();
		if (Nodes.size() == 0)
			return false;
		
		int startIndex = 0;
		for (int i = 0; i < static_cast<int>(Nodes.size()); ++i)
		{
			if (!m_pGraph->FindConnectionsFrom(Nodes[i]->GetId()).empty())
			{
				startIndex = i;
				break;
			}
		}

		std::vector<bool> visited(Nodes.size(), false);
		VisitAllNodesDFS(Nodes, visited, startIndex);

		for (bool wasVisited : visited)
		{
			if (!wasVisited)
				return false;
		}
		return true;
	}
}