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
		// If the graph is not connected, there can be no Eulerian Trail
		if (!IsConnected())
			return Eulerianity::notEulerian;

		// Count nodes with odd degree
		std::vector<Node*> Nodes = m_pGraph->GetActiveNodes();
		int oddDegreeCount = 0;
		for (Node* node : Nodes)
		{
			int degree = static_cast<int>(m_pGraph->FindConnectionsFrom(node->GetId()).size());
			if (degree % 2 != 0)
				++oddDegreeCount;
		}

		// More than 2 odd-degree nodes -> not Eulerian
		if (oddDegreeCount > 2)
			return Eulerianity::notEulerian;

		// Exactly 2 odd-degree nodes -> Semi-Eulerian (has Euler trail, not circuit)
		if (oddDegreeCount == 2)
			return Eulerianity::semiEulerian;

		// No odd-degree nodes -> fully Eulerian (has Euler circuit)
		return Eulerianity::eulerian;
	}

	inline std::vector<Node*> EulerianPath::FindPath(Eulerianity& eulerianity) const
	{
		// Get a copy of the graph because this algorithm involves removing edges
		Graph graphCopy = m_pGraph->Clone();
    	std::vector<Node*> Path = {};
    	std::vector<Node*> Nodes = graphCopy.GetActiveNodes();
    	int currentNodeId{ Graphs::InvalidNodeId };
		
    	// Check if there can be an Euler path
    	eulerianity = IsEulerian();
		
    	// If this graph is not eulerian, return the empty path
    	if (eulerianity == Eulerianity::notEulerian)
    	    return Path;
		
    	// Choose a starting node
    	if (eulerianity == Eulerianity::eulerian)
    	{
    	    // All even degrees -> any node works
    	    if (!Nodes.empty())
    	        currentNodeId = Nodes[0]->GetId();
    	}
    	else // semiEulerian -> must start at a node with odd degree
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
		
    	// Start algorithm loop (Hierholzer's algorithm)
    	std::stack<int> nodeStack;
		
    	// Step 3: repeat until current node has no connections AND stack is empty
    	while (!graphCopy.FindConnectionsFrom(currentNodeId).empty() || !nodeStack.empty())
    	{
    	    auto connections = graphCopy.FindConnectionsFrom(currentNodeId);
		
    	    if (!connections.empty()) // Step 4: current node HAS neighbors
    	    {
    	        // I. Add current node to the stack
    	        nodeStack.push(currentNodeId);
		
    	        // II. Take any neighbor
    	        int neighborId = connections[0]->GetToId();
		
    	        // IV. Remove the edge between current node and that neighbor (from the COPY!)
    	        graphCopy.RemoveConnection(currentNodeId, neighborId);
		
    	        // III. Set neighbor as the current node
    	        currentNodeId = neighborId;
    	    }
    	    else // current node has NO neighbors
    	    {
    	        // Add current node to path - get from ORIGINAL graph!
    	        Path.push_back(m_pGraph->GetNode(currentNodeId).get());
		
    	        // Move back to top of stack
    	        currentNodeId = nodeStack.top();
    	        nodeStack.pop();
    	    }
    	}
		
    	// Step 5: Add the last current node to the path (from original graph!)
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