#pragma once
#include <vector>

#include "NavGraphPathfinding.h"
#include "Movement/Pathfinding/Navmesh/TriPolygon.h"
#include "Shared/Graph/Graph.h"
#include "Shared/Graph/NavGraph/NavGraphNode.h"

namespace GameAI
{
	class SSFA final
	{
	public:
		static std::vector<NavLine> FindPortals(std::vector<Node*> const& Path, TriPolygon const& NavPoly)
		{
			std::vector<NavLine> Portals{};
			if (Path.size() < 2) return Portals;

			FVector2D startPos = Path.front()->GetPosition();
			Portals.push_back(NavLine{ startPos, startPos });

			for (int i = 1; i < static_cast<int>(Path.size()) - 1; ++i)
			{
				const NavGraphNode* navNode = static_cast<const NavGraphNode*>(Path[i]);
				int edgeIdx = navNode->GetEdgeIdx();
				if (edgeIdx < 0) continue;

				const TriPolygon::Edge& edge = NavPoly.GetEdges()[edgeIdx];
				FVector2D V1 = FVector2D{ edge.GetP1(NavPoly) };
				FVector2D V2 = FVector2D{ edge.GetP2(NavPoly) };

				FVector2D travelDir = Path[i + 1]->GetPosition() - Path[i - 1]->GetPosition();
				FVector2D edgeDir   = V2 - V1;
				float crossVal = travelDir.X * edgeDir.Y - travelDir.Y * edgeDir.X;

				if (crossVal > 0.f)
					Portals.push_back(NavLine{ V1, V2 });
				else
					Portals.push_back(NavLine{ V2, V1 });
			}

			FVector2D endPos = Path.back()->GetPosition();
			Portals.push_back(NavLine{ endPos, endPos });

			return Portals;
		}
		
		static std::vector<FVector2D> OptimizePortals(std::vector<NavLine> const& Portals, TriPolygon const& NavPoly)
		{
			std::vector<FVector2D> Path{};
			if (Portals.empty()) return Path;
			auto Cross2D = [](FVector2D A, FVector2D B) -> float
			{
				return A.X * B.Y - A.Y * B.X;
			};

			FVector2D apexPos  = Portals[0].P1;
			FVector2D rightLeg = Portals[0].P1 - apexPos;
			FVector2D leftLeg  = Portals[0].P2 - apexPos;
			int apexIndex      = 0;
			int rightLegIndex  = 0;
			int leftLegIndex   = 0;

			Path.push_back(apexPos);

			for (int portalIdx = 1; portalIdx < static_cast<int>(Portals.size()); ++portalIdx)
			{
				const NavLine& portal = Portals[portalIdx];

				{
				    FVector2D newRightLeg = portal.P1 - apexPos;
				
				    if (Cross2D(rightLeg, newRightLeg) >= 0.f)
				    {
				        if (Cross2D(leftLeg, newRightLeg) > 0.f)
				        {
				            apexPos      += leftLeg;
				            apexIndex     = leftLegIndex;
				            portalIdx     = leftLegIndex + 1;
				            leftLegIndex  = rightLegIndex = portalIdx;
				
				            Path.push_back(apexPos);
				
				            if (portalIdx < static_cast<int>(Portals.size()))
				            {
				                rightLeg = Portals[portalIdx].P1 - apexPos;
				                leftLeg  = Portals[portalIdx].P2 - apexPos;
				            }
				            continue;
				        }
				        else
				        {
				            rightLeg      = newRightLeg;
				            rightLegIndex = portalIdx;
				        }
				    }
				}
				
				{
				    FVector2D newLeftLeg = portal.P2 - apexPos;
				
				    if (Cross2D(leftLeg, newLeftLeg) <= 0.f)
				    {
				        if (Cross2D(rightLeg, newLeftLeg) < 0.f)
				        {
				            apexPos      += rightLeg;
				            apexIndex     = rightLegIndex;
				            portalIdx     = rightLegIndex + 1;
				            leftLegIndex  = rightLegIndex = portalIdx;
				
				            Path.push_back(apexPos);
				
				            if (portalIdx < static_cast<int>(Portals.size()))
				            {
				                rightLeg = Portals[portalIdx].P1 - apexPos;
				                leftLeg  = Portals[portalIdx].P2 - apexPos;
				            }
				            continue;
				        }
				        else
				        {
				            leftLeg      = newLeftLeg;
				            leftLegIndex = portalIdx;
				        }
				    }
				}
			}
			Path.push_back(Portals.back().P1);

			return Path;
		}

	private:
		SSFA() {};
		~SSFA() {};
	};
}
