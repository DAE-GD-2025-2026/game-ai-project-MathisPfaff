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
		// ── FindPortals ────────────────────────────────────────────────────────
		// Converts the A* node path into an ordered list of NavLine portals.
		// P1 = right point of portal, P2 = left point (as seen from travel direction).
		// First and last portals are degenerate (start/end pos) to drive the algorithm.
		static std::vector<NavLine> FindPortals(std::vector<Node*> const& Path, TriPolygon const& NavPoly)
		{
			std::vector<NavLine> Portals{};
			if (Path.size() < 2) return Portals;

			// First degenerate portal = start apex position
			FVector2D startPos = Path.front()->GetPosition();
			Portals.push_back(NavLine{ startPos, startPos });

			// For each interior portal node, get its corresponding edge and orient it
			for (int i = 1; i < static_cast<int>(Path.size()) - 1; ++i)
			{
				const NavGraphNode* navNode = static_cast<const NavGraphNode*>(Path[i]);
				int edgeIdx = navNode->GetEdgeIdx();
				if (edgeIdx < 0) continue; // skip start/end nodes (EdgeIdx = -1)

				const TriPolygon::Edge& edge = NavPoly.GetEdges()[edgeIdx];
				FVector2D V1 = FVector2D{ edge.GetP1(NavPoly) };
				FVector2D V2 = FVector2D{ edge.GetP2(NavPoly) };

				// Determine orientation: P1 should be the RIGHT point, P2 the LEFT point.
				// Use cross(travelDir, edgeDir):
				//   > 0 (CCW) → V1 is right side
				//   <= 0     → V2 is right side
				FVector2D travelDir = Path[i + 1]->GetPosition() - Path[i - 1]->GetPosition();
				FVector2D edgeDir   = V2 - V1;
				float crossVal = travelDir.X * edgeDir.Y - travelDir.Y * edgeDir.X;

				if (crossVal > 0.f)
					Portals.push_back(NavLine{ V1, V2 }); // V1 = right
				else
					Portals.push_back(NavLine{ V2, V1 }); // V2 = right
			}

			// Last degenerate portal to force end evaluation
			FVector2D endPos = Path.back()->GetPosition();
			Portals.push_back(NavLine{ endPos, endPos });

			return Portals;
		}

		// ── OptimizePortals (SSFA) ─────────────────────────────────────────────
		// Takes the oriented portal list and produces a smoothed waypoint path.
		static std::vector<FVector2D> OptimizePortals(std::vector<NavLine> const& Portals, TriPolygon const& NavPoly)
		{
			std::vector<FVector2D> Path{};
			if (Portals.empty()) return Path;

			// 2D cross product helper
			// cross2D(A,B) > 0 → B is CCW (left) of A
			// cross2D(A,B) < 0 → B is CW  (right) of A
			auto Cross2D = [](FVector2D A, FVector2D B) -> float
			{
				return A.X * B.Y - A.Y * B.X;
			};

			// Initialise funnel from first (degenerate) portal
			FVector2D apexPos  = Portals[0].P1;          // start position
			FVector2D rightLeg = Portals[0].P1 - apexPos; // (0,0)
			FVector2D leftLeg  = Portals[0].P2 - apexPos; // (0,0)
			int apexIndex      = 0;
			int rightLegIndex  = 0;
			int leftLegIndex   = 0;

			// Add apex point (first path point)
			Path.push_back(apexPos);

			// Loop over all portals starting from the SECOND portal
			for (int portalIdx = 1; portalIdx < static_cast<int>(Portals.size()); ++portalIdx)
			{
				const NavLine& portal = Portals[portalIdx];

				// ── RIGHT CHECK ───────────────────────────────────────────────
				{
				    FVector2D newRightLeg = portal.P1 - apexPos;
				
				    // Going inwards = CCW.  Use >= 0 so the zero-vector initial legs
				    // are treated as "inwards" and the first portal correctly seeds the funnel.
				    if (Cross2D(rightLeg, newRightLeg) >= 0.f)          // ← was > 0.f
				    {
				        // Crosses leftLeg when strictly CCW past it (> 0, not >= 0)
				        if (Cross2D(leftLeg, newRightLeg) > 0.f)         // ← was >= 0.f
				        {
				            // ── New apex = leftLeg tip ─────────────────────
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
				
				// ── LEFT CHECK ────────────────────────────────────────────────
				{
				    FVector2D newLeftLeg = portal.P2 - apexPos;
				
				    // Going inwards = CW.  Use <= 0 for same zero-vector reason.
				    if (Cross2D(leftLeg, newLeftLeg) <= 0.f)             // ← was < 0.f
				    {
				        // Crosses rightLeg when strictly CW past it (< 0, not <= 0)
				        if (Cross2D(rightLeg, newLeftLeg) < 0.f)         // ← was <= 0.f
				        {
				            // ── New apex = rightLeg tip ────────────────────
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

			// Push the last point (degenerate portal P1 == P2 == endPos)
			Path.push_back(Portals.back().P1);

			return Path;
		}

	private:
		SSFA() {};
		~SSFA() {};
	};
}
