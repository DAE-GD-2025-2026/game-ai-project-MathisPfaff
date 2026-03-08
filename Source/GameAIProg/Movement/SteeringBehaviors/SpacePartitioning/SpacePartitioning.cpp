#include "SpacePartitioning.h"

// --- Cell ---
// ------------
Cell::Cell(float Left, float Bottom, float Width, float Height)
{
	BoundingBox.Min = { Left, Bottom };
	BoundingBox.Max = { BoundingBox.Min.X + Width, BoundingBox.Min.Y + Height };
}

std::vector<FVector2D> Cell::GetRectPoints() const
{
	const float left = BoundingBox.Min.X;
	const float bottom = BoundingBox.Min.Y;
	const float width = BoundingBox.Max.X - BoundingBox.Min.X;
	const float height = BoundingBox.Max.Y - BoundingBox.Min.Y;

	std::vector<FVector2D> rectPoints =
	{
		{ left , bottom  },
		{ left , bottom + height  },
		{ left + width , bottom + height },
		{ left + width , bottom  },
	};

	return rectPoints;
}

// --- Partitioned Space ---
// -------------------------
CellSpace::CellSpace(UWorld* pWorld, float Width, float Height, int Rows, int Cols, int MaxEntities)
	: pWorld{pWorld}
, SpaceWidth{Width}
, SpaceHeight{Height}
, NrOfRows{Rows}
, NrOfCols{Cols}
, NrOfNeighbors{0}
{
	Neighbors.SetNum(MaxEntities);

	CellWidth  = Width  / Cols;
	CellHeight = Height / Rows;

	// Store the origin (bottom-left corner of the grid)
	CellOrigin = FVector2D(-Width * 0.5f, -Height * 0.5f);

	// Create all cells in row-major order
	Cells.reserve(Rows * Cols);
	for (int row = 0; row < Rows; ++row)
	{
		for (int col = 0; col < Cols; ++col)
		{
			const float left   = CellOrigin.X + col * CellWidth;
			const float bottom = CellOrigin.Y + row * CellHeight;
			Cells.emplace_back(left, bottom, CellWidth, CellHeight);
		}
	}
}


void CellSpace::AddAgent(ASteeringAgent& Agent)
{
	const int index = PositionToIndex(FVector2D(Agent.GetActorLocation()));
	if (index >= 0 && index < static_cast<int>(Cells.size()))
		Cells[index].Agents.push_back(&Agent);
}


void CellSpace::UpdateAgentCell(ASteeringAgent& Agent, const FVector2D& OldPos)
{
	const int oldIndex = PositionToIndex(OldPos);
	const int newIndex = PositionToIndex(FVector2D(Agent.GetActorLocation()));

	if (oldIndex != newIndex)
	{
		// Remove from old cell
		if (oldIndex >= 0 && oldIndex < static_cast<int>(Cells.size()))
			Cells[oldIndex].Agents.remove(&Agent);

		// Add to new cell
		if (newIndex >= 0 && newIndex < static_cast<int>(Cells.size()))
			Cells[newIndex].Agents.push_back(&Agent);
	}
}


void CellSpace::RegisterNeighbors(ASteeringAgent& Agent, float QueryRadius)
{
	NrOfNeighbors = 0;

	const FVector2D agentPos = FVector2D(Agent.GetActorLocation());

	// Build a query rect around the agent
	FRect QueryRect;
	QueryRect.Min = FVector2D(agentPos.X - QueryRadius, agentPos.Y - QueryRadius);
	QueryRect.Max = FVector2D(agentPos.X + QueryRadius, agentPos.Y + QueryRadius);

	for (Cell& cell : Cells)
	{
		if (!DoRectsOverlap(QueryRect, cell.BoundingBox))
			continue;

		for (ASteeringAgent* pOther : cell.Agents)
		{
			if (!pOther || pOther == &Agent) continue;

			const FVector2D otherPos = FVector2D(pOther->GetActorLocation());
			if (FVector2D::DistSquared(agentPos, otherPos) <= QueryRadius * QueryRadius)
			{
				Neighbors[NrOfNeighbors] = pOther;
				++NrOfNeighbors;
			}
		}
	}
}


void CellSpace::EmptyCells()
{
	for (Cell& c : Cells)
		c.Agents.clear();
}

void CellSpace::RenderCells() const
{
	if (!pWorld) return;

	for (const Cell& cell : Cells)
	{
		const FVector2D& min = cell.BoundingBox.Min;
		const FVector2D& max = cell.BoundingBox.Max;

		const FVector p0(min.X, min.Y, 5.f);
		const FVector p1(min.X, max.Y, 5.f);
		const FVector p2(max.X, max.Y, 5.f);
		const FVector p3(max.X, min.Y, 5.f);

		const FColor lineColor = cell.Agents.empty() ? FColor::Green : FColor::Yellow;
		DrawDebugLine(pWorld, p0, p1, lineColor, false, -1.f, 0, 7.f);
		DrawDebugLine(pWorld, p1, p2, lineColor, false, -1.f, 0, 7.f);
		DrawDebugLine(pWorld, p2, p3, lineColor, false, -1.f, 0, 7.f);
		DrawDebugLine(pWorld, p3, p0, lineColor, false, -1.f, 0, 7.f);

		const FVector center(
			(min.X + max.X) * 0.5f,
			(min.Y + max.Y) * 0.5f,
			15.f);

		DrawDebugString(pWorld, center,
			FString::FromInt(static_cast<int>(cell.Agents.size())),
			nullptr, FColor::White, 0.f, false, 1.f);
	}
}


int CellSpace::PositionToIndex(FVector2D const& Pos) const
{
	int col = static_cast<int>((Pos.X - CellOrigin.X) / CellWidth);
	int row = static_cast<int>((Pos.Y - CellOrigin.Y) / CellHeight);

	// Clamp to grid bounds
	col = FMath::Clamp(col, 0, NrOfCols - 1);
	row = FMath::Clamp(row, 0, NrOfRows - 1);

	return row * NrOfCols + col;
}


bool CellSpace::DoRectsOverlap(FRect const & RectA, FRect const & RectB)
{
	// Check if the rectangles are separated on either axis
	if (RectA.Max.X < RectB.Min.X || RectA.Min.X > RectB.Max.X) return false;
	if (RectA.Max.Y < RectB.Min.Y || RectA.Min.Y > RectB.Max.Y) return false;
    
	// If they are not separated, they must overlap
	return true;
}