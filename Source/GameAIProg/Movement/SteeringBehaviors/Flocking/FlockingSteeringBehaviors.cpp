#include "FlockingSteeringBehaviors.h"
#include "Flock.h"

// --- Cohesion ---
SteeringOutput Cohesion::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	if (pFlock->GetNrOfNeighbors() == 0)
		return SteeringOutput{};

	FTargetData seekTarget{};
	seekTarget.Position = pFlock->GetAverageNeighborPos();
	SetTarget(seekTarget);

	return Seek::CalculateSteering(deltaT, pAgent);
}

// --- Separation ---
SteeringOutput Separation::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	SteeringOutput output{};

	const int nrNeighbors = pFlock->GetNrOfNeighbors();
	if (nrNeighbors == 0)
		return output;

	const FVector2D agentPos = FVector2D(pAgent.GetActorLocation());
	FVector2D separationVelocity = FVector2D::ZeroVector;

	for (int i = 0; i < nrNeighbors; ++i)
	{
		const ASteeringAgent* pNeighbor = pFlock->GetNeighbors()[i];
		if (!pNeighbor) continue;

		const FVector2D neighborPos = FVector2D(pNeighbor->GetActorLocation());
		FVector2D toAgent = agentPos - neighborPos;
		const float dist = toAgent.Size();

		if (dist > KINDA_SMALL_NUMBER)
			separationVelocity += toAgent.GetSafeNormal() / dist;
	}

	output.LinearVelocity = separationVelocity.GetSafeNormal() * pAgent.GetMaxLinearSpeed();
	return output;
}

// --- VelocityMatch ---
SteeringOutput VelocityMatch::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	SteeringOutput output{};

	if (pFlock->GetNrOfNeighbors() == 0)
		return output;

	// Use the actual average velocity, not a normalized + rescaled version
	output.LinearVelocity = pFlock->GetAverageNeighborVelocity();
	return output;
}

