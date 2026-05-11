#include "PatrolState.h"
#include "DecisionMaking/GameAIController.h"
#include "Movement/SteeringBehaviors/SteeringAgent.h"
#include "Movement/SteeringBehaviors/Steering/SteeringBehaviors.h"

namespace GameAI::FSM
{
	PatrolState::PatrolState(ASteeringAgent* InAgent, AGameAIController* InController,
							 TArray<FVector2D> InPatrolPoints)
		: Agent(InAgent), Controller(InController), PatrolPoints(InPatrolPoints)
	{
		// Create our Arrive behavior once — reused for the whole patrol
		ArriveBehavior = std::make_unique<Arrive>();
	}

	void PatrolState::OnEnter()
	{
		if (!Agent || PatrolPoints.IsEmpty()) return;

		// Start patrolling from wherever we left off (PatrolIndex persists between visits)
		Agent->SetSteeringBehavior(ArriveBehavior.get());
		ArriveBehavior->SetTarget(FTargetData{PatrolPoints[PatrolIndex]});
	}

	void PatrolState::OnExit()
	{
		// Stop movement by clearing the behavior
		Agent->SetSteeringBehavior(nullptr);
	}

	void PatrolState::Update(float DeltaTime)
	{
		if (!Agent || PatrolPoints.IsEmpty()) return;

		// Check if we're close enough to the current waypoint
		FVector2D ToTarget = PatrolPoints[PatrolIndex] - Agent->GetPosition();
		if (ToTarget.Length() < 100.f) // within arrival radius
		{
			// Advance to next waypoint, wrap around
			PatrolIndex = (PatrolIndex + 1) % PatrolPoints.Num();
			ArriveBehavior->SetTarget(FTargetData{PatrolPoints[PatrolIndex]});
		}
	}
}