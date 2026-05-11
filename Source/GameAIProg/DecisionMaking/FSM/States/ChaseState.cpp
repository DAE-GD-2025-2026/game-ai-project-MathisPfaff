#include "ChaseState.h"
#include "DecisionMaking/GameAIController.h"
#include "Movement/SteeringBehaviors/SteeringAgent.h"
#include "Movement/SteeringBehaviors/Steering/SteeringBehaviors.h"
#include "BehaviorTree/BlackboardComponent.h"
#include "GameFramework/Character.h"

namespace GameAI::FSM
{
	ChaseState::ChaseState(ASteeringAgent* InAgent, AGameAIController* InController)
		: Agent(InAgent), Controller(InController)
	{
		PursuitBehavior = std::make_unique<Pursuit>();
	}

	void ChaseState::OnEnter()
	{
		if (!Agent) return;
		Agent->SetSteeringBehavior(PursuitBehavior.get());
	}

	void ChaseState::OnExit()
	{
		Agent->SetSteeringBehavior(nullptr);
	}

	void ChaseState::Update(float DeltaTime)
	{
		if (!Agent || !Controller) return;

		UBlackboardComponent* BB = Controller->GetBlackboardComponent();
		if (!BB) return;

		// Get the target actor from the blackboard
		AActor* Target = Cast<AActor>(BB->GetValueAsObject("TargetActor"));
		if (!Target) return;

		// Update pursuit target with player's current position AND velocity
		FTargetData TargetData;
		TargetData.Position       = FVector2D(Target->GetActorLocation());
		TargetData.LinearVelocity = FVector2D(Target->GetVelocity());
		PursuitBehavior->SetTarget(TargetData);

		// Always keep LastKnownLocation fresh so Search knows where to go
		BB->SetValueAsVector("LastKnownLocation", Target->GetActorLocation());
	}
}