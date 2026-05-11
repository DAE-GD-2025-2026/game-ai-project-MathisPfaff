#include "SearchState.h"
#include "DecisionMaking/GameAIController.h"
#include "Movement/SteeringBehaviors/SteeringAgent.h"
#include "Movement/SteeringBehaviors/Steering/SteeringBehaviors.h"
#include "BehaviorTree/BlackboardComponent.h"

namespace GameAI::FSM
{
    SearchState::SearchState(ASteeringAgent* InAgent, AGameAIController* InController)
        : Agent(InAgent), Controller(InController)
    {
        ArriveBehavior = std::make_unique<Arrive>();
        WanderBehavior = std::make_unique<Wander>();
    }

    void SearchState::OnEnter()
    {
        if (!Agent || !Controller) return;
        UBlackboardComponent* BB = Controller->GetBlackboardComponent();
        if (!BB) return;

        // Reset state
        BB->SetValueAsFloat("SearchTime", 0.f);
        bArrivedAtLastKnown = false;

        // Phase 1: move to where the player was last seen
        FVector LastKnown = BB->GetValueAsVector("LastKnownLocation");
        ArriveBehavior->SetTarget(FTargetData{FVector2D(LastKnown)});
        Agent->SetSteeringBehavior(ArriveBehavior.get());
    }

    void SearchState::OnExit()
    {
        Agent->SetSteeringBehavior(nullptr);
        if (Controller)
            if (UBlackboardComponent* BB = Controller->GetBlackboardComponent())
                BB->SetValueAsFloat("SearchTime", 0.f);
    }

    void SearchState::Update(float DeltaTime)
    {
        if (!Agent || !Controller) return;
        UBlackboardComponent* BB = Controller->GetBlackboardComponent();
        if (!BB) return;

        // Tick the search timer
        BB->SetValueAsFloat("SearchTime", BB->GetValueAsFloat("SearchTime") + DeltaTime);

        if (!bArrivedAtLastKnown)
        {
            // Check if we've reached the last known location
            FVector LastKnown = BB->GetValueAsVector("LastKnownLocation");
            float Dist = FVector2D::Distance(Agent->GetPosition(), FVector2D(LastKnown));

            if (Dist < ArrivalRadius)
            {
                // Phase 2: arrived — now wander to search the area
                bArrivedAtLastKnown = true;
                Agent->SetSteeringBehavior(WanderBehavior.get());
            }
        }
        // If already wandering, Wander handles itself each tick via SteeringAgent
    }
}