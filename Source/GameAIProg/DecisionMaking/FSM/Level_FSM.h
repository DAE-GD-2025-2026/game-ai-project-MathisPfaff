#pragma once

#include <memory>
#include "CoreMinimal.h"
#include "Shared/Level_Base.h"
#include "Movement/SteeringBehaviors/Steering/SteeringBehaviors.h"
#include "Level_FSM.generated.h"

UCLASS()
class GAMEAIPROG_API ALevel_FSM : public ALevel_Base
{
	GENERATED_BODY()

public:
	ALevel_FSM();
	virtual void Tick(float DeltaTime) override;

protected:
	virtual void BeginPlay() override;
	virtual void BindLevelInputActions() override; // handle mouse click

private:
	// ── Thief (mouse-controlled) ──
	UPROPERTY()
	ASteeringAgent* Thief{nullptr};
	std::unique_ptr<Seek> ThiefBehavior; // Seek toward mouse click

	// ── Guard (FSM-controlled) ──
	UPROPERTY()
	ASteeringAgent* Guard{nullptr};

	// Patrol waypoints — set these in the editor Details panel
	UPROPERTY(EditAnywhere, Category="AI|Patrol")
	TArray<FVector2D> PatrolPoints;

	// How long guard searches before giving up
	UPROPERTY(EditAnywhere, Category="AI|Search")
	float SearchDuration{5.f};
};