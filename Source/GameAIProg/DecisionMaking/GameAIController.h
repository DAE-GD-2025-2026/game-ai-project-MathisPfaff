#pragma once
#include "CoreMinimal.h"
#include "AIController.h"
#include "GameAIController.generated.h"

UCLASS()
class GAMEAIPROG_API AGameAIController : public AAIController
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="AI|FSM")
	TObjectPtr<UBlackboardData> FSMBlackboardAsset;

	// How far the guard can see
	UPROPERTY(EditAnywhere, Category="AI|Perception")
	float DetectionRadius{800.f};

	// How often (seconds) the visibility check runs — not every tick!
	UPROPERTY(EditAnywhere, Category="AI|Perception")
	float PerceptionInterval{0.3f};

	AGameAIController();
	virtual void Tick(float DeltaTime) override;

	void RunFiniteStateMachine();

protected:
	virtual void BeginPlay() override;
	void InitFiniteStateMachine();

private:
	// Called on a timer — checks radius + line of sight, writes to blackboard
	void UpdatePerception();

	FTimerHandle PerceptionTimerHandle;
};