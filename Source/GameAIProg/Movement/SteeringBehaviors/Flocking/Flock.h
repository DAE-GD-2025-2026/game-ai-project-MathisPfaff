#pragma once

#include "FlockingSteeringBehaviors.h"
#include "Movement/SteeringBehaviors/SteeringAgent.h"
#include "Movement/SteeringBehaviors/SteeringHelpers.h"
#include "Movement/SteeringBehaviors/CombinedSteering/CombinedSteeringBehaviors.h"
#include <memory>
#include "imgui.h"

// Forward declaration — avoids including SpacePartitioning.h here
class CellSpace;

enum class EEvadeTargetBehavior : uint8
{
    Wander = 0,
    Seek   = 1
};

class Flock final
{
public:
    Flock(
        UWorld* pWorld,
        TSubclassOf<ASteeringAgent> AgentClass,
        int FlockSize = 10,
        float WorldSize = 1000.f,
        ASteeringAgent* const pAgentToEvade = nullptr,
        bool bTrimWorld = false);

    ~Flock();

    void Tick(float DeltaTime);
    void RenderDebug();
    void ImGuiRender(ImVec2 const& WindowPos, ImVec2 const& WindowSize);

    void RegisterNeighbors(ASteeringAgent* const Agent, const FVector2D& OldPos);
    void RegisterNeighbors(ASteeringAgent* const Agent);

    int GetNrOfNeighbors() const;
    const ASteeringAgent* const* GetNeighbors() const;  // unified accessor

    FVector2D GetAverageNeighborPos() const;
    FVector2D GetAverageNeighborVelocity() const;

    void SetTarget_Seek(FSteeringParams const& Target);
    void SetTarget_EvadeTarget(FSteeringParams const& Target);
    

private:
    UWorld* pWorld{nullptr};

    int FlockSize{0};
    TArray<ASteeringAgent*> Agents{};

    // --- Space Partitioning ---
    std::unique_ptr<CellSpace> pPartitionedSpace{};
    TArray<FVector2D> AgentPrevPositions{};
    bool bUseSpacePartitioning{true};

    // --- Fallback neighbors ---
    ASteeringAgent** pNeighbors{nullptr};
    int NrOfNeighbors{0};

    float NeighborhoodRadius{200.f};

    ASteeringAgent* pAgentToEvade{nullptr};

    // ---- EvadeTarget agent ----
    ASteeringAgent* pEvadeTargetAgent{nullptr};
    float EvadeRadius{300.f};
    EEvadeTargetBehavior EvadeTargetBehaviorMode{EEvadeTargetBehavior::Wander};

    std::unique_ptr<Wander> pEvadeTargetWander{};
    std::unique_ptr<Seek>   pEvadeTargetSeek{};

    // Steering behaviors for flock
    std::unique_ptr<Separation>    pSeparationBehavior{};
    std::unique_ptr<Cohesion>      pCohesionBehavior{};
    std::unique_ptr<VelocityMatch> pVelMatchBehavior{};
    std::unique_ptr<Seek>          pSeekBehavior{};
    std::unique_ptr<Wander>        pWanderBehavior{};
    std::unique_ptr<Evade>         pEvadeBehavior{};

    std::unique_ptr<BlendedSteering>  pBlendedSteering{};
    std::unique_ptr<PrioritySteering> pPrioritySteering{};

    float WeightCohesion      {0.2f};
    float WeightSeparation    {0.2f};
    float WeightVelocityMatch {0.2f};
    float WeightSeek          {0.2f};
    float WeightWander        {0.2f};

    bool bHasSeekTarget{false};

    // UI and rendering
    bool DebugRenderSteering{false};
    bool DebugRenderNeighborhood{true};
    bool DebugRenderPartitions{true};
    bool DebugRenderEvadeTarget{true};

    static constexpr int DebugNeighborhoodAgentCount{1};

    void RenderNeighborhood();
    void RenderEvadeTarget();
};
