#pragma once

// #define GAMEAI_USE_SPACE_PARTITIONING

#include "FlockingSteeringBehaviors.h"
#include "Movement/SteeringBehaviors/SteeringAgent.h"
#include "Movement/SteeringBehaviors/SteeringHelpers.h"
#include "Movement/SteeringBehaviors/CombinedSteering/CombinedSteeringBehaviors.h"
#include <memory>
#include "imgui.h"
#ifdef GAMEAI_USE_SPACE_PARTITIONING
#include "../SpacePartitioning/SpacePartitioning.h"
#endif

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
        float WorldSize = 100.f,
        ASteeringAgent* const pAgentToEvade = nullptr,
        bool bTrimWorld = false);

    ~Flock();

    void Tick(float DeltaTime);
    void RenderDebug();
    void ImGuiRender(ImVec2 const& WindowPos, ImVec2 const& WindowSize);

#ifdef GAMEAI_USE_SPACE_PARTITIONING
    //const TArray<ASteeringAgent*>& GetNeighbors() const { return pPartitionedSpace->GetNeighbors(); }
    //int GetNrOfNeighbors() const { return pPartitionedSpace->GetNrOfNeighbors(); }
#else
    void RegisterNeighbors(ASteeringAgent* const Agent);
    int GetNrOfNeighbors() const { return NrOfNeighbors; }
    const ASteeringAgent* const* GetNeighbors() const { return pNeighbors; }
#endif

    FVector2D GetAverageNeighborPos() const;
    FVector2D GetAverageNeighborVelocity() const;

    void SetTarget_Seek(FSteeringParams const& Target);

private:
    UWorld* pWorld{nullptr};

    int FlockSize{0};
    TArray<ASteeringAgent*> Agents{};

#ifdef GAMEAI_USE_SPACE_PARTITIONING
#else
    ASteeringAgent** pNeighbors{nullptr};
#endif

    float NeighborhoodRadius{200.f};
    int NrOfNeighbors{0};

    ASteeringAgent* pAgentToEvade{nullptr};

    // ---- EvadeTarget agent (spawned, identifiable) ----
    ASteeringAgent* pEvadeTargetAgent{nullptr};
    float EvadeRadius{300.f};
    EEvadeTargetBehavior EvadeTargetBehaviorMode{EEvadeTargetBehavior::Wander};

    // EvadeTarget's own steering
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

    static constexpr int DebugNeighborhoodAgentCount{3};

    void RenderNeighborhood();
    void RenderEvadeTarget();
};
