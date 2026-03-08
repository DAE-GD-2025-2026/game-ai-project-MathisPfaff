#pragma optimize("", off)
#include "Flock.h"
#include "FlockingSteeringBehaviors.h"
#include "Shared/ImGuiHelpers.h"

Flock::Flock(
    UWorld* pWorld,
    TSubclassOf<ASteeringAgent> AgentClass,
    int FlockSize,
    float WorldSize,
    ASteeringAgent* const pAgentToEvade,
    bool bTrimWorld)
    : pWorld{pWorld}
    , FlockSize{FlockSize}
    , pAgentToEvade{pAgentToEvade}
{Agents.SetNum(FlockSize);

#ifndef GAMEAI_USE_SPACE_PARTITIONING
    pNeighbors = new ASteeringAgent*[FlockSize];
#endif

    // --- Flocking blended behaviors ---
    pCohesionBehavior   = std::make_unique<Cohesion>(this);
    pSeparationBehavior = std::make_unique<Separation>(this);
    pVelMatchBehavior   = std::make_unique<VelocityMatch>(this);
    pSeekBehavior       = std::make_unique<Seek>();
    pWanderBehavior     = std::make_unique<Wander>();
    pEvadeBehavior      = std::make_unique<Evade>();

    std::vector<BlendedSteering::WeightedBehavior> WeightedBehaviors
    {
        { pCohesionBehavior.get(),   WeightCohesion      },
        { pSeparationBehavior.get(), WeightSeparation     },
        { pVelMatchBehavior.get(),   WeightVelocityMatch  },
        { pSeekBehavior.get(),       WeightSeek           },
        { pWanderBehavior.get(),     WeightWander         }
    };
    pBlendedSteering = std::make_unique<BlendedSteering>(WeightedBehaviors);

    // --- PrioritySteering: Evade first, then Blended ---
    pPrioritySteering = std::make_unique<PrioritySteering>(
        std::vector<ISteeringBehavior*>{ pEvadeBehavior.get(), pBlendedSteering.get() }
    );

    // --- Spawn EvadeTarget agent ---
    if (pWorld && AgentClass)
    {
        const float HalfWorld = WorldSize * 0.5f;
        FVector EvadeSpawnLoc(
            FMath::RandRange(-HalfWorld, HalfWorld),
            FMath::RandRange(-HalfWorld, HalfWorld),
            0.f);

        pEvadeTargetAgent = pWorld->SpawnActor<ASteeringAgent>(
            AgentClass, EvadeSpawnLoc, FRotator::ZeroRotator);

        if (pEvadeTargetAgent)
        {
            pEvadeTargetWander = std::make_unique<Wander>();
            pEvadeTargetSeek   = std::make_unique<Seek>();
            pEvadeTargetAgent->SetSteeringBehavior(pEvadeTargetWander.get());
            pEvadeTargetAgent->SetDebugRenderingEnabled(false);
            pEvadeTargetAgent->SetActorScale3D(FVector(2.5f, 2.5f, 2.5f));
        }
    }

    // --- Spawn flock agents ---
    const float HalfWorld = WorldSize * 0.5f;
    for (int i = 0; i < FlockSize; ++i)
    {
        const float RandX = FMath::RandRange(-HalfWorld, HalfWorld);
        const float RandY = FMath::RandRange(-HalfWorld, HalfWorld);
        FVector SpawnLocation(RandX, RandY, 0.f);

        Agents[i] = pWorld->SpawnActor<ASteeringAgent>(
            AgentClass, SpawnLocation, FRotator::ZeroRotator);

        if (Agents[i])
        {
            Agents[i]->SetDebugRenderingEnabled(false);
            Agents[i]->SetSteeringBehavior(pPrioritySteering.get());
        }
    }

#ifdef GAMEAI_USE_SPACE_PARTITIONING
    pPartitionedSpace = std::make_unique<CellSpace>(pWorld, WorldSize, WorldSize, 10, 10, FlockSize);
    AgentPrevPositions.SetNum(FlockSize);
    for (int i = 0; i < FlockSize; ++i)
    {
        if (Agents[i])
        {
            pPartitionedSpace->AddAgent(*Agents[i]);
            AgentPrevPositions[i] = FVector2D(Agents[i]->GetActorLocation());
        }
    }
#endif

}

Flock::~Flock()
{
#ifndef GAMEAI_USE_SPACE_PARTITIONING
    delete[] pNeighbors;
    pNeighbors = nullptr;
#endif
}

void Flock::Tick(float DeltaTime)
{
    if (pEvadeTargetAgent)
    {
        FSteeringParams evadeParams{};
        evadeParams.Position       = FVector2D(pEvadeTargetAgent->GetActorLocation());
        evadeParams.LinearVelocity = FVector2D(pEvadeTargetAgent->GetVelocity());
        pEvadeBehavior->SetTarget(evadeParams);
    }

    auto& behaviors = pBlendedSteering->GetWeightedBehaviorsRef();
    if (!bHasSeekTarget)
        behaviors[3].Weight = 0.f;

    for (int i = 0; i < FlockSize; ++i)
    {
        if (!Agents[i]) continue;
        Agents[i]->SetDebugRenderingEnabled(DebugRenderSteering);

#ifdef GAMEAI_USE_SPACE_PARTITIONING
        RegisterNeighbors(Agents[i], AgentPrevPositions[i]);
        AgentPrevPositions[i] = FVector2D(Agents[i]->GetActorLocation());
#else
        RegisterNeighbors(Agents[i]);
#endif
    }
}


void Flock::RenderDebug()
{
    if (DebugRenderNeighborhood)
        RenderNeighborhood();

    if (DebugRenderEvadeTarget)
        RenderEvadeTarget();

#ifdef GAMEAI_USE_SPACE_PARTITIONING
    if (DebugRenderPartitions)
        pPartitionedSpace->RenderCells();
#endif
}

void Flock::RenderEvadeTarget()
{
    if (!pWorld || !pEvadeTargetAgent) return;

    const FVector loc = pEvadeTargetAgent->GetActorLocation();
    const FVector loc3D = FVector(loc.X, loc.Y, 10.f);

    DrawDebugCircle(
        pWorld, loc3D, EvadeRadius, 32,
        FColor::Orange, false, -1.f, 0, 8.f,
        FVector(1, 0, 0), FVector(0, 1, 0));

    DrawDebugSphere(pWorld, loc3D, 50.f, 12, FColor::Red, false, -1.f);
}

void Flock::ImGuiRender(ImVec2 const& WindowPos, ImVec2 const& WindowSize)
{
#ifdef PLATFORM_WINDOWS
#pragma region UI
    {
        bool bWindowActive = true;
        ImGui::SetNextWindowPos(WindowPos);
        ImGui::SetNextWindowSize(WindowSize);
        ImGui::Begin("Gameplay Programming", &bWindowActive,
            ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

        ImGui::Text("CONTROLS");
        ImGui::Indent();
        ImGui::Text("LMB: place target");
        ImGui::Text("RMB: move cam.");
        ImGui::Text("Scrollwheel: zoom cam.");
        ImGui::Unindent();

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        ImGui::Text("STATS");
        ImGui::Indent();
        ImGui::Text("%.3f ms/frame", 1000.0f / ImGui::GetIO().Framerate);
        ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
        ImGui::Unindent();

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        ImGui::Text("Flocking");
        ImGui::Spacing();

        ImGui::Text("Behavior Weights");
        ImGui::Spacing();

        auto& behaviors = pBlendedSteering->GetWeightedBehaviorsRef();

        ImGuiHelpers::ImGuiSliderFloatWithSetter("Cohesion",
            behaviors[0].Weight, 0.f, 1.f,
            [this](float v) { pBlendedSteering->GetWeightedBehaviorsRef()[0].Weight = v; }, "%.2f");

        ImGuiHelpers::ImGuiSliderFloatWithSetter("Separation",
            behaviors[1].Weight, 0.f, 1.f,
            [this](float v) { pBlendedSteering->GetWeightedBehaviorsRef()[1].Weight = v; }, "%.2f");

        ImGuiHelpers::ImGuiSliderFloatWithSetter("Velocity Match",
            behaviors[2].Weight, 0.f, 1.f,
            [this](float v) { pBlendedSteering->GetWeightedBehaviorsRef()[2].Weight = v; }, "%.2f");

        ImGuiHelpers::ImGuiSliderFloatWithSetter("Seek",
            behaviors[3].Weight, 0.f, 1.f,
            [this](float v) { pBlendedSteering->GetWeightedBehaviorsRef()[3].Weight = v; }, "%.2f");

        ImGuiHelpers::ImGuiSliderFloatWithSetter("Wander",
            behaviors[4].Weight, 0.f, 1.f,
            [this](float v) { pBlendedSteering->GetWeightedBehaviorsRef()[4].Weight = v; }, "%.2f");

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        ImGui::Text("Evade Target");
        ImGui::Spacing();
        ImGui::Checkbox("Render Evade Target Radius", &DebugRenderEvadeTarget);

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        ImGui::Text("Debug Rendering");
        ImGui::Checkbox("Render Neighborhood (first 3 agents)", &DebugRenderNeighborhood);
        ImGui::Checkbox("Render Steering Debug", &DebugRenderSteering);
        ImGui::Checkbox("Render Partitions", &DebugRenderPartitions);

        ImGui::End();
    }
#pragma endregion
#endif
}

void Flock::RenderNeighborhood()
{
    if (!pWorld) return;

    const int agentsToDebug = FMath::Min(DebugNeighborhoodAgentCount, FlockSize);

    for (int i = 0; i < agentsToDebug; ++i)
    {
        ASteeringAgent* pAgent = Agents[i];
        if (!pAgent) continue;

        const FVector agentLoc   = pAgent->GetActorLocation();
        const FVector agentLoc3D = FVector(agentLoc.X, agentLoc.Y, 10.f);

#ifdef GAMEAI_USE_SPACE_PARTITIONING
        RegisterNeighbors(pAgent, FVector2D(pAgent->GetActorLocation()));
#else
        RegisterNeighbors(pAgent);
#endif

        DrawDebugCircle(pWorld, agentLoc3D, NeighborhoodRadius, 32,
            FColor::Yellow, false, -1.f, 0, 5.f,
            FVector(1,0,0), FVector(0,1,0));

#ifdef GAMEAI_USE_SPACE_PARTITIONING
        const TArray<ASteeringAgent*>& neighbors = pPartitionedSpace->GetNeighbors();
        for (int n = 0; n < NrOfNeighbors; ++n)
        {
            if (!neighbors[n]) continue;
            const FVector neighborLoc = neighbors[n]->GetActorLocation();
            DrawDebugLine(pWorld, agentLoc3D,
                FVector(neighborLoc.X, neighborLoc.Y, 10.f),
                FColor::Cyan, false, -1.f, 0, 2.f);
        }
#else
        for (int n = 0; n < NrOfNeighbors; ++n)
        {
            if (!pNeighbors[n]) continue;
            const FVector neighborLoc = pNeighbors[n]->GetActorLocation();
            DrawDebugLine(pWorld, agentLoc3D,
                FVector(neighborLoc.X, neighborLoc.Y, 10.f),
                FColor::Cyan, false, -1.f, 0, 2.f);
        }
#endif

        DrawDebugSphere(pWorld, agentLoc3D, 30.f, 8,
            i == 0 ? FColor::Red : (i == 1 ? FColor::Green : FColor::Blue),
            false, -1.f);
    }
}

#ifdef GAMEAI_USE_SPACE_PARTITIONING
void Flock::RegisterNeighbors(ASteeringAgent* const pAgent, const FVector2D& OldPos)
{
    if (!pAgent) return;
    pPartitionedSpace->UpdateAgentCell(*pAgent, OldPos);
    pPartitionedSpace->RegisterNeighbors(*pAgent, NeighborhoodRadius);
    NrOfNeighbors = pPartitionedSpace->GetNrOfNeighbors();
}
#else
void Flock::RegisterNeighbors(ASteeringAgent* const pAgent)
{
    NrOfNeighbors = 0;
    if (!pAgent) return;

    const FVector2D agentPos = FVector2D(pAgent->GetActorLocation());

    for (int i = 0; i < FlockSize; ++i)
    {
        ASteeringAgent* pOther = Agents[i];
        if (!pOther || pOther == pAgent) continue;

        const FVector2D otherPos = FVector2D(pOther->GetActorLocation());
        const float distSq = FVector2D::DistSquared(agentPos, otherPos);

        if (distSq <= NeighborhoodRadius * NeighborhoodRadius)
        {
            pNeighbors[NrOfNeighbors] = pOther;
            ++NrOfNeighbors;
        }
    }
}
#endif

FVector2D Flock::GetAverageNeighborPos() const
{
    FVector2D avgPosition = FVector2D::ZeroVector;
    if (NrOfNeighbors == 0) return avgPosition;

#ifdef GAMEAI_USE_SPACE_PARTITIONING
    const TArray<ASteeringAgent*>& neighbors = pPartitionedSpace->GetNeighbors();
    for (int i = 0; i < NrOfNeighbors; ++i)
        avgPosition += FVector2D(neighbors[i]->GetActorLocation());
#else
    for (int i = 0; i < NrOfNeighbors; ++i)
        avgPosition += FVector2D(pNeighbors[i]->GetActorLocation());
#endif

    return avgPosition / static_cast<float>(NrOfNeighbors);
}

FVector2D Flock::GetAverageNeighborVelocity() const
{
    FVector2D avgVelocity = FVector2D::ZeroVector;
    if (NrOfNeighbors == 0) return avgVelocity;

#ifdef GAMEAI_USE_SPACE_PARTITIONING
    const TArray<ASteeringAgent*>& neighbors = pPartitionedSpace->GetNeighbors();
    for (int i = 0; i < NrOfNeighbors; ++i)
        avgVelocity += FVector2D(neighbors[i]->GetVelocity());
#else
    for (int i = 0; i < NrOfNeighbors; ++i)
        avgVelocity += FVector2D(pNeighbors[i]->GetVelocity());
#endif

    return avgVelocity / static_cast<float>(NrOfNeighbors);
}

void Flock::SetTarget_Seek(FSteeringParams const& Target)
{
    if (pSeekBehavior)
    {
        pSeekBehavior->SetTarget(Target);
        if (!bHasSeekTarget)
        {
            pBlendedSteering->GetWeightedBehaviorsRef()[3].Weight = WeightSeek;
            bHasSeekTarget = true;
        }
    }
}
