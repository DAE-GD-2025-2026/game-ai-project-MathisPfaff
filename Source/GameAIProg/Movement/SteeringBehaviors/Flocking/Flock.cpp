#pragma optimize("", off)
#include "Flock.h"
#include "FlockingSteeringBehaviors.h"
#include "Shared/ImGuiHelpers.h"
#include "Movement/SteeringBehaviors/SpacePartitioning/SpacePartitioning.h"

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

    // Always allocate fallback neighbor array
    pNeighbors = new ASteeringAgent*[FlockSize];

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

        FActorSpawnParameters SpawnParams;
        SpawnParams.SpawnCollisionHandlingOverride =
            ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

        pEvadeTargetAgent = pWorld->SpawnActor<ASteeringAgent>(
            AgentClass, EvadeSpawnLoc, FRotator::ZeroRotator, SpawnParams);

        if (pEvadeTargetAgent)
        {
            pEvadeTargetWander = std::make_unique<Wander>();
            pEvadeTargetSeek   = std::make_unique<Seek>();
            pEvadeTargetAgent->SetSteeringBehavior(pEvadeTargetWander.get());
            pEvadeTargetAgent->SetDebugRenderingEnabled(false);
            pEvadeTargetAgent->SetActorScale3D(FVector(2.5f, 2.5f, 2.5f));
            // Disable auto-tick; Flock drives everything manually
            pEvadeTargetAgent->SetActorTickEnabled(false);
        }
    }

    // --- Spawn flock agents ---
    const float HalfWorld = WorldSize * 0.5f;
    for (int i = 0; i < FlockSize; ++i)
    {
        FActorSpawnParameters SpawnParams;
        SpawnParams.SpawnCollisionHandlingOverride =
            ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

        const float RandX = FMath::RandRange(-HalfWorld, HalfWorld);
        const float RandY = FMath::RandRange(-HalfWorld, HalfWorld);
        FVector SpawnLocation(RandX, RandY, 0.f);

        Agents[i] = pWorld->SpawnActor<ASteeringAgent>(
            AgentClass, SpawnLocation, FRotator::ZeroRotator, SpawnParams);

        if (Agents[i])
        {
            Agents[i]->SetDebugRenderingEnabled(false);
            Agents[i]->SetSteeringBehavior(pPrioritySteering.get());
            // Disable auto-tick so we control update order
            Agents[i]->SetActorTickEnabled(false);
        }
    }

    // --- Create CellSpace (always; toggled at runtime) ---
    pPartitionedSpace = std::make_unique<CellSpace>(
        pWorld, WorldSize, WorldSize, 10, 10, FlockSize);

    AgentPrevPositions.SetNum(FlockSize);
    for (int i = 0; i < FlockSize; ++i)
    {
        if (Agents[i])
        {
            pPartitionedSpace->AddAgent(*Agents[i]);
            AgentPrevPositions[i] = FVector2D(Agents[i]->GetActorLocation());
        }
    }
}

Flock::~Flock()
{
    delete[] pNeighbors;
    pNeighbors = nullptr;
    // pPartitionedSpace is a unique_ptr — cleaned up automatically
}

// --- Unified neighbor accessors (runtime toggle) ---

int Flock::GetNrOfNeighbors() const
{
    if (bUseSpacePartitioning && pPartitionedSpace)
        return pPartitionedSpace->GetNrOfNeighbors();
    return NrOfNeighbors;
}

const ASteeringAgent* const* Flock::GetNeighbors() const
{
    if (bUseSpacePartitioning && pPartitionedSpace)
        return pPartitionedSpace->GetNeighbors().GetData();
    return pNeighbors;
}

// --- RegisterNeighbors overloads ---

void Flock::RegisterNeighbors(ASteeringAgent* const pAgent, const FVector2D& OldPos)
{
    if (!pAgent) return;
    if (bUseSpacePartitioning && pPartitionedSpace)
    {
        pPartitionedSpace->UpdateAgentCell(*pAgent, OldPos);
        pPartitionedSpace->RegisterNeighbors(*pAgent, NeighborhoodRadius);
    }
    else
    {
        RegisterNeighbors(pAgent);  // fallback
    }
}

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

void Flock::Tick(float DeltaTime)
{
    // Update evade target
    if (pEvadeTargetAgent)
    {
        pEvadeTargetAgent->Tick(DeltaTime);  // manual tick

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

        if (bUseSpacePartitioning && pPartitionedSpace)
            RegisterNeighbors(Agents[i], AgentPrevPositions[i]);
        else
            RegisterNeighbors(Agents[i]);

        // Manual tick for each agent
        Agents[i]->Tick(DeltaTime);

        // Update previous position after tick
        if (bUseSpacePartitioning)
            AgentPrevPositions[i] = FVector2D(Agents[i]->GetActorLocation());
    }
}

void Flock::RenderDebug()
{
    if (DebugRenderNeighborhood)
        RenderNeighborhood();

    if (DebugRenderEvadeTarget)
        RenderEvadeTarget();

    if (DebugRenderPartitions && bUseSpacePartitioning && pPartitionedSpace)
        pPartitionedSpace->RenderCells();
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

        // --- Space Partitioning toggle ---
        ImGui::Text("Space Partitioning");
        if (ImGui::Checkbox("Enable Space Partitioning", &bUseSpacePartitioning))
        {
            // When switching TO space partitioning, rebuild the cell registrations
            if (bUseSpacePartitioning && pPartitionedSpace)
            {
                pPartitionedSpace->EmptyCells();
                for (int i = 0; i < FlockSize; ++i)
                {
                    if (Agents[i])
                    {
                        pPartitionedSpace->AddAgent(*Agents[i]);
                        AgentPrevPositions[i] = FVector2D(Agents[i]->GetActorLocation());
                    }
                }
            }
        }

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
        ImGui::Checkbox("Render Neighborhood (first agent)", &DebugRenderNeighborhood);
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

        // Re-register neighbors for debug draw
        if (bUseSpacePartitioning && pPartitionedSpace)
            RegisterNeighbors(pAgent, FVector2D(agentLoc));
        else
            RegisterNeighbors(pAgent);

        // Draw neighborhood radius circle
        DrawDebugCircle(pWorld, agentLoc3D, NeighborhoodRadius, 32,
            FColor::Yellow, false, -1.f, 0, 5.f,
            FVector(1,0,0), FVector(0,1,0));

        // Draw query bounding box (useful for partitioning debug)
        if (bUseSpacePartitioning)
        {
            const FVector BoxMin(agentLoc.X - NeighborhoodRadius, agentLoc.Y - NeighborhoodRadius, 5.f);
            const FVector BoxMax(agentLoc.X + NeighborhoodRadius, agentLoc.Y + NeighborhoodRadius, 5.f);
            DrawDebugBox(pWorld, agentLoc3D,
                FVector(NeighborhoodRadius, NeighborhoodRadius, 1.f),
                FColor::Magenta, false, -1.f, 0, 3.f);
        }

        // Draw lines to neighbors
        const int nrNeighbors = GetNrOfNeighbors();
        const ASteeringAgent* const* neighbors = GetNeighbors();
        for (int n = 0; n < nrNeighbors; ++n)
        {
            if (!neighbors[n]) continue;
            const FVector neighborLoc = neighbors[n]->GetActorLocation();
            DrawDebugLine(pWorld, agentLoc3D,
                FVector(neighborLoc.X, neighborLoc.Y, 10.f),
                FColor::Cyan, false, -1.f, 0, 2.f);
        }

        DrawDebugSphere(pWorld, agentLoc3D, 30.f, 8,
            i == 0 ? FColor::Red : (i == 1 ? FColor::Green : FColor::Blue),
            false, -1.f);
    }
}

FVector2D Flock::GetAverageNeighborPos() const
{
    FVector2D avgPosition = FVector2D::ZeroVector;
    const int nrNeighbors = GetNrOfNeighbors();
    if (nrNeighbors == 0) return avgPosition;

    const ASteeringAgent* const* neighbors = GetNeighbors();
    for (int i = 0; i < nrNeighbors; ++i)
        avgPosition += FVector2D(neighbors[i]->GetActorLocation());

    return avgPosition / static_cast<float>(nrNeighbors);
}

FVector2D Flock::GetAverageNeighborVelocity() const
{
    FVector2D avgVelocity = FVector2D::ZeroVector;
    const int nrNeighbors = GetNrOfNeighbors();
    if (nrNeighbors == 0) return avgVelocity;

    const ASteeringAgent* const* neighbors = GetNeighbors();
    for (int i = 0; i < nrNeighbors; ++i)
        avgVelocity += FVector2D(neighbors[i]->GetVelocity());

    return avgVelocity / static_cast<float>(nrNeighbors);
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
