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
    pNeighbors = new ASteeringAgent*[FlockSize];

    pCohesionBehavior   = std::make_unique<Cohesion>(this);
    pSeparationBehavior = std::make_unique<Separation>(this);
    pVelMatchBehavior   = std::make_unique<VelocityMatch>(this);
    pSeekBehavior       = std::make_unique<Seek>();
    pWanderBehavior     = std::make_unique<Wander>();

    std::vector<BlendedSteering::WeightedBehavior> WeightedBehaviors
    {
        { pCohesionBehavior.get(),   WeightCohesion      },
        { pSeparationBehavior.get(), WeightSeparation     },
        { pVelMatchBehavior.get(),   WeightVelocityMatch  },
        { pSeekBehavior.get(),       WeightSeek           },
        { pWanderBehavior.get(),     WeightWander         }
    };

    pBlendedSteering = std::make_unique<BlendedSteering>(WeightedBehaviors);

    const float HalfWorld = WorldSize * 0.5f;
    for (int i = 0; i < FlockSize; ++i)
    {
        const float RandX = FMath::RandRange(-HalfWorld, HalfWorld);
        const float RandY = FMath::RandRange(-HalfWorld, HalfWorld);
        FVector SpawnLocation(RandX, RandY, 0.f);

        Agents[i] = pWorld->SpawnActor<ASteeringAgent>(AgentClass, SpawnLocation, FRotator::ZeroRotator);
        if (Agents[i])
        {
            Agents[i]->SetDebugRenderingEnabled(false);
            Agents[i]->SetSteeringBehavior(pBlendedSteering.get());
        }
            
    }
}

Flock::~Flock()
{
    delete[] pNeighbors;
    pNeighbors = nullptr;
}

void Flock::Tick(float DeltaTime)
{
    auto& behaviors = pBlendedSteering->GetWeightedBehaviorsRef();

    if (!bHasSeekTarget)
        behaviors[3].Weight = 0.f;

    for (int i = 0; i < FlockSize; ++i)
    {
        if (!Agents[i]) continue;

        // Suppress individual steering debug renders unless explicitly enabled
        const bool bWasDebug = Agents[i]->GetDebugRenderingEnabled();
        Agents[i]->SetDebugRenderingEnabled(DebugRenderSteering);

        RegisterNeighbors(Agents[i]);

        Agents[i]->SetDebugRenderingEnabled(bWasDebug);
    }
}


void Flock::RenderDebug()
{
    if (DebugRenderNeighborhood)
        RenderNeighborhood();
}

void Flock::ImGuiRender(ImVec2 const& WindowPos, ImVec2 const& WindowSize)
{
#ifdef PLATFORM_WINDOWS
#pragma region UI
    {
        bool bWindowActive = true;
        ImGui::SetNextWindowPos(WindowPos);
        ImGui::SetNextWindowSize(WindowSize);
        ImGui::Begin("Gameplay Programming", &bWindowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

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
            [this](float InVal) { pBlendedSteering->GetWeightedBehaviorsRef()[0].Weight = InVal; }, "%.2f");

        ImGuiHelpers::ImGuiSliderFloatWithSetter("Separation",
            behaviors[1].Weight, 0.f, 1.f,
            [this](float InVal) { pBlendedSteering->GetWeightedBehaviorsRef()[1].Weight = InVal; }, "%.2f");

        ImGuiHelpers::ImGuiSliderFloatWithSetter("Velocity Match",
            behaviors[2].Weight, 0.f, 1.f,
            [this](float InVal) { pBlendedSteering->GetWeightedBehaviorsRef()[2].Weight = InVal; }, "%.2f");

        ImGuiHelpers::ImGuiSliderFloatWithSetter("Seek",
            behaviors[3].Weight, 0.f, 1.f,
            [this](float InVal) { pBlendedSteering->GetWeightedBehaviorsRef()[3].Weight = InVal; }, "%.2f");

        ImGuiHelpers::ImGuiSliderFloatWithSetter("Wander",
            behaviors[4].Weight, 0.f, 1.f,
            [this](float InVal) { pBlendedSteering->GetWeightedBehaviorsRef()[4].Weight = InVal; }, "%.2f");

        
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();
        
        ImGui::Text("Debug Rendering");
        ImGui::Checkbox("Render Neighborhood (first 3 agents)", &DebugRenderNeighborhood);
        ImGui::Checkbox("Render Steering Debug", &DebugRenderSteering);
        
        
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

        const FVector agentLoc = pAgent->GetActorLocation();
        const FVector agentLoc3D = FVector(agentLoc.X, agentLoc.Y, 10.f);

        // Register neighbors specifically for this debug agent
        RegisterNeighbors(pAgent);

        // Snapshot neighbor count right after registering
        const int snapshotNeighborCount = NrOfNeighbors;

        // Draw neighborhood radius circle
        DrawDebugCircle(
            pWorld,
            agentLoc3D,
            NeighborhoodRadius,
            32,
            FColor::Yellow,
            false, -1.f, 0,
            5.f,
            FVector(1,0,0),
            FVector(0,1,0)
        );

        // Re-register neighbors for this agent to draw lines
        RegisterNeighbors(pAgent);

        for (int n = 0; n < NrOfNeighbors; ++n)
        {
            if (!pNeighbors[n]) continue;

            const FVector neighborLoc = pNeighbors[n]->GetActorLocation();

            DrawDebugLine(
                pWorld,
                agentLoc3D,
                FVector(neighborLoc.X, neighborLoc.Y, 10.f),
                FColor::Cyan,
                false, -1.f, 0,
                2.f
            );
        }

        // Draw a sphere on the debug agents so you can identify them
        DrawDebugSphere(
            pWorld,
            agentLoc3D,
            30.f,
            8,
            i == 0 ? FColor::Red : (i == 1 ? FColor::Green : FColor::Blue),
            false, -1.f
        );
    }
}

#ifndef GAMEAI_USE_SPACE_PARTITIONING
void Flock::RegisterNeighbors(ASteeringAgent* const pAgent)
{
    NrOfNeighbors = 0;

    if (!pAgent) return;

    const FVector2D agentPos = FVector2D(pAgent->GetActorLocation());

    for (int i = 0; i < FlockSize; ++i)
    {
        ASteeringAgent* pOther = Agents[i];
        if (!pOther || pOther == pAgent)  // <-- null check here
            continue;

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
    if (NrOfNeighbors == 0)
        return avgPosition;

    for (int i = 0; i < NrOfNeighbors; ++i)
        avgPosition += FVector2D(pNeighbors[i]->GetActorLocation());

    return avgPosition / static_cast<float>(NrOfNeighbors);
}

FVector2D Flock::GetAverageNeighborVelocity() const
{
    FVector2D avgVelocity = FVector2D::ZeroVector;
    if (NrOfNeighbors == 0)
        return avgVelocity;

    for (int i = 0; i < NrOfNeighbors; ++i)
        avgVelocity += FVector2D(pNeighbors[i]->GetVelocity());

    return avgVelocity / static_cast<float>(NrOfNeighbors);
}

void Flock::SetTarget_Seek(FSteeringParams const& Target)
{
    if (pSeekBehavior)
    {
        pSeekBehavior->SetTarget(Target);
        if (!bHasSeekTarget)
        {
            // Restore default weight on first target set
            pBlendedSteering->GetWeightedBehaviorsRef()[3].Weight = WeightSeek;
            bHasSeekTarget = true;
        }
    }
}
