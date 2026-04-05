#include "Level_GraphTheory.h"

#include "Algorithms/EulerianPath.h"
#include "Shared/GameAISpectator.h"

using namespace GameAI;

ALevel_GraphTheory::ALevel_GraphTheory()
{
    PrimaryActorTick.bCanEverTick = true;
}

void ALevel_GraphTheory::BeginPlay()
{
    Super::BeginPlay();

    // Add the graph editor to our player
    if (PlayerController = Cast<APlayerController>(
            GetWorld()->GetFirstLocalPlayerFromController()->PlayerController);
        GraphEditorClass && PlayerController)
    {
        PlayerGraphEditor = NewObject<UGraphEditorComponent>(
            PlayerController->GetPawn(), GraphEditorClass);
        PlayerGraphEditor->RegisterComponent();
        PlayerGraphEditor->SetEditedGraph(&Graph);
        PlayerGraphEditor->SetNodeFactory(&NodeFactory);
    }
    else
    {
        UE_LOG(LogTemp, Error,
            TEXT("Unable to get PlayerController or GraphEditorClass is null"))
        return;
    }

    // Orthographic camera
    if (AGameAISpectator* Player =
            Cast<AGameAISpectator>(PlayerController->GetPawnOrSpectator()); Player)
    {
        Player->SetCameraProjection(ECameraProjectionMode::Orthographic);
    }

    // Fix the Renderer (was constructed with nullptr world)
    Renderer = GraphRenderer(GetWorld());

    // ── Build an initial Eulerian graph (square → all nodes degree 2) ─────
    Graph.AddNode(std::make_unique<Node>(FVector2D(-300.f,  300.f)));  // id 0
    Graph.AddNode(std::make_unique<Node>(FVector2D( 300.f,  300.f)));  // id 1
    Graph.AddNode(std::make_unique<Node>(FVector2D( 300.f, -300.f)));  // id 2
    Graph.AddNode(std::make_unique<Node>(FVector2D(-300.f, -300.f)));  // id 3

    Graph.AddConnection(0, 1);
    Graph.AddConnection(1, 2);
    Graph.AddConnection(2, 3);
    Graph.AddConnection(3, 0);
    Graph.AddConnection(0, 2);

    // ── Spawn agent ───────────────────────────────────────────────────────
    Agent = GetWorld()->SpawnActor<ASteeringAgent>(
        SteeringAgentClass, FVector{0, 0, 90}, FRotator::ZeroRotator);
    if (!Agent) return;

    Agent->SetSteeringBehavior(&PathFollow);

    // ── Run initial Eulerian path ─────────────────────────────────────────
    EulerianPath eulerianAlgo(&Graph);
    Eulerianity eulerianity;
    std::vector<Node*> trail = eulerianAlgo.FindPath(eulerianity);
    if (!trail.empty())
        UpdateAgentPath(trail);
}

void ALevel_GraphTheory::BeginDestroy()
{
    Super::BeginDestroy();
}

void ALevel_GraphTheory::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

#pragma region UI
    {
        bool windowActive = true;
        ImGui::SetNextWindowPos(WindowPos);
        ImGui::SetNextWindowSize(WindowSize);
        ImGui::Begin("Gameplay Programming", &windowActive,
            ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
            ImGuiWindowFlags_NoCollapse);
        ImGui::SetWindowFocus();
        ImGui::PushItemWidth(70);

        ImGui::Text("CONTROLS");
        ImGui::Indent();
        ImGui::Text("LMB (empty)    - Create node");
        ImGui::Text("LMB (on node)  - Start/finish connection");
        ImGui::Text("RMB (on node)  - Delete node");
        ImGui::Text("MMB (on node)  - Move node");
        ImGui::Unindent();

        ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();

        ImGui::Text("STATS");
        ImGui::Indent();
        ImGui::Text("%.3f ms/frame", 1000.0f / ImGui::GetIO().Framerate);
        ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
        ImGui::Unindent();

        ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();
        ImGui::Text("Graph Theory");

        ImGui::End();
    }
#pragma endregion UI

    Renderer.RenderGraph(Graph);

    // ── Check if the graph was modified this frame ────────────────────────
    if (PlayerGraphEditor && PlayerGraphEditor->HasGraphUpdated())
    {
        EulerianPath eulerianAlgo(&Graph);
        Eulerianity eulerianity;
        std::vector<Node*> trail = eulerianAlgo.FindPath(eulerianity);

        // If a valid Euler path/circuit exists → reset agent to follow it
        if (eulerianity != Eulerianity::notEulerian && !trail.empty())
        {
            UpdateAgentPath(trail);
        }
    }
}

void ALevel_GraphTheory::UpdateAgentPath(std::vector<Node*> const& Trail)
{
    std::vector<FVector2D> path{};
    for (Node const* node : Trail)
        path.push_back(node->GetPosition());

    PathFollow.SetPath(path);

    if (!path.empty() && Agent)
    {
        Agent->SetMaxLinearSpeed(600.f);
        Agent->SetPosition(path[0]);
    }
}
