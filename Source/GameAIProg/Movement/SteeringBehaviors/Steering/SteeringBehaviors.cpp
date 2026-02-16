#include "SteeringBehaviors.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"
#include "DrawDebugHelpers.h"
#include "MeshPaintVisualize.h"

//SEEK
//*******
SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};

	steering.LinearVelocity = Target.Position - Agent.GetPosition();
	
	if (Agent.GetDebugRenderingEnabled())
	{
		DrawDebugPoint(Agent.GetWorld(), FVector(Target.Position, 0), 10.f, FColor::Red);
		DrawDebugLine(Agent.GetWorld(), FVector(Agent.GetPosition(), 0), 
			FVector(Agent.GetPosition(), 0) + Agent.GetVelocity() / 5, FColor::Green);
		DrawDebugLine(Agent.GetWorld(), FVector(Agent.GetPosition(), 0), 
			FVector(Agent.GetPosition(), 0) + Agent.GetActorForwardVector() * 80, FColor::Magenta);
		
	}

	return steering;
}


//FlEE
//*******
SteeringOutput Flee::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};
	
	steering.LinearVelocity = -(Target.Position - Agent.GetPosition());
	
	if (Agent.GetDebugRenderingEnabled())
	{
		DrawDebugPoint(Agent.GetWorld(), FVector(Target.Position, 0), 10.f, FColor::Red);
		DrawDebugLine(Agent.GetWorld(), FVector(Agent.GetPosition(), 0), 
			FVector(Agent.GetPosition(), 0) + Agent.GetVelocity() / 5, FColor::Green);
		DrawDebugLine(Agent.GetWorld(), FVector(Agent.GetPosition(), 0), 
			FVector(Agent.GetPosition(), 0) + Agent.GetActorForwardVector() * 80, FColor::Magenta);
	}
	
	return steering;
}


//ARRIVE
//*******
SteeringOutput Arrive::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};
	float slowRadius{500}, targetRadius{100};
	
	steering.LinearVelocity = Target.Position - Agent.GetPosition();
	
	 
	
	if (Agent.GetDebugRenderingEnabled())
	{
		DrawDebugCircle(Agent.GetWorld(), FVector(Agent.GetPosition(), 0), targetRadius, 16, 
			FColor::Orange, false, -1.f, 0,2.f, 
			FVector(0, 1, 0), FVector(1, 0, 0), false);
		DrawDebugCircle(Agent.GetWorld(), FVector(Agent.GetPosition(), 0), slowRadius, 32, 
			FColor::Blue, false, -1.f, 0,2.f, 
			FVector(0, 1, 0), FVector(1, 0, 0), false);
		
		DrawDebugPoint(Agent.GetWorld(), FVector(Target.Position, 0), 10.f, FColor::Red);
		
		DrawDebugLine(Agent.GetWorld(), FVector(Agent.GetPosition(), 0), 
			FVector(Agent.GetPosition(), 0) + Agent.GetVelocity() / 5, FColor::Green);
		DrawDebugLine(Agent.GetWorld(), FVector(Agent.GetPosition(), 0), 
			FVector(Agent.GetPosition(), 0) + Agent.GetActorForwardVector() * 80, FColor::Magenta);
		
	}
	
	return steering;
}