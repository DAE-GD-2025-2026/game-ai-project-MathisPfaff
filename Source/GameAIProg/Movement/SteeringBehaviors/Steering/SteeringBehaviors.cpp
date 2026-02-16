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
			FVector(Agent.GetPosition(), 0) + Agent.GetVelocity() / 3, FColor::Green);
		DrawDebugLine(Agent.GetWorld(), FVector(Agent.GetPosition(), 0), 
			FVector(Agent.GetPosition(), 0) + Agent.GetActorForwardVector() * 60, FColor::Magenta);
		
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
			FVector(Agent.GetPosition(), 0) + Agent.GetVelocity() / 3, FColor::Green);
		DrawDebugLine(Agent.GetWorld(), FVector(Agent.GetPosition(), 0), 
			FVector(Agent.GetPosition(), 0) + Agent.GetActorForwardVector() * 60, FColor::Magenta);
	}
	
	return steering;
}


//ARRIVE
//*******
SteeringOutput Arrive::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};
	
	if (FirstTime)
	{
		MaxSpeed = Agent.GetMaxLinearSpeed();
		FirstTime = false;
	}
	
	steering.LinearVelocity = Target.Position - Agent.GetPosition();
	
	if (steering.LinearVelocity.Length() < SlowRadius)
	{
		if (steering.LinearVelocity.Length() < TargetRadius)
		{
			Agent.SetMaxLinearSpeed(0);
		}
		else
		{
			Agent.SetMaxLinearSpeed(MaxSpeed * (steering.LinearVelocity.Length() / SlowRadius));
		}
	}
	else
	{
		Agent.SetMaxLinearSpeed(MaxSpeed);
	}
	
	 
	
	if (Agent.GetDebugRenderingEnabled())
	{
		DrawDebugCircle(Agent.GetWorld(), FVector(Agent.GetPosition(), 0), TargetRadius, 16, 
			FColor::Orange, false, -1.f, 0,2.f, 
			FVector(0, 1, 0), FVector(1, 0, 0), false);
		DrawDebugCircle(Agent.GetWorld(), FVector(Agent.GetPosition(), 0), SlowRadius, 32, 
			FColor::Blue, false, -1.f, 0,2.f, 
			FVector(0, 1, 0), FVector(1, 0, 0), false);
		
		DrawDebugPoint(Agent.GetWorld(), FVector(Target.Position, 0), 10.f, FColor::Red);
		
		DrawDebugLine(Agent.GetWorld(), FVector(Agent.GetPosition(), 0), 
			FVector(Agent.GetPosition(), 0) + Agent.GetVelocity() / 3, FColor::Green);
		DrawDebugLine(Agent.GetWorld(), FVector(Agent.GetPosition(), 0), 
			FVector(Agent.GetPosition(), 0) + Agent.GetActorForwardVector() * 60, FColor::Magenta);
		
	}
	
	return steering;
}