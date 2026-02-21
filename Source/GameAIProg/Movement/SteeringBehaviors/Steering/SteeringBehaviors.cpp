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

//FACE
//*******
SteeringOutput Face::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};
	
	// Direction from agent to target
	FVector2D toTarget = Target.Position - Agent.GetPosition();

	// If we're basically on top of the target, don't rotate
	if (toTarget.IsNearlyZero(1.f))
		return steering;

	// Desired angle (in degrees) toward the target
	float desiredAngle = FMath::RadiansToDegrees(FMath::Atan2(toTarget.Y, toTarget.X));

	// Current facing angle (Yaw)
	float currentAngle = Agent.GetRotation();

	// Shortest angle difference (wraps around -180..180)
	float angleDiff = FMath::FindDeltaAngleDegrees(currentAngle, desiredAngle);

	// Output angular velocity proportional to how far we need to turn
	// This gives a smooth "arrive" style rotation (slows down as it nears the target angle)
	steering.AngularVelocity = angleDiff / DeltaT;

	// Clamp to max angular speed
	steering.AngularVelocity = FMath::Clamp(steering.AngularVelocity, 
		-Agent.GetMaxAngularSpeed(), Agent.GetMaxAngularSpeed());

	if (Agent.GetDebugRenderingEnabled())
	{
		DrawDebugPoint(Agent.GetWorld(), FVector(Target.Position, 0), 10.f, FColor::Red);
		DrawDebugLine(Agent.GetWorld(), FVector(Agent.GetPosition(), 0),
			FVector(Agent.GetPosition(), 0) + Agent.GetActorForwardVector() * 60, FColor::Magenta);
	}

	return steering;
}
