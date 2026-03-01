#pragma optimize("", off)
#include "SteeringBehaviors.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"
#include "DrawDebugHelpers.h"
#include "MeshPaintVisualize.h"

//SEEK
//*******
SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};
	Agent.SetIsAutoOrienting(true);

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
	Agent.SetIsAutoOrienting(true);
	
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
	Agent.SetIsAutoOrienting(true);
	
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
	Agent.SetIsAutoOrienting(false);

	FVector2D toTarget = Target.Position - Agent.GetPosition();

	if (toTarget.IsNearlyZero())
	{
		return steering;
	}
	
	float desiredAngle = FMath::RadiansToDegrees(FMath::Atan2(toTarget.Y, toTarget.X));
	float currentAngle = Agent.GetRotation();
	float angleDiff = FMath::FindDeltaAngleDegrees(currentAngle, desiredAngle);

	steering.AngularVelocity = angleDiff / 180.f;

	if (Agent.GetDebugRenderingEnabled())
	{
		DrawDebugPoint(Agent.GetWorld(), FVector(Target.Position, 0), 10.f, FColor::Red);
		DrawDebugLine(Agent.GetWorld(), FVector(Agent.GetPosition(), 0),
			FVector(Agent.GetPosition(), 0) + Agent.GetActorForwardVector() * 60, FColor::Magenta);
	}

	return steering;
}

//PURSUIT
//*******
SteeringOutput Pursuit::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};
	Agent.SetIsAutoOrienting(true);
	
	FVector2D toTarget = Target.Position - Agent.GetPosition();
	float const timeToTarget = toTarget.Length() / Agent.GetMaxLinearSpeed();
	
	FVector2D predictedPosition = Target.Position + Target.LinearVelocity * timeToTarget;
	steering.LinearVelocity = predictedPosition - Agent.GetPosition();
	
	if (Agent.GetDebugRenderingEnabled())
	{
		DrawDebugPoint(Agent.GetWorld(), FVector(predictedPosition, 0), 10.f, FColor::Red);
		DrawDebugLine(Agent.GetWorld(), FVector(Agent.GetPosition(), 0), 
			FVector(Agent.GetPosition(), 0) + Agent.GetVelocity() / 3, FColor::Green);
		DrawDebugLine(Agent.GetWorld(), FVector(Agent.GetPosition(), 0), 
			FVector(Agent.GetPosition(), 0) + Agent.GetActorForwardVector() * 60, FColor::Magenta);
	}
	
	return steering;
}

//EVADE
//*******
SteeringOutput Evade::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};
	Agent.SetIsAutoOrienting(true);
	float const evadeDistance {500.f};
	
	FVector2D toTarget = Target.Position - Agent.GetPosition();
	float const timeToTarget = toTarget.Length() / Agent.GetMaxLinearSpeed();
	
	FVector2D predictedPosition = Target.Position + Target.LinearVelocity * timeToTarget;
	
	if (toTarget.Length() < evadeDistance)
	{
		steering.LinearVelocity = Agent.GetPosition() - predictedPosition;
	}
	
	if (Agent.GetDebugRenderingEnabled())
	{
		DrawDebugCircle(Agent.GetWorld(), FVector(Agent.GetPosition(), 0), evadeDistance, 16, 
			FColor::Orange, false, -1.f, 0,2.f, 
			FVector(0, 1, 0), FVector(1, 0, 0), false);
		
		DrawDebugPoint(Agent.GetWorld(), FVector(predictedPosition, 0), 10.f, FColor::Red);
		DrawDebugLine(Agent.GetWorld(), FVector(Agent.GetPosition(), 0), 
			FVector(Agent.GetPosition(), 0) + Agent.GetVelocity() / 3, FColor::Green);
		DrawDebugLine(Agent.GetWorld(), FVector(Agent.GetPosition(), 0), 
			FVector(Agent.GetPosition(), 0) + Agent.GetActorForwardVector() * 60, FColor::Magenta);
	}
	
	return steering;
}

//WANDER
//*******
SteeringOutput Wander::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};
	Agent.SetIsAutoOrienting(true);
	
	// Move the wander angle by a random amount, clamped to MaxAngleChange
	float angleChange = FMath::RandRange(-m_MaxAngleChange, m_MaxAngleChange);
	m_WanderAngle += angleChange;

	// Get the circle center: project it in front of the agent
	FVector2D agentPos = Agent.GetPosition();
	FVector2D forward = FVector2D(Agent.GetActorForwardVector());
	forward.Normalize();

	FVector2D circleCenter = agentPos + forward * m_OffsetDistance;

	// Calculate the point on the circle's edge using WanderAngle
	FVector2D pointOnCircle = circleCenter + FVector2D(
		FMath::Cos(FMath::DegreesToRadians(m_WanderAngle)),
		FMath::Sin(FMath::DegreesToRadians(m_WanderAngle))
	) * m_Radius;

	// Steer towards that point
	steering.LinearVelocity = pointOnCircle - agentPos;

	if (Agent.GetDebugRenderingEnabled())
	{
		DrawDebugCircle(Agent.GetWorld(), FVector(circleCenter, 0), m_Radius, 32,
			FColor::Yellow, false, -1.f, 0, 2.f,
			FVector(0, 1, 0), FVector(1, 0, 0), false);

		DrawDebugPoint(Agent.GetWorld(), FVector(pointOnCircle, 0), 10.f, FColor::Red);

		DrawDebugLine(Agent.GetWorld(), FVector(agentPos, 0),
			FVector(agentPos, 0) + Agent.GetVelocity() / 3, FColor::Green);
		DrawDebugLine(Agent.GetWorld(), FVector(agentPos, 0),
			FVector(agentPos, 0) + Agent.GetActorForwardVector() * 60, FColor::Magenta);
	}
	
	return steering;
}