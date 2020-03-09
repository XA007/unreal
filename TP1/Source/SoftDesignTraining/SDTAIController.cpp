// Fill out your copyright notice in the Description page of Project Settings.

#include "SoftDesignTraining.h"
#include "SDTAIController.h"
#include "SoftDesignTrainingCharacter.h"
#include "SoftDesignTrainingPlayerController.h"
#include "Engine.h"

/**
 * Function called every tick. 
 *
 * All the behaviour and movement of the agent are calculated here.
 *
 * @param deltaTime Time between ticks
 */
void ASDTAIController::Tick(float DeltaTime)
{
	if (_character == nullptr)
		_character = Cast<ASoftDesignTrainingCharacter>(GetCharacter());
	_actorLocation = GetPawn()->GetActorLocation();
	EnvironmentAnalysis(); // update radar that contains infos about obstacles and Player position
	DefineState();
	Perform(DeltaTime);
	DislayInfo();
}

/**
* Change the state of the agent
*
* The state is changed depending on the elements the agent see in his surrounding.   
*/
void ASDTAIController::DefineState()
{
	mState = Free;
	if (mRadar->detectedPlayer != nullptr && !SweepSingleObstacle(GetPawn()->GetActorLocation(), mRadar->detectedPlayer->posVector))
	{
		mState = (SDTUtils::IsPlayerPoweredUp(GetWorld())) ? Fleeing : Chasing;
	}
    else if (mRadar->targetCollectible != nullptr)
    {
        mState = PickUp;
    }    
}

/**
* Perform actions.
*
* The performed action depends on the state of the agent.
*
* @param deltaTime Time between ticks
*/
void ASDTAIController::Perform(float deltaTime)
{	
	if (_character->Debug)
		DebugMovement();

	switch(mState)
	{
	case Free:
		RandomMove(deltaTime);
		break;
	case Chasing:
        CalculateSpeedVectorRotation(GetTurningAngle(), deltaTime);
        Move(deltaTime, _character->StraightLineMaxSpeed, _character->StraightLineMaxSpeed);
        break;
    case Fleeing:
        RandomMove(deltaTime);
        break;
    case PickUp:
        MoveToTarget(mRadar->targetCollectible->posVector, deltaTime);
        break;
    default:
        RandomMove(deltaTime);
        break;
	}
}

/**
* Random Move.
*
* Make the character move orthogonally while taking into account
* the obtacles and the player character if he is powered up.
*
* @param deltaTime Time between ticks
*/
void ASDTAIController::RandomMove(float deltaTime)
{
	float maxSpeed = 0.0f, acceleration= 0.0f;
	
	switch (mMovement)
	{
	case Straigth:
		maxSpeed = _character->StraightLineMaxSpeed;
		acceleration = _character->StraightLineAcceleration;
		if (mRadar->obstacles->bIsForward || mState == Fleeing) 
		{
			CalculateTurningDirection();
		}
		break;
	case OneEighty:
		maxSpeed = _character->OneEightyMaxSpeed;
		acceleration = -_character->OneEightySpeedDeceleration;
		if (CalculateSpeedVectorRotation(GetTurningAngle(), deltaTime))
		{
			mDirection = mNextDirection;
			mMovement = Straigth;
		}
		break;
	case Turning:
		maxSpeed = _character->TurningMaxSpeed;
		acceleration = - _character->TurningSpeedDeceleration;
		if(CalculateSpeedVectorRotation(GetTurningAngle(), deltaTime))
		{
			mDirection = mNextDirection;
			mMovement = Straigth;
		}
		break;
	default:

		break;
	}

	Move(deltaTime, acceleration, maxSpeed);	
}

/**
* Move toward a targetted position.
*
* The agent will advance and turn in the direction. The speed and the angular speed depends on exposed parameters.
*
* @param targetPos 3D coordinates in the world
* @param deltaTime Time between ticks
*/
void ASDTAIController::MoveToTarget(FVector targetPos, float deltaTime)
{
    float maxSpeed = _character->StraightLineMaxSpeed, acceleration = _character->StraightLineAcceleration;

    FVector pawnPosition = GetPawn()->GetActorLocation();
    FVector pickUpPosition = targetPos;
    FVector direction = pickUpPosition - pawnPosition;
    direction.Normalize();
    
    CalculateSpeedVectorRotation(FMath::RadiansToDegrees(direction.HeadingAngle()), deltaTime);

    Move(deltaTime, acceleration, maxSpeed);
}

/**
 * Calculate the speed vector.
 *
 * This accelerate (or decelerate) the speed vector until it reach its max speed.
 *
 * @param deltaTime Time between ticks
 * @param acceleration Acceleration rate
 * @param maxSpeed Max Speed
 */
void ASDTAIController::Move(float deltaTime, float acceleration, float maxSpeed)
{
	if (acceleration >= 0.0f)
	{
		if (mSpeed < maxSpeed)
			mSpeed += acceleration * deltaTime;

		if (mSpeed > maxSpeed)
			mSpeed = maxSpeed;
	}
	else
	{
		if (mSpeed > maxSpeed)
			mSpeed += acceleration * deltaTime;

		if (mSpeed < maxSpeed)
			mSpeed = maxSpeed;
	}
	const FVector nextLoc = _actorLocation + mSpeedVector * mSpeed * deltaTime;
	GetPawn()->SetActorLocation(nextLoc, true);
}

/**
 * Rotate the speed vector around the Z axis.
 *
 * This rotate the speed vector gradually until it reach the desired angle.
 *
 * @param deltaTime Time between ticks
 * @param angleToReach Desired world angle (0.0f is Up, 180.0f is Down, 90.0f is Right, -90.0f is Left)
 * @return true if the rotation is complete
 */
bool ASDTAIController::CalculateSpeedVectorRotation(float angleToReach ,float deltaTime)
{
	float currentAngle = GetPawn()->GetActorRotation().Yaw;
	if (!FMath::IsNearlyEqual(currentAngle, angleToReach, 0.01f))
	{
		float angleVariation = _character->RotationRate * deltaTime;
		float differenceAbsolue = FMath::Abs(currentAngle - angleToReach);

		if (angleVariation > differenceAbsolue)
			angleVariation = differenceAbsolue;

		if (angleToReach < currentAngle)
			angleVariation *= -1.0f;

		if (differenceAbsolue > 180.0f)
			angleVariation *= -1.0f;

		mSpeedVector = mSpeedVector.RotateAngleAxis(angleVariation, FVector(0.0f, 0.0f, 1.0f));
		CalculatePawnOrientation();

		return false;
	}

	return true;
}


/**
 * Get Turning Angle
 *
 * Get the world angle that the character is trying to reach
 *
 * @return the world angle that the character is trying to reach
 */
float ASDTAIController::GetTurningAngle() const
{
	if (mState == Chasing) 
		return mRadar->detectedPlayer->angle;
	
	switch (mNextDirection)
	{
	case Up:
		return 0.0f;
	case Down:
		return 180.0f;
	case Right:
		return 90.0f;
	case Left:
		return -90.0f;
	default:
		return 0.0f;
	}
}

/**
 * Rotate the pawn towards the speed vector.
 *
 * This rotate the pawn so that it face the way it is moving.
 */
void ASDTAIController::CalculatePawnOrientation() const
{
	FVector lookAt = GetPawn()->GetActorForwardVector();
	float angle = FMath::RadiansToDegrees(FMath::Acos(FVector::DotProduct(lookAt, mSpeedVector) / mSpeedVector.Size()));
	float orientation = lookAt.X * mSpeedVector.Y - lookAt.Y * mSpeedVector.X;

	if (orientation > 0.0f)
		orientation = 1.0f;
	else
		orientation = -1.0f;

	if (angle == 180.0f)
		GetPawn()->AddActorWorldRotation(FRotator(0.0f, angle, 0.0f));
	else if (angle > 0.0f)
		GetPawn()->AddActorWorldRotation(FRotator(0.0f, orientation * angle, 0.0f));
}


/**
* Analyse the environment.
*
* This detect the elements around the agent that matters to his behaviour.
*/
void ASDTAIController::EnvironmentAnalysis()
{
	const FVector pos = GetPawn()->GetActorLocation();
	const TArray<FOverlapResult> detectedPlayers = SphereOverLap(_character->PlayerSphereRadius, pos, COLLISION_PLAYER);
	const TArray<FOverlapResult> detectedCollectables = SphereOverLap(_character->PickupSphereRadius, pos + GetPawn()->GetActorForwardVector() * _character->PickupSphereRadius, COLLISION_COLLECTIBLE);

	mRadar = new Radar();
	mRadar->detectedPlayer = GetDetectedPlayer(detectedPlayers);
	mRadar->obstacles = GetDetectedObstacles();
	mRadar->targetCollectible = GetDetectedCollectible(detectedCollectables);
}

/**
* Draw debug elements.
*
* Draw lines between detected elements and the agent.
*/
void ASDTAIController::DebugMovement() const
{
	DebugLine();
	if (mRadar->detectedPlayer != nullptr)
	{
		FVector const pos = mRadar->detectedPlayer->posVector;
		FColor pColor = (SDTUtils::IsPlayerPoweredUp(GetWorld())) ? FColor::Red : FColor::Black;
		if(!SweepSingleObstacle(GetPawn()->GetActorLocation(), mRadar->detectedPlayer->posVector))
		{
			DrawLine(pos, pColor);
		};
	}	
	if (mRadar->targetCollectible != nullptr)
	{
        FVector const pos = mRadar->targetCollectible->posVector;
        DrawLine(pos, FColor::Yellow);
	}
}


/**
 * Choose Direction To Take
 *
 * This Choose the correct direction to take depending on the chasing state of the AI.
 * It tries to get away from the player character if the mState is "Fleeing".
 * If not, it chooses a random direction depending on his options.
 *
 */
void ASDTAIController::ChooseDirectionToTake()
{
	(mState == Fleeing) ? ChooseFleeingDirection() : ChoseRandomDirection();
}

/**
 * Choose Random Direction
 *
 * This chooses a random direction based on the available options.
 *
 */
void ASDTAIController::ChoseRandomDirection()
{
	Obstacles* obs = mRadar->obstacles;
	if (obs->bIsForward)
	{
		if (!obs->bIsRight && !obs->bIsLeft)
		{
			if (FMath::RandBool())
				obs->bIsRight = true;
			else
				obs->bIsLeft = true;
		}
	}
	else
	{
		if (!obs->bIsRight && !obs->bIsLeft)
		{
			const int random = FMath::RandRange(1, 3);
			if (random == 1)
			{
				obs->bIsRight = true;
				obs->bIsLeft = true;
			}
			else if (random == 2)
			{
				obs->bIsLeft = true;
			}
			else if (random == 3)
			{
				obs->bIsRight = true;
			}
		}
		else
		{
			if (FMath::RandBool())
			{
				obs->bIsRight = true;
				obs->bIsLeft = true;
			}
		}
	}
}

/**
 * Choose Fleeing Direction
 *
 * This chooses a fleeing direction based on the available options.
 *
 */
void ASDTAIController::ChooseFleeingDirection()
{		
	const FVector forwardVect = GetPawn()->GetActorForwardVector();

	//the 4 orthogonal possible direction
	TArray<FVector> options;
	options.Push(FVector(1.0f, 0.0f, 0.0f));
	options.Push(FVector(0.0f, 1.0f, 0.0f));
	options.Push(FVector(-1.0f, 0.0f, 0.0f));
	options.Push(FVector(0.0f, -1.0f, 0.0f));
	
	//find which one of these are the best fleeing direction and the closest to the forward vector
	FVector pDirection = mRadar->detectedPlayer->direction;
	pDirection.Normalize();
	int fleeValIndex = 0;
	float fleeValue = 1.1f;
	int forwardIndex = 0;
	float forwardValue = -1.1f;
	for(int i = 0 ; i < options.Num(); i++)
	{
		const float value = FVector::DotProduct(pDirection, options[i]);
		if(value < fleeValue)
		{
			fleeValue = value;
			fleeValIndex = i;
		}	

		const float value2 = FVector::DotProduct(forwardVect, options[i]);
		if (value2 > forwardValue)
		{
			forwardValue = value2;
			forwardIndex = i;
		}
	}
	
	//find the correct index for the other directions
	int rightIndex = forwardIndex + 1;
	if (rightIndex == options.Num())
		rightIndex = 0;

	int leftIndex = forwardIndex - 1;
	if (leftIndex == -1)
		leftIndex = 3;

	int deathIndex = fleeValIndex + 2;
	if (deathIndex >= options.Num())
		deathIndex -= options.Num();

	int backIndex = forwardIndex + 2;
	if (backIndex >= options.Num())
		backIndex -= options.Num();

	if (deathIndex == forwardIndex) //should do a 180
	{
		mRadar->obstacles->bIsForward = true;
		mRadar->obstacles->bIsRight = true;
		mRadar->obstacles->bIsLeft = true;
	}
	else
	{
		//search for obstacles orthogonally
		mRadar->obstacles->bIsForward = false;
		mRadar->obstacles->bIsRight = false;
		mRadar->obstacles->bIsLeft = false;
		const FVector wForwardVector = options[forwardIndex] * _character->ForwardDetectionDistance;
		const FVector forwardVector = _actorLocation + wForwardVector;
		if (SweepSingleObstacle(_actorLocation, forwardVector))
		{
			mRadar->obstacles->bIsForward = true;
		}

		const FVector wRightVector = options[rightIndex] * _character->SideDetectionDistance;
		const FVector rightVector = _actorLocation + wRightVector;
		if (SweepSingleObstacle(_actorLocation, rightVector))
		{
			mRadar->obstacles->bIsRight = true;
		}

		const FVector leftVector = _actorLocation - wRightVector;
		if (SweepSingleObstacle(_actorLocation, leftVector))
		{
			mRadar->obstacles->bIsLeft = true;
		}

		if (deathIndex == backIndex && !mRadar->obstacles->bIsForward) //should keep going forward
		{
			mRadar->obstacles->bIsForward = false;
			mRadar->obstacles->bIsRight = true;
			mRadar->obstacles->bIsLeft = true;
		}
		else if (mRadar->obstacles->bIsForward) //is facing an obstacle, should turn
		{
			if (deathIndex == rightIndex)
				mRadar->obstacles->bIsRight = true;
			if (deathIndex == leftIndex)
				mRadar->obstacles->bIsLeft = true;
		}		
	}
}

/**
 * Calculate Turning Direction
 *
 * This chooses the correct direction to turn depending on obstacles.
 *
 */
void ASDTAIController::CalculateTurningDirection()
{
	ChooseDirectionToTake();

	if (mRadar->obstacles->bIsRight && mRadar->obstacles->bIsLeft)
	{		
		if (mRadar->obstacles->bIsForward)
		{
			switch (mDirection)
			{
			case Up:
				mNextDirection = Down;
				break;
			case Down:
				mNextDirection = Up;
				break;
			case Right:
				mNextDirection = Left;
				break;
			case Left:
				mNextDirection = Right;
				break;
			default:
				mNextDirection = Up;
				break;
			}
			mMovement = OneEighty;
		}
	}
	else if (!mRadar->obstacles->bIsRight && mRadar->obstacles->bIsForward)
	{
		switch (mDirection)
		{
		case Up:
			mNextDirection = Right;
			break;
		case Down:
			mNextDirection = Left;
			break;
		case Right:
			mNextDirection = Down;
			break;
		case Left:
			mNextDirection = Up;
			break;
		default:
			mNextDirection = Right;
			break;
		}

		mMovement = Turning;
	}
	else if (!mRadar->obstacles->bIsLeft && mRadar->obstacles->bIsForward)
	{
		switch (mDirection)
		{
		case Up:
			mNextDirection = Left;
			break;
		case Down:
			mNextDirection = Right;
			break;
		case Right:
			mNextDirection = Up;
			break;
		case Left:
			mNextDirection = Down;
			break;
		default:
			mNextDirection = Up;
			break;
		}
		mMovement = Turning;
	}
}

/*
 * Displays pawn's directions vector
 */
void ASDTAIController::DebugLine() const
{
	const FVector pos = GetPawn()->GetActorLocation();
	DrawLine(pos + GetPawn()->GetActorForwardVector() * _character->ForwardDetectionDistance, FColor::Red);
	DrawLine( pos + GetPawn()->GetActorRightVector() * _character->SideDetectionDistance, FColor::Green);
	DrawLine(pos - GetPawn()->GetActorRightVector() * _character->SideDetectionDistance, FColor::Blue);
}

/**
 * Determine which pawn's directions are blocked 
 * @return Obstacles
 */
Obstacles* ASDTAIController::GetDetectedObstacles() const
{
	const FVector pawnPosition = GetPawn()->GetActorLocation();

	Obstacles* obstacles = new Obstacles();

	const FVector wForwardVector = GetPawn()->GetActorForwardVector() * _character->ForwardDetectionDistance;
	const FVector forwardVector = pawnPosition + wForwardVector;
	if (SweepSingleObstacle(pawnPosition, forwardVector))
	{
		obstacles->bIsForward = true;
	}

	const FVector wRightVector = GetPawn()->GetActorRightVector() * _character->SideDetectionDistance;
	const FVector rightVector = pawnPosition + wRightVector;
	if (SweepSingleObstacle(pawnPosition, rightVector))
	{
		obstacles->bIsRight = true;
	}

	const FVector leftVector = pawnPosition - wRightVector;
	if (SweepSingleObstacle(pawnPosition, leftVector))
	{
		obstacles->bIsLeft = true;
	}

	return obstacles;
}


/**
 * Draw a line between pawn's position and the defined target
 * @param Vector 
 * @param Color 
 */
void ASDTAIController::DrawLine(const FVector& Vector, const FColor& Color) const
{
	DrawDebugLine(GetWorld(), GetPawn()->GetActorLocation(),
		Vector, Color, false,
		-1.0f, 000, 5.0f);
}

/*
 * Determine if there is an obstacle between two vectors
 */
bool ASDTAIController::SweepSingleObstacle(FVector Start, FVector End) const
{
	FHitResult firstHit;
	UCapsuleComponent* capsule = GetCharacter()->GetCapsuleComponent();
	FCollisionShape shape = FCollisionShape::MakeCapsule(capsule->GetScaledCapsuleRadius(), capsule->GetScaledCapsuleHalfHeight());
	FCollisionObjectQueryParams objectQueryParams;
	objectQueryParams.AddObjectTypesToQuery(ECollisionChannel::ECC_WorldStatic);
	objectQueryParams.AddObjectTypesToQuery(COLLISION_DEATH_OBJECT); //DeathObject
	FCollisionQueryParams queryParams = FCollisionQueryParams();
	FQuat quat = FRotator(0.0f, 0.0f, 0.0f).Quaternion();
	bool detected = GetWorld()->SweepSingleByObjectType(firstHit, Start, End, quat, objectQueryParams, shape, queryParams);
	return detected && firstHit.bBlockingHit;
}

/**
 * 
 * @brief 
 * @param Radius 
 * @param Position 
 * @param CollisionChannel 
 * @return 
 */
TArray<FOverlapResult> ASDTAIController::SphereOverLap(const float Radius, const FVector Position, const ECollisionChannel CollisionChannel) const
{
	TArray<FOverlapResult> results;
	FCollisionObjectQueryParams collisionObjectQueryParams;
	collisionObjectQueryParams.AddObjectTypesToQuery(CollisionChannel);
	const FCollisionQueryParams collisionQueryParams = FCollisionQueryParams::DefaultQueryParam;
	FCollisionShape CollisionShape;
	CollisionShape.SetSphere(Radius);
	GetWorld()->OverlapMultiByObjectType(results, Position, FQuat::Identity, collisionObjectQueryParams, CollisionShape, collisionQueryParams);
	if (_character->Debug)
		DrawDebugSphere(GetWorld(), Position, Radius, 16, FColor::Green);
	return results;
}

/**
* Get the detected player.
*
* If the list of detected player contains at least one, it will return a player struct with information on the detected player.
* 
* @param DetectedPlayers A list of players detected around the agent.
* @return The detected player information
*/
Player* ASDTAIController::GetDetectedPlayer(TArray<FOverlapResult> DetectedPlayers) const
{
	if (DetectedPlayers.Num() == 0) return nullptr;
	Player* detectedPlayer = new Player();
	detectedPlayer->posVector = DetectedPlayers[0].GetActor()->GetActorLocation();
	detectedPlayer->direction = detectedPlayer->posVector - GetPawn()->GetActorLocation();
	detectedPlayer->angle = FMath::RadiansToDegrees(detectedPlayer->direction.HeadingAngle());
	return detectedPlayer;
}

/**
* Get the detected collectible
*
* Will only return information on the closest accessible collectible and will ignore any collectible blocked by a wall or a deathfloor.
* 
* @param DetectedCollectible A list of collectible detected in front of the agent.
* @return The closest accessible collectible information.
*/
Collectible* ASDTAIController::GetDetectedCollectible(TArray<FOverlapResult> DetectedCollectible) const
{
    if (DetectedCollectible.Num() == 0) return nullptr;

    APawn* pawn = GetPawn();

    TArray<AActor*> collectibles;

    for(int i = 0 ; i< DetectedCollectible.Num(); i++)
    {
        if (!SweepSingleObstacle(pawn->GetActorLocation(), DetectedCollectible[i].Actor.Get()->GetActorLocation()))
        {
            ASDTCollectible* collectible = Cast<ASDTCollectible>(DetectedCollectible[i].Actor.Get());

            if (!collectible->IsOnCooldown())
            {
                collectibles.Add(DetectedCollectible[i].Actor.Get());
            }
        };
    }

    Collectible* targetCollectible = new Collectible();
    AActor* closestCollectible = SDTUtils::IsClosestActor(collectibles, pawn->GetActorLocation());

    if (closestCollectible == nullptr) return nullptr;

    targetCollectible->posVector = closestCollectible->GetActorLocation();
    const FVector direction = targetCollectible->posVector - pawn->GetActorLocation();
    targetCollectible->angle = FMath::RadiansToDegrees(direction.HeadingAngle());
    DebugLog(FString::SanitizeFloat(targetCollectible->angle));

    return targetCollectible;
}

/**
* Displays information.
*	
* Display the number of collectible pickedup by agents and the number of death
*/
void ASDTAIController::DislayInfo() const
{
    DebugLog("SIMULATION MODE ENABLED");

    float timeInSecond = UGameplayStatics::GetRealTimeSeconds(GetWorld());
    const FString time = " Time : " + FString::SanitizeFloat(roundf(timeInSecond * 100) / 100.0);

    const FString pickups = "Pick Ups : " + FString::FromInt(ASoftDesignTrainingCharacter::mPickupCount);
    const FString kills = "AI Killed : " + FString::FromInt(ASoftDesignTrainingCharacter::mKillsCount);

    const FString info = pickups + "    " + kills + "    " + time;

    GEngine->AddOnScreenDebugMessage(0, 3.f, FColor::Yellow, info);
}

/**
* Output Debug Log.
*/
void ASDTAIController::DebugLog(FString S) const
{
	UE_LOG(LogTemp, Log, TEXT("%s"), *S);
}
