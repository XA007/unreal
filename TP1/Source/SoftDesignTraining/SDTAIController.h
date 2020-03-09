// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "AIController.h"
#include "DrawDebugHelpers.h"
#include "SDTUtils.h"
#include "SoftDesignTrainingCharacter.h"
#include "SDTCollectible.h"
#include "SDTAIController.generated.h"

struct Player {
	FVector posVector;
	float angle;
	FVector direction;
};

struct Obstacles {
	bool bIsForward = false;
	bool bIsRight = false;
	bool bIsLeft = false;
};

struct Collectible {
    FVector posVector;
    float angle;
    bool bIsOnCooldown = false;
};

struct Radar {
	Player* detectedPlayer;
	Obstacles* obstacles;
    Collectible* targetCollectible;
};

enum Movement {
	Turning,
	AfterTurning,
	OneEighty,
	Straigth
};

enum Direction
{
	Up,
	Down,
	Left,
	Right,
};

enum States {
	Free,
	Chasing,
	Fleeing,
	PickUp,
};

/**
 * 
 */
UCLASS(ClassGroup = AI, config = Game)
class SOFTDESIGNTRAINING_API ASDTAIController : public AAIController
{
    GENERATED_BODY()
public:
    virtual void Tick(float DeltaTime) override;
    void DefineState();
    void DislayInfo() const;

private:
	FVector _actorLocation;
	ASoftDesignTrainingCharacter* _character;
	FVector mSpeedVector = FVector(1.0f, 0.0f, 0.0f);
	Radar* mRadar;
	Movement mMovement = Movement::Straigth;
	bool bIsChasing = true;
	States mState = States::Free;
	float mSpeed = 0.0f;
	Direction mDirection = Direction::Up;
	Direction mNextDirection;

	void Perform(float deltaTime);
	void EnvironmentAnalysis();
	void RandomMove(float deltaTime);
    void MoveToTarget(FVector targetPos, float deltaTime);
    bool CalculateSpeedVectorRotation(float angleToReach, float deltaTime);
    float GetTurningAngle() const;
    void CalculatePawnOrientation() const;
	void Move(float deltaTime, float acceleration, float maxSpeed);
	bool SweepSingleObstacle(FVector Start, FVector End) const;
	TArray<FOverlapResult> SphereOverLap(float Radius, FVector Position, ECollisionChannel CollisionChannel) const;
	Player * GetDetectedPlayer(TArray<FOverlapResult> DetectedPlayers) const;
    Collectible * GetDetectedCollectible(TArray<FOverlapResult> DetectedCollectible) const;

    Obstacles* GetDetectedObstacles() const;

    void DrawLine(const FVector& Vector, const FColor& Red) const;
	void DebugMovement() const;
    void ChoseRandomDirection();
    void ChooseFleeingDirection();
    void ChooseDirectionToTake();
    void CalculateTurningDirection();
    void DebugLine() const;
    void DebugLog(FString S) const;
 };
