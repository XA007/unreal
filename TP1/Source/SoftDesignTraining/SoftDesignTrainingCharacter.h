// Copyright 1998-2015 Epic Games, Inc. All Rights Reserved.
#pragma once
#include "GameFramework/Character.h"
#include "SoftDesignTrainingCharacter.generated.h"


UCLASS()
class ASoftDesignTrainingCharacter : public ACharacter
{
    GENERATED_BODY()

public:
    ASoftDesignTrainingCharacter();

    virtual void BeginPlay() override;
    virtual void OnCollectPowerUp();	

	static int mPickupCount;
	static int mKillsCount;

protected:
    UFUNCTION()
    virtual void OnBeginOverlap(UPrimitiveComponent* OverlappedComponent, AActor* OtherActor, UPrimitiveComponent* OtherComponent, int32 OtherBodyIndex, bool bFromSweep, const FHitResult& SweepResult);

    FVector m_StartingPosition;

public:
	/** Max speed to reach when the AI character is doing a 180 degrees rotation. */
	UPROPERTY(EditAnywhere, Category = "TP1 Parameters")
	bool Debug = false;

	/** Rate of character rotation in degrees per second. */
	UPROPERTY(EditAnywhere, Category = "TP1 Parameters", meta = (ClampMin = "0.0", ClampMax = "720.0"))
	float RotationRate = 360.0f;

	/** Obstacle detection distance in the forward direction of the character. */
	UPROPERTY(EditAnywhere, Category = "TP1 Parameters", meta = (ClampMin = "0.0", ClampMax = "250.0"))
	float ForwardDetectionDistance = 140.0f;

	/** Obstacle detection distance on the left and right sides of the character. */
	UPROPERTY(EditAnywhere, Category = "TP1 Parameters", meta = (ClampMin = "0.0", ClampMax = "500.0"))
	float SideDetectionDistance = 300.0f;

	/** Pickup detection sphere radius. 
	* The sphere is centered in front of the character at a distance equal to the radius. */
	UPROPERTY(EditAnywhere, Category = "TP1 Parameters", meta = (ClampMin = "0.0", ClampMax = "1000.0"))
	float PickupSphereRadius = 400.0f;

	/** Player detection sphere radius. 
	* The sphere is centered on the AI character. */
	UPROPERTY(EditAnywhere, Category = "TP1 Parameters", meta = (ClampMin = "0.0", ClampMax = "1000.0"))
	float PlayerSphereRadius = 500.0f;

	/** Acceleration when the AI character goes in straight line. */
	UPROPERTY(EditAnywhere, Category = "TP1 Parameters", meta = (ClampMin = "0.0", ClampMax = "10000.0"))
	float StraightLineAcceleration = 600.0f;

	/** Max speed to reach when the AI character goes in straight line. */
	UPROPERTY(EditAnywhere, Category = "TP1 Parameters", meta = (ClampMin = "0.0", ClampMax = "1000.0"))
	float StraightLineMaxSpeed = 600.0f;

	/** Deceleration when the AI character is changing direction. */
	UPROPERTY(EditAnywhere, Category = "TP1 Parameters", meta = (ClampMin = "0.0", ClampMax = "10000.0"))
	float TurningSpeedDeceleration = 6000.0f;

	/** Max speed to reach when the AI character is changing direction. */
	UPROPERTY(EditAnywhere, Category = "TP1 Parameters", meta = (ClampMin = "0.0", ClampMax = "1000.0"))
	float TurningMaxSpeed = 300.0f;

	/** Deceleration when the AI character is doing a 180 degrees rotation. */
	UPROPERTY(EditAnywhere, Category = "TP1 Parameters", meta = (ClampMin = "0.0", ClampMax = "100000.0"))
	float OneEightySpeedDeceleration = 60000.0f;

	/** Max speed to reach when the AI character is doing a 180 degrees rotation. */
	UPROPERTY(EditAnywhere, Category = "TP1 Parameters", meta = (ClampMin = "0.0", ClampMax = "1000.0"))
	float OneEightyMaxSpeed = 0.0f;
};

