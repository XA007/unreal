// Copyright 1998-2015 Epic Games, Inc. All Rights Reserved.

#include "SoftDesignTraining.h"
#include "SoftDesignTrainingCharacter.h"
#include "SoftDesignTrainingMainCharacter.h"
#include "SDTUtils.h"
#include "DrawDebugHelpers.h"
#include "SDTCollectible.h"


int ASoftDesignTrainingCharacter::mPickupCount = 0;
int ASoftDesignTrainingCharacter::mKillsCount = 0;

ASoftDesignTrainingCharacter::ASoftDesignTrainingCharacter()
{
    GetCapsuleComponent()->InitCapsuleSize(42.f, 96.0f);
}

void ASoftDesignTrainingCharacter::BeginPlay()
{
    Super::BeginPlay();

    mPickupCount = 0;
	mKillsCount = 0;

    GetCapsuleComponent()->OnComponentBeginOverlap.AddDynamic(this, &ASoftDesignTrainingCharacter::OnBeginOverlap);
    m_StartingPosition = GetActorLocation();
}

void ASoftDesignTrainingCharacter::OnBeginOverlap(UPrimitiveComponent* OverlappedComponent, AActor* OtherActor, UPrimitiveComponent* OtherComponent, int32 OtherBodyIndex, bool bFromSweep, const FHitResult& SweepResult)
{
    if (OtherComponent->GetCollisionObjectType() == COLLISION_DEATH_OBJECT)
    {
        SetActorLocation(m_StartingPosition);
		mKillsCount++;
    }
    else if(ASDTCollectible* collectibleActor = Cast<ASDTCollectible>(OtherActor))
    {
        if (!collectibleActor->IsOnCooldown())
        {
            OnCollectPowerUp();
        }

        collectibleActor->Collect();
    }
    else if (ASoftDesignTrainingMainCharacter* mainCharacter = Cast<ASoftDesignTrainingMainCharacter>(OtherActor))
    {
        if(mainCharacter->IsPoweredUp())
            SetActorLocation(m_StartingPosition);
    }
}

void ASoftDesignTrainingCharacter::OnCollectPowerUp()
{
	mPickupCount++;
}
