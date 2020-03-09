// Fill out your copyright notice in the Description page of Project Settings.

#include "SoftDesignTraining.h"
#include "SDTUtils.h"
#include "SoftDesignTrainingMainCharacter.h"
#include "DrawDebugHelpers.h"
#include "Engine/World.h"

/*static*/ bool SDTUtils::Raycast(UWorld* uWorld, FVector sourcePoint, FVector targetPoint)
{
    FHitResult hitData;
    FCollisionQueryParams TraceParams(FName(TEXT("VictoreCore Trace")), true);
    
    // Fake cost for the exercise
    //Sleep(1);
    // End fake cost

    return uWorld->LineTraceSingleByChannel(hitData, sourcePoint, targetPoint, ECC_Pawn, TraceParams);
}

bool SDTUtils::IsPlayerPoweredUp(UWorld * uWorld)
{
    ACharacter* playerCharacter = UGameplayStatics::GetPlayerCharacter(uWorld, 0);
    if (!playerCharacter)
        return false;

    ASoftDesignTrainingMainCharacter* castedPlayerCharacter = Cast<ASoftDesignTrainingMainCharacter>(playerCharacter);
    if (!castedPlayerCharacter)
        return false;

    return castedPlayerCharacter->IsPoweredUp();
}

AActor* SDTUtils::IsClosestActor(TArray<AActor*> actors, FVector pos)
{
    if (actors.Num() == 0) return nullptr;

    AActor* closestActor = actors[0];
    float minDistance = (closestActor->GetActorLocation() - pos).Size();

    for (int i = 1; i < actors.Num(); i++)
    {
        float newDistance = (actors[i]->GetActorLocation() - pos).Size();

        if (minDistance > newDistance)
        {
            closestActor = actors[i];
            minDistance = newDistance;
        }
    }

    return closestActor;
}