#include "AMaterialChangerActor.h"
#include "Engine/World.h"
#include "TimerManager.h"
#include "GameFramework/Actor.h"
#include "EngineUtils.h"
#include "Components/StaticMeshComponent.h"

// Sets default values
AMaterialChangerActor::AMaterialChangerActor()
{
    // Disable Tick if you don't need it
    PrimaryActorTick.bCanEverTick = false;

    // Initialize the material index
    CurrentMaterialIndex = 0;

    // By default, you might want to set some placeholder names
    TargetActorName = TEXT("TargetActorName");       // can override in the Editor
    TargetMeshName  = TEXT("MeshComponentName");     // can override in the Editor
}

// Called when the game starts or when spawned
void AMaterialChangerActor::BeginPlay()
{
    Super::BeginPlay();


    // Start a recurring timer that calls ChangeMaterial() every second
    if (MaterialList.Num() > 0)
    {
        GetWorldTimerManager().SetTimer(
            MaterialChangeTimerHandle,
            this,
            &AMaterialChangerActor::ChangeMaterial,
            1.0f,        // Interval (seconds)
            true         // bLoop
        );
    }
}

// Helper function to locate the desired Actor and Mesh
void AMaterialChangerActor::FindTargetActorAndMesh()
{
    UWorld* World = GetWorld();
    if (!World) return;

    // Iterate over all Actors in the world
    for (TActorIterator<AActor> It(World); It; ++It)
    {
        AActor* FoundActor = *It;
        if (FoundActor && FoundActor->ActorHasTag(FName(TEXT("VodafoneScreen"))))
        {
            // We found the Actor. Now look for the specified Mesh.
            TArray<UStaticMeshComponent*> Components;
            FoundActor->GetComponents<UStaticMeshComponent>(Components);

            for (UStaticMeshComponent* MeshComp : Components)
            {
                if (MeshComp && MeshComp->GetName().Equals(TargetMeshName, ESearchCase::IgnoreCase))
                {
                    TargetMesh = MeshComp;
                    return; // Found our mesh, stop searching.
                }
            }
        }
    }
}

// Every second, this cycles to the next Material
void AMaterialChangerActor::ChangeMaterial()
{

    // Find the Actor and Mesh by name
    FindTargetActorAndMesh();
    if (!TargetMesh || MaterialList.Num() == 0)
        return;

    // Move to the next material index (wrap using modulo)
    CurrentMaterialIndex = (CurrentMaterialIndex + 1) % MaterialList.Num();

    // Apply the material to the 0th Material slot (or whichever slot you want to change)
    TargetMesh->SetMaterial(0, MaterialList[CurrentMaterialIndex]);
}
