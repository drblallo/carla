#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

#include "AMaterialChangerActor.generated.h"

UCLASS()
class CARLA_API AMaterialChangerActor : public AActor {
  GENERATED_BODY()

public:
  AMaterialChangerActor();

  // Called every second by the Timer to change the material
  void ChangeMaterial(size_t newMaterialIndex = -1);
protected:
  // Called when the game starts or when spawned
  virtual void BeginPlay() override;
  virtual void BeginDestroy() override;

private:
  // Timer handle to repeatedly call the material change
  FTimerHandle MaterialChangeTimerHandle;

  // **Exposed** array of Materials to cycle through. Now settable in the
  // Editor.
  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MaterialChanger",
            meta = (AllowPrivateAccess = "true"))
  TArray<UMaterialInterface *> MaterialList;

  // **Exposed** name of the Actor in the level (to look for). Settable in the
  // Editor.
  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MaterialChanger",
            meta = (AllowPrivateAccess = "true"))
  FString TargetActorName;

  // **Exposed** name of the Mesh component (inside that Actor). Settable in the
  // Editor.
  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MaterialChanger",
            meta = (AllowPrivateAccess = "true"))
  FString TargetMeshName;

  // Pointer to the Mesh that we want to change the material on
  UPROPERTY()
  UMeshComponent *TargetMesh;

  // Used to keep track of which Material we're on
  int32 CurrentMaterialIndex;


  // Helper function to locate the desired Actor and Mesh
  void FindTargetActorAndMesh();
};

void setUserActorDisplayedSignal(size_t newMaterialIndex = -1);
