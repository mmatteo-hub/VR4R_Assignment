#include "CoreMinimal.h"
#include "Engine/StaticMeshActor.h"
#include "EditorLevelUtils.h"
#include "EditorAssetLibrary.h"
#include "Paths.h"
#include "JsonUtilities.h"

// Load and parse the JSON data
void LoadJSONData(FString FilePath, TSharedPtr<FJsonObject>& JsonObject)
{
    FString JSONData;
    FFileHelper::LoadFileToString(JSONData, *FilePath);

    TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(JSONData);
    FJsonSerializer::Deserialize(Reader, JsonObject);
}

void LoadSpheres()
{
    // Connect to the Unreal Editor
    UEditorLevelUtils::GetCurrentLevel();

    // Load the JSON file
    FString FilePath = FPaths::ProjectContentDir() + "graphs/path.json";
    TSharedPtr<FJsonObject> JsonObject;
    LoadJSONData(FilePath, JsonObject);

    // Get a reference to the current level
    ULevel* Level = UEditorLevelUtils::GetCurrentLevel();
    FVector LevelLocation = Level->GetActorLocation();

    // Define the properties of the sphere
    float SphereRadius = 50.0f;

    // Loop through the points and spawn a sphere at each one
    TArray<TSharedPtr<FJsonValue>> Nodes = JsonObject->GetArrayField("nodes");
    for (int32 i = 0; i < Nodes.Num(); i++)
    {
        // Create a new sphere actor
        AStaticMeshActor* SphereActor = Level->SpawnActor<AStaticMeshActor>(
            AStaticMeshActor::StaticClass(),
            FVector(Nodes[i]->AsObject()->GetNumberField("x"), Nodes[i]->AsObject()->GetNumberField("y"), Nodes[i]->AsObject()->GetNumberField("z")) + LevelLocation,
            FRotator::ZeroRotator
        );

        // Set the static mesh of the sphere to a built-in sphere mesh
        UStaticMesh* SphereMesh = Cast<UStaticMesh>(UEditorAssetLibrary::LoadAsset("/Engine/BasicShapes/Sphere"));
        SphereActor->GetStaticMeshComponent()->SetStaticMesh(SphereMesh);

        // Set the radius of the sphere
        SphereActor->GetStaticMeshComponent()->SetSphereRadius(SphereRadius);
    }

    // Save the level
    UEditorLevelUtils::SaveCurrentLevel();
}
