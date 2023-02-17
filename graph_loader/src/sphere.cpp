/* The C++ code assumes that you're running it within the context of an Unreal Engine 4 game project. 
The SpawnSpheres() function can be called from anywhere in your code to spawn the spheres.*/

#include "CoreMinimal.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/StaticMesh.h"
#include "Editor/EditorLevelLibrary.h"
#include "Editor/EditorAssetLibrary.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Misc/FileHelper.h"

void SpawnSpheres()
{
    FString JsonFilePath = FPaths::ProjectContentDir() + "graphs/path.json";
    FString JsonString;

    // Read the JSON file and load the data
    if (FFileHelper::LoadFileToString(JsonString, *JsonFilePath))
    {
        TSharedPtr<FJsonObject> JsonObject;
        TSharedRef<TJsonReader<TCHAR>> JsonReader = TJsonReaderFactory<TCHAR>::Create(JsonString);

        if (FJsonSerializer::Deserialize(JsonReader, JsonObject))
        {
            // Get a reference to the current level
            UWorld* World = GEditor->GetEditorWorldContext().World();
            ULevel* Level = World->GetCurrentLevel();

            // Define the properties of the sphere
            float SphereRadius = 50.f;

            // Loop through the points and spawn a sphere at each one
            TArray<TSharedPtr<FJsonValue>> Nodes = JsonObject->GetArrayField("nodes");
            for (int32 i = 0; i < Nodes.Num(); ++i)
            {
                FVector Location = FVector(Nodes[i]->AsArray()[0]->AsNumber(),
                                           Nodes[i]->AsArray()[1]->AsNumber(),
                                           Nodes[i]->AsArray()[2]->AsNumber());

                // Create a new sphere actor
                AStaticMeshActor* SphereActor = Cast<AStaticMeshActor>(UEditorLevelLibrary::SpawnActorFromClass(AStaticMeshActor::StaticClass(),
                                                                                                              Location,
                                                                                                              Level->GetActorLocation()));

                // Set the static mesh of the sphere to a built-in sphere mesh
                UStaticMesh* SphereMesh = Cast<UStaticMesh>(UEditorAssetLibrary::LoadAsset("/Engine/BasicShapes/Sphere"));
                SphereActor->GetStaticMeshComponent()->SetStaticMesh(SphereMesh);

                // Set the radius of the sphere
                SphereActor->GetStaticMeshComponent()->SetSphereRadius(SphereRadius);
            }

            // Save the level
            UEditorLevelLibrary::SaveCurrentLevel();
        }
    }
}
