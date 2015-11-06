/*=================================================
* FileName: BuoyantMeshComponent.cpp
*
* Created by: quantumv
* Project name: OceanProject
* Unreal Engine version: 4.9
* Created on: 2015/09/21
*
* Last Edited on: 2015/09/23
* Last Edited by: quantumv
*
* -------------------------------------------------
* For parts referencing UE4 code, the following copyright applies:
* Copyright 1998-2015 Epic Games, Inc. All Rights Reserved.
*
* Feel free to use this software in any commercial/free game.
* Selling this as a plugin/item, in whole or part, is not allowed.
* See "OceanProject\License.md" for full licensing details.
* =================================================*/

#include "OceanPluginPrivatePCH.h"
#include "BuoyantMesh/BuoyantMeshComponent.h"

#include "PhysicsEngine/BodySetup.h"
#include "PhysXPublic.h"

#include "OceanManager.h"
#include "BuoyantMesh/BuoyantMeshTriangle.h"
#include "BuoyantMesh/BuoyantMeshSubtriangle.h"

using FForce = UBuoyantMeshComponent::FForce;
using ForceType = UBuoyantMeshComponent::ForceType;

// Sets default values for this component's properties
UBuoyantMeshComponent::UBuoyantMeshComponent()
{
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
	PrimaryComponentTick.bCanEverTick = true;
	bWantsInitializeComponent = true;
	bAutoActivate = true;
	bVerticalForcesOnly = false;
	UActorComponent::SetComponentTickEnabled(true);
	bUseDynamicForces = true;
	bUseStaticForces = true;
	WaterDensity = 0.001027f;
}

void UBuoyantMeshComponent::InitializeComponent()
{
	Super::InitializeComponent();
}

// Called when the game starts
void UBuoyantMeshComponent::BeginPlay()
{
	Super::BeginPlay();
}

float UBuoyantMeshComponent::GetHeightAboveWater(const UWorld& World, const FVector& Position) const
{
	SCOPE_CYCLE_COUNTER(STAT_GetHeightAboveWater);
	float WaterHeight = 0.f;
	if (IsValid(OceanManager))
	{
		WaterHeight = OceanManager->GetWaveHeight(Position, &World);
	}
	return Position.Z - WaterHeight;
}

UPrimitiveComponent* UBuoyantMeshComponent::GetParentPrimitive() const
{
	if (IsValid(AttachParent))
	{
		const auto PrimitiveComponent = Cast<UPrimitiveComponent>(AttachParent);
		if (PrimitiveComponent)
		{
			return PrimitiveComponent;
		}
	}
	return nullptr;
}

UPrimitiveComponent* UBuoyantMeshComponent::GetRootPrimitive() const
{
	// Use the root component if it's a UPrimitiveComponent
	const auto OwnerActor = GetOwner();
	if (IsValid(OwnerActor))
	{
		const auto RootComponent = OwnerActor->GetRootComponent();
		if (IsValid(RootComponent))
		{
			const auto RootPrimitive = Cast<UPrimitiveComponent>(RootComponent);
			if (RootPrimitive != nullptr)
			{
				return RootPrimitive;
			}
		}
	}
	return nullptr;
}

void UBuoyantMeshComponent::Initialize()
{
	if (UpdatedComponent == nullptr)
	{
		const auto ParentPrimitive = GetParentPrimitive();
		UpdatedComponent = ParentPrimitive ? ParentPrimitive : this;
	}

	// This component needs to tick before the updated component.
	if (UpdatedComponent != this)
	{
		UpdatedComponent->PrimaryComponentTick.AddPrerequisite(this, PrimaryComponentTick);
	}

	if (!OceanManager)
	{
		for (auto Actor : TActorRange<AOceanManager>(GetWorld()))
		{
			OceanManager = Actor;
			break;
		}
	}
}

template <class T>
void UBuoyantMeshComponent::GetTriangleVertexIndices(const TArray<FVector>& WorldVertexPositions,
                                                     const T* const VertexIndices,
                                                     const PxU32 TriangleIndex,
                                                     int32* OutIndex1,
                                                     int32* OutIndex2,
                                                     int32* OutIndex3)
{
	*OutIndex1 = VertexIndices[TriangleIndex * 3 + 0];
	*OutIndex2 = VertexIndices[TriangleIndex * 3 + 1];
	*OutIndex3 = VertexIndices[TriangleIndex * 3 + 2];
	return;
}

void UBuoyantMeshComponent::GetTriangleVertexIndices(const TArray<FVector>& WorldVertexPositions,
                                                     const void* const VertexIndices,
                                                     const PxU32 TriangleIndex,
                                                     const bool b16BitIndices,
                                                     int32* OutIndex1,
                                                     int32* OutIndex2,
                                                     int32* OutIndex3)
{
	if (b16BitIndices)
	{
		const auto Indices = static_cast<const PxU16*>(VertexIndices);
		GetTriangleVertexIndices(WorldVertexPositions,
		                         Indices,
		                         TriangleIndex,
		                         /*out*/ OutIndex1,
		                         /*out*/ OutIndex2,
		                         /*out*/ OutIndex3);
		return;
	}
	else
	{
		const auto Indices = static_cast<const PxU32*>(VertexIndices);
		GetTriangleVertexIndices(WorldVertexPositions,
		                         Indices,
		                         TriangleIndex,
		                         /*out*/ OutIndex1,
		                         /*out*/ OutIndex2,
		                         /*out*/ OutIndex3);
		return;
	}
}

void UBuoyantMeshComponent::GetSubmergedTriangleForces(const UWorld& World,
                                                       TArray<FForce>& InOutForces,
                                                       const float GravityMagnitude,
                                                       const FVector& TriangleNormal,
                                                       const FBuoyantMeshSubtriangle& Subtriangle) const
{
	const auto CenterPosition = Subtriangle.GetCenter();
	const FBuoyantMeshVertex CenterVertex{CenterPosition, GetHeightAboveWater(World, CenterPosition)};
	const auto TriangleArea = Subtriangle.GetArea();
	if (FMath::IsNearlyZero(TriangleArea)) return;

	if (bUseStaticForces)
	{
		const auto StaticForce = FBuoyantMeshSubtriangle::GetHydrostaticForce(
		    WaterDensity, GravityMagnitude, CenterVertex, TriangleNormal, TriangleArea);
		InOutForces.Emplace(StaticForce, CenterPosition);
	}

	if (bUseDynamicForces)
	{
		const auto CenterVelocity = UpdatedComponent->GetBodyInstance()->GetUnrealWorldVelocityAtPoint(CenterPosition);
		const auto DynamicForce = FBuoyantMeshSubtriangle::GetHydrodynamicForce(
		    WaterDensity, CenterPosition, CenterVelocity, TriangleNormal, TriangleArea);
		InOutForces.Emplace(DynamicForce, CenterPosition);
	}
}

void UBuoyantMeshComponent::GetTriangleMeshForces(TArray<FForce>& InOutForces,
                                                  UWorld& World,
                                                  const PxTriangleMesh& TriangleMesh) const
{
	// Get vertices
	const PxVec3* const Vertices = TriangleMesh.getVertices();
	const PxU32 VertexCount = TriangleMesh.getNbVertices();

	const auto LocalToWorld = GetComponentTransform();

	TArray<FVector> WorldVertexPositions{};
	TArray<float> VertexHeights{};
	for (PxU32 i = 0; i < VertexCount; ++i)
	{
		const auto Vertex = LocalToWorld.TransformPosition(P2UVector(Vertices[i]));
		WorldVertexPositions.Add(Vertex);
		VertexHeights.Add(GetHeightAboveWater(World, Vertex));
		if (bDrawVertices)
		{
			DrawDebugPoint(&World, Vertex, 2.f, FColor::White);
		}
	}

	// Get triangles
	const PxU32 TriangleCount = TriangleMesh.getNbTriangles();
	const void* const VertexIndices = TriangleMesh.getTriangles();
	const auto b16BitIndices = TriangleMesh.getTriangleMeshFlags() & PxTriangleMeshFlag::e16_BIT_INDICES;

	const float GravityMagnitude = FMath::Abs(World.GetGravityZ());

	for (PxU32 TriangleIndex = 0; TriangleIndex < TriangleCount; ++TriangleIndex)
	{
		// A, B and C are the vertices of the triangle in clockwise order.
		// The order determines the normal.
		int32 AIndex;
		int32 BIndex;
		int32 CIndex;

		GetTriangleVertexIndices(WorldVertexPositions,
		                         VertexIndices,
		                         TriangleIndex,
		                         b16BitIndices,
		                         /*out*/ &AIndex,
		                         /*out*/ &BIndex,
		                         /*out*/ &CIndex);

		const FBuoyantMeshVertex A{WorldVertexPositions[AIndex], VertexHeights[AIndex]};
		const FBuoyantMeshVertex B{WorldVertexPositions[BIndex], VertexHeights[BIndex]};
		const FBuoyantMeshVertex C{WorldVertexPositions[CIndex], VertexHeights[CIndex]};

		if (bDrawTriangles)
		{
			DrawDebugTriangle(World, A.Position, B.Position, C.Position, FColor::White, 4.f);
		}

		const auto Triangle = FBuoyantMeshTriangle::FromClockwiseVertices(A, B, C);

		const auto SubTriangles = Triangle.GetSubmergedPortion(&World, bDrawWaterline);

		for (const auto& SubTriangle : SubTriangles)
		{
			if (bDrawSubtriangles)
			{
				DrawDebugTriangle(World, SubTriangle.A, SubTriangle.B, SubTriangle.C, FColor::Yellow, 6.f);
			}
			SCOPE_CYCLE_COUNTER(STAT_GetHydrostaticForces);
			GetSubmergedTriangleForces(World, /*inout*/ InOutForces, GravityMagnitude, Triangle.Normal, SubTriangle);
		}
	}
}

void UBuoyantMeshComponent::DrawDebugTriangle(const UWorld& World,
                                              const FVector& A,
                                              const FVector& B,
                                              const FVector& C,
                                              const FColor& Color,
                                              const float Thickness)
{
	DrawDebugLine(&World, A, B, Color, false, -1.f, 0, Thickness);
	DrawDebugLine(&World, B, C, Color, false, -1.f, 0, Thickness);
	DrawDebugLine(&World, C, A, Color, false, -1.f, 0, Thickness);
}

void UBuoyantMeshComponent::GetStaticMeshForces(TArray<FForce>& InOutForces,
                                                UWorld& World,
                                                const UBodySetup& BodySetup) const
{
	for (const auto TriangleMesh : BodySetup.TriMeshes)
	{
		if (TriangleMesh != nullptr)
		{
			GetTriangleMeshForces(/*inout*/ InOutForces, World, *TriangleMesh);
		}
	}
}

void UBuoyantMeshComponent::ApplyMeshForce(UWorld& World, UPrimitiveComponent& Component, const FForce& Force)
{
	const auto ForceVector = bVerticalForcesOnly ? FVector{0.f, 0.f, Force.Vector.Z} : Force.Vector;
	const auto bIsValidForce = !ForceVector.IsNearlyZero() && !ForceVector.ContainsNaN();
	if (!bIsValidForce) return;

	Component.AddForceAtLocation(ForceVector, Force.Point);
	if (bDrawForceArrows)
	{
		DrawDebugLine(&World, Force.Point - (ForceVector * ForceArrowSize * 0.0001f), Force.Point, FColor::Blue);
	}
}

void UBuoyantMeshComponent::ApplyMeshForces()
{
	SCOPE_CYCLE_COUNTER(STAT_ApplyHydrostaticForces);
	const auto World = GetWorld();
	if (!IsValid(World)) return;

	const auto BodySetup = GetBodySetup();
	if (!IsValid(BodySetup))
	{
		UE_LOG(LogTemp, Error, TEXT("BuoyantMeshComponent has a missing or invalid mesh."));
		return;
	}

	if (!IsValid(UpdatedComponent) || !UpdatedComponent->IsSimulatingPhysics())
	{
		UE_LOG(LogTemp,
		       Error,
		       TEXT("BuoyantMeshComponent has no updated component set up. Use a ")
		           TEXT("parent component with \"Simulate Physics\" turned on."));
		return;
	}

	TArray<FForce> Forces;
	GetStaticMeshForces(Forces, *World, *BodySetup);

	for (const auto& Force : Forces)
	{
		ApplyMeshForce(*World, *UpdatedComponent, Force);
	}
}

// Called every frame
void UBuoyantMeshComponent::TickComponent(float DeltaTime,
                                          ELevelTick TickType,
                                          FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (bNeedsInitialization)
	{
		bNeedsInitialization = false;
		Initialize();
	}

	ApplyMeshForces();
}