#pragma once

#include "physics_controller.h"

#include "vphysics_interface.h"
#include "tier0/valve_minmax_off.h"

#include "box3d/math_functions.h"

#include <memory>
#include <vector>

class CPhysicsFluidController;

class CBuoyancyController final : public IController
{
public:
	static constexpr int PRIORITY = 1600;

	CBuoyancyController( CPhysicsFluidController *pFluid, CPhysicsObject *pObject );
	~CBuoyancyController() override;

	CPhysicsObject *GetObject() const { return m_pObject; }

	int GetPriority() const override { return PRIORITY; }

	const char *GetName() const override { return "sys:buoyancy"; }

	void Simulate( CPhysicsObject *pObject, float flTimestep ) override;
	void ObjectDestroyed( CPhysicsObject *pObject ) override;

private:
	CPhysicsFluidController *m_pFluid;
	CPhysicsObject *m_pObject;
	std::vector<int> m_Ledges;
	b3Vec3 m_FluidVelocity = b3Vec3_zero;
	float m_flArea = 0.0f;
	float m_flPreviousArea = 0.0f;
};

class CPhysicsFluidController final : public IPhysicsFluidController
{
public:
	CPhysicsFluidController( CPhysicsObject *pFluidObject, const fluidparams_t &params );
	~CPhysicsFluidController() override;

	void SetGameData( void *pGameData ) override { m_pGameData = pGameData; }

	void *GetGameData() const override { return m_pGameData; }

	void GetSurfacePlane( Vector *pNormal, float *pDist ) const override;

	float GetDensity() const override { return m_flDensity; }

	void WakeAllSleepingObjects() override;

	int GetContents() const override { return m_nContents; }

	float GetDamping() const { return m_flDamping; }

	void GetSurface( b3Vec3 &normal, float &flOffset, b3Vec3 &current );

	void ObjectEntered( CPhysicsObject *pObject );
	void ObjectLeft( CPhysicsObject *pObject );
	void RemoveAllObjects();
	void FluidObjectDestroyed();

private:
	CPhysicsObject *m_pFluidObject;
	void *m_pGameData;
	Vector m_LocalNormal;
	float m_flLocalDistance;
	Vector m_LocalCurrent;
	float m_flDensity;
	float m_flDamping;
	int m_nContents;
	uint32_t m_nSurfaceStep = UINT32_MAX;
	b3Vec3 m_SurfaceNormal = b3Vec3_zero;
	float m_flSurfaceOffset = 0.0f;
	b3Vec3 m_SurfaceCurrent = b3Vec3_zero;
	std::vector<std::unique_ptr<CBuoyancyController>> m_Controllers;
};
