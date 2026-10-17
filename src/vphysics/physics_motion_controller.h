#pragma once

#include "physics_controller.h"

#include "vphysics_interface.h"
#include "tier0/valve_minmax_off.h"

#include <vector>

struct vphysics_save_motioncontroller_t;

class CPhysicsMotionController final : public IPhysicsMotionController, public IController
{
public:
	explicit CPhysicsMotionController( IMotionEvent *pHandler );
	~CPhysicsMotionController() override;

	void SetEventHandler( IMotionEvent *handler ) override { m_pHandler = handler; }

	void AttachObject( IPhysicsObject *pObject, bool checkIfAlreadyAttached ) override;
	void DetachObject( IPhysicsObject *pObject ) override;

	int CountObjects() override { return static_cast<int>( m_Objects.size() ); }

	void GetObjects( IPhysicsObject **pObjectList ) override;
	void ClearObjects() override;
	void WakeObjects() override;
	void SetPriority( priority_t priority ) override;

	int GetPriority() const override { return m_nPriority; }

	const char *GetName() const override { return "vphysics:motion"; }

	void Simulate( CPhysicsObject *pObject, float flTimestep ) override;
	void ObjectDestroyed( CPhysicsObject *pObject ) override;

	void WriteRecord( vphysics_save_motioncontroller_t &record ) const;
	void ReadRecord( const vphysics_save_motioncontroller_t &record );

private:
	IMotionEvent *m_pHandler;
	std::vector<CPhysicsObject *> m_Objects;
	int m_nPriority;
};
