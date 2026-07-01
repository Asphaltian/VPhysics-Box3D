//=================================================================================================
//
// Shadow + motion controllers: how the game drives held/animated physics objects.
// Shadow controller  -> +use pickup, physgun hold, doors/elevators (kinematic follow-to-target).
// Motion controller  -> gravity gun grab (game applies force each step via an IMotionEvent).
//
//=================================================================================================

#pragma once

#include "vbox_interface.h"

class Box3DPhysicsObject;

class Box3DPhysicsShadowController final : public IPhysicsShadowController
{
public:
	Box3DPhysicsShadowController( Box3DPhysicsObject *pObject, bool allowTranslation, bool allowRotation );
	~Box3DPhysicsShadowController() override;

	void	Update( const Vector &position, const QAngle &angles, float timeOffset ) override;
	void	MaxSpeed( float maxSpeed, float maxAngularSpeed ) override;
	void	StepUp( float height ) override;
	void	SetTeleportDistance( float teleportDistance ) override;
	bool	AllowsTranslation() override;
	bool	AllowsRotation() override;
	void	SetPhysicallyControlled( bool isPhysicallyControlled ) override;
	bool	IsPhysicallyControlled() override;
	void	GetLastImpulse( Vector *pOut ) override;
	void	UseShadowMaterial( bool bUseShadowMaterial ) override;
	void	ObjectMaterialChanged( int materialIndex ) override;
	float	GetTargetPosition( Vector *pPositionOut, QAngle *pAnglesOut ) override;
	float	GetTeleportDistance() override;
	void	GetMaxSpeed( float *pMaxSpeedOut, float *pMaxAngularSpeedOut ) override;

	// Ticked by the environment before each simulation step.
	void	OnPreSimulate( float flDeltaTime );
	Box3DPhysicsObject *GetObject() const { return m_pObject; }

private:
	Box3DPhysicsObject *m_pObject;

	Vector	m_targetPosition = vec3_origin;
	QAngle	m_targetAngles = vec3_angle;
	float	m_secondsToArrival = 0.0f;
	float	m_maxSpeed = 0.0f;
	float	m_maxAngular = 0.0f;
	float	m_teleportDistance = 0.0f;

	unsigned short m_savedCallbackFlags = 0;
	bool	m_allowTranslation = true;
	bool	m_allowRotation = true;
	bool	m_isPhysicallyControlled = false;
	bool	m_bWasStatic = false;
	bool	m_enabled = false;
};

class Box3DPhysicsMotionController final : public IPhysicsMotionController
{
public:
	explicit Box3DPhysicsMotionController( IMotionEvent *pHandler );

	void	SetEventHandler( IMotionEvent *pHandler ) override;
	void	AttachObject( IPhysicsObject *pObject, bool checkIfAlreadyAttached ) override;
	void	DetachObject( IPhysicsObject *pObject ) override;
	int		CountObjects() override;
	void	GetObjects( IPhysicsObject **pObjectList ) override;
	void	ClearObjects() override;
	void	WakeObjects() override;
	void	SetPriority( priority_t priority ) override;

	// Ticked by the environment before each simulation step.
	void	OnPreSimulate( float flDeltaTime );

private:
	IMotionEvent *m_pHandler;
	CUtlVector< Box3DPhysicsObject * > m_Objects;
};
