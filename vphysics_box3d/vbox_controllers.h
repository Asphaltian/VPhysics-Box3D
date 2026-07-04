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
	Vector	m_lastPosition = vec3_origin;
	Vector	m_lastImpulse = vec3_origin;
	float	m_secondsToArrival = 0.0f;
	float	m_maxSpeed = 0.0f;
	float	m_maxDampSpeed = 0.0f;
	float	m_maxAngular = 0.0f;
	float	m_maxDampAngular = 0.0f;
	float	m_teleportDistance = 0.0f;
	float	m_dampFactor = 1.0f;
	float	m_savedMass = 0.0f;

	unsigned short m_savedCallbackFlags = 0;
	bool	m_allowTranslation = true;
	bool	m_allowRotation = true;
	bool	m_isPhysicallyControlled = false;
	bool	m_bWasStatic = false;
	bool	m_enabled = false;
};

class Box3DPhysicsPlayerController final : public IPhysicsPlayerController
{
public:
	explicit Box3DPhysicsPlayerController( Box3DPhysicsObject *pObject );
	~Box3DPhysicsPlayerController() override;

	void	Update( const Vector &position, const Vector &velocity, float secondsToArrival, bool onground, IPhysicsObject *ground ) override;
	void	SetEventHandler( IPhysicsPlayerControllerEvent *handler ) override;
	bool	IsInContact() override;
	void	MaxSpeed( const Vector &maxVelocity ) override;
	void	SetObject( IPhysicsObject *pObject ) override;
	int		GetShadowPosition( Vector *position, QAngle *angles ) override;
	void	StepUp( float height ) override;
	void	Jump() override;
	void	GetShadowVelocity( Vector *velocity ) override;
	IPhysicsObject *GetObject() override;
	void	GetLastImpulse( Vector *pOut ) override;
	void	SetPushMassLimit( float maxPushMass ) override;
	void	SetPushSpeedLimit( float maxPushSpeed ) override;
	float	GetPushMassLimit() override;
	float	GetPushSpeedLimit() override;
	bool	WasFrozen() override;

	// Ticked by the environment before each simulation step.
	void	OnPreSimulate( float flDeltaTime );
	void	ClearGround( Box3DPhysicsObject *pObject );
	Box3DPhysicsObject *GetControlledObject() const { return m_pObject; }

private:
	void	SetObjectInternal( Box3DPhysicsObject *pObject );
	void	SetGround( Box3DPhysicsObject *pGround );
	int		TryTeleportObject();

	Box3DPhysicsObject *m_pObject = nullptr;
	Box3DPhysicsObject *m_pGround = nullptr;
	IPhysicsPlayerControllerEvent *m_pHandler = nullptr;

	Vector	m_targetPosition = vec3_origin;
	Vector	m_groundPosition = vec3_origin;
	Vector	m_maxSpeed = vec3_origin;
	Vector	m_currentSpeed = vec3_origin;
	Vector	m_lastImpulse = vec3_origin;
	float	m_secondsToArrival = 0.0f;
	float	m_maxDeltaPosition = 24.0f;
	float	m_dampFactor = 1.0f;
	float	m_pushableMassLimit = VPHYSICS_MAX_MASS;
	float	m_pushableSpeedLimit = 1e4f;
	float	m_flSavedAngularDamping = 0.0f;
	bool	m_enable = false;
	bool	m_updatedSinceLast = false;
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

// Water buoyancy, drag and current, applied per step to bodies overlapping the fluid volume.
class Box3DPhysicsFluidController final : public IPhysicsFluidController
{
public:
	Box3DPhysicsFluidController( Box3DPhysicsObject *pFluidObject, const fluidparams_t *pParams );
	~Box3DPhysicsFluidController() override;

	void	SetGameData( void *pGameData ) override;
	void *	GetGameData() const override;
	void	GetSurfacePlane( Vector *pNormal, float *pDist ) const override;
	float	GetDensity() const override;
	void	WakeAllSleepingObjects() override;
	int		GetContents() const override;

	// Ticked by the environment before each simulation step.
	void	OnPreSimulate( float flDeltaTime );
	// A tracked object was destroyed.
	void	DetachObject( Box3DPhysicsObject *pObject );
	Box3DPhysicsObject *GetFluidObject() const { return m_pFluidObject; }

private:
	cplane_t GetWorldSurfacePlane() const;

	Box3DPhysicsObject *m_pFluidObject;
	fluidparams_t m_Params;
	cplane_t m_LocalPlane;	// surface plane in fluid-object local space
	CUtlVector< Box3DPhysicsObject * > m_ObjectsInFluid;
};
