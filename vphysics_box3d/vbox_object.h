//=================================================================================================
//
// A physics object
//
//=================================================================================================

#pragma once

class IPredictedPhysicsObject;
class Box3DPhysicsEnvironment;
class Box3DPhysicsShadowController;

#if defined( GAME_CSGO_OR_NEWER )
using IPhysicsObjectInterface = IPredictedPhysicsObject;
#else
using IPhysicsObjectInterface = IPhysicsObject;
#endif

class Box3DPhysicsObject final : public IPhysicsObjectInterface
{
public:
	Box3DPhysicsObject( b3BodyId bodyId, Box3DPhysicsEnvironment *pEnvironment, bool bStatic, int nMaterialIndex, const CPhysCollide *pCollide, const objectparams_t *pParams );
	~Box3DPhysicsObject() override;

	bool			IsStatic() const override;
	bool			IsAsleep() const override;
	bool			IsTrigger() const override;
	bool			IsFluid() const override;
	bool			IsHinged() const override;
	bool			IsCollisionEnabled() const override;
	bool			IsGravityEnabled() const override;
	bool			IsDragEnabled() const override;
	bool			IsMotionEnabled() const override;
	bool			IsMoveable() const override;
	bool			IsAttachedToConstraint( bool bExternalOnly ) const override;

	void			EnableCollisions( bool enable ) override;
	void			EnableGravity( bool enable ) override;
	void			EnableDrag( bool enable ) override;
	void			EnableMotion( bool enable ) override;

	void			SetGameData( void *pGameData ) override;
	void *			GetGameData() const override;
	void			SetGameFlags( unsigned short userFlags ) override;
	unsigned short	GetGameFlags() const override;
	void			SetGameIndex( unsigned short gameIndex ) override;
	unsigned short	GetGameIndex() const override;

	void			SetCallbackFlags( unsigned short callbackflags ) override;
	unsigned short	GetCallbackFlags() const override;

	void			Wake() override;
	void			Sleep() override;
	void			RecheckCollisionFilter() override;
	void			RecheckContactPoints( bool bSearchForNewContacts ) override_portal2;
	void			RecheckContactPoints() override_not_portal2 { RecheckContactPoints( false ); }

	void			SetMass( float mass ) override;
	float			GetMass() const override;
	float			GetInvMass() const override;
	Vector			GetInertia() const override;
	Vector			GetInvInertia() const override;
	void			SetInertia( const Vector &inertia ) override;

	void			SetDamping( const float *speed, const float *rot ) override;
	void			GetDamping( float *speed, float *rot ) const override;

	void			SetDragCoefficient( float *pDrag, float *pAngularDrag ) override;
	void			SetBuoyancyRatio( float ratio ) override;

	int				GetMaterialIndex() const override;
	void			SetMaterialIndex( int materialIndex ) override;

	unsigned int	GetContents() const override;
	void			SetContents( unsigned int contents ) override;

	float			GetSphereRadius() const override;
	void			SetSphereRadius( float radius ) override_asw;
	float			GetEnergy() const override;
	Vector			GetMassCenterLocalSpace() const override;

	void			SetPosition( const Vector &worldPosition, const QAngle &angles, bool isTeleport ) override;
	void			SetPositionMatrix( const matrix3x4_t &matrix, bool isTeleport ) override;

	void			GetPosition( Vector *worldPosition, QAngle *angles ) const override;
	void			GetPositionMatrix( matrix3x4_t *positionMatrix ) const override;
	void			SetVelocity( const Vector *velocity, const AngularImpulse *angularVelocity ) override;

	void			SetVelocityInstantaneous( const Vector *velocity, const AngularImpulse *angularVelocity ) override;

	void			GetVelocity( Vector *velocity, AngularImpulse *angularVelocity ) const override;

	void			AddVelocity( const Vector *velocity, const AngularImpulse *angularVelocity ) override;
	void			GetVelocityAtPoint( const Vector &worldPosition, Vector *pVelocity ) const override;
	void			GetImplicitVelocity( Vector *velocity, AngularImpulse *angularVelocity ) const override;
	void			LocalToWorld( Vector *worldPosition, const Vector &localPosition ) const override;
	void			WorldToLocal( Vector *localPosition, const Vector &worldPosition ) const override;

	void			LocalToWorldVector( Vector *worldVector, const Vector &localVector ) const override;
	void			WorldToLocalVector( Vector *localVector, const Vector &worldVector ) const override;

	void			ApplyForceCenter( const Vector &forceVector ) override;
	void			ApplyForceOffset( const Vector &forceVector, const Vector &worldPosition ) override;
	void			ApplyTorqueCenter( const AngularImpulse &torque ) override;

	void			CalculateForceOffset( const Vector &forceVector, const Vector &worldPosition, Vector *centerForce, AngularImpulse *centerTorque ) const override;
	void			CalculateVelocityOffset( const Vector &forceVector, const Vector &worldPosition, Vector *centerVelocity, AngularImpulse *centerAngularVelocity ) const override;
	float			CalculateLinearDrag( const Vector &unitDirection ) const override;
	float			CalculateAngularDrag( const Vector &objectSpaceRotationAxis ) const override;

	bool			GetContactPoint( Vector *contactPoint, IPhysicsObject **contactObject ) const override;

	void			SetShadow( float maxSpeed, float maxAngularSpeed, bool allowPhysicsMovement, bool allowPhysicsRotation ) override;
	void			UpdateShadow( const Vector &targetPosition, const QAngle &targetAngles, bool tempDisableGravity, float timeOffset ) override;

	int							GetShadowPosition( Vector *position, QAngle *angles ) const override;
	IPhysicsShadowController *	GetShadowController() const override;
	void						RemoveShadowController() override;
	float						ComputeShadowControl( const hlshadowcontrol_params_t &params, float secondsToArrival, float dt ) override;

	const CPhysCollide *	GetCollide() const override;
	const char *			GetName() const override;

	void			BecomeTrigger() override;
	void			RemoveTrigger() override;

	void			BecomeHinged( int localAxis ) override;
	void			RemoveHinged() override;

	IPhysicsFrictionSnapshot *CreateFrictionSnapshot() override;
	void DestroyFrictionSnapshot( IPhysicsFrictionSnapshot *pSnapshot ) override;

	void			OutputDebugInfo() const override;

	void			SetUseAlternateGravity( bool bSet ) override_asw;
	void			SetCollisionHints( uint32 collisionHints ) override_asw;
	uint32			GetCollisionHints() const override_asw;

	IPredictedPhysicsObject *	GetPredictedInterface() const override_csgo;
	void						SyncWith( IPhysicsObject *pOther ) override_csgo;

	void SetErrorDelta_Position( const Vector& vPosition ) override_csgo {}
	void SetErrorDelta_Velocity( const Vector& vVelocity ) override_csgo {}

public:
	Box3DPhysicsEnvironment *GetEnvironment() { return m_pEnvironment; }
	b3BodyId GetBodyID() const { return m_BodyId; }

	// Called each frame for bodies that moved, to fire game callbacks (M2+).
	void PostSimulation( float flTimestep ) {}

private:
	// Josh:
	// Always put m_pGameData first. Some games that will remain un-named offset by the
	// vtable to get to this instead of calling GetGameData().
	void *m_pGameData = nullptr;
	const char *m_pName = "NoName";

	uint16 m_gameFlags = 0;
	uint16 m_gameIndex = 0;
	uint16 m_callbackFlags = CALLBACK_GLOBAL_COLLISION | CALLBACK_GLOBAL_FRICTION | CALLBACK_FLUID_TOUCH | CALLBACK_GLOBAL_TOUCH | CALLBACK_GLOBAL_COLLIDE_STATIC | CALLBACK_DO_FLUID_SIMULATION;
	uint32 m_collisionHints = 0;

	bool m_bStatic = false;
	bool m_bMotionEnabled = true;
	bool m_bGravityEnabled = true;
	bool m_bCollisionEnabled = true;
	bool m_bDragEnabled = false;

	int m_materialIndex = 0;
	uint m_contents = CONTENTS_SOLID;

	float m_flLinearDamping = 0.0f;
	float m_flAngularDamping = 0.0f;

	// Box3D reports zero mass for static bodies, so cache what the game set.
	float m_flCachedMass = 0.0f;
	float m_flCachedInvMass = 0.0f;

	const CPhysCollide *m_pCollide = nullptr;

	b3BodyId m_BodyId = b3_nullBodyId;
	Box3DPhysicsEnvironment *m_pEnvironment = nullptr;
	b3WorldId m_WorldId = b3_nullWorldId;

	Box3DPhysicsShadowController *m_pShadowController = nullptr;
};
