#pragma once

#include "physics_constraint_solver.h"
#include "physics_controller.h"

#include "vphysics_interface.h"
#include "vphysics/constraints.h"
#include "isaverestore.h"
#include "tier0/valve_minmax_off.h"

#include "box3d/id.h"
#include "box3d/types.h"

#include <cstdint>
#include <utility>
#include <vector>

class CPhysicsConstraint;
class CPhysicsEnvironment;
class CPhysicsObject;
struct vphysics_save_cphysicsconstraintgroup_t;

class CPhysicsConstraintGroup final : public IPhysicsConstraintGroup, public IController
{
public:
	static constexpr int PRIORITY = 405;
	static constexpr int BASE_ITERATIONS = 2;
	static constexpr int MAX_ITERATIONS = 9;

	CPhysicsConstraintGroup( CPhysicsEnvironment *pEnvironment, int nIterations, int nMinErrorTicks, float flErrorTolerance );
	~CPhysicsConstraintGroup() override;

	void Activate() override;

	bool IsInErrorState() override { return m_nMinErrorTicks <= m_nErrorTicks; }

	void ClearErrorState() override { m_nErrorTicks = 0; }

	void GetErrorParams( constraint_groupparams_t *pParams ) override;
	void SetErrorParams( const constraint_groupparams_t &params ) override;
	void SolvePenetration( IPhysicsObject *pObj0, IPhysicsObject *pObj1 ) override;

	int GetPriority() const override { return PRIORITY; }

	const char *GetName() const override { return "sys:constraint"; }

	void Simulate( CPhysicsObject *pObject, float flTimestep ) override;
	void ObjectDestroyed( CPhysicsObject *pObject ) override;

	bool IsActive() const { return m_bActive; }

	void Deactivate();
	void AddConstraint( CPhysicsConstraint *pConstraint );
	void RemoveConstraint( CPhysicsConstraint *pConstraint );
	bool HasExternalObject( const CPhysicsObject *pObject ) const;
	void OutputDebugInfo() const;
	void WriteRecord( vphysics_save_cphysicsconstraintgroup_t &record ) const;

private:
	void AddObject( CPhysicsObject *pObject );
	void RebuildObjects();
	void RemovePenetrationPairs( const CPhysicsObject *pObject );
	void ApplyPenetrationPairs( float flTimestep );

	static constexpr int MAX_PENETRATION_PAIRS = 4;

	CPhysicsEnvironment *m_pEnvironment;
	std::vector<CPhysicsConstraint *> m_Constraints;
	std::vector<CPhysicsObject *> m_Objects;
	std::pair<CPhysicsObject *, CPhysicsObject *> m_PenetrationPairs[MAX_PENETRATION_PAIRS] = {};
	int m_nPenetrationPairs = 0;
	int m_nIterations;
	int m_nRowConstraints = 0;
	int m_nMinErrorTicks;
	int m_nErrorTicks = 0;
	float m_flErrorTolerance;
	uint32_t m_nSimulatedStep = 0;
	bool m_bActive = false;
	bool m_bOrderDirty = false;
};

class CPhysicsConstraint final : public IPhysicsConstraint
{
public:
	enum Type_t : short
	{
		CONSTRAINT_NONE = 0,
		CONSTRAINT_RAGDOLL = 1,
		CONSTRAINT_HINGE = 2,
		CONSTRAINT_FIXED = 3,
		CONSTRAINT_BALLSOCKET = 4,
		CONSTRAINT_SLIDING = 5,
		CONSTRAINT_PULLEY = 6,
		CONSTRAINT_LENGTH = 7,
	};

	static CPhysicsConstraint *CreateRagdoll( CPhysicsEnvironment *pEnvironment, CPhysicsObject *pReference, CPhysicsObject *pAttached, CPhysicsConstraintGroup *pGroup, const constraint_ragdollparams_t &ragdoll );
	static CPhysicsConstraint *CreateHinge( CPhysicsEnvironment *pEnvironment, CPhysicsObject *pReference, CPhysicsObject *pAttached, CPhysicsConstraintGroup *pGroup, const constraint_limitedhingeparams_t &hinge );
	static CPhysicsConstraint *CreateFixed( CPhysicsEnvironment *pEnvironment, CPhysicsObject *pReference, CPhysicsObject *pAttached, CPhysicsConstraintGroup *pGroup, const constraint_fixedparams_t &fixed );
	static CPhysicsConstraint *CreateSliding( CPhysicsEnvironment *pEnvironment, CPhysicsObject *pReference, CPhysicsObject *pAttached, CPhysicsConstraintGroup *pGroup, const constraint_slidingparams_t &sliding );
	static CPhysicsConstraint *CreateBallsocket( CPhysicsEnvironment *pEnvironment, CPhysicsObject *pReference, CPhysicsObject *pAttached, CPhysicsConstraintGroup *pGroup, const constraint_ballsocketparams_t &ballsocket );
	static CPhysicsConstraint *CreatePulley( CPhysicsEnvironment *pEnvironment, CPhysicsObject *pReference, CPhysicsObject *pAttached, CPhysicsConstraintGroup *pGroup, const constraint_pulleyparams_t &pulley );
	static CPhysicsConstraint *CreateEmpty( CPhysicsEnvironment *pEnvironment );
	static CPhysicsConstraint *CreateLength( CPhysicsEnvironment *pEnvironment, CPhysicsObject *pReference, CPhysicsObject *pAttached, CPhysicsConstraintGroup *pGroup, const constraint_lengthparams_t &length );

	~CPhysicsConstraint() override;

	void Activate() override;
	void Deactivate() override;

	void SetGameData( void *gameData ) override { m_pGameData = gameData; }

	void *GetGameData() const override { return m_pGameData; }

	IPhysicsObject *GetReferenceObject() const override;
	IPhysicsObject *GetAttachedObject() const override;
	void SetLinearMotor( float speed, float maxLinearImpulse ) override;
	void SetAngularMotor( float rotSpeed, float maxAngularImpulse ) override;
	void UpdateRagdollTransforms( const matrix3x4_t &constraintToReference, const matrix3x4_t &constraintToAttached ) override;
	bool GetConstraintTransform( matrix3x4_t *pConstraintToReference, matrix3x4_t *pConstraintToAttached ) const override;
	bool GetConstraintParams( constraint_breakableparams_t *pParams ) const override;
	void OutputDebugInfo() override;

	const char *GetSolverName() const;

	CPhysicsObject *GetReference() const { return m_pReference; }

	CPhysicsObject *GetAttached() const { return m_pAttached; }

	bool IsFixed() const { return m_nType == CONSTRAINT_FIXED; }

	bool ReportsError() const { return m_nType == CONSTRAINT_RAGDOLL && !m_bOnlyAngularLimits; }

	bool HasJoint() const { return B3_IS_NON_NULL( m_Joint ); }

	bool IsSolvedByRows() const { return m_nType == CONSTRAINT_PULLEY || ( m_nType == CONSTRAINT_RAGDOLL && ( m_bOnlyAngularLimits || m_Ragdoll.IsPlaneOnly() ) ); }

	void SetupRows( const ConstraintStep_t &step );
	void SolveRows( float flAlpha, float flBeta, const ConstraintStep_t &step );

	void CreateJoint();
	void DestroyJoint( bool bWakeAttached );
	void UpdateBreakThresholds( float flTimestep );
	float GetJointSeparation() const;
	void ObjectDestroyed( CPhysicsObject *pObject );

	void GroupDestroyed() { m_pGroup = nullptr; }

	void Save( ISave *pSave ) const;

private:
	CPhysicsConstraint( CPhysicsEnvironment *pEnvironment, CPhysicsObject *pReference, CPhysicsObject *pAttached, Type_t nType );

	void Initialize( CPhysicsConstraintGroup *pGroup, const constraint_breakableparams_t &constraint, bool bActivate );
	void BuildRagdoll( const constraint_ragdollparams_t &ragdoll );
	void BuildHinge( const constraint_limitedhingeparams_t &hinge );
	int SelectTwistAxis() const;
	b3Transform GetRagdollFrame( int nBody ) const;
	void UpdateRagdollSolverAxes();
	void PrepareBodies( ConstraintBody_t bodies[2], float scales[2] ) const;
	template<typename Function>
	void RunRows( Function function );
	void DetachObjects();
	void NotifyWokenObjects() const;
	float GetBreakMass() const;
	constraint_breakableparams_t GetSavedBreakableParams() const;

	union JointDef_t
	{
		b3RevoluteJointDef m_Revolute;
		b3PrismaticJointDef m_Prismatic;
		b3WeldJointDef m_Weld;
		b3SphericalJointDef m_Spherical;
		b3DistanceJointDef m_Distance;
		b3FilterJointDef m_Filter;
	};

	static b3JointDef &GetBaseDefinition( JointDef_t &def, b3JointType nType );

	struct RagdollFrames_t
	{
		b3Vec3 m_Axes[2][3];
		b3Vec3 m_Origins[2];
		int m_nTwistAxis;
		int m_nConeAxis;
		int m_nPlaneAxis;
	};

	CPhysicsEnvironment *m_pEnvironment;
	CPhysicsObject *m_pReference;
	CPhysicsObject *m_pAttached;
	CPhysicsConstraintGroup *m_pGroup = nullptr;
	CPhysicsConstraintGroup *m_pPrivateGroup = nullptr;
	void *m_pGameData = nullptr;
	b3JointId m_Joint = b3_nullJointId;
	b3JointType m_nJointType = b3_filterJoint;
	JointDef_t m_Definition = {};
	RagdollFrames_t m_RagdollFrames = {};
	CPulleySolver m_Pulley;
	CRagdollSolver m_Ragdoll;
	matrix3x4_t m_AttachedRefXform = {};
	b3Vec3 m_Anchors[2] = {};
	b3Vec3 m_SlideAxis = b3Vec3_zero;
	constraint_axislimit_t m_RagdollAxes[3] = {};
	float m_flConeRange = 0.0f;
	bool m_bConeCentered = false;
	bool m_bUseClockwiseRotations = false;
	float m_flForceLimit = 0.0f;
	float m_flTorqueLimit = 0.0f;
	float m_BodyMassScale[2] = { 1.0f, 1.0f };
	float m_flBreakMass = -1.0f;
	float m_flBreakTimestep = 0.0f;
	Type_t m_nType;
	bool m_bBreakable = false;
	bool m_bOnlyAngularLimits = false;
	bool m_bListenerRegistered[2] = {};
};
