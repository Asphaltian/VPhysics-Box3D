#pragma once

#include "compact_builder/math_util.h"

#include "box3d/math_functions.h"

#include <numbers>

class CPhysicsObject;

inline constexpr float PI = std::numbers::pi_v<float>;
inline constexpr float TWO_PI = 2.0f * std::numbers::pi_v<float>;
inline constexpr float SERVO_GAIN = 0.8f;

struct FloatMatrix_t
{
	float m[3][3];
};

struct ConstraintBody_t
{
	CPhysicsObject *m_pObject;
	float m_flInverseMass;
	b3Vec3 m_InverseInertia;
	bool m_bDynamic;
};

struct ConstraintStep_t
{
	float m_flTimestep;
	float m_flInverseTimestep;
};

class CPulleySolver
{
public:
	void Initialize( const b3Vec3 anchors[2], const b3Vec3 pulleys[2], float flLength, float flGearRatio, float flStrength, bool bRigid );
	void Setup( ConstraintBody_t bodies[2], const ConstraintStep_t &step );
	void Solve( ConstraintBody_t bodies[2], float flAlpha, float flBeta, const ConstraintStep_t &step );

	b3Vec3 GetAnchor( int nBody ) const { return m_Anchors[nBody]; }

	b3Vec3 GetPulley( int nBody ) const { return m_Pulleys[nBody]; }

	float GetLength() const { return m_flLength; }

	float GetGearRatio() const { return m_flGearRatio; }

	bool IsRigid() const { return m_bRigid; }

private:
	float MeasureError( const b3Vec3 points[2], b3Vec3 directions[2] ) const;
	bool IsSlackAhead( ConstraintBody_t bodies[2], const ConstraintStep_t &step ) const;
	void Apply( ConstraintBody_t bodies[2], float flImpulse ) const;

	b3Vec3 m_Anchors[2] = {};
	b3Vec3 m_Pulleys[2] = {};
	float m_flLength = 0.0f;
	float m_flGearRatio = 1.0f;
	float m_flStrength = 1.0f;
	bool m_bRigid = false;

	b3Vec3 m_Linear[2] = {};
	b3Vec3 m_Angular[2] = {};
	b3Vec3 m_Response[2] = {};
	float m_flError = 0.0f;
	float m_flEffectiveMass = 0.0f;
	bool m_bActive = false;
};

class CLengthSolver
{
public:
	void Initialize( const b3Vec3 anchors[2], float flMinLength, float flMaxLength, float flStrength );
	void Setup( ConstraintBody_t bodies[2], const ConstraintStep_t &step );
	void Solve( ConstraintBody_t bodies[2], float flAlpha, float flBeta, const ConstraintStep_t &step );

	b3Vec3 GetAnchor( int nBody ) const { return m_Anchors[nBody]; }

	float GetMinLength() const { return m_flMinLength; }

	float GetMaxLength() const { return m_flMaxLength; }

private:
	bool IsOutsideAhead( ConstraintBody_t bodies[2], const ConstraintStep_t &step ) const;
	void Apply( ConstraintBody_t bodies[2], float flImpulse ) const;

	b3Vec3 m_Anchors[2] = {};
	float m_flMinLength = 0.0f;
	float m_flMaxLength = 0.0f;
	float m_flStrength = 1.0f;

	b3Vec3 m_Linear[2] = {};
	b3Vec3 m_Angular[2] = {};
	b3Vec3 m_Response[2] = {};
	float m_flError = 0.0f;
	float m_flEffectiveMass = 0.0f;
	bool m_bActive = false;
};

class CSlidingSolver
{
public:
	void Initialize( b3Vec3 axis, float flWeight, float flLower, float flUpper, bool bLimited, float flReference );
	void SetMotor( float flTargetSpeed, float flMaxForce, bool bActive );

	bool NeedsSolve() const { return m_bLimited || m_bMotor; }

	void Setup( ConstraintBody_t bodies[2], const ConstraintStep_t &step, float flPosition );
	void Solve( ConstraintBody_t bodies[2], float flAlpha, float flBeta, const ConstraintStep_t &step );

private:
	b3Vec3 m_Axis = b3Vec3_zero;
	float m_flWeight = 0.0f;
	float m_flLower = 0.0f;
	float m_flUpper = 0.0f;
	float m_flReference = 0.0f;
	float m_flTargetSpeed = 0.0f;
	float m_flMaxForce = 0.0f;
	bool m_bLimited = false;
	bool m_bMotor = false;

	b3Vec3 m_Linear[2] = {};
	b3Vec3 m_Angular[2] = {};
	float m_flPosition = 0.0f;
	float m_flInverseEffectiveMass = 0.0f;
	float m_flEffectiveMass = 0.0f;
};

class CRagdollSolver
{
public:
	enum Slot_t
	{
		SLOT_TWIST,
		SLOT_CONE,
		SLOT_PLANE,
		SLOT_COUNT,
	};

	struct Limit_t
	{
		float m_flLower = 0.0f;
		float m_flUpper = 0.0f;
		float m_flMaxTorque = 0.0f;
		float m_flTargetSpeed = 0.0f;
		float m_flReference = 0.0f;
		bool m_bLimited = false;
		bool m_bFriction = false;
	};

	void SetAxes( int nBody, b3Vec3 twist, b3Vec3 cone );

	void SetPlaneOnly( bool bPlaneOnly ) { m_bPlaneOnly = bPlaneOnly; }

	bool IsPlaneOnly() const { return m_bPlaneOnly; }

	bool NeedsSolve() const { return !m_bPlaneOnly || m_Limits[SLOT_PLANE].m_bLimited; }

	void Setup( ConstraintBody_t bodies[2], const ConstraintStep_t &step );
	void Solve( ConstraintBody_t bodies[2], float flAlpha, float flBeta, const ConstraintStep_t &step );
	float GetSwingLimit() const;

	float GetTwist() const { return m_Rows[SLOT_TWIST].m_flMeasure; }

	Limit_t &GetLimit( Slot_t nSlot ) { return m_Limits[nSlot]; }

	const Limit_t &GetLimit( Slot_t nSlot ) const { return m_Limits[nSlot]; }

private:
	struct Row_t
	{
		b3Vec3 m_Jacobian[2];
		b3Vec3 m_Response[2];
		float m_flInverseEffectiveMass;
		float m_flEffectiveMass;
		float m_flMeasure;
		bool m_bEnabled;
	};

	static float GetRowVelocity( const ConstraintBody_t bodies[2], const Row_t &row );
	void BuildRow( ConstraintBody_t bodies[2], const FloatMatrix_t rotations[2], Row_t &row, b3Vec3 axis ) const;
	void BuildSwingRow( ConstraintBody_t bodies[2], const FloatMatrix_t rotations[2], Row_t &row, b3Vec3 a, b3Vec3 b ) const;
	void SolveTwist( ConstraintBody_t bodies[2], float flAlpha, float flBeta, const ConstraintStep_t &step );
	void SolveSwing( ConstraintBody_t bodies[2], Slot_t nSlot, float flAlpha, float flBeta, const ConstraintStep_t &step );
	static void ApplyRow( ConstraintBody_t bodies[2], const Row_t &row, float flImpulse );

	b3Vec3 m_Twist[2] = {};
	b3Vec3 m_Cone[2] = {};
	Limit_t m_Limits[SLOT_COUNT];
	Row_t m_Rows[SLOT_COUNT] = {};
	float m_flHalfSwingCosine = 0.0f;
	float m_flSwingCosine = 1.0f;
	bool m_bPlaneOnly = false;
};
