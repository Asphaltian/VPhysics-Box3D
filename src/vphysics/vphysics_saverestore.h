#pragma once

#include "datamap.h"
#include "vphysics_interface.h"
#include "vphysics/constraints.h"
#include "tier0/valve_minmax_off.h"

class CPhysCollide;

struct vphysics_save_cphysicsobject_t
{
	const CPhysCollide *pCollide;
	const char *pName;
	float sphereRadius;
	bool isStatic;
	bool collisionEnabled;
	bool gravityEnabled;
	bool dragEnabled;
	bool motionEnabled;
	bool isAsleep;
	bool isTrigger;
	bool asleepSinceCreation;
	bool hasTouchedDynamic;
	bool hasShadowController;
	short collideType;
	short gameIndex;
	int hingeAxis;
	int materialIndex;
	float mass;
	Vector rotInertia;
	float speedDamping;
	float rotSpeedDamping;
	Vector massCenterOverride;
	int callbacks;
	int gameFlags;
	int contentsMask;
	float volume;
	float dragCoefficient;
	float angDragCoefficient;
	IPhysicsShadowController *pShadow;
	Vector origin;
	QAngle angles;
	Vector velocity;
	AngularImpulse angVelocity;

	DECLARE_SIMPLE_DATADESC();
};

struct vphysics_save_shadowcontrolparams_t
{
	Vector targetPosition;
	QAngle targetRotation;
	float maxAngular;
	float maxDampAngular;
	float maxSpeed;
	float maxDampSpeed;
	float dampFactor;
	float teleportDistance;

	DECLARE_SIMPLE_DATADESC();
};

struct vphysics_save_cshadowcontroller_t
{
	IPhysicsObject *pObject;
	float secondsToArrival;

	struct
	{
		float k[3];
	} saveRot, savedRI, currentSpeed;

	float savedMass;
	int savedMaterial;
	int savedFlags;
	bool enable;
	bool allowPhysicsMovement;
	bool allowPhysicsRotation;
	bool isPhysicallyControlled;
	vphysics_save_shadowcontrolparams_t shadowParams;

	DECLARE_SIMPLE_DATADESC();
};

struct vphysics_save_cphysicsspring_t
{
	float constant;
	float naturalLength;
	float damping;
	float relativeDamping;
	Vector startPosition;
	Vector endPosition;
	bool useLocalPositions;
	bool onlyStretch;
	IPhysicsObject *pObjStart;
	IPhysicsObject *pObjEnd;

	DECLARE_SIMPLE_DATADESC();
};

struct vphysics_save_cphysicsconstraintgroup_t
{
	int additionalIterations;
	int minErrorTicks;
	float errorTolerance;
	bool isActive;

	DECLARE_SIMPLE_DATADESC();
};

struct vphysics_save_motioncontroller_t
{
	CUtlVector<IPhysicsObject *> m_objectList;
	int m_nPriority = 0;

	DECLARE_SIMPLE_DATADESC();
};

struct vphysics_save_cphysicsconstraint_t
{
	int constraintType;
	IPhysicsConstraintGroup *pGroup;
	IPhysicsObject *pObjReference;
	IPhysicsObject *pObjAttached;

	DECLARE_SIMPLE_DATADESC();
};

struct vphysics_save_constraintbreakable_t : public constraint_breakableparams_t
{
	DECLARE_SIMPLE_DATADESC();
};

struct vphysics_save_constraintaxislimit_t : public constraint_axislimit_t
{
	DECLARE_SIMPLE_DATADESC();
};

struct vphysics_save_constrainthinge_t : public constraint_hingeparams_t
{
	DECLARE_SIMPLE_DATADESC();
};

struct vphysics_save_constraintfixed_t : public constraint_fixedparams_t
{
	DECLARE_SIMPLE_DATADESC();
};

struct vphysics_save_constraintballsocket_t : public constraint_ballsocketparams_t
{
	DECLARE_SIMPLE_DATADESC();
};

struct vphysics_save_constraintsliding_t : public constraint_slidingparams_t
{
	DECLARE_SIMPLE_DATADESC();
};

struct vphysics_save_constraintpulley_t : public constraint_pulleyparams_t
{
	DECLARE_SIMPLE_DATADESC();
};

struct vphysics_save_constraintlength_t : public constraint_lengthparams_t
{
	DECLARE_SIMPLE_DATADESC();
};

struct vphysics_save_constraintragdoll_t : public constraint_ragdollparams_t
{
	DECLARE_SIMPLE_DATADESC();
};
