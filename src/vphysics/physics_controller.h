#pragma once

class CPhysicsObject;

inline constexpr float STANDING_NORMAL_Z = -0.7f;

class IController
{
public:
	virtual ~IController() = default;

	virtual int GetPriority() const = 0;
	virtual const char *GetName() const = 0;
	virtual void Simulate( CPhysicsObject *pObject, float flTimestep ) = 0;
	virtual void ObjectDestroyed( CPhysicsObject *pObject ) = 0;
};

class CGravityController final : public IController
{
public:
	static constexpr int PRIORITY = 1000;

	int GetPriority() const override { return PRIORITY; }

	const char *GetName() const override { return "sys:gravity"; }

	void Simulate( CPhysicsObject *pObject, float flTimestep ) override;

	void ObjectDestroyed( CPhysicsObject * ) override {}
};

class CDragController final : public IController
{
public:
	static constexpr int PRIORITY = 500;

	int GetPriority() const override { return PRIORITY; }

	const char *GetName() const override { return "vphysics:drag"; }

	void Simulate( CPhysicsObject *pObject, float flTimestep ) override;

	void ObjectDestroyed( CPhysicsObject * ) override {}
};
