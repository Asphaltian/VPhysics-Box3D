//=================================================================================================
//
// Source / Box3D utilities
//
// All unit and type conversion between Source and Box3D lives here. Source works in inches,
// Box3D in metres. Both are Z-up right-handed, so live conversion is pure scale + type; the
// only genuine axis remap in the codebase is reading legacy IVP collision data (see vbox_collide).
//
//=================================================================================================

#pragma once

inline constexpr float InchesToMetres = 0.0254f;
inline constexpr float MetresToInches = 1.0f / 0.0254f;

inline Vector VectorHalfExtent(Vector mins, Vector maxs)
{
    return 0.5f * (maxs - mins);
}

inline Quaternion ToQuaternion(const QAngle& angles)
{
    Quaternion result;
    AngleQuaternion(angles, result);
    return result;
}

inline QAngle ToQAngle(const Quaternion& q)
{
    QAngle result;
    QuaternionAngles(q, result);
    return result;
}

inline QAngle ToQAngle(const matrix3x4_t& m)
{
    QAngle result;
    MatrixAngles(m, result);
    return result;
}

inline Vector Abs(const Vector& v)
{
    Vector result;
    VectorAbs(v, result);
    return result;
}

inline Vector Rotate(const Vector& vector, const Quaternion& angle)
{
    Vector out;
    VectorRotate(vector, angle, out);
    return out;
}

inline Vector Rotate(const Vector& vector, const matrix3x4_t& matrix)
{
    Vector out;
    VectorRotate(vector, matrix, out);
    return out;
}

template<typename T> constexpr T Cube(T x)
{
    return x * x * x;
}

namespace MatrixAxis
{
    enum SourceMatrixAxes
    {
        Forward = 0,
        Left = 1,
        Up = 2,

        X = 0,
        Y = 1,
        Z = 2,

        Origin = 3,
        Projective = 3,
    };
}
using SourceMatrixAxes = MatrixAxis::SourceMatrixAxes;

inline Vector GetColumn(const matrix3x4_t& m, SourceMatrixAxes axis)
{
    Vector value;
    MatrixGetColumn(m, (int)axis, value);
    return value;
}

//
// BoxToSource:
//
// Type conversions from Box3D -> Source types, and unit conversions from
// metres -> inches (distance, area, volume, energy) and radians -> degrees.
namespace BoxToSource
{
    inline constexpr float Factor = MetresToInches;
    inline constexpr float InvFactor = InchesToMetres;

    // Direct type conversions: normals, directions, coefficients, dimensionless quantities.
    inline float Unitless(float value)
    {
        return value;
    }
    inline Vector Unitless(b3Vec3 value)
    {
        return Vector(value.x, value.y, value.z);
    }

    // Any unit with a singular metre factor: distance (m), velocity (m/s), acceleration, force.
    inline float Distance(float value)
    {
        return value * Factor;
    }
    inline Vector Distance(b3Vec3 value)
    {
        return Vector(Distance(value.x), Distance(value.y), Distance(value.z));
    }

    inline float Area(float value)
    {
        return value * Factor * Factor;
    }
    inline float Volume(float value)
    {
        return value * Factor * Factor * Factor;
    }

    // b3Quat -> Quaternion is a direct passthrough (both are x,y,z,w). Angle also does rad -> deg.
    inline Quaternion Quat(b3Quat value)
    {
        return Quaternion(value.v.x, value.v.y, value.v.z, value.s);
    }
    inline float Angle(float value)
    {
        return RAD2DEG(value);
    }
    inline QAngle Angle(b3Quat value)
    {
        return ToQAngle(Quat(value));
    }

    inline float Energy(float value)
    {
        return value / (InvFactor * InvFactor);
    }

    inline float AngularImpulse(float value)
    {
        return Angle(value);
    }
    inline Vector AngularImpulse(b3Vec3 value)
    {
        return Vector(AngularImpulse(value.x), AngularImpulse(value.y), AngularImpulse(value.z));
    }

    // Box3D AABBs are min/max, matching Source's mins/maxs.
    inline void AABBBounds(const b3AABB& box, Vector& outMins, Vector& outMaxs)
    {
        outMins = Distance(box.lowerBound);
        outMaxs = Distance(box.upperBound);
    }

    inline matrix3x4_t Matrix(const b3Transform& t)
    {
        matrix3x4_t m;
        QuaternionMatrix(Quat(t.q), Distance(t.p), m);
        return m;
    }
} // namespace BoxToSource

//
// SourceToBox:
//
// Type conversions from Source -> Box3D types, and unit conversions from
// inches -> metres (distance, area, volume, energy) and degrees -> radians.
namespace SourceToBox
{
    inline constexpr float Factor = InchesToMetres;
    inline constexpr float InvFactor = MetresToInches;

    inline float Unitless(float value)
    {
        return value;
    }
    inline b3Vec3 Unitless(Vector value)
    {
        return b3Vec3{ value.x, value.y, value.z };
    }

    constexpr float Distance(float value)
    {
        return value * Factor;
    }
    inline b3Vec3 Distance(Vector value)
    {
        return b3Vec3{ Distance(value.x), Distance(value.y), Distance(value.z) };
    }

    inline float Area(float value)
    {
        return value * Factor * Factor;
    }
    inline float Volume(float value)
    {
        return value * Factor * Factor * Factor;
    }

    inline b3Quat Quat(Quaternion value)
    {
        return b3Quat{ b3Vec3{ value.x, value.y, value.z }, value.w };
    }
    inline float Angle(float value)
    {
        return DEG2RAD(value);
    }
    inline b3Quat Angle(QAngle value)
    {
        return Quat(ToQuaternion(value));
    }

    inline float Energy(float value)
    {
        return value / (InvFactor * InvFactor);
    }

    inline float AngularImpulse(float value)
    {
        return Angle(value);
    }
    inline b3Vec3 AngularImpulse(Vector value)
    {
        return b3Vec3{ AngularImpulse(value.x), AngularImpulse(value.y), AngularImpulse(value.z) };
    }

    inline b3AABB AABBBounds(Vector mins, Vector maxs)
    {
        return b3AABB{ Distance(mins), Distance(maxs) };
    }

    inline b3Transform Transform(const matrix3x4_t& m)
    {
        Quaternion q;
        MatrixQuaternion(m, q);
        return b3Transform{ Distance(GetColumn(m, MatrixAxis::Origin)), Quat(q) };
    }
} // namespace SourceToBox

// Same as CM_ClearTrace
inline void ClearTrace(trace_t* trace)
{
    memset(trace, 0, sizeof(*trace));
    trace->fraction = 1.0f;
    trace->fractionleftsolid = 0.0f;
    trace->surface.name = "**empty**";
}

//
// Shadow / held-object control math, shared by the shadow controller
// and IPhysicsObject::ComputeShadowControl.
//

// Velocity servo: damp the current velocity, then accelerate toward the remaining delta, each clamped.
inline void ShadowComputeVelocity(
    Vector& vecVelocity, const Vector& vecDelta, float flMaxSpeed, float flMaxDampSpeed, float flScaleDelta, float flDamping,
    Vector* pOutImpulse = nullptr)
{
    if (vecVelocity.LengthSqr() < 1e-6f)
    {
        vecVelocity = vec3_origin;
    }
    else if (flMaxDampSpeed > 0.0f)
    {
        Vector vecDampen = vecVelocity * -flDamping;
        const float flSpeed = vecVelocity.Length() * fabsf(flDamping);
        if (flSpeed > flMaxDampSpeed)
            vecDampen *= flMaxDampSpeed / flSpeed;
        vecVelocity += vecDampen;
    }

    Vector vecAccel = vec3_origin;
    if (flMaxSpeed > 0.0f)
    {
        vecAccel = vecDelta * flScaleDelta;
        const float flSpeed = vecDelta.Length() * flScaleDelta;
        if (flSpeed > flMaxSpeed)
            vecAccel *= flMaxSpeed / flSpeed;
        vecVelocity += vecAccel;
    }

    if (pOutImpulse)
        *pOutImpulse = vecAccel;
}

// Shortest-arc rotation from cur to target as axis * angle (degrees). Clamp w before acos: float drift
// past 1.0 yields a NaN velocity and a deleted entity. (QuaternionAxisAngle returns a unit axis.)
inline Vector ShadowRotationDeltaDegrees(const QAngle& curAngles, const QAngle& targetAngles)
{
    Quaternion qCur, qTarget;
    AngleQuaternion(curAngles, qCur);
    AngleQuaternion(targetAngles, qTarget);

    Quaternion qInv, qDelta;
    QuaternionInvert(qCur, qInv);
    QuaternionMult(qTarget, qInv, qDelta);

    qDelta.w = clamp(qDelta.w, -1.0f, 1.0f);

    Vector axis;
    float flAngleDeg;
    QuaternionAxisAngle(qDelta, axis, flAngleDeg);
    return axis * flAngleDeg;
}

template<typename T> bool VectorContains(const std::vector<T>& vector, const T& object)
{
    return std::find(vector.begin(), vector.end(), object) != vector.end();
}

template<typename T, typename Value> constexpr void Erase(T& c, const Value& value)
{
    auto it = std::remove(c.begin(), c.end(), value);
    c.erase(it, c.end());
}

template<typename T, typename Pred> constexpr void EraseIf(T& c, Pred pred)
{
    auto it = std::remove_if(c.begin(), c.end(), pred);
    c.erase(it, c.end());
}

template<typename T, typename Value> constexpr bool Contains(const T& c, const Value& value)
{
    return c.find(value) != c.end();
}
