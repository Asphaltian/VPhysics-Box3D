#pragma once

class CPhysConvex;
class CPolyhedron;
class Vector;

CPhysConvex *CreateTriangleConvex( const Vector &a, const Vector &b, const Vector &c );
CPhysConvex *CreateConvexFromPolyhedron( const CPolyhedron &polyhedron );
CPolyhedron *CreatePolyhedronFromConvex( const CPhysConvex &convex, bool bUseTempPolyhedron );
