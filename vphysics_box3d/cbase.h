//=================================================================================================
//
// This is the precompiled header
//
//=================================================================================================

#pragma once

// Tier0
#include "tier0/basetypes.h"
#include "tier0/dbg.h"

// Workaround mem.h #defining offsetof
// on public SDKs when on Linux.
// We don't want this behaviour.
#ifdef LINUX
#define WAS_LINUX
#undef LINUX
#endif
#include "tier0/mem.h"
#ifdef WAS_LINUX
#define LINUX
#undef WAS_LINUX
#endif

#ifndef GAME_SDK2013
#include "tier0/logging.h"
#endif

#if defined( GAME_SDK2013 )
#include "compat/compat_sdk2013.h"
#elif defined( GAME_ASW )
#include "compat/compat_asw.h"
#endif

#include "compat/branch_overrides.h"

// STD
// Ensure cmath is included everywhere
// so we get those sweet overloaded maths functions
#include <cstdlib>
#include <cmath>

// STL
#include <array>
#include <string>
#include <vector>
#include <algorithm>
#include <utility>
#include <fstream>
#include <unordered_map>
#include <unordered_set>

// Mathlib
#include "mathlib/mathlib.h"
#include "mathlib/vector.h"

// Tier1
#include "tier1/tier1.h"
#include "tier1/strtools.h"
#include "tier1/interface.h"
#ifndef GAME_L4D2_OR_NEWER
#include "tier1/KeyValues.h"
#else
#include "tier1/keyvalues.h"
#endif
#include "tier1/UtlStringMap.h"
#include "tier1/utlbuffer.h"

// Misc
#include "bspfile.h"
#include "cmodel.h"
#include "const.h"
#include "isaverestore.h"
#include "vcollide_parse.h"

// VPhysics Interface
#include "vphysics_interface.h"
#include "vphysics/collision_set.h"
#include "vphysics/constraints.h"
#include "vphysics/friction.h"
#include "vphysics/object_hash.h"
#include "vphysics/performance.h"
#include "vphysics/player_controller.h"
#include "vphysics/stats.h"
#include "vphysics/vehicles.h"
#include "vphysics/virtualmesh.h"

// Box3D
#include <box3d/box3d.h>

// Ourselves
#include "vbox_interface.h"
#include "vbox_util.h"
