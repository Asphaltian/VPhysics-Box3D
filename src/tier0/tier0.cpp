#include "tier0/dbg.h"
#include "tier0/memalloc.h"
#include "tier0/minidump.h"
#include "tier0/platform.h"

#include <cstdlib>

void _SpewInfo( SpewType_t, const tchar *, int )
{
}

SpewRetval_t _SpewMessage( const tchar *, ... )
{
	return SPEW_CONTINUE;
}

void _ExitOnFatalAssert( const tchar *, int )
{
}

bool ShouldUseNewAssertDialog()
{
	return false;
}

bool DoNewAssertDialog( const tchar *, int, const tchar * )
{
	return false;
}

void CallAssertFailedNotifyFunc( const char *, int, const char * )
{
}

void Msg( const tchar *, ... )
{
}

void Warning( const tchar *, ... )
{
}

void Error( const tchar *, ... )
{
	std::abort();
}

void DevMsg( int, const tchar *, ... )
{
}

void DevMsg( const tchar *, ... )
{
}

const CPUInformation *GetCPUInformation()
{
	return nullptr;
}

bool Plat_IsInDebugSession()
{
	return false;
}

void CClockSpeedInit::Init()
{
}

void WriteMiniDump( const char * )
{
}

IMemAlloc *g_pMemAlloc = nullptr;
