// Box3D's ctz.h (b3PopCount64) calls the __popcnt64 intrinsic unconditionally on
// MSVC, but that intrinsic only exists on x64. Source SDK 2013 is a 32-bit target,
// so provide a software fallback here rather than patching the box3d submodule.

#if defined( _MSC_VER ) && !defined( _WIN64 )

unsigned __int64 __popcnt64( unsigned __int64 value )
{
	value = value - ( ( value >> 1 ) & 0x5555555555555555ull );
	value = ( value & 0x3333333333333333ull ) + ( ( value >> 2 ) & 0x3333333333333333ull );
	value = ( value + ( value >> 4 ) ) & 0x0f0f0f0f0f0f0f0full;
	return ( value * 0x0101010101010101ull ) >> 56;
}

#endif
