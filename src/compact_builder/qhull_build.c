#include "qhull_build.h"

#include "qhull_a.h"

int BuildQhull( char *pCommand, double *pPoints, int nCount, FILE *pOutput )
{
	qh_initqhull_start( NULL, pOutput, pOutput );
	int nExitCode = setjmp( qh errexit );
	if ( !nExitCode )
	{
		qh NOerrexit = False;
		qh_initflags( pCommand );
		qh_init_B( pPoints, nCount, 3, False );
		qh_qhull();
		qh_check_output();
		if ( qh VERIFYoutput && !qh STOPpoint && !qh STOPcone )
			qh_check_points();
	}
	qh NOerrexit = True;
	return nExitCode;
}
