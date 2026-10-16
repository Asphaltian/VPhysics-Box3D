#include "parse_file.h"

#include <cstring>

static bool IsWhitespace( char c )
{
	return static_cast<signed char>( c ) <= ' ';
}

static bool IsBreakCharacter( char c )
{
	switch ( c )
	{
	case '{':
	case '}':
	case '(':
	case ')':
	case '\'':
	case ':':
		return true;
	default:
		return false;
	}
}

static void ToLower( char *pString )
{
	for ( ; *pString; ++pString )
	{
		if ( *pString >= 'A' && *pString <= 'Z' )
			*pString += 'a' - 'A';
	}
}

const char *ParseFile( const char *pFileBytes, char *pTokenOut, size_t nMaxTokenLen )
{
	pTokenOut[0] = '\0';

	if ( !pFileBytes || nMaxTokenLen < 2 )
		return nullptr;

	const char *pCursor = pFileBytes;
	for ( ;; )
	{
		while ( IsWhitespace( *pCursor ) )
		{
			if ( *pCursor == '\0' )
				return nullptr;
			++pCursor;
		}

		if ( pCursor[0] == '/' && pCursor[1] == '/' )
		{
			while ( *pCursor && *pCursor != '\n' )
				++pCursor;
			continue;
		}

		if ( pCursor[0] == '/' && pCursor[1] == '*' )
		{
			pCursor += 2;
			while ( *pCursor && !( pCursor[0] == '*' && pCursor[1] == '/' ) )
				++pCursor;
			if ( *pCursor )
				pCursor += 2;
			continue;
		}

		break;
	}

	size_t nLength = 0;

	if ( *pCursor == '"' )
	{
		++pCursor;
		while ( *pCursor && *pCursor != '"' )
		{
			pTokenOut[nLength++] = *pCursor++;
			if ( nLength == nMaxTokenLen - 1 )
				break;
		}

		pTokenOut[nLength] = '\0';
		return *pCursor == '"' ? pCursor + 1 : pCursor;
	}

	if ( IsBreakCharacter( *pCursor ) )
	{
		pTokenOut[0] = *pCursor;
		pTokenOut[1] = '\0';
		return pCursor + 1;
	}

	while ( !IsWhitespace( *pCursor ) && !IsBreakCharacter( *pCursor ) )
	{
		pTokenOut[nLength++] = *pCursor++;
		if ( nLength == nMaxTokenLen - 1 )
			break;
	}

	pTokenOut[nLength] = '\0';
	return pCursor;
}

const char *ParseKeyValue( const char *pFileBytes, char *pKeyOut, char *pValueOut, size_t nMaxTokenLen )
{
	pValueOut[0] = '\0';

	const char *pCursor = ParseFile( pFileBytes, pKeyOut, nMaxTokenLen );
	if ( !std::strcmp( pKeyOut, "}" ) )
		return pCursor;

	ToLower( pKeyOut );
	pCursor = ParseFile( pCursor, pValueOut, nMaxTokenLen );
	ToLower( pValueOut );
	return pCursor;
}
