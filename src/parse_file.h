#pragma once

#include <cstddef>

const char *ParseFile( const char *pFileBytes, char *pTokenOut, size_t nMaxTokenLen );
const char *ParseKeyValue( const char *pFileBytes, char *pKeyOut, char *pValueOut, size_t nMaxTokenLen );
