
#include "vbox_keyvalues_schema.h"

#include "cbase.h"
#include "vbox_surfaceprops.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

//-------------------------------------------------------------------------------------------------

const Box3DKVSchemaFunc_t FillBaseProp = { sizeof(surfacedata_t), [](KeyValues* pProp, void* pPtr, size_t size) {
    surfacedata_t* pSurfaceDataPtr = reinterpret_cast<surfacedata_t*>(pPtr);

    int nSurfaceIndex = Box3DPhysicsSurfaceProps::GetInstance().GetSurfaceIndex(pProp->GetString());
    if (nSurfaceIndex == -1)
    {
        Log_Warning(
            LOG_VBox3D, "You must specify the base material %s before it can be used. Defaulting to 'default' as a base.\n",
            pProp->GetString());
        // Always the default material.
        nSurfaceIndex = 0;
    }

    *pSurfaceDataPtr = *Box3DPhysicsSurfaceProps::GetInstance().GetSurfaceData(nSurfaceIndex);
} };

const Box3DKVSchemaFunc_t FillStringProp = { 0 /* Varies */, [](KeyValues* pProp, void* pPtr, size_t size) {
    char* pszStringPtr = reinterpret_cast<char*>(pPtr);
    V_strncpy(pszStringPtr, pProp->GetString(), static_cast<strlen_t>(size));
} };

const Box3DKVSchemaFunc_t FillIntProp = { sizeof(int), [](KeyValues* pProp, void* pPtr, size_t size) {
    int* pIntPtr = reinterpret_cast<int*>(pPtr);
    *pIntPtr = pProp->GetInt();
} };

const Box3DKVSchemaFunc_t FillFloatProp = { sizeof(float), [](KeyValues* pProp, void* pPtr, size_t size) {
    float* pFloatPtr = reinterpret_cast<float*>(pPtr);
    *pFloatPtr = pProp->GetFloat();
} };

const Box3DKVSchemaFunc_t FillUnsignedCharProp = { sizeof(unsigned char), [](KeyValues* pProp, void* pPtr, size_t size) {
    unsigned char* pCharPtr = reinterpret_cast<unsigned char*>(pPtr);
    *pCharPtr = static_cast<unsigned char>(pProp->GetInt());
} };

const Box3DKVSchemaFunc_t FillBoolProp = { sizeof(bool), [](KeyValues* pProp, void* pPtr, size_t size) {
    bool* pBoolPtr = reinterpret_cast<bool*>(pPtr);
    *pBoolPtr = pProp->GetBool();
} };

const Box3DKVSchemaFunc_t FillVectorProp = { sizeof(Vector), [](KeyValues* pProp, void* pPtr, size_t size) {
    Vector* pVectorPtr = reinterpret_cast<Vector*>(pPtr);
    sscanf(pProp->GetString(), "%f %f %f", &pVectorPtr->x, &pVectorPtr->y, &pVectorPtr->z);
} };

const Box3DKVSchemaFunc_t FillVector4DProp = { sizeof(Vector4D), [](KeyValues* pProp, void* pPtr, size_t size) {
    Vector4D* pVector4DPtr = reinterpret_cast<Vector4D*>(pPtr);
    sscanf(pProp->GetString(), "%f %f %f %f", &pVector4DPtr->x, &pVector4DPtr->y, &pVector4DPtr->z, &pVector4DPtr->w);
} };

const Box3DKVSchemaFunc_t FillQAngleProp = { sizeof(QAngle), [](KeyValues* pProp, void* pPtr, size_t size) {
    QAngle* pQAnglePtr = reinterpret_cast<QAngle*>(pPtr);
    sscanf(pProp->GetString(), "%f %f %f", &pQAnglePtr->x, &pQAnglePtr->y, &pQAnglePtr->z);
} };

const Box3DKVSchemaFunc_t FillIntPairProp = { sizeof(Box3DPhysicsIntPair), [](KeyValues* pProp, void* pPtr, size_t size) {
    Box3DPhysicsIntPair* pIntPairPtr = reinterpret_cast<Box3DPhysicsIntPair*>(pPtr);
    sscanf(pProp->GetString(), "%d,%d", &pIntPairPtr->Index0, &pIntPairPtr->Index1);
} };

const Box3DKVSchemaFunc_t FillGameMaterialProp = { sizeof(unsigned short), [](KeyValues* pProp, void* pPtr, size_t size) {
    const char* pValue = pProp->GetString();
    unsigned short* pShortPtr = reinterpret_cast<unsigned short*>(pPtr);

    if (V_strlen(pValue) == 1 && !V_isdigit(pValue[0]))
        *pShortPtr = FastASCIIToUpper(pValue[0]);
    else
        *pShortPtr = pProp->GetInt();
} };

const Box3DKVSchemaFunc_t FillSoundProp = { sizeof(unsigned short), [](KeyValues* pProp, void* pPtr, size_t size) {
    const char* pValue = pProp->GetString();
    unsigned short* pShortPtr = reinterpret_cast<unsigned short*>(pPtr);

    *pShortPtr = Box3DPhysicsSurfaceProps::GetInstance().RegisterSound(pValue);
} };

const Box3DKVSchemaFunc_t FillSurfaceProp = { sizeof(int), [](KeyValues* pProp, void* pPtr, size_t size) {
    const char* pValue = pProp->GetString();
    int* pIntPtr = reinterpret_cast<int*>(pPtr);

    *pIntPtr = Box3DPhysicsSurfaceProps::GetInstance().GetSurfaceIndex(pValue);
} };

//-------------------------------------------------------------------------------------------------

void ParseBox3DKVSchema(
    KeyValues* pKV, const Box3DKVSchemaProp_t* pDescs, uint count, void* pObj, void* pUnknownKeyObj,
    IVPhysicsKeyHandler* pUnknownKeyHandler)
{
    bool bHandled = false;

    for (KeyValues* pProp = pKV->GetFirstSubKey(); pProp != nullptr; pProp = pProp->GetNextKey())
    {
        const char* pName = pProp->GetName();

        for (uint i = 0; i < count; i++)
        {
            const Box3DKVSchemaProp_t& desc = pDescs[i];

            if (!V_stricmp(pName, desc.pszName))
            {
                VBoxAssertMsg(
                    desc.func.ptr_size == 0 || desc.size == desc.func.ptr_size,
                    "Desc element size does not match function size");

                int* pArraySize = nullptr;
                if (desc.arrayOffset != static_cast<size_t>(~0llu))
                    pArraySize = reinterpret_cast<int*>(reinterpret_cast<char*>(pObj) + desc.arrayOffset);

                char* pElement = reinterpret_cast<char*>(pObj) + desc.offset;
                if (pArraySize)
                    pElement += *pArraySize * desc.size;

                desc.func.ReadFunc(pProp, reinterpret_cast<void*>(pElement), desc.size);
                if (desc.fixupFunc)
                    desc.fixupFunc(pObj);
                if (pArraySize)
                    (*pArraySize)++;
                bHandled = true;
            }
        }

        if (!bHandled && pUnknownKeyHandler)
            pUnknownKeyHandler->ParseKeyValue(pUnknownKeyObj, pName, pProp->GetString());
    }
}

void ParseBox3DKVCustom(KeyValues* pKV, void* pUnknownKeyObj, IVPhysicsKeyHandler* pUnknownKeyHandler)
{
    // Josh:
    // Parse out custom KV entries like "vehicle_sounds" etc
    // out recursively.

    for (KeyValues* pProp = pKV->GetFirstSubKey(); pProp != nullptr; pProp = pProp->GetNextKey())
    {
        if (pUnknownKeyHandler)
            pUnknownKeyHandler->ParseKeyValue(pUnknownKeyObj, pProp->GetName(), pProp->GetString());

        ParseBox3DKVCustom(pProp, pUnknownKeyObj, pUnknownKeyHandler);
    }
}

//-------------------------------------------------------------------------------------------------

KeyValues* HeaderlessKVBufferToKeyValues(const char* pszBuffer, const char* pszSetName)
{
    CUtlBuffer buffer;
    buffer.SetBufferType(true, true);

    buffer.SeekPut(CUtlBuffer::SEEK_HEAD, 0);
    buffer.PutString("\"PhysProps\"\r\n{");
    buffer.PutString(pszBuffer);
    buffer.PutString("\r\n}");
    buffer.PutChar('\0');

    KeyValues* pszKV = new KeyValues(pszSetName);

    if (!pszKV->LoadFromBuffer(pszSetName, buffer))
    {
        pszKV->deleteThis();
        pszKV = nullptr;
    }

    return pszKV;
}
