
#pragma once

// This function fixes up the base object
// after loading a single KV value.
using Box3DKVSchemaFixupFunc_t = void (*)(void* pBaseObject);

struct Box3DKVSchemaFunc_t
{
    size_t ptr_size;
    void (*ReadFunc)(KeyValues* pProp, void* pPtr, size_t size);
};

struct Box3DKVSchemaProp_t
{
    const char* pszName;
    size_t offset;
    size_t size;
    size_t arrayOffset;
    Box3DKVSchemaFunc_t func;

    Box3DKVSchemaFixupFunc_t fixupFunc;
};

struct Box3DPhysicsIntPair
{
    int Index0;
    int Index1;
};

extern const Box3DKVSchemaFunc_t FillBaseProp;
extern const Box3DKVSchemaFunc_t FillStringProp;
extern const Box3DKVSchemaFunc_t FillIntProp;
extern const Box3DKVSchemaFunc_t FillFloatProp;
extern const Box3DKVSchemaFunc_t FillUnsignedCharProp;
extern const Box3DKVSchemaFunc_t FillBoolProp;
extern const Box3DKVSchemaFunc_t FillVectorProp;
extern const Box3DKVSchemaFunc_t FillVector4DProp;
extern const Box3DKVSchemaFunc_t FillQAngleProp;
extern const Box3DKVSchemaFunc_t FillIntPairProp;
extern const Box3DKVSchemaFunc_t FillGameMaterialProp;
extern const Box3DKVSchemaFunc_t FillSurfaceProp;
extern const Box3DKVSchemaFunc_t FillSoundProp;

#define KVSCHEMA_DESC_ARRAY(type, x, len) offsetof(type, x), sizeof(*type::x), offsetof(type, len)

#define KVSCHEMA_DESC(type, x) offsetof(type, x), sizeof(type::x), static_cast<size_t>(~0llu)

#define KVSCHEMA_DESC_NO_OFFSET(type) 0, sizeof(type), static_cast<size_t>(~0llu)

void ParseBox3DKVSchema(
    KeyValues* pKV, const Box3DKVSchemaProp_t* pDescs, uint count, void* pObj, void* pUnknownKeyObj = nullptr,
    IVPhysicsKeyHandler* pUnknownKeyHandler = nullptr);
void ParseBox3DKVCustom(KeyValues* pKV, void* pUnknownKeyObj, IVPhysicsKeyHandler* pUnknownKeyHandler);
KeyValues* HeaderlessKVBufferToKeyValues(const char* pszBuffer, const char* pszSetName);
