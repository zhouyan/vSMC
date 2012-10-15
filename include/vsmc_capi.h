#ifndef VSMC_CAPI_H
#define VSMC_CAPI_H

#include <stddef.h>

#define VSMC_SAMPLER_PTR  void *

#define VSMC_RESAMPLE_MULTINOMIAL           101
#define VSMC_RESAMPLE_RESIDUAL              102
#define VSMC_RESAMPLE_STRATIFIED            103
#define VSMC_RESAMPLE_SYSTEMATIC            104
#define VSMC_RESAMPLE_RESIDUAL_STRATIFIED   105
#define VSMC_RESAMPLE_RESIDUAL_SYSTEMATIC   106

#define VSMC_BASE_SEQUENTIAL 201
#define VSMC_BASE_CILK       202
#define VSMC_BASE_OMP        203
#define VSMC_BASE_TBB        204
#define VSMC_BASE_THREAD     205

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

typedef struct
{
    void *sampler_ptr;
    int sampler_type;
} vsmcSamplerInfo;

vsmcSamplerInfo vsmc_new_sampler (size_t, unsigned, int, double, int);
void vsmc_delete_sampler (VSMC_SAMPLER_PTR);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif // VSMC_CAPI_H
