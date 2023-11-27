#ifndef COMMON_TOOLS_H
#define COMMON_TOOLS_H

#include <stdbool.h>
#include "common_types.h"

#define AbsFloat(X)   (f32_t) fabsf((f32_t)(X))

#define SafeDivideFloat(x, y) (f32_t)((y) != 0.0f ? (x) / (y) : F32_MAX)

#define SignumFloat(x) (f32_t)((x) != 0.0f ? (x) / AbsFloat(x) : 0.0f)

#define SqrtFloat(X) (f32_t) sqrtf((f32_t)(X))

#define LimitsFloat(X, Xlow, Xup) (f32_t)((X) > (Xup) ? (Xup) : ((X) < (Xlow) ? (Xlow) : (X)))

#endif
