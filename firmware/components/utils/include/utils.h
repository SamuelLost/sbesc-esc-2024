#ifndef UTILS_H
#define UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

/** Check if a pointer is valid */
#define IS_VALID(ptr) ((ptr) != NULL)

/** Unused parameter macro */
#define UNUSED(x) (void)(x)

/** Convert a macro to a string */
#define STRINGIFY(x) #x

/** G-force constant */
#define G_FORCE 9.81

#ifdef __cplusplus
}
#endif

#endif // UTILS_H