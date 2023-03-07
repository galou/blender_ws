#ifndef TRAJECTORY_BLENDER__VISIBILITY_CONTROL_H_
#define TRAJECTORY_BLENDER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define TRAJECTORY_BLENDER_EXPORT __attribute__ ((dllexport))
    #define TRAJECTORY_BLENDER_IMPORT __attribute__ ((dllimport))
  #else
    #define TRAJECTORY_BLENDER_EXPORT __declspec(dllexport)
    #define TRAJECTORY_BLENDER_IMPORT __declspec(dllimport)
  #endif
  #ifdef TRAJECTORY_BLENDER_BUILDING_LIBRARY
    #define TRAJECTORY_BLENDER_PUBLIC TRAJECTORY_BLENDER_EXPORT
  #else
    #define TRAJECTORY_BLENDER_PUBLIC TRAJECTORY_BLENDER_IMPORT
  #endif
  #define TRAJECTORY_BLENDER_PUBLIC_TYPE TRAJECTORY_BLENDER_PUBLIC
  #define TRAJECTORY_BLENDER_LOCAL
#else
  #define TRAJECTORY_BLENDER_EXPORT __attribute__ ((visibility("default")))
  #define TRAJECTORY_BLENDER_IMPORT
  #if __GNUC__ >= 4
    #define TRAJECTORY_BLENDER_PUBLIC __attribute__ ((visibility("default")))
    #define TRAJECTORY_BLENDER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define TRAJECTORY_BLENDER_PUBLIC
    #define TRAJECTORY_BLENDER_LOCAL
  #endif
  #define TRAJECTORY_BLENDER_PUBLIC_TYPE
#endif

#endif  // TRAJECTORY_BLENDER__VISIBILITY_CONTROL_H_
