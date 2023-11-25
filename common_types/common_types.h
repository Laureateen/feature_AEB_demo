/**
 * File: common_types.h
 * ==========================================
 *
 * Common analogies for standard data types to be used across all platforms
 *
 */

#ifndef COMMON_TYPES_H
#define COMMON_TYPES_H

#include <stdbool.h>


	/**
	* \defgroup common Common
	*/

	/**
	* \defgroup com_types Common Types
	* \ingroup common
	*/

	/* Basic Type definitions
	 * -------------------------------------------------------------------------
	 */

	 /**
	  * A type alias for bool. Please do not use this, use bool directly.
	  * Also: Please include stdbool.h wherever its needed instead of including common_types.h
	  * \ingroup com_types
	 */
	 // typedef bool bool_t;

	 /**
	 * \ingroup com_types
	 */
	typedef unsigned char u8_t, * pu8_t;

	/**
	* \ingroup com_types
	*/
	typedef signed char   s8_t, * ps8_t;

	/**
	* \ingroup com_types
	*/
	typedef unsigned short u16_t, * pu16_t;

	/**
	* \ingroup com_types
	*/
	typedef signed short   s16_t, * ps16_t;

	/**
	* \ingroup com_types
	*/
	typedef unsigned int u32_t, * pu32_t;

	/**
	* \ingroup com_types
	*/
	typedef signed int   s32_t, * ps32_t;

	/**
	* \ingroup com_types
	*/
	typedef float        f32_t, * pf32_t;

	/**
	* \ingroup com_types
	*/
	typedef unsigned long long u64_t, * pu64_t;

	/**
	* \ingroup com_types
	*/
	typedef signed long long   s64_t, * ps64_t;

	/**
	* \ingroup com_types
	*/
	typedef double             f64_t, * pf64_t;

	/* Basic Type Limits
	 * -------------------------------------------------------------------------
	 */

	 /**
	 * \ingroup com_types
	 */
#define VERY_SMALL_VALUE (0.0000001f)

	 /**
	 * \ingroup com_types
	 */
#define U8_MAX (255u)

	 /**
	 * \ingroup com_types
	 */
#define U8_MIN (0u)

	 /**
	 * \ingroup com_types
	 */
#define S8_MAX (127)

	 /**
	 * \ingroup com_types
	 */
#define S8_MIN (-128)

	 /**
	 * \ingroup com_types
	 */
#define U16_MAX (65535u)

	 /**
	 * \ingroup com_types
	 */
#define U16_MIN (0u)

	 /**
	 * \ingroup com_types
	 */
#define S16_MAX (32767)

	 /**
	 * \ingroup com_types
	 */
#define S16_MIN (-32768)

	 /**
	 * \ingroup com_types
	 */
#define U32_MAX (4294967295u)

	 /**
	 * \ingroup com_types
	 */
#define U32_MIN (0u)

	 /**
	 * \ingroup com_types
	 */
#define S32_MAX (2147483647)

	 /**
	 * \ingroup com_types
	 */
#define S32_MIN (-2147483648)

	 /**
	 * \ingroup com_types
	 */
#define F32_MAX (f32_t)(3.4028237f * pow(10, 38))

	 /**
	 * \ingroup com_types
	 */
#define F32_MIN (f32_t)(-F32_MAX)


#endif /* #ifndef COMMON_TYPES_H */
