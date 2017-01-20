#include <systemlib/param/param.h>

// 16 is max name length

/**
 * Test 1
 *
 * @group IEKF
 * @unit m
 * @min -1
 * @max 1
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(IEKF_TEST1, 1.0f);

/**
 * Test 1
 *
 * @group IEKF
 * @unit m
 * @min -1
 * @max 2
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(IEKF_TEST2, 2.0f);
