/**
 ************************************************************************************************
 * @file    quaternion.c
 * @author  Miles Richmond (milesf.richmond@gmail.com)
 * @brief   .
 ************************************************************************************************
 */

#include "quaternion.h"

void quat_mult(const quaternion_t *lhs, const quaternion_t *rhs, quaternion_t *result)
{
    result->w = (lhs->w * rhs->w) - (lhs->x * rhs->x) - (lhs->y * rhs->y) - (lhs->z * rhs->z);
    result->x = (lhs->w * rhs->x) + (lhs->x * rhs->w) + (lhs->y * rhs->z) - (lhs->z * rhs->y);
    result->y = (lhs->w * rhs->y) - (lhs->x * rhs->z) + (lhs->y * rhs->w) + (lhs->z * rhs->x);
    result->z = (lhs->w * rhs->z) + (lhs->x * rhs->y) - (lhs->y * rhs->x) + (lhs->z * rhs->w);
}

void quat_inv(const quaternion_t *quat, quaternion_t *result)
{
    result->w = quat->w;
    result->x = -quat->x;
    result->y = -quat->y;
    result->z = -quat->z;
}
