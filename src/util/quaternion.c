/**
 ************************************************************************************************
 * @file    quaternion.c
 * @author  Miles Richmond (milesf.richmond@gmail.com)
 * @brief   .
 ************************************************************************************************
 */

#include "quaternion.h"
#include <math.h>

void quat(const euler_t *euler, quaternion_t *quat)
{
    float cos_yaw = cosf(euler->yaw * 0.5f);
    float sin_yaw = sinf(euler->yaw * 0.5f);
    float cos_pit = cosf(euler->pitch * 0.5f);
    float sin_pit = sinf(euler->pitch * 0.5f);
    float cos_rol = cosf(euler->roll * 0.5f);
    float sin_rol = sinf(euler->roll * 0.5f);

    quat->w = (cos_rol * cos_pit * cos_yaw) + (sin_rol * sin_pit * sin_yaw);
    quat->x = (sin_rol * cos_pit * cos_yaw) - (cos_rol * sin_pit * sin_yaw);
    quat->y = (cos_rol * sin_pit * cos_yaw) + (sin_rol * cos_pit * sin_yaw);
    quat->z = (cos_rol * cos_pit * sin_yaw) - (sin_rol * sin_pit * cos_yaw);
}

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

void quat_error(const quaternion_t *current, const quaternion_t *desired, quaternion_t *result)
{
    quaternion_t conjugate;
    quat_inv(current, &conjugate);
    quat_mult(desired, &conjugate, result);
}
