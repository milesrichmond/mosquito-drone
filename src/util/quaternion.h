/**
 ************************************************************************************************
 * @file    quaternion.h
 * @author  Miles Richmond (milesf.richmond@gmail.com)
 * @brief   .
 ************************************************************************************************
 */

#ifndef QUATERNION_H
#define QUATERNION_H

typedef struct {
    float w;
    float x;
    float y;
    float z;
} quaternion_t;

typedef struct {
    float roll;
    float pitch;
    float yaw;
} euler_t;

void quat(const euler_t *euler, quaternion_t *quat);

void quat_mult(const quaternion_t *lhs, const quaternion_t *rhs, quaternion_t *result);

void quat_inv(const quaternion_t *quat, quaternion_t *result);

void quat_error(const quaternion_t *current, const quaternion_t *desired, quaternion_t *result);

#endif
