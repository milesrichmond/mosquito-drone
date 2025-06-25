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
    float x;
    float y;
    float z;
} euler_t;

/**
 * A euler counterpart to this function is intentionally omitted.
 * All calculations should be performed using quaternions, but the motors
 * need to have euler.
 */
euler_t convert_quat(const quaternion_t *quat);

void quat_mult(const quaternion_t *lhs, const quaternion_t *rhs, quaternion_t *result);

void quat_inv(const quaternion_t *quat, quaternion_t *result);

#endif
