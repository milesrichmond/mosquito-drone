/**
 *******************************************************************************
 * @file    parameters.h
 * @author  Miles Richmond (milesf.richmond@gmail.com)
 * @brief   Flight parameters to be used for the mosquito-drone project.
 *******************************************************************************
 */

#define ATTITUDE_PITCH_GAIN     (1.0f)
#define ATTITUDE_ROLL_GAIN      (1.0f)
#define ATTITUDE_YAW_GAIN       (1.0f)

/* The true maximum throttle value that would occur can be found through
 * THROTTLE_MAX + THROTTLE_CORRECTION_MAX */
#define THROTTLE_MAX            (0.7f)
#define THROTTLE_CORRECTION_MAX (0.1f)
