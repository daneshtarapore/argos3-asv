/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics2d/chipmunk-physics/include/cpASVDynamics.h>
 *
 * @author Danesh Tarapore - <daneshtarapore@gmail.com>
 */

/**
  ASV Body parameters and kinetic functions governing the movement of the ASV for Chipmunk
 */


#ifndef CHIPMUNK_ASVDYNAMICS_H
#define CHIPMUNK_ASVDYNAMICS_H



typedef struct cpASVBodyParameters
{
    /* Mass of ASV body. Scalar quantity */
    float m;

    /* Inverse of total mass matrix of ASV body (Mass of rigid body + Added mass matrix). Matrix of size 3x3*/
    /* Order of coefficients m_inv<Row><Column> */
    float m_inv11, m_inv12, m_inv13;
    float m_inv21, m_inv22, m_inv23;
    float m_inv31, m_inv32, m_inv33;

    /* Added mass coefficients of the ASV body*/
    float xu1, yv1, nr1, yr1;

    /* Linear drag coefficients of the ASV body */
    float xu2, yv2, nr2;

    /* Quadratic drag coefficients of the ASV body */
    float xu3, yv3, nr3;

    /* Coefficients of ASV thrust vector */
    float t_11, t_21, t_31;
}cpASVBodyParameters;

#endif
