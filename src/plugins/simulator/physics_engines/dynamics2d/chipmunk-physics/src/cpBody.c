/* Copyright (c) 2007 Scott Lembcke
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
 
#include <stdlib.h>
#include <float.h>
#include <math.h>
#include <stdio.h>

#include "chipmunk_private.h"
#include "constraints/util.h"

// initialized in cpInitChipmunk()
cpBody cpStaticBodySingleton;

cpBody*
cpBodyAlloc(void)
{
	return (cpBody *)cpcalloc(1, sizeof(cpBody));
}

cpBody *
cpBodyInit(cpBody *body, cpFloat m, cpFloat i)
{
	body->space = NULL;
	body->shapeList = NULL;
	body->arbiterList = NULL;
	body->constraintList = NULL;
	
	body->velocity_func = cpBodyUpdateVelocity;
	body->position_func = cpBodyUpdatePosition;
	
    body->ControlledBody = NULL;

	cpComponentNode node = {NULL, NULL, 0.0f};
	body->node = node;
	
	body->p = cpvzero;
	body->v = cpvzero;
    body->vl = cpvzero;
    body->v_dot = cpvzero;
	body->f = cpvzero;
	
	body->w = 0.0f;
    body->w_dot = 0.0f;
	body->t = 0.0f;
	
	body->v_bias = cpvzero;
	body->w_bias = 0.0f;
	
	body->v_limit = (cpFloat)INFINITY;
	body->w_limit = (cpFloat)INFINITY;
	
	body->data = NULL;
	
	// Setters must be called after full initialization so the sanity checks don't assert on garbage data.
	cpBodySetMass(body, m);
	cpBodySetMoment(body, i);
	cpBodySetAngle(body, 0.0f);
	
	return body;
}

cpBody*
cpBodyNew(cpFloat m, cpFloat i)
{
	return cpBodyInit(cpBodyAlloc(), m, i);
}

cpBody *
cpBodyInitStatic(cpBody *body)
{
	cpBodyInit(body, (cpFloat)INFINITY, (cpFloat)INFINITY);
	body->node.idleTime = (cpFloat)INFINITY;
	
	return body;
}

cpBody *
cpBodyNewStatic()
{
	return cpBodyInitStatic(cpBodyAlloc());
}

void cpBodyDestroy(cpBody *body){}

void
cpBodyFree(cpBody *body)
{
	if(body){
		cpBodyDestroy(body);
		cpfree(body);
	}
}

static void cpv_assert_nan(cpVect v, char *message){cpAssertSoft(v.x == v.x && v.y == v.y, message);}
static void cpv_assert_infinite(cpVect v, char *message){cpAssertSoft(cpfabs(v.x) != INFINITY && cpfabs(v.y) != INFINITY, message);}
static void cpv_assert_sane(cpVect v, char *message){cpv_assert_nan(v, message); cpv_assert_infinite(v, message);}

#ifdef __cplusplus
extern "C" {
#endif

void
cpBodySanityCheck(cpBody *body)
{
	cpAssertSoft(body->m == body->m && body->m_inv == body->m_inv, "Body's mass is invalid.");
	cpAssertSoft(body->i == body->i && body->i_inv == body->i_inv, "Body's moment is invalid.");
	
	cpv_assert_sane(body->p, "Body's position is invalid.");
	cpv_assert_sane(body->v, "Body's velocity is invalid.");
	cpv_assert_sane(body->f, "Body's force is invalid.");

	cpAssertSoft(body->a == body->a && cpfabs(body->a) != INFINITY, "Body's angle is invalid.");
	cpAssertSoft(body->w == body->w && cpfabs(body->w) != INFINITY, "Body's angular velocity is invalid.");
	cpAssertSoft(body->t == body->t && cpfabs(body->t) != INFINITY, "Body's torque is invalid.");
	
	cpv_assert_sane(body->rot, "Internal error: Body's rotation vector is invalid.");
	
	cpAssertSoft(body->v_limit == body->v_limit, "Body's velocity limit is invalid.");
	cpAssertSoft(body->w_limit == body->w_limit, "Body's angular velocity limit is invalid.");
}

#ifdef __cplusplus
}
#endif

void
cpBodySetMass(cpBody *body, cpFloat mass)
{
	cpBodyActivate(body);
	body->m = mass;
	body->m_inv = 1.0f/mass;
}

void
cpBodySetMoment(cpBody *body, cpFloat moment)
{
	cpBodyActivate(body);
	body->i = moment;
	body->i_inv = 1.0f/moment;
}

void
cpBodyAddShape(cpBody *body, cpShape *shape)
{
	cpShape *next = body->shapeList;
	if(next) next->prev = shape;
	
	shape->next = next;
	body->shapeList = shape;
}

void
cpBodyRemoveShape(cpBody *body, cpShape *shape)
{
  cpShape *prev = shape->prev;
  cpShape *next = shape->next;
  
  if(prev){
		prev->next = next;
  } else {
		body->shapeList = next;
  }
  
  if(next){
		next->prev = prev;
	}
  
  shape->prev = NULL;
  shape->next = NULL;
}

static cpConstraint *
filterConstraints(cpConstraint *node, cpBody *body, cpConstraint *filter)
{
	if(node == filter){
		return cpConstraintNext(node, body);
	} else if(node->a == body){
		node->next_a = filterConstraints(node->next_a, body, filter);
	} else {
		node->next_b = filterConstraints(node->next_b, body, filter);
	}
	
	return node;
}

void
cpBodyRemoveConstraint(cpBody *body, cpConstraint *constraint)
{
	body->constraintList = filterConstraints(body->constraintList, body, constraint);
}

void
cpBodySetPos(cpBody *body, cpVect pos)
{
	cpBodyActivate(body);
	cpBodyAssertSane(body);
	body->p = pos;
}

static inline void
setAngle(cpBody *body, cpFloat angle)
{
	body->a = angle;//fmod(a, (cpFloat)M_PI*2.0f);
	body->rot = cpvforangle(angle);
}

void
cpBodySetAngle(cpBody *body, cpFloat angle)
{
	cpBodyActivate(body);
	cpBodyAssertSane(body);
	setAngle(body, angle);
}

void
cpBodyUpdateVelocity(cpBody *body, cpVect gravity, cpFloat damping, cpFloat dt)
{

//    cpASVBodyInverseDynamics(body);
//    //printf("\n cpBodyUpdateVelocity body->v_dot: <%f %f>; \n",body->v_dot.x, body->v_dot.y);

//    body->v = cpvadd(body->v, cpvmult(body->v_dot, dt));
//    body->w = body->w + body->w_dot * dt;

//    //printf("\n cpBodyUpdateVelocity body->v: <%f %f>; \n",body->v.x, body->v.y);


    body->v = cpvadd(cpvmult(body->v, damping), cpvmult(cpvadd(gravity, cpvmult(body->f, body->m_inv)), dt));

	body->v = cpvclamp(cpvadd(cpvmult(body->v, damping), cpvmult(cpvadd(gravity, cpvmult(body->f, body->m_inv)), dt)), body->v_limit);
	
	cpFloat w_limit = body->w_limit;
    body->w = cpfclamp(body->w*damping + body->t*body->i_inv*dt, -w_limit, w_limit);
	
	cpBodySanityCheck(body);
}

void
cpBodyUpdatePosition(cpBody *body, cpFloat dt)
{
    //printf("\ndt in cpBodyUpdatePosition %f\n  body->v_bias is %f,%f \n", dt, body->v_bias.x, body->v_bias.y);

    //printf("\n cpBodyUpdatePosition body->v: <%f %f>; \n",body->v.x, body->v.y);

	body->p = cpvadd(body->p, cpvmult(cpvadd(body->v, body->v_bias), dt));
	setAngle(body, body->a + (body->w + body->w_bias)*dt);

    //printf("\n cpBodyUpdatePosition body->p: <%f %f>; \n",body->p.x, body->p.y);
	
	body->v_bias = cpvzero;
	body->w_bias = 0.0f;
	
	cpBodySanityCheck(body);
}

void
cpBodyResetForces(cpBody *body)
{
	cpBodyActivate(body);
	body->f = cpvzero;
	body->t = 0.0f;
}

void
cpBodyApplyForce(cpBody *body, cpVect force, cpVect r)
{
	cpBodyActivate(body);
	body->f = cpvadd(body->f, force);
	body->t += cpvcross(r, force);
}

void
cpBodyApplyImpulse(cpBody *body, const cpVect j, const cpVect r)
{
	cpBodyActivate(body);
	apply_impulse(body, j, r);
}

void
cpBodyEachShape(cpBody *body, cpBodyShapeIteratorFunc func, void *data)
{
	cpShape *shape = body->shapeList;
	while(shape){
		cpShape *next = shape->next;
		func(body, shape, data);
		shape = next;
	}
}

void
cpBodyEachConstraint(cpBody *body, cpBodyConstraintIteratorFunc func, void *data)
{
	cpConstraint *constraint = body->constraintList;
	while(constraint){
		cpConstraint *next = cpConstraintNext(constraint, body);
		func(body, constraint, data);
		constraint = next;
	}
}

void
cpBodyEachArbiter(cpBody *body, cpBodyArbiterIteratorFunc func, void *data)
{
	cpArbiter *arb = body->arbiterList;
	while(arb){
		cpArbiter *next = cpArbiterNext(arb, body);
		
		arb->swappedColl = (body == arb->body_b);
		func(body, arb, data);
		
		arb = next;
	}
}

void
cpASVBodyInverseDynamics(cpBody *body)
{
    //printf("\n cpASVBodyInverseDynamics body->asv_parameters.tt: <%f %f %f>; \n",body->asv_parameters.t_11, body->asv_parameters.t_21, body->asv_parameters.t_31);

    // Convert velocity vector from world to local coordinate frame
    //cpVect vl = cpvunrotate(body->v, body->rot);

    // Compute acceleration in local coordinate frame
    body->v_dot.x =
    body->asv_parameters.m_inv11 *
    (body->asv_parameters.t_11 -
     body->vl.x * body->asv_parameters.xu2 +
     body->w * (body->w * body->asv_parameters.yr1 + body->vl.y * (body->asv_parameters.m + body->asv_parameters.yv1)) -
     body->vl.x * body->asv_parameters.xu3 * abs(body->vl.x));

    body->v_dot.y =
    body->asv_parameters.m_inv22 *
    (body->asv_parameters.t_21 -
     body->w * body->vl.x * (body->asv_parameters.m  + body->asv_parameters.xu1) -
     body->vl.y * body->asv_parameters.yv2 -
     body->vl.y * body->asv_parameters.yv3 * abs(body->vl.y));

     body->w_dot =
     body->asv_parameters.m_inv33 *
     (body->asv_parameters.t_31 -
      body->asv_parameters.nr2 * body->w +
      body->vl.x * body->vl.y * body->asv_parameters.xu1 -
      body->w * body->vl.x * body->asv_parameters.yr1 -
      body->vl.x * body->vl.y * body->asv_parameters.yv1 -
      body->asv_parameters.nr3 * body->w * abs(body->w));
}

void
cpControlBodyUpdateVelocity(cpBody *body, cpFloat dt)
{
    // Compute acceleration in local coordinate frame
    cpASVBodyInverseDynamics(body);
    //printf("\n cpBodyUpdateVelocity body->vlocal_dot: <%f %f>; \n",body->v_dot.x, body->v_dot.y);

    // Convert velocity vector from world to local coordinate frame
    // cpVect vl = cpvunrotate(body->v, body->rot);

    // Compute new velocity vector and then convert to world coordinate frame
    body->vl = cpvadd(body->vl, cpvmult(body->v_dot, dt));
    body->w = body->w + body->w_dot * dt;

    body->v = cpvrotate(body->vl, body->ControlledBody->rot);

    //body->v.x = cos(body->a)*body->vl.x - sin(body->a)*body->vl.y;
    //body->v.y = sin(body->a)*body->vl.x + cos(body->a)*body->vl.y;

    //printf("\n cpBodyUpdateVelocity body->v: <%f %f>; \n",body->v.x, body->v.y);

    /*body->v = cpvadd(cpvmult(body->v, damping), cpvmult(cpvadd(gravity, cpvmult(body->f, body->m_inv)), dt));

    body->v = cpvclamp(cpvadd(cpvmult(body->v, damping), cpvmult(cpvadd(gravity, cpvmult(body->f, body->m_inv)), dt)), body->v_limit);

    cpFloat w_limit = body->w_limit;
    body->w = cpfclamp(body->w*damping + body->t*body->i_inv*dt, -w_limit, w_limit);*/

    cpBodySanityCheck(body);
}
