#include "Context.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>

// ------------------------------------------------

Particle getParticle(Context* context, int id)
{
  return context->particles[id];
}

// ------------------------------------------------

void addParticle(Context* context, float x, float y, float radius, float mass, int draw_id)
{
    assert(context->num_particles<context->capacity_particles); // currently no resize in context
    context->particles[context->num_particles].position.x = x;
    context->particles[context->num_particles].position.y = y;
    context->particles[context->num_particles].velocity.x = 0.0F;
    context->particles[context->num_particles].velocity.y = 0.0F;
    context->particles[context->num_particles].inv_mass = 1.0F/mass;
    context->particles[context->num_particles].radius = radius;
    context->particles[context->num_particles].draw_id = draw_id;
    context->num_particles += 1;
}

// ------------------------------------------------

void setDrawId(Context* context, int sphere_id, int draw_id)
{
  context->particles[sphere_id].draw_id = draw_id;
}

// ------------------------------------------------

SphereCollider getGroundSphereCollider(Context* context, int id)
{
  return context->ground_spheres[id];
  // *(c->g_s+i)
}

PlanCollider getPlanCollider(Context* context)
{
  return *(context->plan);
}

// ------------------------------------------------

Context* initializeContext(int capacity)
{
  Context* context = malloc(sizeof(Context));
  context->num_particles = 0;
  context->capacity_particles = capacity;
  context->particles = malloc(capacity*sizeof(Particle));
  memset(context->particles,0,capacity*sizeof(Particle));

  //Ground Sphere
  context->num_ground_sphere = 4;
  context->ground_spheres = malloc(4*sizeof(SphereCollider));
  Vec2 p0 = {3.0f, 5.0f};
  Vec2 p1 = {-3.0f, 5.0f};
  Vec2 p2 = {7.0f, -3.0f};
  Vec2 p3 = {-7.0f, -3.0f};
  context->ground_spheres[0].center = p0;
  context->ground_spheres[1].center = p1;
  context->ground_spheres[2].center = p2;
  context->ground_spheres[3].center = p3;
  context->ground_spheres[0].radius = 1;
  context->ground_spheres[1].radius = 1;
  context->ground_spheres[2].radius = 1.7;
  context->ground_spheres[3].radius = 1.7;
  //for(int i; i<=4; i++){
  //  context->ground_spheres[i].radius = 1.7;
  //}

  //Plan
  context->plan = malloc(sizeof(PlanCollider));
  context->plan->start.x = -10.0f;
  context->plan->start.y = -5.0f;
  context->plan->end.x = 10.0f;
  context->plan->end.y = -5.0f;

  return context;
}

// ------------------------------------------------

void updatePhysicalSystem(Context* context, float dt, int num_constraint_relaxation)
{
  applyExternalForce(context, dt);
  dampVelocities(context);
  updateExpectedPosition(context, dt);
  addDynamicContactConstraints(context);
  addStaticContactConstraints(context);
 
  for(int k=0; k<num_constraint_relaxation; ++k) {
    projectConstraints(context);
  }

  updateVelocityAndPosition(context, dt);
  applyFriction(context);

  deleteContactConstraints(context);
}

// ------------------------------------------------

void applyExternalForce(Context* context, float dt)
{
  if (context->num_particles == 0) return;
  Particle *p = context->particles;
  for (int i = 0; i<context->num_particles; i++){
    (p+i)->velocity.y += - dt*(p+i)->inv_mass * 9.81;
  }
}

void dampVelocities(Context* context)
{
}

void updateExpectedPosition(Context* context, float dt)
{
  if (context->num_particles == 0) return;
  Particle *p = context->particles;
  for (int i = 0; i < context->num_particles; i++){
    (p+i)->next_pos = vect_sum((p+i)->position, scalar_mult((p+i)->velocity, dt));
  }
}

void addDynamicContactConstraints(Context* context)
{
}

void addStaticContactConstraints(Context* context)
{
  if (context->num_particles == 0) return;
  for (int i = 0; i<context->num_particles; i++){
    checkContactWithPlane(context, i , context->plan);
    for(int j=0; j<context->num_ground_sphere; j++){
      checkContactWithSphere(context, i, &context->ground_spheres[j]);
    }
  }
}

void projectConstraints(Context* context)
{
}

void updateVelocityAndPosition(Context* context, float dt)
{
  if (context->num_particles==0) return;
  Particle *p = context->particles;
  for (int i = 0; i<context->num_particles; i++){
    (p+i)->velocity.y = (1.0/dt) * ((p+i)->next_pos.y - (p+i)->position.y);
    (p+i)->position.y = (p+i)->next_pos.y;
    (p+i)->velocity.x = (1.0/dt) * ((p+i)->next_pos.x - (p+i)->position.x);
    (p+i)->position.x = (p+i)->next_pos.x;
    //(p+i)->velocity = *scalar_mult(vect_sub(&(p+i)->next_pos, &(p+i)->position), (1.0/dt));
    //(p+i)->position = (p+i)->next_pos;
  }
}

void applyFriction(Context* context)
{
}

void deleteContactConstraints(Context* context)
{
}

// ------------------------------------------------

/*
int main()
{
  Context* context = initializeContext(10);
  addParticle(context, 100, 5, 10, 1, 0);
  printf("%f\n", getParticle(context, 0).position.y);
  applyExternalForce(context,1);
  updateExpectedPosition(context,1);
  updateVelocityAndPosition(context, 1);
  printf("%f\n", getParticle(context,0).position.y);
  return 0;
}
*/
