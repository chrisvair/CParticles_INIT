#include "Context.h"
#include "Vec2.h"
#include "Constraint.h"
#include <stdio.h>

int isInGroundSphere(Context* context, float x, float y, float radius){
  Vec2 position = create_vec2(x, y);
  for(int j=0; j<context->num_ground_sphere; j++){
    Vec2 c = context->ground_spheres[j].center;
    float R = context->ground_spheres[j].radius;
    float sdf = norm(vect_sub(position, c)) - R;
    if(sdf < radius){
      return 1;
    }
  }
  return 0;
}

void checkContactWithPlane(Context* context, int particle_id, PlanCollider* collider){
    Particle *p = &context->particles[particle_id];
    Vec2 n = create_vec2(0, 1);
    Vec2 pi = (p->next_pos);
    Vec2 pc = (collider->start);
    float r = p->radius;
    if ((scalar_product(vect_sub(pi, pc),n)) < r){
        Vec2 qc = vect_sub(pi, scalar_mult(vect_sub(pi, pc),scalar_product(n,n)));
        float C =  scalar_product(vect_sub(pi, qc),n) - r;
        Vec2 delta = scalar_mult(n, -C);
        p->next_pos = vect_sum(p->position, delta);
    }
}

void checkContactWithSphere(Context* context, int particle_id, SphereCollider* collider){
    Particle *p = &context->particles[particle_id];
    Vec2 pi = p->next_pos;
    float r = p->radius;
    Vec2 c = collider->center;
    float R = collider->radius;
    float sdf = norm(vect_sub(pi, c))- R;
    if ( sdf < r){
      Vec2 n = normalisation(vect_sub(pi, c));
      Vec2 pc = vect_sub(p->next_pos, scalar_mult(n, sdf));
      Vec2 qc = vect_sub(pi, scalar_mult(vect_sub(pi, pc),scalar_product(n,n)));
      float C =  scalar_product(vect_sub(pi, qc),n) - r;
      Vec2 delta = scalar_mult(n, -C);
      p->next_pos = vect_sum(p->position, delta);
    }
}
