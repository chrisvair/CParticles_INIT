#include "Context.h"
#include "Vec2.h"
#include "Constraint.h"
#include <stdio.h>

void checkContactWithPlane(Context* context, int particle_id, PlanCollider* collider){
    Particle *p = &context->particles[particle_id];
    Vec2 n = create_vec2(0, 1);
    Vec2 pi = (p->next_pos);
    Vec2 pc = (collider->start);
    float r = p->radius;
    if ((scalar_product(vect_sub(pi, pc),n)) < r){
        p-> radius = 4;
    }
}

void checkContactWithSphere(Context* context, int particle_id, SphereCollider* collider){
    Particle *p = &context->particles[particle_id];
    Vec2 pi = p->next_pos;
    float r = p->radius;
    Vec2 c = collider->center;
    float R = collider->radius;
    if (norm(vect_sub(pi, c))-R < r){
        p-> radius = 2;
    }
}
