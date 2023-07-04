#ifndef CONSTRAINT_H_
#define CONSTRAINT_H_

// ------------------------------------------------
typedef struct Context Context;

typedef struct SphereCollider {
  Vec2 center;
  float radius;
} SphereCollider;

typedef struct PlanCollider {
  Vec2 start;
  Vec2 end;
  Vec2 normal;
} PlanCollider;

int isInGroundSphere(Context* context, float x, float y, float radius);
void checkContactWithPlane(Context* context, int particle_id, PlanCollider* collider);
void checkContactWithSphere(Context* context, int particle_id, SphereCollider* collider);

//void enforceStaticGroundConstraint(GroundConstraint* constraint, Particle* particle);
// ------------------------------------------------

#endif