#ifndef CONSTRAINT_H_
#define CONSTRAINT_H_

// ------------------------------------------------
typedef struct Context Context ;

typedef struct SphereCollider {
  Vec2 center;
  float radius;
} SphereCollider;

typedef struct PlanCollider {
  Vec2 start;
  Vec2 end;
} PlanCollider;

void checkContactWithPlane(Context* context, int particle_id, PlanCollider* collider);
void checkContactWithSphere(Context* context, int particle_id, SphereCollider* collider);

// ------------------------------------------------

#endif