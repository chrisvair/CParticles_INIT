#ifndef PARTICLE_H_
#define PARTICLE_H_

typedef struct Context Context;

// ------------------------------------------------

typedef struct Particle {
  Vec2 position;   // current position of the particle
  Vec2 next_pos;   // only used during update loop
  Vec2 velocity;   // current velocity of the particle
  float inv_mass;  // 1/mass of the particle
  float radius;    // radius of the particle
  int solid_id;    // all particles associated to the same solid should have the same id
  int draw_id;     // id used to identify drawing element associated to the sphere
  //uint32_t status; // can be used as a bitfield to store additional information
} Particle;

// ------------------------------------------------

//PARTICLES COLLISIONS CHANGE FILE LATER
void checkSphereCollisions(Context* context, int particle_id_1, int particle_id_2);
//void enforceSphereCollisions(Particle* particle1, Particle* particle1);

#endif
