#include "Vec2.h"
#include <stdint.h>
#include "Context.h"
#include "Particle.h"

void checkParticleCollisions(Context* context, int particle_id_i, int particle_id_j){
    Particle *pi = &context->particles[particle_id_i];
    Particle *pj = &context->particles[particle_id_j];
    Vec2 posi = pi->next_pos;
    Vec2 posj = pj->next_pos;
    float ri = pi->radius;
    float rj = pj->radius;
    float mi = pi->inv_mass;
    float mj = pj->inv_mass;
    Vec2 xji = vect_sub(posi, posj);
    float C = norm(xji) - (ri+rj);
    if ( C < 0){
        float sigma_i = 2*C*(mi/(mi+mj));
        Vec2 delta_i = scalar_mult(normalisation(xji), -sigma_i);
        float sigma_j = 2*C*(mj/(mi+mj));
        Vec2 delta_j = scalar_mult(normalisation(xji), sigma_j);
        pi->next_pos = vect_sum(pi->position, delta_i);
        pj->next_pos = vect_sum(pj->position, delta_j);
    }
}
