/*  This file can be compiled and executed separetly in order to visualise the coordinates of a specific particle. 
     It was used to debug the rebounds issues.
*/

#include <stdlib.h>
#include <stdio.h>
#include "Vec2.h"
#include "Particle.h"
#include "Constraint.h"
#include "Context.h"


void testRebound(){
    Context* local_context = initializeContext(5);
    addParticle(local_context, 0, 0, 0.2f, 1.0f, 1);
    for(int i=0; i<372; i++){
        updatePhysicalSystem(local_context, 0.016/6, 1);
        Particle* particule =local_context->particles;
        if(particule->position.y <-4.7){
            printf("The position of the particle at time %d is (%f,%f) \n",i , particule->position.x, particule->position.y);
    }
    }
}

void testFriction(){
    Context* local_context = initializeContext(5);
    addParticle(local_context, 0, -4.0f, 0.2f, 1.0f, 1);
    local_context->particles->velocity.x = 1;
    for(int i=0; i<160; i++){
        updatePhysicalSystem(local_context, 0.016/6, 1);
        Particle* particule =local_context->particles;
        if(particule->position.y <-4.7){
            printf("The position of the particle at time %d is (%f,%f) \n",i , particule->position.x, particule->position.y);
    }
    }
}

int main(){
    testRebound();
    testFriction();
    return 0;
}