#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

// Mass(Vector2D position, float mass, bool pinned);
//   Spring(Mass *a, Mass *b, float k)


namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.

//        Comment-in this part when you implement the constructor

        Vector2D step = (end - start) / num_nodes;
        // Mass* start = new Mass(start, node_mass, true); 

        // masses.emplace_back(start);

        for(int i = 0; i < num_nodes; ++i)
        {
            Vector2D pos = start + step * i;
            // std::cout << pos << std::endl;
            Mass* mass = new Mass(pos, node_mass, false);
            masses.push_back(mass);
        }

        for(int i = 1; i < num_nodes; ++i)
        {
            Spring* spring = new Spring(masses[i-1], masses[i], k);
            springs.push_back(spring);
        }   

       for (auto &i : pinned_nodes) {
           masses[i]->pinned = true;
       }
        // for (auto &s : springs)
        // {
        //     // forces
        //     // TODO (Part 2): Use Hooke's law to calculate the force on a node
        //     // std::cout << (s->m2->position - s->m1->position) << std::endl;
        //     std::cout << s->m2->position << std::endl;

        // }


    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // forces
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            // std::cout << (s->m2->position - s->m1->position) << std::endl;
            Vector2D force = 1.5 * s->k * 0.01 * (s->m2->position - s->m1->position).unit() * ((s->m2->position - s->m1->position).norm() - s->rest_length);
            s->m1->forces += force;
            s->m2->forces -= force;

            Vector2D velocityDiff = s->m2->velocity - s->m1->velocity;
            Vector2D posHat = (s->m2->position - s->m1->position).unit();
            Vector2D dampingForce = posHat * 0.05 * dot(velocityDiff, posHat);

            Vector2D velocityT = posHat * dot(velocityDiff, posHat) - (s->m1->velocity - s->m2->velocity); 


            s->m1->forces -= dampingForce;
            s->m1->forces += velocityT * 0.05;
            s->m2->forces += dampingForce;
            s->m2->forces -= velocityT * 0.05;

        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                auto force = m->forces + gravity;
                // // std::cout << m->forces << std::endl;
                auto acc = force / m->mass;
                // // std::cout << a << std::endl;
                auto v_t = m->velocity;
                m->velocity = v_t + acc * delta_t;
                m->position = m->position + v_t * delta_t;

                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position


                // TODO (Part 2): Add global damping
                // m->velocity += m->velocity * -0.002;

                // m->velocity = m->velocity * 0.99;
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {

            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            Vector2D dir = s->m2->position - s->m1->position;
            float diff = dir.norm();
            float dist = diff - s->rest_length;
            Vector2D UnitV = dir.unit();

            if (!s->m1->pinned)
            {
                s->m1->position += 0.5 * dist * UnitV;
            }

            if(!s->m2->pinned)
            {
                s->m2->position -= 0.5 * dist * UnitV;
            }

        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D acc = gravity / m->mass;
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                Vector2D new_position = temp_position + (1-0.00005) * (temp_position - m->last_position) + acc * delta_t * delta_t;

                m->position = new_position;
                m->last_position = temp_position;
                
                // TODO (Part 4): Add global Verlet damping
            }
        }
    }

      Rope::~Rope()
      {
          for (auto& mass : masses)
          {
              delete mass;
          }

          for(auto& spring: springs)
          {
              delete spring;
          }

          std::cout << "~Rope()" << std::endl;
      }
}
