#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Cloth::Cloth(double width, double height, int num_width_points,
             int num_height_points, float thickness) {
  this->width = width;
  this->height = height;
  this->num_width_points = num_width_points;
  this->num_height_points = num_height_points;
  this->thickness = thickness;

  buildGrid();
  buildClothMesh();
}

Cloth::~Cloth() {
  point_masses.clear();
  springs.clear();

  if (clothMesh) {
    delete clothMesh;
  }
}

void Cloth::buildGrid() {
  // TODO (Part 1): Build a grid of masses and springs.
    point_masses.reserve(num_width_points * num_height_points);
    double unit_width = width / num_width_points;
    double unit_height = height / num_height_points;
        for (int j = 0; j < num_height_points; j++) {
          for (int i = 0; i < num_width_points; i++) {
            int p = j * num_width_points + i;
            if(orientation == HORIZONTAL) {
                point_masses[p].position.x = i * unit_width;
                point_masses[p].position.y = 1.;
                point_masses[p].position.z = j * unit_height;
            }
            else if(orientation == VERTICAL) {
                point_masses[p].position.x = i * unit_width;
                point_masses[p].position.y = j * unit_height;
                point_masses[p].position.z = rand() / (double)RAND_MAX * 1/500 - 1/1000;
            }

            std::vector<int> v = {i, j};
            std::vector<vector<int>>::iterator itr = std::find(pinned.begin(), pinned.end(), v);
            if(itr != pinned.end()) {
                PointMass pointmass = PointMass(point_masses[p].position, true);
                point_masses.emplace_back(pointmass);
            } else {
                PointMass pointmass = PointMass(point_masses[p].position, false);
                point_masses.emplace_back(pointmass);
            }
        }
    }

    //spring
    for(int i = 0; i < num_width_points; i++) {
        for(int j = 0; j < num_height_points; j++) {
            int p = j * num_width_points + i;
            
            int str_x1 = i - 1, str_y1 = j; // left
            int str_x2 = i, str_y2 = j - 1; // above
            
            int shear_x1 = i - 1, shear_y1 = j - 1; // upper left
            int shear_x2 = i + 1, shear_y2 = j - 1; // upper right
            
            int bend_x1 = i - 2, bend_y1 = j; // two away left
            int bend_x2 = i, bend_y2 = j - 2; // two away above
            
            if(str_x1 >= 0 && str_x1 < num_width_points && str_y1 >= 0 && str_y1 < num_height_points) {
                Spring str_1 = Spring(&point_masses[str_y1 * num_width_points + str_x1], &point_masses[p], STRUCTURAL);
                springs.emplace_back(str_1);
            }
            if(str_x2 >= 0 && str_x2 < num_width_points && str_y2 >= 0 && str_y2 < num_height_points) {
                Spring str_2 = Spring(&point_masses[str_y2 * num_width_points + str_x2], &point_masses[p], STRUCTURAL);
                springs.emplace_back(str_2);
            }
            
            if(shear_x1 >= 0 && shear_x1 < num_width_points && shear_y1 >= 0 && shear_y1 < num_height_points) {
                Spring shear_1 = Spring(&point_masses[shear_y1 * num_width_points + shear_x1], &point_masses[p], SHEARING);
                springs.emplace_back(shear_1);
            }
            if(shear_x2 >= 0 && shear_x2 < num_width_points && shear_y2 >= 0 && shear_y2 < num_height_points) {
                Spring shear_2 = Spring(&point_masses[shear_y2 * num_width_points + shear_x2], &point_masses[p], SHEARING);
                springs.emplace_back(shear_2);
            }
            
            if(bend_x1 >= 0 && bend_x1 < num_width_points && bend_y1 >= 0 && bend_y1 < num_height_points) {
                Spring bend_1 = Spring(&point_masses[bend_y1 * num_width_points + bend_x1], &point_masses[p], BENDING);
                springs.emplace_back(bend_1);
            }
            if(bend_x2 >= 0 && bend_x2 < num_width_points && bend_y2 >= 0 && bend_y2 < num_height_points) {
                Spring bend_2 = Spring(&point_masses[bend_y2 * num_width_points + bend_x2], &point_masses[p], BENDING);
                springs.emplace_back(bend_2);
            }
        }
    }
}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
  double mass = width * height * cp->density / num_width_points / num_height_points;
  double delta_t = 1.0f / frames_per_sec / simulation_steps;

  // TODO (Part 2): Compute total force acting on each point mass.
    Vector3D total_a;
    for(Vector3D a : external_accelerations) {
        total_a += a;
    }
    Vector3D total_f = total_a * mass;
    for(int i = 0; i < num_width_points; i++) {
        for(int j = 0; j < num_height_points; j++) {
            int p = j * num_width_points + i;
            point_masses[p].forces = total_f;
        }
    }
//    //spring correction forces
    for(Spring s: springs) {
        if(!cp->enable_structural_constraints && !cp->enable_bending_constraints && !cp->enable_shearing_constraints) {
            continue;
        } else if ((cp->enable_structural_constraints && s.spring_type == STRUCTURAL) ||
                   (cp->enable_bending_constraints && s.spring_type == BENDING) ||
                   (cp->enable_shearing_constraints && s.spring_type == SHEARING)) {
            Vector3D pa_pb = s.pm_a->position - s.pm_b->position;
            double ab_dis = pa_pb.norm();
            if(s.spring_type == BENDING) {
                Vector3D spring_force = 0.2 * cp->ks * (ab_dis - s.rest_length) * pa_pb.unit();
                s.pm_a->forces += -spring_force;
                s.pm_b->forces += spring_force;
            }
            Vector3D spring_force = cp->ks * (ab_dis - s.rest_length) * pa_pb.unit();
            s.pm_a->forces += -spring_force;
            s.pm_b->forces += spring_force;
        }
    }
    

  // TODO (Part 2): Use Verlet integration to compute new point mass positions
    double d = cp->damping / 100.;
    for (PointMass &point_mass: point_masses) {
        if(!point_mass.pinned) {
            Vector3D a = point_mass.forces / mass;
            Vector3D xt_next = point_mass.position + (1. - d) * (point_mass.position - point_mass.last_position) + a * delta_t * delta_t;
            point_mass.last_position = point_mass.position;
            point_mass.position = xt_next;

        }
    }



  // TODO (Part 4): Handle self-collisions.
    build_spatial_map();
    for(PointMass &point_mass: point_masses) {
        self_collide(point_mass, simulation_steps);
    }

  // TODO (Part 3): Handle collisions with other primitives.
    for(PointMass &point_mass: point_masses) {
        for(CollisionObject *co : *collision_objects) {
            co->collide(point_mass);
        }
    }


  // TODO (Part 2): Constrain the changes to be such that the spring does not change
  // in length more than 10% per timestep [Provot 1995].
    for(Spring &s: springs) {
        Vector3D pa_pb = s.pm_a->position - s.pm_b->position; //b->a
        Vector3D half_pa_pb = pa_pb / 2.;
        double full_dis = pa_pb.norm() - 1.1 * s.rest_length;
        if(full_dis > 0){
            if(!s.pm_a->pinned && !s.pm_b->pinned) { // half correction
                double half_dis = full_dis / 2.;
                Vector3D a_cor = half_pa_pb.unit() * half_dis;
                Vector3D b_cor = -half_pa_pb.unit() * half_dis;
                s.pm_a->position -= a_cor;
                s.pm_b->position -= b_cor;
            }
            if(!s.pm_a->pinned && s.pm_b->pinned) { // a full correction
                Vector3D full_cor = pa_pb.unit() * full_dis;
                s.pm_a->position -= full_cor;
            }
            if(s.pm_a->pinned && !s.pm_b->pinned) { // b full correction
                Vector3D full_cor = -pa_pb.unit() * full_dis;
             s.pm_b->position -= full_cor;
            }
        }
    }
    
}

void Cloth::build_spatial_map() {
    for (const auto &entry : map) {
        delete(entry.second);
    }
    map.clear();

  // TODO (Part 4): Build a spatial map out of all of the point masses.
    for(PointMass &point_mass : point_masses) {
        float hashkey = hash_position(point_mass.position);
        unordered_map<float, vector<PointMass *> *> :: iterator value;
        value = map.find(hashkey);
        if(value == map.end()) {
            map[hashkey] = new vector<PointMass *>();
        }
        map[hashkey]->push_back(&point_mass);
    }
}

void Cloth::self_collide(PointMass &pm, double simulation_steps) {
//   TODO (Part 4): Handle self-collision for a given point mass.
    float hashkey = hash_position(pm.position);
    Vector3D final_correction;
    int count = 0.;
    unordered_map<float, vector<PointMass *> *> :: iterator value;
    value = map.find(hashkey);
    if(value != map.end()) {
        for(PointMass *can_pm : *value->second) {
            if(&pm != can_pm) {
                Vector3D dir = can_pm->position - pm.position;
                if (dir.norm() < 2 * thickness) {
                    double dis = 2 * thickness - dir.norm();
                    Vector3D correction = dis * dir.unit();
                    final_correction += correction;
                    count++;
                    }
                }
            }
        }
    if(count) {
        final_correction = final_correction / count / simulation_steps;
        pm.position -= final_correction;
    }
}

float Cloth::hash_position(Vector3D pos) {
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents membership in some 3D box volume.
    float w = 3. * width / num_width_points;
    float h = 3. * height / num_height_points;
    float t = max(w, h);

    float x_uni = floor(pos.x / w);
    float y_uni = floor(pos.y / h);
    float z_uni = floor(pos.z / t);

    float key = x_uni * 31 * 31 + y_uni * 31 + z_uni;
    return key;
}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Cloth::reset() {
  PointMass *pm = &point_masses[0];
  for (int i = 0; i < point_masses.size(); i++) {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
    pm++;
  }
}

void Cloth::buildClothMesh() {
  if (point_masses.size() == 0) return;

  ClothMesh *clothMesh = new ClothMesh();
  vector<Triangle *> triangles;

  // Create vector of triangles
  for (int y = 0; y < num_height_points - 1; y++) {
    for (int x = 0; x < num_width_points - 1; x++) {
      PointMass *pm = &point_masses[y * num_width_points + x];
      // Get neighboring point masses:
      /*                      *
       * pm_A -------- pm_B   *
       *             /        *
       *  |         /   |     *
       *  |        /    |     *
       *  |       /     |     *
       *  |      /      |     *
       *  |     /       |     *
       *  |    /        |     *
       *      /               *
       * pm_C -------- pm_D   *
       *                      *
       */
      
      float u_min = x;
      u_min /= num_width_points - 1;
      float u_max = x + 1;
      u_max /= num_width_points - 1;
      float v_min = y;
      v_min /= num_height_points - 1;
      float v_max = y + 1;
      v_max /= num_height_points - 1;
      
      PointMass *pm_A = pm                       ;
      PointMass *pm_B = pm                    + 1;
      PointMass *pm_C = pm + num_width_points    ;
      PointMass *pm_D = pm + num_width_points + 1;
      
      Vector3D uv_A = Vector3D(u_min, v_min, 0);
      Vector3D uv_B = Vector3D(u_max, v_min, 0);
      Vector3D uv_C = Vector3D(u_min, v_max, 0);
      Vector3D uv_D = Vector3D(u_max, v_max, 0);
      
      
      // Both triangles defined by vertices in counter-clockwise orientation
      triangles.push_back(new Triangle(pm_A, pm_C, pm_B, 
                                       uv_A, uv_C, uv_B));
      triangles.push_back(new Triangle(pm_B, pm_C, pm_D, 
                                       uv_B, uv_C, uv_D));
    }
  }

  // For each triangle in row-order, create 3 edges and 3 internal halfedges
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    // Allocate new halfedges on heap
    Halfedge *h1 = new Halfedge();
    Halfedge *h2 = new Halfedge();
    Halfedge *h3 = new Halfedge();

    // Allocate new edges on heap
    Edge *e1 = new Edge();
    Edge *e2 = new Edge();
    Edge *e3 = new Edge();

    // Assign a halfedge pointer to the triangle
    t->halfedge = h1;

    // Assign halfedge pointers to point masses
    t->pm1->halfedge = h1;
    t->pm2->halfedge = h2;
    t->pm3->halfedge = h3;

    // Update all halfedge pointers
    h1->edge = e1;
    h1->next = h2;
    h1->pm = t->pm1;
    h1->triangle = t;

    h2->edge = e2;
    h2->next = h3;
    h2->pm = t->pm2;
    h2->triangle = t;

    h3->edge = e3;
    h3->next = h1;
    h3->pm = t->pm3;
    h3->triangle = t;
  }

  // Go back through the cloth mesh and link triangles together using halfedge
  // twin pointers

  // Convenient variables for math
  int num_height_tris = (num_height_points - 1) * 2;
  int num_width_tris = (num_width_points - 1) * 2;

  bool topLeft = true;
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    if (topLeft) {
      // Get left triangle, if it exists
      if (i % num_width_tris != 0) { // Not a left-most triangle
        Triangle *temp = triangles[i - 1];
        t->pm1->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm1->halfedge->twin = nullptr;
      }

      // Get triangle above, if it exists
      if (i >= num_width_tris) { // Not a top-most triangle
        Triangle *temp = triangles[i - num_width_tris + 1];
        t->pm3->halfedge->twin = temp->pm2->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle to bottom right; guaranteed to exist
      Triangle *temp = triangles[i + 1];
      t->pm2->halfedge->twin = temp->pm1->halfedge;
    } else {
      // Get right triangle, if it exists
      if (i % num_width_tris != num_width_tris - 1) { // Not a right-most triangle
        Triangle *temp = triangles[i + 1];
        t->pm3->halfedge->twin = temp->pm1->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle below, if it exists
      if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f) { // Not a bottom-most triangle
        Triangle *temp = triangles[i + num_width_tris - 1];
        t->pm2->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm2->halfedge->twin = nullptr;
      }

      // Get triangle to top left; guaranteed to exist
      Triangle *temp = triangles[i - 1];
      t->pm1->halfedge->twin = temp->pm2->halfedge;
    }

    topLeft = !topLeft;
  }

  clothMesh->triangles = triangles;
  this->clothMesh = clothMesh;
}
