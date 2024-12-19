/*Copyright (c) 2020, University of Maryland, Baltimore County. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the University of Maryland, Baltimore County.
Neither the name of the University of Maryland, Baltimore County nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY University of Maryland, Baltimore County AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL University of Maryland, Baltimore County BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef rigidbody_h
#define rigidbody_h
#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Dense>

struct Particle {
  Eigen::Vector3d pos;
};

struct RigidBody {
  double mass;
  Eigen::Matrix3d I0;
  Eigen::Vector3d pos, P, frc;
  Eigen::Quaterniond q;
  Eigen::Vector3d L, trq;
};

class Tri {
public:
  int indices[3];
  inline int &operator[](const unsigned int &i) { return indices[i];};
  inline int operator[](const unsigned int &i) const { return indices[i];};
};

struct SimulationParameters {
  double dt, total_time, cor;
  std::string output_fname;
};

struct Object {
  RigidBody rb;
  std::vector<Particle> particles;
  std::vector<Tri> triangles;
};


// I/O
#include "json/json.h"
#include <fstream>

bool readObject(const char *fname, Object &object);
bool readInputFile(const char *fname, SimulationParameters &params, std::vector<Object> &objects);
void writeObj(char *fname, const Object &obj);

#endif
