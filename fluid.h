/*Copyright (c) 2020, University of Maryland, Baltimore County. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the University of Maryland, Baltimore County.
Neither the name of the University of Maryland, Baltimore County nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY University of Maryland, Baltimore County AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL University of Maryland, Baltimore County BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#ifndef FLUID_H
#define FLUID_H

#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include "array3D.h"

struct SimulationParameters {
  double dt, total_time, density, flip_ratio, h;
  int nx, ny, nz;
  Eigen::Vector3d lc, uc;
  std::string output_fname;
};

struct Particle {
  Eigen::Vector3d pos, vel;
};

class StaggeredGrid {
public:
  StaggeredGrid(const SimulationParameters &params);
  
  Eigen::Vector3d interpVel(const Eigen::Vector3d &x) const;
  double interpxface(const Eigen::Vector3d &x, const Array3D<double> &f) const;
  double interpyface(const Eigen::Vector3d &x, const Array3D<double> &f) const;
  double interpzface(const Eigen::Vector3d &x, const Array3D<double> &f) const;
  
  void advectParticles(double dt, std::vector<Particle> &particles);
  void particlesToGrid(const std::vector<Particle> &particles);
  void gridToParticles(double flip_ration, std::vector<Particle> &particles);

  void setBoundaryVelocity();
  void computePressure();
  
  void gravity(double dt);

  static const unsigned char AIR = 1;
  static const unsigned char LIQUID = 2;
  static const unsigned char OBSTACLE = 4;
  
private:
  Array3D<double> u,v,w;
  Array3D<unsigned char> cellLabels;
  unsigned int nx, ny, nz, nynz;
  double h,halfh;
  Eigen::Vector3d lc, uc;
  
  void splatParticleValue(Array3D<double> &u, Array3D<double> &fu,
      double w0, double w1, double w2,
      int i, int j, int k, double v);
  void applyA(Array3D<double> &x, Array3D<double> &b);
  inline void clipToGrid(Eigen::Vector3d &x) const;
  inline bool boundary(unsigned int i, unsigned int j, unsigned int k) const;
  Array3D<double> nu,nv,nw,fu,fv,fw;
  Array3D<double> p, r, d, q;
  Array3D<unsigned short> laplacian;
};


// I/O

void readParticles(const char *fname, std::vector<Particle> &particles);
void writeParticles(const char *fname, const std::vector<Particle> &particles);
void readInputFile(const char *fname, SimulationParameters &params, std::vector<Particle> &particles);

  
#endif
