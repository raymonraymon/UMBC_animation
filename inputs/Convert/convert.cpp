#include <iostream>
#include <fstream>
#include <vector>
#include "slVector.H"
//#include "slUtil.H"
#include <unordered_map>

using std::cout;
using std::cerr;
using std::endl;

static const double FEPS = 1e-6;

class SimpleTet {
public:
  int indices[4];
  inline int &operator[](const unsigned int &i) { return indices[i];};
  inline int operator[](const unsigned int &i) const { return indices[i];};
  double vol;
};

class SimpleNode {
public:
	SlVector3 pos;
	double m;
	SimpleNode() : pos(0.0) {};
};


class SimpleTri {
public:
  int indices[3];
  inline int &operator[](const unsigned int &i) { return indices[i];};
  inline int operator[](const unsigned int &i) const { return indices[i];};
  inline void set(int x, int y, int z) {indices[0] = x; indices[1] = y; indices[2] = z;};
  inline SimpleTri(int x, int y, int z) {indices[0] = x; indices[1] = y; indices[2] = z;};
  inline SimpleTri() {};
};

class SimpleEdge {
public:
  int indices[2];
  inline int &operator[](const unsigned int &i) { return indices[i];};
  inline int operator[](const unsigned int &i) const { return indices[i];};
  inline SimpleEdge(int a, int b) {indices[0] = a; indices[1] = b;};
  SimpleEdge(){};
};

struct eqSimpleTri {
  bool operator()(const SimpleTri &t1, const SimpleTri &t2) const {
    return ((t1[0] == t2[0] && t1[1] == t2[1] && t1[2] == t2[2]) ||
	    (t1[1] == t2[0] && t1[0] == t2[1] && t1[2] == t2[2]) ||
	    (t1[2] == t2[0] && t1[1] == t2[1] && t1[0] == t2[2]) ||
	    (t1[0] == t2[0] && t1[2] == t2[1] && t1[1] == t2[2]) ||
	    (t1[1] == t2[0] && t1[2] == t2[1] && t1[0] == t2[2]) ||
	    (t1[2] == t2[0] && t1[0] == t2[1] && t1[1] == t2[2]));
  }
};

struct hashSimpleTri {
  size_t operator()(const SimpleTri& t) const {
    return (t[0] ^ t[1] ^ t[2]);
  }
};

typedef std::unordered_map<SimpleTri, SimpleEdge, hashSimpleTri, eqSimpleTri> TriToTetMap;

inline void updateTriToTetMap(TriToTetMap &triToTetMap, int n, int x, int y, int z) {
  SimpleTri tri(x, y, z);
  if (triToTetMap.count(tri) == 0) {
    SimpleEdge e(n, -1);
    triToTetMap.insert(TriToTetMap::value_type(tri,e));
  } else {
    triToTetMap.find(tri)->second[1] = n;
  }
}

bool readInputFile(const char *fname, std::vector<SimpleNode> &nodes,
	std::vector<SimpleTet> &tets) {
  char ch;
  SimpleNode n;
  SimpleTet t;
  
  SlVector3 bbmin = 1000000000.0;
  SlVector3 bbmax = -1000000000.0;
  

  std::ifstream in(fname, std::ios::in);
  while (in>>ch) {
    if (ch == 'v') {
	  in>>n.pos[0]>>n.pos[1]>>n.pos[2];
	  nodes.push_back(n);
	  bbmin[0] = std::min<double>(bbmin[0], n.pos[0]);
	  bbmin[1] = std::min<double>(bbmin[1], n.pos[1]);
	  bbmin[2] = std::min<double>(bbmin[2], n.pos[2]);
	  bbmax[0] = std::max<double>(bbmax[0], n.pos[0]);
	  bbmax[1] = std::max<double>(bbmax[1], n.pos[1]);
	  bbmax[2] = std::max<double>(bbmax[2], n.pos[2]);
      continue;
    }
    if (ch == 't') {
	  in>>t[0]>>t[1]>>t[2]>>t[3];
	  tets.push_back(t);
      continue;
    }
  }
  std::cout<<bbmin<<" x "<<bbmax<<std::endl;
  SlVector3 center = 0.5*(bbmax+bbmin);

  bbmin = 1000000000.0;
  bbmax = -1000000000.0;

  for (unsigned int i=0; i<nodes.size(); i++) {
	SimpleNode &n = nodes[i];
	n.pos -= center;
	n.pos[2] += 5;
	bbmin[0] = std::min<double>(bbmin[0], n.pos[0]);
	bbmin[1] = std::min<double>(bbmin[1], n.pos[1]);
	bbmin[2] = std::min<double>(bbmin[2], n.pos[2]);
	bbmax[0] = std::max<double>(bbmax[0], n.pos[0]);
	bbmax[1] = std::max<double>(bbmax[1], n.pos[1]);
	bbmax[2] = std::max<double>(bbmax[2], n.pos[2]);
  }
  std::cout<<bbmin<<" x "<<bbmax<<std::endl;

  in.close();
  return true;
}

int main(int argc, char *argv[]) {
  double dens = 1000;
  std::vector<SimpleNode> nodes;
  std::vector<SimpleTet> tets;
  std::vector<SimpleTri> tris;

  TriToTetMap triToTetMap;
  int i;
  readInputFile(argv[1], nodes, tets);
  std::vector<SimpleTet>::iterator t;
  std::vector<SimpleNode>::iterator n;
  
  for (t=tets.begin(), i=0; t!=tets.end(); t++, i++) {
	updateTriToTetMap(triToTetMap, i, (*t)[0], (*t)[2], (*t)[1]);
	updateTriToTetMap(triToTetMap, i, (*t)[3], (*t)[0], (*t)[1]);
	updateTriToTetMap(triToTetMap, i, (*t)[3], (*t)[2], (*t)[0]);
	updateTriToTetMap(triToTetMap, i, (*t)[2], (*t)[3], (*t)[1]);
  }
  
  for (TriToTetMap::const_iterator m=triToTetMap.begin(); m!=triToTetMap.end(); m++) {
	if (m->second[1] == -1) {
	  tris.push_back(m->first);
	}
  }
  
  std::ofstream out(argv[2], std::ios::out);
  
  for (unsigned int i=0; i<nodes.size(); i++) {
	out<<"p "<<nodes[i].pos[0]<<" "<<nodes[i].pos[1]<<" "<<nodes[i].pos[2]<<" 0.0 0.0 0.0 "<<std::endl;
  }
  
  for (t=tets.begin(); t!=tets.end(); t++) {
    out<<"e "<<(*t)[0]<<" "<<(*t)[1]<<" "<<(*t)[2]<<" "<<(*t)[3]<<" "<<std::endl;
  }	
  
  for (unsigned int i=0; i<tris.size(); i++) {
	out<<"t "<<tris[i][0]<<" "<<tris[i][1]<<" "<<tris[i][2]<<std::endl;
  }
  
  return 0;
};
