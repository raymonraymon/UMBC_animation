#include <iostream>

int main(int argc, char *argv[]) {
  char ch;
  double a, b, c, d, e, f, g;
  
  while (std::cin>>ch) {
    if (ch == 'p') {
      std::cin>>a>>b>>c>>d>>e>>f>>g;
      std::cout<<"p "<<d<<" "<<a<<" "<<b<<" "<<c<<" "<<e<<" "<<f<<" "<<g<<std::endl;
    }
  }
}
