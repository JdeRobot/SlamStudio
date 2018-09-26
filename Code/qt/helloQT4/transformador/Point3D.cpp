#include "Point3D.h"
#include <iostream>
using namespace std;

// Constructor - The default values are specified in the declaration
//Point3D::Point3D(double x, double y , double z) : x(x), y(y) ,z(z) { }
Point3D::Point3D() {
	//cout << "constructor por defecot "<<"\n";
}
/*Point3D::Point3D(double x, double y , double z) {
	this-> x=x;
	this->y=y;
	this->z=z;
}*/

// Getters
double Point3D::getX() const { return x; }
double Point3D::getY() const { return y; }
double Point3D::getZ() const { return z; }

// Setters
void Point3D::setX(double x) { this->x = x; }
void Point3D::setY(double y) { this->y = y; }
void Point3D::setZ(double z) { this->z = z; }

// Public Functions
void Point3D::setXYZ(double x, double y , double z) { this->x = x; this->y = y; this->z = z; }

void Point3D::print() const {
   cout << "Point @ (" << x << "," << y << "," << z <<")";
}
