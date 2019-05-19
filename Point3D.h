#ifndef POINT3D_H
#define POINT3D_H
class Point3D {
private:
   double x, y, z;  // Private data members

public:
   Point3D();

   //Point3D( double x = 0, double y = 0 , double z=0); // Constructor with default arguments
   double getX() const;  // Getter
   void setX(double x);  // Setter
   double getY() const;
   void setY(double y);
   double getZ() const;
   void setZ(double z);
   void setXYZ(double x, double y , double z);
   void print() const;
};
#endif
