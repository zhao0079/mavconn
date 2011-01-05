#ifndef PXTRANSFORM_H_
#define PXTRANSFORM_H_

#include <cv.h>
#include <string>

class PxTransform
{
public:
  PxTransform();
  PxTransform(const PxTransform& t);
  PxTransform(const double* data);

  void identity(void);

  void rotateX(double theta);
  void rotateY(double theta);
  void rotateZ(double theta);
  void setRotation(double x, double y, double z);
  void getRotation(double &x, double &y, double &z);

  void translate(double x, double y, double z);
  void getTranslation(double &x, double &y, double &z);

  void leftMultiply(const PxTransform &t);
  void transformPoint(double &x, double &y, double &z);

  void copy(PxTransform &dest);
  std::string toString(void);
  cv::Mat toCvMat(void);

private:
  double matrix[4][4];
};

#endif
