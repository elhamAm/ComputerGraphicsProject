
#ifndef __MICROHELPER_H__
#define __MICROHELPER_H__

#include <nori/object.h>
#include <nori/warp.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN


class MicroHelper {

public:
    static float eval(float alpha, const Vector3f &m);
    static float G(float alpha, const Vector3f &i, const Vector3f &o, const Vector3f &m);
    static float pdf(float alpha, const Vector3f &m);
    static Vector3f sample(float alpha, Point2f xi);
private:
    static float smithBeckmannG1(float alpha, const Vector3f &v, const Vector3f &m);
};

NORI_NAMESPACE_END
#endif