#include <nori/microHelper.h>

NORI_NAMESPACE_BEGIN

/// Smith's shadowing-masking function 
float MicroHelper::smithBeckmannG1(float alpha, const Vector3f &v, const Vector3f &m){
    float tanTheta = Frame::tanTheta(v);

    /* Perpendicular incidence -- no shadowing/masking */
    if (tanTheta == 0.0f)
        return 1.0f;

    /* Can't see the back side from the front and vice versa */
    if (m.dot(v) * Frame::cosTheta(v) <= 0)
        return 0.0f;

    float a = 1.0f / (alpha * tanTheta);
    if (a >= 1.6f)
        return 1.0f;
    float a2 = a * a;

    /* Use a fast and accurate (<0.35% rel. error) rational
    approximation to the shadowing-masking function */
    return (3.535f * a + 2.181f * a2) 
         / (1.0f + 2.276f * a + 2.577f * a2);
}
float MicroHelper::eval(float alpha, const Vector3f &m){
    if (m.z() <= 0.0f)
        return 0.0f;

    return Warp::squareToBeckmannPdf(m, alpha);
}
float MicroHelper::G(float alpha, const Vector3f &i, const Vector3f &o, const Vector3f &m){
    return smithBeckmannG1(alpha, i, m) * smithBeckmannG1(alpha, o, m);
}
float MicroHelper::pdf(float alpha, const Vector3f &m){
    return MicroHelper::eval(alpha, m) * Frame::cosTheta(m);
}
Vector3f MicroHelper::sample(float alpha, Point2f xi){
    return Warp::squareToBeckmann(xi, alpha);
}




NORI_NAMESPACE_END