#ifndef __PHASE_H__
#define __PHASE_H__


#include <nori/warp.h>
#include <nori/object.h>



NORI_NAMESPACE_BEGIN

class PhaseFunction : public NoriObject{
public:

    virtual float sample_phase_function(const Vector3f &wo, Vector3f &wi, const Point2f &sample) const=0;

    virtual std::string toString() const =0;

    virtual EClassType  getClassType() const { return EPhaseFunction; }


};



NORI_NAMESPACE_END

#endif 