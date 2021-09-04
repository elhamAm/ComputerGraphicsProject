#include <nori/integrator.h>
#include <nori/emitter.h>
#include <nori/warp.h>
#include <nori/scene.h>
#include <nori/shape.h>



NORI_NAMESPACE_BEGIN

class PointLight: public Emitter {
public:
    //Point3f postion;
    //Point3f power;
    PointLight(const PropertyList &props){
        m_position = props.getPoint3("position", Point3f());
        m_power = props.getColor("power", Color3f());

    }

    Color3f sample(EmitterQueryRecord &lRec, const Point2f &sample) const{
        //to implement
        //need to fill out different fields of the lRec
        lRec.p = m_position;
        //the vector going from the hit point to the incident point
        lRec.wi = (lRec.p-lRec.ref).normalized(); 
        // the ray going from the intersection to the light source and parallel to the wi
        lRec.shadowRay = Ray3f(lRec.ref, lRec.wi, Epsilon, (lRec.p - lRec.ref).norm());
        return eval(lRec)/pdf(lRec);
    }

    Color3f eval(const EmitterQueryRecord &lRec) const{
        //to implement
        // the power of the source at the point ref: we divide the power of the light source by the surface of a sphere with the radius ref to the point sampled on the emitter
        return m_power/( 4 * M_PI * (lRec.ref - m_position).squaredNorm());
    }

    //there is only one point on the emitter so the probability of choosing it is
    float pdf(const EmitterQueryRecord &lRec) const{
        //to implement
        return 1;
    }

    std::string toString() const {
        return "PointLight[]";
    }

    protected:
        Point3f m_position;
        Color3f m_power;

};

NORI_REGISTER_CLASS(PointLight, "point");
NORI_NAMESPACE_END