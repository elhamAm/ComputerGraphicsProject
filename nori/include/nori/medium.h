#if !defined(__NORI_MEDIUM_H)
#define __NORI_MEDIUM_H

#include <nori/object.h>
#include <nori/bbox.h>
#include <nori/PhaseFunction.h>
#include <nori/sampler.h>

NORI_NAMESPACE_BEGIN

struct MediumQueryRecord {
    //Interaction point
    Point3f p; 
    //success in sampling
    bool success;
    //max free path
    float maxDistance;
    float sampleDist;
    MediumQueryRecord(){
        success = false;
        maxDistance = 0;
    }

};


class Medium : public NoriObject {
public:
    Medium(const PropertyList &props);
    /* Color3f transmittance(const Point3f x, const Point3f y) const;*/
    Color3f sample(const Ray3f &ray, Sampler *sampler, MediumQueryRecord &mi) const;
    Color3f transmittance(const Ray3f &ray, Sampler *sampler, MediumQueryRecord &medRec) const;
    Color3f transmittance(const Point3f a, const Point3f b) const;

    Color3f getSigma_a() const {
        return m_sigma_a; 
    }
    Color3f getSigma_s() const {
        return m_sigma_s;
    }
    Color3f getSigma_t() const {
        return m_sigma_t; 
    }
    Color3f getAlbedo() const {
        return m_albedo; 
    }

    BoundingBox3f getAabb() const {
        return aabb; 
    }

    PhaseFunction* getPhaseFunction() const {
        return m_phaseFunction; 
    }
     EClassType  getClassType() const override {
        return EMedium; 
    }

    void addChild(NoriObject *child) override;
    float getDensity(const Point3f &p) const;
    std::string toString() const override;

   

private:
    Color3f m_sigma_a;
    Color3f m_sigma_s;
    Color3f m_sigma_t;
    Color3f m_albedo;
    PhaseFunction* m_phaseFunction = nullptr;
    BoundingBox3f aabb;
};


NORI_NAMESPACE_END
#endif 