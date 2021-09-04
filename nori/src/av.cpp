#include <nori/integrator.h>
#include <nori/scene.h>
#include<nori/warp.h>
#include<nori/ray.h>


NORI_NAMESPACE_BEGIN

class AverageVisibility: public Integrator {
public:
    float lengthOfRay;
    AverageVisibility(const PropertyList &props) {
        lengthOfRay = (float)props.getFloat("length", 1);
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        /* Find the surface that is visible in the requested direction */
        Intersection its;
        //If there's no intersection between the ray and the scene then it's occluded
        if (!scene->rayIntersect(ray, its))
            return Color3f(1.0f);

  
        Normal3f normal = its.shFrame.n;
        //find a vector normal to sphere in the hemisphere
        Vector3f vecNormalToHem = Warp::sampleUniformHemisphere(sampler, normal);
        //the ray is made from the vector and the starting point which is the hit point, the ray starts from min Epsilon and goes on to the length passed in the xml file
        Ray3f tray(its.p, vecNormalToHem, Epsilon, lengthOfRay);

        //dark if the ray segment is occluded: the ray hits another point in the intersections
        if(!scene->rayIntersect(tray, its)){
            return Color3f(1.0f);
        }
        
        return Color3f(0.0f);
    }

    std::string toString() const {
        return "AverageVisibility[]";
    }
};

NORI_REGISTER_CLASS(AverageVisibility, "av");
NORI_NAMESPACE_END