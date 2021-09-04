#include <nori/integrator.h>
#include <nori/emitter.h>
#include <nori/warp.h>
#include <nori/scene.h>
#include <nori/shape.h>
#include <nori/bsdf.h>


NORI_NAMESPACE_BEGIN;
using namespace std;

class Direct: public Integrator{
    public:
    Direct(const PropertyList &props){
    }

    Color3f Li(const Scene *scene, Sampler *sample, const Ray3f &ray)const{ 
        // take the pointLights of the scene
        vector<Emitter *> points = scene->getLights();
        Color3f pointColor=0;
        Intersection its;
        // check to see if there's even an intersection between the camera ray and the scene
        if(scene->rayIntersect(ray, its)){

            //add all the reflected lights caused by different pointLight sources
            for(int i = 0; i < points.size(); i++){
                Point2f sample;
                EmitterQueryRecord rec;
                //the point where we sample the transmittion of the light
                rec.ref = its.p;
                Color3f lightArriving = points[i]->sample(rec, sample);
                //need to give the query the wi(already calculated in sample()), the wr--->given in the frame of the intersection
                BSDFQueryRecord query = BSDFQueryRecord(its.toLocal(rec.wi), its.toLocal(ray.o - its.p), ESolidAngle);
                query.uv = its.uv;
                
                //check to see if the shadow ray(wi) arrives to the light source
                //if it does not then it is occluded 
                if(!scene->rayIntersect(rec.shadowRay)){
                        if(!scene->rayIntersect(rec.shadowRay)){
                        //calculating the cosinus of the angle between the normal and the incident ray: the projection divided by ...
                        float cos = (rec.wi.dot(its.shFrame.n)) / rec.wi.norm();
                        auto shape = its.mesh;
                        //getting bsdf depending on the intersection mesh and the query(the wi, wo)
                        auto bsdf = shape->getBSDF();
                        // the integral
                        pointColor += bsdf->eval(query) * lightArriving * abs(cos);
                    }
                }

            
            }

        }
        
        


        return pointColor;
    }

    std::string toString() const {
        return "Direct[]";
    }

};

NORI_REGISTER_CLASS(Direct, "direct");
NORI_NAMESPACE_END;

    