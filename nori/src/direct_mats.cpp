#include <nori/integrator.h>
#include <nori/emitter.h>
#include <nori/warp.h>
#include <nori/scene.h>
#include <nori/shape.h>
#include <nori/bsdf.h>
#include <time.h>


NORI_NAMESPACE_BEGIN;
using namespace std;

class Direct_MATS: public Integrator{
    public:
    Direct_MATS(const PropertyList &props){
    }

    Color3f Li(const Scene *scene, Sampler *sample, const Ray3f &ray)const{ 

        Color3f pointColor=0.0f;
        Intersection its;

        // check to see if there's even an intersection between the camera ray and the scene
        bool intersects = scene->rayIntersect(ray, its);

        if(intersects){
            BSDFQueryRecord bRec(its.shFrame.toLocal(-ray.d));
            //Color3f lightArriving = e->sample(rec, sample->next2D());
            auto bsdf = its.mesh->getBSDF();
            //make sure shadow ray is does not hit an obstacle on its way

            Color3f sampledBSDF =  bsdf->sample(bRec, sample->next2D());
            Ray3f shadow = Ray3f(its.p, its.toWorld(bRec.wo));
            Intersection hitEmitter;
            if(!scene->rayIntersect(shadow, hitEmitter)) return pointColor;
            


            if(hitEmitter.mesh->isEmitter()){

                EmitterQueryRecord eRec = EmitterQueryRecord();
                eRec.p = hitEmitter.p;
                eRec.ref = its.p;
                eRec.n = hitEmitter.shFrame.n; 
                Color3f lightArriving = hitEmitter.mesh->getEmitter()->eval(eRec);
                pointColor += sampledBSDF * lightArriving;

            }
          

            //check maybe the intersection is an emitter
            bool emits = its.mesh->isEmitter();
            //rec.ref is the sampled point on the emitter as the intersection is accounted as an emitter
            // rec.o is the camera where we want to calculate arrived light
            EmitterQueryRecord recEmitter(ray.o, its.p, its.shFrame.n);

            //recEmitter.wi = (recEmitter.p-recEmitter.ref).normalized();
        
            if(emits){
                Color3f Le =  its.mesh->getEmitter()->eval(recEmitter);
                pointColor += Le;
            }
        
        }
   
        return pointColor;
    }

    std::string toString() const {
        return "Direct[]";
    }

};

NORI_REGISTER_CLASS(Direct_MATS, "direct_mats");
NORI_NAMESPACE_END;

    
