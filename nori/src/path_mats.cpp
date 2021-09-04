#include <nori/integrator.h>
#include <nori/emitter.h>
#include <nori/warp.h>
#include <nori/scene.h>
#include <nori/shape.h>
#include <nori/bsdf.h>
#include <time.h>


NORI_NAMESPACE_BEGIN;
using namespace std;

class PATH_MATS: public Integrator{
    public:
    PATH_MATS(const PropertyList &props){
    }

    Color3f Li(const Scene *scene, Sampler *sample, const Ray3f &ray)const{ 

        //we need a q for the russian roulette
        float q = 0.03;

        Color3f pointColor=0.0f;
        Intersection its;

        // check to see if there's even an intersection between the camera ray and the scene
        bool intersects = scene->rayIntersect(ray, its);

        if(!intersects){
            return Color3f(0.0f);
        }
            BSDFQueryRecord bRec(its.toLocal(-ray.d.normalized()));
            bRec.p = its.p;
            //Color3f lightArriving = e->sample(rec, sample->next2D());
            auto bsdf = its.mesh->getBSDF();
            //make sure shadow ray is does not hit an obstacle on its way

            Color3f sampledBSDF = bsdf->sample(bRec, sample->next2D());
            //Ray3f shadow = Ray3f(its.p, its.toWorld(bRec.wo));
            
            Color3f Le= Color3f(0.0f);
            if(its.mesh->getEmitter()){
                EmitterQueryRecord recEmitter(ray.o, its.p, its.shFrame.n);
                Le = its.mesh->getEmitter()->eval(recEmitter);
            }

            Ray3f newRay(its.p, its.toWorld(bRec.wo));

            if(sample->next1D() > q){
                Color3f lightArriving  = this->Li(scene, sample, newRay);
                //formula like the one in the course slides
                return Le + (lightArriving * sampledBSDF) / (1-q);
            }
            else
            {
                return Le;
            }
            
        
        
   
    }

    std::string toString() const {
        return "Direct[]";
    }

};

NORI_REGISTER_CLASS(PATH_MATS, "path_mats");
NORI_NAMESPACE_END;

    
