#include <nori/integrator.h>
#include <nori/emitter.h>
#include <nori/warp.h>
#include <nori/scene.h>
#include <nori/shape.h>
#include <nori/bsdf.h>
#include <time.h>


NORI_NAMESPACE_BEGIN;
using namespace std;

class Direct_EMS: public Integrator{
    public:
    Direct_EMS(const PropertyList &props){
    }

    Color3f Li(const Scene *scene, Sampler *sample, const Ray3f &ray)const{ 
        // take the Emitter of the scene
        //srand( (unsigned)time( NULL ) );
        //float rnd = (float) rand()/RAND_MAX;

        Color3f pointColor=0.0f;
        Intersection its;

        // check to see if there's even an intersection between the camera ray and the scene
        bool intersects = scene->rayIntersect(ray, its);
  
        EmitterQueryRecord rec= EmitterQueryRecord();
        rec.ref = its.p;

        
        if(intersects){
            for(auto &e: scene->getLights()){
                //Point2f sample;

                //the point where we sample the transmittion of the light: the intersection of the camera ray with the scene
                
                Color3f lightArriving = e->sample(rec, sample->next2D());

                float cos = (rec.wi.dot(its.shFrame.n)) / rec.wi.norm();     
                //getting bsdf depending on the intersection mesh and the query(the wi, wo)
                BSDFQueryRecord query = BSDFQueryRecord(its.toLocal(rec.wi), its.toLocal(-ray.d), ESolidAngle);
                query.uv = its.uv;
                auto bsdf = its.mesh->getBSDF();
                //make sure shadow ray is does not hit an obstacle on its way
                pointColor += (!scene->rayIntersect(rec.shadowRay)) * bsdf->eval(query) * lightArriving * abs(cos);
            }

            //emitted light
            bool emits = its.mesh->isEmitter();
            //rec.ref is the sampled point on the emitter as the intersection is accounted as an emitter
            // rec.o is the camera where we want to calculate arrived light
            EmitterQueryRecord recEmitter(ray.o, its.p, its.shFrame.n);
            Color3f Le= 0.0f;

            recEmitter.wi = (recEmitter.p-recEmitter.ref).normalized();
        
            if(emits){
                Le =  its.mesh->getEmitter()->eval(recEmitter);
            }
            pointColor += Le;


        
		}
   
        return pointColor;
    }

    std::string toString() const {
        return "Direct[]";
    }

};

NORI_REGISTER_CLASS(Direct_EMS, "direct_ems");
NORI_NAMESPACE_END;

    