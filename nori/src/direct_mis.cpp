#include <nori/integrator.h>
#include <nori/emitter.h>
#include <nori/warp.h>
#include <nori/scene.h>
#include <nori/shape.h>
#include <nori/bsdf.h>
#include <time.h>


NORI_NAMESPACE_BEGIN;
using namespace std;

class Direct_MIS: public Integrator{
    public:
    Direct_MIS(const PropertyList &props){
    }

    	Color3f Li(const Scene *scene, Sampler *sample, const Ray3f &ray) const{

            //initialize ***********************************************************************************
            Color3f pointColorMAT = Color3f(0.0f);
            float w_mat = 0.0f;
            Color3f pointColorEMS = Color3f(0.0f);
            float w_em = 0.0f;

            Intersection its;
            if (!scene->rayIntersect(ray, its))
                return Color3f(0.0f);


            //emitter sampling ***********************************************************************************
            const BSDF* bsdf = its.mesh->getBSDF();

            auto s = sample->next2D();
            const Emitter* randEmitter = scene->getRandomEmitter(s.y());
            EmitterQueryRecord emiRecord = EmitterQueryRecord(its.p);
            Color3f Lo = scene->getLights().size() * randEmitter->sample(emiRecord, sample->next2D());
            

            BSDFQueryRecord query = BSDFQueryRecord(its.toLocal(-ray.d),its.toLocal(emiRecord.wi), ESolidAngle);
            query.uv = its.uv;
            Color3f bsdf_val= bsdf->eval(query);
            //make sure that Lo does not generate the Lo value
            if (!scene->rayIntersect(emiRecord.shadowRay) && Lo.maxCoeff() > Epsilon){
                float cos = its.shFrame.n.dot(emiRecord.wi.normalized());
                

                	if(std::abs(randEmitter->pdf(emiRecord) - 0.f) > Epsilon )
	                { 
		                   w_em = (randEmitter->pdf(emiRecord))/ ((randEmitter->pdf(emiRecord)) + bsdf->pdf(query));
	                }

                pointColorEMS = Lo * bsdf_val* abs(cos);
                //cout << "point color ems: " << pointColor << endl;

            }
    
            
            //bsdf sampling **************************************************************************************************
            BSDFQueryRecord bsdfRec(its.toLocal(-ray.d.normalized())); 
            bsdfRec.p = its.p;
            Color3f bsdfVal = bsdf->sample(bsdfRec, sample->next2D()); 
            Intersection hitEmitter;
            Ray3f shadowRay(its.p, its.toWorld(bsdfRec.wo));
            // if the sample hits a light source
            if(scene->rayIntersect(shadowRay, hitEmitter)){
                if(hitEmitter.mesh->isEmitter()){
                    const Emitter* emitter = hitEmitter.mesh->getEmitter();
                    EmitterQueryRecord eRec = EmitterQueryRecord(its.p, hitEmitter.p, hitEmitter.shFrame.n);

                    pointColorMAT = emitter->eval(eRec) * bsdfVal;
                    //cout << "point color mat: " << pointColorMAT << endl;

                    if(std::abs(bsdf->pdf(bsdfRec) - 0.f) > Epsilon && ((emitter->pdf(eRec)) + bsdf->pdf(bsdfRec)) > Epsilon)
	                { 
		                   w_mat = bsdf->pdf(bsdfRec) / ((emitter->pdf(eRec)) + bsdf->pdf(bsdfRec));
	                }
                }
            }

            //adding the radiance if the intersection is itself an emitter******************************************************
            Color3f Le = Color3f(0.0f);
            if (its.mesh->isEmitter()){
                EmitterQueryRecord recRad(ray.o, its.p, its.shFrame.n);
                Le = its.mesh->getEmitter()->eval(recRad);
            }

            return Le + w_em * pointColorEMS + w_mat * pointColorMAT;

	}

    std::string toString() const {
        return "Direct[]";
    }

};

NORI_REGISTER_CLASS(Direct_MIS, "direct_mis");
NORI_NAMESPACE_END;

    
