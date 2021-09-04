#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>
#include <nori/sampler.h>
#include <nori/warp.h>


NORI_NAMESPACE_BEGIN

class VolPathIntegrator : public Integrator {
public:
	VolPathIntegrator(const PropertyList &props) {
		/* No parameters this time */
	}
    bool russianRoulette(float randomNumber, float successProbability)const{
        if(randomNumber >= successProbability) return true;
        return false;
    }

	Color3f Li(const Scene *scene, Sampler *sample, const Ray3f &ray) const {
		// by defalut values
		Color3f t(1.0f);
        Color3f pointColor(0.0f);
        // this ray will be updated every iteration
        Ray3f curRay = ray;
		const Medium* medium = scene->getMedium();
        

		while (true) {
			Intersection its;

			float maxDist;
			if(scene->rayIntersect(curRay, its))
				maxDist = (its.p - curRay.o).norm();
			else
				maxDist =its.t;

            MediumQueryRecord mRec;
            mRec.maxDistance = maxDist;
            Color3f albedo = medium->sample(curRay, sample, mRec);

            //if hits volume
			if (mRec.success){
                //cout << "in" << endl;
                Vector3f wo;
                float phaseSampled = medium->getPhaseFunction()->sample_phase_function(curRay.d, wo, sample->next2D());
				curRay = Ray3f(curRay.o + mRec.sampleDist * curRay.d.normalized(), wo);
				const Emitter* emitter = scene->getRandomEmitter(sample->next1D());
				EmitterQueryRecord eRec;
				eRec.ref = curRay.o;
				Color3f Le = emitter->sample(eRec, sample->next2D())*scene->getLights().size();
				if (scene->rayIntersect(eRec.shadowRay))
					Le = Color3f(0, 0, 0);
				t *= albedo;
				pointColor += t * medium->transmittance(curRay.o, eRec.p) * phaseSampled;
			} 
            //if hits surface
            else{
                //cout << "surface"<< endl;
                //self emitter
				if(its.mesh->isEmitter()){
					EmitterQueryRecord eRec(curRay.o, its.p, its.shFrame.n);
					Color3f L = its.mesh->getEmitter()->eval(eRec);
					pointColor += L * t * medium->transmittance(curRay.o, its.p);
				} 
                else{
					const Emitter* emitter = scene->getRandomEmitter(sample->next1D());
					EmitterQueryRecord eRec;
					eRec.ref = its.p;
					Color3f L = emitter->sample(eRec, sample->next2D())*scene->getLights().size();
					if (!scene->rayIntersect(eRec.shadowRay)){
                        BSDFQueryRecord bRecE(its.shFrame.toLocal(-curRay.d), its.shFrame.toLocal(eRec.wi), ESolidAngle);
                        bRecE.uv = its.uv;
                        auto bsdf = its.mesh->getBSDF();
                        Color3f bsdfval = bsdf->eval(bRecE);
                        pointColor += L * t * bsdfval * medium->transmittance(curRay.o, eRec.p);

                    }

				}


				BSDFQueryRecord bRec(its.shFrame.toLocal(-curRay.d));
                auto bsdf = its.mesh->getBSDF();
				Color3f bsdf_val = bsdf->sample(bRec, sample->next2D());
				t *= bsdf_val;
				//updating ray
				curRay = Ray3f(its.p, its.toWorld(bRec.wo));
			}
		                                /**************************/
                                        /**** Russian Roulette ****/
                                        /**************************/

            float successProbability = std::min(t.maxCoeff(), float(0.99));
            float randomNumber = sample->next1D();
            if (russianRoulette(randomNumber, successProbability))
                break;
            else t /= successProbability;
		}
        return pointColor;
	}

	std::string toString() const {
		return "VolPathIntegrator[]";
	}
};


NORI_REGISTER_CLASS(VolPathIntegrator, "vol_path");
NORI_NAMESPACE_END