#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>
#include <nori/sampler.h>



NORI_NAMESPACE_BEGIN

class Path_MIS : public Integrator {
public:
    Path_MIS(const PropertyList &props){      
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
        float w_mat = 1.0f;
        float w_em = 1.0f;

        

		while (true){

			Intersection its;
            //std:: cout << "inside while" << endl;

			if (!scene->rayIntersect(curRay, its)) {
                //std:: cout << "inside if" << endl;
                //std::cout << "before" << endl;
                //cout << "env: "<<scene->getEnvEmitter() << endl;
				if (scene->getEnvEmitter() == nullptr) {
                    //std:: cout << "inside first if" << endl;
					//return pointColor;
                    //std::cout << "before2" << endl;
                    break;
				} else {
                    //std:: cout << "inside sec if" << endl;
					EmitterQueryRecord lRec;
					lRec.wi = curRay.d;
                    //cout << "got in" << endl;
					pointColor += w_mat * t * scene->getEnvEmitter()->eval(lRec);
                    break;
				}
                //std::cout << "after" << endl;
			}
            //std:: cout << "ooutside if" << endl;


                                        /**************************/
                                        /**** Emitter itself   ****/
                                        /**************************/

			Color3f Le = 0;
			if (its.mesh->isEmitter()) {
                EmitterQueryRecord recRad(curRay.o, its.p, its.shFrame.n);
                pointColor +=  (its.mesh->getEmitter()->eval(recRad) * w_mat * t);
			}

		                                /**************************/
                                        /**** Russian Roulette ****/
                                        /**************************/
            float successProbability = std::min(t.maxCoeff(), float(0.99));
            float randomNumber = sample->next1D();

            //cout << "t before: " << t << endl;
            //std:: cout << "here1" << endl;
            // russian roulette
            if (russianRoulette(randomNumber, successProbability))
                break;
            else t /= successProbability;

                                        /**************************/
                                        /**** emitter sampling ****/
                                        /**************************/
            Color3f pointColorEMS(0.f);
            BSDFQueryRecord bsdfRec(its.toLocal(-curRay.d.normalized()));
            if(bsdfRec.measure == EDiscrete){
                w_mat = 1.f;
                w_em = 0.f;
            }
            else{
                Color3f pointColorEMS = 0;
                //std::cout << "rand" << endl;
                const Emitter* randEmitter= scene->getRandomEmitter(sample->next1D());
                //std::cout << "after rand" << endl;
                EmitterQueryRecord emiRecord = EmitterQueryRecord(its.p);
                //std:: cout << "here5" << endl;
                Color3f Lo = randEmitter->sample(emiRecord, sample->next2D()) * scene->getLights().size();
                

                BSDFQueryRecord query = BSDFQueryRecord(its.shFrame.toLocal(-curRay.d), its.shFrame.toLocal(emiRecord.wi), EMeasure::ESolidAngle);
                query.uv = its.uv;
                auto bsdf = its.mesh->getBSDF();
                Color3f bsdf_val = bsdf->eval(query);
                
                float pdf_ems = randEmitter->pdf(emiRecord);
                float pdf_mat = its.mesh->getBSDF()->pdf(query);
                if (pdf_ems + pdf_mat != 0)
                    w_em = pdf_ems / (pdf_ems + pdf_mat);
                //std:: cout << "here3" << endl;
      
                if (!scene->rayIntersect(emiRecord.shadowRay)){
                    float cos = std::max(0.f, Frame::cosTheta(its.shFrame.toLocal(emiRecord.wi)));
                    pointColorEMS = bsdf_val* Lo * cos;
                }
                pointColor += t * w_em * pointColorEMS;



                                                    /**************************/
                                                    /****    bsdf sampling ****/
                                                    /**************************/
                
                bsdf_val = bsdf->eval(bsdfRec);
                Color3f sampledBSDF = bsdf->sample(bsdfRec, sample->next2D());
                t *= sampledBSDF;
                
                
                //update ray
                curRay = Ray3f(its.p, its.toWorld(bsdfRec.wo));
                //std:: cout << "here4" << endl;
                //updating mats
                float pdf_mats = its.mesh->getBSDF()->pdf(bsdfRec);
                //std:: cout << "here5" << endl;
                Intersection hitEmitter;
                //std::cout << "before3" << endl;
                //cout << "env3: "<<scene->getEnvEmitter() << endl;
                if (scene->rayIntersect(curRay, hitEmitter)){
                    //std::cout << "in ray inter" << endl;
                    if(hitEmitter.mesh->isEmitter()){
                        EmitterQueryRecord emRec(its.p, hitEmitter.p, hitEmitter.shFrame.n);
                        float pdf_ems = hitEmitter.mesh->getEmitter()->pdf(emRec);
                        if (pdf_mats + pdf_ems != 0)
                            w_mat = pdf_mats / (pdf_mats + pdf_ems);
                    }
                } 
                else if(scene->getEnvEmitter() != nullptr){
                    EmitterQueryRecord emRec;
                    emRec.wi = curRay.d;
                    //cout << "got in" << endl;
                    float pdf_ems = scene->getEnvEmitter()->pdf(emRec);
                    if (pdf_mats +  pdf_ems != 0)
                        w_mat = pdf_mats / (pdf_mats + pdf_ems);
                }
                //std::cout << "after" << endl;
                //std:: cout << "here6" << endl;
            }
		}
        //cout << "point" << endl;
        return pointColor;
	} 

    std::string toString() const {
        return "indirect[]";
    }
};

NORI_REGISTER_CLASS(Path_MIS, "path_mis");
NORI_NAMESPACE_END;
