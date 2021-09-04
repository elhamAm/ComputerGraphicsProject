/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/integrator.h>
#include <nori/sampler.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <nori/scene.h>
#include <nori/photon.h>

NORI_NAMESPACE_BEGIN

class PhotonMapper : public Integrator {
public:
    /// Photon map data structure
    typedef PointKDTree<Photon> PhotonMap;

    PhotonMapper(const PropertyList &props) {
        /* Lookup parameters */
        m_photonCount  = props.getInteger("photonCount", 1000000);
        m_photonRadius = props.getFloat("photonRadius", 0.0f /* Default: automatic */);
    }

    bool russianRoulette(float successProbability, float randomNumber)const{
        if(randomNumber > successProbability) return true;
        return false;
    }

    virtual void preprocess(const Scene *scene) override {
        cout << "Gathering " << m_photonCount << " photons .. ";
        cout.flush();

        /* Create a sample generator for the preprocess step */
        Sampler *sampler = static_cast<Sampler *>(
            NoriObjectFactory::createInstance("independent", PropertyList()));

        /* Allocate memory for the photon map */
        m_photonMap = std::unique_ptr<PhotonMap>(new PhotonMap());
        m_photonMap->reserve(m_photonCount);

		/* Estimate a default photon radius */
		if (m_photonRadius == 0)
			m_photonRadius = scene->getBoundingBox().getExtents().norm() / 500.0f;

	

		/* How to add a photon?
		 * m_photonMap->push_back(Photon(
		 *	Point3f(0, 0, 0),  // Position
		 *	Vector3f(0, 0, 1), // Direction
		 *	Color3f(1, 2, 3)   // Power
		 * ));
		 */

		// put your code to trace photons here

		/* Build the photon map */

        emitPhoton= 0;
        while( m_photonMap->size() < m_photonCount){
            emitPhoton++;
            Ray3f ray;
            Color3f emittedPhoton = scene->getRandomEmitter(sampler->next1D())->samplePhoton(ray, sampler->next2D(), sampler->next2D()) * scene->getLights().size();
            float successProbability = 1.0f;
            Color3f W = emittedPhoton;
            //ray = Ray3f(ray.o,  its.toWorld(ray.d));
            while(true){
                Intersection its;
                bool hits = scene->rayIntersect(ray, its);
                if(!hits) break;
                Color3f t(1.f);
                //if the surface is diffuse, set down as photon
                if(its.mesh->getBSDF()->isDiffuse()){
                    m_photonMap->push_back(Photon(its.p, (-ray.d), W*t));
                }

                float randomNumber = sampler->next1D();
                successProbability = std::min(t.maxCoeff(), float(0.99));
                if(russianRoulette(successProbability, randomNumber)) break;
                else t /= successProbability;

                BSDFQueryRecord query = BSDFQueryRecord(its.toLocal(-ray.d));
                query.uv = its.uv;
                Color3f bsdf_val= its.mesh->getBSDF()->sample(query, sampler->next2D());
                //multiply the weight by the bsdf value everytime

                t *= bsdf_val;
                //this is the new ray
                ray = Ray3f(its.p,  its.toWorld(query.wo));
                
            }
        }

        m_photonMap->build();
    }

    virtual Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &_ray) const override {
    	
		/* How to find photons?
		 * std::vector<uint32_t> results;
		 * m_photonMap->search(Point3f(0, 0, 0), // lookup position
		 *                     m_photonRadius,   // search radius
		 *                     results);
		 *
		 * for (uint32_t i : results) {
		 *    const Photon &photon = (*m_photonMap)[i];
		 *    cout << "Found photon!" << endl;
		 *    cout << " Position  : " << photon.getPosition().toString() << endl;
		 *    cout << " Power     : " << photon.getPower().toString() << endl;
		 *    cout << " Direction : " << photon.getDirection().toString() << endl;
		 * }
		 */

		// put your code for path tracing with photon gathering here

        Color3f pointColor(0.0f);
        Color3f t(1.0f);
        // the ray will be updated with each operation
        Ray3f curRay = _ray;

        while(true)
        {
         
            Intersection its;
            if(!scene->rayIntersect(curRay, its))
                break;

            auto bsdf = its.mesh->getBSDF();
            Color3f pointColorEMS(0.f);

            //the emission of the point of intersection
            if(its.mesh->isEmitter())
            {
                EmitterQueryRecord recRad(curRay.o, its.p, its.shFrame.n);
                pointColor +=  its.mesh->getEmitter()->eval(recRad) *  t;
            }
            
            BSDFQueryRecord bsdfRec(its.toLocal(-curRay.d.normalized()));
            Color3f sampledBSDF = bsdf->sample(bsdfRec, sampler->next2D());

            // if diffuse then don't even calculate, just calculate the photon density 
            if(its.mesh->getBSDF()->isDiffuse()){
                std::vector<uint32_t> results;
		        m_photonMap->search(its.p, m_photonRadius, results);
                Color3f photonDensityEstimation = 0.0f;
		        for (uint32_t i : results){
		            const Photon &photon = (*m_photonMap)[i];
                    float A =  M_PI * m_photonRadius * m_photonRadius;
                    BSDFQueryRecord BSDFQueryOfp(its.toLocal(-curRay.d.normalized()), its.toLocal(photon.getDirection()), EMeasure::ESolidAngle);
                    Color3f bsdfOfp = bsdf->eval(BSDFQueryOfp);
                    photonDensityEstimation += (photon.getPower() * bsdfOfp) / A;
		            //cout << "Found photon!" << endl;
		            //cout << " Position  : " << photon.getPosition().toString() << endl;
		            //cout << " Power     : " << photon.getPower().toString() << endl;
		            //cout << " Direction : " << photon.getDirection().toString() << endl;
                    
		        }
                pointColor += t * photonDensityEstimation/ emitPhoton;
                
                break;
            }

            float successProbability = std::min(t.x(), float(0.99));
            float randomNumber = sampler->next1D();
            if (russianRoulette(successProbability, randomNumber))break;
            else t /= successProbability;
           
        
            /*MAT*/
            
            Ray3f shadowray(its.p, its.toWorld(bsdfRec.wo));
            
            //if (std::abs(sampledBSDF.maxCoeff() - 0.f) < Epsilon || std::abs(sampledBSDF.minCoeff() - 0.f) < Epsilon)
			//	break;


            //update the t and the ray for next time
            t *= sampledBSDF;
            curRay = shadowray;
            
        }
        return pointColor;
    }

    virtual std::string toString() const override {
        return tfm::format(
            "PhotonMapper[\n"
            "  photonCount = %i,\n"
            "  photonRadius = %f\n"
            "]",
            m_photonCount,
            m_photonRadius
        );
    }
private:
    /* 
     * Important: m_photonCount is the total number of photons deposited in the photon map,
     * NOT the number of emitted photons. You will need to keep track of those yourself.
     */ 
    int m_photonCount;
    float m_photonRadius;
    int emitPhoton;
    std::unique_ptr<PhotonMap> m_photonMap;
};

NORI_REGISTER_CLASS(PhotonMapper, "photonmapper");
NORI_NAMESPACE_END
