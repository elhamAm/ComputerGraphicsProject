/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Romain Prévost

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

#include <nori/emitter.h>
#include <nori/warp.h>
#include <nori/shape.h>

NORI_NAMESPACE_BEGIN

class AreaEmitter : public Emitter {
public:
    AreaEmitter(const PropertyList &props) {
        m_radiance = props.getColor("radiance");
    }

    virtual std::string toString() const override {
        return tfm::format(
                "AreaLight[\n"
                "  radiance = %s,\n"
                "]",
                m_radiance.toString());
    }

    virtual Color3f eval(const EmitterQueryRecord & lRec) const override {
        if(!m_shape)
            throw NoriException("There is no shape attached to this Area light!");
        
        //should return the associated radiance value when the front side of an emissive shape was intersected and zero otherwise
        if(lRec.n.dot(-lRec.wi) >= 0){
            return m_radiance;
        }
        return Color3f(0.f);


        //throw NoriException("To implement...");
    }

    virtual Color3f sample(EmitterQueryRecord & lRec, const Point2f & sample) const override {
        if(!m_shape)
            throw NoriException("There is no shape attached to this Area light!");
        ShapeQueryRecord sRec(lRec.ref, lRec.p);
        sRec.ref = lRec.ref; 
        //uniformly samples a positions on the surface  
        m_shape->sampleSurface(sRec, sample);
        
        lRec.p = sRec.p;
        lRec.n = sRec.n;
        //the vector going from the hit point to the incident point
        lRec.wi = (lRec.p-lRec.ref).normalized(); 
        // the ray going from the intersection to the light source and parallel to the wi
        lRec.shadowRay = Ray3f(lRec.ref, lRec.wi, Epsilon, (lRec.p - lRec.ref).norm()-Epsilon);
        if(pdf(lRec) > 0) return eval(lRec)/pdf(lRec);
		else return Color3f(0.f);
        //throw NoriException("To implement...");
    }

    virtual float pdf(const EmitterQueryRecord &lRec) const override {
        if(!m_shape)
            throw NoriException("There is no shape attached to this Area light!");

        //throw NoriException("To implement...");
        ShapeQueryRecord sRec(lRec.ref, lRec.p);
        //sRec.p = lRec.p;
        sRec.n = lRec.n;
        //sRec.ref = lRec.ref;
       //getting solid angles
		float cosTheta0 = abs(lRec.n.dot(-lRec.wi));
        //converting dA to dwi with the course slides formula
		return m_shape->pdfSurface(sRec) * (lRec.p-lRec.ref).squaredNorm()/cosTheta0;
    }


    virtual Color3f samplePhoton(Ray3f &ray, const Point2f &sample1, const Point2f &sample2) const override {
        //throw NoriException("To implement...");

        ShapeQueryRecord sRec= ShapeQueryRecord();//lRec.p);
        //sRec.ref = lRec.ref; 
        //uniformly samples a positions on the surface  
        m_shape->sampleSurface(sRec, sample1);

        //we have to find a random ray
        Vector3f wo = Warp::squareToCosineHemisphere(sample2);
        //Vector3f rayDir = Warp::sampleUniformHemisphere(sample1, sRec.n);
        Frame frame(sRec.n);


        ray = Ray3f(sRec.p,  frame.toWorld(wo));
        float A = 1/sRec.pdf;
        return M_PI *  A * m_radiance;

    }


protected:
    Color3f m_radiance;
};

NORI_REGISTER_CLASS(AreaEmitter, "area")
NORI_NAMESPACE_END