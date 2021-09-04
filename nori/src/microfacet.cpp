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

#include <nori/bsdf.h>
#include <nori/frame.h>
#include <nori/warp.h>
#include <nori/common.h>

NORI_NAMESPACE_BEGIN

using namespace std;

class Microfacet : public BSDF {
public:
    Microfacet(const PropertyList &propList) {
        /* RMS surface roughness */
        m_alpha = propList.getFloat("alpha", 0.1f);

        /* Interior IOR (default: BK7 borosilicate optical glass) */
        m_intIOR = propList.getFloat("intIOR", 1.5046f);

        /* Exterior IOR (default: air) */
        m_extIOR = propList.getFloat("extIOR", 1.000277f);

        /* Albedo of the diffuse base material (a.k.a "kd") */
        m_kd = propList.getColor("kd", Color3f(0.5f));

        /* To ensure energy conservation, we must scale the 
           specular component by 1-kd. 

           While that is not a particularly realistic model of what 
           happens in reality, this will greatly simplify the 
           implementation. Please see the course staff if you're 
           interested in implementing a more realistic version 
           of this BRDF. */
        m_ks = 1 - m_kd.maxCoeff();
    }

    /// Evaluate the microfacet normal distribution D
    float evalBeckmann(const Normal3f &m) const {
        float temp = Frame::tanTheta(m) / m_alpha,
              ct = Frame::cosTheta(m), ct2 = ct*ct;

        return std::exp(-temp*temp) 
            / (M_PI * m_alpha * m_alpha * ct2 * ct2);
    }

    /// Evaluate Smith's shadowing-masking function G1 
    float smithBeckmannG1(const Vector3f &v, const Normal3f &m) const {
        float tanTheta = Frame::tanTheta(v);

        /* Perpendicular incidence -- no shadowing/masking */
        if (tanTheta == 0.0f)
            return 1.0f;

        /* Can't see the back side from the front and vice versa */
        if (m.dot(v) * Frame::cosTheta(v) <= 0)
            return 0.0f;

        float a = 1.0f / (m_alpha * tanTheta);
        if (a >= 1.6f)
            return 1.0f;
        float a2 = a * a;

        /* Use a fast and accurate (<0.35% rel. error) rational
           approximation to the shadowing-masking function */
        return (3.535f * a + 2.181f * a2) 
             / (1.0f + 2.276f * a + 2.577f * a2);
    }

    /// Evaluate the BRDF for the given pair of directions
    virtual Color3f eval(const BSDFQueryRecord &bRec) const override {
        //throw NoriException("MicrofacetBRDF::eval(): not implemented!");
        float coswo = Frame::cosTheta(bRec.wo);
        if(coswo <= 0){
            return Color3f(0.f);
        }
        Color3f res = 0;
        res += m_kd/M_PI;
        Vector3f wh = (bRec.wo + bRec.wi).normalized();
        float beckmann = evalBeckmann(wh);
        float shadow = smithBeckmannG1(bRec.wi, wh) * smithBeckmannG1(bRec.wo, wh);
        float cosThetaI = wh.dot(bRec.wo);
        float f = fresnel(cosThetaI, m_extIOR, m_intIOR);
        res += m_ks* (beckmann * f * shadow)/ (4 * Frame::cosTheta(bRec.wi) * Frame::cosTheta(bRec.wo));
        return res;
    }


    /// Evaluate the sampling density of \ref sample() wrt. solid angles
    virtual float pdf(const BSDFQueryRecord &bRec) const override {
        //throw NoriException("MicrofacetBRDF::pdf(): not implemented!");
        if(Frame::cosTheta(bRec.wo) <= 0)
            return 0.0f;
        Vector3f wh = (bRec.wi + bRec.wo).normalized();
        float J = abs(1/(4 * (wh.dot(bRec.wo))));
        float D = evalBeckmann(wh);
        return m_ks * D * Frame::cosTheta(wh) * J + (1 - m_ks) * (Frame::cosTheta(bRec.wo)/M_PI);
    }


    /// Sample the BRDF
    virtual Color3f sample(BSDFQueryRecord &bRec, const Point2f &_sample) const override {
        //throw NoriException("MicrofacetBRDF::sample(): not implemented!");

        //depends on ks: sample is uniform --> with probability ks we will do specular ans with probabality 1-ks we will do diffuse
        if(_sample.x() < m_ks){

            // transform the x to a number from 0 to 1
            float _sample_x = _sample.x()/m_ks;
            Point2f sample_temp = Point2f(_sample_x, _sample.y());
            //take a sample
            Vector3f wh = Warp::squareToBeckmann(sample_temp, m_alpha);
            //find wi which is the reflected of wi with wo being the norm
            Vector3f wo = (2 * bRec.wi.dot(wh) * wh- bRec.wi).normalized();
            //is it below the surface
            float coswo = Frame::cosTheta(wo);
            if(coswo <= 0){
                return Color3f(0.f);
            }
            bRec.wo = wo;
            return eval(bRec) / pdf(bRec) * Frame::cosTheta(bRec.wo);

        }
        else{
            // transform the x to a number from 0 to 1
            float _sample_x = (_sample.x()-m_ks)/(1-m_ks);
            Point2f sample_temp = Point2f(_sample_x, _sample.y());
            Vector3f wo = Warp::squareToCosineHemisphere(sample_temp);
            if(Frame::cosTheta(wo) <= 0)
                return Color3f(0.f);
            bRec.wo = wo;
            return eval(bRec) / pdf(bRec) * Frame::cosTheta(bRec.wo);

        }



    }


    virtual std::string toString() const override {
        return tfm::format(
            "Microfacet[\n"
            "  alpha = %f,\n"
            "  intIOR = %f,\n"
            "  extIOR = %f,\n"
            "  kd = %s,\n"
            "  ks = %f\n"
            "]",
            m_alpha,
            m_intIOR,
            m_extIOR,
            m_kd.toString(),
            m_ks
        );
    }
private:
    float m_alpha;
    float m_intIOR, m_extIOR;
    float m_ks;
    Color3f m_kd;
};

NORI_REGISTER_CLASS(Microfacet, "microfacet");
NORI_NAMESPACE_END
