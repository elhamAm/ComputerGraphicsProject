//this is a simplified version of the mitsuba roughDielectric
#include <nori/bsdf.h>
#include <nori/frame.h>
#include <nori/warp.h>
#include <nori/microHelper.h>

NORI_NAMESPACE_BEGIN


class roughDielectric : public BSDF{
public:
    roughDielectric(const PropertyList &propList) {
        m_intIOR = propList.getFloat("intIOR", 1.5f);
        m_extIOR = propList.getFloat("extIOR", 1.000277f);
        m_alpha = propList.getFloat("alpha", 0.2f);
    }

    //mitsuba
    //https://github.com/mitsuba-renderer/mitsuba/blob/master/src/bsdfs/roughdielectric.cpp
    Color3f eval(const BSDFQueryRecord &bRec) const{
         /* Discrete BRDFs always evaluate to zero in Nori */
        if (bRec.measure != ESolidAngle) return Color3f(0.0f);
       
        /* Determine the type of interaction */
        bool reflect = bRec.wi.z()*bRec.wo.z() > 0.0f;
        //eta
        float eta;
        if(Frame::cosTheta(bRec.wi) > 0.0f ){
            eta = m_intIOR / m_extIOR;
        }
        else{
            eta = m_extIOR / m_intIOR;
        }

        Vector3f H;

        if(reflect){
            /* Calculate the reflection half-vector */
            H = (bRec.wo+bRec.wi).normalized();
        } 
        else{
            H = (bRec.wi + bRec.wo*eta).normalized();
        }

        /* Ensure that the half-vector points into the
           same hemisphere as the macrosurface normal */
        H *= sign(Frame::cosTheta(H));

        /* Evaluate the microfacet normal distribution */
        const float D = MicroHelper::eval(m_alpha, H);
        if (D == 0.0f) {
            return Color3f(0.0f);
        }

        float wiDotH = bRec.wi.dot(H);

        /* Fresnel factor */
        float cosThetaI;
        float F =  fresnelDielectricExt(wiDotH, cosThetaI, eta);

        /* Smith's shadow-masking function */
        const float G = MicroHelper::G(m_alpha, bRec.wi, bRec.wo, H);


        if (reflect){
            float fr = (F * G * D)/ (4.0f * std::abs(Frame::cosTheta(bRec.wi)));
            return Color3f(fr);
        } 
        else{            
            /* Calculate the total amount of transmission */
            float sqrtDenom = bRec.wi.dot(H) + eta * bRec.wo.dot(H);
            float value = ((1 - F) * D * G * eta * eta
                * bRec.wi.dot(H) * bRec.wo.dot(H)) /
                (Frame::cosTheta(bRec.wi) * sqrtDenom * sqrtDenom);

            return std::abs(value);
        }
    }
    static inline float sign(float x){
        if (x >= 0.0) return 1.0;
        else return -1.0;

    }

    static Vector3f refract(const Vector3f &wi, const Vector3f &n, float eta, float cosThetaT){
        if (cosThetaT < 0)
            eta = 1 / eta;

        return n * (wi.dot(n) * eta + cosThetaT) - wi * eta;
    }

    //mitsuba
    //https://github.com/mitsuba-renderer/mitsuba/blob/master/src/bsdfs/roughdielectric.cpp
    float pdf(const BSDFQueryRecord &bRec)const{
        if (bRec.measure != ESolidAngle)
            return 0.0f;

        bool reflect  = Frame::cosTheta(bRec.wi) * Frame::cosTheta(bRec.wo) > 0.0f;

        float eta = Frame::cosTheta(bRec.wi) > 0.0f ? m_intIOR / m_extIOR : m_extIOR / m_intIOR;

        Vector3f H;
        float dwh_dwo;

        if (reflect){
            /* Zero probability if this component was not requested */
            H  = (bRec.wi + bRec.wo).normalized();

            /* Jacobian of the half-direction mapping */
            dwh_dwo = 1.0f / (4.0f * bRec.wo.dot(H));
        } 
        else{
            H  = (bRec.wi + bRec.wo * eta).normalized();

            /* Jacobian of the half-direction mapping */
            float sqrtDenom = bRec.wi.dot(H) + eta * bRec.wo.dot(H);
            dwh_dwo = (eta*eta * bRec.wo.dot(H)) / (sqrtDenom*sqrtDenom);
        }

        /* Ensure that the half-vector points into the
           same hemisphere as the macrosurface normal */
        H *= sign(Frame::cosTheta(H));

        float prob = MicroHelper::pdf(m_alpha, H);
        float cosThetaT = 0.0f;
        float F = fresnelDielectricExt(bRec.wi.dot(H), cosThetaT,  m_intIOR / m_extIOR);
        prob *= reflect ? F : (1-F);

        return std::abs(prob * dwh_dwo);
    }

    //mitsuba
    //https://github.com/mitsuba-renderer/mitsuba/blob/master/src/bsdfs/roughdielectric.cpp
    Color3f sample(BSDFQueryRecord &bRec, const Point2f &sample) const{        

        /* Sample M, the microfacet normal */
        Vector3f m = MicroHelper::sample(m_alpha, sample);
        float microfacetPDF = MicroHelper::pdf(m_alpha, m);
        if(microfacetPDF == 0.0) {
            return Color3f(0.0f);
        }

        float wiDotM = bRec.wi.dot(m);
        float cosThetaT = 0.0f;
        float F = fresnelDielectricExt(wiDotM, cosThetaT, m_intIOR / m_extIOR);


        bool sampleReflection = true;

        if(sample.x() > F){
            sampleReflection = false;
        }

        if(sampleReflection){
            /* Perfect specular reflection based on the microfacet normal */
            bRec.wo = 2.0f * wiDotM * m - bRec.wi;;
            bRec.eta = 1.0f;

            /* Side check */
            if (Frame::cosTheta(bRec.wi) * Frame::cosTheta(bRec.wo) <= 0)
                return Color3f(0.0f);
        } 
        else{
            if(cosThetaT == 0)
                return Color3f(0.0f);
            bRec.wo = refract(bRec.wi, m, m_intIOR / m_extIOR, cosThetaT);
            bRec.eta = cosThetaT < 0 ? m_intIOR / m_extIOR : m_extIOR / m_extIOR;

            /* Side check */
            if (Frame::cosTheta(bRec.wi) * Frame::cosTheta(bRec.wo) >= 0)
                return Color3f(0.0f);
        }

        float G = MicroHelper::G(m_alpha, bRec.wi, bRec.wo, m);
        float D = MicroHelper::eval(m_alpha, m);

        return std::abs(D * G * bRec.wi.dot(m) / (microfacetPDF * Frame::cosTheta(bRec.wi)));
    }

    std::string toString() const {
        return tfm::format(
            "roughDielectric[\n"
            "  intIOR = %f,\n"
            "  extIOR = %f\n"
            "  m_alpha = %f,\n"
            "]",
            m_intIOR, m_extIOR, m_alpha);
    }
    bool isDeltaBSDF() const {
        return false;
    }
private:
    float m_intIOR;
    float m_extIOR;
    //roughness 
    float m_alpha;
};

NORI_REGISTER_CLASS(roughDielectric, "roughDielectric");
NORI_NAMESPACE_END