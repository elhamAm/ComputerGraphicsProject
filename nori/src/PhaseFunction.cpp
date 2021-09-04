#include <nori/PhaseFunction.h>
#include <nori/warp.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN


class Isotropic : public PhaseFunction{
    public:

        Isotropic(const PropertyList& props){}

        float sample_phase_function(const Vector3f &wo, Vector3f &wi, const Point2f &sample) const{
            wi = Warp::squareToUniformSphere(sample);
            return INV_FOURPI;
        }

        std::string toString() const {
            return tfm::format("[ Isotropic ]");
        }

  

};

class Henyey_Greenstein: public PhaseFunction{
    public:

        Henyey_Greenstein(const PropertyList &props){
            m_g = props.getFloat("g", 0.0f);
        }

        float sample_phase_function(const Vector3f &wo, Vector3f &wi, const Point2f &sample) const {
            float cosTheta;
  
            float term = (1.f - (m_g * m_g)) / (1.f - m_g + 2.f * m_g * sample.y());
            cosTheta = -(1.f + (m_g * m_g) - term * term) / abs(2.f * m_g);
       

            float sinTheta = sqrt(std::max(0.f, 1.f - cosTheta * cosTheta));
            float phi = 2.f * M_PI * sample.x();


            wi = Vector3f(sinTheta * cos(phi), sinTheta * sin(phi), cosTheta);

            return INV_FOURPI * (1.f - (m_g * m_g)) / pow(1.f + (m_g * m_g) - 2.f * m_g * cosTheta, 3/2);
        }

        std::string toString() const {
            return tfm::format("[ HenyeyGreenstein g: %f ]", m_g);
        }


    private:
        float m_g;
};



NORI_REGISTER_CLASS(Isotropic, "isotropic");
NORI_REGISTER_CLASS(Henyey_Greenstein, "henyeygreenstein");
NORI_NAMESPACE_END

