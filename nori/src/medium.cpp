#include <nori/medium.h>



NORI_NAMESPACE_BEGIN

Medium::Medium(const PropertyList &props) {
    m_sigma_a = (props.getColor("sigma_a", Color3f(0.2f)));
    m_sigma_s = (props.getColor("sigma_s", Color3f(0.2f)));
    m_sigma_t = m_sigma_a + m_sigma_s;
    m_albedo = m_sigma_s / m_sigma_t;
    Vector3f lengthFromCenter = props.getVector3("lengthFromCenter", Vector3f(0.4)).cwiseAbs();
    Vector3f center = props.getVector3("center", Vector3f(0.f));
    aabb = BoundingBox3f(center - lengthFromCenter, center + lengthFromCenter);
}

//Homogeneous transmission
Color3f Medium::transmittance(const Point3f a, const Point3f b) const {

    float norm = (a-b).norm();
    Color3f trans(0.0f);
	for (int i = 0; i < 3; i++){
		trans[i] = std::exp(-m_sigma_t[i] * norm);
	}
	return trans;
}


Color3f Medium::sample(const Ray3f &ray, Sampler *sampler, MediumQueryRecord &mRec) const
	{

		int i = floor(sampler->next1D() * 3.0f);
		if (i == 3) i = 1;
		float sampled_sigma_t = m_sigma_t[i];
		float sampled_distance = -log(1 - sampler->next1D()) / sampled_sigma_t;
		bool status;
        if (sampled_distance > mRec.maxDistance /*|| !aabb.rayIntersect(Ray3f(ray(sampled_distance), ray.d))*/) {
            mRec.success = false;
            return 1;
        }
        //mRec.p = ray(sampled_distance);
        mRec.sampleDist = sampled_distance;
        mRec.success = true;
        return m_albedo;
        
	}


void Medium::addChild(NoriObject *child) {
    switch (child->getClassType()) {
        case EPhaseFunction:
            if (m_phaseFunction) {
                throw NoriException("Medium: tried register multiple phaseFunctions");
            }
            m_phaseFunction = static_cast<PhaseFunction*>(child);
            break;
        default:
            throw NoriException("Medium: addChild is not supported other than phaseFunction");
    }
}


std::string Medium::toString() const {
    return "medium";
}

NORI_REGISTER_CLASS(Medium, "medium")
NORI_NAMESPACE_END