//based on pbr book and spot.cpp https://github.com/mmp/pbrt-v3/blob/master/src/lights/spot.cpp
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>
#include <nori/emitter.h>

NORI_NAMESPACE_BEGIN

class spotLight : public Emitter{
public:
	spotLight(const PropertyList &propList) {
		m_radiance = propList.getColor("radiance");
		position = propList.getPoint3("position");
		direction = propList.getVector3("direction");
		fallOffStart = (M_PI * propList.getFloat("fallOffStart") / 180);
		totalWidth = (M_PI * propList.getFloat("totalWidth") / 180);

	}	

	virtual std::string toString() const override {
		return "spotLight[]";
	}

	virtual Color3f sample(EmitterQueryRecord &lRec, const Point2f &sample)const override {
		
		lRec.p = position;
		lRec.wi = (position - lRec.ref).normalized();
		lRec.n = direction.normalized();

		Ray3f shadowRay(lRec.ref, lRec.wi / lRec.wi.norm(), Epsilon, (position - lRec.ref).norm() -Epsilon);
		lRec.shadowRay = shadowRay;
		return eval(lRec) / pdf(lRec);
	}

	virtual float fallOff(float cosTheta, float fallOffStart, float totalWidth) const {
		if (cosTheta < std::cos(totalWidth)) {
			return 0;
		}
		else if (cosTheta > std::cos(fallOffStart)) {
			return 1;
		}
		else {
			float delta = (cosTheta - std::cos(totalWidth)) / (std::cos(fallOffStart) - std::cos(totalWidth));
			return delta*delta*delta*delta;				
		}
	}

	virtual Color3f eval(const EmitterQueryRecord &lRec) const override {
		float cosTheta = direction.normalized().dot((lRec.ref - position).normalized());
		float fA = fallOff(cosTheta, fallOffStart, totalWidth);
		return m_radiance * fA / (2.0f * M_PI) * (1.0f - 0.5f*(std::cos(fallOffStart) + std::cos(totalWidth)));
	}

	virtual float pdf(const EmitterQueryRecord &lRec)const override {
		float pdfVal = Warp::squareToUniformSphereCapPdf(Vector3f(0,0,1),std::cos(totalWidth));
		float cosThetaLight = abs(lRec.n.dot(-lRec.wi));
		return pdfVal*(lRec.p - lRec.ref).squaredNorm() / cosThetaLight;
	}

private:
	Point3f position;
	Color3f m_radiance;
	Vector3f direction;
	float totalWidth;
	float fallOffStart;
};

NORI_REGISTER_CLASS(spotLight, "spotlight");
NORI_NAMESPACE_END