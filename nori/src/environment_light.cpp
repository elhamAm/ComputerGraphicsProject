#include <nori/emitter.h>
#include <nori/bitmap_light_env.h>
#include <nori/frame.h>
#include <nori/shape.h>


NORI_NAMESPACE_BEGIN

class Environment_Light : public Emitter {
public:
    Environment_Light(const PropertyList &props) {
    	m_bitmap = Bitmap_Light_Env(props.getString("file_path"));
        //cout << m_bitmap << endl;
    }

    virtual std::string toString() const override {
        return tfm::format(
                "environment_map"
        		);
    }

    virtual Color3f eval(const EmitterQueryRecord & lRec) const override {
    	return m_bitmap.eval(lRec.wi);
    }

    virtual Color3f sample(EmitterQueryRecord & lRec, const Point2f & sample) const override {
        //cout << "insample1" << endl;
    	Color3f col = m_bitmap.sample(lRec.wi, sample);
        //cout << "insample2" << endl;
    	lRec.shadowRay = Ray3f(lRec.ref, lRec.wi, Epsilon, 10000);
        //cout << "insample3" << endl;
    	return col;
    }

    virtual float pdf(const EmitterQueryRecord &lRec) const override {
    	return m_bitmap.pdf(lRec.wi);
    }

private:
    Bitmap_Light_Env m_bitmap;
};

NORI_REGISTER_CLASS(Environment_Light, "environment_map")
NORI_NAMESPACE_END
