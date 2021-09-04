#if !defined(__NORI_DENOISING_H)
#define __NORI_DENOISING_H

#include <nori/object.h>
#include <nori/scene.h>
#include <nori/shape.h>
#include <nori/bsdf.h>
#include <nori/warp.h>
#include <nori/block.h>

NORI_NAMESPACE_BEGIN

struct FeatureSet
{
    Color3f normal;
    Color3f texture;
    float depth;
    float visibility;
};

class FeatureVarianceBlock
{
public:
    ImageBlock block1;
    ImageBlock block2;
    ImageBlock variance_block;
    int count;

    FeatureVarianceBlock(const Vector2i& size, const ReconstructionFilter* filter) :
        block1(size, filter),
        block2(size, filter),
        variance_block(size, filter){
        count = 0;
    }
};


class FeatureGatherer
{
public:
    FeatureGatherer();
    ~FeatureGatherer();

    static FeatureSet gather(const Scene* scene, Sampler* sampler, const Ray3f& ray) {
        /* Find the surface that is visible in the requested direction */

        FeatureSet result = { Color3f(0), Color3f(0), 0, 0 };
        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return result;

        // normal remapped to 0,1
        result.normal = Color3f(its.shFrame.n.x(), its.shFrame.n.y(), its.shFrame.n.z());
        result.normal = (result.normal + 1.0) / 2.0;

        // texture
        BSDFQueryRecord bRec(its.shFrame.toLocal(-ray.d), its.shFrame.toLocal(-ray.d), ESolidAngle);
        bRec.uv = its.uv;
        
        // WARNING: assuming only have diffuse bsdf 
        // TODO: change to mean of BRDF
        assert(its.mesh->getBSDF()->isDiffuse());
        result.texture = its.mesh->getBSDF()->eval(bRec) * M_PI; // eval = albedo / pi

        //depth
        result.depth = (its.p - ray.o).norm();

        //visibility
        EmitterQueryRecord lRec(its.p);
        const Emitter* emitter_ptr = scene->getRandomEmitter(sampler->next1D());
        Color3f L_d = emitter_ptr->sample(lRec, sampler->next2D());
        // sampling fail, continue
        if (L_d.isZero() || !L_d.isValid() || scene->rayIntersect(lRec.shadowRay))
            result.visibility = 0;
        else
            result.visibility = 1;

        return result;

    }

private:

};



NORI_NAMESPACE_END

#endif /* __NORI_DENOISING_H */
