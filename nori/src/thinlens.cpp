/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob, Romain Prévost

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

#include <nori/camera.h>
#include <nori/rfilter.h>
#include <nori/warp.h>
#include <Eigen/Geometry>
#include <nori/texture.h>

NORI_NAMESPACE_BEGIN

/**
 * \brief Perspective camera with depth of field
 *
 * This class implements a simple perspective camera model. It uses an
 * infinitesimally small aperture, creating an infinite depth of field.
 */
class ThinLensCamera : public Camera {
public:
    ThinLensCamera(const PropertyList &propList) {
        /* Width and height in pixels. Default: 720p */
        m_outputSize.x() = propList.getInteger("width", 1280);
        m_outputSize.y() = propList.getInteger("height", 720);
        m_invOutputSize = m_outputSize.cast<float>().cwiseInverse();

        /* Specifies an optional camera-to-world transformation. Default: none */
        m_cameraToWorld = propList.getTransform("toWorld", Transform());
        useMotionBlur = propList.getBoolean("motionBlur", false);
        if (useMotionBlur)
            m_cameraToWorldEnd = propList.getTransform("toWorldEnd");

        /* Horizontal field of view in degrees */
        m_fov = propList.getFloat("fov", 30.0f);

        /* Near and far clipping planes in world-space units */
        m_nearClip = propList.getFloat("nearClip", 1e-4f);
        m_farClip = propList.getFloat("farClip", 1e4f);
        m_lensRadius = propList.getFloat("lensRadius", 0.0f);
        m_focalDistance = propList.getFloat("focalDistance", 0.0f);
        
        m_caCoef = propList.getFloat("chromaticAberration", -1.0f);
        if (m_caCoef > 0.0f)
            use_ca = true;

        m_rfilter = NULL;
    }
    virtual ~ThinLensCamera() {
        if (m_apertureKernel == nullptr)
            delete m_apertureKernel;
    }

    virtual void activate() override {
        float aspect = m_outputSize.x() / (float) m_outputSize.y();

        /* Project vectors in camera space onto a plane at z=1:
         *
         *  xProj = cot * x / z
         *  yProj = cot * y / z
         *  zProj = (far * (z - near)) / (z * (far-near))
         *  The cotangent factor ensures that the field of view is 
         *  mapped to the interval [-1, 1].
         */
        float recip = 1.0f / (m_farClip - m_nearClip),
              cot = 1.0f / std::tan(degToRad(m_fov / 2.0f));

        Eigen::Matrix4f perspective;
        perspective <<
            cot, 0,   0,   0,
            0, cot,   0,   0,
            0,   0,   m_farClip * recip, -m_nearClip * m_farClip * recip,
            0,   0,   1,   0;

        /**
         * Translation and scaling to shift the clip coordinates into the
         * range from zero to one. Also takes the aspect ratio into account.
         */
        m_sampleToCamera = Transform( 
            Eigen::DiagonalMatrix<float, 3>(Vector3f(0.5f, -0.5f * aspect, 1.0f)) *
            Eigen::Translation<float, 3>(1.0f, -1.0f/aspect, 0.0f) * perspective).inverse();

        /* If no reconstruction filter was assigned, instantiate a Gaussian filter */
        if (!m_rfilter) {
            m_rfilter = static_cast<ReconstructionFilter *>(
                    NoriObjectFactory::createInstance("gaussian", PropertyList()));
            m_rfilter->activate();
        }
    }

    Color3f sampleRay(Ray3f &ray,
            const Point2f &samplePosition,
            Sampler *sampler,
            const float sampleTime = 0.0f,
            int channel = -1) const {

        /* Compute the corresponding position on the 
           near plane (in local camera space) */
        Point3f nearP = m_sampleToCamera * Point3f(
            samplePosition.x() * m_invOutputSize.x(),
            samplePosition.y() * m_invOutputSize.y(), 0.0f);

        /* Turn into a normalized ray direction, and
           adjust the ray interval accordingly */
        ray.o = Point3f(0, 0, 0);
        ray.d = nearP.normalized();
        


        if (m_lensRadius > 0) {

            Point2f pLens;
            if (m_apertureKernel == nullptr) // sphere aperture
                pLens = m_lensRadius * Warp::squareToUniformDisk(sampler->next2D());
            else
            {
                while (true)
                {
                    /// Uniformly sample a vector on a 2D disk with radius 1, centered around the origin
                    Point2f cur_sample = sampler->next2D();
                    Color3f color = m_apertureKernel->eval(cur_sample);
                    if (color.maxCoeff() > 0.9f)
                    {
                        pLens = m_lensRadius * (cur_sample - Point2f(0.5f));
                        break;
                    }
                     
                }

            }

            if (use_ca)
            {   
                float delta_x = samplePosition.x() * m_invOutputSize.x() - 0.5f;
                float delta_y = samplePosition.y() * m_invOutputSize.y() - 0.5f;

                
                if (channel == 0) // red
                {
                    pLens.x() += delta_x * m_caCoef;
                    pLens.y() += -delta_y * m_caCoef;
                }
                else if (channel == 1) // green
                {
                    // do nothing
                }
                else if (channel == 2) // blue
                {
                    pLens.x() -= delta_x * m_caCoef;
                    pLens.y() -= -delta_y * m_caCoef;
                }
            }

            float ft = m_focalDistance / ray.d.z();
            Point3f pFocus = ray(ft);

            ray.o = Point3f(pLens.x(), pLens.y(), 0);
            ray.d = (pFocus - ray.o).normalized();
        }

        float invZ = 1.0f / ray.d.z();
        if (!useMotionBlur)
        {
            ray.o = m_cameraToWorld * ray.o;
            ray.d = m_cameraToWorld * ray.d;
        }
        else
        {
            Transform trans(m_cameraToWorld.getMatrix() * (1.0 - sampleTime) + m_cameraToWorldEnd.getMatrix() * sampleTime);
            ray.o = trans * ray.o;
            ray.d = trans * ray.d;
        }


        ray.mint = m_nearClip * invZ;
        ray.maxt = m_farClip * invZ;
        ray.update();

        if (!use_ca)
            return Color3f(1.0f);
        else if (channel == -1)
            return Color3f(1.0f);
        else if (channel == 0)
            return Color3f(1.0f, 0.0f, 0.0f);
        else if (channel == 1)
            return Color3f(0.0f, 1.0f, 0.0f);
        else if (channel == 2)
            return Color3f(0.0f, 0.0f, 1.0f);


    }

    virtual void addChild(NoriObject *obj) override {
        switch (obj->getClassType()) {
            case EReconstructionFilter:
                if (m_rfilter)
                    throw NoriException("Camera: tried to register multiple reconstruction filters!");
                m_rfilter = static_cast<ReconstructionFilter *>(obj);
                break;
            case ETexture:
                if (obj->getIdName() == "apertureKernel") {
                    if (m_apertureKernel)
                        throw NoriException("There is already an albedo defined!");
                    m_apertureKernel = static_cast<Texture<Color3f>*>(obj);
                }
                break;
            default:
                throw NoriException("Camera::addChild(<%s>) is not supported!",
                    classTypeName(obj->getClassType()));
        }
    }

    /// Return a human-readable summary
    virtual std::string toString() const override {
        return tfm::format(
            "ThinLensCamera[\n"
            "  cameraToWorld = %s,\n"
            "  outputSize = %s,\n"
            "  fov = %f,\n"
            "  clip = [%f, %f],\n"
            "  lensRadius = %f, \n"
            "  focalDistance = %f, \n"  
            "  rfilter = %s\n"
            "]",
            indent(m_cameraToWorld.toString(), 18),
            m_outputSize.toString(),
            m_fov,
            m_nearClip,
            m_farClip,
            m_lensRadius,
            m_focalDistance,
            indent(m_rfilter->toString())
        );
    }
private:
    Vector2f m_invOutputSize;
    Transform m_sampleToCamera;
    Transform m_cameraToWorld;
    Transform m_cameraToWorldEnd;
    bool useMotionBlur;
    float m_fov;
    float m_nearClip;
    float m_farClip;
    float m_lensRadius;
    float m_focalDistance;

    Texture<Color3f>* m_apertureKernel = nullptr;
    float m_caCoef;

};

NORI_REGISTER_CLASS(ThinLensCamera, "thin_lens");
NORI_NAMESPACE_END
