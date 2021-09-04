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

#include <nori/shape.h>
#include <nori/bsdf.h>
#include <nori/emitter.h>
//#include <nori/warp.h>
#include <Eigen/Geometry>

NORI_NAMESPACE_BEGIN

Shape::~Shape() {
    delete m_bsdf;
    //delete m_emitter; // scene is responsible for deleting the emitter
}

void Shape::activate() {
    if (!m_bsdf) {
        /* If no material was assigned, instantiate a diffuse BRDF */
        m_bsdf = static_cast<BSDF *>(
            NoriObjectFactory::createInstance("diffuse", PropertyList()));
        m_bsdf->activate();
    }
}

void Shape::addChild(NoriObject *obj) {
    switch (obj->getClassType()) {
        case EBSDF:
            if (m_bsdf)
                throw NoriException(
                    "Shape: tried to register multiple BSDF instances!");
            m_bsdf = static_cast<BSDF *>(obj);
            break;

        case EEmitter:
            if (m_emitter)
                throw NoriException(
                    "Shape: tried to register multiple Emitter instances!");
            m_emitter = static_cast<Emitter *>(obj);
            m_emitter->setShape(static_cast<Shape*>(this));
            break;

        case ETexture:
            if (m_normalmap)
                throw NoriException(
                    "Shape: tried to register multiple NormalMap instances!");
            m_normalmap =  static_cast<Texture<Color3f>*>(obj);
            break;

        default:
            throw NoriException("Shape::addChild(<%s>) is not supported!",
                                classTypeName(obj->getClassType()));
    }
}
/*
void Shape::applyNormalMap(Intersection& its, Vector3f & tangent) const {
    Vector3f bitangent = -its.geoFrame.n.cross(tangent);
    Frame frame(tangent, bitangent, its.geoFrame.n);
    Color3f normal_color = m_normalmap->eval(its.uv);
    Vector3f normal(normal_color.x(), normal_color.y(), normal_color.z());
    normal = (normal * 2.0 - Vector3f(1.0)).normalized();


    its.shFrame.n = frame.toWorld(normal);
}

void Shape::applyNormalMap(Intersection& its, Vector3f & tangent) const {
    Vector3f n_vec = Vector3f(its.geoFrame.n.x(), its.geoFrame.n.y(), its.geoFrame.n.z());
    Vector3f bitangent = -n_vec.cross(tangent);
    Frame frame(tangent, bitangent, its.geoFrame.n);
    Color3f normal_color = m_normalmap->eval(its.uv);
    Vector3f normal(normal_color.x(), normal_color.y(), normal_color.z());
    normal = (normal * 2.0 - Vector3f(1.0)).normalized();


    its.shFrame.n = frame.toWorld(normal);
}
*/

void Shape::applyNormalMap(Intersection& its, Vector3f & tangent) const {
    Vector3f n_vec = Vector3f(its.geoFrame.n.x(), its.geoFrame.n.y(), its.geoFrame.n.z());
    Vector3f bitangent = -n_vec.cross(tangent);

    Frame frame(tangent, bitangent, n_vec);
    Color3f normal_color = m_normalmap->eval(its.uv);
    Vector3f normal(normal_color.x(), normal_color.y(), normal_color.z());
    normal = (normal * 2.0 - Vector3f(1.0)).normalized();


    its.shFrame.n = frame.toWorld(normal);
}
std::string Intersection::toString() const {
    if (!mesh)
        return "Intersection[invalid]";

    return tfm::format(
        "Intersection[\n"
        "  p = %s,\n"
        "  t = %f,\n"
        "  uv = %s,\n"
        "  shFrame = %s,\n"
        "  geoFrame = %s,\n"
        "  mesh = %s\n"
        "]",
        p.toString(),
        t,
        uv.toString(),
        indent(shFrame.toString()),
        indent(geoFrame.toString()),
        mesh ? mesh->toString() : std::string("null")
    );
}

NORI_NAMESPACE_END
