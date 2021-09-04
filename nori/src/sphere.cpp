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
#include <nori/warp.h>
#include <math.h>
#include<iostream>

NORI_NAMESPACE_BEGIN

class Sphere : public Shape {
public:
    Sphere(const PropertyList & propList) {
        m_position = propList.getPoint3("center", Point3f());
        m_radius = propList.getFloat("radius", 1.f);

        m_bbox.expandBy(m_position - Vector3f(m_radius));
        m_bbox.expandBy(m_position + Vector3f(m_radius));
    }

    virtual BoundingBox3f getBoundingBox(uint32_t index) const override { return m_bbox; }

    virtual Point3f getCentroid(uint32_t index) const override { return m_position; }

    virtual bool rayIntersect(uint32_t index, const Ray3f &ray, float &u, float &v, float &t) const override {

	/* to be implemented */

        //finding the t for which the ray intersects the sphere
        float D = (m_position - ray.o).squaredNorm() - pow((m_position - ray.o).transpose() * ray.d, 2);
        //then there is no intersection if D is outside the sphere
        if(D > pow(m_radius,2)){
            return false;
        }

        //t2 is smaller since the second term is always positive
        float t1 = (m_position - ray.o).transpose() * ray.d + sqrt(pow(m_radius,2) - D);
        float t2 = (m_position - ray.o).transpose() * ray.d - sqrt(pow(m_radius,2) - D);
 
        //finding the closest root which is contained between the the min and max time
        //cout << "t1: " << t1 << " t2: " << t2 << endl;
        if(ray.mint <= t2 && ray.maxt >= t2 && t2 >= 0){
            t = t2;
        }
        else if(ray.mint <= t1 && ray.maxt >= t1 && t1 >= 0){
            t = t1;
        }
        //the case when it is occluded
        else{
            return false;
        }

        return true;

    }

    virtual void setHitInformation(uint32_t index, const Ray3f &ray, Intersection & its, Vector3f & tangent) const override {
        /* to be implemented */
        float u, v, t;
        if(rayIntersect(index, ray, u, v, t)){
            //finding the point of intersection by replacing t in the ray equation
            its.p = ray.o + t * ray.d;
            //the vector that is perpendicular to the intersection point on the sphere
            Vector3f normOnSphere = its.p-m_position;
            //creating a frame made out of 3 normal vectors one being the normal shade vector
            its.geoFrame = Frame(normOnSphere.normalized());
            its.shFrame = its.geoFrame;
            //finding the sphere coordiantes for the hit point
            //its.uv[0] =  std::atan(its.p[1]/ its.p[0]) ;
            //its.uv[1] = std::acos(its.p[2]/ m_radius) ;
            its.uv = sphericalCoordinates((its.p - m_position)/(its.p - m_position).norm());

            //have to fit the coordiantes in 0,1 interval
            // atan returns double between [-pi, pi]
            its.uv[0] = (its.uv[0] + M_PI) / (2 * M_PI);
            // acos return double between [0,pi]
            its.uv[1] = (its.uv[1]) / M_PI;



        }
        
    }

    virtual void sampleSurface(ShapeQueryRecord & sRec, const Point2f & sample) const override {
        Vector3f q = Warp::squareToUniformSphere(sample);
        sRec.p = m_position + m_radius * q;
        sRec.n = q;
        sRec.pdf = std::pow(1.f/m_radius,2) * Warp::squareToUniformSpherePdf(Vector3f(0.0f,0.0f,1.0f));
    }
    virtual float pdfSurface(const ShapeQueryRecord & sRec) const override {
        return std::pow(1.f/m_radius,2) * Warp::squareToUniformSpherePdf(Vector3f(0.0f,0.0f,1.0f));
    }


    virtual std::string toString() const override {
        return tfm::format(
                "Sphere[\n"
                "  center = %s,\n"
                "  radius = %f,\n"
                "  bsdf = %s,\n"
                "  emitter = %s\n"
                "]",
                m_position.toString(),
                m_radius,
                m_bsdf ? indent(m_bsdf->toString()) : std::string("null"),
                m_emitter ? indent(m_emitter->toString()) : std::string("null"));
    }

protected:
    Point3f m_position;
    float m_radius;
};

NORI_REGISTER_CLASS(Sphere, "sphere");
NORI_NAMESPACE_END
