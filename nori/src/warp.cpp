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

#include <nori/warp.h>
#include <nori/vector.h>
#include <nori/frame.h>
#include<math.h>
#include<iostream>
#include<nori/common.h>

NORI_NAMESPACE_BEGIN

Vector3f Warp::sampleUniformHemisphere(Sampler *sampler, const Normal3f &pole) {
    // Naive implementation using rejection sampling
    Vector3f v;
    do {
        v.x() = 1.f - 2.f * sampler->next1D();
        v.y() = 1.f - 2.f * sampler->next1D();
        v.z() = 1.f - 2.f * sampler->next1D();
    } while (v.squaredNorm() > 1.f);

    if (v.dot(pole) < 0.f)
        v = -v;
    v /= v.norm();

    return v;
}

Point2f Warp::squareToUniformSquare(const Point2f &sample) {
    return sample;
}

float Warp::squareToUniformSquarePdf(const Point2f &sample) {
    return ((sample.array() >= 0).all() && (sample.array() <= 1).all()) ? 1.0f : 0.0f;
}

Point2f Warp::squareToUniformDisk(const Point2f &sample) {
    //throw NoriException("Warp::squareToUniformDisk() is not yet implemented!");
    float r = sqrt(sample[0]);
    float theta = 2 * M_PI * sample[1];
    float x = r * cos(theta);
    float y = r * sin(theta);

    return Point2f(x, y);
}

float Warp::squareToUniformDiskPdf(const Point2f &p) {
    //throw NoriException("Warp::squareToUniformDiskPdf() is not yet implemented!");
    //float r = sqrt(p[0]);
    //return r/M_PI;
    if(sqrt(pow(p[0], 2) + pow(p[1], 2)) <= 1){
        return 1/M_PI;
    }
    else{
        return 0;
    }
}

Point3f Warp::squareToUniformCylinder(const Point2f &sample, float h, float r){
  float x = sample[0];
  float y = sample[1];
  // use random y for generation of wz from -h/2 to h/2
  float wz = y * h - h/2;
  //use the random x for generation of wx, wy: create an angle from x and project it on to x axis and y axis with sin, cos and then times r so if falls on a circle of radius r
  float wx = r * sin(x * 2 * M_PI);
  float wy = r * cos(x * 2 * M_PI);

  return Vector3f(wx, wy, wz);

}

/*float Warp::squareToUniformDiskCylinderPdf(const Point3f &p){
    float z = p[2];
    float y = p[1];
    float x = p[0];
    if(abs(z) <= 1 and (  pow(x, 2) + pow(y, 2) < 1 )){
        return 1/(6*pi);
    }

}*/

Vector3f Warp::squareToUniformSphereCap(const Point2f &sample, float cosThetaMax) {
    //throw NoriException("Warp::squareToUniformSphereCap() is not yet implemented!");
    float sinThetaMax = sqrt(1-cosThetaMax*cosThetaMax);
    //the height of the cylinder is 1-cosThetaMax and the raduis is sinThetaMax
    Point3f p = squareToUniformCylinder(sample, 1-cosThetaMax, sinThetaMax);
    float x = p[0];
    float y = p[1];
    float z = p[2];

    // shift the cylinder down so that the first z starts from 0
    z += (1-cosThetaMax)/2 + cosThetaMax;
    //calculated the r of the circle at the height z
    return Vector3f(x * sqrt(1-pow(z,2)), y * sqrt(1-pow(z,2)), z);


}

float Warp::squareToUniformSphereCapPdf(const Vector3f &v, float cosThetaMax) {
    //throw NoriException("Warp::squareToUniformSphereCapPdf() is not yet implemented!");
    float z = v[2] ;//+ cosThetaMax;
    if(z > cosThetaMax && ( sqrt(pow(v[0], 2) + pow(v[1],2) + pow(z, 2))  <= Epsilon+1)){
        return 1/(2 * M_PI * (1-cosThetaMax));
    }
    return 0;
    //return 
}

Vector3f Warp::squareToUniformSphere(const Point2f &sample) {
    //throw NoriException("Warp::squareToUniformSphere() is not yet implemented!");
    /*float x = sample[0];
    float y = sample[1];
    float theta = acos(2 * x - 1);
    float phi = 2 * M_PI * y;

    float wx = sin(theta) * cos(phi);
    float wy = sin(theta) * sin(phi);
    float wz = cos(theta);*/
    //projecting the cylinder distribution onto a sphere
    Point3f s = squareToUniformCylinder(sample, 2, 1);
    float x = s[0];
    float y = s[1];
    float z = s[2];
    //the r of each circle on the sphere is shrinked
    float shrinkFac = sqrt(1-pow(z,2));

    return Vector3f(x * shrinkFac, y * shrinkFac, z);

    /*
    float x = sample[0];
    float y = sample[1];
    float wz = 2 * x - 1;
    float r = sqrt((1-pow(wz,2)));
    float phi = 2 * M_PI * y;
    float wx = r * cos(phi);
    float wy = r * sin(phi);*/

}

float Warp::squareToUniformSpherePdf(const Vector3f &v) {
    //throw NoriException("Warp::squareToUniformSpherePdf() is not yet implemented!");
    //{
        //std::cout << sqrt(pow(v[0], 2) + pow(v[1], 2) + pow(v[2], 2)) << endl;
    if(sqrt((pow(v[0], 2) + pow(v[1], 2) + pow(v[2], 2))) <= 1 + Epsilon){
        return 1/(4 * M_PI);
    }
    else{
        return 0;
    }    
}

Vector3f Warp::squareToUniformHemisphere(const Point2f &sample) {
    //throw NoriException("Warp::squareToUniformHemisphere() is not yet implemented!");
    //the height of the cylinder is 1
     Point3f hemPoints = squareToUniformCylinder(sample, 1 , 1);
     float x = hemPoints[0];
     float y = hemPoints[1];
     float z = hemPoints[2];

    //BRING THE cylindre higher so the z starts from 0
     z += 0.5;
     return Vector3f(x * sqrt(1-z*z), y * sqrt(1-z*z), z);

}

float Warp::squareToUniformHemispherePdf(const Vector3f &v) {
    //throw NoriException("Warp::squareToUniformHemispherePdf() is not yet implemented!");
    if(((  pow(v[0], 2) + pow(v[1], 2) + pow(v[2], 2)  ) <= (1 + Epsilon)) && v[2] > 0){
        return 1/(2 * M_PI);
    }    
    return 0; 

}

Vector3f Warp::squareToCosineHemisphere(const Point2f &sample) {
    //throw NoriException("Warp::squareToCosineHemisphere() is not yet implemented!");
    //will project the points generated on a disk onto the hemisphere
    Point2f point = squareToUniformDisk(sample);
    float x = point[0];
    float y = point[1];

    //so just need to find the corresspoding height for given x and y
    float dist = sqrt(pow(x,2) + pow(y,2));
    float z = sqrt(1-dist*dist);
    return Vector3f(x, y, z);

}

float Warp::squareToCosineHemispherePdf(const Vector3f &v) {
    //throw NoriException("Warp::squareToCosineHemispherePdf() is not yet implemented!");
    // if the point is given on the surface of the hemisphere then the cos density is returned
    float cosTheta = v[2]/1;
    if(((  pow(v[0], 2) + pow(v[1], 2) + pow(v[2], 2)  ) <= (1 + Epsilon)) && v[2] > 0){
        return cosTheta/M_PI;
    }    
    return 0; 

}

Vector3f Warp::squareToBeckmann(const Point2f &sample, float alpha) {
    //throw NoriException("Warp::squareToBeckmann() is not yet implemented!");
    // the tantheta is found by finding the probability density for the beckmann and then inverting 
    float tanTheta = sqrt(-1 * alpha * alpha * log(1-sample[0]));
    // the density function is not dependant on the phi so we suppose a uniform density
    float phi = 2 * M_PI * sample[1];
    float theta = atan(tanTheta);
    return sphericalDirection(theta, phi);
}

float Warp::squareToBeckmannPdf(const Vector3f &m, float alpha) {
    //throw NoriException("Warp::squareToBeckmannPdf() is not yet implemented!");
    Point2f s = sphericalCoordinates(m);
    float theta = s[0];
    float phi = s[1];

    if(m[2] > 0 && sqrt(pow(m[0],2) +  pow(m[1],2) + pow(m[2],2) ) <= 1 + Epsilon){
        return (exp(-1 * tan(theta) * tan(theta) / (alpha * alpha)) ) / (M_PI * alpha * alpha * pow(cos(theta), 3));
    }
    return 0;
    

}

Vector3f Warp::squareToUniformTriangle(const Point2f &sample) {
    float su1 = sqrtf(sample.x());
    float u = 1.f - su1, v = sample.y() * su1;
    return Vector3f(u,v,1.f-u-v);
}


NORI_NAMESPACE_END
