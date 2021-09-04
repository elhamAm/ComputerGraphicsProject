#if !defined(__NORI_ENVMAP_H)
#define __NORI_ENVMAP_H

#include <nori/bitmap.h>
#include <nori/frame.h>
#include <nori/object.h>

typedef Eigen::Array<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> floatmat;

NORI_NAMESPACE_BEGIN

class Bitmap_Light_Env : public Bitmap{
public:

    Bitmap_Light_Env(const std::string &path);
    Bitmap_Light_Env();

    Color3f sample(Vector3f &dir, const Point2f & sample) const;
    Color3f eval(const Vector3f &dir) const;
    float pdf(const Vector3f &dir) const;
    float pdfOfPixel(const Point2f &point) const;
    Point2f sampleAPixel(const Point2f &sample) const;
    Color3f getPixelColor(const Point2f &point) const;
    Vector3f pixelToDir(const Point2f &p) const;
    Point2f dirToPixel(const Vector3f &vec) const;
    float preCompute1D(int row, const floatmat &f, floatmat &pf, floatmat &Pf) const; 
    void sample1D(int row, const floatmat &pf, const floatmat &Pf, const float &unif, float &x, float &p) const;
    Color3f bilinearInterpolation(float x2x1, float y2y1, float x2x, float xx1, float yy1, float y2y, Color3f Q11, Color3f Q21, Color3f Q12, Color3f Q22) const;
    Color3f linearInterpolation(float rand, Color3f v1, Color3f v2) const;

private:
    floatmat m_Lenv;
    floatmat m_Pdf_cond;
    floatmat m_Cdf_cond;
    floatmat m_Pdf;
    floatmat m_Cdf;
};


NORI_NAMESPACE_END

#endif /* __NORI_ENVMAP_H */