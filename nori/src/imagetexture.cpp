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

#include <nori/object.h>
#include <nori/texture.h>
#include <filesystem/resolver.h>
#include <lodepng.h>

NORI_NAMESPACE_BEGIN

template <typename T>
class ImageTexture : public Texture<T> {
public:
    ImageTexture(const PropertyList& props);

    virtual std::string toString() const override;

    virtual T eval(const Point2f& uv) override {


        float u = uv.x() - std::floor(uv.x());
        float v = uv.y() - std::floor(uv.y());
        int x = std::floor(u * width), y = std::floor(v * height);
        if (x < 0) x = 0; if (x >= width) x = width - 1;
        
        if (y < 0) y = 0; if (y >= height) y = height - 1;
        y = height - 1 - y;


        float r = (float)image[4 * (y * width + x) + 0] / 255.0;
        float g = (float)image[4 * (y * width + x) + 1] / 255.0;
        float b = (float)image[4 * (y * width + x) + 2] / 255.0;

        //std::cout << result.x() << " " << result.y() << " " << result.z() << std::endl;
        if (raw)
            return Color3f(r, g, b);
        else
            return Color3f(r, g, b).toLinearRGB();
    }

protected:
    std::string filename;
    bool raw;
    std::vector<unsigned char> image;
    unsigned width, height;
};



template <>
ImageTexture<Color3f>::ImageTexture(const PropertyList& props) {
    filename = props.getString("filename");
    raw = props.getBoolean("raw", false);

    filesystem::path filename_full = getFileResolver()->resolve(props.getString("filename"));


    //decode
    unsigned error = lodepng::decode(image, width, height, filename_full.str());

    //if there's an error, display it
    if (error)
        std::cout << "decoder error " << error << ": " << lodepng_error_text(error) << std::endl;



}



template <>
std::string ImageTexture<Color3f>::toString() const {
    return tfm::format(
        "ImageTexture[filename = %s]\n",
        filename
    );
}

//template <>
//ImageTexture<float>::ImageTexture(const PropertyList &props) {
//    m_delta = props.getPoint2("delta", Point2f(0));
//    m_scale = props.getVector2("scale", Vector2f(1));
//    m_value1 = props.getFloat("value1", 0.f);
//    m_value2 = props.getFloat("value2", 1.f);
//}

//template <>
//std::string ImageTexture<float>::toString() const {
//    return tfm::format(
//        "Checkerboard[\n"
//                "  delta = %s,\n"
//                "  scale = %s,\n"
//                "  value1 = %f,\n"
//                "  value2 = %f,\n"
//                "]",
//        m_delta.toString(),
//        m_scale.toString(),
//        m_value1,
//	    m_value2
//    );
//}

NORI_REGISTER_TEMPLATED_CLASS(ImageTexture, Color3f, "imagetexture")
NORI_NAMESPACE_END
