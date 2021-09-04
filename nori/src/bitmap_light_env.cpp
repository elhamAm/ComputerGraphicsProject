//https://web.cs.wpi.edu/~emmanuel/courses/cs563/S07/projects/envsample.pdf
#include "nori/bitmap_light_env.h"
#include<algorithm>

NORI_NAMESPACE_BEGIN

Bitmap_Light_Env::Bitmap_Light_Env(const std::string &path): Bitmap(path){

	m_Lenv = floatmat(rows(), cols());
	m_Pdf = floatmat(rows(), cols());
	m_Cdf = floatmat(rows(), cols() + 1);
	m_Pdf_cond = floatmat(1, rows());
	m_Cdf_cond = floatmat(1, rows() + 1);


	for (int i = 0; i < rows(); ++i) {
		for (int j = 0; j < cols(); ++j) {
			Color3f color = (*this)(i,j);
			m_Lenv(i,j) = .3 * color.x() + .6 * color.y() + .1 * color.z() + Epsilon/10000000;
			//cout << "i: " << i << "j: " << j << " lum: " << m_Lenv(i, j) << " "; 
			if(isinf(m_Lenv(i,j)))
				m_Lenv(i,j) = Epsilon;
		}
		//cout << endl;
	}

	floatmat colsum(1, rows());
	for (int i = 0; i < m_Pdf.rows(); ++i) {
		colsum(0, i) = preCompute1D(i, m_Lenv, m_Pdf, m_Cdf);
	}
	//cout << "luminance: " << endl;
	for (int i = 0; i < rows(); ++i) {
		for (int j = 0; j < cols(); ++j) {
			//cout << "i: " << i << "j: " << j << " lum: " << m_Lenv(i, j) << " "; 

		}

		//cout << endl;
	}
	/*cout << "---------------------------------------------------------------------------" << endl;
	for (int i = 0; i < rows(); ++i) {
		for (int j = 0; j < cols(); ++j) {
			//cout << "i: " << i << "j: " << j << " pdf: " << m_Pdf(i, j) << " "; 
			//cout << "i: " << i << "j: " << j << " cdf: " << m_Cdf(i, j) << " ";
		}

		//cout << endl;
	}*/

	preCompute1D(0, colsum, m_Pdf_cond, m_Cdf_cond);
}

Bitmap_Light_Env::Bitmap_Light_Env()
	:Bitmap()
{}


                                    /*************************************/
                                    /******                         ******/
                                    /******      Emitter Methods    ******/
                                    /******                         ******/
                                    /*************************************/

Color3f Bitmap_Light_Env::sample(Vector3f &dir, const Point2f & sample) const {
	//cout << "samplesample1" << endl;
	Point2f uv = sampleAPixel(sample);
	//cout << "samplesample2" << endl;
	dir = pixelToDir(uv);
	//cout << "samplesample3" << endl;
	// paper
	float jacobian = (cols() - 1) * (rows() - 1) / (2 * std::pow(M_PI, 2) * Frame::sinTheta(dir));
	//cout << "cols(): " << cols() - 1 << "dir: " << dir << endl;
	//cout << "jac: "<<jacobian<< endl;
	//cout << "samplesample4" << endl;
	//cout << "uv: " << uv << endl;
	//cout << "pdf: " << pdfOfPixel(uv) << endl;
	return getPixelColor(uv) / (pdfOfPixel(uv) * jacobian);
}

Color3f Bitmap_Light_Env::eval(const Vector3f & dir) const {
	Point2f uv = dirToPixel(dir);
	return getPixelColor(uv);
}

float Bitmap_Light_Env::pdf(const Vector3f &dir) const {
	Point2f uv = dirToPixel(dir);
	return pdfOfPixel(uv);
}

//drawing samples
Point2f Bitmap_Light_Env::sampleAPixel(const Point2f &sample)const {
	//cout << "samplpixel1" << endl;
	float u, v, pdf_u, pdf_v;
    // p and x are filled out
    //sample1D(int row, const floatmat &pdf, const floatmat &cdf, const float &unif, float &x, float &p)
	//cout << "samplepixel2" << endl;
	sample1D(0, m_Pdf_cond, m_Cdf_cond, sample.x(), u, pdf_u);
	//cout << "samplepixel3" << endl;
	sample1D(std::min(static_cast<int>(round(u)), static_cast<int>(rows() - 1) ), m_Pdf, m_Cdf, sample.y(), v, pdf_v);
	//cout << "samplepixel4" << endl;
	return Point2f(std::min(u, (float) rows() - 1 ), v);
}

float Bitmap_Light_Env::pdfOfPixel(const Point2f &point)const {
	int xx = floor(point.x());
	int yy = floor(point.y());
	if(xx >= rows()) xx = rows()-1;
	if(yy >= cols()) yy = cols()-1;
	if(xx < 0) xx = 0;
	if(yy < 0) yy = 0;
	return m_Pdf_cond(0, xx) * m_Pdf(xx, yy);
}

Color3f Bitmap_Light_Env::getPixelColor(const Point2f &point)const {
	int u = floor(point.x());
	int v = floor(point.y());

	if (u >= rows() - 1 || v >= cols() - 1) {
		return (*this)(u,v);
	}
	//wiki
	//Q11 : u v
	Color3f Q11 = Color3f(0);
	if(u >= 0 && u < rows() && v >= 0 && v < cols())
		Q11 = (*this)(u, v);
	//Q21 u+1 v
	Color3f Q21 = Color3f(0);
	if((u+1) >= 0 && (u+1) < rows() && v >= 0 && v < cols())
		Q21 = (*this)(u+1, v);
	//Q12 u v+1
	Color3f Q12 = Color3f(0);
	if(u >= 0 && u < rows() && (v+1) >= 0 && (v+1) < cols())
		Q12 = (*this)(u, v+1);
	//Q22 u+1 v+1
	Color3f Q22 =Color3f(0);
	if((u+1) >= 0 && (u+1) < rows() && (v+1) >= 0 && (v+1) < cols())
		Q22 = (*this)(u+1, v+1);

	float x2x1 = 1;
	float y2y1 = 1;
	float x2x = (u + 1) - point.x();
	float xx1 = point.x() - u;
	float yy1 = point.y() - v;
	float y2y = (v + 1) - point.y();

	//cout << "coolor of pixel" << " Q11: " << Q11 << " Q21: " << Q21 << " Q12: " << Q12 << " Q22: " << Q22 << endl;
	return bilinearInterpolation(x2x1, y2y1, x2x, xx1, yy1, y2y, Q11, Q21, Q12, Q22);
}

Point2f Bitmap_Light_Env::dirToPixel(const Vector3f &dir) const{
	Point2f uv = sphericalCoordinates(dir);
	float theta = uv.x();
	float phi = uv.y();
	float u = theta * INV_PI * (rows() - 1);
	float v = phi  * INV_PI * 0.5 * (cols() - 1);
	if(u == 0 || v == 0) {
		return Point2f(0,0);
	}
	return Point2f(u,v);
}

Vector3f Bitmap_Light_Env::pixelToDir(const Point2f &p) const {

	float theta = p.x()* M_PI / (rows() - 1);
	float phi = p.y() * 2 * M_PI / (cols() - 1);
    float x = std::sin(theta) * std::cos(phi);
    float y = std::sin(theta) * std::sin(phi);
    float z = std::cos(theta);

	return Vector3f(x, y, z).normalized();
}



                                    /*************************************/
                                    /******                         ******/
                                    /******      Interpolation      ******/
                                    /******                         ******/
                                    /*************************************/
Color3f Bitmap_Light_Env::linearInterpolation(float rand, Color3f col1, Color3f col2) const{
	return (1 - rand) * col1 + rand * col2;
}


Color3f Bitmap_Light_Env::bilinearInterpolation(float x2x1, float y2y1, float x2x, float xx1, float yy1, float y2y, Color3f Q11, Color3f Q21, Color3f Q12, Color3f Q22) const{
	//Color3f up = linearInterpolation(rand1, col1, col2);
	//Color3f bottom = linearInterpolation(rand1, col3, col4);
	return 1/((x2x1)*(y2y1)) * (Q11 * x2x * y2y + Q21 * xx1 * y2y + Q12 * x2x * yy1 + Q22 * xx1 * yy1);
}

                                    /*************************************/
                                    /******                         ******/
                                    /****** paper functions         ******/
                                    /******                         ******/
                                    /*************************************/
float Bitmap_Light_Env::preCompute1D(int row, const floatmat &f, floatmat &pf, floatmat &Pf) const{
	float nf = f.cols();

	//cout << "preComputer1D" << endl;
	//cout << "nf: " << nf << endl;

	float I = 0;
	for (int i = 0; i < nf; ++i){
		//cout << "I: "<< I  << " f: " << f(row, i) << " ";
		I += f(row, i);
	}
	cout << endl;

	if (I == 0)
		return I;

	for (int i = 0; i < nf; ++i){
		//cout << "I: "<< I << " ";
		pf(row, i) = f(row, i) / I;
		//cout << "pf: "<< pf(row, i) << endl;
		//cout << "inside first for of preComputer1D: " << pf(row, i) << endl;
	}

	Pf(row, 0) = 0;
	for (int i = 1; i < nf; ++i){
		Pf(row, i) = Pf(row, i-1) + pf(row, i-1);
		//cout << "inside secondfor of preComputer1D Pf(row, i): " << Pf(row, i) << endl;
	}
	Pf(row, nf) = 1;

	return I;
}


void Bitmap_Light_Env::sample1D(int row, const floatmat &pf, const floatmat &Pf, const float &unif, float &x, float &p) const{
    //int ind;
	//cout << "sample1D--1" << endl;
	int i=0;
	for (; i < Pf.cols()-2; i++) {
        //ind = i;
		if (unif >= Pf(row, i) && unif < Pf(row, i+1))
			break;
	}
	
	//cout << "i: " << i << " rows: " << rows() << " cols: "<< cols() << " last: " << Pf.cols() << endl;
	//cout << "sample1D--2" << endl;
	//cout << "Pf row: " << Pf(row, i) << endl;
	//cout << "Pf row after bla bla: " << Pf(row, i+1) << endl;
	
	float t = (Pf(row, i+1) - unif) / (Pf(row, i+1) - Pf(row, i));
	//cout << "sample1D--3" << endl;
	x = (1-t) * i + t * (i+1);
	//cout << "sample1D--4" << endl;
	p = pf(row, i);
}



NORI_NAMESPACE_END
