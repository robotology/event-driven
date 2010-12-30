#include "eLucasKanade.h"

using namespace std;
using namespace yarp::sig;
using namespace yarp::math;

eLucasKanade::eLucasKanade(int argc, void** argv)
{
	sMapX	= reinterpret_cast<int*>(argv[0]);
	sMapY	= reinterpret_cast<int*>(argv[1]);
	neighLR	= reinterpret_cast<int*>(argv[2]);
	stdDev	= reinterpret_cast<int*>(argv[3]);

	sWin	= 2*(*neighLR)+1;

	Vector tmp(sWin);
	for(int i=0; i<sWin; i++)
		tmp(i)= i-*neighLR;
    Matrix A(sWin, sWin);
    Matrix B(sWin, sWin);
    for(int i=0; i<sWin; i++)
    {
     	A.setCol(i, tmp);
        B.setRow(i, tmp);
    }

	gaussian.resize(sWin, sWin);
	gsl_matrix_mul_elements((gsl_matrix *)A.getGslMatrix(), (gsl_matrix *)A.getGslMatrix());//A=square(A);
	gsl_matrix_mul_elements((gsl_matrix *)A.getGslMatrix(), (gsl_matrix *)A.getGslMatrix());//B=square(B);
	A=A+B;

    Matrix tmp2(sWin, sWin);
    tmp2=(double)(2*(*stdDev)*(*stdDev));
	Matrix zeroMat = zeros(sWin, sWin);
    gaussian=zeroMat-A;//gsl_matrix_sub(zeroMat.getGslMatrix(), A.getGslMatrix());//A=-A;
	gsl_matrix_div_elements((gsl_matrix *)gaussian.getGslMatrix(), (gsl_matrix *)tmp2.getGslMatrix());//gaussian = exp( A/tmp2 );
	exp(gaussian);

	Vector tmpCol(sWin*sWin);
	tmpCol = mat2vec(gaussian);
	W.resize(sWin*sWin, sWin*sWin);
	W.diagonal(tmpCol);
    W_s=W;
    gsl_matrix_mul_elements((gsl_matrix *)W_s.getGslMatrix(), (gsl_matrix *)W_s.getGslMatrix());

//	dIdx.zeros(sWin*sWin);
//	dIdy.zeros(sWin*sWin);
	dIdxy.resize(sWin*sWin,2);  dIdxy.zero();
	dIdt.resize(sWin*sWin);     dIdt.zero();
	zeroVec.resize(sWin*sWin);  zeroVec.zero();

	neighboorhood.resize(sWin, sWin);

	sobelX.resize(3,3);
	sobelY.resize(3,3);

//	sobelX = " -1   -2  -1;\
//				0    0   0;\
//				1    2   1";
    sobelX(0,0) = -1;
    sobelX(0,1) = 0;
    sobelX(0,2) = 1;
    sobelX(1,0) = -2;
    sobelX(1,1) = 0;
    sobelX(1,2) = 2;
    sobelX(2,0) = -1;
    sobelX(2,1) = 0;
    sobelX(2,2) = 1;

	sobelX=0.125*sobelX;

//	sobelY = "  -1 0 1;\
//				-2 0 2;\
//				-1 0 1";
    sobelY(0,0) = -1;
    sobelY(0,1) = -2;
    sobelY(0,2) = -1;
    sobelY(1,0) = 0;
    sobelY(1,1) = 0;
    sobelY(1,2) = 0;
    sobelY(2,0) = 1;
    sobelY(2,1) = 2;
    sobelY(2,2) = 1;

	sobelY=0.125*sobelY;

    lambda=1.5;
//    port.open("/image/opticalFlow:o");
}

eLucasKanade::~eLucasKanade()
{
//    port.close();
}

void eLucasKanade::operator=(const eLucasKanade& obj)
{
	sWin=obj.sWin;

	sMapX=obj.sMapX;
	sMapY=obj.sMapY;
	neighLR=obj.neighLR;
	stdDev=obj.stdDev;

	gaussian=obj.gaussian;
	W=obj.W;
    W_s=obj.W_s;

	sobelX=obj.sobelX;
	sobelY=obj.sobelY;

	neighboorhood=obj.neighboorhood;

//	dIdx=obj.dIdx;
//	dIdy=obj.dIdy;
	dIdt=obj.dIdt;
	dIdxy=obj.dIdxy;

    zeroVec=obj.zeroVec;

    lambda=obj.lambda;
//    port.open("/image/opticalFlow:o");
}
void eLucasKanade::compute(int argc, void** argv)
{
//    cout    << "eLucasKanade::compute(...)"          << endl
//            << "\tArgs: "                            << endl
//            << "\t\targv[0]: "  << *v2uvec(argv[0])  << endl
//            << "\t\targv[1]: "  << *v2uvec(argv[1])  << endl
////            << "\t\targv[2]: "  << *v2m(argv[2])     << endl
////            << "\t\targv[3]: "  << *v2m(argv[3])     << endl;
//            << "Sobel X: "      << sobelX            << endl
//            << "Sobel Y: "      << sobelY            << endl;
//    cout << "Vector of interesting address from filter:synapse received" << endl;
	for(int addr=0; addr<(*v2uvec(argv[0])).size(); addr++)
	{
//cout << "la " << 0 << endl;
		dIdxy=0;
		int nbNeigh=0;
		for(int L=-(*neighLR); L<=(*neighLR); L++)
			for(int R=-(*neighLR); R<=(*neighLR); R++)
			{
				neighboorhood = (*v2m(argv[2])).submatrix(  ((*v2uvec(argv[0]))(addr)+L-1),
                                                            ((*v2uvec(argv[0]))(addr)+L+1),
                                                            ((*v2uvec(argv[1]))(addr)+R-1),
                                                            ((*v2uvec(argv[1]))(addr)+R+1));
//				cout << "Neighboorhood: " << neighboorhood << endl;
                dIdxy(nbNeigh,0)=0;
                dIdxy(nbNeigh,1)=0;
				for(int i=0; i<sobelX.rows(); i++)
					for(int j=0; j<sobelX.cols(); j++)
					{
						dIdxy(nbNeigh,0)+=sobelX(i, j)*neighboorhood( (neighboorhood.rows()-1)-i, (neighboorhood.cols()-1)-j);
						dIdxy(nbNeigh,1)+=sobelY(i, j)*neighboorhood( (neighboorhood.rows()-1)-i, (neighboorhood.cols()-1)-j);
					}
                dIdt(nbNeigh) = (*v2m(argv[3]))((*v2uvec(argv[0]))(addr)+L, (*v2uvec(argv[1]))(addr)+R)-(*v2m(argv[2]))((*v2uvec(argv[0]))(addr)+L, (*v2uvec(argv[1]))(addr)+R);
                nbNeigh++;
			}
//        cout << "dIdxy, and dIdt computed" << endl;
//cout << "la " << 1 << endl;
		Matrix dIdxy_t(2, sWin*sWin);
		dIdxy_t = dIdxy.transposed();
//cout << "la " << 2 << endl;
        Matrix dIdxyWdIdxy = dIdxy_t*W_s*dIdxy;

//        cout << "compute the velocity vector vxy" << endl;

		Vector o_vxy=pinv(dIdxyWdIdxy)*dIdxy_t*W_s*(zeroVec-dIdt);

//		cout << "the computed velocites are: " << o_vxy(0) << " / " << o_vxy(1) << endl;
//		cout << "Velocity computed: " << o_vxy << endl;
//cout << "la " << 3 << endl;


		Vector eval(dIdxyWdIdxy.rows());
		Matrix evec(dIdxyWdIdxy.rows(), dIdxyWdIdxy.cols());

//        cout << "Juste avant eigen-value decomposition" << endl;

//		eig(dIdxyWdIdxy, eval, evec);
//        gsl_complex eval_i;
//        for(int i=0; i<eval.size(); i++)
//        {
//            eval_i = gsl_vector_complex_get ((gsl_vector_complex*) eval.getGslVector(), i);
//            cout << "eigen-value_" << i << " = " << GSL_REAL(eval_i) << endl;
//        }
//        gsl_complex evalMin = gsl_vector_complex_get ((gsl_vector_complex*) eval.getGslVector(), eval.size()-1);
//        gsl_complex evalMax = gsl_vector_complex_get ((gsl_vector_complex*) eval.getGslVector(), 0);
//        gsl_vector_complex_view evecMax_ = gsl_matrix_complex_column ((gsl_matrix_complex*) evec.getGslMatrix(), 0);
//        Vector evecMax(dIdxyWdIdxy.rows());
//        cout << "Computed eigen-value: min" << GSL_REAL(evalMin) << " / max: " << GSL_REAL(evalMax) << endl;
//        if(GSL_REAL(evalMin)>=lambda)
//            o_vxy=o_vxy;
//        else
//            if(GSL_REAL(evalMin)<lambda && GSL_REAL(evalMax)>=lambda)
//            {
//                for(int i=0; i<evecMax.size(); i++)
//                {
//                    gsl_complex z = gsl_vector_complex_get(&evecMax_.vector, i);
//                    evecMax(i) = GSL_REAL(z);
//                }
//                o_vxy=(o_vxy*evecMax)*evecMax;
//            }
////            o_vxy=(eigval(0)==max(eigval))?as_scalar(trans(o_vxy)*eigvec.col(0))*eigvec.col(0):as_scalar(trans(o_vxy)*eigvec.col(1))*eigvec.col(1);
//            else
//                o_vxy = 0;
        eigNonSym(dIdxyWdIdxy, eval, evec);
//        cout << "Computed eigen-value: min: " << eval(1) << " / max: " << eval(0) << endl;
        if(eval(1)>=lambda)
            o_vxy=o_vxy;
        else
            if(eval(1)<lambda && eval(0)>=lambda)
            {
                o_vxy=(o_vxy*evec.getCol(0))*evec.getCol(0);
            }
//            o_vxy=(eigval(0)==max(eigval))?as_scalar(trans(o_vxy)*eigvec.col(0))*eigvec.col(0):as_scalar(trans(o_vxy)*eigvec.col(1))*eigvec.col(1);
            else
                o_vxy = 0;
//        double sVx=o_vxy(0);
//        double sVy=o_vxy(1);
        int addrX = (int)(*v2uvec(argv[0]))(addr);
        int addrY = (int)(*v2uvec(argv[1]))(addr);

        cout << "Address of the update: " << (int)(*v2uvec(argv[0]))(addr) << " / " << (int)(*v2uvec(argv[1]))(addr) << endl;
        cout << "the updated velocites before 'send' are: " << o_vxy(0) << " / " << o_vxy(1) << endl;
//        cout << "the updated velocites in 'double': " << sVx << " / " << sVy << endl;

//		this->send((int&)(*v2uvec(argv[0]))(addr), (int&)(*v2uvec(argv[1]))(addr), o_vxy(0), o_vxy(1));
//        this->send((int&)(*v2uvec(argv[0]))(addr), (int&)(*v2uvec(argv[1]))(addr), sVx, sVy);
        this->send(addrX, addrY, o_vxy(0), o_vxy(1));
//		cout << "the updated velocites after 'send' are: " << o_vxy(0) << " / " << o_vxy(1) << endl;
	}
}

void eLucasKanade::send(int& X, int& Y, double& VX, double& VY)
{
	cout 	<< "X: " 	<< X	<< endl
			<< "Y: " 	<< Y	<< endl
			<< "VX: "	<< VX	<< endl
			<< "VY: "	<< VY	<< endl;

    vecBuffer& tmp = port.prepare();
    tmp = vecBuffer(X, Y, VX, VY);
    port.write();
}

void eLucasKanade::eig(Matrix& in, Vector& eval, Matrix& evec)
{
    gsl_eigen_symmv_workspace * w = gsl_eigen_symmv_alloc (4);
    gsl_eigen_symmv ((gsl_matrix *) in.getGslMatrix(), (gsl_vector*) eval.getGslVector(), (gsl_matrix *) evec.getGslMatrix(), w);
    gsl_eigen_symmv_free (w);
    gsl_eigen_symmv_sort ((gsl_vector*) eval.getGslVector(), (gsl_matrix *) evec.getGslMatrix(), GSL_EIGEN_SORT_ABS_ASC);
}

void eLucasKanade::exp(Matrix& a)
{
    for(int i=0; i<a.rows(); i++)
        for(int ii=0; ii<a.cols(); ii++)
            a(i, ii)=std::exp(a(i, ii));
}

Vector eLucasKanade::mat2vec(Matrix& in)
{
    Vector res(in.rows()*in.cols());
    int i=0;
    for(int ii=0; ii<in.rows(); ii++)
        for(int iii=0; iii<in.cols(); iii++)
        {
            res(i)=in(ii, iii);
            i++;
        }
    return res;
}

void eLucasKanade::eigNonSym(Matrix& in, Vector& eval, Matrix& evec)
{
//    gsl_matrix_view m = gsl_matrix_view_array (data, 4, 4);
    gsl_vector_complex* eval_ = gsl_vector_complex_alloc (eval.size());
    gsl_matrix_complex* evec_ = gsl_matrix_complex_alloc (evec.rows(), evec.cols());
    gsl_eigen_nonsymmv_workspace * w = gsl_eigen_nonsymmv_alloc (eval.size());
    gsl_eigen_nonsymmv ((gsl_matrix*) in.getGslMatrix(), eval_, evec_, w);
    gsl_eigen_nonsymmv_free (w);
    gsl_eigen_nonsymmv_sort (eval_, evec_, GSL_EIGEN_SORT_ABS_DESC);

    for (int i = 0; i < evec.cols(); i++)
    {
        gsl_complex eval_i = gsl_vector_complex_get (eval_, i);
        gsl_vector_complex_view evec_i = gsl_matrix_complex_column (evec_, i);

        eval(i)=GSL_REAL(eval_i); //GSL_IMAG(eval_i));
        for (int j = 0; j < evec.rows(); j++)
        {
            gsl_complex z = gsl_vector_complex_get(&evec_i.vector, j);
            evec(j, i) = GSL_REAL(z);// GSL_IMAG(z));
        }
    }
}
