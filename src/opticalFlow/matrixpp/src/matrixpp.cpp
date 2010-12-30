#include "matrixpp.h"

using namespace std;
using namespace yarp::sig;
Matrix yarp::matrixpp::square(Matrix root)
{
    Matrix sq(root.rows(), root.cols());
    for(int i=0; i<root.rows(); i++)
        for(int ii=0; ii<root.cols(); ii++)
            sq(i, ii)=root(i, ii)*root(i, ii);
    return sq;
}
Matrix yarp::matrixpp::exp(Matrix in)
{
    Matrix expM(in.rows(), in.cols());
    for(int i=0; i<in.rows(); i++)
        for(int ii=0; ii<in.cols(); ii++)
            expM(i, ii)=std::exp( in(i, ii) );
    return expM;
}

Matrix yarp::matrixpp::operator/(Matrix& A, Matrix& B)
{
    Matrix div(A.rows(), B.cols());
    for(int i=0; i<A.rows(); i++)
        for(int ii=0; ii<A.cols(); ii++)
            div(i, ii)=A(i, ii)/B(i, ii);
    return div;
}

Matrix yarp::matrixpp::operator/(Matrix& a, double& sc)
{
    Matrix out(a.rows(), a.cols());
    for(int i=0; i<a.rows(); i++)
        for(int ii=0; ii<a.cols(); ii++)
            out(i, ii)=a(i, ii)/sc;
    return out;
}

Matrix yarp::matrixpp::operator/(Matrix& a, int& sc)
{
    Matrix out(a.rows(), a.cols());
    for(int i=0; i<a.rows(); i++)
        for(int ii=0; ii<a.cols(); ii++)
            out(i, ii)=a(i, ii)/sc;
    return out;
}

Matrix yarp::matrixpp::operator%(Matrix& A, Matrix& B)
{
    Matrix mul(A.rows(), A.cols());
    for(int i=0; i<A.rows(); i++)
        for(int ii=0; ii<A.cols(); ii++)
            mul(i, ii)=A(i, ii)*B(i, ii);
    return mul;
}

Matrix yarp::matrixpp::operator-(Matrix& A)
{
    Matrix out(A.rows(), A.cols());
    for(int i=0; i<A.rows(); i++)
        for(int ii=0; ii<A.cols(); ii++)
            out(i, ii)=-A(i, ii);
    return out;
}

Matrix yarp::matrixpp::operator>=(Matrix& A, double& hit)
{
    Matrix tmp(A.rows()*A.cols(), 2);
    int found=0;
    for(int i=0; i<A.rows(); i++)
        for(int ii=0; ii<A.cols(); ii++)
        {
            if(A(i, ii)>=hit)
            {
                tmp(found, 0)=i;
                tmp(found, 1)=ii;
                found++;
            }
        }
    if(found<A.rows()*A.cols())
    {
        Matrix res(found, 2);
        for(int iii=0; iii<found; iii++)
        {
            res(iii, 0)=tmp(iii, 0);
            res(iii, 1)=tmp(iii, 1);
        }
        return res;
    }
    else
        return tmp;
}

Matrix yarp::matrixpp::operator>=(Matrix& A, int& hit)
{
    Matrix tmp(A.rows()*A.cols(), 2);
    int found=0;
    for(int i=0; i<A.rows(); i++)
        for(int ii=0; ii<A.cols(); ii++)
        {
            if(A(i, ii)>=(double)hit)
            {
                tmp(found, 0)=i;
                tmp(found, 1)=ii;
                found++;
            }
        }
    if(found<A.rows()*A.cols())
    {
        Matrix res(found, 2);
        for(int iii=0; iii<found; iii++)
        {
            res(iii, 0)=tmp(iii, 0);
            res(iii, 1)=tmp(iii, 1);
        }
        return res;
    }
    else
        return tmp;
}

Matrix yarp::matrixpp::operator!=(Matrix& A, double& hit)
{
    Matrix tmp(A.rows()*A.cols(), 2);
    int found=0;
    for(int i=0; i<A.rows(); i++)
        for(int ii=0; ii<A.cols(); ii++)
        {
            if(A(i, ii)!=hit)
            {
                tmp(found, 0)=i;
                tmp(found, 1)=ii;
                found++;
            }
        }
    if(found<A.rows()*A.cols())
    {
        Matrix res(found, 2);
        for(int iii=0; iii<found; iii++)
        {
            res(iii, 0)=tmp(iii, 0);
            res(iii, 1)=tmp(iii, 1);
        }
        return res;
    }
    else
        return tmp;
}

void yarp::matrixpp::upSubMat(Matrix& toUp, Matrix& in, int r1, int c1, int r2, int c2)
{
    for(int i=r1; i<=r2; i++)
        for(int ii=c1; ii<=c2; ii++)
            toUp(i, ii)=in(i, ii);
}

Vector yarp::matrixpp::mat2vec(Matrix& in)
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
