#include "synapse.h"

using namespace std;
using namespace yarp::sig;
using namespace yarp::math;

synapse::synapse(int i_argc, void** i_argv)
{
	sMapX		= reinterpret_cast<int*>(i_argv[0]);
	sMapY		= reinterpret_cast<int*>(i_argv[1]);
	neighLR		= reinterpret_cast<int*>(i_argv[2]);
	stdDev		= reinterpret_cast<double*>(i_argv[3]);
	threshold	= reinterpret_cast<int*>(i_argv[4]);
	alpha		= reinterpret_cast<int*>(i_argv[5]);
	tauC		= reinterpret_cast<int*>(i_argv[6]);
	tauD		= reinterpret_cast<int*>(i_argv[7]);
	accTime		= reinterpret_cast<int*>(i_argv[8]);
//	fn2call		= reinterpret_cast<void(*)(int, void**)>(i_argv[9]);
//	fn2call		= reinterpret_cast<void(eLucasKanade::*)(int, void**)>(i_argv[9]);

	nbArg		=reinterpret_cast<int*>(i_argv[10]);
	selectedArg	= reinterpret_cast<int*>(i_argv[11]);

//    cout    << "The arguments are: "            << endl
//            << "sMapX: "        << *sMapX        << endl
//            << "sMapY: "        << *sMapY        << endl
//            << "neighLR: "      << *neighLR      << endl
//            << "stdDev: "       << *stdDev       << endl
//            << "threshold: "    << *threshold    << endl
//            << "alpha: "        << *alpha        << endl
//            << "tauC: "         << *tauC         << endl
//            << "tauD: "         << *tauD         << endl
//            << "accTime: "      << *accTime      << endl
//            << "nbArg: "        << *nbArg        << endl
//            << "selectedArg: "  << *selectedArg  << endl;

	arg			= new void*[*nbArg];

	argELK= new void*[4];

	argELK[0]= sMapX;
	argELK[1]= sMapY;
	argELK[2]= neighLR;
	stdDev2  = 0.2;	argELK[3]= &stdDev2;

	objFlow = eLucasKanade(4, argELK);
//	objFlow.openPort();


	td.resize(*sMapX, *sMapY);		    td.zero();
	potential.resize(*sMapX, *sMapY);	potential.zero();

	sWin	= 2*(*neighLR)+1;

	discharge.resize(sWin, sWin);       discharge.zero();
	repT.resize(sWin, sWin);

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

//    cout    << "A: " << A << endl
//            << "B: " << B << endl;
    Matrix tmp2(sWin, sWin);
    tmp2=(double)(2*(*stdDev)*(*stdDev));

//    cout << "2*theta^2: " << tmp2 << endl;
    zeroMat.resize(sWin, sWin); zeroMat.zero();

	gaussian.resize(sWin, sWin);

//	cout << "avant toutes convertion" << endl;

	gsl_matrix_mul_elements((gsl_matrix *)A.getGslMatrix(), (gsl_matrix *)A.getGslMatrix());//A=square(A);

//	cout << "Apres la premiere convertion" << endl;

	gsl_matrix_mul_elements((gsl_matrix *)A.getGslMatrix(), (gsl_matrix *)A.getGslMatrix());//B=square(B);

//	cout << "Apres la deuxieme convertion" << endl;
	A=A+B;
    gaussian=zeroMat-A;//gsl_matrix_sub(zeroMat.getGslMatrix(), A.getGslMatrix());//A=-A;
	gsl_matrix_div_elements((gsl_matrix *)gaussian.getGslMatrix(), (gsl_matrix *)tmp2.getGslMatrix());//gaussian = exp( A/tmp2 );

//	cout << "Apres la troisieme convertion" << endl;

//	cout << "exp" << endl;
	exp(gaussian);
//    cout << "gaussian: " << gaussian << endl;

	gs.resize(*sMapX, *sMapY);		gs.zero();
	pgs.resize(*sMapX, *sMapY);		pgs.zero();


//    cout << "sWin: " << sWin << endl;

    subGs.resize(sWin, sWin);

//    cout << "Size subMat in constructor: " << subGs.rows() << " / " << subGs.cols() << endl;

    tmpSub.resize(sWin, sWin);
    tmpSub2.resize(sWin, sWin);

	refTs		= -1;
//    cout << "Constructor OK" << endl;
}
synapse::~synapse()
{
//    objFlow.closePort();
}
void synapse::operator=(const synapse& obj)
{
	sWin=obj.sWin;
	sMapX=obj.sMapX;
	sMapY=obj.sMapY;
	neighLR=obj.neighLR;
	accTime=obj.accTime;

	refTs=obj.refTs;

	stdDev=obj.stdDev;
	threshold=obj.threshold;
	alpha=obj.alpha;
	tauC=obj.tauC;
	tauD=obj.tauD;

	td=obj.td;
	potential=obj.potential;
	discharge=obj.discharge;
	gaussian=obj.gaussian;
	repT=obj.repT;

	nbArg=obj.nbArg;
	selectedArg=obj.selectedArg;

	arg = new void*[*nbArg];

	objFlow=obj.objFlow;

	gs=obj.gs;
	pgs=obj.pgs;

    subGs=obj.subGs;
    tmpSub=obj.tmpSub;
    tmpSub2=obj.tmpSub2;

    zeroMat=obj.zeroMat;
//    cout    << "The arguments copied are: "     << endl
//            << "sMapX: "        << *sMapX        << endl
//            << "sMapY: "        << *sMapY        << endl
//            << "neighLR: "      << *neighLR      << endl
//            << "stdDev: "       << *stdDev       << endl
//            << "threshold: "    << *threshold    << endl
//            << "alpha: "        << *alpha        << endl
//            << "tauC: "         << *tauC         << endl
//            << "tauD: "         << *tauD         << endl
//            << "accTime: "      << *accTime      << endl
//            << "nbArg: "        << *nbArg        << endl
//            << "selectedArg: "  << *selectedArg  << endl;
}
//void synapse::printArg()
//{
//	char toSay[]="Print done !";
//	arg[0]=toSay;
////	arg[0]=new char[12];
////	memcpy(arg[0], "Hello world", 12*sizeof(char));
//	cout 	<< "Size of the map in X: "	<< *sMapX		<< endl
//			<< "Size of the map in Y: " << *sMapY		<< endl
//			<< "L-R Neighboor Number: "	<< *neighLR		<< endl
//			<< "Standard deviation: "	<< *stdDev		<< endl
//			<< "Threshold: "			<< *threshold	<< endl
//			<< "alpha: "				<< *alpha		<< endl
//			<< "tauC: "					<< *tauC		<< endl
//			<< "tauD: "					<< *tauD		<< endl
//			<< "accumulation time: "	<< *accTime		<< endl
//			<< "fn2call Arg number: "	<< *nbArg		<< endl
//			<< "the called function say: ";
//
//	fn2call(*nbArg, arg);
//}

void synapse::filter(int argc, void** argv)
{
//    cout    << "synapse::filter(...)"               << endl
//            << "\targs: "                           << endl
//            << "\t\targv[0]: "  << *v2i(argv[0])    << endl
//            << "\t\targv[1]: "  << *v2i(argv[1])    << endl
//            << "\t\targv[2]: "  << *v2i(argv[2])    << endl
//            << "\t\targv[3]: "  << *v2i(argv[3])    << endl;
//
//    cout    << "Members variables needed:"          << endl
//            << "\t\trefTs: "    << refTs            << endl
//            << "\t\tneighLR: "  << *neighLR         << endl
//            << "\t\ttauD: "     << *tauD            << endl
//            << "\t\tsMapX: "    << *sMapX           << endl
//            << "\t\tsMapY: "    << *sMapY           << endl;
	if (refTs==-1)
        *v2i(argv[3]);
//    cout << "First submatrixation" << endl;
//    cout << "neighLR: " << *neighLR << endl;
//    cout << "Coordinate of the point: " << *v2i(argv[0]) << " / " << *v2i(argv[1]) << endl;
//    cout << "Size gs: " << gs.rows() << " / " << gs.cols() << endl;
//    cout << "Borne: axis X: " << *v2i(argv[0])-(*neighLR) << " / " << *v2i(argv[0])+(*neighLR) << " Axis Y: " << *v2i(argv[1])-(*neighLR) << " / " << *v2i(argv[1])+(*neighLR) << endl;
//    cout << "Size extraction: " << (*v2i(argv[0])+(*neighLR) - (*v2i(argv[0])-(*neighLR))) << " / " << (*v2i(argv[1])+(*neighLR) - (*v2i(argv[1])-(*neighLR))) << endl;
//    cout << "Size subMat: " << subGs.rows() << " / " << subGs.cols() << endl;
	subGs = gs.submatrix(	*v2i(argv[0])-(*neighLR),
                            *v2i(argv[0])+(*neighLR),
                            *v2i(argv[1])-(*neighLR),
                            *v2i(argv[1])+(*neighLR));
//    cout << "First submatrixation done" << endl;
    tmpSub=gaussian*(*v2i(argv[2]))*5;
    subGs=subGs+tmpSub;
//    cout << "First update" << endl;
    updateSubMatrix(gs, subGs, *v2i(argv[0])-*neighLR,
                        *v2i(argv[1])-*neighLR,
                        *v2i(argv[0])+*neighLR,
                        *v2i(argv[1])+*neighLR);

//    cout << "Filling of a matrix by a unique scalar" << endl;
	repT = *v2i(argv[3]);

//    cout << "second submatrixation" << endl;
    tmpSub=td.submatrix(	*v2i(argv[0])-*neighLR,
                            *v2i(argv[0])+*neighLR,
                            *v2i(argv[1])-*neighLR,
                            *v2i(argv[1])+*neighLR);
//    cout << "Substraction of matrix" << endl;
    tmpSub=repT-tmpSub;
    tmpSub2=(*tauD);
//    cout << "Division element by element thanks to GSL" << endl;
    gsl_matrix_div_elements((gsl_matrix *)tmpSub.getGslMatrix(), (gsl_matrix *)tmpSub2.getGslMatrix());//tmpSub=tmpSub/tmpSub2;
//    cout << "Substraction 0-Mat" << endl;
//    cout << "size mat zero: " << zeroMat.rows() << " / " << zeroMat.cols() << endl;
//    cout << "size tmpSub: " << tmpSub.rows() << " / " << tmpSub.cols() << endl;
    tmpSub=zeroMat-tmpSub;

//    cout << "third submatrixation" << endl;
    tmpSub2=potential.submatrix(	*v2i(argv[0])-*neighLR,
                                    *v2i(argv[0])+*neighLR,
                                    *v2i(argv[1])-*neighLR,
                                    *v2i(argv[1])+*neighLR);
//    cout << "exponential of mat" << endl;
    exp(tmpSub);
    tmpSub=( *alpha)*tmpSub;
//    cout << "Multiplication element by element thanks to GSL" << endl;
    gsl_matrix_mul_elements((gsl_matrix *)tmpSub2.getGslMatrix(), (gsl_matrix *)tmpSub.getGslMatrix());
	discharge=tmpSub2;

    tmpSub=discharge+gaussian;
//    cout << "Update of potential" << endl;
	updateSubMatrix(potential, tmpSub, *v2i(argv[0])-*neighLR,
                                            *v2i(argv[1])-*neighLR,
                                            *v2i(argv[0])+*neighLR,
                                            *v2i(argv[1])+*neighLR);

	updateSubMatrix(td, repT,  *v2i(argv[0])-*neighLR,
                        *v2i(argv[1])-*neighLR,
                        *v2i(argv[0])+*neighLR,
                        *v2i(argv[1])+*neighLR);
//    cout << "findSupEq" << endl;
	Matrix ind;
	int nbMatch=findSupEq(potential, (double)(*threshold), ind);//(potential>=(*threshold));

//	cout << "Size of above-threshold pixels: " << ind.n_elem << endl;
//    cout << "Kek choZ a fR?" << endl;
	if(nbMatch>0)//if(ind.rows()>0)
	{
		X.resize(ind.rows());
		Y.resize(ind.rows());
		for(int i=0; i<ind.rows(); i++)
		{
			X(i)=ind(i, 0);
//			X=ind(i)%(*sMapX);
			Y(i)=ind(i, 1);
			potential(ind(i, 0), ind(i, 1))=-0.5;
		}
//		cout    << "X: " << X << endl
//                << "Y: " << Y << endl;
		setArg();
//		fn2call(*nbArg, arg);
//        cout << "Call the eLucasKanade module" << endl;
//        cout << "Appel de Lucas et Kanade en event based, en esperant qu'ils comprennent ^^" << endl;
		objFlow.fnCall(*nbArg, arg);
//		cout << "Apparement z'ont comprit :D" << endl;
	}
//	cout    << "Time ref: "     << refTs         << endl
//            << "Time of acc"    << *accTime      << endl
//            << "Current time: " << *v2i(argv[3]) << endl;

	if(*v2i(argv[3])>=refTs+(*accTime))
	{
//	    cout << "time pasted ..." << endl;
		pgs=gs;
		refTs=*v2i(argv[3]);
	}
}

void synapse::setArg()
{
//    cout    << "selectedArg: "          << (*selectedArg) << endl
//            << "selectedArg: "          << hex << (*selectedArg) << endl
//            << "ADDRX: "                << hex << (ADDRX) << endl
//            << "ADDRY: "                << hex << (ADDRY) << endl
//            << "SIZEX: "                << hex << (SIZEX) << endl
//            << "SIZEY: "                << hex << (SIZEY) << endl
//            << "NEIGH: "                << hex << (NEIGH) << endl
//            << "CGRAYMAP: "             << hex << (CGRAYMAP) << endl
//            << "PGRAYMAP: "             << hex << (PGRAYMAP) << endl
//            << "selectedArg&ADDRX: "    << hex << ((*selectedArg)&ADDRX) << endl
//            << "selectedArg&ADDRY: "    << hex << ((*selectedArg)&ADDRY) << endl
//            << "selectedArg&SIZEX: "    << hex << ((*selectedArg)&SIZEX) << endl
//            << "selectedArg&SIZEY: "    << hex << ((*selectedArg)&SIZEY) << endl
//            << "selectedArg&NEIGH: "    << hex << ((*selectedArg)&NEIGH) << endl
//            << "selectedArg&CGRAYMAP: " << hex << ((*selectedArg)&CGRAYMAP) << endl
//            << "selectedArg&PGRAYMAP: " << hex << ((*selectedArg)&PGRAYMAP) << endl;
	int i=0;
	if(*selectedArg&ADDRX)
	{
//	    cout << "X selected" << endl;
		arg[i]=&X;
		i++;
	}
	if(*selectedArg&ADDRY)
	{
//	    cout << "Y selected" << endl;
		arg[i]=&Y;
		i++;
	}
	if(*selectedArg&SIZEX)
	{
//	    cout << "sMapX selected" << endl;
		arg[i]=sMapX;
		i++;
	}
	if(*selectedArg&SIZEY)
	{
//	    cout << "sMapY selected" << endl;
		arg[i]=sMapY;
		i++;
	}
	if(*selectedArg&NEIGH)
	{
//	    cout << "neighLR selected" << endl;
		arg[i]=neighLR;
		i++;
	}
	if(*selectedArg&CGRAYMAP)
	{
//	    cout << "gs selected" << endl;
		arg[i]=&gs;
		i++;
	}
	if(*selectedArg&PGRAYMAP)
	{
//	    cout << "pgs selected" << endl;
		arg[i]=&pgs;
		i++;
	}
}

void synapse::exp(Matrix& a)
{
    for(int i=0; i<a.rows(); i++)
        for(int ii=0; ii<a.cols(); ii++)
            a(i, ii)=std::exp(a(i, ii));
}

void synapse::updateSubMatrix(Matrix& toUp, Matrix& newVal, int R1, int C1, int R2, int C2)
{
    for(int i=R1; i<=R2; i++)
        for(int ii=C1; ii<=C2; ii++)
            toUp(i, ii)=newVal(i-R1, ii-C1);
}

int synapse::findSupEq(Matrix& A, double hit, Matrix& result)
{
//    cout << "\t synapse::findSupEq" << endl;
//    cout << "\t size in matrix: " << A.rows() << " / " << A.cols() << endl;
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
//    cout << "\t Number of match: " << found << endl;
    if(found<A.rows()*A.cols() && found > 0)
    {
        Matrix res(found, 2);
        for(int iii=0; iii<found; iii++)
        {
            res(iii, 0)=tmp(iii, 0);
            res(iii, 1)=tmp(iii, 1);
        }
        result = res;
    }
    else
        result = tmp;
    return found;
}
