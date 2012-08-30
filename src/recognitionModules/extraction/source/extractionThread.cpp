#include "extractionThread.hpp"

//#define _DEBUG_

using namespace std;

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace emorph::eunmask;
using namespace emorph::evolume;

extractionThread::extractionThread()
//:RateThread(THRATE), szSpace(0), szTemp(0), eigenvalSim(0), eigenvecSim(0), dim(0)
:szSpace(0), szTemp(0), eigenvalSim(0), eigenvecSim(0), dim(0)
{
    ptr_pose=0;
    ptr_nege=0;
    ptr_posk=0;
    ptr_negk=0;

    source=NULL;
}

extractionThread::extractionThread(std::string _src, uint _type, std::string _eye, unsigned int _szSpace, unsigned int _szTemp, double _eigenvalSim, double _eigenvecSim, unsigned int _dim)
//:RateThread(THRATE), szSpace(_szSpace), szTemp(_szTemp), eigenvalSim(_eigenvalSim), eigenvecSim(_eigenvecSim), dim(_dim)
:szSpace(_szSpace), szTemp(_szTemp), eigenvalSim(_eigenvalSim), eigenvecSim(_eigenvecSim), dim(_dim)
{
    ptr_pose=0;
    ptr_nege=0;
    ptr_posk=0;
    ptr_negk=0;
    initialized=false;
    stvStack=eventSpatiotemporalVolumeStack();
    if(!_src.compare("icub"))
       source=new eventUnmaskICUB();
    else
        source=new eventUnmaskDVS128(_type);

    if(!_eye.compare("left"))
        eyeSel=0;
    else
        eyeSel=1;
}

extractionThread::~extractionThread()
{
    delete ptr_pose;
    delete ptr_nege;
    delete ptr_posk;
    delete ptr_negk;

    delete source;
}

void extractionThread::run()
{
    extract();
    //cerr << "Save the features" << endl;
    save();
}

int extractionThread::extract()
{
    int p_;
    unsigned int x_, y_, e_, t_;
    int res;
    if(!initialized)
    {
        cout << "\tInitialization:" << endl;
        res=initialize(x_, y_, p_, t_);
        feed(x_, y_, p_, t_);
        initialized=true;
    }
#ifdef _DEBUG_
    cout << "\t\tres after init: " << res << endl;
#endif
    cout << "Feed" << endl;
    do
    {
#ifdef _DEBUG_
    cout << "\t\tFeed..." << endl;
#endif
        //res=jaer.read(x_, y_, p_, t_);
#ifdef _DEBUG_
    cout << "\t\tRead..." << endl;
#endif
        //res=source->read(x_, y_, p_, t_);
        res=source->getUmaskedData(x_, y_, p_, e_, t_);
#ifdef _DEBUG_
        cout << "\t\tres in feed loop: " << res << endl;
#endif
        if(res && e_==eyeSel)
            feed(x_, y_, p_, t_); 
    }while(res!=2);
    cout << "Compute the PCAs" << endl;
    pca();
/*    cout << "Number of pca for the OFF events: " << nnege-1 << endl
         << "First elements: " << endl
         << "\tEigenval: " << endl
         << "\t\t" << ptr_nege[0] << " " << ptr_nege[1] << endl
         << "\tEigenvec:" << endl
         << "\t\t" << ptr_nege[2] << " " << ptr_nege[3] << endl
         << "\t\t" << ptr_nege[4] << " " << ptr_nege[5] << endl

         << "Last elements: " << endl
         << "\tEigenval: " << endl
         << "\t\t" << ptr_nege[(nnege-1)-5] << " " << ptr_nege[(nnege-1)-4] << endl
         << "\tEigenvec:" << endl
         << "\t\t" << ptr_nege[(nnege-1)-3] << " " << ptr_nege[(nnege-1)-2] << endl
         << "\t\t" << ptr_nege[(nnege-1)-1] << " " << ptr_nege[(nnege-1)-0] << endl

         << "Number of pca for the ON events: "  << npose-1 << endl
         << "First elements: " << endl
         << "\tEigenval: " << endl
         << "\t\t" << ptr_pose[0] << " " << ptr_pose[1] << endl
         << "\tEigenvec:" << endl
         << "\t\t" << ptr_pose[2] << " " << ptr_pose[3] << endl
         << "\t\t" << ptr_pose[4] << " " << ptr_pose[5] << endl
         << "Last elements: " << endl
         << "\tEigenval: " << endl
         << "\t\t" << ptr_pose[(npose-1)-5] << " " << ptr_pose[(npose-1)-4] << endl
         << "\tEigenvec:" << endl
         << "\t\t" << ptr_pose[(npose-1)-3] << " " << ptr_pose[(npose-1)-2] << endl
         << "\t\t" << ptr_pose[(npose-1)-1] << " " << ptr_pose[(npose-1)-0] << endl;
*/
    createKernels();
/*    cout << "Number of kernel for the OFF events: " << nnegk-1 << endl
         << "First elements: " << endl
         << "\tEigenval: " << endl
         << "\t\t" << ptr_negk[0] << " " << ptr_negk[1] << endl
         << "\tEigenvec:" << endl
         << "\t\t" << ptr_negk[2] << " " << ptr_negk[3] << endl
         << "\t\t" << ptr_negk[4] << " " << ptr_negk[5] << endl

         << "Last elements: " << endl
         << "\tEigenval: " << endl
         << "\t\t" << ptr_negk[(nnegk-1)-5] << " " << ptr_negk[(nnegk-1)-4] << endl
         << "\tEigenvec:" << endl
         << "\t\t" << ptr_negk[(nnegk-1)-3] << " " << ptr_negk[(nnegk-1)-2] << endl
         << "\t\t" << ptr_negk[(nnegk-1)-1] << " " << ptr_negk[(nnegk-1)-0] << endl

         << "Number of kernel for the ON events: "  << nposk-1 << endl
         << "First elements: " << endl
         << "\tEigenval: " << endl
         << "\t\t" << ptr_posk[0] << " " << ptr_posk[1] << endl
         << "\tEigenvec:" << endl
         << "\t\t" << ptr_posk[2] << " " << ptr_posk[3] << endl
         << "\t\t" << ptr_posk[4] << " " << ptr_posk[5] << endl
         << "Last elements: " << endl
         << "\tEigenval: " << endl
         << "\t\t" << ptr_posk[(nposk-1)-5] << " " << ptr_posk[(nposk-1)-4] << endl
         << "\tEigenvec:" << endl
         << "\t\t" << ptr_posk[(nposk-1)-3] << " " << ptr_posk[(nposk-1)-2] << endl
         << "\t\t" << ptr_posk[(nposk-1)-1] << " " << ptr_posk[(nposk-1)-0] << endl;
*/
    initialized=false;
    return 1;
}

int extractionThread::initialize(unsigned int& _x, unsigned int& _y, int& _p, unsigned int& _t)
{
    //int res=jaer.read(_x, _y, _p, _t);
    int res;
    unsigned int _e;
    do
    {
        //res=source->read(_x, _y, _p, _t);
        res=source->getUmaskedData(_x, _y, _p, _e, _t);
        //cout << "[extractionThread] initialization res: " << res << endl;
    }while(res!=2);
    return res; 
}

void extractionThread::feed(unsigned int& _x, unsigned int& _y, int& _p, unsigned int& _t)
{
    if(_x>floor(static_cast<double>(szSpace)/2.0) && _x<127-floor(static_cast<double>(szSpace)/2.0) &&
        _y>floor(static_cast<double>(szSpace)/2.0) && _y<127-floor(static_cast<double>(szSpace)/2.0))
    {
#ifdef _DEBUG_
    cout << "extractionThread::feed(...)" << endl;
#endif
    int res=stvStack.upper_bound(_t, szTemp);
#ifdef _DEBUG_
    cout << "\tres upper_bound: " << res << endl;
    cout << "\tsize of the stack: " << stvStack.size() << endl;
#endif
    if(res!=-1)
        for(int i=res+1; i<stvStack.size(); i++)
        {
#ifdef _DEBUG_
            cout << "\t\tAdd event at " << i << endl;
#endif
            if(     (_x<=(stvStack.at(i)->get_head()->get_x()+floor(static_cast<double>(szSpace)/2))) &&
                    (_x>=(stvStack.at(i)->get_head()->get_x()-floor(static_cast<double>(szSpace)/2))) &&
                    (_y<=(stvStack.at(i)->get_head()->get_y()+floor(static_cast<double>(szSpace)/2))) &&
                    (_y>=(stvStack.at(i)->get_head()->get_y()-floor(static_cast<double>(szSpace)/2))) )
            {
                stvStack.at(i)->add_evt(_x, _y, _p, _t);
            }
        }
#ifdef _DEBUG_
            cout << "\t\tCreate a new volume..." << endl;
#endif
    stvStack.push(_x, _y, _p, _t); 
    }
}

void extractionThread::pca()
{
    unsigned int szStack=stvStack.size();
    cout << "Size of the stack: " << szStack << endl;
    npose=0; 
    ptr_pose=new double[6*szStack];
    cout << "\tCompute pos pca" << endl;
    computePCA(ptr_pose, npose, 1);

    nnege=0;
    ptr_nege=new double[6*szStack];
    cout << "\tCompute neg pca" << endl;
    computePCA(ptr_nege, nnege, -1);
}

int extractionThread::computePCA(double *eig, unsigned int& n, int pol)
{
    unsigned int szStack=stvStack.size();
    while(szStack)
    {
        --szStack;
//        cout << "Get the number of event from the current stv" << endl;
        int nEvts=stvStack.at(szStack)->get_numberOfevent();
//        cout << "\t-> " << nEvts << endl;
        if(nEvts>=szTemp/1000)
        {
            int res;
//            cout << "Declaration of the matrix Atmp..." << endl;
            Matrix Atmp(nEvts, dim);
            Matrix checkRank(szSpace, szSpace);
//            cout << "Rewind the stv" << endl;
            stvStack.at(szStack)->rewind();
//            cout << "Fill Atmp and checkrank..." << endl;
            int nEventOfGoodPol=0;
            unsigned int x;
            unsigned int y;

            for(int i=0; i<nEvts; i++)
            {
                if(stvStack.at(szStack)->get_pointed()->get_pol()==pol)
                {
                    x=(stvStack.at(szStack)->get_pointed()->get_x()-stvStack.at(szStack)->get_head()->get_x())+                floor(static_cast<double>(szSpace)/2.0);
                    y=(stvStack.at(szStack)->get_pointed()->get_y()-stvStack.at(szStack)->get_head()->get_y())+                floor(static_cast<double>(szSpace)/2.0);
//                    cout << "x: " << x << ", y: " << y << endl;
                    Atmp(nEventOfGoodPol, 0)=static_cast<double>(x);
                    Atmp(nEventOfGoodPol, 1)=static_cast<double>(y);
                    checkRank(x, y)=static_cast<double>(stvStack.at(szStack)->get_pointed()->get_ts());
                    if(dim==3)
                        Atmp(nEventOfGoodPol, 2)=static_cast<double>(stvStack.at(szStack)->get_pointed()->get_ts());
                    nEventOfGoodPol++;
                }
                if(!stvStack.at(szStack)->forward())
                    break;
            }
            if(nEventOfGoodPol>3)
            {

//                cout << "Compute the rank of the matrix" << endl;
                int rank=0;
                //Compute the rank...
                Matrix U(szSpace, szSpace);
                Vector S(szSpace);
                Matrix V(szSpace, szSpace);
                SVDMod (checkRank, U, S, V);
                for(int i=0; i<szSpace; i++)
                {
                    if(S(i)>1E-6)
                        rank+=1;
                }
                //if(1)
                if(rank>=szSpace-1)
                {
//                cout << "Create the matrix A with the valid element of Atmp..." << endl;
                    Matrix A(nEventOfGoodPol, dim);
                    A=Atmp.submatrix(0, nEventOfGoodPol-1, 0, dim-1);
//                    cout << "Declaration of the matrices and vector for the svd computation..." << endl;
                    U.resize(nEventOfGoodPol, dim);
                    S.resize(dim);
                    V.resize(dim, dim);
                    Vector colAi(nEvts);
                    double meanColA;
//                    cout << "Prepare the data for the svd computation" << endl;
                    for(int i=0; i<dim; i++)
                    {   
                        colAi=A.getCol(i);
                        A.setCol(i, -mean(colAi.data(), nEvts)+colAi); 
                    }
                    A/=((double)dim-1);
//                    cout << "Compute the svd" << endl;
                    SVDMod (A, U, S, V);
//                    cout << "Save the components of the pca" << endl;
//                    cout << "Copy data at &eval+" << 2*n << " to " << 2*n+2*sizeof(double)-1  << "/" << 2*stvStack.size() << endl;
//                    cout << "Copy data at &evec+" << 4*n << " to " << 4*n+4*sizeof(double)-1  << "/" << 4*stvStack.size() << endl;
//                    cout << "Compute the eigenvalues from the singularvalues..." << endl;
                    S=S*S;
                    double Q=yarp::math::norm(S);//sqrt((S(0)*S(0)+S(1)*S(1)));
                    if(Q>1E-6)
                    {
//                        cout << "Save the eigencomponent" << endl;
                        S/=Q;
                        memcpy(eig+6*n, S.data(),2*sizeof(double));
                        memcpy(eig+6*n+2, V.data(),4*sizeof(double));
                        n++;
                    }
                }
//                else
//                    stvStack.pop(szStack);
            }
        }
//        else
//            stvStack.pop(szStack);
    }
    return 0;
}

void extractionThread::createKernels()
{
    nposk=0;
    ptr_posk=new double[6*npose];
    bruteForce(ptr_pose, npose, ptr_posk, nposk);

    nnegk=0;
    ptr_negk=new double[6*nnege];
    bruteForce(ptr_nege, nnege, ptr_negk, nnegk);
}
int extractionThread::bruteForce(double* eige, unsigned int& ne, double* eigk, unsigned int& nk)
{
    memcpy(eigk, eige, (dim+dim*dim)*sizeof(double));
    nk++;

    unsigned int *nMember=new unsigned int[ne];
    memset(nMember, 0, ne*sizeof(int));

    int res;

    double norme;
    double normk;
    double mulek;

    double simval_0;
    double simval_1;
    double simval_2;
    double simvec_0;
    double simvec_1;
    double simvec_2;

    Vector vece_0(dim);
    Vector vece_1(dim);
    Vector vece_2(dim);
    Vector veck_0(dim);
    Vector veck_1(dim);
    Vector veck_2(dim);
    
    bool found;
    for(unsigned int i=0; i<ne; i++)
    {
        found=false;
        for(unsigned int ii=0; ii<nk; ii++)
        {
            if(dim==2)
            {
                simval_0=eige[6*i]/eigk[6*ii];
                simval_1=eige[6*i+1]/eigk[6*ii+1];
                if( (( simval_0>=eigenvalSim && eige[6*i]<=eigk[6*ii] ) || ( simval_0<=1/eigenvalSim && eige[6*i]>=eigk[6*ii] )) &&
                    (( simval_1>=eigenvalSim && eige[6*i+1]<=eigk[6*ii+1] ) || ( simval_1<=1/eigenvalSim && eige[6*i+1]>=eigk[6*ii+1] )) )
//                if( (( simval_0>=eigenvalSim ) || ( simval_0<=1/eigenvalSim )) &&
//                    (( simval_1>=eigenvalSim ) || ( simval_1<=1/eigenvalSim )) )
                {
                    memcpy(vece_0.data(), eige+(6*i+2), 2*sizeof(double));
                    memcpy(veck_0.data(), eigk+(6*ii+2), 2*sizeof(double));
                    simvec_0=dot(vece_0, veck_0)/(norm(vece_0)*norm(veck_0));

                    memcpy(vece_1.data(), eige+(6*i+4), 2*sizeof(double));
                    memcpy(veck_1.data(), eigk+(6*ii+4), 2*sizeof(double));
                    simvec_1=dot(vece_1, veck_1)/(norm(vece_1)*norm(veck_1));

                    if(abs(simvec_0)>=eigenvecSim && abs(simvec_1)>=eigenvecSim)
                    {
                        found=true;
                        nMember[ii]++;
                        veck_0=veck_0+((1/nMember[ii])*(vece_0-veck_0));
                        veck_1=veck_1+((1/nMember[ii])*(vece_1-veck_1));
                        memcpy(eigk+(6*ii+2), veck_0.data(), 2*sizeof(double));
                        memcpy(eigk+(6*ii+4), veck_1.data(), 2*sizeof(double));
                        break;
                    }
                    //else
                    //    cout << "Not okay because of the eigenvec, simvec_0=" << simvec_0 << "<>" << eigenvecSim << ", simvec_1=" << simvec_1 << "<>" << eigenvecSim << endl; 
                }
                //else
                //    cout << "Not okay because of eigenval, simval_0=" << simval_0 << "<>" << eigenvalSim << ", simval_1=" << simval_1 << "<>" << eigenvalSim << endl;
            }
            else if(dim==3)
            {
                cout << "dim 3" << endl;
            }
        }
        if(found==false)
        {
            memcpy(eigk+nk*(dim+dim*dim), eige+i*(dim+dim*dim), (dim+dim*dim)*sizeof(double));
            nk++;
        }
    }
    delete nMember;
}

double extractionThread::mean(double* _data, unsigned int _sz)
{
    double res=0;
    for(unsigned int i=0; i<_sz; i++)
        res+=_data[i];
    return res/_sz;
}

void extractionThread::save()
{
    stringstream concatenation;
    ofstream of;
    time_t rawtime;
    time ( &rawtime );
    string subdate=ctime(&rawtime);
    size_t found=subdate.find(' ');
    while(found!=string::npos)
    {
        subdate.replace(found, 1, "_");
        found=subdate.find(' ');
    }

    concatenation << subdate.substr(0, subdate.size()-1) << "_" << eigenvalSim << "_" << eigenvecSim << "_" << szSpace << "_" << szTemp << "_eye" << eyeSel << "_base_pos.data";
    of.open(concatenation.str().c_str(), ios::binary);
    double *toSave=new double[nposk*(dim+dim*dim)+2];
    toSave[0]=static_cast<double>(nposk);
    toSave[1]=static_cast<double>(dim);
    memcpy(toSave+2, ptr_posk, nposk*(dim+dim*dim)*sizeof(double));
    //of.write(static_cast<char*>(toSave), (1+nposk*(dim+dim*dim))*sizeof(double));
    of.write(reinterpret_cast<char*>(toSave), (2+nposk*(dim+dim*dim))*sizeof(double));
    of.close();
    delete toSave;

    concatenation.str("");

    concatenation << subdate.substr(0, subdate.size()-1) << "_" << eigenvalSim << "_" << eigenvecSim << "_" << szSpace << "_" << szTemp << "_eye" << eyeSel << "_base_neg.data";
    of.open(concatenation.str().c_str(), ios::binary);
    toSave=new double[nnegk*(dim+dim*dim)+2];
//    cout << "Copy nnegk (" << nnegk << ") in the allocated block of memory" << endl;
    toSave[0]=static_cast<double>(nnegk);
//    cout << "Copy dim (" << dim << ") in the allocated block of memory" << endl;
    toSave[1]=static_cast<double>(dim);
    memcpy(toSave+2, ptr_negk, nnegk*(dim+dim*dim)*sizeof(double));
    //of.write(static_cast<char*>(toSave), (1+nnegk*(dim+dim*dim))*sizeof(double));
    of.write(reinterpret_cast<char*>(toSave), (2+nnegk*(dim+dim*dim))*sizeof(double));
    of.close();
    delete toSave;
}

//http://www.cplusplus.com/reference/string/string/find_last_of/
void extractionThread::splitFilename (const string& str, string& _name)
{
  size_t found;
//  cout << "Splitting: " << str << endl;
  found=str.find_last_of("/\\");
//  cout << " folder: " << str.substr(0,found) << endl;
//  cout << " file: " << str.substr(found+1) << endl;
    _name = str.substr(found+1);
}

void extractionThread::setBuffer(char* _buf, uint _sz)
{
    //cout << "Set buffer in unmask instance" << endl;
    source->setBuffer(_buf, _sz);
}
