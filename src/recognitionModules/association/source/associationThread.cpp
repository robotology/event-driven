#include "associationThread.hpp"

#define MAXCMD 100

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace emorph::eunmask;
using namespace emorph::evolume;
using namespace emorph::ehist;

associationThread::associationThread()
//:RateThread(RATETH), featuresFile(""), szSpace(0), szTemp(0), eigenvalSim(0), eigenvecSim(0), dim(0), learn(false)
:featuresFile(""), szSpace(0), szTemp(0), eigenvalSim(0), eigenvecSim(0), dim(0), learn(false)
{
    ptr_pose=0;
    ptr_nege=0;
    ptr_posk=0;
    ptr_negk=0;
    ptr_posh=0;
    ptr_negh=0;

    lastUpperBound=-1;
}

associationThread::associationThread(string _src, unsigned int _type, string _eye, string _features, unsigned int _szSpace, unsigned int _szTemp, double _eigenvalSim, double _eigenvecSim, unsigned int _dim, bool _learn, yarp::os::BufferedPort<emorph::ehist::eventHistBuffer> *_port)
//:RateThread(RATETH), featuresFile(_features), szSpace(_szSpace), szTemp(_szTemp), eigenvalSim(_eigenvalSim), eigenvecSim(_eigenvecSim), dim(_dim), learn(_learn)
:featuresFile(_features), szSpace(_szSpace), szTemp(_szTemp), eigenvalSim(_eigenvalSim), eigenvecSim(_eigenvecSim), dim(_dim), learn(_learn)
{
    ptr_pose=0;
    ptr_nege=0;
    ptr_posk=0;
    ptr_negk=0;
    ptr_posh=0;
    ptr_negh=0;

    lastUpperBound=-1;

    if(!_eye.compare("left"))
        eyeSel=0;
    else
        eyeSel=1;

    stringstream concatenation;
    ofstream of;
/*    cout << "*******************************************" << endl
         << "*            Loading features             *" << endl
         << "*******************************************" << endl;
    cout << "Load the negative features..." << endl;
*/
    //concatenation << featuresFile << "_" << szSpace << "_" << szTemp << "_base_neg.data";
    concatenation << featuresFile << "_" << eigenvalSim << "_" << eigenvecSim << "_" << szSpace << "_" << szTemp << "_eye" << eyeSel << "_base_neg.data";
    cout << "[associationThread] load file \"" << concatenation.str() << "\"" << endl;
    int res=loadFeatures(concatenation.str().c_str(), &ptr_negk, nnegk);
    if(res==0)
    {
/*        cout << "Pointed addr of ptr_negk: " << ptr_negk << endl
             << "Number of kernel for the OFF events: " << nnegk-1 << endl
             << "First elements: " << endl
             << "\tEigenval: " << endl
             << "\t\t" << ptr_negk[0] << " " << ptr_negk[1] << endl
             << "\tEigenvec:" << endl
             << "\t\t" << ptr_negk[2] << " " << ptr_negk[3] << endl
             << "\t\t" << ptr_negk[4] << " " << ptr_negk[5] << endl;

        cout << "Last elements: " << endl
             << "\tEigenval: " << endl
             << "\t\t" << ptr_negk[(nnegk-1)-5] << " " << ptr_negk[(nnegk-1)-4] << endl
             << "\tEigenvec:" << endl
             << "\t\t" << ptr_negk[(nnegk-1)-3] << " " << ptr_negk[(nnegk-1)-2] << endl
             << "\t\t" << ptr_negk[(nnegk-1)-1] << " " << ptr_negk[(nnegk-1)-0] << endl;
*/
        cout << "[associationThread] File loaded" << res << endl;
    }
    else
        cerr << "Load negative features failed, error " << res << endl;
        
    concatenation.str("");

//    cout << "Load the positive features..." << endl;
    //concatenation << featuresFile << "_" << szSpace << "_" << szTemp << "_base_pos.data";
    concatenation << featuresFile << "_" << eigenvalSim << "_" << eigenvecSim << "_" << szSpace << "_" << szTemp << "_eye" << eyeSel << "_base_pos.data";
    cout << "[associationThread] load file \"" << concatenation.str() << "\"" << endl;
    res=loadFeatures(concatenation.str().c_str(), &ptr_posk, nposk);
    if(res==0)
    {
/*        cout << "Number of kernel for the ON events: "  << nposk-1 << endl
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
        cout << "[associationThread] File loaded" << res << endl;
    }
    else
        cerr << "Load positive features failed, error " << res << endl;

    if(!_src.compare("icub"))
       target=new eventUnmaskICUB();
    else
        target=new eventUnmaskDVS128(_type);

    stvStack=eventSpatiotemporalVolumeStack();

    initialized=false;

    imgPreview = cv::Mat::ones(128, 128, CV_8U)*126;

    //outputPort.open("/associationThread:o");
    outputPort=_port;
}

associationThread::~associationThread()
{
    if(ptr_pose!=0)
        delete ptr_pose;
    if(ptr_nege!=0)
        delete ptr_nege;
    if(ptr_posk!=0)
        delete ptr_posk;
    if(ptr_negk!=0)
        delete ptr_negk;
    if(ptr_posh!=0)
        delete[] ptr_posh;
    if(ptr_negh!=0)
        delete[] ptr_negh;

    delete target;
}

void associationThread::run()
{
    while(1)
    {
    //    cout << "Start extraction..." << endl;
        int res=associate();
    //    cout << "Exctraction finished" << endl;
        if(res!=-1)
        {
            sendhist();
            if(learn)
            {
                save();
                cout << "[associationThread] Association saved" << endl;
                imgPreview = cv::Mat::ones(128, 128, CV_8U)*126;
            }
        }
        else
            cerr << "Association error: No synch found" << endl;
        stvStack.reset();
        target->reset();
        initialized=false; 
    }
}

int associationThread::loadFeatures(std::string _file, double** _ptr_k, unsigned int& _nk)
{
    double _dim;
    double __nk;
    ifstream ifile;

    ifile.open(_file.c_str(), ios::binary);
    if(!ifile.is_open())
        return -2;
    ifile.read(reinterpret_cast<char*>(&__nk), sizeof(double)); 
    _nk=static_cast<unsigned int>(__nk);
//    cout    << "\tNumber of features in " << _file << ": " << __nk << endl
//            << "\tCast as an uint: " << _nk << endl;
    ifile.read(reinterpret_cast<char*>(&_dim), sizeof(double));
//    cout << "\tDimension of the features in " << _file << ": " << _dim << endl
//            << "\tCast as an uint: " << static_cast<unsigned int>(_dim) << endl;
    if(static_cast<unsigned int>(_dim)!=dim)
        return -1;
//    cout << "\tAllocate a block of memory to store the data from file..." << endl;
    *_ptr_k=new double[_nk*(dim+dim*dim)];
//    cout << "Pointed addr of _ptr_k: " << *_ptr_k << endl;
//    cout << "\tLoad the data from file..." << endl;
    ifile.read(reinterpret_cast<char*>(*_ptr_k), _nk*(dim+dim*dim)*sizeof(double));
//    cout << "Data stored" << endl;
    ifile.close();
    return 0;
}

int associationThread::associate()
{
    int p_;
    unsigned int x_, y_, e_, t_, fed;
    fed=0;
//    cout << "\tInitialization:" << endl;
    int res=0;
    if(!initialized)
    {
        res=initialize(x_, y_, p_, t_);
        feed(x_, y_, p_, t_); 
        initialized=true;
    }
    cout << "[associationThread] Initialized" << endl;
    refts=t_;
    //int res=target->read(x_, y_, p_, t_);
    do
    {
        res=target->getUmaskedData(x_, y_, p_, e_, t_);
        t_=t_-refts;
        if(res && e_==eyeSel)
        {
            feed(x_, y_, p_, t_);
            fed++;
            if(refts+5000>=t_)
                createPreview(x_, y_, p_);
        }
    }while(res!=2);
    cout << "[associationThread] Second synch received, fed=" << fed << endl;
    
//    cout << "Occurence of the feeding loop: " << fed << endl;
    if(!fed)
        return -1;
//    cout << "Compute the PCAs" << endl;
    pca();
/*    cout << "Number of pca for the OFF events: " << nnege-1 << endl
         << "First elements: " << endl
         << "\tEigenval: " << endl
         << "\t\t" << ptr_nege[0] << " " << ptr_nege[1] << endl
         << "\tEigenvec:" << endl
         << "\t\t" << ptr_nege[2] << " " << ptr_nege[3] << endl
         << "\t\t" << ptr_nege[4] << " " << ptr_nege[5] << endl
         << "\tat: " << ptr_nege[6] << " (" << ptr_nege[7] << ", " << ptr_nege[8] << ")" << endl

         << "Last elements: " << endl
         << "\tEigenval: " << endl
         << "\t\t" << ptr_nege[(dim+dim*dim+3)*(nnege-1)+0] << " " << ptr_nege[(dim+dim*dim+3)*(nnege-1)+1] << endl
         << "\tEigenvec:" << endl
         << "\t\t" << ptr_nege[(dim+dim*dim+3)*(nnege-1)+2] << " " << ptr_nege[(dim+dim*dim+3)*(nnege-1)+3] << endl
         << "\t\t" << ptr_nege[(dim+dim*dim+3)*(nnege-1)+4] << " " << ptr_nege[(dim+dim*dim+3)*(nnege-1)+5] << endl
         << "\tat: " << ptr_nege[(dim+dim*dim+3)*(nnege-1)+6] << " (" << ptr_nege[(dim+dim*dim+3)*(nnege-1)+7] << ", " << ptr_nege[(dim+dim*dim+3)*(nnege-1)+8] << ")" << endl

         << "Number of pca for the ON events: "  << npose-1 << endl
         << "First elements: " << endl
         << "\tEigenval: " << endl
         << "\t\t" << ptr_pose[0] << " " << ptr_pose[1] << endl
         << "\tEigenvec:" << endl
         << "\t\t" << ptr_pose[2] << " " << ptr_pose[3] << endl
         << "\t\t" << ptr_pose[4] << " " << ptr_pose[5] << endl
         << "\tat: " << ptr_pose[6] << " (" << ptr_pose[7] << ", " << ptr_pose[8] << ")" << endl

         << "Last elements: " << endl
         << "\tEigenval: " << endl
         << "\t\t" << ptr_pose[(dim+dim*dim+3)*(npose-1)+0] << " " << ptr_pose[(dim+dim*dim+3)*(npose-1)+1] << endl
         << "\tEigenvec:" << endl
         << "\t\t" << ptr_pose[(dim+dim*dim+3)*(npose-1)+2] << " " << ptr_pose[(dim+dim*dim+3)*(npose-1)+3] << endl
         << "\t\t" << ptr_pose[(dim+dim*dim+3)*(npose-1)+4] << " " << ptr_pose[(dim+dim*dim+3)*(npose-1)+5] << endl
         << "\tat: " << ptr_pose[(dim+dim*dim+3)*(npose-1)+6] << " (" << ptr_pose[(dim+dim*dim+3)*(npose-1)+7] << ", " << ptr_pose[(dim+dim*dim+3)*(npose-1)+8] << ")" << endl;
*/
//    cout << "*************************************************" << endl
//         << "*            Compute the distance               *" << endl
//         << "*************************************************" << endl;
    computeDistances();
//    cout << "Number of negative stv associated: " << nnegh << endl
//         << "\tFirst: " << ptr_negh[0] << " at " << ptr_negh[1] << endl
//         << "\tLast: " << ptr_negh[4*(nnegh-1)] << " at " << ptr_negh[4*(nnegh-1)+1] << " (" << ptr_negh[4*(nnegh-1)+2] << ", " << ptr_negh[4*(nnegh-1)+3] << ")" << endl;
//    cout << "Number of positive stv associated: " << nposh << endl
//         << "\tFirst: " << ptr_posh[0] << " at " << ptr_posh[1] << endl
//         << "\tLast: " << ptr_posh[4*(nposh-1)] << " at " << ptr_posh[4*(nposh-1)+1] << " (" << ptr_posh[4*(nposh-1)+2] << ", " << ptr_posh[4*(nposh-1)+3] << ")" << endl;
    return 1;
}

int associationThread::initialize(unsigned int& _x, unsigned int& _y, int& _p, unsigned int& _t)
{
    int res=0;
    unsigned int _e;
    do
    {
        res=target->getUmaskedData(_x, _y, _p, _e, _t);
    }while(res!=2);
    return res; 
}

void associationThread::feed(unsigned int& _x, unsigned int& _y, int& _p, unsigned int& _t)
{
    //cout << "ts in feed: " << _t << endl;
    if(_x>floor(static_cast<double>(szSpace)/2.0) && _x<127-floor(static_cast<double>(szSpace)/2.0) &&
        _y>floor(static_cast<double>(szSpace)/2.0) && _y<127-floor(static_cast<double>(szSpace)/2.0))
    {
        int res=stvStack.upper_bound(_t, szTemp);
        //cout << "upper_bound res: " << res << endl;
        if(res!=-1)
            for(int i=res+1; i<stvStack.size(); i++)
            {
                if(     (_x<=(stvStack.at(i)->get_head()->get_x()+floor(static_cast<double>(szSpace)/2))) &&
                    (_x>=(stvStack.at(i)->get_head()->get_x()-floor(static_cast<double>(szSpace)/2))) &&
                    (_y<=(stvStack.at(i)->get_head()->get_y()+floor(static_cast<double>(szSpace)/2))) &&
                    (_y>=(stvStack.at(i)->get_head()->get_y()-floor(static_cast<double>(szSpace)/2))) )
                {
                    stvStack.at(i)->add_evt(_x, _y, _p, _t);
                }
            }
        stvStack.push(_x, _y, _p, _t);
        //Compute the PCA on the flow, followed by the dist computation? (using lastUpperBound)
    }
}

void associationThread::pca()
{
    unsigned int szStack=stvStack.size();
    cout << "Size of  the stack: " << szStack << endl;
    npose=0;
    delete[] ptr_pose; 
    ptr_pose=new double[(dim+dim*dim+3)*szStack]; //dim: eigval; dim*dim: eigvec; 3: ts, cx, cy
//    cout << "\tCompute pos pca" << endl;
    computePCA(ptr_pose, npose, 1);
    cout << "[associationThread] Number of pos pca: " << npose << endl;
    nnege=0;
    delete[] ptr_nege;
    ptr_nege=new double[(dim+dim*dim+3)*szStack];
//    cout << "\tCompute neg pca" << endl;
    computePCA(ptr_nege, nnege, -1);
    cout << "[associationThread] Number of neg pca: " << nnege << endl;
}

int associationThread::computePCA(double *eig, unsigned int& n, int pol)
{
    unsigned int szStack=stvStack.size();
    while(szStack)
    {
        --szStack;
        //cout << "Get the number of event from the current stv" << endl;
        int nEvts=stvStack.at(szStack)->get_numberOfevent();
        //cout << "\t-> " << nEvts << endl;
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
                        memcpy(eig+(dim+dim*dim+3)*n, S.data(), 2*sizeof(double));
                        memcpy(eig+(dim+dim*dim+3)*n+2, V.data(), 4*sizeof(double));
                        eig[(dim+dim*dim+3)*n+6]=static_cast<double>(stvStack.at(szStack)->get_head()->get_ts());
                        eig[(dim+dim*dim+3)*n+7]=static_cast<double>(stvStack.at(szStack)->get_head()->get_x());
                        eig[(dim+dim*dim+3)*n+8]=static_cast<double>(stvStack.at(szStack)->get_head()->get_y());
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

void associationThread::computeDistances()
{
    nposh=0;
    delete[] ptr_posh;
    ptr_posh=new unsigned int[4*npose]; //4: kernelID + ts + cx + cy
    computeDistance(ptr_pose, npose, ptr_posk, nposk, ptr_posh, nposh);

    nnegh=0;
    delete[] ptr_negh;
    ptr_negh=new unsigned int[4*nnege];
    computeDistance(ptr_nege, nnege, ptr_negk, nnegk, ptr_negh, nnegh);
}

int associationThread::computeDistance(double* eige, unsigned int& ne, double* eigk, unsigned int& nk, unsigned int* hist, unsigned int& nh)
{
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
    double dist;
    double distc;
    unsigned int kernelID;
    for(unsigned int i=0; i<ne; i++)
    {
        dist=0;
        found=false;
        for(unsigned int ii=0; ii<nk; ii++)
        {
            if(dim==2)
            {
                simval_0=eige[(dim+dim*dim+3)*i]/eigk[(dim+dim*dim)*ii];
                simval_1=eige[(dim+dim*dim+3)*i+1]/eigk[(dim+dim*dim)*ii+1];
                if( (( simval_0>=eigenvalSim && eige[(dim+dim*dim+3)*i]<=eigk[(dim+dim*dim)*ii] ) || ( simval_0<=1/eigenvalSim && eige[(dim+dim*dim+3)*i]>=eigk[(dim+dim*dim)*ii] )) &&
                    (( simval_1>=eigenvalSim && eige[(dim+dim*dim+3)*i+1]<=eigk[(dim+dim*dim)*ii+1] ) || ( simval_1<=1/eigenvalSim && eige[(dim+dim*dim+1)*i+1]>=eigk[(dim+dim*dim)*ii+3] )) )
                {
                    memcpy(vece_0.data(), eige+((dim+dim*dim+3)*i+2), 2*sizeof(double));
                    memcpy(veck_0.data(), eigk+((dim+dim*dim)*ii+2), 2*sizeof(double));
                    simvec_0=dot(vece_0, veck_0)/(norm(vece_0)*norm(veck_0));

                    memcpy(vece_1.data(), eige+((dim+dim*dim+3)*i+4), 2*sizeof(double));
                    memcpy(veck_1.data(), eigk+((dim+dim*dim)*ii+4), 2*sizeof(double));
                    simvec_1=dot(vece_1, veck_1)/(norm(vece_1)*norm(veck_1));

                    if(abs(simvec_0)>=eigenvecSim && abs(simvec_1)>=eigenvecSim)
                    {
                        distc=(abs(simvec_0)+abs(simvec_1))/2;
                        if(distc>dist)
                        {
                            dist=distc;
                            kernelID=ii;
                        }
                        found=true;
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
        }//for nk
        if(found)
        {
            hist[4*nh]=kernelID;
            hist[4*nh+1]=eige[(dim+dim*dim+3)*i+6];
            hist[4*nh+2]=eige[(dim+dim*dim+3)*i+7];
            hist[4*nh+3]=eige[(dim+dim*dim+3)*i+8];
            nh++;
        }
    }
}

double associationThread::mean(double* _data, unsigned int _sz)
{
    double res=0;
    for(unsigned int i=0; i<_sz; i++)
        res+=_data[i];
    return res/_sz;
}

void associationThread::save()
{
    string fileName;
    //splitFilename(featuresFile, fileName);
    //splitFilename(dataFile, fileName);
    stringstream concatenation;
    stringstream listOfFile;
    ofstream fd;
    ofstream fdImg;
    ofstream of;
    
    listOfFile << eigenvalSim << "_" << eigenvecSim << "_" << szSpace << "_" << szTemp << "_eye" << eyeSel << ".list";

/*    cout << "Enter a name for the new knowledge file: " << endl;
    char cmdl[MAXCMD];
    char c='\0';
    unsigned int ic=0;
    cout << "$ ";
    while(c!='\n' && ic<MAXCMD-1)
    {
        c=getchar();
        cmdl[ic++]=c;
    }
    cmdl[ic-1]='\0';
    cout << "[CHAR*] cmdl=" << cmdl << endl;
    fileName=cmdl;
    cout << "[STRING] cmdl=" << fileName << endl;
//    cin >> fileName;
*/
    time_t rawtime;
    time ( &rawtime );
    string subdate=ctime(&rawtime);
    size_t found=subdate.find(' ');
    while(found!=string::npos)
    {
        subdate.replace(found, 1, "_");
        found=subdate.find(' ');
    }
    fileName=subdate.substr(0, subdate.size()-1);

    concatenation << fileName << "_" << eigenvalSim << "_" << eigenvecSim << "_" << szSpace << "_" << szTemp << "_eye" << eyeSel << "_hist_pos.data";
    if(!verifIfExist(listOfFile.str(), concatenation.str()))
    {
        fd.open(listOfFile.str().c_str(), ios::app);
        fd.write(concatenation.str().c_str(), concatenation.str().size());
        fd.put('\n');
        fd.close();
    }

    of.open(concatenation.str().c_str(), ios::binary);
    unsigned int *toSave=new unsigned int[nposh*4+2];
    toSave[0]=nposk;
    toSave[1]=nposh;
    memcpy(toSave+2, ptr_posh, nposh*4*sizeof(unsigned int));
    of.write(reinterpret_cast<char*>(toSave), (2+nposh*4)*sizeof(unsigned int));
    of.close();
    delete toSave;

    concatenation.str("");
    concatenation << fileName << "_" << eigenvalSim << "_" << eigenvecSim << "_" << szSpace << "_" << szTemp << "_eye" << eyeSel << "_hist_neg.data";
    if(!verifIfExist(listOfFile.str(), concatenation.str()))
    {
        fd.open(listOfFile.str().c_str(), ios::app);
        fd.write(concatenation.str().c_str(), concatenation.str().size());
        fd.put('\n');
        fd.close();
    }
    of.open(concatenation.str().c_str(), ios::binary);
    toSave=new unsigned int[nnegh*4+2];
    toSave[0]=nnegk;
    toSave[1]=nnegh;
    memcpy(toSave+2, ptr_negh, nnegh*4*sizeof(unsigned int));
    //of.write(static_cast<char*>(toSave), (1+nnegk*(dim+dim*dim))*sizeof(double));
    of.write(reinterpret_cast<char*>(toSave), (2+nnegh*4)*sizeof(unsigned int));
    of.close();
    delete toSave;

    concatenation.str("");

////////// SAVE PREVIEW
    listOfFile.str("");
    listOfFile << eigenvalSim << "_" << eigenvecSim << "_" << szSpace << "_" << szTemp << "_eye" << eyeSel << ".img";
    concatenation << fileName << "_" << eigenvalSim << "_" << eigenvecSim << "_" << szSpace << "_" << szTemp << "_eye" << eyeSel << "_preview.pgm";
    //imshow("preview", imgPreview);
    //cv::waitKey(0);
    imwrite(concatenation.str().c_str(), imgPreview);
    if(!verifIfExist(listOfFile.str(), concatenation.str()))
    {
        fdImg.open(listOfFile.str().c_str(), ios::app);
        fdImg.write(concatenation.str().c_str(), concatenation.str().size());
        fdImg.put('\n');
        fdImg.close();
    }
    concatenation.str("");
}

//http://www.cplusplus.com/reference/string/string/find_last_of/
void associationThread::splitFilename (const string& str, string& _name)
{
  size_t found;
//  cout << "Splitting: " << str << endl;
  found=str.find_last_of("/\\");
//  cout << " folder: " << str.substr(0,found) << endl;
//  cout << " file: " << str.substr(found+1) << endl;
    _name = str.substr(found+1);
}

bool associationThread::verifIfExist(string _file, string _line)
{
    ifstream fd;
    string dfile;
    fd.open(_file.c_str());
    //fd.open(_file.c_str(), ios::in|ios::app);
//    cout << "Look for " << _line << endl;
    //fd.seekg(0, ios::beg);
//    cout << "Position: " << fd.tellg() << endl; 

    bool exist=false;
//    int ic;
//    char c='\0';
//    char *ptr_line=new char[100];
    while(fd.good())
    {
//        c='\0';
//        ic=-1;
//        memset(ptr_line, 0, 100);
//        while(ic<99 && !fd.eof())
//        {
            getline(fd, dfile);
//            ic++;
//            fd.get(c);
//            if(c=='\n')
//                break;
//            *(ptr_line+ic)=c;
//        }
//        *(ptr_line+ic)='\0';
//        cout << "Compare with " << ptr_line << endl;
//        if(!_line.compare(ptr_line))
        if(!dfile.compare(_line))
        {
//            cout << "Line found" << endl;
            exist=true;
            break;
        }
    }
    fd.close();
    return exist;
}

unsigned int associationThread::get_sizeh(string _pol)
{
    if(!_pol.compare("neg"))
        return nnegh;
    else if(!_pol.compare("pos"))
        return nposh;
    return 0;
}

unsigned int* associationThread::get_ptrh(string _pol)
{
    if(!_pol.compare("neg"))
        return ptr_negh;
    else if(!_pol.compare("pos"))
        return ptr_posh;
    return NULL;
}

void associationThread::sendhist()
{
    cout << "[associationThread] Size of the positive hist: " << nposh << endl
         << "[associationThread] tstamp of the first elem of the positive hist: " << ptr_posh[1] << endl
         << "[associationThread] size of the negative hist: " << nnegh << endl
         << "[associationThread] tstamp of the first elem of the negative hist: " << ptr_negh[1] << endl;

    eventHistBuffer outputBuffer(ptr_posh, nposh, ptr_negh, nnegh);
    eventHistBuffer& tmp = outputPort->prepare();
    tmp = outputBuffer;
    outputPort->write();
}

void associationThread::setBuffer(char* _buf, unsigned int _sz)
{
    target->setBuffer(_buf, _sz);
}

void associationThread::createPreview(unsigned int _x, unsigned int _y, int _pol)
{
    imgPreview.at<char>(_x,_y)+=(char)(20*_pol);
}
