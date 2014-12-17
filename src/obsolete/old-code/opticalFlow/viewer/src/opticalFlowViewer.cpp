#include "opticalFlowViewer.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;
using namespace yarp::math;

opticalFlowViewer::opticalFlowViewer()
{
    x.resize(HEIGHT);
    y.resize(WIDTH);

    vx.resize(HEIGHT, WIDTH);
    vy.resize(HEIGHT, WIDTH);

    for(int i=0; i<WIDTH; i++)
    {
        x(i)=i;
        y(i)=i;
    }

    vx.zero();
    vy.zero();

    indQuiv = 0;

    black = 0;
    base_img.resize((10*WIDTH), (10*HEIGHT));
    for(int i=0; i<(10*WIDTH); i++)
    {
        for(int j=0; j<(10*HEIGHT); j++)
        {
            base_img(i,j) = 125;
        }
    }
    port.open("/image/opticalFlowFrame:o");

    clock_gettime(CLOCK_REALTIME, &start);
}
opticalFlowViewer::~opticalFlowViewer()
{
}
void opticalFlowViewer::onRead(vecBuffer& i_vec)
{
    cout << "Receive events" << endl;
    cout << "addr received: " << i_vec.get_x() << " / " << i_vec.get_y() << endl;
    printf("Velocity received %f %f\n", i_vec.get_vx(), i_vec.get_vy());

    clock_gettime(CLOCK_REALTIME, &current);
    unsigned long int diff_time = ((current.tv_sec*1E9+current.tv_nsec)-(start.tv_sec*1E9+start.tv_nsec));
    double normalization = sqrt(i_vec.get_vx()*i_vec.get_vx()+i_vec.get_vy()*i_vec.get_vy());
    vx( i_vec.get_x(), i_vec.get_y() ) = i_vec.get_vx()/normalization;
    vy( i_vec.get_x(), i_vec.get_y() ) = i_vec.get_vy()/normalization;

//    cout << "current.tv_sec : " << current.tv_sec << " | current.tv_nsec : " << current.tv_nsec << " | current.tv_sec*1E9+current.tv_nsec : " << current.tv_sec*1E9+current.tv_nsec << endl;
//    cout << diff_time << endl;

    if( diff_time>=TNK_TIME)
    {
//        cout << "Will send the vector as an image" << endl;
        ImageOf<PixelMono16> img=base_img;
        Matrix addVec=vx+vy;
        Matrix inds = findNz(addVec, 0.0);//(addVec!=0);
        for(int i=0; i<inds.rows(); i++)
        {
            int X=inds(i, 0);//inds(i)%HEIGHT;
            int Y=inds(i, 1);//floor(inds(i)/WIDTH);
            int origX=5+10*(X);
            int origY=5+10*(127-Y);
//            addSegment(img, black, origX, origY, (int)(origX+vx(X,Y)*30+0.5), (int)(origY-vy(X,Y)*30+0.5));
//            addSegment(img, black, origX, origY, (int)(origX+vx(X,Y)*30+0.5), (int)(origY+vy(X,Y)*30+0.5));
//            addSegment(img, black, origX, origY, (int)(origX-vx(X,Y)*30+0.5), (int)(origY-vy(X,Y)*30+0.5));

//            addSegment(img, black, origY, origX, (int)(origY-vy(X,Y)*30+0.5), (int)(origX+vx(X,Y)*30+0.5));
//            addSegment(img, black, origY, origX, (int)(origY+vy(X,Y)*30+0.5), (int)(origX+vx(X,Y)*30+0.5));

            addSegment(img, black, origX, origY, (int)(origX+vy(X,Y)*30+0.5), (int)(origY-vx(X,Y)*30+0.5));
//            addSegment(img, black, origX, origY, (int)(origX-vy(X,Y)*30+0.5), (int)(origY+vx(X,Y)*30+0.5));
//            addSegment(img, black, origX, origY, (int)(origX+vy(X,Y)*30+0.5), (int)(origY+vx(X,Y)*30+0.5));
        }
        send_frame(img);

//        ssBuffer << "vx_" << indQuiv << ".txt";
//        vx.save(ssBuffer.str(), raw_ascii);
//        ssBuffer.str("");
//        ssBuffer << "vy_" << indQuiv << ".txt";
//        vy.save(ssBuffer.str(), raw_ascii);
//        ssBuffer.str("");
//        indQuiv++;

        vx.zero();
        vy.zero();
        clock_gettime(CLOCK_REALTIME, &start);
    }
}
void opticalFlowViewer::send_frame(ImageOf<PixelMono16> i_img)
{
//    cout << "Send image of vector" << endl;
    ImageOf<PixelMono16>& tmp = port.prepare();
    tmp = i_img;
    port.write();
}

Matrix opticalFlowViewer::findNz(Matrix& A, double hit)
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
