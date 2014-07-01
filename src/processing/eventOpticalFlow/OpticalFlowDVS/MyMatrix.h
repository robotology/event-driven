/* 
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Fouzhan Hosseini
 * email:  fouzhan.hosseini@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/


#ifndef MYMATRIX_H_
#define MYMATRIX_H_

#include <vector>
#include <iostream>

using namespace std;

template< class T>
class MyMatrix {
	vector<T> storages;
	int nRows;
	int nCols;

public:

    MyMatrix(){
        nRows = 0;
        nCols = 0;
    };
    MyMatrix(int rMax, int cMax){
        nRows = rMax;
        nCols = cMax;
        storages.resize(nRows*nCols);
    }



    /*template<class U>
    operator MyMatrix<U> (){
        MyMatirx<U> res;
        for (int i = 0; i < nRows; ++i) {
            for (int j = 0; j < nCols; ++j) {
                res (i, j) = (U) storages[i*nRows + j];
            }
        }
        return res;
    }*/

    int rowSize(){
        return nRows;
    }

    int columnSize(){
        return nCols;
    }

    void initialize(T initValue){
        int size = nRows * nCols;
        for (int storageIdx = 0; storageIdx < size; ++storageIdx) {
            storages[storageIdx] = initValue;
        }
    }

    template <class U>
    MyMatrix<U>  castFunc(U t){  //TODO to be tested
        MyMatrix<U> res(nRows, nCols);
        int idx =  0;
        for (int i = 0; i < nRows; ++i) {
            for (int j = 0; j < nCols; ++j) {
                res(i, j) = (U) storages[idx];
                idx++;
            }
        }
        return res;
    }


    void resize(int rMax, int cMax){
        nRows = rMax;
        nCols = cMax;
        storages.resize(nRows*nCols);
    }

    const T &operator()(int rwIdx, int clmIdx) const{
        return storages[rwIdx*nCols + clmIdx];
    }

    T & operator()(int rwIdx, int clmIdx){
        return storages[rwIdx*nCols + clmIdx];
    }


    MyMatrix<T>  operator*(T  operand){  //TODO to be tested
        int storageIdx = 0;
        MyMatrix<T>  result(nRows,nCols);

        for (int rwIdx = 0; rwIdx < nRows; ++rwIdx) {
            for (int clmIdx = 0; clmIdx < nCols; ++clmIdx) {
                result.storages[storageIdx] = storages[storageIdx] * operand;
                storageIdx++;
            }//end loop on columns
        }// end loop on rows
        return result;
    }

    MyMatrix<T> operator+(T  operand){   //TODO to be tested
       int storageIdx = 0;
       MyMatrix<T> result(nRows,nCols);

       for (int rwIdx = 0; rwIdx < nRows; ++rwIdx) {
           for (int clmIdx = 0; clmIdx < nCols; ++clmIdx) {
               result.storages[storageIdx] = storages[storageIdx] + operand;
               storageIdx++;
           }//end loop on columns
       }// end loop on rows
       return result;
    }

    MyMatrix<T> operator-(T  operand){   //TODO to be tested
       int storageIdx = 0;
       MyMatrix<T> result(nRows,nCols);

       for (int rwIdx = 0; rwIdx < nRows; ++rwIdx) {
           for (int clmIdx = 0; clmIdx < nCols; ++clmIdx) {
               result.storages[storageIdx] = storages[storageIdx] - operand;
               storageIdx++;
           }//end loop on columns
       }// end loop on rows
       return result;
    }

    MyMatrix<T> operator+(MyMatrix<T>  operand){  //TODO to be tested
       int storageIdx = 0;
       MyMatrix<T> result(nRows,nCols);

       for (int rwIdx = 0; rwIdx < nRows; ++rwIdx) {
           for (int clmIdx = 0; clmIdx < nCols; ++clmIdx) {
               result.storages[storageIdx] = storages[storageIdx] + operand.storages[storageIdx];
               storageIdx++;
           }//end loop on columns
       }// end loop on rows
       return result;
    }

    void subScaler(T operand){
        int mtxSize = nRows * nCols;
        for (int idx = 0; idx < mtxSize; ++idx) {
            storages[idx] = operand - storages[idx];
        }
    }

    void getSubMatrix(MyMatrix<T>& result, int rsIdx, int csIdx, int rMax, int cMax){ //TODO : to be tested
        int srcIdx, destIdx;

        srcIdx = rsIdx * nCols + csIdx;
        destIdx = 0;
        for (int rwIdx = 0; rwIdx < rMax; ++rwIdx) {
            for (int clmIdx = 0; clmIdx < cMax; ++clmIdx) {
                result.storages[destIdx] = storages[srcIdx];
                srcIdx++;
                destIdx++;
            }
            srcIdx += nCols - cMax;
        }
    }


    T convolveSubMatrix (int rsIdx, int csIdx, T * operand, int oprRMax, int oprCMax){ //TODO to be tested

        T * oprandPntr = operand;
        T result = 0;

        for (int rwCnt = 0, rIdx = rsIdx ; rwCnt < oprRMax; ++rIdx, ++rwCnt) {
            for (int clmCnt = 0, cIdx = csIdx ; clmCnt < oprCMax;++cIdx, ++clmCnt) {
                result += *oprandPntr * this->operator ()(rIdx, cIdx);
                oprandPntr++;
            }
        }

        return result;
    }

    void addSubMatrix(int rsIdx, int csIdx, const MyMatrix<T> & operand){ //TODO to be tested
        int storageIdx;
        int oprStrgIdx;
        storageIdx = rsIdx * nCols + csIdx;
        oprStrgIdx = 0;
        for (int rIdx = 0; rIdx < operand.nRows; ++rIdx) {
            for (int cIdx = 0; cIdx < operand.nCols; ++cIdx) {
                storages[storageIdx] += operand.storages[oprStrgIdx++];
                storageIdx++;
            }// end loop on the columns
            storageIdx += nCols - operand.nCols;
        }// end loop on the rows
    }


    void mulSubMatrix(int rsIdx, int csIdx, const MyMatrix<T> & operand){ //TODO to be tested
        int storageIdx;
        int oprStrgIdx;
        storageIdx = rsIdx * nCols + csIdx;
        oprStrgIdx = 0;
        for (int rIdx = 0; rIdx < operand.nRows; ++rIdx) {
            for (int cIdx = 0; cIdx < operand.nCols; ++cIdx) {
                storages[storageIdx] *= operand.storages[oprStrgIdx++];
                storageIdx++;
            }// end loop on the columns
            storageIdx += nCols - operand.nCols;
        }// end loop on the rows
    }


    void updateSubMatrix(int rsIdx, int csIdx, MyMatrix<T> & newValue){ //TODO to be tested
        int storageIdx;
        int rMax, cMax;

        rMax = newValue.nRows;
        cMax= newValue.nCols;

        storageIdx = rsIdx  * nCols + csIdx;
        for (int rwCnt = 0 ; rwCnt < rMax; ++rwCnt) {
            for (int clmCnt = 0; clmCnt < cMax; ++clmCnt) {
                storages[storageIdx++] = newValue(rwCnt, clmCnt);
            }
            storageIdx += nCols - cMax;
        }
    }

    void updateMatrix(MyMatrix<T> * newValue){
        int storageIdx = 0;
        for (int i = 0; i < nRows; ++i) {
            for (int j = 0; j < nCols; ++j) {
                storages[storageIdx] = newValue->storages[storageIdx];
                storageIdx++;
            }
        }
    }

    void updateSubMatrix(int rsIdx, int csIdx, T *newValue, int rMax, int cMax){//TODO: Is it used? //TODO to be tested
        T * nwValuPtr = newValue;
        int storageIdx;
        storageIdx = rsIdx  * nCols + csIdx;
        for (int rwCnt = 0 ; rwCnt < rMax; ++rwCnt) {
            for (int clmCnt = 0; clmCnt < cMax; ++clmCnt) {
                storages[storageIdx++] = *nwValuPtr;
                nwValuPtr++;
            }
            storageIdx += nCols - cMax;
        }
    }

    void updateSubMatrix(int rsIdx, int csIdx, T newValue, int rMax, int cMax){ //TODO to be tested
        int storageIdx;
        storageIdx = rsIdx  * nCols + csIdx;
        for (int rwCnt = 0 ; rwCnt < rMax; ++rwCnt) {
            for (int clmCnt = 0; clmCnt < cMax; ++clmCnt) {
                storages[storageIdx++] = newValue;
            }
            storageIdx += nCols - cMax;
        }
    }

    void printSubMatrix(int rsIdx, int csIdx, int rMax, int cMax){
        int storageIdx;
        storageIdx = rsIdx  * nCols + csIdx;
        for (int rwCnt = 0 ; rwCnt < rMax; ++rwCnt) {
           // cout << "{";
            for (int clmCnt = 0; clmCnt < cMax; ++clmCnt) {
                cout << storages[storageIdx++] << " ";
            }
          //  cout << "}";
            cout << endl;
            storageIdx += nCols - cMax;
        }
        //cout << endl;
    }


    ~MyMatrix(){
      //  cout << "Matrix is closed finely" << endl;
    };

};


#endif /* MATRIX_H_ */

