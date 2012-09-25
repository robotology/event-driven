#ifndef JAERPARSER_H
#define JAERPARSER_H

#include <iostream>
#include <fstream>
#include <cstring>
#include <string>

class jaerParser
{
    public:
        //Default constructor
        jaerParser();
        jaerParser(const jaerParser&);
        jaerParser(std::string&);
        //Destructor
        ~jaerParser();

        jaerParser& operator=(const jaerParser&);

        int parse(std::string&);

        int read(unsigned int&, unsigned int&, int&, unsigned int&);

        int size(){return bodySize;};
        int type(){return jaerSizeType;};
    private:
        void unmaskEvent(unsigned int, unsigned int&, unsigned int&, int&);

        int bodySize;
        int jaerSizeType;
        int currentEvent;

        int timestampMonotonyWrap;

        char* buffer;
};

#endif //JAERPARSER_H
