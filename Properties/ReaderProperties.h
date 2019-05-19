/*
 * ReaderProperties.h
 *
 *  Created on: Apr 22, 2019
 *      Author: tfm3
 */

#ifndef READERPROPERTIES_H_
#define READERPROPERTIES_H_

#include <iostream>
#include <fstream>
#include <map>
#include <string>
using namespace std;
class ReaderProperties {


public:
	ReaderProperties();
    ReaderProperties(string fileProperties);
	virtual ~ReaderProperties();
    void readFile(string fileProperties);
    typedef map<const string,string> CfgMap;
    string getPropertyValue(string propName);
    CfgMap config;
    string interpolationStep;
    string maxLineInputFile;
    string maxLineInterpolation;
    string offsetLimit;
    string getInterpolationStep();
    string getMaxLineInputFile();
    string getMaxLineInterpolation();
    string getOffsetLimit();
};

#endif /* READERPROPERTIES_H_ */
