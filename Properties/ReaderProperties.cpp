/*
 * ReaderProperties.cpp
 *
 *  Created on: Apr 22, 2019
 *      Author: tfm3
 */
#include <iostream>
#include <fstream>
#include <map>
#include <string>
#include "ReaderProperties.h"

using namespace std;

typedef map<const string,string> CfgMap;

ReaderProperties::ReaderProperties() {
	// TODO Auto-generated constructor stub

}

ReaderProperties::ReaderProperties(string fileProperties) {
    // TODO Auto-generated constructor stub
    this->readFile( fileProperties);
    string property, propertyName;

    for(CfgMap::iterator i=config.begin();i!=config.end();i++) {
        string property=i->first;
        propertyName=property.substr(0,propertyName.find('='));
        //cout << "Item \"" << propertyName.substr(0,propertyName.find('=') )<< "\" has value \"" << i->second << '\"' << endl ;
        cout << "Item \"" << property<< "\" has value \"" << i->second << '\"' << endl ;
      }
}

ReaderProperties::~ReaderProperties() {
	// TODO Auto-generated destructor stub
}

string ReaderProperties::getPropertyValue(string propName){
    std::cout<<"config[propName]="<<config[propName]<<"\n";
    string property, propertyName;

    for(CfgMap::iterator i=config.begin();i!=config.end();i++) {
        string property=i->first;
        propertyName=property.substr(0,propertyName.find('='));
        //cout << "Item \"" << propertyName.substr(0,propertyName.find('=') )<< "\" has value \"" << i->second << '\"' << endl ;
        if (propertyName.compare(propName)==0) {
            cout << "Item \"" << propName<< "\" has value \"" << i->second << '\"' << endl ;
        }

      }
    string myString = config[propName];
    return myString;
}
void ReaderProperties::readFile(string fileProperties){

    string s;
    char * inputFileName;
    inputFileName="/tmp/myProperties.txt";
    ifstream infile(inputFileName);
    string line;


    while(infile >> line){
          config[line.substr(0,s.find('='))]=line.substr(line.find('=')+1);

    }

    if(config["offsetLimit"].empty())
        cout << "Setting foo missing\n";
     else
        cout << "Foo is " << config["foo"] << endl;

    cout << "All settings:\n";
    string propertyName;
    for(CfgMap::iterator i=config.begin();i!=config.end();i++) {
        propertyName=i->first;
        cout << "Item \"" << propertyName.substr(0,propertyName.find('=') )<< "\" has value \"" << i->second << '\"' << endl ;
      }

   // CfgMap config;
//    string s;
//    string inputFileName;
//    inputFileName=fileProperties;
//    ifstream infile(inputFileName);
//    string line;
//    //infile >> std::setprecision(6) >> fixed;

//    while(infile >> line){
//        config[line.substr(0,s.find('='))]=line.substr(line.find('=')+1);

//    }
//    if(config["offsetLimit"].empty()) cout << "Setting offsetLimit missing\n";
//    else cout << "offsetLimit " << config["offsetLimit"] << endl;

//    if (config["interpolationStep"].empty())
//        cout << "ERROR  propertie interpolationStep is missing"<<"\n";
//    else
//        interpolationStep=config["interpolationStep"];

//    if (config["maxLineInputFile"].empty())
//        cout << "ERROR propertie maxLineInputFile is missing\n";
//    else
//        maxLineInputFile=config["maxLineInputFile"];

//    if (config["maxLineInterpolation"].empty())
//        cout << "ERROR propertie maxLineInterpolation is missing\n";
//    else
//        maxLineInterpolation=config["maxLineInterpolation"];

//    if (config["offsetLimit"].empty())
//        cout << "ERROR propertie offsetLimit is missing";
//    else
//        offsetLimit=config["offsetLimit"];
//    //stod(myReaderProperties.getPropertyValue("offsetLimit"));


//    cout << "All settings:\n";
//    string propertyName;

//    for(CfgMap::iterator i=config.begin();i!=config.end();i++) {
//        if (!i->second.empty()){
//            propertyName=i->first;
//            cout << "Item \"" << propertyName.substr(0,propertyName.find('=') )<< "\" has value \"" << i->second << '\"' << endl ;
//        }

//    }
//infile.close();

}
string ReaderProperties::getInterpolationStep(){
 return  interpolationStep;
}

string ReaderProperties::getMaxLineInputFile(){
 return maxLineInputFile;

}
string ReaderProperties::getMaxLineInterpolation(){
 return maxLineInterpolation;

}

string ReaderProperties::getOffsetLimit(){
 return offsetLimit;

}
//int main()

//{
//  CfgMap config;
//  string s;
//  char * inputFileName;
//  inputFileName="/tmp/myProperties.txt";
//  ifstream infile(inputFileName);
//  string line;
//  //infile >> std::setprecision(6) >> fixed;

//  while(infile >> line){
//	  config[line.substr(0,s.find('='))]=line.substr(line.find('=')+1);

//  }

//  if(config["offsetLimit"].empty()) cout << "Setting foo missing\n";
//  else cout << "Foo is " << config["foo"] << endl;

//  cout << "All settings:\n";
//  string propertyName;
//  for(CfgMap::iterator i=config.begin();i!=config.end();i++) {
//	propertyName=i->first;
//    cout << "Item \"" << propertyName.substr(0,propertyName.find('=') )<< "\" has value \"" << i->second << '\"' << endl ;
//  }
//}

