/* @author Aytaç Kahveci */
#ifndef KRLSTRUCT_H
#define KRLSTRUCT_H

#include <string>
#include <vector>
#include <map>
#include <sstream>
#include <algorithm>

template <class T>
class KRLStruct
{


public:
    KRLStruct(){}

    KRLStruct(std::vector<std::string> nodes)
    {
        nodes_ = nodes;
    }

    ~KRLStruct(){}

    /**
     * The nodes
     * @return the name of the variables that this struct contains
     */
    std::vector<std::string> getNodes()
    {
        return nodes_;
    }


};

#endif
