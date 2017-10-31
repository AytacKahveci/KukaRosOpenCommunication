/* @author Aytaç Kahveci */
#ifndef KRLPOS_H
#define KRLPOS_H

#include <kuka_ros_open_comm/KRLFrame.h>
#include <vector>
#include <string>

/**
 * Represents a Real variable from the KRL language
 */

class KRLPos : public KRLFrame
{

public:
    KRLPos(){}

    KRLPos(std::string name, std::vector<std::string> nodes = {"X", "Y", "Z", "A", "B", "C", "S", "T"})
    : KRLFrame(name, nodes)
    {

    }

    ~KRLPos(){}


    double getS()
    {
        return map_["S"];
    }


    double getT()
    {
        return map_["T"];
    }

    void setS(double d)
    {
        map_.at("S") = d;
    }

    void setT(double d)
    {
        map_.at("T") = d;
    }

    std::vector<double> asArrayXToC()
    {
        std::vector<double> arr = {map_["X"], map_["Y"], map_["Z"],map_["A"], map_["B"], map_["C"]};
        return arr;
    }
};
#endif
