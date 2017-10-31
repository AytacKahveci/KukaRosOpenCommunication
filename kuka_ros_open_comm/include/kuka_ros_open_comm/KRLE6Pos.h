/* @author Aytaç Kahveci */
#ifndef KRLE6POS_H
#define KRLE6POS_H

#include <kuka_ros_open_comm/KRLFrame.h>
#include <kuka_ros_open_comm/KRLPos.h>

#include <vector>
#include <string>
#include <stdexcept>

class KRLE6Pos : public KRLPos
{

public:
    KRLE6Pos(){}

    KRLE6Pos(std::string name, std::vector<std::string> nodes = {"X", "Y", "Z", "A", "B", "C", "E1", "E2", "E3", "E4", "E5", "E6", "S", "T"})
        :KRLPos(name, nodes)
    {

    }

    ~KRLE6Pos(){}

    void setE1ToE6(std::vector<double> values)
    {
        if(values.size() != 6)
        {
            throw std::invalid_argument("The number of values should be exactly 6!");
        }
        setE1(values[0]);
        setE2(values[1]);
        setE3(values[2]);
        setE4(values[3]);
        setE5(values[4]);
        setE6(values[5]);
    }

    double getE1()
    {
        return map_["E1"];
    }


    double getE2()
    {
        return map_["E2"];
    }

    double getE3()
    {
        return map_["E3"];
    }

    double getE4()
    {
        return map_["E4"];
    }

    double getE5()
    {
        return map_["E5"];
    }

    double getE6()
    {
        return map_["E6"];
    }

    KRLE6Pos setE1(double d)
    {
        map_.at(getNodes()[6]) = d;
        return *this;
    }

    KRLE6Pos setE2(double d)
    {
        map_.at(getNodes()[7]) = d;
        return *this;
    }

    KRLE6Pos setE3(double d)
    {
        map_.at(getNodes()[8]) = d;
        return *this;
    }

    KRLE6Pos setE4(double d)
    {
        map_.at(getNodes()[9]) = d;
        return *this;
    }

    KRLE6Pos setE5(double d)
    {
        map_.at(getNodes()[10]) = d;
        return *this;
    }

    KRLE6Pos setE6(double d)
    {
        map_.at(getNodes()[11]) = d;
        return *this;
    }

    std::vector<double> asArrayE1ToE6()
    {
        std::vector<double> arr_ = {map_["E1"], map_["E2"],map_["E3"],map_["E4"],map_["E5"],map_["E6"]};
        return arr_;
    }

};

#endif
