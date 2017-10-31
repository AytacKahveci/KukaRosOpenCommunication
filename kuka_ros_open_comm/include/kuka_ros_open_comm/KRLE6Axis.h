/* @author Aytaç Kahveci */
#ifndef KRLE6AXIS_H
#define KRLE6AXIS_H

#include <kuka_ros_open_comm/KRLAxis.h>
#include <vector>
#include <string>
#include <stdexcept>

/**
 * Represents a E6Axis struct variable from the KRL language
 *
 * @author Aytac Kahveci
 */

class KRLE6Axis : public KRLAxis
{

public:
    KRLE6Axis(){}

    KRLE6Axis(std::string name ,std::vector<std::string> nodes = {"A1", "A2", "A3", "A4", "A5", "A6", "E1", "E2", "E3", "E4", "E5", "E6"})
        :KRLAxis(name,nodes)
    {

    }

    ~KRLE6Axis(){}

    void setE1ToE6(std::vector<double> values)
    {
         if (values.size() != 6)
         {
             throw std::invalid_argument("The number of values should be exatly 6!");
         }
         setE1(values[0]);
         setE2(values[1]);
         setE3(values[2]);
         setE4(values[3]);
         setE5(values[4]);
         setE6(values[5]);
    }

    void setE1(double d)
    {
        map_.at(getNodes()[6]) = d;
    }

    void setE2(double d)
    {
        map_.at(getNodes()[7]) = d;
    }

    void setE3(double d)
    {
        map_.at(getNodes()[8]) = d;
    }


    void setE4(double d)
    {
        map_.at(getNodes()[9]) = d;
    }

    void setE5(double d)
    {
        map_.at(getNodes()[10]) = d;
    }


    void setE6(double d)
    {
        map_.at(getNodes()[11]) = d;
    }

    std::vector<double> asArrayE1ToE6()
    {
        std::vector<double> arr {map_["E1"], map_["E2"],map_["E3"],map_["E4"],map_["E5"],map_["E6"]};
        return arr;
    }
};

#endif
