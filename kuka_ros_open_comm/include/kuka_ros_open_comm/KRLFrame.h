/* @author Aytaç Kahveci */
#ifndef KRLFRAME_H
#define KRLFRAME_H

#include <kuka_ros_open_comm/KRLVariable.h>
#include <string>
#include <vector>
#include <map>
#include <stdexcept>
#include <algorithm>
#include <iostream>
#include <sstream>


/**
 * Represents a Frame struct variable from the KRL language
 *
 * @author Aytaç Kahveci
 */
class KRLFrame
{
public:

    template<typename Out>
    void split(const std::string &s, char delim, Out result) {
        std::stringstream ss;
        ss.str(s);
        std::string item;
        while (std::getline(ss, item, delim)) {
            *(result++) = item;
        }
    }

    std::vector<std::string> split(const std::string &s, char delim) {
        std::vector<std::string> elems;
        split(s, delim, std::back_inserter(elems));
        return elems;
    }

    static inline std::string &ltrim(std::string &s) {
            s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
            return s;
    }
    static inline std::string &rtrim(std::string &s) {
        s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
        return s;
    }
    static inline std::string &trim(std::string &s) {
        return ltrim(rtrim(s));
    }

private:
    std::string name_;
    long readTime_;
    int id_;
    KRLVariable *krl_variable_;
    std::vector<std::string> nodes_;

public:
    std::map<std::string, double> map_;
    KRLFrame(){}

    KRLFrame(std::string name, std::vector<std::string> nodes = {"X", "Y", "Z", "A", "B", "C"})
    {
        krl_variable_ = new KRLVariable(name);
        id_ = krl_variable_->getId();
        name_ = krl_variable_->getName();
        nodes_ = nodes;
        for(std::string str : nodes_)
        {
            map_.insert(std::pair<std::string,double>(str, 0.0));
        }
    }

    ~KRLFrame()
    {}

    std::vector<std::string> getNodes()
    {
        return nodes_;
    }

    double getX() {
        return map_["X"];
    }

    double getY() {
        return map_["Y"];
    }

    double getZ() {
        return map_["Z"];
    }

    double getA() {
        return map_["A"];
    }

    double getB() {
        return map_["B"];
    }

    double getC() {
        return map_["C"];
    }

    KRLFrame setX(double d) {
        map_[getNodes()[0]] = d;
        return *this;
    }

    KRLFrame setY(double d) {
        map_[getNodes()[1]] = d;
        return *this;
    }

    KRLFrame setZ(double d) {
        map_[getNodes()[2]] = d;
        return *this;
    }

    KRLFrame setA(double d) {
        map_[getNodes()[3]] = d;
        return *this;
    }

    KRLFrame setB(double d) {
        map_[getNodes()[4]] = d;
        return *this;
    }

    KRLFrame setC(double d) {
        map_[getNodes()[5]] = d;
        return *this;
    }


    void setXToZ(std::vector<double> values)
    {
        if(values.size() != 3)
        {
            throw std::invalid_argument("The number of values should be exatly 3!");
        }
        setX(values[0]);
        setY(values[1]);
        setZ(values[2]);
    }

    void setAToC(std::vector<double> values)
    {
        if(values.size() != 3)
        {
            throw std::invalid_argument("The number of values should be exactly 3!");
        }
        setA(values[0]);
        setB(values[1]);
        setC(values[2]);
    }

    std::vector<double> asArray() {

        std::vector<double> arr;
        arr.resize(getNodes().size());
        for (int i = 0; i < arr.size(); i++) {
            arr[i] = map_[getNodes()[i]];
        }
        return arr;
    }

    std::vector<double> asArrayXToZ() {
        std::vector<double> arr = {map_["X"], map_["Y"], map_["Z"]};
        return arr;
    }

    std::vector<double> asArrayAToC() {
        std::vector<double> arr = {map_["A"], map_["B"], map_["C"]};
        return arr;
    }

    void setValue(std::string str, std::string obj) {
        std::string::size_type sz;
        double db =  std::stod(obj, &sz);
        map_[str] = db;
    }

    std::map<std::string, double> getValue() {
        return map_;
    }

    std::string getStringValue()
    {
        std::string sb;
        sb.append("{");
        unsigned int i = 0;
        for(std::string str : nodes_)
        {
            if(map_.count(str) > 0)
            {
                double get = map_[str];
                map_.erase(map_.find(str));
                sb.append(str).append(" ").append(std::to_string(get));
                if(!map_.empty() && (i != map_.size()))
                {
                    sb.append(", ");
                }
            }
        }
        sb.append("}");
        return sb;
    }

    void setValueFromString(std::string strValue)
    {
        std::string substring;
        if(strValue.find(":") != std::string::npos)
        {
            std::vector<std::string> split_ = split(strValue,':');
            std::string trim_ = trim(split_[1]);
            substring = trim_.substr(0, trim_.find('}'));
        }
        else
        {
            std::string trim_ = trim(strValue);
            substring = trim_.substr(1, trim_.size() - 1);
        }

        std::vector<std::string> split1 = split(substring,',');

        for(std::string n : split1)
        {
            trim(n);
            std::vector<std::string> split2 = split(n, ' ');

            setValue(split2[0], split2[1]);
        }
    }

    void update(int id, std::string strValue, long readTime)
    {
        if( id_ != id)
        {
           throw std::runtime_error("The returned id does not match the variable id! Should not happen...");
        }
       readTime_ = readTime;
       setValueFromString(strValue);
    }

    std::vector<unsigned char> getReadCommand()
    {
        return krl_variable_->getReadCommand();
    }

    std::vector<unsigned char> getWriteCommand()
    {
        return krl_variable_->getWriteCommand(getStringValue());
    }

};
#endif
