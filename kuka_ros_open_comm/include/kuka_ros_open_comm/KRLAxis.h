/* @author Aytaç Kahveci */
#ifndef KRLAXIS_H
#define KRLAXIS_H

#include <kuka_ros_open_comm/KRLVariable.h>
#include <vector>
#include <string>
#include <map>
#include <stdexcept>
#include <sstream>
#include <algorithm>

/**
 * Represents a Axis Struct variable from the KRL language
 *
 * @author Aytac Kahveci
 */
 class KRLAxis
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
     KRLVariable *krl_var_;
     std::vector<std::string> nodes_;

public:
    std::map<std::string, double> map_;

    KRLAxis(){}

    KRLAxis(std::string name, std::vector<std::string> nodes = {"A1", "A2", "A3", "A4", "A5", "A6"})
    {
        krl_var_ = new KRLVariable(name);
        id_ = krl_var_->getId();
        name_ = krl_var_->getName();
        nodes_ = nodes;
        for(std::string str : nodes)
        {
            map_.insert(std::pair<std::string,double>(str, 0.0));
        }
    }

    ~KRLAxis()
    {
        delete krl_var_;
    }

    std::vector<std::string> getNodes()
    {
        return nodes_;
    }

    void setA1ToA6(std::vector<double> values) {
        if (values.size() != 6) {
            throw std::invalid_argument("The number of values should be exatly 6!");
        }
        setA1(values[0]);
        setA2(values[1]);
        setA3(values[2]);
        setA4(values[3]);
        setA5(values[4]);
        setA6(values[5]);
    }


    void setA1(double d)
    {
        map_.at(getNodes()[0]) = d;
    }


    void setA2(double d)
    {
        map_.at(getNodes()[1]) = d;
    }

    void setA3(double d)
    {
        map_.at(getNodes()[2]) = d;
    }

    void setA4(double d)
    {
        map_.at(getNodes()[3]) = d;
    }

    void setA5(double d)
    {
        map_.at(getNodes()[4]) = d;
    }


    void setA6(double d)
    {
        map_.at(getNodes()[5]) = d;
    }

    /**
     * Get a double array representation of this object
     *
     * @return a new double array with the values contained in this struct
     */
     std::vector<double> asArray()
     {
        std::vector<double> arr;
        arr.resize(this->getNodes().size());
        for (int i = 0; i < arr.size(); i++)
        {
            arr[i] = map_[this->getNodes()[i]];
        }
        return arr;
    }

    std::vector<double> asArrayA1ToA6()
    {
        std::vector<double> arr = {map_["A1"], map_["A2"], map_["A3"], map_["A4"], map_["A5"], map_["A6"]};
        return arr;
    }

    void setValue(std::string str, std::string obj)
    {
        std::string::size_type sz;
        double db = std::stod(obj, &sz);
        map_[str] = db;
    }

    std::map<std::string, double> getValue()
    {
        return map_;
    }

    std::string getStringValue()
    {
        std::string sb="";
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
       return krl_var_->getReadCommand();
    }


    std::vector<unsigned char> getWriteCommand()
    {
        return krl_var_->getWriteCommand(getStringValue());
    }
};
#endif
