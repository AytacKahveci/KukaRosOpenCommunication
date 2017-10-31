/* @author Aytaç Kahveci */
#ifndef KRLREAL_H
#define KRLREAL_H

#include <kuka_ros_open_comm/KRLVariable.h>
#include <string>

class KRLReal
{
private:
    std::string name_;
    int id_;
    long readTime_;
    KRLVariable* krl_var_;

public:
    double value_ = (double) NULL;

    KRLReal(){}

    KRLReal(std::string name)
    {
        krl_var_ = new KRLVariable(name);
        name_ = krl_var_->getName();
        id_ = krl_var_->getId();
    }

    ~KRLReal()
    {
        delete krl_var_;
    }

    double getValue()
    {
        return value_;
    }

    std::string getStringValue()
    {
        return std::to_string(value_);
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

    void setValue(double value)
    {
        value_ = value;
    }

private:
    void setValueFromString(std::string strValue)
    {
        std::string::size_type sz;
        value_ = std::stod(strValue, &sz);
    }
};
#endif
