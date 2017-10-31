/* @author Aytaç Kahveci */
#ifndef KRLENUM_H
#define KRLENUM_H

#include <kuka_ros_open_comm/KRLVariable.h>
#include <string>

class KRLEnum
{
private:
    std::string name_;
    int id_;
    long readTime_;
    KRLVariable* krl_var_;

public:
    std::string value_ = (std::string) NULL;

    KRLEnum(std::string name)
    {
        krl_var_ = new KRLVariable(name);
    }

    ~KRLEnum()
    {
        delete krl_var_;
    }

    std::string getValue()
    {
        return value_;
    }

    std::string getStringValue()
    {
        return value_;
    }

    void setValue(std::string value)
    {
        value_ = value;
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

private:
    void setValueFromString(std::string strValue)
    {
        value_ = strValue;
    }
};
#endif
