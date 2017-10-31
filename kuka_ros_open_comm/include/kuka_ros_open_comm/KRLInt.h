/* @author Aytaç Kahveci */
#ifndef KRLINT_H
#define KRLINT_H

#include <kuka_ros_open_comm/KRLVariable.h>
#include <string>

class KRLInt
{
private:
    std::string name_;
    int id_;
    long readTime_;
    KRLVariable* krl_var_;

public:
    int value_ = (int) NULL;

    KRLInt(std::string name)
    {
        krl_var_ = new KRLVariable(name);
        name_ = krl_var_->getName();
        id_ = krl_var_->getId();
    }

    ~KRLInt()
    {
        delete krl_var_;
    }

    int getValue()
    {
        return value_;
    }

    std::string getStringValue()
    {
        return std::to_string(value_);
    }

    void setValue(int value)
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
        std::string::size_type sz;
        value_ = std::stoi(strValue, &sz);
    }
};
#endif
