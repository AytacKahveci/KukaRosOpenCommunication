/* @author Aytaç Kahveci */
#ifndef KRLVARIABLE_H
#define KRLVARIABLE_H

#include <string>
#include <atomic>
#include <vector>
#include <stdexcept>

class KRLVariable
{
public:
    int id_;
    std::string name_;
    long readTime_ = -1;

public:
    KRLVariable(){}

    KRLVariable(std::string name)
    {
        static std::atomic<std::uint32_t> atomicInt { 0 };
        name_ = name;
        id_ = atomicInt.fetch_add(1, std::memory_order_relaxed);
    }

    ~KRLVariable(){}

    int getId()
    {
        return id_;
    }


    std::string getName()
    {
        return name_;
    }


    long getReadTimeNano()
    {
        return readTime_;
    }


    long getReadTimeMillis()
    {
        return readTime_ / 1000000;
    }

    double getReadTimeSec()
    {
        return ((double) readTime_ / 1000000000);
    }


     std::vector<unsigned char> getReadCommand()
    {
         std::vector<unsigned char> cmd(name_.c_str(), name_.c_str() + name_.size());
         std::vector<unsigned char> header;
         std::vector<unsigned char> block;

         int varnamelen = cmd.size();
         unsigned char hbyte, lbyte;
         hbyte = (varnamelen & 0xff00) >> 8;
         lbyte = (varnamelen & 0x00ff);

         block.push_back(0);
         block.push_back(hbyte);
         block.push_back(lbyte);
         block.insert(block.end(),cmd.begin(),cmd.end());

         int blocklength = block.size();

         hbyte = ((blocklength & 0xff00) >> 8);
         lbyte = (blocklength & 0x00ff);

         unsigned char hbytemsg = ((id_ & 0xff00) >> 8);
         unsigned char lbytemsg = (id_ & 0x00ff);

         header.push_back(hbytemsg);
         header.push_back(lbytemsg);
         header.push_back(hbyte);
         header.push_back(lbyte);
         block.insert(block.begin(), header.begin(), header.end());

         return block;
    }

     /*
     * The write command. This is what's actually beeing sent to the robot.
     * It's a implementation of the OpenShowVar c++ source
     * @return the write command
     */
    std::vector<unsigned char> getWriteCommand(std::string val)
    {
         std::vector<unsigned char> cmd(name_.c_str(), name_.c_str() + name_.size());
         std::vector<unsigned char> value(val.c_str(), val.c_str() + val.size());
         std::vector<unsigned char> header;
         std::vector<unsigned char> block;

         int varnamelen = cmd.size();
         unsigned char hbyte, lbyte;
         hbyte = (varnamelen & 0xff00) >> 8;
         lbyte = (varnamelen & 0x00ff);

         block.push_back((unsigned char) 1);
         block.push_back(hbyte);
         block.push_back(lbyte);
         block.insert(block.end(),cmd.begin(),cmd.end());

         int vallen = value.size();
         hbyte = (vallen & 0xff00) >> 8;
         lbyte = (vallen & 0x00ff);

         block.push_back(hbyte);
         block.push_back(lbyte);
         block.insert(block.end(),value.begin(),value.end());

         int blocklength = block.size();

         hbyte = ((blocklength & 0xff00) >> 8);
         lbyte = (blocklength & 0x00ff);

         unsigned char hbytemsg = (id_ & 0xff00) >> 8;
         unsigned char lbytemsg = (id_ & 0x00ff);

         header.push_back(hbytemsg);
         header.push_back(lbytemsg);
         header.push_back(hbyte);
         header.push_back(lbyte);

         block.insert(block.begin(), header.begin(), header.end());
         return block;
    }

};

#endif
