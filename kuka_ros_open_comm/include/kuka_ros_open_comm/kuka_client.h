/* @author Aytaç Kahveci */
#ifndef KUKA_ClIENT_H
#define KUKA_ClIENT_H

#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <netdb.h>
#include <iostream>
#include <chrono>
#include <stdio.h>
#include <string>
#include <string.h>
#include <ros/ros.h>
#include <netinet/tcp.h>
namespace kuka_hw_interface
{
    class kukaClient
    {
    public:
        kukaClient(){}

        kukaClient(char* host, int port)
        {
            sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
            if(sockfd_ < 0)
            {
                ROS_ERROR("ERROR creating a socket");
            }
            server = gethostbyname(host);
            if(server == NULL)
            {
                ROS_ERROR("ERROR No such host");
            }
            memset((char*)&serverAddr_, 0, sizeof(serverAddr_));
            memcpy((char*)&serverAddr_.sin_addr.s_addr, (char*)server->h_addr, server->h_length);
            serverAddr_.sin_port = htons(port);
            serverAddr_.sin_family = AF_INET;

            if((connect(sockfd_, (struct sockaddr*)&serverAddr_, sizeof(serverAddr_))) < 0)
            {
                ROS_ERROR("ERROR connecting to the server");
            }

        }

        ~kukaClient()
        {
            close(sockfd_);
        }

        template <class T>
        void readVariable(T *var)
        {
            std::vector<unsigned char> buffer = var->getReadCommand();
            std::vector<unsigned char>::iterator next = buffer.begin();
            while (next != buffer.end())
            {
                int n = send(sockfd_, &(*next), std::distance(next, buffer.end()), 0);
                if (n == -1)
                {
                    ROS_ERROR_STREAM("ERROR in readVariable function");
                    break;  // ERROR
                }
                next += n;
            }
            char head[7];
            int rec=0;
            while(rec < sizeof(head))
            {
                n = recv(sockfd_, &head, sizeof(head)-rec, 0);
                if(n < 0)
                {
                    ROS_ERROR_STREAM("ERROR reading header from server");
                }
                rec += n;
            }
            char block[getInt(head,2)-3];
            rec=0;
            while(rec < sizeof(block))
            {
                n = recv(sockfd_, &block, sizeof(block)-rec, 0);
                if(n < 0)
                {
                    ROS_ERROR_STREAM("ERROR reading block from server");
                }
                rec += n;
            }
            std::vector<unsigned char> data;
            for(char c : head)
            {
                data.push_back(c);
            }
            for(char c : block)
            {
                data.push_back(c);
            }
            int id = getInt(head, 0);
            std::string strValue = "";
            for (int i=0; i<data.size(); i++)
            {
                strValue += data[7+i];
            }
            ROS_INFO_STREAM("Received value: "<< strValue);
            ROS_INFO_STREAM("Received id: "<< id);
            var->update(id, strValue, 1);
        }

        template <class T>
        void writeVariable(T *var)
        {
            std::vector<unsigned char> buffer = var->getWriteCommand();
            std::vector<unsigned char>::iterator it = buffer.begin();
            while(it != buffer.end())
            {
                if((n = send(sockfd_, &(*it), std::distance(it,buffer.end()),0)) < 0)
                {
                    ROS_ERROR_STREAM("ERROR in writeVariable function");
                    break;
                }
                it += n;
            }
            char head[7];
            int rec=0;
            while(rec < sizeof(head))
            {
                n = recv(sockfd_, &head, sizeof(head)-rec, 0);
                if(n < 0)
                {
                    ROS_ERROR_STREAM("ERROR reading header from server");
                }
                rec += n;
            }
            char block[getInt(head,2)-3];
            rec=0;
            while(rec < sizeof(block))
            {
                n = recv(sockfd_, &block, sizeof(block)-rec, 0);
                if(n < 0)
                {
                    ROS_ERROR_STREAM("ERROR reading block from server");
                }
                rec += n;
            }
            std::vector<unsigned char> data;
            for(char c : head)
            {
                data.push_back(c);
            }
            for(char c : block)
            {
                data.push_back(c);
            }
            int id = getInt(head, 0);
            std::string strValue = "";
            for (int i=0; i<data.size(); i++)
            {
                strValue += data[7+i];
            }
            ROS_INFO_STREAM("Received value in writeVariable function: "<< strValue);
            ROS_INFO_STREAM("Received id in writeVariable function: "<< id);
        }

        int getInt(char* bytes, int off)
        {
            int a = (((bytes[off] << 8) & 0xFF00) | (bytes[off + 1] & 0xFF));
            return a;
        }
    private:
        int sockfd_, n;
        hostent *server;
        sockaddr_in serverAddr_, clientAddr_;
    };
}
#endif
