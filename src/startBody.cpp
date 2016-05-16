/*************************************************************************************
Copyright (c) 2013, Carnegie Mellon University
All rights reserved.
Authors: Anurag Jakhotia<ajakhoti@andrew.cmu.edu>, Prasanna Velagapudi<pkv@cs.cmu.edu>

Redistribution and use in source and binary forms, with or without modification, are 
permitted provided that the following conditions are met:

 -    Redistributions of source code must retain the above copyright notice, this list 
     of conditions and the following disclaimer.
 -    Redistributions in binary form must reproduce the above copyright notice, this 
     list of conditions and the following disclaimer in the documentation and/or other 
     materials provided with the     distribution.
 -    Neither the name of Carnegie Mellon University nor the names of its contributors 
     may be used to endorse or promote products derived from this software without 
     specific prior written     permission.
 
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY 
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED 
TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR 
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY 
WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************/
#include "k2_client/k2_client.h"
#include "k2_client/BodyArray.h"
#include <iconv.h>
#include <cstdio>

std::string topicName = "bodyArray";
size_t streamSize = 60000;
size_t readSkipSize = 60000;
size_t stringSize = 30000;

int main(int argC,char **argV)
{
    ros::init(argC,argV,"startBody");
    ros::NodeHandle n;
    std::string serverAddress;
    n.getParam("/serverNameOrIP",serverAddress);
    Socket mySocket(serverAddress.c_str(),"9003",streamSize);
    ros::Publisher bodyPub = n.advertise<k2_client::BodyArray>(topicName,1);
    while(ros::ok())
    {
        mySocket.readData();        
        std::string jsonString(mySocket.mBuffer);
        Json::Value jsonObject;
        Json::Reader jsonReader;
        bool parsingSuccessful = jsonReader.parse(jsonString,jsonObject,false);
        if(!parsingSuccessful)
        {
            ROS_ERROR("Failure to parse");
            continue;
        }
        k2_client::BodyArray bodyArray;
        try
        {
            for(int i=0;i<6;i++)
            {
                k2_client::Body body;
                body.header.stamp = ros::Time::now();
                body.header.frame_id =  ros::this_node::getNamespace().substr(1,std::string::npos) + "/depthFrame";
                body.fromX = jsonObject[i]["FromX"].asInt();
                body.fromY = jsonObject[i]["FromY"].asInt();
                body.toX = jsonObject[i]["toX"].asInt();
                body.toY = jsonObject[i]["toY"].asInt();
                body.leanTrackingState = jsonObject[i]["Body"]["LeanTrackingState"].asInt();
                body.lean.leanX = jsonObject[i]["Body"]["Lean"]["X"].asDouble();
                body.lean.leanY = jsonObject[i]["Body"]["Lean"]["Y"].asDouble();
                body.isTracked = jsonObject[i]["Body"]["IsTracked"].asBool();
                if (!body.isTracked) {
                    continue;
                }
                body.trackingId = jsonObject[i]["Body"]["TrackingId"].asUInt64();
                body.clippedEdges = jsonObject[i]["Body"]["ClippedEdges"].asInt();
                body.engaged = jsonObject[i]["Body"]["Engaged"].asBool();
                body.handRightConfidence = jsonObject[i]["Body"]["HandRightConfidence"].asInt();
                body.handRightState = jsonObject[i]["Body"]["HandRightState"].asInt();
                body.handLeftConfidence = jsonObject[i]["Body"]["HandLeftConfidence"].asInt();
                body.handLeftState = jsonObject[i]["Body"]["HandLeftState"].asInt();
                body.appearance.wearingGlasses = jsonObject[i]["Body"]["Appearance"]["WearingGlasses"].asBool();
                body.activities.eyeLeftClosed = jsonObject[i]["Body"]["Activities"]["EyeLeftClosed"].asBool();
                body.activities.eyeRightClosed = jsonObject[i]["Body"]["Activities"]["EyeRightClosed"].asBool();
                body.activities.mouthOpen = jsonObject[i]["Body"]["Activities"]["MouthOpen"].asBool();
                body.activities.mouthMoved = jsonObject[i]["Body"]["Activities"]["MouthMoved"].asBool();
                body.activities.lookingAway = jsonObject[i]["Body"]["Activities"]["LookingAway"].asBool();
                body.expressions.neutral = jsonObject[i]["Body"]["Expressions"]["Neutral"].asBool();
                body.expressions.neutral = jsonObject[i]["Body"]["Expressions"]["Happy"].asBool();
                for(int j=0;j<25;j++)
                {
                    k2_client::JointOrientationAndType JOAT;
                    k2_client::JointPositionAndState JPAS;
                    std::string fieldName;
                    switch (j)
                    {
                        case 0: fieldName = "SpineBase";break;
                        case 1: fieldName = "SpineMid";break;
                        case 2: fieldName = "Neck";break;
                        case 3: fieldName = "Head";break;
                        case 4: fieldName = "ShoulderLeft";break;
                        case 5: fieldName = "ElbowLeft";break;
                        case 6: fieldName = "WristLeft";break;
                        case 7: fieldName = "HandLeft";break;
                        case 8: fieldName = "ShoulderRight";break;
                        case 9: fieldName = "ElbowRight";break;
                        case 10: fieldName = "WristRight";break;
                        case 11: fieldName = "HandRight";break;
                        case 12: fieldName = "HipLeft";break;
                        case 13: fieldName = "KneeLeft";break;
                        case 14: fieldName = "AnkleLeft";break;
                        case 15: fieldName = "FootLeft";break;
                        case 16: fieldName = "HipRight";break;
                        case 17: fieldName = "KneeRight";break;
                        case 18: fieldName = "AnkleRight";break;
                        case 19: fieldName = "FootRight";break;
                        case 20: fieldName = "SpineShoulder";break;
                        case 21: fieldName = "HandTipLeft";break;
                        case 22: fieldName = "ThumbLeft";break;
                        case 23: fieldName = "HandTipRight";break;
                        case 24: fieldName = "ThumbRight";break;
                    }

                    JOAT.orientation.x = jsonObject[i]["JointOrientations"][fieldName]["Orientation"]["X"].asDouble();
                    JOAT.orientation.y = jsonObject[i]["JointOrientations"][fieldName]["Orientation"]["Y"].asDouble();
                    JOAT.orientation.z = jsonObject[i]["JointOrientations"][fieldName]["Orientation"]["Z"].asDouble();
                    JOAT.orientation.w = jsonObject[i]["JointOrientations"][fieldName]["Orientation"]["W"].asDouble();
                    JOAT.jointType = jsonObject[i]["JointOrientations"][fieldName]["JointType"].asInt();

                    JPAS.trackingState = jsonObject[i]["Joints"][fieldName]["TrackingState"].asBool();
                    JPAS.position.x = jsonObject[i]["Joints"][fieldName]["Position"]["X"].asDouble();
                    JPAS.position.y = jsonObject[i]["Joints"][fieldName]["Position"]["Y"].asDouble();
                    JPAS.position.z = jsonObject[i]["Joints"][fieldName]["Position"]["Z"].asDouble();
                    JPAS.jointType = jsonObject[i]["Joints"][fieldName]["JointType"].asInt();
                    
                    body.jointOrientations.push_back(JOAT);
                    body.jointPositions.push_back(JPAS);
                }
                bodyArray.bodies.push_back(body);
            }
        }
        catch (...)
        {
            ROS_ERROR("startBody - An exception occured");
            continue;
        }
        if (bodyArray.bodies.size() > 0) 
            bodyPub.publish(bodyArray);
    }
    return 0;
}
