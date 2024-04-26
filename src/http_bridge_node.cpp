/**
* @file     http_bridge_node.cpp
* @brief    http_bridge_nodeパッケージのmain関数
* @author   S.Kumada
* @date     2023/12/7
* @note     ノードの初期化、マネージャの初期化を行う
*/

#include "http_bridge/http_bridge.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "http_bridge");
    ros::NodeHandle node;

    HttpBridge bridge(node);
    bridge.bridgeLoop();

    return (0);
}