/**
* @file     http_bridge.cpp
* @brief    HTTP通信の実行クラスHttpBridgeのソースファイル
* @author   S.Kumada
* @date     2023/10/6
* @note     HTTP通信により、サーバーからのデータ取得や配信を行うクラスの実装
*/

#include "http_bridge/http_bridge.hpp"

template <typename T>
bool getParam(const ros::NodeHandle &node, const std::string get_key, T &param, T def_param)
{
    bool is_sccess = false;

    if(node.getParam(get_key, param))
    { // 読み込み成功
        ROS_INFO_STREAM("Successful reading of parameters with key: \"" << get_key.c_str() << "\" value: "<< param);
        is_sccess = true;
    }
    else
    { // 読み込み失敗
        ROS_WARN_STREAM("Failure to read parameters with key: \"" << get_key.c_str() << "\" value: "<< def_param);
        param = def_param;
    }

    return (is_sccess);
}


HttpBridge::HttpBridge(ros::NodeHandle &node)
{

    ros::NodeHandle param_node("~");

    // ロボット名の読み込み
    getParam(param_node, "robot_id", robot_id_, std::string(DEF_ROBOT_ID));

    // 接続先のURI
    getParam(param_node, "set_url", set_url_, std::string(DEF_SET_URL));

    // ロボットの機体情報をRDRに登録するAPI
    std::string topic_name_req_put_robot_info;
    std::string topic_name_res_put_robot_info;
    getParam(param_node, "robot_info/http_api",       api_pub_robot_info_,           std::string(DEF_PUT_ROBOT_INFO_API));
    getParam(param_node, "robot_info/req_topic_name", topic_name_req_put_robot_info, std::string(DEF_PUT_ROBOT_REQ_TOPIC));
    getParam(param_node, "robot_info/res_topic_name", topic_name_res_put_robot_info, std::string(DEF_PUT_ROBOT_RES_TOPIC));
    sub_put_robot_info_     = node.subscribe("/" + robot_id_ + "/" + topic_name_req_put_robot_info, ROS_QUEUE_SIZE_1, &HttpBridge::reqRobotInfoCB, this);
    pub_put_robot_info_     = node.advertise<uoa_poc4_msgs::r_res_robot_info>("/" + robot_id_ + "/" + topic_name_res_put_robot_info, ROS_QUEUE_SIZE_1, true);

    // 地図のデータリスト（最新10件）するAPI
    std::string topic_name_req_map_data_list;
    std::string topic_name_res_map_data_list;
    getParam(param_node, "get_map_data_list/http_api",       api_get_map_data_list_,        std::string(DEF_GET_MAP_DATA_LIST_API));
    getParam(param_node, "get_map_data_list/req_topic_name", topic_name_req_map_data_list,  std::string(DEF_GET_MAP_DATA_LIST_REQ_TOPIC));
    getParam(param_node, "get_map_data_list/res_topic_name", topic_name_res_map_data_list,  std::string(DEF_GET_MAP_DATA_LIST_RES_TOPIC));
    sub_get_map_data_list_  = node.subscribe("/" + robot_id_ + "/" + topic_name_req_map_data_list, ROS_QUEUE_SIZE_2, &HttpBridge::reqMapdataListCB, this);
    pub_get_map_data_list_  = node.advertise<uoa_poc4_msgs::r_res_mapdata_list>("/" + robot_id_ + "/" + topic_name_res_map_data_list, ROS_QUEUE_SIZE_1, true);

    // 地図をRDRへPUTするAPI
    std::string topic_name_req_put_map_data;
    std::string topic_name_res_put_map_data;
    getParam(param_node, "put_map_data/http_api",       api_put_map_data_,           std::string(DEF_PUT_MAP_DATA_API));
    getParam(param_node, "put_map_data/req_topic_name", topic_name_req_put_map_data, std::string(DEF_PUT_MAP_DATA_REQ_TOPIC));
    getParam(param_node, "put_map_data/res_topic_name", topic_name_res_put_map_data, std::string(DEF_PUT_MAP_DATA_RES_TOPIC));
    sub_put_map_data_  = node.subscribe("/" + robot_id_ + "/" + topic_name_req_put_map_data, ROS_QUEUE_SIZE_2, &HttpBridge::reqPutMapdataCB, this);
    pub_put_map_data_  = node.advertise<uoa_poc4_msgs::r_res_put_mapdata>("/" + robot_id_ + "/" + topic_name_res_put_map_data, ROS_QUEUE_SIZE_1, true);

    // 地図をRDRからGETするAPI
    std::string topic_name_req_get_map_data;
    std::string topic_name_res_get_map_data;
    getParam(param_node, "get_map_data/http_api",       api_get_map_data_,               std::string(DEF_GET_MAP_DATA_API));
    getParam(param_node, "get_map_data/req_topic_name", topic_name_req_get_map_data,  std::string(DEF_GET_MAP_DATA_REQ_TOPIC));
    getParam(param_node, "get_map_data/res_topic_name", topic_name_res_get_map_data, std::string(DEF_GET_MAP_DATA_RES_TOPIC));
    sub_get_map_data_   = node.subscribe("/" + robot_id_ + "/" + topic_name_req_get_map_data, ROS_QUEUE_SIZE_2, &HttpBridge::reqGetMapdataCB, this);
    pub_get_map_data_   = node.advertise<uoa_poc4_msgs::r_res_get_mapdata>("/" + robot_id_ + "/" + topic_name_res_get_map_data, ROS_QUEUE_SIZE_1, true);

    // RDRに登録されている環境地図に紐づくレイヤー地図1件を取得するAPI
    std::string topic_name_req_get_layer_map_data;
    std::string topic_name_res_get_layer_map_data;
    getParam(param_node, "get_layer_map_data/http_api",       api_get_layer_map_data_,           std::string(DEF_GET_LAYER_MAP_DATA_API));
    getParam(param_node, "get_layer_map_data/req_topic_name", topic_name_req_get_layer_map_data,  std::string(DEF_GET_LAYER_MAP_DATA_REQ_TOPIC));
    getParam(param_node, "get_layer_map_data/res_topic_name", topic_name_res_get_layer_map_data, std::string(DEF_GET_LAYER_MAP_DATA_RES_TOPIC));
    sub_get_layer_map_data_ = node.subscribe("/" + robot_id_ + "/" + topic_name_req_get_layer_map_data, ROS_QUEUE_SIZE_2, &HttpBridge::reqGetLayerMapdataCB, this);
    pub_get_layer_map_data_ = node.advertise<uoa_poc4_msgs::r_res_get_mapdata>("/" + robot_id_ + "/" + topic_name_res_get_layer_map_data, ROS_QUEUE_SIZE_1, true);

    // 物体認識システムで認識したロボットの推定位置を取得するAPI
    std::string topic_name_req_get_cam_position_data;
    std::string topic_name_res_get_cam_position_data;
    getParam(param_node, "get_cam_position_data/http_api",       api_get_cam_position_data_,            std::string(DEF_GET_POSITION_DATA_API));
    getParam(param_node, "get_cam_position_data/req_topic_name", topic_name_req_get_cam_position_data,  std::string(DEF_GET_POSITION_DATA_REQ_TOPIC));
    getParam(param_node, "get_cam_position_data/res_topic_name", topic_name_res_get_cam_position_data,  std::string(DEF_GET_POSITION_DATA_RES_TOPIC));

    sub_get_cam_position_data_ = node.subscribe("/" + robot_id_ + "/" + topic_name_req_get_cam_position_data, ROS_QUEUE_SIZE_2, &HttpBridge::reqGetPositiondataCB, this);
    pub_get_cam_position_data_ = node.advertise<uoa_poc5_msgs::r_res_get_position_data>("/" + robot_id_ + "/" + topic_name_res_get_cam_position_data, ROS_QUEUE_SIZE_1, true);

    // 地図の補正値情報を配信するAPI
    std::string topic_name_req_put_correct_info;
    std::string topic_name_res_put_correct_info;
    getParam(param_node, "put_correct_info/http_api",       api_put_correct_info_,              std::string(DEF_PUT_CORRECT_INFO_API));
    getParam(param_node, "put_correct_info/req_topic_name", topic_name_req_put_correct_info,    std::string(DEF_PUT_CORRECT_INFO_REQ_TOPIC));
    getParam(param_node, "put_correct_info/res_topic_name", topic_name_res_put_correct_info,    std::string(DEF_PUT_CORRECT_INFO_RES_TOPIC));

    sub_put_correct_info_ = node.subscribe("/" + robot_id_ + "/" + topic_name_req_put_correct_info, ROS_QUEUE_SIZE_1, &HttpBridge::reqPutCorrectInfoCB, this);
    pub_put_correct_info_ = node.advertise<uoa_poc6_msgs::r_res_put_map_pose_correct>("/" + robot_id_ + "/" + topic_name_res_put_correct_info, ROS_QUEUE_SIZE_1, true);

    // 物体認識システムで認識した準静的物体の位置を取得するAPI
    std::string topic_name_req_change_obj_loc;
    std::string topic_name_res_change_obj_loc;
    getParam(param_node, "get_object_location/http_api",       api_get_object_location_,        std::string(DEF_GET_OBJECT_LOCATION_API));
    getParam(param_node, "get_object_location/req_topic_name", topic_name_req_change_obj_loc,   std::string(DEF_GET_OBJECT_LOCATION_REQ_TOPIC));
    getParam(param_node, "get_object_location/res_topic_name", topic_name_res_change_obj_loc,   std::string(DEF_GET_OBJECT_LOCATION_RES_TOPIC));

    sub_get_object_location_ = node.subscribe("/" + robot_id_ + "/" + topic_name_req_change_obj_loc, ROS_QUEUE_SIZE_1, &HttpBridge::reqGetObjectLocationCB, this);
    pub_get_object_location_ = node.advertise<uoa_poc6_msgs::r_res_change_obj_loc>("/" + robot_id_ + "/" + topic_name_res_change_obj_loc, ROS_QUEUE_SIZE_1, true);

    // 地図の補正値情報を取得するAPI
    std::string topic_name_req_get_correct_info;
    std::string topic_name_res_get_correct_info;
    getParam(param_node, "get_correct_info/http_api",       api_get_correct_info_,              std::string(DEF_GET_CORRECT_INFO_API));
    getParam(param_node, "get_correct_info/req_topic_name", topic_name_req_get_correct_info,    std::string(DEF_GET_CORRECT_INFO_REQ_TOPIC));
    getParam(param_node, "get_correct_info/res_topic_name", topic_name_res_get_correct_info,    std::string(DEF_GET_CORRECT_INFO_RES_TOPIC));

    sub_get_correct_info_ = node.subscribe("/" + robot_id_ + "/" + topic_name_req_get_correct_info, ROS_QUEUE_SIZE_1, &HttpBridge::reqGetCorrectInfoCB, this);
    pub_get_correct_info_ = node.advertise<uoa_poc6_msgs::r_res_get_map_pose_correct>("/" + robot_id_ + "/" + topic_name_res_get_correct_info, ROS_QUEUE_SIZE_1, true);

#if DEBUG
    getParam(param_node, "debug_print_map_data",    debug_print_map_data_,    DEF_DEBUG_PRINT_MAP_DATA);
#endif

}

HttpBridge::~HttpBridge()
{
    // do nothing
}

void HttpBridge::convertHeaderToJson(nlohmann::json& json_data, uoa_poc4_msgs::r_header header)
{
    if(!header.id.empty())
    {
        json_data["id"]     = header.id;
    }
    if(!header.name.empty())
    {
        json_data["name"]   = header.name;
    }
    if(!header.type.empty())
    {
        json_data["type"]   = header.type;
    }
    if(!header.space.empty())
    {
        json_data["space"]  = header.space;
    }
    if(!header.time.empty())
    {
        json_data["time"]   = header.time;
    }

}

void HttpBridge::convertJsonToHeader(uoa_poc4_msgs::r_header& header, nlohmann::json json_data)
{
    if(!json_data["id"].is_null())
    {
        header.id      = json_data["id"];
    }
    if(!json_data["name"].is_null())
    {
        header.name    = json_data["name"];
    }
    if(!json_data["type"].is_null())
    {
        header.type    = json_data["type"];
    }
    if(!json_data["space"].is_null())
    {
        header.space   = json_data["space"];
    }
    if(!json_data["time"].is_null())
    {
        header.time    = json_data["time"];
    }

}

void HttpBridge::convertLocationInfoToJson(nlohmann::json& json_data, uoa_poc4_msgs::r_map_location_info info, bool location_only)
{
    /********Data example********
    *   "location_info": {
    *        "location": "lictia_1f",
    *        "lat": 35.39181525,
    *        "lon": 140.05162781,
    *        "azimuth": 0
    *   }
    *****************************/

    if(location_only)
    {
        json_data["location"]   = info.location;
    }
    else
    {
        json_data["location_info"]["location"]   = info.location;
        json_data["location_info"]["lat"]        = info.lat;
        json_data["location_info"]["lon"]        = info.lon;
        json_data["location_info"]["azimuth"]    = info.azimuth;
    }

}

void HttpBridge::convertJsonToLocationInfo(uoa_poc4_msgs::r_map_location_info& info, nlohmann::json json_data)
{
    /********Data example********
    *   "location_info": {
    *        "location": "lictia_1f",
    *        "lat": 35.39181525,
    *        "lon": 140.05162781,
    *        "azimuth": 0
    *   }
    *****************************/

    if(!json_data["location_info"]["location"].is_null())
    {
        info.location   = json_data["location_info"]["location"];
    }
    if(!json_data["location_info"]["lat"].is_null())
    {
        info.lat        = json_data["location_info"]["lat"];
    }
    if(!json_data["location_info"]["lon"].is_null())
    {
        info.lon        = json_data["location_info"]["lon"];
    }
    if(!json_data["location_info"]["azimuth"].is_null())
    {
        info.azimuth    = json_data["location_info"]["azimuth"];
    }

}

void HttpBridge::convertMapdataInfoToJson(nlohmann::json& json_data, nav_msgs::OccupancyGrid map_data)
{
    
    json_data["map_data"]["header"]["seq"]                        = map_data.header.seq;
    json_data["map_data"]["header"]["stamp"]["secs"]              = map_data.header.stamp.sec;
    json_data["map_data"]["header"]["stamp"]["nsecs"]             = map_data.header.stamp.nsec;
    json_data["map_data"]["header"]["frame_id"]                   = map_data.header.frame_id;
    json_data["map_data"]["info"]["map_load_time"]["secs"]        = map_data.info.map_load_time.sec;
    json_data["map_data"]["info"]["map_load_time"]["nsecs"]       = map_data.info.map_load_time.nsec;
    json_data["map_data"]["info"]["resolution"]                   = map_data.info.resolution;
    json_data["map_data"]["info"]["width"]                        = map_data.info.width;
    json_data["map_data"]["info"]["height"]                       = map_data.info.height;
    json_data["map_data"]["info"]["origin"]["position"]["x"]      = map_data.info.origin.position.x;
    json_data["map_data"]["info"]["origin"]["position"]["y"]      = map_data.info.origin.position.y;
    json_data["map_data"]["info"]["origin"]["position"]["z"]      = map_data.info.origin.position.z;
    json_data["map_data"]["info"]["origin"]["orientation"]["x"]   = map_data.info.origin.orientation.x;
    json_data["map_data"]["info"]["origin"]["orientation"]["y"]   = map_data.info.origin.orientation.y;
    json_data["map_data"]["info"]["origin"]["orientation"]["z"]   = map_data.info.origin.orientation.z;
    json_data["map_data"]["info"]["origin"]["orientation"]["w"]   = map_data.info.origin.orientation.w;
    
    nlohmann::json array;
    
    for(unsigned int idx = 0; idx < map_data.data.size(); idx++)
    {
        array[idx] = map_data.data[idx];
    }

    json_data["map_data"]["data"] = array;

}

void HttpBridge::convertJsonToMapdataInfo(nav_msgs::OccupancyGrid& map_data, nlohmann::json json_data)
{
    if(!json_data["map_data"]["header"]["seq"].is_null())
    {
        map_data.header.seq =                   json_data["map_data"]["header"]["seq"];
    }
    if(!json_data["map_data"]["header"]["stamp"]["secs"].is_null())
    {
        map_data.header.stamp.sec =             json_data["map_data"]["header"]["stamp"]["secs"];
    }
    if(!json_data["map_data"]["header"]["stamp"]["nsecs"].is_null())
    {
        map_data.header.stamp.nsec =            json_data["map_data"]["header"]["stamp"]["nsecs"];
    }
    if(!json_data["map_data"]["header"]["frame_id"].is_null())
    {
        map_data.header.frame_id =              json_data["map_data"]["header"]["frame_id"];
    }
    if(!json_data["map_data"]["info"]["map_load_time"]["secs"].is_null())
    {
        map_data.info.map_load_time.sec =       json_data["map_data"]["info"]["map_load_time"]["secs"];
    }
    if(!json_data["map_data"]["info"]["map_load_time"]["nsecs"].is_null())
    {
        map_data.info.map_load_time.nsec =      json_data["map_data"]["info"]["map_load_time"]["nsecs"];
    }
    if(!json_data["map_data"]["info"]["resolution"].is_null())
    {
        map_data.info.resolution =              json_data["map_data"]["info"]["resolution"];
    }
    if(!json_data["map_data"]["info"]["width"].is_null())
    {
        map_data.info.width =                   json_data["map_data"]["info"]["width"];
    }
    if(!json_data["map_data"]["info"]["height"].is_null())
    {
        map_data.info.height =                  json_data["map_data"]["info"]["height"];
    }
    if(!json_data["map_data"]["info"]["origin"]["position"]["x"].is_null())
    {
        map_data.info.origin.position.x =       json_data["map_data"]["info"]["origin"]["position"]["x"];
    }
    if(!json_data["map_data"]["info"]["origin"]["position"]["y"].is_null())
    {
        map_data.info.origin.position.y =       json_data["map_data"]["info"]["origin"]["position"]["y"];
    }
    if(!json_data["map_data"]["info"]["origin"]["position"]["z"].is_null())
    {
        map_data.info.origin.position.z =       json_data["map_data"]["info"]["origin"]["position"]["z"];
    }
    if(!json_data["map_data"]["info"]["origin"]["orientation"]["x"].is_null())
    {
        map_data.info.origin.orientation.x =    json_data["map_data"]["info"]["origin"]["orientation"]["x"];
    }
    if(!json_data["map_data"]["info"]["origin"]["orientation"]["y"].is_null())
    {
        map_data.info.origin.orientation.y =    json_data["map_data"]["info"]["origin"]["orientation"]["y"];
    }
    if(!json_data["map_data"]["info"]["origin"]["orientation"]["z"].is_null())
    {
        map_data.info.origin.orientation.z =    json_data["map_data"]["info"]["origin"]["orientation"]["z"];
    }
    if(!json_data["map_data"]["info"]["origin"]["orientation"]["w"].is_null())
    {
        map_data.info.origin.orientation.w =    json_data["map_data"]["info"]["origin"]["orientation"]["w"];
    }
    
    // map_data.data = 
    int data_size = json_data["map_data"]["data"].size();
    
    map_data.data.resize(data_size);

    for(unsigned int idx = 0; idx < data_size; idx++)
    {
        map_data.data[idx] =  json_data["map_data"]["data"][idx];
    }

}

void HttpBridge::convertSystemInfoToJson(nlohmann::json& json_data, uoa_poc4_msgs::r_map_system_info info)
{

    if(!info.data_type.empty())
    {
        json_data["data_type"] = info.data_type;
    }
    if(!info.map_layer.empty())
    {
        json_data["map_layer"] = info.map_layer;
    }
    if(!info.revision.empty())
    {
        json_data["revision"] = info.revision;
    }
    if(!info.linked_revision.empty())
    {
        json_data["linked_revision"] = info.linked_revision;
    }

}

void HttpBridge::convertJsonToSystemInfo(uoa_poc4_msgs::r_map_system_info& info, nlohmann::json json_data)
{

    if(!json_data["data_type"].is_null())
    {
        info.data_type  = json_data["data_type"];
    }
    if(!json_data["map_layer"].is_null())
    {
        info.map_layer  = json_data["map_layer"];
    }
    if(!json_data["revision"].is_null())
    {
        info.revision   = json_data["revision"];
    }
    if(!json_data["linked_revision"].is_null())
    {
        info.linked_revision   = json_data["linked_revision"];
    }

}

bool HttpBridge::postData(cpr::Response& response, std::string api_url, nlohmann::json payload)
{
    bool is_success = false;

    // 設定
    cpr::Url url = cpr::Url{api_url};
    cpr::Header header{{"Content-Type", "application/json"}};
    cpr::Timeout timeout{10000};
    cpr::Body body{payload.dump()};

    // レスポンスのデータ格納
    response = cpr::Post(url, header, timeout, body);

    if(response.status_code == 200) is_success = true;

    return is_success;

}

void HttpBridge::reqRobotInfoCB(const uoa_poc4_msgs::r_req_robot_info& put_robot_info_msg)
{ // 起動時にロボットの情報を通知する

    // データのチェック
    if( !put_robot_info_msg.header.id.empty() &&
        !put_robot_info_msg.header.type.empty() &&
        !put_robot_info_msg.header.space.empty() &&
        !put_robot_info_msg.header.time.empty() &&
        !put_robot_info_msg.size.robot_radius + DBL_EPSILON > 0.0  &&
        !put_robot_info_msg.size.inflation_radius + DBL_EPSILON > 0.0)
    { // 必要なデータは格納されている

        // json形式に変換
        nlohmann::json  payload;
        
        convertHeaderToJson(payload, put_robot_info_msg.header);

        unsigned int idx = 0;
        nlohmann::json array; // footprint

        for(auto &i : put_robot_info_msg.size.footprint)
        {
            array[idx]["x"] = i.x;
            array[idx]["y"] = i.y;

            idx++;
        }

        payload["robot_size"]["radius"]           =   put_robot_info_msg.size.robot_radius;
        payload["robot_size"]["inflation_radius"] =   put_robot_info_msg.size.inflation_radius;
        payload["robot_size"]["footprint"]        =   array;

#ifdef DEBUG
        std::string body = payload.dump(4);
        std::wcout << "RobotInfoReqMSG: " << body.c_str() << std::endl;
#endif

        // データPOST
        cpr::Response response;
        postData(response, set_url_ + api_pub_robot_info_, payload);

        // レスポンスデータの検証
        nlohmann::json res_payload;
        if(response.status_code == 0)
        {
            ROS_ERROR_STREAM( response.error.message );
        }
        else if (response.status_code == 200) 
        { // 正常応答
            res_payload = nlohmann::json::parse(response.text);

#ifdef DEBUG
        std::string res_body = res_payload.dump(4);
        std::wcout << "RobotInfoResMSG: " << res_body.c_str() << std::endl;
        std::wcout << "Response Status Code: " << response.status_code << std::endl;
#endif

            // レスポンスデータの変換
            uoa_poc4_msgs::r_res_robot_info pub_data;
            // pub_data.status_code = response.status_code;
            pub_data.status_code = res_payload.at("code");

            // レスポンスデータの配信
            pub_put_robot_info_.publish(pub_data);

        }
        else 
        {
            ROS_ERROR_STREAM("reqMapdataList : Error [" << response.status_code << "] making request");
        }

    }

#if DEBUG
    else
    {
        ROS_WARN("reqRobotInfoCB : Param Error !!");
    }
#endif

    return;

}

void HttpBridge::reqMapdataListCB(const uoa_poc4_msgs::r_req_mapdata_list& get_map_list_msg)
{ // 地図のデータリストのリクエスト

    if( !get_map_list_msg.info.header.id.empty() &&
        !get_map_list_msg.info.header.type.empty() &&
        !get_map_list_msg.info.location.location.empty())
    { // 必要なデータは格納されている
        // json形式に変換
        nlohmann::json payload;
        convertHeaderToJson(payload, get_map_list_msg.info.header);
        convertSystemInfoToJson(payload, get_map_list_msg.info.system);
        convertLocationInfoToJson(payload, get_map_list_msg.info.location, ONLY_MAP_LOCATION);

#ifdef DEBUG
        std::string body = payload.dump(4);
        std::wcout << "GetMapDataListReqMSG: " << body.c_str() << std::endl;
#endif

        // データPOST
        cpr::Response response;
        postData(response, set_url_ + api_get_map_data_list_, payload);

        // レスポンスデータの検証
        nlohmann::json res_payload;
        if(response.status_code == 0)
        {
            ROS_ERROR_STREAM( response.error.message );
        }
        else if (response.status_code == 200) 
        { // 正常応答
            res_payload = nlohmann::json::parse(response.text);

#ifdef DEBUG
        std::string res_body = res_payload.dump(4);
        std::wcout << "GetMapDataListResMSG: " << res_body.c_str() << std::endl;
        std::wcout << "Response Status Code: " << response.status_code << std::endl;
#endif

            // レスポンスデータの変換
            // uoa_poc4_msgs::r_res_mapdata_list pub_data
            
            // // レスポンスデータの配信
            // pub_get_map_data_list_.publish(pub_data);

        }
        else 
        {
            ROS_ERROR_STREAM("reqMapdataList : Error [" << response.status_code << "] making request");
        }


#ifdef DEBUG
        std::cout << response.text << std::endl;
#endif

    }
#if DEBUG
    else
    {
        ROS_WARN("reqMapdataListCB : Param Error !!");
    }
#endif

    return;
}

void HttpBridge::reqPutMapdataCB(const uoa_poc4_msgs::r_req_put_mapdata& put_mapdata_msg)
{

    if( !put_mapdata_msg.map_info.header.id.empty() &&
        !put_mapdata_msg.map_info.header.type.empty() &&
        !put_mapdata_msg.map_info.header.space.empty() &&
        !put_mapdata_msg.map_info.header.time.empty() &&
        !put_mapdata_msg.map_info.location.location.empty() &&
        !put_mapdata_msg.map_info.system.map_layer.empty() &&
        (put_mapdata_msg.map_info.system.map_layer == DEF_EGO_MAP_NAME ||
        put_mapdata_msg.map_info.system.map_layer == DEF_SOCIO_MAP_NAME ||
        put_mapdata_msg.map_info.system.map_layer == DEF_REFERENCE_MAP_NAME ||
        put_mapdata_msg.map_info.system.map_layer == DEF_ENVIRONMENT_MAP_NAME ||
        put_mapdata_msg.map_info.system.map_layer == DEF_STATIC_LAYER_MAP_NAME ||
        put_mapdata_msg.map_info.system.map_layer == DEF_QUASI_STATIC_LAYER_MAP_NAME ||
        put_mapdata_msg.map_info.system.map_layer == DEF_EXCLUSION_ZONE_LAYER_MAP_NAME) 
    )
    { // 必要なデータは格納されている
        
        // json形式に変換
        nlohmann::json  payload;
        convertHeaderToJson(payload, put_mapdata_msg.map_info.header);
        convertLocationInfoToJson(payload, put_mapdata_msg.map_info.location, NOT_ONLY_MAP_LOCATION);
        convertMapdataInfoToJson(payload, put_mapdata_msg.map_data);
        convertSystemInfoToJson(payload, put_mapdata_msg.map_info.system);
        
        if(put_mapdata_msg.map_info.system.linked_revision.empty())
        {
            payload["linked_revision"] = "";
        }

#ifdef DEBUG
        std::string body = payload.dump(4);
        std::wcout << "PutMapDataReqMSG: " << body.c_str() << std::endl;
#endif
// データPOST
        cpr::Response response;
        std::string post_url = set_url_ + api_put_map_data_;
        postData(response, post_url, payload);

        // レスポンスデータの検証
        nlohmann::json res_payload;
        
        if(response.status_code == 0)
        {
            ROS_ERROR_STREAM( response.error.message );
        }
        else if (response.status_code == 200) 
        { // 正常応答
            res_payload = nlohmann::json::parse(response.text);

#ifdef DEBUG
        std::string res_body = res_payload.dump(4);
        std::wcout << "PutMapDataResMSG: " << res_body.c_str() << std::endl;
        std::wcout << "Response Status Code: " << response.status_code << std::endl;
#endif
            // レスポンスデータの変換
            uoa_poc4_msgs::r_res_put_mapdata pub_data;

            convertJsonToHeader(pub_data.info.header, res_payload);
            convertJsonToLocationInfo(pub_data.info.location, res_payload);
            convertJsonToSystemInfo(pub_data.info.system, res_payload);

            pub_data.status_code = response.status_code;

            // レスポンスデータの配信
            pub_put_map_data_.publish(pub_data);

        }
        else 
        {
            ROS_ERROR_STREAM("reqPutMapdata : Error [" << response.status_code << "] making request");
        }

    }
#if DEBUG

    else
    {
        ROS_WARN("reqPutMapdataCB : Param Error !!");
    }
#endif

    return;
}

void HttpBridge::reqGetMapdataCB(const uoa_poc4_msgs::r_req_get_mapdata& get_map_msg)
{
    // データのチェック
    if( 
        // !get_map_msg.info.header.time.empty() &&
        // !get_map_msg.info.location.location.empty() &&
        !get_map_msg.info.system.data_type.empty() &&
        (get_map_msg.info.system.latest == true || get_map_msg.info.system.latest == false ) // &&
        // !get_map_msg.info.system.map_layer.empty() &&
        // !get_map_msg.info.system.revision.empty() 
      )
    { // 必要なデータは格納されている
        
        // json形式に変換
        nlohmann::json  payload;

        convertHeaderToJson(payload, get_map_msg.info.header);
        convertSystemInfoToJson(payload, get_map_msg.info.system);

        if(get_map_msg.info.header.time.empty())
        {
            payload["time"] = "";
        }
        
        if(get_map_msg.info.header.space.empty())
        {
            payload["space"] = "";
        }
        
        if(get_map_msg.info.system.revision.empty())
        {
            payload["revision"] = "";
        }
        
        if(get_map_msg.info.system.map_layer.empty())
        {
            payload["map_layer"] = "";
        }

        payload["location"] = get_map_msg.info.location.location;
        payload["latest"] = get_map_msg.info.system.latest;
        
#ifdef DEBUG
        std::string body = payload.dump(4);
        std::wcout << "GetMapdataReqMSG: " << body.c_str() << std::endl;
#endif
// データPOST
        cpr::Response response;
        postData(response, set_url_ + api_get_map_data_, payload);

        // レスポンスデータの検証
        nlohmann::json res_payload;
        if(response.status_code == 0)
        {
            ROS_ERROR_STREAM( response.error.message );
        }
        else if (response.status_code == 200) 
        { // 正常応答
            res_payload = nlohmann::json::parse(response.text);

#ifdef DEBUG
        nlohmann::json temp_json = res_payload;
        
        if(!debug_print_map_data_)
        {   // 地図データを一部表示モード
            temp_json["map_data"]["data"].clear();
            std::wcout << "---- Contents of [data] are omitted. ----" << std::endl;    
        }
        
        std::string res_body = temp_json.dump(4);
        std::wcout << "GetMapdataResMSG: " << res_body.c_str() << std::endl;
        std::wcout << "Response Status Code: " << response.status_code << std::endl;
#endif

            // レスポンスデータの変換
            uoa_poc4_msgs::r_res_get_mapdata pub_data;

            convertJsonToHeader(pub_data.map_info.header, res_payload);
            convertJsonToLocationInfo(pub_data.map_info.location, res_payload);
            convertJsonToSystemInfo(pub_data.map_info.system, res_payload);
            convertJsonToMapdataInfo(pub_data.map_data, res_payload);
            pub_data.status_code = response.status_code;

            // レスポンスデータの配信
            pub_get_map_data_.publish(pub_data);

        }
        else 
        {
            ROS_ERROR_STREAM("reqGetMapdata : Error [" << response.status_code << "] making request");
        }
    
    }
#if DEBUG
    else
    {
        ROS_WARN("reqGetMapdataCB : Param Error !!");
    }
#endif

    return;
}

void HttpBridge::reqGetLayerMapdataCB(const uoa_poc4_msgs::r_req_get_mapdata& get_map_msg)
{
    // データのチェック
    if( 
        !get_map_msg.info.system.revision.empty() &&
        !get_map_msg.info.system.map_layer.empty()  
      )
    { // 必要なデータは格納されている
        
        // json形式に変換
        nlohmann::json  payload;
        
        payload["map_layer"] = get_map_msg.info.system.map_layer;
        convertSystemInfoToJson(payload, get_map_msg.info.system);
                
#ifdef DEBUG
        std::string body = payload.dump(4);
        std::wcout << "GetLayerMapdataReqMSG: " << body.c_str() << std::endl;
#endif
        // データPOST
        cpr::Response response;
        postData(response, set_url_ + api_get_layer_map_data_, payload);

        // レスポンスデータの検証
        nlohmann::json res_payload;
        if(response.status_code == 0)
        {
            ROS_ERROR_STREAM( response.error.message );
        }
        else if (response.status_code == 200) 
        { // 正常応答
            res_payload = nlohmann::json::parse(response.text);

#ifdef DEBUG
        nlohmann::json temp_json = res_payload;
        
        if(!debug_print_map_data_)
        {   // 地図データを一部表示モード
            temp_json["map_data"]["data"].clear();
            std::wcout << "---- Contents of [data] are omitted. ----" << std::endl;    
        }
        
        std::string res_body = temp_json.dump(4);
        std::wcout << "GetLayerMapdataResMSG: " << res_body.c_str() << std::endl;
        std::wcout << "Response Status Code: " << response.status_code << std::endl;
#endif

            // レスポンスデータの変換
            uoa_poc4_msgs::r_res_get_mapdata pub_data;

            convertJsonToHeader(pub_data.map_info.header, res_payload);
            convertJsonToLocationInfo(pub_data.map_info.location, res_payload);
            convertJsonToSystemInfo(pub_data.map_info.system, res_payload);
            convertJsonToMapdataInfo(pub_data.map_data, res_payload);
            pub_data.status_code = response.status_code;

            // レスポンスデータの配信
            pub_get_layer_map_data_.publish(pub_data);

        }
        else 
        {
            ROS_ERROR_STREAM("reqGetLayerMapdata : Error [" << response.status_code << "] making request");
        }
    }
#if DEBUG
    else
    {
        ROS_WARN("reqGetLayerMapdataCB : Param Error !!");
    }
#endif


    return;
}

void HttpBridge::reqGetPositiondataCB(const uoa_poc4_msgs::r_req_get_position_data& get_position_msg)
{
    // データのチェック
    if( 
        !get_position_msg.header.name.empty() 
      )
    { // 必要なデータは格納されている
        
        // json形式に変換
        nlohmann::json  payload;
        convertHeaderToJson(payload, get_position_msg.header);

#ifdef DEBUG
        std::string body = payload.dump(4);
        std::wcout << "GetPositionDataReqMSG: " << body.c_str() << std::endl;
#endif

        // データPOST
        cpr::Response response;
        postData(response, set_url_ + api_get_cam_position_data_, payload);

        // レスポンスデータの検証
        nlohmann::json res_payload;
        if(response.status_code == 0)
        {
            ROS_ERROR_STREAM( response.error.message );
        }
        else if (response.status_code == 200) 
        { // 正常応答
            res_payload = nlohmann::json::parse(response.text);

#ifdef DEBUG
        std::string res_body = res_payload.dump(4);
        std::wcout << "GetPositionDataResMSG: " << res_body.c_str() << std::endl;
        std::wcout << "Response Status Code: " << response.status_code << std::endl;
#endif

            // レスポンスデータの変換
            uoa_poc5_msgs::r_res_get_position_data pub_data;

            if(!res_payload["name"].is_null())
            {
                pub_data.name               = res_payload["name"];
            }
            if(!res_payload["acquisition_time"].is_null())
            {
                pub_data.acquisition_time   = res_payload["acquisition_time"]; // 位置情報取得時間
            }
            pub_data.pose.position.x    = res_payload["position"]["x"]; // 位置データ
            pub_data.pose.position.y    = res_payload["position"]["y"];
            pub_data.pose.position.z    = res_payload["position"]["z"];
            pub_data.pose.orientation.x = res_payload["orientation"]["x"];
            pub_data.pose.orientation.y = res_payload["orientation"]["y"];
            pub_data.pose.orientation.z = res_payload["orientation"]["z"];
            pub_data.pose.orientation.w = res_payload["orientation"]["w"];
            
            pub_data.status_code = response.status_code;


            // レスポンスデータの配信
            pub_get_cam_position_data_.publish(pub_data);

        }
        else 
        {
            ROS_ERROR_STREAM("reqGetPositionData : Error [" << response.status_code << "] making request");
        }
     }
#if DEBUG
    else
    {
        ROS_WARN("reqGetPositionDataCB : Param Error !!");
    }
#endif

    return;
}

void HttpBridge::reqPutCorrectInfoCB(const uoa_poc6_msgs::r_req_put_map_pose_correct& put_correct_info_msg)
{
    // データのチェック
    if( !put_correct_info_msg.map_identities.source_map.system.map_layer.empty() &&
        !put_correct_info_msg.map_identities.source_map.header.space.empty() &&
        !put_correct_info_msg.map_identities.target_map.system.map_layer.empty() &&
        !put_correct_info_msg.map_identities.target_map.header.space.empty() )
    { // 必要なデータは格納されている
        
        // json形式に変換
        nlohmann::json  payload;
        convertHeaderToJson(payload["map_identities"]["target_map"], put_correct_info_msg.map_identities.target_map.header);
        convertSystemInfoToJson(payload["map_identities"]["target_map"], put_correct_info_msg.map_identities.target_map.system);
        convertHeaderToJson(payload["map_identities"]["source_map"], put_correct_info_msg.map_identities.source_map.header);
        convertSystemInfoToJson(payload["map_identities"]["source_map"], put_correct_info_msg.map_identities.source_map.system);

        payload["rmse"]                                      = put_correct_info_msg.rmse;
        payload["relative_position"]["position"]["x"]        = put_correct_info_msg.relative_position.position.x;
        payload["relative_position"]["position"]["y"]        = put_correct_info_msg.relative_position.position.y;
        payload["relative_position"]["position"]["z"]        = put_correct_info_msg.relative_position.position.z;
        payload["relative_position"]["orientation"]["x"]     = put_correct_info_msg.relative_position.orientation.x;
        payload["relative_position"]["orientation"]["y"]     = put_correct_info_msg.relative_position.orientation.y;
        payload["relative_position"]["orientation"]["z"]     = put_correct_info_msg.relative_position.orientation.z;
        payload["relative_position"]["orientation"]["w"]     = put_correct_info_msg.relative_position.orientation.w;

#ifdef DEBUG
        std::string body = payload.dump(4);
        std::wcout << "PutCorrectInfoReqMSG: " << body.c_str() << std::endl;
#endif

        // データPOST
        cpr::Response response;
        postData(response, set_url_ + api_put_correct_info_, payload);

        // レスポンスデータの検証
        nlohmann::json res_payload;
        if(response.status_code == 0)
        {
            ROS_ERROR_STREAM( response.error.message );
        }
        else if (response.status_code == 200) 
        { // 正常応答
            if(!response.text.empty())
            {
                res_payload = nlohmann::json::parse(response.text); // レスポンスが空の場合、json形式じゃないという例外（nlohmann::detail::parse_error）になる
            }
            
#ifdef DEBUG
        std::string res_body = res_payload.dump(4);
        std::wcout << "PutCorrectInfoResMSG: " << res_body.c_str() << std::endl;
        std::wcout << "Response Status Code: " << response.status_code << std::endl;
#endif

            // レスポンスデータの変換
            uoa_poc6_msgs::r_res_put_map_pose_correct pub_data;

            pub_data.status_code    = response.status_code;

            // レスポンスデータの配信
            pub_put_correct_info_.publish(pub_data);

        }
        else 
        {
            ROS_ERROR_STREAM("reqPutCorrectInfo : Error [" << response.status_code << "] making request");
        }
     }
#if DEBUG
    else
    {
        ROS_WARN("reqPutCorrectInfo : Param Error !!");
    }
#endif

    return;
}

void HttpBridge::reqGetObjectLocationCB(const uoa_poc6_msgs::r_req_change_obj_loc& get_obj_location_msg)
{
    // データのチェック
    if( 
        !get_obj_location_msg.range.start.empty() ||
        !get_obj_location_msg.range.end.empty() 
      )
    { // 必要なデータは格納されている
        
        // json形式に変換
        nlohmann::json  payload;
        payload["range"]["start"]   = get_obj_location_msg.range.start;
        payload["range"]["end"]     = get_obj_location_msg.range.end;
        if(get_obj_location_msg.latest)
        {
            payload["latest"]       = true;
        }
        else
        {
            payload["latest"]       = false;
        }
        
        // オブジェクト個別取得化対応用
        // unsigned int idx = 0;
        // nlohmann::json array; // footprint
        // for(auto &i : get_obj_location_msg.object_name)
        // {
        //     array[idx] = i;

        //     idx++;
        // }
        // payload["object_name"] = array;


        // 2023/12/15：準静的物体の位置変化更新時間取得の暫定コード
        cpr::Response res_update_time;
        nlohmann::json  update_time_req_payload;
        nlohmann::json  update_time_res_payload;
        postData(res_update_time, set_url_ + "/RDBS/data/rdr/get/LayoutInfo", update_time_req_payload);
   
#ifdef DEBUG
        std::string body = payload.dump(4);
        std::wcout << "GetObjectLocationReqMSG: " << body.c_str() << std::endl;
#endif

        // データPOST
        cpr::Response response;
        
#ifdef DEBUG
        std::wcout << "GetObjectLocation API: " << (set_url_ + api_get_object_location_).c_str() << std::endl;
#endif
        postData(response, set_url_ + api_get_object_location_, payload);

        // レスポンスデータの検証
        nlohmann::json res_payload;
        if(response.status_code == 0)
        {
            ROS_ERROR_STREAM( response.error.message );
        }
        else if (response.status_code == 200)
        { // 正常応答
            if(res_update_time.status_code == 200)
            { // 時間取得結果が正常応答
                res_payload = nlohmann::json::parse(response.text);
                update_time_req_payload = nlohmann::json::parse(res_update_time.text);
            
#ifdef DEBUG
            std::string res_body = res_payload.dump(4);
            std::wcout << "GetPositionDataResMSG: " << res_body.c_str() << std::endl;
            std::wcout << "Response Status Code: " << response.status_code << std::endl;
#endif

                // レスポンスデータの変換
                uoa_poc6_msgs::r_res_change_obj_loc pub_data;
                size_t obj_size = res_payload.size();
                pub_data.objects.resize(obj_size);

                for(unsigned int idx = 0; idx < res_payload.size(); idx++)
                {
                    pub_data.objects[idx].name                   = res_payload[idx]["name"];
                    pub_data.objects[idx].new_pose.position.x    = res_payload[idx]["new_pose"]["position"]["x"]; // 位置データ
                    pub_data.objects[idx].new_pose.position.y    = res_payload[idx]["new_pose"]["position"]["y"];
                    pub_data.objects[idx].new_pose.position.z    = res_payload[idx]["new_pose"]["position"]["z"];
                    pub_data.objects[idx].new_pose.orientation.x = res_payload[idx]["new_pose"]["orientation"]["x"];
                    pub_data.objects[idx].new_pose.orientation.y = res_payload[idx]["new_pose"]["orientation"]["y"];
                    pub_data.objects[idx].new_pose.orientation.z = res_payload[idx]["new_pose"]["orientation"]["z"];
                    pub_data.objects[idx].new_pose.orientation.w = res_payload[idx]["new_pose"]["orientation"]["w"];
                }

                pub_data.update_time = update_time_req_payload["datetime"];
                pub_data.status_code = response.status_code;

                // レスポンスデータの配信
                pub_get_object_location_.publish(pub_data);

            }
        }
        else 
        {
            ROS_ERROR_STREAM("reqGetObjectLocation : Error [" << response.status_code << "] making request");
        }
     }
#if DEBUG
    else
    {
        ROS_WARN("reqGetObjectLocation : Param Error !!");
    }
#endif

    return;
}

void HttpBridge::reqGetCorrectInfoCB(const uoa_poc6_msgs::r_req_get_map_pose_correct& get_correct_info_msg)
{
    // データのチェック
    // if( !get_correct_info_msg.map_identities.source_map.system.map_layer.empty() &&
    //     !get_correct_info_msg.map_identities.source_map.header.space.empty() &&
    //     !get_correct_info_msg.map_identities.target_map.system.map_layer.empty() &&
    //     !get_correct_info_msg.map_identities.target_map.header.space.empty() )
    // { // 必要なデータは格納されている
        
        // json形式に変換
        nlohmann::json  payload;
        convertHeaderToJson(payload["map_identities"]["target_map"], get_correct_info_msg.map_identities.target_map.header);
        convertSystemInfoToJson(payload["map_identities"]["target_map"], get_correct_info_msg.map_identities.target_map.system);
        convertHeaderToJson(payload["map_identities"]["source_map"], get_correct_info_msg.map_identities.source_map.header);
        convertSystemInfoToJson(payload["map_identities"]["source_map"], get_correct_info_msg.map_identities.source_map.system);

        // 空間情報が無い場合
        if(get_correct_info_msg.map_identities.target_map.header.space.empty())
        {
            payload["map_identities"]["target_map"]["space"] = "";
        }

        if(get_correct_info_msg.map_identities.source_map.header.space.empty())
        {
            payload["map_identities"]["source_map"]["space"] = "";
        }

        // 地図種別が無い場合
        if(get_correct_info_msg.map_identities.target_map.system.map_layer.empty())
        {
            payload["map_identities"]["target_map"]["map_layer"] = "";
        }

        if(get_correct_info_msg.map_identities.source_map.system.map_layer.empty())
        {
            payload["map_identities"]["source_map"]["map_layer"] = "";
        }

        // リビジョンが無い場合
        if(get_correct_info_msg.map_identities.target_map.system.revision.empty())
        {
            payload["map_identities"]["target_map"]["revision"] = "";
        }

        if(get_correct_info_msg.map_identities.source_map.system.revision.empty())
        {
            payload["map_identities"]["source_map"]["revision"] = "";
        }

#ifdef DEBUG
        std::string body = payload.dump(4);
        std::wcout << "GetCorrectInfoReqMSG: " << body.c_str() << std::endl;
#endif

        // データPOST
        cpr::Response response;
        postData(response, set_url_ + api_get_correct_info_, payload);

        // レスポンスデータの検証
        nlohmann::json res_payload;
        if(response.status_code == 0)
        {
            ROS_ERROR_STREAM( response.error.message );
        }
        else if (response.status_code == 200) 
        { // 正常応答
            if(!response.text.empty())
            {
                res_payload = nlohmann::json::parse(response.text); // レスポンスが空の場合、json形式じゃないという例外（nlohmann::detail::parse_error）になる
            }
            
#ifdef DEBUG
        std::string res_body = res_payload.dump(4);
        std::wcout << "GetCorrectInfoCBResMSG: " << res_body.c_str() << std::endl;
        std::wcout << "Response Status Code: " << response.status_code << std::endl;
#endif

            // レスポンスデータの変換
            uoa_poc6_msgs::r_res_get_map_pose_correct pub_data;
            pub_data.relative_position.position.x    = res_payload["relative_position"]["position"]["x"]; // 位置データ
            pub_data.relative_position.position.y    = res_payload["relative_position"]["position"]["y"];
            pub_data.relative_position.position.z    = res_payload["relative_position"]["position"]["z"];
            pub_data.relative_position.orientation.x = res_payload["relative_position"]["orientation"]["x"];
            pub_data.relative_position.orientation.y = res_payload["relative_position"]["orientation"]["y"];
            pub_data.relative_position.orientation.z = res_payload["relative_position"]["orientation"]["z"];
            pub_data.relative_position.orientation.w = res_payload["relative_position"]["orientation"]["w"];
            pub_data.rmse                            = res_payload["rmse"];

            pub_data.status_code    = response.status_code;

            // レスポンスデータの配信
            pub_get_correct_info_.publish(pub_data);

        }
        else 
        {
            ROS_ERROR_STREAM("reqGetCorrectInfo : Error [" << response.status_code << "] making request");
        }
    //  }
#if DEBUG
    // else
    // {
    //     ROS_WARN("reqPutCorrectInfo : Param Error !!");
    // }
#endif

    return;
}

void HttpBridge::bridgeLoop(void)
{
    ros::Rate sleep_rate = ROS_RATE_1HZ; // Hz

    while(ros::ok)
    {
        sleep_rate.sleep(); // Sleep
        ros::spinOnce();    // コールバック関数実行

    }

    return;
}