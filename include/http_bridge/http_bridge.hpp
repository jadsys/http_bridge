/**
* @file     http_bridge.hpp
* @brief    HTTP通信の実行クラスHttpBridgeのヘッダファイル
* @author   S.Kumada
* @date     2023/10/6
* @note     HTTP通信により、サーバーからのデータ取得や配信を行うクラスの定義
*/

#include <cpr/cpr.h>
#include <nlohmann/json.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Quaternion.h>

#include <cstdio>
// #include <fmt/format.h>

// #undef U // 202205roscd 23 Boost/cpprestsdkによるU(x)マクロ２重定義エラー対策

#include "uoa_poc3_msgs/r_info.h"
#include "uoa_poc4_msgs/r_req_robot_info.h"
#include "uoa_poc4_msgs/r_res_robot_info.h"
#include "uoa_poc4_msgs/r_res_mapdata_list.h"
#include "uoa_poc4_msgs/r_req_mapdata_list.h"
#include "uoa_poc4_msgs/r_req_put_mapdata.h"
#include "uoa_poc4_msgs/r_res_put_mapdata.h"
#include "uoa_poc4_msgs/r_req_get_mapdata.h"
#include "uoa_poc4_msgs/r_res_get_mapdata.h"
#include "uoa_poc4_msgs/r_sub_map_update_notify.h"
#include "uoa_poc4_msgs/r_req_get_position_data.h"
#include "uoa_poc5_msgs/r_res_get_position_data.h"
#include "uoa_poc6_msgs/r_req_change_obj_loc.h"
#include "uoa_poc6_msgs/r_res_change_obj_loc.h"
#include "uoa_poc6_msgs/r_req_put_map_pose_correct.h"
#include "uoa_poc6_msgs/r_res_put_map_pose_correct.h"
#include "uoa_poc6_msgs/r_req_get_map_pose_correct.h"
#include "uoa_poc6_msgs/r_res_get_map_pose_correct.h"

// define定義
#define DEF_ROBOT_ID                        "turtlebot_01"
#define DEF_ROBOT_TYPE                      "turtlebot"
#define DEF_MAP_LOCATION                    "lictia_1f"
#define DEF_RETRY_TIME                      5 // 5秒
#define DEF_RETRY_COUNT                     5 // 5回
#define DEF_EGO_MAP_TOPIC_NAME              "map"
#define DEF_SOCIO_MAP_TOPIC_NAME            "map_movebase"
#define DEF_SYSTEM_TYPE                     "ROS"
#define DEF_SPACE_INFOMATION                "real"
#define DEF_EGO_MAP_NAME                    "ego"
#define DEF_SOCIO_MAP_NAME                  "socio"
#define DEF_STATIC_LAYER_MAP_NAME           "static_layer"
#define DEF_QUASI_STATIC_LAYER_MAP_NAME     "semi_static_layer"
#define DEF_EXCLUSION_ZONE_LAYER_MAP_NAME   "exclusion_zone_layer"
#define DEF_ENVIRONMENT_MAP_NAME            "environment"
#define DEF_REFERENCE_MAP_NAME              "reference"

#define DEF_MAP_FLAME_NAME                  "map"


#define DEF_TOPIC_PREFIX        "turtlebot_01"
#define DEF_MAP_NAME            "map"
#define DEF_TOPIC_NAME          "map"
#define DEF_FILE_NAME           "map"
#define DEF_SAVE_LOCATION       "~/maps"
#define DEF_SAVE_INTERVAL_TIME  5.0
#define DEF_SAVE_TIMES          0
#define DEF_THRESHOLD_OCCUPIED  65
#define DEF_THRESHOLD_FREE      25
#define DEF_SAVE_REGULARLY      true
#define ONLY_MAP_LOCATION       true
#define NOT_ONLY_MAP_LOCATION   false
#define DEF_DEBUG_PRINT_MAP_DATA    true

#define DEF_SET_URL                 "https://httpbin.org/"
#define DEF_PUT_ROBOT_INFO_API      "/RDBS/data/rdr/put/RobotInfo" // API名
#define DEF_PUT_ROBOT_REQ_TOPIC     "/ros_bridge/req_robot_info" // リクエスト受信トピック
#define DEF_PUT_ROBOT_RES_TOPIC     "/ros_bridge/res_robot_info" // レスポンス配信トピック
#define DEF_GET_MAP_DATA_LIST_API   "/RDBS/data/rdr/get/MapDataList" // API名
#define DEF_GET_MAP_DATA_LIST_REQ_TOPIC "/ros_bridge/req_map_data_list" // リクエスト受信トピック
#define DEF_GET_MAP_DATA_LIST_RES_TOPIC "/ros_bridge/res_map_data_list" // レスポンス配信トピック
#define DEF_PUT_MAP_DATA_API        "/RDBS/data/rdr/put/MapData" // API名
#define DEF_PUT_MAP_DATA_REQ_TOPIC  "/ros_bridge/req_put_map_data" // リクエスト受信トピック
#define DEF_PUT_MAP_DATA_RES_TOPIC  "/ros_bridge/res_put_map_data" // レスポンス配信トピック
#define DEF_GET_MAP_DATA_API        "/RDBS/data/rdr/get/MapData" // API名
#define DEF_GET_MAP_DATA_REQ_TOPIC  "/ros_bridge/req_get_map_data" // リクエスト受信トピック
#define DEF_GET_MAP_DATA_RES_TOPIC  "/ros_bridge/res_get_map_data" // レスポンス配信トピック
#define DEF_GET_LAYER_MAP_DATA_API        "/RDBS/data/rdr/get/MapData/layer" // API名
#define DEF_GET_LAYER_MAP_DATA_REQ_TOPIC  "/ros_bridge/req_get_layer_map_data" // リクエスト受信トピック
#define DEF_GET_LAYER_MAP_DATA_RES_TOPIC  "/ros_bridge/res_get_layer_map_data" // レスポンス配信トピック
#define DEF_GET_POSITION_DATA_API        "/RDBS/data/rdr/get/DetectionPosition" // API名
#define DEF_GET_POSITION_DATA_REQ_TOPIC  "/ros_bridge/req_get_position_data" // リクエスト受信トピック
#define DEF_GET_POSITION_DATA_RES_TOPIC  "/ros_bridge/res_get_position_data" // レスポンス配信トピック
#define DEF_GET_OBJECT_LOCATION_API        "/RDBS/data/rdr/get/ObjectLocation" // API名
#define DEF_GET_OBJECT_LOCATION_REQ_TOPIC  "/ros_bridge/req_get_object_location" // リクエスト受信トピック
#define DEF_GET_OBJECT_LOCATION_RES_TOPIC  "/ros_bridge/res_get_object_location" // レスポンス配信トピック
#define DEF_PUT_CORRECT_INFO_API        "/RDBS/data/rdr/put/MapPoseCorrect" // API名
#define DEF_PUT_CORRECT_INFO_REQ_TOPIC  "/ros_bridge/http/req/put_correction_value" // リクエスト受信トピック
#define DEF_PUT_CORRECT_INFO_RES_TOPIC  "/ros_bridge/http/res/put_correction_value" // レスポンス配信トピック
#define DEF_GET_CORRECT_INFO_API        "/RDBS/data/rdr/get/MapPoseCorrect" // API名
#define DEF_GET_CORRECT_INFO_REQ_TOPIC  "/ros_bridge/http/req/get_correction_value" // リクエスト受信トピック
#define DEF_GET_CORRECT_INFO_RES_TOPIC  "/ros_bridge/http/res/get_correction_value" // レスポンス配信トピック


// ROS Time
#define ROS_RATE_1HZ                        1
#define ROS_RATE_10HZ                       10
#define ROS_RATE_15HZ                       15
#define ROS_RATE_30HZ                       30
#define ROS_RATE_45HZ                       45
#define ROS_RATE_60HZ                       50
#define ROS_RATE_1HZ_TIME                   1/ROS_RATE_1HZ
#define ROS_RATE_10HZ_TIME                  1/ROS_RATE_10HZ
#define ROS_RATE_15HZ_TIME                  1/ROS_RATE_15HZ
#define ROS_RATE_30HZ_TIME                  1/ROS_RATE_30HZ
#define ROS_RATE_45HZ_TIME                  1/ROS_RATE_45HZ
#define ROS_RATE_60HZ_TIME                  1/ROS_RATE_60HZ
#define ROS_TIME_5_SEC                      5

// queueサイズ
#define ROS_QUEUE_SIZE_1                    1
#define ROS_QUEUE_SIZE_2                    2
#define ROS_QUEUE_SIZE_10                   10



/**
 * @brief http通信ノードクラス
 * @details RDR/Planner側とのhttp通信を行うためのブリッジプログラム。
 */
class HttpBridge
{
    private:

        std::string robot_id_;              // ロボットの機体名
        std::string set_url_;               // 接続先のURI

    // ロボットの機体情報をRDRに登録するAPI
        std::string     api_pub_robot_info_;        // ロボットの情報一覧するAPI
        ros::Publisher  pub_put_robot_info_;
        ros::Subscriber sub_put_robot_info_;

    // 地図のデータリスト（最新10件）するAPI
        std::string     api_get_map_data_list_;
        ros::Publisher  pub_get_map_data_list_;
        ros::Subscriber sub_get_map_data_list_;

    // 地図をRDRへPUTするAPI
        std::string     api_put_map_data_;      // 地図をRDRへPUTするAPI
        ros::Publisher  pub_put_map_data_;
        ros::Subscriber sub_put_map_data_;

    // 地図をRDRからGETするAPI
        std::string     api_get_map_data_;
        ros::Publisher  pub_get_map_data_;
        ros::Subscriber sub_get_map_data_;

    // RDRに登録されている環境地図に紐づくレイヤー地図1件を取得するAPI
        std::string     api_get_layer_map_data_;      // 環境地図に紐づくレイヤー地図1件を取得するAPI
        ros::Publisher  pub_get_layer_map_data_;
        ros::Subscriber sub_get_layer_map_data_;

    // 物体認識システムで認識したロボットの推定位置を取得するAPI
        std::string     api_get_cam_position_data_;      // 物体認識システムで認識したロボットの推定位置を取得するAPI
        ros::Publisher  pub_get_cam_position_data_;
        ros::Subscriber sub_get_cam_position_data_;

    // 地図の補正値情報を配信するAPI
        std::string     api_put_correct_info_;
        ros::Publisher  pub_put_correct_info_;
        ros::Subscriber sub_put_correct_info_;

    // 物体認識システムで認識した準静的物体の位置情報を取得するAPI
        std::string     api_get_object_location_;
        ros::Publisher  pub_get_object_location_;
        ros::Subscriber sub_get_object_location_;

    // 地図の補正値情報を取得するAPI
        std::string     api_get_correct_info_;
        ros::Publisher  pub_get_correct_info_;
        ros::Subscriber sub_get_correct_info_;

#if DEBUG
    // デバックオプション
        bool debug_print_map_data_;
#endif

    public:
        /**
        * @brief   HttpBridgeクラスのコンストラクタ
        * @details 初期化を行う
        */
        HttpBridge(ros::NodeHandle &node);

        /**
        * @brief   HttpBridgeクラスのデストラクタ
        * @details オブジェクトの破棄を行う
        */
        ~HttpBridge();

        /**
        * @brief        コールバック受信ループ関数
        * @param[in]    void
        * @return       void
        * @details      送信するメッセージのコールバック受信待機関数
        */
        void bridgeLoop();

        /**
        * @brief        送信するメッセージのデータからペイロードデータを作成する関数
        * @param[in]    json_data     送信するJSONデータオブジェクト
        * @param[in]    header      変換対象のメッセージデータ
        * @return       void
        * @details      uoa_poc4_msgs::r_headerメッセージデータをJSON形式に変換する
        */
        void convertHeaderToJson(nlohmann::json& json_data, uoa_poc4_msgs::r_header header);

        /**
        * @brief        受信したレスポンスメッセージのペイロードデータからROSのメッセージ形式に変換する関数
        * @param[in]    header          変換後のメッセージデータ
        * @param[in]    json_data       変換対象のJSONデータオブジェクト
        * @return       void
        * @details      受信したJSON形式のヘッダデータをuoa_poc4_msgs::r_headerメッセージデータに変換する
        */
        void convertJsonToHeader(uoa_poc4_msgs::r_header& header, nlohmann::json json_data );
        
        /**
        * @brief        送信するメッセージのデータからペイロードデータを作成する関数
        * @param[in]    json_data         送信するJSONデータオブジェクト
        * @param[in]    info            変換対象のメッセージデータ
        * @param[in]    location_only   location_infoキーの使用フラグ
        * @return       void
        * @details      uoa_poc4_msgs::r_map_location_infoメッセージデータをJSON形式に変換する
        */
        void convertLocationInfoToJson(nlohmann::json& json_data, uoa_poc4_msgs::r_map_location_info info, bool location_only);
        
        /**
        * @brief        受信したレスポンスメッセージのペイロードデータからROSのメッセージ形式に変換する関数
        * @param[in]    map_data    変換後のメッセージデータ
        * @param[in]    json_data     変換対象のJSONデータオブジェクト
        * @return       void
        * @details      受信したJSON形式のロケーション情報をuoa_poc4_msgs::r_map_location_info形式のメッセージデータに変換する
        */
        void convertJsonToLocationInfo(uoa_poc4_msgs::r_map_location_info& info, nlohmann::json json_data);
       
        /**
        * @brief        送信するメッセージのデータからペイロードデータを作成する関数
        * @param[in]    json_data     送信するJSONデータオブジェクト
        * @param[in]    info        変換対象のメッセージデータ
        * @return       void
        * @details      uoa_poc4_msgs::r_map_system_infoメッセージデータをJSON形式に変換する
        */
        void convertSystemInfoToJson(nlohmann::json& json_data, uoa_poc4_msgs::r_map_system_info info);
        
        /**
        * @brief        受信したレスポンスメッセージのペイロードデータからROSのメッセージ形式に変換する関数
        * @param[in]    info        変換後のメッセージデータ
        * @param[in]    json_data     変換対象のJSONデータオブジェクト
        * @return       void
        * @details      受信したJSON形式のシステム情報をuoa_poc4_msgs::r_map_system_infoメッセージデータに変換する
        */
        void convertJsonToSystemInfo(uoa_poc4_msgs::r_map_system_info& info, nlohmann::json json_data);
                
        /**
        * @brief        送信するメッセージのデータからペイロードデータを作成する関数
        * @param[in]    json_data     送信するJSONデータオブジェクト
        * @param[in]    map_data    変換対象のメッセージデータ
        * @return       void
        * @details      nav_msgs::OccupancyGridメッセージデータをJSON形式に変換する
        */
        void convertMapdataInfoToJson(nlohmann::json& json_data, nav_msgs::OccupancyGrid map_data);
        
        /**
        * @brief        受信したレスポンスメッセージのペイロードデータからROSのメッセージ形式に変換する関数
        * @param[in]    map_data    変換後のメッセージデータ
        * @param[in]    json_data     変換対象のJSONデータオブジェクト
        * @return       void
        * @details      受信したJSON形式の地図データをnav_msgs::OccupancyGrid形式のメッセージデータに変換する
        */
        void convertJsonToMapdataInfo(nav_msgs::OccupancyGrid& map_data, nlohmann::json json_data);
       
        /**
        * @brief        ペイロードをPostで配信する関数
        * @param[in]    responce   送信するJSONデータオブジェクト
        * @param[in]    api_url    変換対象のメッセージデータ
        * @param[in]    payload    変換対象のメッセージデータ
        * @return       bool    reqestの成功可否(成功:True、失敗:False)
        * @details      指定したサーバーに、ペイロードをHTTP::Postメソッドで送信し、レスポンスを受け取る
        */
        bool postData(cpr::Response& response, std::string api_url, nlohmann::json payload);

        /**
        * @brief        リクエストするROSメッセージデータのサブスクライバ
        * @param[in]    robot_info ROSメッセージデータ
        * @return       void
        * @details      配信するロボット情報のメッセージのサブスクライバ
        */
        void reqRobotInfoCB(const uoa_poc4_msgs::r_req_robot_info& put_robot_info_msg);
        
        /**
        * @brief        リクエストするROSメッセージデータのサブスクライバ
        * @param[in]    map_list ROSメッセージデータ
        * @return       void
        * @details      最新10件の地図データリストの取得メッセージのサブスクライバ
        */
        void reqMapdataListCB(const uoa_poc4_msgs::r_req_mapdata_list& get_map_list_msg);

        /**
        * @brief        リクエストするROSメッセージデータのサブスクライバ
        * @param[in]    put_mapdata ROSメッセージデータ
        * @return       void
        * @details      配信する地図データのメッセージのサブスクライバ
        */
        void reqPutMapdataCB(const uoa_poc4_msgs::r_req_put_mapdata& put_mapdata_msg);

        /**
        * @brief        リクエストするROSメッセージデータのサブスクライバ
        * @param[in]    get_map_msg ROSメッセージデータ(地図の取得)
        * @return       void
        * @details      取得する地図データのメッセージのサブスクライバ
        */
        void reqGetMapdataCB(const uoa_poc4_msgs::r_req_get_mapdata& get_map_msg);

        /**
        * @brief        リクエストするROSメッセージデータのサブスクライバ
        * @param[in]    get_map_msg ROSメッセージデータ(レイヤ地図の取得)
        * @return       void
        * @details      取得するレイヤ地図データのメッセージのサブスクライバ
        */
        void reqGetLayerMapdataCB(const uoa_poc4_msgs::r_req_get_mapdata& get_map_msg);

        /**
        * @brief        リクエストするROSメッセージデータのサブスクライバ
        * @param[in]    get_position_msg 位置情報取得のリクエストメッセージ
        * @return       void
        * @details      取得する位置情報のメッセージのサブスクライバ
        */
        void reqGetPositiondataCB(const uoa_poc4_msgs::r_req_get_position_data& get_position_msg);
       
        /**
        * @brief        リクエストするROSメッセージデータのサブスクライバ
        * @param[in]    put_correct_info_msg 地図の補正値配信のリクエストメッセージ
        * @return       void
        * @details      配信する地図の補正値情報のメッセージのサブスクライバ
        */
        void reqPutCorrectInfoCB(const uoa_poc6_msgs::r_req_put_map_pose_correct& put_correct_info_msg);
       
        /**
        * @brief        リクエストするROSメッセージデータのサブスクライバ
        * @param[in]    get_obj_location_msg 位置情報取得のリクエストメッセージ
        * @return       void
        * @details      取得する準静的物体の位置情報のメッセージのサブスクライバ
        */
        void reqGetObjectLocationCB(const uoa_poc6_msgs::r_req_change_obj_loc& get_obj_location_msg);
       
        /**
        * @brief        リクエストするROSメッセージデータのサブスクライバ
        * @param[in]    get_correct_info_msg 地図の補正値取得のリクエストメッセージ
        * @return       void
        * @details      取得する地図の補正値情報のメッセージのサブスクライバ
        */
        void reqGetCorrectInfoCB(const uoa_poc6_msgs::r_req_get_map_pose_correct& get_correct_info_msg);
       
};
