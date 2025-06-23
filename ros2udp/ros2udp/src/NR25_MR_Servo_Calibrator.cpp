/*
RRST-NHK-Project 2025
サーボのキャリブレーション用
使い方
NR25_Parameter_Tuner.cppを参照
*/

#include <atomic>
#include <chrono>
#include <fstream>
#include <iostream>
#include <mutex>
#include <nlohmann/json.hpp> //C++でJSON読み取り
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <string>
#include <thread>
#include <vector>

using json = nlohmann::json;

// ソースコードのあるディレクトリを取得
const std::string SOURCE_DIR = std::filesystem::path(__FILE__).parent_path().string();

// 保存先ディレクトリ
const std::string BASE_DIR = SOURCE_DIR + "/config";

// ファイルのフルパス
const std::string PARAM_FILE = BASE_DIR + "/mr_servo_cal.json";
const std::string CSV_FILE = BASE_DIR + "/mr_servo_cal.csv";

class ParameterNode : public rclcpp::Node {
public:
    ParameterNode() : Node("mr_servo_cal"), shoot_state(0), dribble_state(0) {
        show_usage(); // 起動時に使い方を表示

        // パラメータファイル読み込み
        load_parameters();
        publisher = this->create_publisher<std_msgs::msg::Int32MultiArray>(
            "mr_servo_cal", 10);

        running = true;
        publish_thread = std::thread(&ParameterNode::publish_parameters, this);
        input_thread = std::thread(&ParameterNode::handle_user_input, this);
    }

    ~ParameterNode() {
        // セーブ
        save_parameters();
        save_logs();
        running = false;
        if (publish_thread.joinable())
            publish_thread.join();
        if (input_thread.joinable())
            input_thread.join();
    }

private:
    std::vector<int> params;
    std::mutex param_mutex;
    std::atomic<int> shoot_state;
    std::atomic<int> dribble_state;
    std::atomic<bool> running;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher;
    std::thread publish_thread;
    std::thread input_thread;

    // 使い方を表示する関数
    void show_usage() {
        std::cout << "\n=== MR Servo Calibrator ===\n";
        std::cout << "サーボ独ステのキャリブレーション\n";
        std::cout << "使用方法:\n";
        std::cout << "  - <index> <value>: 指定したインデックスのキャリブレーション値を変更 (0-3, -135-135)\n";
        std::cout << "  - show: 現在のパラメータを表示\n";
        std::cout << "=====================================\n";
    }

    void publish_parameters() {
        while (running) {
            {
                std::lock_guard<std::mutex> lock(
                    param_mutex); // shoot_state と dribble_state も含めてロック
                std_msgs::msg::Int32MultiArray msg;
                msg.data = {params[0], params[1], params[2],
                            params[3], shoot_state.load(), dribble_state.load()};
                publisher->publish(msg);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    void save_parameters() {
        json json;
        json["params"] = params;
        std::ofstream file(PARAM_FILE);
        if (file.is_open()) {
            file << json.dump(4);
            std::cout << "パラメータを保存しました。\n";
        } else {

            std::cout << "パラメータを保存に失敗しました。\n";
        }
    }

    void load_parameters() {
        std::ifstream file(PARAM_FILE);
        if (file.is_open()) {
            json json;
            file >> json;
            params = json.value("params", std::vector<int>{0, 0, 0, 0});
            std::cout << "パラメータファイルのロードに成功。\n";
        } else {
            params = {0, 0, 0, 0};
            std::cout
                << "パラメータファイルのロードに失敗。デフォルト値を適用します。\n";
        }
    }

    void save_logs() {
        std::ofstream file(CSV_FILE, std::ios::app);
        if (file.is_open()) {
            auto now = std::chrono::system_clock::now();
            auto time_t_now = std::chrono::system_clock::to_time_t(now);
            file << std::ctime(&time_t_now) << params[0] << "," << params[1] << ","
                 << params[2] << "," << params[3] << "," << shoot_state.load() << ","
                 << dribble_state.load() << "\n";
            file.close();
            std::cout << "ログを保存しました\n";
        } else {
            std::cout << "ログの保存に失敗。\n";
        }
    }

    void handle_user_input() {
        while (running) {
            std::string input;
            std::getline(std::cin, input);
            if (input.empty())
                continue;

            std::lock_guard<std::mutex> lock(param_mutex);

            if (input == "s") {
                shoot_state = 1;
                show_parameters(); // 状態変更後に表示
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(500)); // 0.5秒維持
                // shoot_state = 0;
                show_parameters(); // 状態リセット後に表示
            } else if (input == "d") {
                dribble_state = 1;
                show_parameters();
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                // dribble_state = 0;
                show_parameters();
            } else if (input == "show") {
                show_parameters();
            } else {
                int idx, value;
                if (sscanf(input.c_str(), "%d %d", &idx, &value) == 2 && idx >= 0 &&
                    idx < 4 && value >= -135 && value <= 135) {
                    params[idx] = value;
                    show_parameters();
                } else {
                    std::cout << "Invalid input. Use: <index> <value> (0-3, 0-100), 's' "
                                 "(shoot), 'd' (dribble), or 'show'\n";
                }
            }
        }
    }

    void show_parameters() {
        std::cout << "\n=== Current Parameters ===\n";
        std::cout << "0: SERVO1_CAL = " << params[0] << "\n";
        std::cout << "1: SERVO2_CAL = " << params[1] << "\n";
        std::cout << "2: SERVO3_CAL = " << params[2] << "\n";
        std::cout << "3: SERVO4_CAL = " << params[3] << "\n";
        std::cout << "==========================\n";
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ParameterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}