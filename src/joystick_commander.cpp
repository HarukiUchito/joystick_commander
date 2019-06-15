/*
 * This program is based on jstest.c version 1.2
 * --------------------------------------------------
 * jstest.c version 1.2
 * Copyright (c) 1996-1999 Vojtech Pavlik
 * Sponsored by SuSE
 */

#include <iostream>

#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#include <linux/input.h>
#include <linux/joystick.h>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include "axbtnmap.h"

// ジョイスティック状態表示
//#define DEBUG

#define NAME_LENGTH 128

#if 0
char *axis_names[ABS_MAX + 1]
= {"X", "Y", "Z", "Rx", "Ry", "Rz", "Throttle",
   "Rudder", "Wheel", "Gas", "Brake", "?", "?", "?", "?", "?", "Hat0X",
   "Hat0Y", "Hat1X", "Hat1Y", "Hat2X", "Hat2Y", "Hat3X", "Hat3Y", "?",
   "?", "?", "?", "?", "?", "?",};

char *button_names[KEY_MAX - BTN_MISC + 1]
= {"Btn0", "Btn1", "Btn2", "Btn3", "Btn4", "Btn5", "Btn6", "Btn7",
   "Btn8", "Btn9", "?", "?", "?", "?", "?", "?", "LeftBtn",
   "RightBtn", "MiddleBtn", "SideBtn", "ExtraBtn", "ForwardBtn",
   "BackBtn", "TaskBtn", "?", "?", "?", "?", "?", "?", "?", "?",
   "Trigger", "ThumbBtn", "ThumbBtn2", "TopBtn", "TopBtn2",
   "PinkieBtn", "BaseBtn", "BaseBtn2", "BaseBtn3", "BaseBtn4",
   "BaseBtn5", "BaseBtn6", "BtnDead", "BtnA", "BtnB", "BtnC",
   "BtnX", "BtnY", "BtnZ", "BtnTL", "BtnTR", "BtnTL2", "BtnTR2",
   "BtnSelect", "BtnStart", "BtnMode", "BtnThumbL", "BtnThumbR",
   "?", "?", "?", "?", "?", "?", "?", "?", "?", "?", "?", "?",
   "?", "?", "?", "?", "?", "WheelBtn", "Gear up",};
#endif

// rad を deg に換算
inline const double rad2deg(const double a_angle_rad) {
    return a_angle_rad / M_PI * 180.0;
}
// deg を rad に換算
inline const double deg2rad(const double a_angle_deg) {
    return a_angle_deg / 180.0 * M_PI;
}

int main(int argc, char **argv) {
    // 引数が正しくない場合は使い方を表示
    /*if (argc > 2 || argc == 1) {
        std::cout << "USAGE:" << std::endl;
        std::cout << "  $ " << argv[0] << " <dev_file_path>" << std::endl;
        std::cout << "  e.g.)  $ " << argv[0] << " /dev/input/js0" << std::endl;
        return -1;
    }*/

    std::string port;
    //port = argv[argc - 1];
    port = "/dev/input/by-id/usb-Sony_PLAYSTATION_R_3_Controller-joystick";

    ros::init(argc, argv, "joystick_commander");
    ros::NodeHandle node;
    ros::NodeHandle private_node_handle("~");
    std::string pub_name;
    private_node_handle.param<std::string>("pub_name", pub_name, "/ypspur_ros/cmd_vel");

    ros::Publisher cmd_publisher = node.advertise<geometry_msgs::Twist>(pub_name, 100);

    // 固定小数点記法に設定
    std::cout << std::fixed;
    std::cerr << std::fixed;

    // 初期速度
    const double init_trans_vel_mps = 0.5;
    const double init_angular_vel_radps = deg2rad(15.0);

    // 最大最小速度
    const double max_limit_trans_vel_mps = 1.1;  // 1.1 m/s = 3.96 km/h
    const double max_limit_angular_vel_radps = deg2rad(45.0);
    const double min_limit_trans_vel_mps = 0.1;
    const double min_limit_angular_vel_radps = deg2rad(5.0);

    // ボタン入力による速度調整幅
    const double limit_step_trans_vel_mps = 0.1;
    const double limit_step_angular_vel_radps = deg2rad(5.0);

    // コマンド送信の最短周期
    const double cmd_send_interval_time_s = 0.01;

    double max_trans_vel_mps = init_trans_vel_mps;
    double max_angular_vel_radps = init_angular_vel_radps;

    double trans_vel_mps, angular_vel_radps;

    try {
        // 入力速度保持用のパラメータ
        double input_trans_vel_ratio = 0.0, input_angular_vel_ratio = 0.0;

        // ジョイスティック関連
        int fd;
        unsigned char num_of_axes = 2;
        unsigned char num_of_buttons = 2;
        int version = 0x000800;
        char name[NAME_LENGTH] = "Unknown";
        uint16_t btnmap[BTNMAP_SIZE];
        uint8_t axmap[AXMAP_SIZE];
        int btnmapok = 1;

        if ((fd = open(port.c_str(), O_RDONLY)) < 0) {
            perror("ERROR: Can't connect to the joystick!! ");
            return -1;
        }

        ioctl(fd, JSIOCGVERSION, &version);
        ioctl(fd, JSIOCGAXES, &num_of_axes);
        ioctl(fd, JSIOCGBUTTONS, &num_of_buttons);
        ioctl(fd, JSIOCGNAME(NAME_LENGTH), name);

        // nonblocking mode
        fcntl(fd, F_SETFL, O_NONBLOCK);

        getaxmap(fd, axmap);
        getbtnmap(fd, btnmap);

        printf("Driver version: %d.%d.%d\n", version >> 16, (version >> 8) & 0xff, version & 0xff);

        /* Determine whether the button map is usable. */
        for (int i = 0; btnmapok && i < num_of_buttons; ++i) {
            if (btnmap[i] < BTN_MISC || btnmap[i] > KEY_MAX) {
                btnmapok = 0;
                break;
            }
        }
        if (!btnmapok) {
            /* btnmap out of range for names. Don't print any. */
            printf("INFO: Not fully compatible with your kernel. Unable to retrieve button map!!");
            printf("Joystick (%s) has %d axes and %d buttons.\n", name, num_of_axes, num_of_buttons);
        } else {
#if 0
            printf("Joystick (%s) has %d axes (", name, num_of_axes);
            for (int i = 0; i < num_of_axes; ++i) {
                printf("%s%s", i > 0 ? ", " : "", axis_names[axmap[i]]);
            }
            puts(")");

            printf("and %d buttons (", num_of_buttons);
            for (int i = 0; i < num_of_buttons; ++i) {
                printf("%s%s", i > 0 ? ", " : "", button_names[btnmap[i] - BTN_MISC]);
            }
            puts(").");
#else
            printf("Joystick (%s) has %d axes and %d buttons.\n", name, num_of_axes, num_of_buttons);
#endif
        }

        // 操作モード（軸の割り当ては型番によって異なる。ボタンの割り当てはBUFFALOとELECOMは同じ。）
        // 1：Logitech Logitech Attack 3 用
        // 2：DragonRise Inc. Generic USB Joystick (BUFFALO) 用
        // 3：USB Gamepad (BUFFALO、無線) 用
        // 4：Sony PLAYSTATION(R)3 Controller 用
        int operation_mode;

        // ジョイスティック名による操作モード変更
        if (strncmp(name, "Logitech Logitech Attack 3", 26) == 0) {
            operation_mode = 1;
        } else if (strncmp(name, "DragonRise Inc.   Generic   USB  Joystick", 41) == 0) {
            operation_mode = 2;
        } else if (strncmp(name, "USB Gamepad", 11) == 0) {
            operation_mode = 3;
        } else if (strncmp(name, "Sony PLAYSTATION(R)3 Controller", 31) == 0) {
            operation_mode = 4;
        } else {
            operation_mode = -1;
            std::cout
                    << "INFO: Unknown joystick. Limited functions are running."
                    << std::endl;
        }

        struct js_event read_event;

        std::vector<int> axis_states(num_of_axes, 0);
        std::vector<char> button_states(num_of_buttons, 0);

        // 処理状態フラグ
        bool is_read_event = false;
        bool is_passed_interval_time = false;
        double last_cmd_sent_time_s = 0.0;
        double start_time_s = ros::Time::now().toSec();

        // インターバル表示用
        int loop_num = 0;

        while(ros::ok()) {
            ros::spinOnce();

            double current_time_s = ros::Time::now().toSec();

            if (read(fd, &read_event, sizeof(read_event)) == sizeof(read_event)) {
                // 入力イベントを読んだ
                is_read_event = true;
            } else {
                // エラー判定
                if (errno != EAGAIN) {
                    perror("ERROR: Read joystick error!! ");
                    return -1;
                }
                // コマンド送信周期まではデータ待ち
                if (current_time_s - last_cmd_sent_time_s < cmd_send_interval_time_s) {
                    usleep(5000);
                    continue;  // Ctrl-C をポーリング
                }
                is_read_event = false;
            }
            if (current_time_s - last_cmd_sent_time_s >= cmd_send_interval_time_s) {
                // コマンド送信周期の時間が経った
                is_passed_interval_time = true;
            } else {
                is_passed_interval_time = false;
            }
            // (入力イベントを読んだ || 読んでないがコマンド送信周期の時間が経った)場合に以下の処理を実行

            if (is_read_event == true) {
                //std::cout << "read event !!" << std::endl;
                for (int i = 0; i < axis_states.size(); ++i) {
                    //std::cout << axis_states[i];
                    //if (i == axis_states.size()-1) std::cout << std::endl;
                    //else std::cout << " ";
                }
                // 送信コマンドのパラメータ設定
                switch (read_event.type & ~JS_EVENT_INIT) {
                case JS_EVENT_AXIS:
                    axis_states[read_event.number] = read_event.value;
                    break;
                case JS_EVENT_BUTTON:
                    button_states[read_event.number] = read_event.value;
                    // イベント時のみパラメータが変更される（ボタンを押し続けても値は変化しない）
                    if (operation_mode == 1) {  // Logitech
                    } else if (operation_mode == 2 || operation_mode == 3) {  // BUFFALO
                    } else if (operation_mode == 4) {  // Sony PLAYSTATION(R)3 Controller
                        if (read_event.number == 10 && button_states[10]) {
                            max_trans_vel_mps += limit_step_trans_vel_mps;
                            if (max_trans_vel_mps > max_limit_trans_vel_mps) {
                                max_trans_vel_mps = max_limit_trans_vel_mps;
                            }
                            std::cout << ">> Max trans vel: " << max_trans_vel_mps << " [m/s]." << std::endl;
                        }
                        if (read_event.number == 8 && button_states[8]) {
                            max_trans_vel_mps -= limit_step_trans_vel_mps;
                            if (max_trans_vel_mps < min_limit_trans_vel_mps) {
                                max_trans_vel_mps = min_limit_trans_vel_mps;
                            }
                            std::cout << ">> Max trans vel: " << max_trans_vel_mps << " [m/s]." << std::endl;
                        }
                        if (read_event.number == 11 && button_states[11]) {
                            max_angular_vel_radps += limit_step_angular_vel_radps;
                            if (max_angular_vel_radps > max_limit_angular_vel_radps) {
                                max_angular_vel_radps = max_limit_angular_vel_radps;
                            }
                            std::cout << ">> Max angular vel: " << rad2deg(max_angular_vel_radps) << " [deg/s]." << std::endl;
                        }
                        if (read_event.number == 9 && button_states[9]) {
                            max_angular_vel_radps -= limit_step_angular_vel_radps;
                            if (max_angular_vel_radps < min_limit_angular_vel_radps) {
                                max_angular_vel_radps = min_limit_angular_vel_radps;
                            }
                            std::cout << ">> Max angular vel: " << rad2deg(max_angular_vel_radps) << " [deg/s]." << std::endl;
                        }
                    }
                    break;
                }

                // 起動後１秒間はジョイスティックの入力を無視する
                // ジョイスティックを傾けた状態で、USBケーブルを抜いてプロセスが落ちると、次回の起動直後にmax値が代入される
                if (ros::Time::now().toSec() - start_time_s < 1.0) {
                    axis_states[0] = 0;
                    axis_states[1] = 0;
                    axis_states[2] = 0;
                    axis_states[3] = 0;
                    axis_states[4] = 0;
                    if (operation_mode == 1) {  // Logitech
                        axis_states[2] = 32767;
                    }
                }

                // 軸入力による走行速度設定
                if (operation_mode == 1) {  // Logitech
                    input_trans_vel_ratio = axis_states[1] / -32767.0;
                    if (axis_states[1] < 5000.0) {  // 前進
                        input_angular_vel_ratio = axis_states[0] / -32767.0;
                    } else if (axis_states[1] > 25000.0) {  // 後進
                        input_angular_vel_ratio = axis_states[0] / 32767.0;
                    } else {  // 不感帯
                        input_angular_vel_ratio = 0.0;
                    }
                    //vもwにも独立にゼロ付近での操作に遊びを設ける
                    if (fabs(input_trans_vel_ratio) < 0.1) {
                        input_trans_vel_ratio = 0;
                    } else {
                        if(input_trans_vel_ratio > 0) {input_trans_vel_ratio = (input_trans_vel_ratio-0.1)/(1.0-0.1);}
                        if(input_trans_vel_ratio < 0) {input_trans_vel_ratio = (input_trans_vel_ratio+0.1)/(1.0-0.1);}
                    }
                    if (fabs(input_angular_vel_ratio) < 0.1) {
                        input_angular_vel_ratio = 0;
                    } else {
                        if (input_angular_vel_ratio > 0) {input_angular_vel_ratio = (input_angular_vel_ratio-0.1)/(1.0-0.1);}
                        if (input_angular_vel_ratio < 0) {input_angular_vel_ratio = (input_angular_vel_ratio+0.1)/(1.0-0.1);}
                    }
                    //制御性をリニアではなく二次関数形状にする
                    input_trans_vel_ratio = input_trans_vel_ratio * fabs(input_trans_vel_ratio);
                    input_angular_vel_ratio = input_angular_vel_ratio * fabs(input_angular_vel_ratio);
                } else if (operation_mode == 2) {  // BUFFALO
                    input_trans_vel_ratio = axis_states[1] / -32767.0;
                    input_angular_vel_ratio = axis_states[3] / -32767.0;
                } else if (operation_mode == 3) {  // BUFFALO、無線
                    input_trans_vel_ratio = axis_states[1] / -32767.0;
                    input_angular_vel_ratio = axis_states[4] / -32767.0;
                } else if (operation_mode == 4) {  // Sony PLAYSTATION(R)3 Controller
                    input_trans_vel_ratio = axis_states[1] / -32767.0;
                    input_angular_vel_ratio = axis_states[3] / -32767.0;
/*                    input_trans_vel_ratio = axis_states[1] / -32767.0;
                    input_angular_vel_ratio = axis_states[2] / -32767.0;
*/
                }

#ifdef DEBUG
                // ジョイスティック状態表示
                if (num_of_axes) {
                    printf("Axes: ");
                    for (int i = 0; i < num_of_axes; ++i) {
                        printf("%2d:%6d ", i, axis_states[i]);
                    }
                }
                printf("\n");
                if (num_of_buttons) {
                    printf("Buttons: ");
                    for (int i = 0; i < num_of_buttons; ++i) {
                        printf("%2d:%s ", i, button_states[i] ? "on " : "off");
                    }
                }
                printf("\n");
#endif
            }

            // インターバル表示用
            if (is_passed_interval_time == true) {
                ++loop_num;
            }

            // コマンド送信の判定
            // (送信周期の時間経過)の場合にコマンド送信
            // その他のイベントは送信周期の時間が経過していなければコマンド送信しない
            if (is_passed_interval_time == true) {
                // 速度の計算
                trans_vel_mps = input_trans_vel_ratio * max_trans_vel_mps;
                angular_vel_radps = input_angular_vel_ratio * max_angular_vel_radps;
                // 後進は速度を遅くする
                if (trans_vel_mps < 0) {
                    trans_vel_mps *= 0.3;
                    angular_vel_radps *= 0.5;
                }
                // 速度コマンド送信
                geometry_msgs::Twist cmd_vel_msg;
                cmd_vel_msg.linear.x = trans_vel_mps;
                cmd_vel_msg.linear.y = 0.0;
                cmd_vel_msg.linear.z = 0.0;
                cmd_vel_msg.angular.x = 0.0;
                cmd_vel_msg.angular.y = 0.0;
                cmd_vel_msg.angular.z = angular_vel_radps;
                cmd_publisher.publish(cmd_vel_msg);

                if (loop_num >= 50) {
                    loop_num = 0;
                    std::cout << "vel: " << trans_vel_mps << " [m/s], "
                              << rad2deg(angular_vel_radps) << " [deg/s]" << std::endl;
                }

                // 時刻を記録
                last_cmd_sent_time_s = current_time_s;
            }

            // 処理状態フラグをリセット
            is_read_event = false;
            is_passed_interval_time = false;
        }
    } catch (std::exception& e) {  // 例外
        std::cerr << "ERROR: Exception : " << e.what() << std::endl;
    } catch (...) {  // 例外
        std::cerr << "ERROR: Unknown exception!!" << std::endl;
    }

    return 0;
}
