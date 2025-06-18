#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <random>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "CDTmap.h"

#include <nlohmann/json.hpp>
using json = nlohmann::json;

// 回调函数，当鼠标左键按下时调用
int32_t updata = false;
BIpoint mousePoint = {-1, -1};
void onMouse(int event, int x, int y, int flags, void *userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        updata = true;
        mousePoint.x = x;
        mousePoint.y = y;
        // 在控制台输出鼠标左击位置的坐标
        std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
    }
}

// map1 {1241,1241}

int main()
{
    BImap cemap;
    THPPtask task;

    // 创建窗口并注册鼠标回调函数
    cv::namedWindow("Video", 0);
    cv::resizeWindow("Video", 1200, 1200);
    cv::setMouseCallback("Video", onMouse, &task);
    cemap.MaptoBInavi((char *)"/home/tzyh_subsys/WorkSpace/THPP/expMap/THPP_map1.png", 0.1, 0.9);
    cemap.DrawBImap(true);

    BIpoint Origin = {1241, 1241};
    cv::Mat image = cemap.BIdmap.clone();
    int32_t key = -1;
    int32_t mod = -1;
    int64_t t0 = utime_ns();
    cemap.THPPtaskInit(task, Origin, 3000.0, 0);
    std::cout << "task.EncodingTree.size(): " << task.EncodingTree.size() << " time: " << utime_ns() - t0 << std::endl;

    BIpoint init = {918, 831};
    int32_t HomotopyPolyIndex_Init = 119; // 默认初始配置的系留编码
    std::vector<BIpoint> goals;           // 默认目标点
    goals.push_back({697, 1075});
    goals.push_back({438, 1218});
    goals.push_back({564, 918});
    goals.push_back({566, 713});
    goals.push_back({707, 566});
    std::list<BIpoint> PathInit;
    {
        std::vector<BIline> cpath;
        cemap.THPPgetCDTencodingCutline(task, HomotopyPolyIndex_Init, cpath, init);
        double Costtemp;
        std::list<BIpoint> PathTemp;
        cemap.GetLeastHomotopyPath(cpath, PathTemp, Costtemp);
        std::swap(PathInit, PathTemp);
    }

    key = 0;
    mod = '0';
    while (key != 27)
    {
        image = cemap.BIdmap.clone();
        switch (mod)
        {
        case '0': // 添加目标点
            if (updata)
            {
                updata = false;
                BIpoint goal = mousePoint;
                int32_t PolyIndex = cemap.BIimap.at<uint16_t>(goal.y, goal.x);
                if (PolyIndex == 0xFFFF)
                    break;
                std::cout << "PolyIndex: " << PolyIndex << ", goalEncodingSet: " << task.EncodingSet[PolyIndex] << std::endl;
                goals.push_back(goal);
            }
            break;
        case '1': // 重设锚点
            if (updata)
            {
                updata = false;
                Origin = mousePoint;
                int32_t PolyIndex = cemap.BIimap.at<uint16_t>(Origin.y, Origin.x);
                if (PolyIndex == 0xFFFF)
                    break;
                t0 = utime_ns();
                cemap.THPPtaskInit(task, Origin, 4000.0, 0);
                std::cout << "task.EncodingTree.size(): " << task.EncodingTree.size() << " time: " << utime_ns() - t0 << std::endl;
            }
            break;
        case '2': // 选择所点击位置的成本最小的最优可行配置作为TMV任务的初始配置
            if (updata)
            {
                updata = false;
                init = mousePoint;
                int32_t PolyIndex = cemap.BIimap.at<uint16_t>(init.y, init.x);
                if (PolyIndex & 0xC000)
                    break;
                std::cout << "PolyIndex: " << PolyIndex << ", initEncodingSet: " << task.EncodingSet[PolyIndex] << std::endl;
                if (task.EncodingSet[PolyIndex] != -1)
                {
                    double minCost = doubleMax;
                    for (int32_t i = 0; i <= task.EncodingSet[PolyIndex]; i++)
                    {
                        // std::cout << "i: " << i << std::endl;
                        std::vector<BIline> cpath;
                        cemap.THPPgetCDTencodingCutline(task, (i << 16) | PolyIndex, cpath, init);
                        double Costtemp;
                        std::list<BIpoint> PathTemp;
                        cemap.GetLeastHomotopyPath(cpath, PathTemp, Costtemp);
                        if (minCost > Costtemp)
                        {
                            HomotopyPolyIndex_Init = (i << 16) | PolyIndex;
                            minCost = Costtemp;
                            std::swap(PathInit, PathTemp);
                        }
                    }
                    std::cout << "HomotopyPolyIndex_Init: " << HomotopyPolyIndex_Init << std::endl;
                    drawBIpath(PathInit, image);
                    cv::waitKey(0);
                }
            }
            break;
        }
        if (key == ' ' && init.x >= 0 && Origin.x >= 0 && goals.size()) // 按' '进行TMV规划
        {
            std::list<BIpoint> minpath;
            t0 = utime_ns();
            bool tf = cemap.TMVoptimalPlanner(task, HomotopyPolyIndex_Init, init, goals, minpath);
            std::cout << "TMVoptimalTime: " << utime_ns() - t0 << std::endl;
            if (tf == false)
                std::cout << "TMV Error" << std::endl;
            drawBIpath(PathInit, image, 0, cv::Scalar(255, 0, 0));
            drawBIpath(minpath, image, 15, cv::Scalar(0, 0, 255));
            cv::waitKey(0);
        }

        drawBIfreeID(cemap.BIgraphList[0], image);
        if (Origin.x >= 0)
            cv::circle(image, cv::Point((int)(Origin.x), (int)(Origin.y)),
                       5, cv::Scalar(255, 0, 255), -1);
        if (init.x >= 0)
            cv::circle(image, cv::Point((int)(init.x), (int)(init.y)),
                       5, cv::Scalar(255, 0, 0), -1);
        for (BIpoint goal : goals)
        {
            if (goal.x >= 0)
                cv::circle(image, cv::Point((int)(goal.x), (int)(goal.y)),
                           5, cv::Scalar(0, 0, 255), -1);
        }
        cv::imshow("Video", image);
        key = cv::waitKey(50);
        if ('0' <= key && key <= '9')
            mod = key;
        if (key == 8 && goals.size() != 0) // 退格删除目标点
            goals.pop_back();
    }

    return 0;
}
