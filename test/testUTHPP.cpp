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

int main()
{
    BImap cemap;
    THPPtask task;

    // 创建窗口并注册鼠标回调函数
    cv::namedWindow("Video", 0);
    cv::resizeWindow("Video", 1200, 1200);
    cv::setMouseCallback("Video", onMouse, &task);
    cemap.MaptoBInavi((char *)"./expMap/THPP_map1.png", 0.1, 0.9);
    cemap.DrawBImap(true);

    BIpoint Origin = {1241, 1241};
    cv::Mat image = cemap.BIdmap.clone();
    int32_t key = -1;
    int32_t mod = -1;
    int64_t t0 = utime_ns();
    cemap.THPPtaskInit(task, Origin, 4000.0, 1);
    std::cout << "task.EncodingTree.size(): " << task.EncodingTree.size() << " time: " << utime_ns() - t0 << std::endl;

    BIpoint init = {-1, -1}, goal = {-1, -1};
    key = 0;
    mod = '0';
    while (key != 27)
    {
        switch (mod)
        {
        case '0': // 设置起点位置
            if (updata)
            {
                updata = false;
                goal = mousePoint;
                int32_t PolyIndex = cemap.BIimap.at<uint16_t>(goal.y, goal.x);
                if (PolyIndex == 0xFFFF)
                    break;
                std::cout << "PolyIndex: " << PolyIndex << ", goalEncodingSet: " << task.EncodingSet[PolyIndex] << std::endl;
                if (init.x >= 0 && goal.x >= 0)
                {
                    std::list<BIpoint> minpath;
                    t0 = utime_ns();
                    cemap.UTHPPoptimalPlanner(task, init, goal, minpath);
                    int64_t t1 = utime_ns();
                    std::cout << "planning time: " << t1 - t0 << std::endl; //注意由于opencv的窗口显示中断，此时记录的planning time并不精准
                    image = cemap.BIdmap.clone();
                    drawBIfreeID(cemap.BIgraphList[0], image);
                    drawBIpath(minpath, image);
                    if (cv::waitKey(10) == 'Q')
                        break;
                }
            }
            break;
        case '1': // 设置终点位置
            if (updata)
            {
                updata = false;
                init = mousePoint;
                int32_t PolyIndex = cemap.BIimap.at<uint16_t>(init.y, init.x);
                if (PolyIndex == 0xFFFF)
                    break;
                std::cout << "PolyIndex: " << PolyIndex << ", goalEncodingSet: " << task.EncodingSet[PolyIndex] << std::endl;
                if (init.x >= 0 && goal.x >= 0)
                {
                    std::list<BIpoint> minpath;
                    t0 = utime_ns();
                    cemap.UTHPPoptimalPlanner(task, init, goal, minpath);
                    int64_t t1 = utime_ns();
                    std::cout << "planning time: " << t1 - t0 << std::endl;
                    image = cemap.BIdmap.clone();
                    drawBIfreeID(cemap.BIgraphList[0], image);
                    drawBIpath(minpath, image);
                    if (cv::waitKey(10) == 'Q')
                        break;
                }
            }
            break;
        }
        // image = cemap.BIdmap.clone();
        drawBIfreeID(cemap.BIgraphList[0], image);
        if (Origin.x >= 0)
            cv::circle(image, cv::Point((int)(Origin.x), (int)(Origin.y)),
                       5, cv::Scalar(255, 0, 255), -1);
        if (init.x >= 0)
            cv::circle(image, cv::Point((int)(init.x), (int)(init.y)),
                       5, cv::Scalar(255, 0, 0), -1);
        if (goal.x >= 0)
            cv::circle(image, cv::Point((int)(goal.x), (int)(goal.y)),
                       5, cv::Scalar(0, 0, 255), -1);
        cv::imshow("Video", image);
        key = cv::waitKey(50);
        if ('0' <= key && key <= '9')
            mod = key;
    }

    // cv::waitKey(0);
    return 0;
}
