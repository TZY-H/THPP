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
    while (key != 27) // 左点击设置锚点, Esc进行下一步
    {
        if (updata)
        {
            Origin = mousePoint;
            std::cout << "PolyIndex: " << cemap.BIimap.at<uint16_t>(Origin.y, Origin.x) << std::endl;
            updata = false;
        }
        image = cemap.BIdmap.clone();
        drawBIfreeID(cemap.BIgraphList[0], image);
        if (Origin.x >= 0)
            cv::circle(image, cv::Point((int)(Origin.x), (int)(Origin.y)),
                       5, cv::Scalar(255, 0, 255), -1);
        cv::imshow("Video", image);
        key = cv::waitKey(50);
        if ('0' <= key && key <= '9')
            mod = key;
    }
    int64_t t0 = utime_ns();
    cemap.THPPtaskInit(task, Origin, 3000.0, 0);
    std::cout << "task.EncodingTree.size(): " << task.EncodingTree.size() << " time: " << utime_ns() - t0 << std::endl;

    BIpoint goal;
    key = 0;
    mod = '0';
    while (key != 27)
    {
        switch (mod)
        {
        case '0'://按0 左点击绘制该位置的所有最优可行配置, Q退出
            if (updata)
            {
                goal = mousePoint;
                int32_t PolyIndex = cemap.BIimap.at<uint16_t>(goal.y, goal.x);
                std::cout << "PolyIndex: " << PolyIndex << ", goalEncodingSet: " << task.EncodingSet[PolyIndex] << std::endl;
                updata = false;
                if (task.EncodingSet[PolyIndex] != -1)
                {
                    for (int32_t i = 0; i <= task.EncodingSet[PolyIndex]; i++)
                    {
                        std::cout << "i: " << task.EncodingSet[PolyIndex] - i << std::endl;
                        std::vector<BIline> cpath;
                        cemap.THPPgetCDTencodingCutline(task, (i << 16) | PolyIndex, cpath, goal);
                        std::list<BIpoint> Pathtemp;
                        double Costtemp;
                        image = cemap.BIdmap.clone();
                        drawBIfreeID(cemap.BIgraphList[0], image);
                        cemap.GetLeastHomotopyPath(cpath, Pathtemp, Costtemp);
                        drawBIpath(Pathtemp, image);
                        if (cv::waitKey(0) == 'Q')
                            break;
                    }
                    // cv::waitKey(0);
                }
            }
            break;
        case '1'://按0 左点击绘制从初始配置到该位置的最优路径, 当前程序默认初始配置为原点 Esc退出
            if (updata)
            {
                goal = mousePoint;
                int32_t PolyIndex = cemap.BIimap.at<uint16_t>(goal.y, goal.x);
                std::cout << "PolyIndex: " << PolyIndex << ", goalEncodingSet: " << task.EncodingSet[PolyIndex] << std::endl;
                updata = false;
                task.Goal = goal;
                if (cemap.THPPoptimalPlanner(task))
                {
                    image = cemap.BIdmap.clone();
                    std::vector<BIline> cpath;
                    cemap.THPPgetCDTencodingCutline(task, task.HomotopyPolyIndex_Init, cpath, task.Init);
                    double PathMinCost;
                    std::list<BIpoint> PathTemp;
                    cemap.GetLeastHomotopyPath(cpath, PathTemp, PathMinCost);
                    drawBIpath(task.minPath, image, 15, cv::Scalar(0, 0, 255));
                    drawBIpath(PathTemp, image, 15, cv::Scalar(0, 255, 0));
                }
            }
            break;
        }
        drawBIfreeID(cemap.BIgraphList[0], image);
        if (Origin.x >= 0)
            cv::circle(image, cv::Point((int)(Origin.x), (int)(Origin.y)),
                       5, cv::Scalar(255, 0, 255), -1);
        cv::imshow("Video", image);
        key = cv::waitKey(50);
        if ('0' <= key && key <= '9')
            mod = key;
    }

    return 0;
}
