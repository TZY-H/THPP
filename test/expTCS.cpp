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
        std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
    }
}

// map1 {1241,1241}

struct testTCStask
{
    char *name;
    BIpoint anchor;
    double tetherlength;
    BIpoint init;
    int32_t HomotopyIndex;
    std::vector<BIpoint> goals;
};


std::vector<testTCStask> testTasks = {
    {(char *)"./expMap/MESS.png", {676, 761}, 1500, {708, 281}, 2, {{368, 535}, {116, 82}, {129, 1161}, {933, 1193}}},
    {(char *)"./expMap/TRAP.png", {660, 660}, 2300, {660, 958}, 1, {{435, 327}, {306, 1177}, {203, 58}, {1168, 316}}},
    {(char *)"./expMap/GAME.png", {1574, 1574}, 3200, {1236, 1346}, 5, {{365, 1990}, {443, 670}, {1809, 655}, {2108, 1562}}},
    {(char *)"./expMap/MAZE.png", {1239, 1239}, 4000, {1540, 1080}, 1, {{932, 631}, {641, 1915}, {1538, 190}, {2186, 942}}},
};
std::vector<testTCStask> testTasks1 = {
    {(char *)"./expMap/MESS.png", {676, 761}, 1500, {708, 281}, 8, {{368, 535}, {116, 82}, {129, 1161}, {933, 1193}}},
    {(char *)"./expMap/TRAP.png", {660, 660}, 2300, {660, 958}, 4, {{435, 327}, {306, 1177}, {203, 58}, {1168, 316}}},
    {(char *)"./expMap/GAME.png", {1574, 1574}, 3200, {1236, 1346}, 34, {{365, 1990}, {443, 670}, {1809, 655}, {2108, 1562}}},
    {(char *)"./expMap/MAZE.png", {1239, 1239}, 4000, {1540, 1080}, 10, {{932, 631}, {641, 1915}, {1538, 190}, {2186, 942}}},
};

// #define udebug

int main()
{
    BImap cemap;

#ifdef udebug
    // 创建窗口并注册鼠标回调函数
    cv::namedWindow("Video", 0);
    cv::resizeWindow("Video", 1200, 1200);
    cv::setMouseCallback("Video", onMouse);
#endif

    int32_t map_index = 0;
    std::cout << "Input map_index:";
    std::cin >> map_index;
    if (map_index >= testTasks.size())
        return 1;

    testTCStask &testTask = testTasks[map_index];
    cemap.MaptoBInavi(testTask.name, 0.1, 0.9);
#ifdef udebug
    cemap.DrawBImap(true);
    cv::Mat image = cemap.BIdmap.clone();
    drawBIfreeID(cemap.BIgraphList[0], image);
    cv::circle(image, cv::Point((int)(testTask.anchor.x), (int)(testTask.anchor.y)), 5, cv::Scalar(255, 0, 255), -1);
    cv::circle(image, cv::Point((int)(testTask.init.x), (int)(testTask.init.y)), 5, cv::Scalar(255, 0, 0), -1);
    cv::circle(image, cv::Point((int)(testTask.goals[0].x), (int)(testTask.goals[0].y)), 4, cv::Scalar(0, 0, 255), -1);
    cv::circle(image, cv::Point((int)(testTask.goals[1].x), (int)(testTask.goals[1].y)), 4, cv::Scalar(0, 0, 255), -1);
    cv::circle(image, cv::Point((int)(testTask.goals[2].x), (int)(testTask.goals[2].y)), 4, cv::Scalar(0, 0, 255), -1);
    cv::circle(image, cv::Point((int)(testTask.goals[3].x), (int)(testTask.goals[3].y)), 4, cv::Scalar(0, 0, 255), -1);
    cv::imshow("Video", image);
    cv::waitKey();
    int32_t PolyIndex = cemap.BIimap.at<uint16_t>(testTask.init.y, testTask.init.x);
    THPPtask task;
    cemap.THPPtaskInit(task, testTask.anchor, testTask.tetherlength, 0);
    std::cout << "PolyIndex: " << PolyIndex << ", goalEncodingSet: " << task.EncodingSet[PolyIndex] << std::endl;

    if (task.EncodingSet[PolyIndex] != -1)
    {
        for (int32_t i = 0; i <= task.EncodingSet[PolyIndex]; i++)
        {
            std::vector<BIline> cpath;
            cemap.THPPgetCDTencodingCutline(task, (i << 16) | PolyIndex, cpath, testTask.init);
            std::list<BIpoint> Pathtemp;
            double Costtemp;
            image = cemap.BIdmap.clone();
            drawBIfreeID(cemap.BIgraphList[0], image);
            cemap.GetLeastHomotopyPath(cpath, Pathtemp, Costtemp);
            if (Costtemp > task.TetherLength)
                continue;
            drawBIpath(Pathtemp, image);
            std::cout << "i: " << i << " Cost: " << Costtemp << std::endl;
            std::cout << "   1: " << Costtemp + (task.Init % testTask.goals[0]) << std::endl;
            std::cout << "   2: " << Costtemp + (task.Init % testTask.goals[1]) << std::endl;
            std::cout << "   3: " << Costtemp + (task.Init % testTask.goals[2]) << std::endl;
            std::cout << "   4: " << Costtemp + (task.Init % testTask.goals[3]) << std::endl;
            if (cv::waitKey() == 'Q')
                break;
        }
    }
#else
    cemap.DrawBImap(false);
    int32_t testNN = 50;
    int64_t t0 = utime_ns();
    for (int kk = 0; kk < testNN; kk++)
    {
        THPPtask task;
        cemap.THPPtaskInit(task, testTask.anchor, testTask.tetherlength, 0);
    }
    std::cout << " time: " << (utime_ns() - t0) / testNN << std::endl;

    THPPtask task;
    cemap.THPPtaskInit(task, testTask.anchor, testTask.tetherlength, 0);
    std::map<int32_t, std::pair<std::list<BIpoint>, double>>
        ConfigList;
    for (int32_t g = 0; g < 4; g++)
    {
        t0 = utime_ns();
        for (int32_t kk = 0; kk < testNN; kk++)
        {
            cemap.GetAllOptConfigurations(task, testTask.goals[g], ConfigList);
        }
        std::cout << "time" << g << ": " << (utime_ns() - t0) / testNN << std::endl;
        std::cout << "ConfigList.size(): " << ConfigList.size() << std::endl;
        std::cout << "cost: ";
        for (auto &hps : ConfigList)
        {
            std::cout << hps.second.second << ", ";
        }
        std::cout << std::endl;
    }
#endif
    return 0;
}
