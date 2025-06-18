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
    {(char *)"./expMap/MESS.png", {676, 761}, 2000, {708, 281}, 2, {{368, 535}, {116, 82}, {129, 1161}, {933, 1193}}},
    {(char *)"./expMap/TRAP.png", {660, 660}, 3000, {660, 958}, 1, {{435, 327}, {306, 1177}, {203, 58}, {1168, 316}}},
    {(char *)"./expMap/GAME.png", {1574, 1574}, 4000, {1236, 1346}, 5, {{365, 1990}, {443, 670}, {1809, 655}, {2108, 1562}}},
    {(char *)"./expMap/MAZE.png", {1239, 1239}, 5000, {1540, 1080}, 1, {{932, 631}, {641, 1915}, {1538, 190}, {2186, 942}}},
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
    int64_t t0 = utime_ns();
    THPPtask task;
    cemap.THPPtaskInit(task, testTask.anchor, testTask.tetherlength, 1);
    std::cout << " time: " << (utime_ns() - t0) << std::endl;

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
    for (int32_t goal_index = 0; goal_index < 4; goal_index++)
    {
        BIpoint &goal = testTask.goals[goal_index];

        std::list<BIpoint> minpath;
        t0 = utime_ns();
        cemap.UTHPPoptimalPlanner(task, testTask.init, goal, minpath);
        int64_t t1 = utime_ns();
        std::cout << "planning time: " << t1 - t0 << std::endl;

        image = cemap.BIdmap.clone();
        drawBIfreeID(cemap.BIgraphList[0], image);
        drawBIpath(minpath, image);
        cv::waitKey();
    }
#else
    cemap.DrawBImap(false);
    int64_t t0 = utime_ns();
    THPPtask task;
    cemap.THPPtaskInit(task, testTask.anchor, testTask.tetherlength, 1);
    std::cout << "THPPtaskInit time: " << (utime_ns() - t0) << std::endl;
    int32_t PolyIndex = cemap.BIimap.at<uint16_t>(testTask.init.y, testTask.init.x);
    std::cout << "init PolyIndex: " << task.EncodingSet[PolyIndex] << std::endl;

    int32_t testNN = 100;
    for (int32_t goal_index = 0; goal_index < 4; goal_index++)
    {
        BIpoint &goal = testTask.goals[goal_index];

        std::list<BIpoint> minpath;
        double cost;
        t0 = utime_ns();
        for (int32_t kk = 0; kk < testNN; kk++)
            cost = cemap.UTHPPoptimalPlanner(task, testTask.init, goal, minpath);
        int64_t t1 = utime_ns();
        std::cout << " time: " << (t1 - t0) / testNN << std::endl;
        PolyIndex = cemap.BIimap.at<uint16_t>(goal.y, goal.x);
        std::cout << " PolyIndex: " << task.EncodingSet[PolyIndex] << std::endl;
        std::cout << " cost: " << cost << std::endl;
    }

#endif

    return 0;
}
