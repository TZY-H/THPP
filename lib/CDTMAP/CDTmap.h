#ifndef __CHIsso_H
#define __CHIsso_H
#include <iostream>
#include <stdio.h>
#include <vector>
#include <queue>
#include <list>
#include <set>
#include <map>
#include <unordered_set>
#include <unordered_map>
#include <thread>
#include <random>
#include <functional>
// #include <zip.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// #include "CSubdivision.h"
#include <nlohmann/json.hpp>
using json = nlohmann::json;
class BIbridge;
class BIisland;

struct BIpoint
{
    double x;
    double y;
    bool operator==(const BIpoint &other) const
    {
        return std::abs(x - other.x) <= 1e-16 &&
               std::abs(y - other.y) <= 1e-16;
    }
    bool operator!=(const BIpoint &other) const
    {
        return std::abs(x - other.x) > 1e-16 ||
               std::abs(y - other.y) > 1e-16;
    }
    bool operator<(const BIpoint &other) const
    {
        // 在这里定义比较规则
        if (x != other.x)
        {
            return x < other.x;
        }
        return y < other.y;
    }
    BIpoint operator+(const BIpoint &other) const
    {
        return {x + other.x, y + other.y};
    }
    BIpoint operator-(const BIpoint &other) const
    {
        return {x - other.x, y - other.y};
    }
    BIpoint operator*(const double &ratio) const
    {
        return {ratio * x, ratio * y};
    }
    BIpoint operator/(const double &ratio) const
    {
        return {x / ratio, y / ratio};
    }
    double operator%(const BIpoint &other) const
    {
        double dx = x - other.x;
        double dy = y - other.y;
        return sqrt(dx * dx + dy * dy);
    }
    // 叉乘
    double cross(const BIpoint &other) const
    {
        return x * other.y - y * other.x;
    }
    // 从json到BIpoint的转换函数
    void from_json(const json &j)
    {
        x = j.at(0).get<double>();
        y = j.at(1).get<double>();
    }
};

typedef struct
{
    double cost;
    BIpoint point;
    int32_t par; // 父节点
    // int32_t sub;    //子节点
    int32_t bridge; // 所属分割线
    int32_t index;
} Node;

struct BIangle
{
    BIpoint O, S, E;
    bool Concave;
    bool operator<(const BIangle &other) const
    {
        if (O < other.O)
            return true;
        if (O != other.O)
            return false;
        if (S < other.S)
            return true;
        if (S != other.S)
            return false;
        return E < other.E;
    }
    bool operator==(const BIangle &other) const
    {
        return (O == other.O) && (S == other.S) && (E == other.E);
    }
};
typedef std::vector<BIpoint> BIpolygon;
typedef std::vector<BIpolygon> BIpolygons;

struct BIline
{
    BIpoint S;
    BIpoint E;
    // 构造函数，自动排序端点
    BIline(BIpoint p1, BIpoint p2)
    {
        if (p2 < p1)
        {
            S = p2;
            E = p1;
        }
        else
        {
            S = p1;
            E = p2;
        }
    }
    bool operator<(const BIline &other) const
    {
        if (S < other.S)
            return true;
        if (S != other.S)
            return false;
        return E < other.E;
    }
    bool operator==(const BIline &other) const
    {
        return (S == other.S) && (E == other.E);
    }
};

struct BIcutline
{
    int32_t index;
    BIline line;
    BIpoint core;
    std::set<int32_t> polygonlink; // polygon index set
    std::set<int32_t> cutlinelink; // cutline index set
};

struct BIobspolygon
{
    int32_t index;
    BIpolygon polygon;
    std::map<int32_t, int32_t> polygonRmap; // 周边的多边形索引集合
    std::vector<int32_t> polygonRlist; // 周边的多边形索引有序列表
};

struct BIfreepolygon
{
    int32_t index;
    BIpolygon polygon;
    BIpoint core;
    std::set<int32_t> polygonlink; // polygon index set
    std::set<int32_t> cutlinelink; // cutline index set
};

// struct BIgoalpoint
// {
//     int32_t index;
//     // BIpolygon polygon;
//     BIpoint core;
//     int32_t polygonlink; // polygon index set
// };

struct BIinvnode
{
    int32_t polyS; // 源多边形引索，BIfreepolygon
    int32_t polyE; // 汇多边形引索，BIfreepolygon
    // int32_t index; // 同伦不变节点引索，到分割线(BIcutline)引索：((int)(index/2))
    bool operator<(const BIinvnode &other) const
    {
        if (polyS < other.polyS)
            return true;
        if (polyS != other.polyS)
            return false;
        return polyE < other.polyE;
    }
    bool operator==(const BIinvnode &other) const
    {
        return (polyS == other.polyS) && (polyE == other.polyE);
    }
};

struct BIgraph
{
    int32_t cutlineBaseNum;
    int32_t freepolygonBaseNum;
    std::vector<BIcutline> cutlineList;
    std::vector<BIobspolygon> obspolygonList;
    std::vector<BIfreepolygon> freepolygonList;
    std::map<BIinvnode, int32_t> invnode2cutlineMap;
};

// template <typename T>
// class RandomContainer
// {
// public:
//     RandomContainer() : distrib(0, 0xFFFF), gen(rd()) {} // 使用预设的较大范围

//     void insert(T value) // 添加元素
//     {
//         if (index_map.find(value) == index_map.end())
//         {
//             elements.push_back(value);
//             index_map[value] = elements.size() - 1;
//         }
//     }

//     void remove(T value) // 删除元素
//     {
//         auto it = index_map.find(value);
//         if (it != index_map.end())
//         {
//             // 将最后一个元素移动到被删除元素的位置
//             T lastElement = elements.back();
//             elements[it->second] = lastElement;
//             index_map[lastElement] = it->second;

//             // 删除最后一个元素
//             // std::cout << "elements.size0:" << elements.size() << std::endl;
//             elements.pop_back();
//             // std::cout << "elements.size0:" << elements.size() << std::endl;
//             index_map.erase(value);
//         }
//     }

//     T getRandom() // 随机选取元素
//     {
//         if (elements.empty())
//             throw std::runtime_error("Cannot get random element from an empty container.");
//         // distrib.param(std::uniform_int_distribution<>::param_type(0, elements.size() - 1));
//         // int randomIndex = distrib(gen);
//         int randomIndex = distrib(gen) % elements.size();
//         return elements[randomIndex];
//     }

//     T popRandom() // 随机弹出元素
//     {
//         T outdata = getRandom();
//         remove(outdata);
//         return outdata;
//     }

//     void clear()
//     {
//         index_map.clear();
//         elements.clear();
//     }

//     size_t size() const
//     {
//         return elements.size();
//     }

// private:
//     std::vector<T> elements;                 // 存储元素
//     std::unordered_map<T, int> index_map;    // 存储元素的索引
//     std::random_device rd;                   // 随机数生成器
//     std::mt19937 gen;                        // Mersenne Twister 随机数生成器
//     std::uniform_int_distribution<> distrib; // 均匀分布
// };

// class RandomRingContainer
// {
// public:
//     size_t insert(const std::vector<int32_t> &ring) // 添加元素
//     {
//         size_t hashValue = hasher(ring);
//         hashRC.insert(hashValue);
//         ringMap[hashValue] = ring;
//         return hashValue;
//     }

//     void remove(size_t hashValue) // 删除元素
//     {
//         hashRC.remove(hashValue);
//         ringMap.erase(hashValue);
//     }

//     size_t getRandomIndex() // 随机选取元素的索引
//     {
//         if (ringMap.empty())
//             throw std::runtime_error("Cannot get random element from an empty container.");
//         size_t hashValue = hashRC.getRandom();
//         return hashValue;
//     }

//     size_t getRandom(std::vector<int32_t> &ring) // 随机选取元素的索引
//     {
//         if (ringMap.empty())
//             throw std::runtime_error("Cannot get random element from an empty container.");
//         size_t hashValue = hashRC.getRandom();
//         ring = ringMap[hashValue];
//         return hashValue;
//     }
//     size_t popRandom(std::vector<int32_t> &ring) // 随机弹出元素, 不建议大量使用
//     {
//         size_t hashValue = getRandom(ring);
//         remove(hashValue);
//         return hashValue;
//     }
//     void clear()
//     {
//         hashRC.clear();
//         ringMap.clear();
//     }

//     size_t size() const
//     {
//         return ringMap.size();
//     }

//     std::unordered_map<size_t, std::vector<int32_t>> ringMap;

// private:
//     RandomContainer<size_t> hashRC;
//     static size_t hasher(const std::vector<int32_t> &ring) // 同伦不变量哈希计算器
//     {
//         size_t seed = ring.size();
//         for (int32_t i : ring)
//         {
//             seed ^= i + 0x9e3779b9 + (seed << 6) + (seed >> 2);
//         }
//         return seed;
//     }
// };

// class BIinformed
// {
// public:
//     void Initialize(std::vector<BIpoint> &points);
//     double informedFun(BIpoint point);
//     double informedFun(BIline line, double threshold = 3);

//     BIpolygon _polygon;

// private:
//     BIpoint _core;
//     std::map<double, int32_t> _polarIndex;
//     double _baseperimeter;
//     // 用于比较两个点的函数，首先按照y坐标比较，如果相同则按照x坐标比较
//     static bool comparePoint(const BIpoint &p1, const BIpoint &p2);
//     // 计算叉乘
//     static double crossProduct(const BIpoint &O, const BIpoint &A, const BIpoint &B);
//     // 计算两点之间的距离
//     static double distance(const BIpoint &p1, const BIpoint &p2);
//     // 用于比较极角的函数
//     static bool comparePolar(const BIpoint &base, const BIpoint &p1, const BIpoint &p2);
//     // Graham扫描算法计算凸包
//     static double grahamScan(std::vector<BIpoint> &points, std::vector<BIpoint> &hull);
// };

// // 将origin、targetObs、targetPoint进行多边形引索的抽象(or 分割线引索的抽象)
// #define originIndexFun() ((int32_t)(0x40000000))
// #define obsNodeIndexFun(x) ((int32_t)(0x80000000 | x))
// #define pointNodeIndexFun(x) ((int32_t)(0xC0000000 | x))
// struct SSOtask
// {
//     BIpoint Origin;          // 起始点
//     int32_t OriginPolyIndex; // 起始poly引索
//     int32_t graphIndex;      // 图引索

//     std::vector<int32_t> targetObsList;       // SSO缠绕目标
//     std::vector<BIgoalpoint> targetPointList; // TSP访问目标
//     std::set<int32_t> targetobjSet;           // 目标对象的多边形抽象索引集合

//     // 目标环路容器，储存各目标已搜索到的环路
//     std::unordered_map<int32_t, RandomRingContainer> targetRingContainer;
//     // // 目标环路容器，储存各目标已搜索到的环路
//     // std::unordered_map<int32_t, RandomRingContainer> targetRingContainer;
//     // // 穿过各有效多边形下环路的哈希集，用于知情集缩小时清除无效的环路 //键int32_t需修改为BIline
//     // std::unordered_map<BIinvnode, std::set<std::pair<int32_t, size_t>>>
//     //     validPolyRingHashSet; // 弃用，每次知情集缩小后重搜索目标配置

//     // 有效的多边形引索：链接的有效多边形引索；
//     std::map<int32_t, std::set<int32_t>> validPolyIndexMap;
//     // 有效的不变量节点优先序列
//     std::set<std::pair<double, BIinvnode>> InvnodePriorityQueue;

//     BIinformed informed; // 知情集控制器

//     std::unordered_set<size_t> HomotopicPathHashSet; // 同伦不变量哈希集

//     double optCost; // 当前解最优代价
//     std::list<BIpoint> optPath;
//     int32_t itNumber = 1;
//     int64_t workTime = 0;

//     static size_t hasher(const std::vector<int> &path) // 同伦不变量哈希计算器
//     {
//         size_t seed = path.size();
//         for (int i : path)
//         {
//             seed ^= i + 0x9e3779b9 + (seed << 6) + (seed >> 2);
//         }
//         return seed;
//     }
//     void NarrowTheInformedSpace(void)
//     {
//         // 确定所有知情函数值大于optCost的不变节点
//         auto upper = InvnodePriorityQueue.upper_bound({optCost, {INT32_MIN, INT32_MIN}});
//         for (auto it = upper; it != InvnodePriorityQueue.end(); ++it)
//         {
//             const BIinvnode &invnode = it->second;
//             if ((validPolyIndexMap.count(invnode.polyS)) &&
//                 (validPolyIndexMap.count(invnode.polyE)))
//                 continue;

//             // 清除invnode在其两端多边形的链接，并尝试清除单连通节点
//             int32_t polyIndex, polyIndexOld;
//             polyIndex = invnode.polyS;
//             polyIndexOld = invnode.polyE;
//             for (;;)
//             {
//                 validPolyIndexMap[polyIndex].erase(polyIndexOld);
//                 int32_t polyIndexSize = validPolyIndexMap[polyIndex].size();
//                 if (polyIndexSize == 0)
//                 {
//                     validPolyIndexMap.erase(polyIndex);
//                     break;
//                 }
//                 else if (polyIndexSize == 1)
//                 {
//                     polyIndexOld = polyIndex;
//                     polyIndex = *validPolyIndexMap[polyIndex].begin();
//                     validPolyIndexMap.erase(polyIndexOld);
//                 }
//                 else
//                     break;
//             }
//             polyIndex = invnode.polyE;
//             polyIndexOld = invnode.polyS;
//             for (;;)
//             {
//                 validPolyIndexMap[polyIndex].erase(polyIndexOld);
//                 int32_t polyIndexSize = validPolyIndexMap[polyIndex].size();
//                 if (polyIndexSize == 0)
//                 {
//                     validPolyIndexMap.erase(polyIndex);
//                     break;
//                 }
//                 else if (polyIndexSize == 1)
//                 {
//                     polyIndexOld = polyIndex;
//                     polyIndex = *validPolyIndexMap[polyIndex].begin();
//                     validPolyIndexMap.erase(polyIndexOld);
//                 }
//                 else
//                     break;
//             }
//             // 清除旧环路
//             targetRingContainer.clear();
//         }
//         InvnodePriorityQueue.erase(upper, InvnodePriorityQueue.end());
//     }
// };

struct THPPtask
{
    BIpoint Origin;          // 起始点
    int32_t OriginPolyIndex; // 起始poly引索
    int32_t graphIndex;      // 图引索
    double TetherLength = 0;

    int8_t mod = 0; // 0:获取系留机器人全可行同伦类、1：系留机器人无冗余环路可行同伦类

    std::vector<int32_t> EncodingSet;        // PolyIndex, HomotopyID (The number of HomotopyPolyIndex)
                                             // HomotopyPolyIndex: HomotopyID<<16 | PolyIndex
    std::map<int32_t, int32_t> EncodingTree; // HomotopyPolyIndex, parentHomotopyPolyIndex

    int32_t HomotopyPolyIndex_Init;
    BIpoint Init;
    BIpoint Goal;
    std::list<BIpoint> minPath;
};

// struct ONCEInvIndex
// {
//     int32_t Encoding;
//     double minCost;
//     double maxCost;
//     bool operator<(const ONCEInvIndex &other) const
//     {
//         if (minCost != other.minCost)
//             return minCost < other.minCost;
//         if (maxCost != other.maxCost)
//             return maxCost < other.maxCost;
//         return Encoding < other.Encoding;
//     }
// };

// struct ONCEsubCutline
// {
//     BIpoint core;
//     double DiscreteLength_2;

//     double MaxCost;                     // min_{EncodingSet}(Encoding.maxCost)
//     std::set<ONCEInvIndex> EncodingSet; // any Encoding.minCost <= maxCost
//     // std::set<int32_t> ExistedEncodingSet; //对于该subCL存在过的编码，避免重复计算
// };

// struct ONCEtask
// {
//     BIpoint Init;          // 起始点
//     int32_t InitPolyIndex; // 起始poly引索
//     int32_t graphIndex;    // 图引索

//     double DiscreteLength;

//     std::vector<std::set<int32_t>> PolyEncodingSet;

//     std::vector<std::vector<ONCEsubCutline>> CL2SubCL; // Cutline to subCutline

//     std::vector<int32_t> EncodingSet;                         // PolyIndex, HomotopyID (The number of HomotopyPolyIndex)
//                                                               // HomotopyPolyIndex: HomotopyID<<16 | PolyIndex
//     std::map<int32_t, int32_t> EncodingTree;                  // HomotopyPolyIndex, parentHomotopyPolyIndex, subHomotopyPolyIndex
//     std::map<int32_t, std::pair<double, int32_t>> EPointTree; // EncodingPointTree
// };

// typedef struct
// {
//     int32_t par;                   // 父节点
//     RandomContainer<int32_t> pars; // 潜在父节点
// } NodeType1;                       // 用于涉及随机游走搜索过程的构造树
// typedef struct
// {
//     int32_t par;            // 父节点
//     std::set<int32_t> subs; // 子节点集
// } NodeType2;                // 用于一般树结构的构造

#define BIdebug 1
class BImap
{
private:
public:

    uint32_t shapeX;  // 地图x轴像素数
    uint32_t shapeY;  // 地图y轴像素数
    double mapratio;  // 地图分辨率
    double robotsize; // 机器人直径

    cv::Mat BIamap; // 岛群图像
    cv::Mat BIimap; // 岛号图像
    cv::Mat BIdmap; // debug图象
    cv::Mat BIfmap; // free图象
    int testcount = 0;

    void MaptoBInavi(char *IMGmap, double ratio, double rsize, const char *addr = "127.0.0.1", int port = 23231);
    void DrawBImap(bool debugmap);

    std::vector<BIgraph> BIgraphList;
    void FindConcave(const BIpolygons &polygons, std::set<BIangle> &ConcaveSet);
    void FindConcave(const BIpolygons &polygons, std::set<BIangle> &ConcaveSet, std::set<BIangle> &AngleSet);
    void ViewablePoint(const BIangle &ConcaveAngle, const BIpolygons &polygons, const std::list<BIline> cutlineList, std::vector<BIpoint> &VPList);
    BIpoint WeightViewablePoint(const BIangle &ConcaveAngle, const BIpolygons &polygons, const std::list<BIline> cutlineList);
    void StartCut(const BIpolygons &polygons, BIgraph &graph);
    void ReversePathClearing(std::vector<int32_t> &polyPath);
    void GetLeastHomotopyPath(const std::vector<BIline> &f_path, std::list<BIpoint> &Path, double &PathMinCost);

    //**********THPP**********//
    void THPPtaskInit(THPPtask &task, BIpoint &origin, double tl, int8_t mod = 0);                                                  // 初始化THPP任务，mod = 0 系留配置约束，mod = 1 一般最优路径约束
    bool THPPExpandingClassValidity(THPPtask &task, int32_t HomotopyPolyIndexPar, int32_t PolyIndexSub, double threshold = 3);      // 边的有效性检测
    void THPPgetCDTencoding(THPPtask &task, int32_t HomotopyPolyIndex, std::vector<int32_t> &polyPath);                             // 获取CDT编码，回溯HomotopyPoly编码树
    void THPPgetCDTencodingCutline(THPPtask &task, int32_t HomotopyPolyIndex, std::vector<BIline> &cpath, BIpoint goal = {-1, -1}); // 获取CDT编码的对偶形式

    double THPPoptimalReConfig(THPPtask &task, std::vector<int32_t> &polyPathS, std::vector<int32_t> &polyPathG,
                               BIpoint Init, BIpoint Goal, std::list<BIpoint> &minPath); // 待测试
    double THPPoptimalPlanner(THPPtask &task, int32_t HomotopyPolyIndex_Init, int32_t &HomotopyPolyIndex_Goal,
                            BIpoint Init, BIpoint Goal, std::list<BIpoint> &minPath);                  // 系留机器人最优路径规划器，完整形式，初始化mod0
    double THPPoptimalPlanner(THPPtask &task);                                                           // 系留机器人最优路径规划器，简化形式
    double UTHPPoptimalPlanner(THPPtask &task, BIpoint Init, BIpoint Goal, std::list<BIpoint> &minPath); // 非系留机器人最优路径规划器，初始化mod1
                                                                                                       // 以初步测试
    double TMVoptimalPlanner(THPPtask &task, int32_t HomotopyPolyIndex_Init, BIpoint Init,
                           std::vector<BIpoint> Goals, std::list<BIpoint> &minPath); // 系留机器人最优多目标访问规划器，初始化mod0
    double TMVoptimalPlannerViolent(THPPtask &task, int32_t HomotopyPolyIndex_Init, BIpoint Init,
                                  std::vector<BIpoint> Goals, std::list<BIpoint> &minPath); // 暴力搜索版本，对比算法

    void GetAllOptConfigurations(THPPtask &task, BIpoint goal,
                                 std::map<int32_t, std::pair<std::list<BIpoint>, double>>
                                     &ConfigList);

    BImap();
    ~BImap();
};

// 通用函数与参数 >>>
#define doubleMax (1.79769e+307)
// vector<pair<int32_t, size_t>>的哈希计算器
size_t hash_vec64(const std::vector<std::pair<int32_t, size_t>> &vec);
// 返回第一个共同元素，已弃用
int32_t findCommonElement(const std::set<int32_t> &set1, const std::set<int32_t> &set2);
// 简化的计时器
int64_t utime_ns(void);
// 计算点p到点o连线沿x轴正方向顺时针的夹角（单位：弧度）
double calculateAngle(BIpoint o, BIpoint p);
// 计算叉乘
double crossProduct(const BIpoint &O, const BIpoint &A, const BIpoint &B);
// 松弛的：检查两条线段是否相交，无视端点，线段长度不为0
bool doIntersect(const BIline &l1, const BIline &l2);
// 严谨的：检查两条线段是否相交，检查端点，长度可为0
bool doIntersect_rigorous(const BIline &l1, const BIline &l2);

// 可视化调试函数 >>>
void drawBIgraphObs(BIgraph &graph, cv::Mat image);
void drawPolygons(const BIpolygons &polygons, cv::Mat image);
void drawConcaves(std::list<BIangle> &ConcaveSet, cv::Mat image);
void drawAngle(BIangle &nowAngle, cv::Mat image);
void drawBIgraph(BIgraph &graph, cv::Mat image);

void drawBIring(std::list<BIpoint> &Path, cv::Mat &image);
void drawBIpath(const std::list<BIpoint> &path, cv::Mat &image, int32_t waittime = 30, cv::Scalar color = cv::Scalar(255, 0, 0));
void drawBIfreeID(const BIgraph &graph, cv::Mat &image);

#endif