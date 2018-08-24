#ifndef CONSTANTS
#define CONSTANTS
#include <iostream>
#include <vector>

enum class STATUS : uint8_t //enum class
{
    OUT,//处于地图外面
    NONE,//没有被搜索过的初始状态
    OPEN,   //被搜索过一次
    CLOSED, //被提出的node
    OBS     //障碍
};

struct pose_index
{
    int x;
    int y;
};

struct search_node
{
    pose_index index;
    double g;
    double h;
    double g_h;
    pose_index pre_index;
    STATUS status;
    bool operator>(const search_node &my_node) const //运算符重载,如果较大,就返回true
    {
        return g_h > my_node.g_h;
    }
};

inline static bool isEqual(const pose_index &a, const pose_index &b)
{
    return (a.x == b.x && a.y == b.y) ? true : false;
}

#endif