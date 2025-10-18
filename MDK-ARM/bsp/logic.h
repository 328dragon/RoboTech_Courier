#ifndef __LOGIC_H
#define __LOGIC_H
#include <array>
enum Dir 
    { 
        UP, 
        DOWN, 
        LEFT, 
        RIGHT 
    };
/*
    迷宫类，包含迷宫地图和相关操作
*/
class Maze {
    public:
        int _whichground = 0; // 0表示左下到右上场地，1表示右下到左上场地
        float _block_distance = 25; // 视作障碍物的距离阈值，单位米
        float _current_distance = 0; // 当前超声波测距值，单位米
        // 5行8列的迷宫地图(0:空地, 1:障碍物)
        std::array<std::array<int, 8>, 5> _map;// 该地图的0,0为左上角，x正方向向下，y正方向向右
        int _x, _y;  // 当前位置坐标
        bool _be_block = false; // 记录被堵车，重点标志位，宝龙记得把这个成员变量丢到主函数里面，如果发现被堵住了，就接收我的转向信号
        int _be_block_time = -1; // 记录被堵车的次数
        bool _next_step=0;
        // bool _is_obstacle_detected = false; // 记录是否探测过障碍物
        Dir _prior_dir[4] = {UP, RIGHT, DOWN, LEFT}; // 默认左下到右上的方向优先级
        Dir _forbid_dir = DOWN; // 禁止出界方向
        Dir _current_dir; // 当前车头朝向(用于探测障碍物)
        Maze(int whichground,float block_distance) : _whichground(whichground), _block_distance(block_distance)
        {
            if(_whichground==0)
            {
                _x = 4; // 初始位置(左下角)
                _y = 0;

            }
            else if(_whichground==1)
            {
                _x = 4; // 初始位置(右下角)
                _y = 7;
                _prior_dir[1]= LEFT;
                _prior_dir[3]= RIGHT;
            }
            // 初始化地图为全空地
            for (auto& row :_map) row.fill(0);
        }
        bool update_next_dir(); // 刷新要走的方向
        bool detect_obstacle(); // 探测障碍物,返回这次在这个方向是否探测到一步的障碍物
        void update_self_position(); // 更新当前位置

};
#endif


