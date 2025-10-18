/*
 * @Author: Nagisa 2964793117@qq.com
 * @Date: 2025-10-13 19:15:07
 * @LastEditors: Nagisa 2964793117@qq.com
 * @LastEditTime: 2025-10-14 21:20:08
 * @FilePath: \MDK-ARM\bsp\logic.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 * @Author: Nagisa 2964793117@qq.com
 * @Date: 2025-10-13 19:15:07
 * @LastEditors: Nagisa 2964793117@qq.com
 * @LastEditTime: 2025-10-13 21:40:49
 * @FilePath: \MDK-ARM\bsp\logic.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
// 技能赛智慧快递的逻辑文件+接口的实现
// 请注意，本文件的基础在车只有前向灰度和超声波传感器（一样一个）的基础上进行编写的
#include "logic.h"
bool Maze::detect_obstacle()
{
    // 测量车的前方是否有障碍物
    if (_current_distance <= _block_distance ) 
    {
        // 如果当前距离小于阈值，说明前方有障碍物
        if(_current_dir == UP) 
        {
            _map[_x-1][_y] = 1; // 在地图上标记障碍物
        } 
        else if (_current_dir == DOWN) 
        {
            _map[_x+1][_y] = 1; // 在地图上标记障碍物
        } 
        else if (_current_dir == LEFT) 
        {
            _map[_x][_y-1] = 1; // 在地图上标记障碍物
        } 
        else if (_current_dir == RIGHT) 
        {
            _map[_x][_y+1] = 1; // 在地图上标记障碍物
        }
        return true; // 返回探测到障碍物
    }
    return false; // 返回未探测到障碍物
}
/*
    极其重要的避障函数，在每次转向后都需要执行一次
*/
bool Maze::update_next_dir() 
{
    bool found_obstacle = detect_obstacle(); // 探测障碍物,首先看当前方向前一格是否有障碍物
    //然后看当前是否在边界,如果在边界则禁止出界方向
    if(_x==0)
    {
         _forbid_dir = UP;
    }
    if(_x==4)
    {
         _forbid_dir = DOWN;
    }
    if(_y==0 && _whichground==1)
    {
         _forbid_dir = LEFT;
    }
    if(_y==7 && _whichground==0)
    {
         _forbid_dir = RIGHT;
    }
    //如果没有障碍，将current_dir设为_prior_dir数组中从头到尾遍历第一个不和_forbid_dir相同的方向
    //并在此逻辑中恢复堵车状态，但不改变此时的current_dir
    if(!found_obstacle)
    {
        if(_be_block) // 如果之前是堵车状态，说明现在已经通了
        {
            _be_block_time = -1; // 堵车次数清零
        }
        if(!_be_block)
        {
            for(int i=0;i<4;i++)
            {
                if(_prior_dir[i]!=_forbid_dir)
                {
                    _current_dir = _prior_dir[i];
                    break;
                }
            }
        }
        _be_block = false; // 堵车状态
        return false; 
    }
    else if(found_obstacle)
    {
        _be_block = true; // 堵车状态
        _be_block_time++;
        // 从_prior_dir第_be_block_time个开始遍历，找到第一个不和_forbid_dir相同的方向
        for(int i=_be_block_time;i<4;i++)
        {
            if(_prior_dir[i]!=_forbid_dir)
            {
                _current_dir = _prior_dir[i];
                break;
            }
        }
        return true; 
    }
		return false;
}
/*
    逻辑顺序为，每次运动一步，在运动前首先探测障碍物，刷新要走的方向，并转向，然后走一步，然后更新当前位置，循环往复
*/
void Maze::update_self_position() 
{
    // 根据当前方向更新位置
    if (_current_dir == UP) 
    {
        _x -= 1; // 向上移动一格
    } 
    else if (_current_dir == DOWN) 
    {
        _x += 1; // 向下移动一格
    } 
    else if (_current_dir == LEFT) 
    {
        _y -= 1; // 向左移动一格
    } 
    else if (_current_dir == RIGHT) 
    {
        _y += 1; // 向右移动一格
    }
}