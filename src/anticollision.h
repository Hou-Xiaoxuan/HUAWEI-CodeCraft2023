#ifndef __ANTICOLLISION_H__
#define __ANTICOLLISION_H__

#include "const.h"
#include "model.h"
#include "route_fool.h"
#include <iostream>
#include <optional>
#include <set>
#include <vector>

namespace anticollision
{
using namespace std;
using io::instructions;


struct CollisionTree {
    int robot_id = 0;
    int type = 0;
    Point shelter_point;
    vector<CollisionTree> children;
    static double min_distance;
    static int index;
    static void clear_index()
    {
        min_distance = 1e9;
        index = 0;
    }

    static bool add_edge(int robot_i, int robot_j, CollisionTree &tree)
    {

        if (tree.robot_id == 0)
        {
            tree.robot_id = robot_i;
            tree.children.push_back({robot_j, 1});
            return 1;
        }

        double distance_i = Point::distance(meta.robot[tree.robot_id].loc, meta.robot[robot_i].loc);
        double distance_j = Point::distance(meta.robot[tree.robot_id].loc, meta.robot[robot_j].loc);
        int chirld_i, chirld_j;
        double tmp_min;
        if (distance_i < distance_j)
        {
            chirld_i = robot_i;
            chirld_j = robot_j;
            tmp_min = distance_i;
        }
        else
        {
            chirld_i = robot_j;
            chirld_j = robot_i;
            tmp_min = distance_j;
        }

        if (tmp_min < min_distance)
        {
            index = tree.robot_id;
            min_distance = tmp_min;
        }


        if (tree.robot_id == robot_i)
        {
            tree.children.push_back({robot_j, 1});
            return 1;
        }

        if (tree.robot_id == robot_j)
        {
            tree.children.push_back({robot_i, 1});
            return 1;
        }


        for (auto &child : tree.children)
        {
            if (add_edge(robot_i, robot_j, child))
            {
                return 1;
            }
        }

        if (index == tree.robot_id)
        {
            tree.children.push_back({chirld_i, 2});
            tree.children[tree.children.size() - 1].children.push_back({chirld_j, 1});
            return 1;
        }
        return 0;
    }
    static Point get_spec_dir_shelter(int robot_id, int RotateDirection)
    {
        const auto &robot = meta.robot[robot_id];
        Point center;    // 圆心
        center.x = robot.loc.x + RotateDirection * ComVar::max_radius * sin(robot.dirc);
        center.y = robot.loc.y - RotateDirection * ComVar::max_radius * cos(robot.dirc);

        double shelter_angular = robot.dirc + M_PI / 2;
        Point shelter_point;
        shelter_point.x = center.x - RotateDirection * ComVar::max_radius * sin(shelter_angular);
        shelter_point.y = center.y + RotateDirection * ComVar::max_radius * cos(shelter_angular);
        return shelter_point;
    }

    static Point _shelter(int robot_id, Point point)
    {
        vector<Point> tmp_shelter = {get_spec_dir_shelter(robot_id, 1), get_spec_dir_shelter(robot_id, -1)};
        return Point::distance(point, tmp_shelter[0]) > Point::distance(point, tmp_shelter[1])
            ? tmp_shelter[0]
            : tmp_shelter[1];
    }


    static void shelter(CollisionTree &tree)
    {
        if (tree.robot_id == 0)
        {
            return;
        }

        if (tree.type == 1 || tree.type == 2)
        {
            int robot_i = tree.robot_id;
            int robot_j = tree.children[0].robot_id;
            vector<Point> shelter_i = {get_spec_dir_shelter(robot_i, 1), get_spec_dir_shelter(robot_j, -1)};
            vector<Point> shelter_j = {get_spec_dir_shelter(robot_j, 1), get_spec_dir_shelter(robot_j, -1)};
            int shelter_index_i = 0, shelter_index_j = 0;
            double shelter_dis = 0;
            for (int i = 0; i < 2; ++i)
            {
                for (int j = 0; j < 2; ++j)
                {
                    double tmp_dis = Point::distance(shelter_i[i], shelter_j[j]);
                    if (tmp_dis >= shelter_dis)
                    {
                        shelter_dis = tmp_dis;
                        shelter_index_i = i;
                        shelter_index_j = j;
                    }
                }
            }
            tree.shelter_point = shelter_i[shelter_index_i];
            tree.children[0].shelter_point = shelter_j[shelter_index_j];
            for (int i = 1; i < tree.children.size(); ++i)
            {
                tree.children[i].shelter_point = _shelter(tree.children[i].robot_id, tree.shelter_point);
                shelter(tree.children[i]);
            }
        }
        else
        {
            for (int i = 0; i < tree.children.size(); ++i)
            {
                tree.children[i].shelter_point = _shelter(tree.children[i].robot_id, tree.shelter_point);
                shelter(tree.children[i]);
            }
        }

        navigate::move_to(meta.robot[tree.robot_id], tree.shelter_point);
    }
};

double CollisionTree::min_distance = 1e9;
int CollisionTree::index = 0;

void anticollision(const vector<optional<route_fool::Route>> &routes)
{

    int max_predict_flame = 15;
    const auto &robots = meta.robot;
    vector<vector<Point>> predict_point(max_predict_flame + 1, vector<Point>(robots.size()));

    for (int predict_flame = 1; predict_flame <= max_predict_flame; ++predict_flame)
    {

        double predict_time = predict_flame * ComVar::flametime;
        for (int i = 1; i < robots.size(); ++i)
        {
            const auto &robot = robots[i];

            if (fabs(robot.w) < M_PI / 10)
            {
                predict_point[predict_flame][i].x = robot.loc.x + robot.v.x * predict_time;
                predict_point[predict_flame][i].y = robot.loc.y + robot.v.y * predict_time;
                continue;
            }

            double radius = robot.v.len() / robot.w;    // 半径
            Point center;                               // 圆心
            center.x = robot.loc.x - radius * sin(robot.dirc);
            center.y = robot.loc.y + radius * cos(robot.dirc);

            double predict_angular = robot.dirc + robot.w * predict_time;
            predict_point[predict_flame][i].x = center.x + radius * sin(predict_angular);
            predict_point[predict_flame][i].y = center.y - radius * cos(predict_angular);
        }
    }

    CollisionTree tree;
    vector<vector<int>> vis = vector<vector<int>>(robots.size(), vector<int>(robots.size(), 0));
    for (int predict_flame = 1; predict_flame <= max_predict_flame; ++predict_flame)
    {
        for (int i = 1; i < robots.size(); ++i)
        {
            for (int j = i + 1; j < robots.size(); ++j)
            {
                if (vis[i][j]) continue;

                double min_distance
                    = navigate::__get_robot_radius(robots[i]) + navigate::__get_robot_radius(robots[j]);
                if (Point::distance(predict_point[predict_flame][i], predict_point[predict_flame][j])
                    <= min_distance)
                {
                    cerr << "info: robot " << i << " and robot " << j << " will collide in "
                         << predict_flame << " flame(s)." << endl;
                    CollisionTree::clear_index();
                    CollisionTree::add_edge(i, j, tree);
                    vis[i][j] = 1;
                }
            }
        }
    }
    CollisionTree::shelter(tree);
}
}

#endif