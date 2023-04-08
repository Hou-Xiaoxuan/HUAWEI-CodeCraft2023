#ifndef _NVV_MODEL_H_
#define _NVV_MODEL_H_
#include "model.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>
namespace navmesh
{
const double EPS = 1e-6;
using namespace std;
// 点
struct Vertex {
    double x;
    double y;
    Vertex() = default;
    Vertex(double x, double y) : x(x), y(y) { }
    Vertex(const Point &p) : x(p.x), y(p.y) { }
    // 两点距离
    static double distance(const Vertex &v1, const Vertex &v2)
    {
        return sqrt((v1.x - v2.x) * (v1.x - v2.x) + (v1.y - v2.y) * (v1.y - v2.y));
    }

    bool operator==(const Vertex &other) const
    {
        return fabs(x - other.x) < EPS && fabs(y - other.y) < EPS;
    }
    friend ostream &operator<<(ostream &os, const Vertex &v)
    {
        os << "(" << v.x << ", " << v.y << ")";
        return os;
    }

    friend bool operator<(const Vertex &v1, const Vertex &v2)
    {
        return v1.x < v2.x or (v1.x == v2.x and v1.y < v2.y);
    }
};

// 二维向量
struct Vec2 {
    double x;
    double y;
    Vec2() = default;
    Vec2(double x, double y) : x(x), y(y) { }
    Vec2(const Vertex &v1, const Vertex &v2) : x(v2.x - v1.x), y(v2.y - v1.y) { }
    // 向量
    double length() const { return sqrt(x * x + y * y); }
    friend double operator^(const Vec2 &v1, const Vec2 &v2) { return v1.x * v2.y - v1.y * v2.x; }
    // 点乘
    friend double operator*(const Vec2 &v1, const Vec2 &v2) { return v1.x * v2.x + v1.y * v2.y; }
    friend Vec2 operator*(double k, const Vec2 &v) { return {v.x * k, v.y * k}; }
    // 加
    friend Vec2 operator+(const Vec2 &v1, const Vec2 &v2) { return {v1.x + v2.x, v1.y + v2.y}; }
    // 减
    friend Vec2 operator-(const Vec2 &v1, const Vec2 &v2) { return {v1.x - v2.x, v1.y - v2.y}; }
    // 乘浮点数
    friend Vec2 operator*(const Vec2 &v, double k) { return {v.x * k, v.y * k}; }
    // 除浮点数
    friend Vec2 operator/(const Vec2 &v, double k)
    {
        if (fabs(k) < EPS)
        {
            if (_USE_LOG_)
            {
                cerr << "[error][Vec2 /]"
                     << "divide by zero" << endl;
            }
        }
        return {v.x / k, v.y / k};
    }

    // 夹角
    static double angle(const Vec2 &v1, const Vec2 &v2)
    {
        return acos((v1 * v2) / (v1.length() * v2.length()));
    }
    // angle > 0 逆时针 angle:弧度
    Vec2 rotate(double angle) const
    {
        double x = this->x * cos(angle) - this->y * sin(angle);
        double y = this->x * sin(angle) + this->y * cos(angle);
        return {x, y};
    }
};
// 线段
struct Segment {
    Vertex a;
    Vertex b;
    Segment() = default;
    Segment(const Vertex &a, const Vertex &b) : a(a), b(b) { }

    double length() const { return Vec2(a, b).length(); }

    // 判断是否相交
    static bool is_cross(const Segment &s1, const Segment &s2)
    {
        Vec2 v1 {s1.a, s1.b}, v2 {s1.a, s2.a}, v3 {s1.a, s2.b};
        Vec2 v4 {s2.a, s2.b}, v5 {s2.a, s1.a}, v6 {s2.a, s1.b};
        double cross_1 = (v1 ^ v2) * (v1 ^ v3);
        double cross_2 = (v4 ^ v5) * (v4 ^ v6);
        if (fabs(cross_1) <= 0 and fabs(cross_2) <= 0)
        {
            // 共线,根据x大小交换点坐标
            auto _s1 = s1;
            auto _s2 = s2;
            if (_s1.a.x > _s1.b.x) std::swap(_s1.a, _s1.b);
            if (_s2.a.x > _s2.b.x) std::swap(_s2.a, _s2.b);
            if (_s1.a.x < _s2.a.x and _s1.b.x >= _s2.b.x) return true;
            if (_s2.a.x < _s1.a.x and _s2.b.x >= _s1.b.x) return true;
            return false;
        }
        return cross_1 <= 0 && cross_2 <= 0;
    }


    static bool is_cross_2(const Segment &s1, const Segment &s2);

    // 两条线段最短距离
    static double distance(const Segment &s1, const Segment &s2);

    static Vertex get_mid(const Segment &s) { return {(s.a.x + s.b.x) / 2, (s.a.y + s.b.y) / 2}; }

    friend bool operator<(const Segment &s1, const Segment &s2)
    {
        return s1.a < s2.a or (s1.a == s2.a and s1.b < s2.b);
    }

    friend bool operator==(const Segment &s1, const Segment &s2)
    {
        return (s1.a == s2.a and s1.b == s2.b) or (s1.a == s2.b and s1.b == s2.a);
    }
};
// 三角形
struct Triangle {
    Vertex a;
    Vertex b;
    Vertex c;

    Triangle() = default;
    Triangle(const Vertex &a, const Vertex &b, const Vertex &c) : a(a), b(b), c(c) { }
    // 三角形面积
    double area() const
    {
        Vec2 v1 {a, b}, v2 {a, c};
        return fabs(v1 ^ v2) / 2;
    }
    // 顺时针
    void clockwise()
    {
        Vec2 v1 {a, b}, v2 {a, c};
        if ((v1 ^ v2) < 0) std::swap(b, c);
    }
    // 点p是否在三角形内部(精度要求高)
    inline bool in_triangle(Vertex p)
    {
        this->clockwise();
        if ((Vec2 {a, b} ^ Vec2 {a, p}) < 0) return false;
        if ((Vec2 {b, c} ^ Vec2 {b, p}) < 0) return false;
        if ((Vec2 {c, a} ^ Vec2 {c, p}) < 0) return false;
        return true;
    }
    friend ostream &operator<<(ostream &os, const Triangle &t)
    {
        os << "<" << t.a << " " << t.b << " " << t.c << ">";
        return os;
    }
};

// 多边形(最多一次内嵌)
struct Polygon {
    std::vector<Vertex> vertices;    // 默认顺时针
    std::vector<Polygon> holes;      // 默认逆时针
    Polygon() = default;
    Polygon(std::vector<Vertex> vertices) : vertices(std::move(vertices)) { }
    Polygon(std::vector<Vertex> vertices, std::vector<Polygon> holes) :
        vertices(std::move(vertices)), holes(std::move(holes))
    { }
};

// 点是否在line, line方向为from->to右侧
inline bool point_right_line(const Vertex &p, const Segment &line)
{
    return (Vec2(line.a, line.b) ^ Vec2(line.a, p)) < 0;
}
// 点是否在line 左侧
inline bool point_left_line(const Vertex &p, const Segment &line)
{
    return (Vec2(line.a, line.b) ^ Vec2(line.a, p)) > 0;
}
// 点是否在line上
inline bool point_on_line(const Vertex &p, const Segment &line)
{
    return fabs(Vec2(line.a, line.b) ^ Vec2(line.a, p)) == 0;
}


// 点到线段距离
inline double dis_point_to_segment(const Vertex &p, const Segment &line)
{
    Vec2 v1 {line.a, line.b}, v2 {line.a, p};
    if (v1 * v2 < 0) return v2.length();
    Vec2 v3 {line.b, line.a}, v4 {line.b, p};
    if (v3 * v4 < 0) return v4.length();
    return fabs(v1 ^ v2) / v1.length();
}


// 点到线段最短距离 对应线段上的点
inline Vertex point_point_to_segment(const Vertex &p, const Segment &line)
{
    Vec2 v1 {line.a, line.b}, v2 {line.a, p};
    if (v1 * v2 < 0) return line.a;
    Vec2 v3 {line.b, line.a}, v4 {line.b, p};
    if (v3 * v4 < 0) return line.b;
    // p 到 line 的垂足
    double t = (v1 * v2) / (v1 * v1);
    return {line.a.x + t * v1.x, line.a.y + t * v1.y};
}

double Segment::distance(const Segment &s1, const Segment &s2)
{
    if (is_cross(s1, s2)) return 0;
    return min({dis_point_to_segment(s1.a, s2), dis_point_to_segment(s1.b, s2),
        dis_point_to_segment(s2.a, s1), dis_point_to_segment(s2.b, s1)});
}

bool Segment::is_cross_2(const Segment &s1, const Segment &s2)
{
    if (Segment::is_cross(s1, s2))
    {
        if (point_on_line(s1.a, s2) or point_on_line(s1.b, s2) or point_on_line(s2.a, s1)
            or point_on_line(s2.b, s1))
            return false;
        return true;
    }
    return false;
}

}
#endif