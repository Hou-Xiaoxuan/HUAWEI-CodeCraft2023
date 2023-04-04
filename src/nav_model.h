#ifndef _NVV_MODEL_H_
#define _NVV_MODEL_H_
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
    bool operator==(const Vertex &other) const
    {
        return fabs(x - other.x) < EPS && fabs(y - other.y) < EPS;
    }
    friend ostream &operator<<(ostream &os, const Vertex &v)
    {
        os << "(" << v.x << ", " << v.y << ")";
        return os;
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
    // 加
    friend Vec2 operator+(const Vec2 &v1, const Vec2 &v2) { return {v1.x + v2.x, v1.y + v2.y}; }
    // 减
    friend Vec2 operator-(const Vec2 &v1, const Vec2 &v2) { return {v1.x - v2.x, v1.y - v2.y}; }
    // 夹角
    static double angle(const Vec2 &v1, const Vec2 &v2)
    {
        return acos((v1 * v2) / (v1.length() * v2.length()));
    }
};
// 线段
struct Segment {
    Vertex a;
    Vertex b;
    Segment() = default;
    Segment(const Vertex &a, const Vertex &b) : a(a), b(b) { }

    // 判断是否相交
    static bool is_cross(const Segment &s1, const Segment &s2)
    {
        Vec2 v1 {s1.a, s1.b}, v2 {s1.a, s2.a}, v3 {s1.a, s2.b};
        Vec2 v4 {s2.a, s2.b}, v5 {s2.a, s1.a}, v6 {s2.a, s1.b};
        double cross_1 = (v1 ^ v2) * (v1 ^ v3);
        double cross_2 = (v4 ^ v5) * (v4 ^ v6);
        if (fabs(cross_1) <=0 and fabs(cross_2) <=0)
        {
            // 共线,根据x大小交换点坐标
            auto _s1 = s1;
            auto  _s2 = s2;
            if (_s1.a.x > _s1.b.x) std::swap(_s1.a, _s1.b);
            if (_s2.a.x > _s2.b.x) std::swap(_s2.a, _s2.b);
            if (_s1.a.x < _s2.a.x and _s1.b.x >= _s2.b.x) return true;
            if (_s2.a.x < _s1.a.x and _s2.b.x >= _s1.b.x) return true;
            return false;
        }
        return cross_1 <= 0 && cross_2 <= 0;
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
}
#endif