#ifndef _NVV_MODEL_H_
 #include <cmath>
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
};

// 三角形
struct Triangle {
    Vertex a;
    Vertex b;
    Vertex c;

    Triangle() = default;
    Triangle(const Vertex &a, const Vertex &b, const Vertex &c) : a(a), b(b), c(c) { }

    // 一个点是否在三角形内部
    inline bool in_triangle(Vertex p) const
    {
        double s = a.x * b.y + b.x * c.y + c.x * a.y - a.y * b.x - b.y * c.x - c.y * a.x;
        double s1 = p.x * b.y + b.x * c.y + c.x * p.y - p.y * b.x - b.y * c.x - c.y * p.x;
        double s2 = a.x * p.y + p.x * c.y + c.x * a.y - a.y * p.x - p.y * c.x - c.y * a.x;
        double s3 = a.x * b.y + b.x * p.y + p.x * a.y - a.y * b.x - b.y * p.x - p.y * a.x;
        return fabs(s - s1 - s2 - s3) < EPS;
    }
};

// 多边形(最多一次内嵌)
struct Polygon {
    std::vector<Vertex> vertices;    // 默认顺时针
    std::vector<Polygon> holes;      // 默认逆时针
    Polygon() = default;
    Polygon(const std::vector<Vertex> &vertices) : vertices(vertices) { }
    Polygon(const std::vector<Vertex> &vertices, const std::vector<Polygon> &holes) :
        vertices(vertices), holes(holes)
    { }
};

// 点是否在line右侧
inline bool point_right_line(const Vertex &p, const Vertex &from, const Vertex &to)
{
    return (to.x - from.x) * (p.y - from.y) - (to.y - from.y) * (p.x - from.x) > 0;
}
// 点是否在line 左侧
inline bool point_left_line(const Vertex &p, const Vertex &from, const Vertex &to)
{
    return (to.x - from.x) * (p.y - from.y) - (to.y - from.y) * (p.x - from.x) < 0;
}
// 点是否在line上
inline bool point_on_line(const Vertex &p, const Vertex &from, const Vertex &to)
{
    return (to.x - from.x) * (p.y - from.y) - (to.y - from.y) * (p.x - from.x) == 0;
}
}
#endif