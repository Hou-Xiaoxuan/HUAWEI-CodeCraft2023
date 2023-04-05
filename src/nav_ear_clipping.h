#ifndef _NAV_EAR_CLIPPING_H_
/* 折耳法三角剖分 */
 #include "nav_model.h"
 #include <unordered_set>
namespace navmesh
{
class EarClipping
{
    Polygon polygon;
    // 是否是折耳法可以切割的点：凸点，且不与多边形内部的任何边相交

    /*用来简化计算的特殊处理*/
    struct Node : Vertex {
        Node *prev {nullptr};
        Node *next {nullptr};

        bool is_ear {false};          // 是否是折耳
        bool is_processed {false};    // 是否处理过了
        Node() = default;
        Node(const Vertex &v) : Vertex(v) { }
        const Vertex &point() const { return *this; }
    };
    vector<Node> node_list;
    struct NodeHash {
        size_t operator()(const Node &node) const { return hash<double>()(node.x * 100 + node.y); }
    };

    unordered_set<Node, NodeHash>
        black_list;    // 当剔除一条三边重合的边时，需要把这个点加入黑名单，避免影响后续的判断

    bool can_cut(const Node &pre, const Node &cur, const Node &next)
    {
        // 凸点
        if (point_right_line(cur, {pre, next})) return false;

        // // 判断是否与其他边相交 // 似乎不太好用
        // for (int i = 0; i < polygon.vertices.size(); i++)
        // {
        //     int j = i + 1;
        //     const Vertex &p1 = polygon.vertices[i];
        //     const Vertex &p2 = polygon.vertices[j];
        //     if (p1 == pre or p1 == next or p2 == pre or p2 == next) continue;
        //     if (Segment::is_cross({pre, next}, {p1, p2})) return false;
        // }
        Triangle tmp = {pre, cur, next};
        tmp.clockwise();
        for (auto node : node_list)
        {
            if (node == pre or node == cur or node == next) continue;
            if (tmp.in_triangle(node))
            {
                if (this->black_list.count(node))
                {
                    if (point_on_line(node, {pre, next})) continue;
                    if (point_on_line(node, {cur, next})) continue;
                    if (point_on_line(node, {pre, cur})) continue;
                }
                return false;
            }
        }
        return true;
    }
    void process_hole()
    {
        // 根据最大横坐标排序this->holes
        vector<int> hole_max_x(this->polygon.holes.size(), 0);
        for (int i = 0; i < this->polygon.holes.size(); i++)
        {
            auto &hole = this->polygon.holes[i];
            for (auto &p : hole.vertices)
                if (p.x > hole_max_x[i]) hole_max_x[i] = p.x;
        }
        vector<int> vis(this->polygon.holes.size(), 0);
        vector<int> order;
        while (order.size() < this->polygon.holes.size())
        {
            int max_index = -1;
            int max_x = -1;
            for (int i = 0; i < hole_max_x.size(); i++)
                if (hole_max_x[i] > max_x and vis[i] == 0)
                {
                    max_x = hole_max_x[i];
                    max_index = i;
                }
            order.push_back(max_index);
            vis[max_index] = 1;
        }
        // 从最大横坐标的洞开始，逐个处理
        for (int i = 0; i < order.size(); i++)
        {
            auto &hole = this->polygon.holes[order[i]];
            // 找到最右边的点和下标
            int max_index = -1;
            double max_x = -1;
            for (int j = 0; j < hole.vertices.size(); j++)
                if (hole.vertices[j].x > max_x)
                {
                    max_x = hole.vertices[j].x;
                    max_index = j;
                }
            auto p = hole.vertices[max_index];

            // 找p点向右，与多边形边相交的点
            struct {
                Vertex a{};
                Vertex b{};
                double cross_x{0};
            } cross_point;
            for (int _i = 0; _i < this->polygon.vertices.size(); _i++)
            {
                int _j = _i + 1;
                if (_j >= this->polygon.vertices.size()) _j = 0;
                const auto &a = this->polygon.vertices[_i];
                const auto &b = this->polygon.vertices[_j];
                if (Segment::is_cross(
                        {
                            a, b
                },
                        {p, {p.x + 1000, p.y}})
                    == false)
                    continue;

                double cross_x;
                if (a.x == b.x)    // 
                    cross_x = a.x;
                else if (a.y == b.y)    // 水平线（不可能重合的）
                    cross_x = a.x < b.x ? a.x : b.x;
                else
                {
                    double k = (b.y - a.y) / (b.x - a.x);
                    double b = a.y - k * a.x;
                    cross_x = (p.y - b) / k;
                }

                if (fabs(cross_point.cross_x) < EPS || cross_x < cross_point.cross_x)
                {
                    cross_point.a = a;
                    cross_point.b = b;
                    cross_point.cross_x = cross_x;
                }
            }

            Vertex connect_point = cross_point.a.x > cross_point.b.x ? cross_point.a : cross_point.b;
            if (fabs(connect_point.y - p.y) > EPS)    // 线重合时不用处理(大概)
            {
                // 检查三角形p, connect_point, cross_point中是否有其他点，如果有，找出其中与横轴夹角最小的点
                double min_angle = 1000;
                Vertex min_angle_point;
                Triangle tmp = {
                    p, connect_point, {cross_point.cross_x, p.y}
                };
                tmp.clockwise();
                for (int j = 0; j < this->polygon.vertices.size(); j++)
                {
                    const auto st = this->polygon.vertices[j];
                    if (st == connect_point) continue;
                    if (tmp.in_triangle(st))
                    {
                        auto tmp_angle = fabs(Vec2::angle(Vec2(p, st), Vec2(p, {p.x + 1000, p.y})));
                        if (tmp_angle < min_angle)
                        {
                            min_angle = tmp_angle;
                            min_angle_point = st;
                        }
                    }
                }

                if (min_angle < 1000) connect_point = min_angle_point;
            }
            cerr << "connect_point: ("<<p<<"," << connect_point <<")"<< endl;
            // 拼接p和connect_point
            vector<Vertex> new_vertices;
            new_vertices.reserve(this->polygon.vertices.size() + hole.vertices.size());
            int index = 0;
            for (int j = 0; j < this->polygon.vertices.size(); j++)
                if (this->polygon.vertices[j] == connect_point)
                    index = j;
            
            for (int j = 0; j < this->polygon.vertices.size(); j++)
            {
                new_vertices.push_back(this->polygon.vertices[j]);
                if (j == index)
                {
                    auto ite = find(hole.vertices.begin(), hole.vertices.end(), p);
                    if (ite == hole.vertices.end()) throw std::runtime_error("error");
                    auto ite_copy = ite;
                    while (ite != hole.vertices.end())
                    {
                        new_vertices.push_back(*ite);
                        ite++;
                    }
                    ite = hole.vertices.begin();
                    while (ite != ite_copy)
                    {
                        new_vertices.push_back(*ite);
                        ite++;
                    }
                    new_vertices.push_back(p);
                    new_vertices.push_back(connect_point);
                }
            }
            this->polygon.vertices = new_vertices;
        }

        // log
        cerr << "process_hole = [";
        for (auto &v : this->polygon.vertices)
            cerr << v << ',';
        cerr << "]" << endl;

        
    }



public:
    EarClipping(const Polygon &polygon) : polygon(polygon)
    {
        this->process_hole();

        // init
        for (int i = 0; i < this->polygon.vertices.size(); i++)
            node_list.emplace_back(this->polygon.vertices[i]);

        for (int i = 0; i < node_list.size(); i++)
        {
            int prev_index = i - 1;
            int next_index = i + 1;
            if (prev_index < 0) prev_index = static_cast<int>(node_list.size()) - 1;
            if (next_index >= node_list.size()) next_index = 0;
            node_list[i].prev = &node_list[prev_index];
            node_list[i].next = &node_list[next_index];
        }
        for (int i = 0; i < node_list.size(); i++)
            node_list[i].is_ear = this->can_cut(*node_list[i].prev, node_list[i], *node_list[i].next);
    }
    // 执行三角剖分
    std::vector<Triangle> triangulate()
    {
        size_t node_count = this->node_list.size();
        vector<Triangle> triangles;
        while (node_count > 3u)
        {
            bool is_cut = false;
            for (auto &node : node_list)
            {
                if (node.is_processed) continue;
                if (node.is_ear)
                {
                    // 判断是否共线
                    if ((Vec2 {node.prev->point(), node.point()} ^ Vec2 {node.point(), node.next->point()})
                        == 0)
                        black_list.insert(node);
                    else
                        triangles.emplace_back(node.prev->point(), node.point(), node.next->point());
                    node.is_processed = true;
                    node_count--;
                    is_cut = true;
                    node.prev->next = node.next;
                    node.next->prev = node.prev;
                    node.prev->is_ear = can_cut(*node.prev->prev, *node.prev, *node.prev->next);
                    node.next->is_ear = can_cut(*node.next->prev, *node.next, *node.next->next);

                    if (node_count <= 3u) break;
                }
            }
            if (node_count > 3 and !is_cut)
            {
                // 无法剪切，说明有问题
                cerr << "cut_data_faild" << endl;
                // 输出所有点
                cerr << "points = [";
                for (auto p : this->polygon.vertices)
                    cerr << p << ',';
                cerr << ']' << endl;
                // 输出所有三角形
                cerr << "triangles = [";
                for (auto t : triangles)
                    cerr << "(" << t.a << "," << t.b << "," << t.c << "),";
                cerr << ']' << endl;
                cerr << "cut_data_faild_over" << node_count << endl;

                cerr << "leave_node = [";
                for (auto &node : node_list)
                    if (!node.is_processed)
                    {
                        auto s = this->can_cut(*node.prev, node, *node.next);
                        cerr << node.point() << ',';
                    }
                cerr << "]" << endl;
                throw std::runtime_error("cut_data_faild");
                return {};
            }
        }
        // 剩下的三个点组成一个三角形
        for (auto &node : node_list)
        {
            if (!node.is_processed)
            {
                triangles.emplace_back(node.prev->point(), node.point(), node.next->point());
                break;
            }
        }
        return triangles;
    }
};
}

#endif