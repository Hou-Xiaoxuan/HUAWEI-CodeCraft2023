#ifndef _NAV_EAR_CLIPPING_H_
/* 折耳法三角剖分 */
 #include "nav_model.h"
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
    bool _interior(const Node &v1, const Node &v2)
    {
        // pre点是凸点，则与起点相连的两个点应该在line两侧。如果pre点是凹点，则……
        //  convex point
        if (!point_right_line(v1.prev->point(), {v1.point(), v1.next->point()}))
        {
            return point_left_line(v1.prev->point(), {v1.point(), v2.point()})
                and point_right_line(v1.next->point(), {v1.point(), v2.point()});
        }
        // reflex point
        // XXX 需要理解
        return !(point_right_line(v1.next->point(), {v1.point(), v2.point()})
            and point_right_line(v1.prev->point(), {v1.point(), v2.point()}));
    }
    bool can_cut(const Node &pre, const Node &cur, const Node &next)
    {
        // 凸点
        if (point_right_line(cur, {pre, next})) return false;

        // 判断是否与其他边相交
        for (int i = 0; i < polygon.vertices.size(); i++)
        {
            int j = i + 1;
            const Vertex &p1 = polygon.vertices[i];
            const Vertex &p2 = polygon.vertices[j];
            if (p1 == pre or p1 == next or p2 == pre or p2 == next) continue;
            if (Segment::is_cross({pre, next}, {p1, p2})) return false;
        }

        // 判断是完全在内部还是在外部
        Vertex tmp = {(pre.x + next.x) / 2, (pre.y + next.y) / 2};
        return point_right_line(tmp, {pre, cur}) and point_right_line(tmp, {cur, next});
    }
    void process_hole()
    {
        // 根据最大横坐标排序this->holes
        vector<int> vis(this->polygon.holes.size(), 0);
        vector<int> order;
        for (int i = 0; i < this->polygon.holes.size(); i++)
        {
            int max_index = -1;
            double max_x = -1;
            for (int j = 0; j < this->polygon.holes.size(); j++)
            {
                if (vis[j]) continue;
                if (this->polygon.holes[j].vertices[0].x > max_x)
                {
                    max_x = this->polygon.holes[j].vertices[0].x;
                    max_index = j;
                }
            }
            vis[max_index] = 1;
            order.push_back(max_index);
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
                Vertex a;
                Vertex b;
                double cross_x;
            } cross_point;
            for (int _i = 0; _i < hole.vertices.size(); _i++)
            {
                int _j = _i + 1;
                if (_j >= hole.vertices.size()) _j = 0;
                const auto &a = hole.vertices[_i];
                const auto &b = hole.vertices[_j];
                if (!(Segment::is_cross(
                        {
                            a, b
                },
                        {p, {p.x + 1000, p.y}})))
                    continue;

                double cross_x = a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y);
                if (cross_point.cross_x == 0 || cross_x < cross_point.cross_x)
                {
                    cross_point.a = a;
                    cross_point.b = b;
                    cross_point.cross_x = cross_x;
                }
            }

            Vertex connect_point = cross_point.a.x > cross_point.b.x ? cross_point.a : cross_point.b;
            // 检查三角形p, connect_point, cross_point中是否有其他点，如果有，找出其中与横轴夹角最小的点
            double min_angle = 1000;
            Vertex min_angle_point;
            Triangle tmp = {
                p, connect_point, {cross_point.cross_x, p.y}
            };
            tmp.clockwise();

            for (int j = 0; j < this->polygon.vertices.size(); j++)
            {
                const auto st_p = this->polygon.vertices[j];
                if (st_p == connect_point) continue;
                if (tmp.in_triangle(p))
                {
                    auto tmp_angle = fabs(Vec2::angle(Vec2(p, st_p), Vec2(p, {p.x + 1000, p.y})));
                    if (tmp_angle < min_angle)
                    {
                        min_angle = tmp_angle;
                        min_angle_point = st_p;
                    }
                }
            }

            if (min_angle < 1000) connect_point = min_angle_point;

            // 拼接p和connect_point
            vector<Vertex> new_vertices;
            new_vertices.reserve(this->polygon.vertices.size() + hole.vertices.size());
            for (int j = 0; j < this->polygon.vertices.size(); j++)
            {
                new_vertices.push_back(this->polygon.vertices[j]);
                if (this->polygon.vertices[j] == connect_point)
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
    }



public:
    EarClipping(const Polygon &polygon) : polygon(polygon)
    {
        // this->process_hole();

        // init
        for (int i = 0; i < polygon.vertices.size(); i++)
            node_list.emplace_back(polygon.vertices[i]);

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
                    triangles.emplace_back(node.prev->point(), node.point(), node.next->point());
                    node.is_processed = true;
                    node_count--;
                    is_cut = true;
                    // // 更新相邻节点
                    // node_list[node.prev_index].next_index = node.next_index;
                    // node_list[node.next_index].prev_index = node.prev_index;
                    // // 更新相邻节点的is_ear
                    // node_list[node.prev_index].is_ear
                    //     = can_cut(polygon.vertices[node_list[node.prev_index].prev_index],
                    //         polygon.vertices[node_list[node.prev_index].index],
                    //         polygon.vertices[node_list[node.prev_index].next_index]);
                    // node.is_ear
                    //     = can_cut(polygon.vertices[node_list[node.next_index].prev_index],
                    //         polygon.vertices[node_list[node.next_index].index],
                    //         polygon.vertices[node_list[node.next_index].next_index]);
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