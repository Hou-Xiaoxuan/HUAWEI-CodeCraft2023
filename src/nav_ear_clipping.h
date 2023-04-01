#ifndef _NAV_EAR_CLIPPING_H_
/* 折耳法三角剖分 */
 #include "nav_model.h"
namespace navmesh
{
class EarClipping
{
    Polygon polygon;
    // 是否是折耳法可以切割的点：凸点，且除cur点外，其余点都在pre->next的右侧
    bool can_cut(const Vertex &pre, const Vertex &cur, const Vertex &next)
    {
        // XXX 可优化
        if (point_right_line(pre, cur, next)) return false;

        for (auto &v : polygon.vertices)
        {
            if (v == pre || v == cur || v == next) continue;
            if (point_left_line(v, pre, next)) return false;
        }
        return true;
    }

    /*用来简化计算的特殊处理*/
    struct Node {
        int index;
        int prev_index;
        int next_index;
        bool is_ear {false};          // 是否是折耳
        bool is_processed {false};    // 是否处理过了
        Node() = default;
        Node(int index, int prev_index, int next_index) :
            index(index), prev_index(prev_index), next_index(next_index)
        { }
    };
    vector<Node> node_list;
    void process_hole()
    { /*TODO*/
    }

public:
    EarClipping(const Polygon &polygon) : polygon(polygon)
    {
        this->process_hole();

        // init
        for (int i = 0; i < polygon.vertices.size(); i++)
        {
            int prev_index = i - 1;
            int next_index = i + 1;
            if (prev_index < 0) prev_index = polygon.vertices.size() - 1;
            if (next_index >= polygon.vertices.size()) next_index = 0;
            node_list.push_back(Node(i, prev_index, next_index));
            node_list.back().is_ear
                = can_cut(polygon.vertices[prev_index], polygon.vertices[i], polygon.vertices[next_index]);
        }
    }
    // 执行三角剖分
    std::vector<Triangle> triangulate()
    {
        size_t node_count = this->node_list.size();
        vector<Triangle> triangles;
        while (node_count > 3u)
        {
            for (auto &node : node_list)
            {
                if (node.is_processed) continue;
                if (node.is_ear)
                {
                    triangles.emplace_back(polygon.vertices[node.prev_index],
                        polygon.vertices[node.index],
                        polygon.vertices[node.next_index]);
                    node.is_processed = true;
                    node_count--;
                    // 更新相邻节点
                    node_list[node.prev_index].next_index = node.next_index;
                    node_list[node.next_index].prev_index = node.prev_index;
                    // 更新相邻节点的is_ear
                    node_list[node.prev_index].is_ear
                        = can_cut(polygon.vertices[node_list[node.prev_index].prev_index],
                            polygon.vertices[node_list[node.prev_index].index],
                            polygon.vertices[node_list[node.prev_index].next_index]);
                    node_list[node.next_index].is_ear
                        = can_cut(polygon.vertices[node_list[node.next_index].prev_index],
                            polygon.vertices[node_list[node.next_index].index],
                            polygon.vertices[node_list[node.next_index].next_index]);
                    if (node_count <= 3u) break;
                }
            }
        }
        return triangles;
    }
};
}

#endif