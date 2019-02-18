#ifndef KDTREE_KDTREE_H
#define KDTREE_KDTREE_H

#include <vector>
#include <limits>
#include <queue>
#include <string>
#include <cassert>
#include <algorithm>
#include <stack>
#include <unordered_map>
#include <cmath>
#include <iostream>
#include <fstream>
#include <sys/time.h>
#include <numeric>

namespace kt {

    enum SPLIT_MODE {
        DEPTH, VARIANCE
    };

    struct _node {
        int id;
        int split_dim;
        double distance;
        bool is_visit;
        _node *left, *right;

        _node()
                :
                is_visit(false),
                left(nullptr),
                right(nullptr),
                distance(-1),
                split_dim(-1),
                id(-1) {}

        bool operator<(_node &o) {
            if (this->distance == o.distance) return this->id < o.id;
            return this->distance < o.distance;
        }
    };

    struct node_ptr_cmp {
        bool operator()(const _node *n1, const _node *n2) {
            if (n1->distance == n2->distance) return n1->id < n2->id;
            return n1->distance < n2->distance;
        }
    };


    using IndiceIter = std::vector<int>::iterator;
    using NodeQueue  = std::priority_queue<_node *, std::vector<_node *>, node_ptr_cmp>;

    template<typename T>
    class kdtree {
    private:

        _node *_root_;

        unsigned long mSampleNum;

        unsigned long mFeatureNum; // assume that all samples have the same feature number.

        _node *BuildTree(IndiceIter begin, IndiceIter end, int depth);

        void DeleteTree();

        int FindTheSplitDim(IndiceIter begin, IndiceIter end, int depth);

        IndiceIter GetMidNum(IndiceIter begin, IndiceIter end, int dim);

        std::string Node2Dot(_node *node, bool erase_info);

        std::vector<std::vector<T>> *data;

        const std::vector<T> *input;

        SPLIT_MODE mSplitMode;

    public:

        kdtree() : _root_(nullptr) {}

        /**
         * create a kdtree
         * @param data which is used to create kdtree, row stands for a sample and col stands for feature.
         * @param mode method to split the hyperplane, DEPTH or VARIANCE
         */
        void SetData(std::vector<std::vector<T>> &data, SPLIT_MODE mode);

        /**
         * find all samples within certain distance from input target.
         * @param input target point.
         * @param indices index of samples within certain distance from input target, no order.
         * @param distances distance of samples within certain distance from input target, no order.
         * @param radius max distance from sample to target.
         */
        void RadiusSearch(const std::vector<T> &input, std::vector<int> &indices, std::vector<double> &distances,
                          const double &radius);

        /**
         * find top K nearest data samples from input.
         * @param input target point.
         * @param indices index of top K nearest data samples, order by distance.
         * @param distances distance of top K nearest data samples, order by distance.
         * @param K number of sample want to find.
         */
        void NearestSearch(const std::vector<T> &input, std::vector<int> &indices, std::vector<double> &distances,
                           const int &K = 1);

        /**
         * generate a bitmap of tree's structure. PNG file can be find in the workspace folder
         * Power by [Graphviz](https://www.graphviz.org/)
         * @param erase_info if ture, only ID and split dimension info will keep, otherwise lastest search info will
         *                   be wrote to image.
         * @return string content of .dot file
         */
        std::string ToDot(bool erase_info = false);


        ~kdtree();

    };


    static inline std::string getCurrentTimeStr() {
        struct timeval tv{};
        gettimeofday(&tv, nullptr);
        return std::to_string(tv.tv_sec * 1000 + tv.tv_usec / 1000);
    }

    static inline void _initialNodes(_node *root) {
        std::stack<_node *> stack;
        stack.push(root);
        while (!stack.empty()) {
            _node *_cur = stack.top();
            stack.pop();

            if (_cur->left) stack.push(_cur->left);
            if (_cur->right) stack.push(_cur->right);

            _cur->is_visit = false;
            _cur->distance = -1;
        }
    }

    template<typename T>
    void kdtree<T>::RadiusSearch(const std::vector<T> &input, std::vector<int> &indices, std::vector<double> &distances,
                                 const double &radius) {
        assert(radius >= 0);
        assert(input.size() == mFeatureNum);
        this->input = &input;
        _initialNodes(_root_);

        std::stack<_node *> path;

        indices.clear();
        distances.clear();

        auto Push = [&](_node *ptr) {

            if (ptr->is_visit) return;
            else ptr->is_visit = true;

            ptr->distance = 0;
            for (int feat_i = 0; feat_i < mFeatureNum; ++feat_i)
                ptr->distance += pow(input[feat_i] - (*data)[ptr->id][feat_i], 2.0);
            ptr->distance = sqrt(ptr->distance);

            path.push(ptr);

            if (ptr->distance <= radius) {
                indices.emplace_back(ptr->id);
                distances.emplace_back(ptr->distance);
            }
        };

        auto Search = [&](_node *node) {
            while (node) {
                Push(node);
                auto &dim = node->split_dim;
                node = input[dim] <= (*data)[node->id][dim] ? node->left : node->right;
            }
        };

        Search(_root_);

        while (!path.empty()) {
            _node *_cur = path.top();
            path.pop();

            const double &vi = input[_cur->split_dim];
            const double &vn = (*data)[_cur->id][_cur->split_dim];

            if (vi > vn) {
                if (_cur->right) Push(_cur->right);
                if (_cur->left && (vi - vn) < radius) Search(_cur->left);
            } else {
                if (_cur->left) Push(_cur->left);
                if (_cur->right && (vn - vi) < radius) Search(_cur->right);
            }
        }
    }

    template<typename T>
    void
    kdtree<T>::NearestSearch(const std::vector<T> &input, std::vector<int> &indices, std::vector<double> &distances,
                             const int &K) {

        assert(input.size() == mFeatureNum);
        this->input = &input;

        _initialNodes(_root_);
        std::stack<_node *> path;
        NodeQueue k_heap;

        indices.clear();
        distances.clear();

        auto Push = [&](_node *ptr) {
            if (ptr->is_visit) return;
            else ptr->is_visit = true;

            path.push(ptr);

            ptr->distance = 0;
            for (int feat_i = 0; feat_i < mFeatureNum; ++feat_i)
                ptr->distance += pow(input[feat_i] - (*data)[ptr->id][feat_i], 2.0);
            ptr->distance = sqrt(ptr->distance);

            if (k_heap.size() < K) {
                k_heap.push(ptr);
            } else if ((*ptr) < (*k_heap.top())) {
                k_heap.pop();
                k_heap.push(ptr);
            }
        };

        auto Search = [&](_node *node) {
            while (node) {
                Push(node);
                auto &dim = node->split_dim;
                node = input[dim] <= (*data)[node->id][dim] ? node->left : node->right;
            }
        };

        Search(_root_);
        while (!path.empty()) {
            _node *_cur = path.top();
            path.pop();

            const double &vi = input[_cur->split_dim];
            const double &vn = (*data)[_cur->id][_cur->split_dim];
            const double &vh = k_heap.top()->distance;

            // Search the other side of the splitting plane if it may contain
            // points closer than the distance.

            if (vi > vn) {
                if (_cur->right) Push(_cur->right);
                if (_cur->left && (vi - vn) < vh) {
                    Search(_cur->left);
                }
            } else {
                if (_cur->left) Push(_cur->left);
                if (_cur->right && (vn - vi) < vh) {
                    Search(_cur->right);
                }
            }

        }

        unsigned long s = k_heap.size();
        assert(s > 0);
        indices.resize(s);
        distances.resize(s);
        for (int i = 1; i <= s; ++i) {
            indices[s - i] = k_heap.top()->id;
            distances[s - i] = k_heap.top()->distance;
            k_heap.pop();
        }
    }

    template<typename T>
    kdtree<T>::~kdtree() {
        DeleteTree();
    }

    template<typename T>
    void kdtree<T>::SetData(std::vector<std::vector<T>> &data, SPLIT_MODE mode) {
        assert(!data.empty());
        DeleteTree();

        this->data = &data;
        this->mFeatureNum = data[0].size();
        this->mSampleNum = data.size();
        this->mSplitMode = mode;

        std::vector<int> _indices(mSampleNum);
        std::iota(_indices.begin(), _indices.end(), 0);

        _root_ = BuildTree(_indices.begin(), _indices.end(), 0);
    }

    template<typename T>
    _node *kdtree<T>::BuildTree(IndiceIter begin, IndiceIter end, int depth) {

        auto _root = new _node;
        _root->split_dim = FindTheSplitDim(begin, end, depth);

        auto mid_id = GetMidNum(begin, end, _root->split_dim);
        _root->id = *mid_id;

        if (std::distance(begin, mid_id) > 0)
            _root->left = BuildTree(begin, mid_id, depth + 1);

        if (std::distance(mid_id + 1, end) > 0)
            _root->right = BuildTree(mid_id + 1, end, depth + 1);

        return _root;
    }

    template<typename T>
    int kdtree<T>::FindTheSplitDim(IndiceIter begin, IndiceIter end, int depth) {

        int best_dim = depth % mFeatureNum;

        if (this->mSplitMode == DEPTH) {
            // DO NOTHING
        } else if (this->mSplitMode == VARIANCE) {
            double avg[mFeatureNum];

            std::fill(avg, avg + mFeatureNum, 0);

            for (auto it = begin; it != end; ++it) {
                for (int f = 0; f < mFeatureNum; ++f) {
                    avg[f] += (*data)[*it][f];
                }
            }


            double max_var = std::numeric_limits<double>::min();

            auto _sample_num = std::distance(begin, end);
            for (int f = 0; f < mFeatureNum; ++f) {
                avg[f] /= _sample_num;
                double _var = 0;
                for (auto it = begin; it != end; ++it)
                    _var += pow(avg[f] - (*data)[*it][f], 2);

                if (max_var < _var) {
                    best_dim = f;
                    max_var = _var;
                }
            }
        }

        return best_dim;
    }

    template<typename T>
    IndiceIter kdtree<T>::GetMidNum(IndiceIter begin, IndiceIter end, int dim) {

        auto idx = (end - begin) / 2;

        std::nth_element(begin, begin + idx, end,
                         [&](int idx1, int idx2) -> bool {
                             return (*data)[idx1][dim] < (*data)[idx2][dim];
                         });
        return begin + idx;
    }

    template<typename T>
    std::string kdtree<T>::Node2Dot(_node *node, bool erase_info) {
        if (node == nullptr)
            return "";

        char buf[256];

        if (erase_info) {
            sprintf(buf,
                    R"([shape=circle, label="ID: %d DIM: %d", fillcolor="%s", style=filled])",
                    node->id, node->split_dim, "#FFFFFF");
        } else {
            if (node->is_visit) {
                sprintf(buf,
                        R"([shape=circle, label="ID: %d DIM:%d\nDIS: %8.4lf\nVN: %8.4lf\nVI: %8.4lf", fillcolor="%s", style=filled])",
                        node->id,
                        node->split_dim,
                        node->distance,
                        double((*data)[node->id][node->split_dim]),
                        double((*input)[node->split_dim]),
                        "#FFAA22");
            } else {
                sprintf(buf,
                        R"([shape=circle, label="ID: %d DIM: %d\nVN: %8.4lf\nVI: %8.4lf", fillcolor="%s", style=filled])",
                        node->id,
                        node->split_dim,
                        double((*data)[node->id][node->split_dim]),
                        double((*input)[node->split_dim]),
                        "#FFFFFF");
            }
        }

        std::string cur_node_name = "node" + std::to_string(node->id);
        std::string cur_node_info = cur_node_name + buf + "\n";

        if (node->left == nullptr && node->right == nullptr) {
            return cur_node_info;
        }


        std::string _left, _right;
        if (node->left) {
            _left = cur_node_name + " -> node" + std::to_string(node->left->id) + "\n";
            _left += Node2Dot(node->left, erase_info);
        }
        if (node->right) {
            _right = cur_node_name + " -> node" + std::to_string(node->right->id) + "\n";
            _right += Node2Dot(node->right, erase_info);
        }

        return cur_node_info + _left + _right;
    }

    template<typename T>
    std::string kdtree<T>::ToDot(bool erase_info) {
        if (_root_ == nullptr) {
            std::cout << "No receive any data" << std::endl;
            return std::string();
        }
        std::string _tmp = "digraph kdtree {\n" + Node2Dot(_root_, erase_info) + "}\n";
        std::ofstream out(".kdtree.dot");
        out << _tmp;
        out.close();
        char command_str[64];
        sprintf(command_str, "/usr/bin/dot .kdtree.dot -T png -o %s.png", getCurrentTimeStr().c_str());
        system(command_str);
        return _tmp;
    }

    template<typename T>
    void kdtree<T>::DeleteTree() {

        if (_root_ == nullptr) return;

        std::stack<_node *> stack;
        stack.push(_root_);
        _node *_cur;
        while (!stack.empty()) {
            _cur = stack.top();
            stack.pop();

            if (_cur->left) stack.push(_cur->left);
            if (_cur->right) stack.push(_cur->right);

            free(_cur);
        }
        _root_ = nullptr;
    }
}

#endif //KDTREE_KDTREE_H
